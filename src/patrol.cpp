#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm> // for std::sort
#include <chrono>
#include <cmath>   // for std::isinf(), for atan2(y, x)
#include <utility> // for std::pair
#include <vector>

#define PI_ 3.14159265359
#define DEG2RAD (PI_ / 180.0)
#define RAD2DEG (180.0 / PI_)

using namespace std::chrono_literals; // for s, ms, ns, etc.
using std::placeholders::_1;          // for callback parameter placeholder in
                                      // std::bind()

class Patrol : public rclcpp::Node {
public:
  Patrol();
  ~Patrol() = default;

private:
  sensor_msgs::msg::LaserScan laser_scan_data_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

  bool have_laser_;
  bool have_odom_;
  bool laser_scanner_parametrized_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double direction_; // angle in radians
  double yaw_;       // current orientation
  bool turning_;     // if turning_ don't publish cmd_vel

  // Laser scanner parametrization and smoothing

  // The laser scanner is parametrized to a number of rays.
  // The range readings from these rays are recieved in the
  // `ranges` vector. This is the size of the vector.
  int RANGES_SIZE;

  // For ranges outside the following distances the scanner
  // returns `inf`!
  double RANGE_MIN, RANGE_MAX;

  // Angle in radians between two rays
  double ANGLE_INCREMENT;

  // Work directly with ranges[] indices

  // Using just a single ray in front or to the sides to detect
  // obstacles is brittle and unreliable, so pick a spread.
  // Split in two and center in the direction.
  // The _FROM and _TO ranges below are to be used with this.
  const double DIRECTION_SPREAD_DEG = 30;

  // Ranges index for "right"
  int RIGHT, RIGHT_FROM, RIGHT_TO;
  // Ranges index for "left"
  int LEFT, LEFT_FROM, LEFT_TO;
  // Ranges index for "forward"
  // This roughly protects the robot from catching a wheel on an
  // obstacle it can't sense (e.g. a traffic sign base)
  int FRONT, FRONT_FROM, FRONT_TO;
  // Ranges index for "back(ward)"
  // This is more difficult because of the discontinuity
  // Use i = (i + 1) % RANGES_SIZE
  int BACK, BACK_FROM, BACK_TO;

  // The following help avoid "narrow" width affordances for the robot
  // A NUM_PEAKS longest ranges will be compared by the SUM of NUM_NEIGHBORS
  // neighboring ranges on both sides. This will allow picking a direction
  // that is most likely to be "wide" enough for the robot to move in.
  const int NUM_PEAKS = 100;
  const int NUM_NEIGHBORS = 30;

  // Range of the center of next direction (+/- pi radians) REQUIREMENT
  const int DIR_FROM = LEFT;
  const int DIR_TO = RIGHT;

  // Motion parameters
  const double VELOCITY_INCREMENT = 0.1;
  const double ANGULAR_BASE = 0.5;
  const double LINEAR_BASE = 0.1; // REQUIREMENT
  const double OBSTACLE_PROXIMITY = 0.35;
  const double ANGULAR_TOLERANCE_DEG = 1.5;
  const double ANGULAR_TOLERANCE = ANGULAR_TOLERANCE_DEG * DEG2RAD;

  // Misc. parameters
  const double FLOAT_COMPARISON_TOLERANCE = 1e-9;

  // Robot state machine
  enum class State { STOPPED, FORWARD, FIND_NEW_DIR, TURNING };
  State state;

  // callbacks
  // publishers
  void velocity_callback();

  // subscribers
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // end callbacks

  // utility functions
  void parametrize_laser_scanner();
  bool obstacle_in_range(int from, int to, double dist);
  double yaw_from_quaternion(double x, double y, double z, double w);
  void find_safest_direction();
  double normalize_angle(double angle);
  // end utility functions
};

// constructors

Patrol::Patrol() : Node("robot_patrol_node") {
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Patrol::laser_scan_callback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Patrol::odometry_callback, this, _1));
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_ = this->create_wall_timer(100ms,
                                   std::bind(&Patrol::velocity_callback, this));
  have_laser_ = false;
  have_odom_ = false;
  turning_ = false;
  state = State::STOPPED;
  laser_scanner_parametrized_ = false;
}

// callbacks

// publisher
void Patrol::velocity_callback() {
  RCLCPP_DEBUG(this->get_logger(), "Velocity callback");

  // Avoid accessing uninitialized data structures
  // Both laser scan and odometry data is required
  // in velocity_callback
  if (!have_laser_ || !have_odom_) {
    RCLCPP_INFO(this->get_logger(), "No nav data. Velocity callback no-op.");
    return;
  } else if (!laser_scanner_parametrized_) {
    // data is available but scanner has not parametrized yet
    parametrize_laser_scanner();
    RCLCPP_INFO(this->get_logger(),
                "Parametrizing laser scanner. Velocity callback no-op.");
    return;
  }

  switch (state) {
  case State::STOPPED:
    // if no obstacle in front, go to FORWARD
    // set cmd_vel_msg_.linear.x = LINEAR_BASE
    // set cmd_vel_msg_.angular.z = 0.0
    if (!obstacle_in_range(FRONT_FROM, FRONT_TO, OBSTACLE_PROXIMITY)) {
      cmd_vel_msg_.linear.x = LINEAR_BASE;
      cmd_vel_msg_.angular.z = 0.0;
      state = State::FORWARD;
      RCLCPP_INFO(this->get_logger(), "Going forward");
    } else {
      cmd_vel_msg_.linear.x = 0.0;
      cmd_vel_msg_.angular.z = 0.0;
      state = State::FIND_NEW_DIR;
    }
    break;
  case State::FORWARD:
    // if no obstacle in front, stay in FORWARD
    // if obstacle in front, go to TURNING
    // set cmd_vel_msg_.linear.x = 0.0
    // set cmd_vel_msg_.angular.z = 0.0
    if (obstacle_in_range(FRONT_FROM, FRONT_TO, OBSTACLE_PROXIMITY)) {
      cmd_vel_msg_.linear.x = 0.0;
      cmd_vel_msg_.angular.z = 0.0;
      state = State::FIND_NEW_DIR;
      RCLCPP_INFO(this->get_logger(), "Obstacle in front, stopping");
    }
    break;
  case State::FIND_NEW_DIR:
    // find new direction
    // do not change cmd_vel_msg_
    // go to TURNING
    find_safest_direction();
    state = State::TURNING;
    RCLCPP_INFO(this->get_logger(), "Found new direction (robot frame) %f",
                direction_);
    RCLCPP_DEBUG(this->get_logger(), "Starting yaw %f", yaw_);
    break;
  case State::TURNING:
    // if not turned in new direction, stay at TURNING
    // issue cmd_vel_msg_.linear.x = 0.0
    // issue cmd_vel_msg_.angular.z = direction_ / 2.0
    // if new direction achieved, go to STOPPED
    // issue cmd_vel_msg_.linear.x = 0.0
    // issue cmd_vel_msg_.angular.z = 0.0
    // this is done to avoid error depending on direction of turning

    static double last_angle;
    static double turn_angle;
    static double goal_angle;

    // if not turning, initialize to start
    if (!turning_) {
      last_angle = yaw_;
      turn_angle = 0;
      goal_angle = direction_;
    }

    if ((goal_angle > 0 &&
         (abs(turn_angle + ANGULAR_TOLERANCE) < abs(goal_angle))) ||
        (goal_angle < 0 && (abs(turn_angle - ANGULAR_TOLERANCE) <
                            abs(direction_)))) { // need to turn (more)
      cmd_vel_msg_.linear.x = 0.0;
      cmd_vel_msg_.angular.z = direction_ / 2.0;

      double temp_yaw = yaw_;
      double delta_angle = normalize_angle(temp_yaw - last_angle);

      turn_angle += delta_angle;
      last_angle = temp_yaw;

      turning_ = true;
    } else {
      // reached goal angle within tolerance, stop turning
      RCLCPP_DEBUG(this->get_logger(), "Resulting yaw %f", yaw_);
      RCLCPP_INFO(this->get_logger(),
                  "Turned within tolerance of new direction");
      cmd_vel_msg_.linear.x = 0.0;
      cmd_vel_msg_.angular.z = 0.0;

      turning_ = false;
      state = State::STOPPED;
    }
    break;
  }

  // single point of publishing
  publisher_->publish(cmd_vel_msg_);
}

// subscriber
void Patrol::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Laser scan callback");
  laser_scan_data_ = *msg;
  have_laser_ = true;
  RCLCPP_DEBUG(this->get_logger(), "Distance to the left is %f",
               laser_scan_data_.ranges[LEFT]);
}

// subscriber
void Patrol::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Odometry callback");
  have_odom_ = true;
  yaw_ = yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  RCLCPP_DEBUG(this->get_logger(), "Current orientation is %f", yaw_);
}

// end callbacks

// utility functions
void Patrol::parametrize_laser_scanner() {
  // TODO:
  // migrate private const to private variables
  // derive variables from laser_scan_data_ message parameters
  // perform assert checks (e.g. size of ranges * angle_increment)
  // check for inf
  // initialize inf substitute data

  // NOTE: Assuming this function won't be called before laser scan
  //       data is recieved, so `laser_scan_data_` is valid.
  // NOTE: Assuming a 2 * pi radian (360 degree) laser scanner.
  //            __front__
  //          |||   ^   |||
  //          |||       |||
  //      left  |       |  right
  //            |       |
  //            |_back__|--> Discontinuity due to wrap-around
  //       Zero angle is at front.
  //       Angles to the right are negative, to the left - positive.
  //       Zero ranges index is at back.
  //       Indices grow CCW from 0 to ranges.size() - 1.
  //       For an even ranges.size(), this likely means that the
  //       above directions are only approximated by the indices.

  // Initialize from the local geometry_msgs/msg/LaserScan
  RANGES_SIZE = laser_scan_data_.ranges.size();
  RANGE_MIN = laser_scan_data_.range_min;
  RANGE_MAX = laser_scan_data_.range_max;
  ANGLE_INCREMENT = laser_scan_data_.angle_increment;

  // Amount of angle added on both sides of a direction
  // to give the robot some "peripheral" vision.
  double HALF_SPREAD = DIRECTION_SPREAD_DEG * DEG2RAD / 2.0; // in rad
  RCLCPP_INFO(this->get_logger(), "DIRECTION_SPREAD_DEGREES = %f", DIRECTION_SPREAD_DEG);
  RCLCPP_INFO(this->get_logger(), "HALF_SPREAD = %f", HALF_SPREAD);

  // TODO: indices from HALF_SPREAD and angle_increment!
  int SPREAD_INDICES = static_cast<int>(HALF_SPREAD / ANGLE_INCREMENT);
  RCLCPP_INFO(this->get_logger(), "SPREAD_INDICES = %d\n", SPREAD_INDICES);

  // Ranges index for "right".
  RIGHT = static_cast<int>(floor(0.25 * RANGES_SIZE)) - 1;
  RIGHT_FROM = RIGHT - SPREAD_INDICES;
  RIGHT_TO = RIGHT + SPREAD_INDICES;
  RCLCPP_INFO(this->get_logger(), "RIGHT = %d", RIGHT);
  RCLCPP_INFO(this->get_logger(), "RIGHT_FROM = %d", RIGHT_FROM);
  RCLCPP_INFO(this->get_logger(), "RIGHT_TO = %d", RIGHT_TO);

  // Ranges index for "left"
  LEFT = static_cast<int>(floor(0.75 * RANGES_SIZE)) - 1;
  LEFT_FROM = LEFT - SPREAD_INDICES;
  LEFT_TO = LEFT + SPREAD_INDICES;
  RCLCPP_INFO(this->get_logger(), "LEFT = %d", LEFT);
  RCLCPP_INFO(this->get_logger(), "LEFT_FROM = %d", LEFT_FROM);
  RCLCPP_INFO(this->get_logger(), "LEFT_TO = %d", LEFT_TO);

  // Ranges index for "forward"
  FRONT = static_cast<int>(floor(0.5 * RANGES_SIZE)) - 1;
  FRONT_FROM = FRONT - SPREAD_INDICES;
  FRONT_TO = FRONT + SPREAD_INDICES;
  RCLCPP_INFO(this->get_logger(), "FRONT = %d", FRONT);
  RCLCPP_INFO(this->get_logger(), "FRONT_FROM = %d", FRONT_FROM);
  RCLCPP_INFO(this->get_logger(), "FRONT_TO = %d", FRONT_TO);

  // Ranges index for "back(ward)"
  // This is a bit tricky because of the discontinuity
  // Use i = (i + 1) % RANGES_SIZE
  BACK = 0;
  BACK_FROM = RANGES_SIZE - 1 - SPREAD_INDICES;
  BACK_TO = BACK + SPREAD_INDICES;
  RCLCPP_INFO(this->get_logger(), "BACK = %d", BACK);
  RCLCPP_INFO(this->get_logger(), "BACK_FROM = %d", BACK_FROM);
  RCLCPP_INFO(this->get_logger(), "BACK_TO = %d\n", BACK_TO);

  laser_scanner_parametrized_ = true;
}

// yaw in radians
double Patrol::yaw_from_quaternion(double x, double y, double z, double w) {
  return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}

bool Patrol::obstacle_in_range(int from, int to, double dist) {
  bool is_obstacle = false;
  for (int i = from; i <= to; ++i)
    // inf >> dist
    if (!std::isinf(laser_scan_data_.ranges[i]))
      if (laser_scan_data_.ranges[i] <= dist) {
        is_obstacle = true;
        break;
      }
  return is_obstacle;
}
// The simplest algorithm. Not very robust.
// void Patrol::find_safest_direction() {
//   // safest direction
//   double largest_range = 0.0;
//   int largest_range_index = -1;
//   for (int i = 0; i < static_cast<int>(laser_scan_data_.ranges.size()); ++i)
//     // include only between RIGHT and LEFT (REQUIREMENT)
//     if (i >= RIGHT && i <= LEFT) {
//       if (laser_scan_data_.ranges[i] > largest_range) {
//         largest_range = laser_scan_data_.ranges[i];
//         largest_range_index = i;
//       }
//     }
//   RCLCPP_INFO(this->get_logger(), "largest_range_index %d",
//               largest_range_index);
//   RCLCPP_INFO(this->get_logger(), "largest_range %f", largest_range);
//   RCLCPP_INFO(this->get_logger(), "range in front %f",
//               laser_scan_data_.ranges[FRONT]);
//   direction_ = (largest_range_index - FRONT) *
//   laser_scan_data_.angle_increment;
//   // this direction is relative to the robot
//   // the rotation algorithm should take care of that
// }

// A more sophisticated algorithm. More robust.
void Patrol::find_safest_direction() {
  RCLCPP_INFO(this->get_logger(), "Looking for safest direction");

  // put ranges and indices into a vector for sorting
  std::vector<std::pair<int, float>> v_indexed_ranges;
  for (int i = 0; i < static_cast<int>(laser_scan_data_.ranges.size()); ++i)
    // include only ray indices between RIGHT and LEFT (REQUIREMENT)
    if (i >= RIGHT && i <= LEFT)
      v_indexed_ranges.push_back(std::make_pair(i, laser_scan_data_.ranges[i]));

  // sort by ranges in descending order to get the peak ranges first
  std::sort(v_indexed_ranges.begin(), v_indexed_ranges.end(),
            [](const std::pair<int, float> &a, const std::pair<int, float> &b) {
              return a.second > b.second;
            });

  // for the top NUM_PEAKS, sum up their neighboring ranges
  // (by NUM_NEIGHBORS) on both sides and keep track of the
  // index with the largest sum
  float highest_sum = 0, sum;
  int highest_sum_index = -1, peak_index;
  for (int i = 0; i < NUM_PEAKS; ++i) {
    peak_index = v_indexed_ranges[i].first;
    for (int j = peak_index - NUM_NEIGHBORS; j <= peak_index + NUM_NEIGHBORS;
         ++j)
      sum += laser_scan_data_.ranges[j];
    // // we don't want the highest peak but the widest direction
    // // so remove the peak value from the sum
    sum -= laser_scan_data_.ranges[peak_index];
    if (sum > highest_sum) {
      highest_sum = sum;
      highest_sum_index = peak_index;

      RCLCPP_INFO(this->get_logger(), "Peak index sort pos = %d", i);
      RCLCPP_INFO(this->get_logger(), "Peak ranges index = %d", peak_index);
      RCLCPP_INFO(this->get_logger(), "Peak range = %f",
                  laser_scan_data_.ranges[peak_index]);
      RCLCPP_INFO(this->get_logger(), "Neighbor sum of ranges = %f\n",
                  highest_sum);
    }
    sum = 0;
  }

  // for the highest_sum_index, calculate the angle
  // negative - CW, positive - CCW
  // angle_increment is in radians
  direction_ = (highest_sum_index - FRONT) * laser_scan_data_.angle_increment;
}

double Patrol::normalize_angle(double angle) {
  double res = angle;
  while (res > PI_)
    res -= 2.0 * PI_;
  while (res < -PI_)
    res += 2.0 * PI_;
  return res;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}