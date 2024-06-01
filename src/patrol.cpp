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

  bool have_laser;
  bool have_odom;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double direction_;  // angle in radians
  double yaw_;        // current orientation
  bool turning_;      // if turning_ don't publish cmd_vel
  bool done_turning_; // if has turned in new direction

  // Laser scanner parametrization and smoothing

  // The following is done so as to work directly with ranges[] indices
  // Angle ranges contain 660 rays covering 360 degrees or 2*pi radians
  // 1 deg is about 1.83 angle increments
  // 15 deg is 27.5 ~ 28 angle increments
  // 25 deg is 45.8 ~ 46 angle increments

  // Ranges index for "right" is 164
  const int RIGHT = 164;
  const int RIGHT_FROM = RIGHT - 28, RIGHT_TO = RIGHT + 28; // ~30 deg angle
  // Ranges index for "left" is 493
  const int LEFT = 493;
  const int LEFT_FROM = LEFT - 28, LEFT_TO = LEFT + 28; // ~30 deg angle
  // Ranges index for "forward" is 329
  const int FRONT = 329;
  //   const int FRONT_FROM = FRONT - 46, FRONT_TO = FRONT + 46; // ~50 deg
  //   angle
  const int FRONT_FROM = FRONT - 28, FRONT_TO = FRONT + 28; // ~30 deg angle

  // The following is done to avoid "narrow" width affordances for the robot
  // A NUM_PEAKS longest ranges will be compared by the SUM or NUM_NEIGHBORS
  // neighboring ranges on both sides. This will approximate fitting a normal
  // distribution and allow picking a direction that is most likely to be
  // "wide" enough for the robot to move in.
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
  const double PI = 3.14159265359;
  const double ANGULAR_TOLERANCE_DEG = 1.5;
  const double ANGULAR_TOLERANCE = ANGULAR_TOLERANCE_DEG * PI / 180.0;
  const double FLOAT_COMPARISON_TOLERANCE = 1e-9;

  enum class State { STOPPED, FORWARD, FIND_NEW_DIR, TURNING };
  State state;

  // publisher
  void velocity_callback();

  // subscriber
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // utility functions
  bool obstacle_in_range(int from, int to, double dist);
  double yaw_from_quaternion(double x, double y, double z, double w);
  void find_safest_direction();
  bool turn_safest_direction();
  double normalize_angle(double angle);
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
  have_laser = false;
  have_odom = false;
  turning_ = false;
  done_turning_ = false;
  state = State::STOPPED;
}

// callbacks

// publisher
void Patrol::velocity_callback() {
  //   RCLCPP_INFO(this->get_logger(), "Velocity callback");
  if (!have_laser || !have_odom) {
    RCLCPP_INFO(this->get_logger(), "No nav data. Velocity callback no-op.");
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
    RCLCPP_INFO(this->get_logger(), "Found new direction %f", direction_);
    RCLCPP_INFO(this->get_logger(), "Starting yaw %f", yaw_);
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
      RCLCPP_INFO(this->get_logger(), "Resulting yaw %f", yaw_);
      RCLCPP_INFO(this->get_logger(), "Within tolerance of new direction");
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
  //   RCLCPP_INFO(this->get_logger(), "Laser scan callback");
  laser_scan_data_ = *msg;
  have_laser = true;
  RCLCPP_DEBUG(this->get_logger(), "Distance to the left is %f",
               laser_scan_data_.ranges[LEFT]);
}

// subscriber
void Patrol::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  //   RCLCPP_INFO(this->get_logger(), "Odometry callback");
  have_odom = true;
  yaw_ = yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  RCLCPP_DEBUG(this->get_logger(), "Current orientation is %f", yaw_);
}

// end callbacks

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

// A more sophisticated algorithm to try later

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
      RCLCPP_INFO(this->get_logger(), "Peak range = %f", laser_scan_data_.ranges[peak_index]);      
      RCLCPP_INFO(this->get_logger(), "Neighbor sum of ranges = %f\n", highest_sum);      
    }
    sum = 0;
  }

  // for the highest_sum_index, calculate the angle
  // negative - CW, positive - CCW
  // angle_increment is in radians
  direction_ = (highest_sum_index - FRONT) * laser_scan_data_.angle_increment;

  RCLCPP_INFO(this->get_logger(), "Found new direction %f", direction_);
}

// this is a pass-through version of the function:
// no while loop, just starting, monitoring, and
// stopping the turning
bool Patrol::turn_safest_direction() {
  // this is done to avoid error depending on direction of turning
  if ((direction_ > 0 && (abs(yaw_ + ANGULAR_TOLERANCE) < abs(direction_))) ||
      (direction_ < 0 && (abs(yaw_ - ANGULAR_TOLERANCE) <
                          abs(direction_)))) { // need to turn (more)
    // if not turning already, start, otherwise continue
    if (!turning_) {
      turning_ = true;
      cmd_vel_msg_.angular.z = direction_ / 2.0;
      RCLCPP_INFO(this->get_logger(), "Starting to turn");
    } else {
      RCLCPP_INFO(this->get_logger(), "Still turning");
      RCLCPP_INFO(this->get_logger(), "Linear %f", cmd_vel_msg_.linear.x);
      RCLCPP_INFO(this->get_logger(), "Angular %f", cmd_vel_msg_.angular.z);
      RCLCPP_INFO(this->get_logger(), "Direction %f", direction_);
      RCLCPP_INFO(this->get_logger(), "Yaw %f\n", yaw_);
    }
  } else {
    // reached goal angle within tolerance, stop turning
    turning_ = false;
    cmd_vel_msg_.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Completed turning");
  }

  return turning_;
}

double Patrol::normalize_angle(double angle) {
  double res = angle;
  while (res > PI)
    res -= 2.0 * PI;
  while (res < -PI)
    res += 2.0 * PI;
  return res;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}