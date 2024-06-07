#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm> // for std::sort
#include <chrono>
#include <cmath>     // for std::isinf(), atan2(y, x)
#include <stdexcept> // for InvalidStateError
#include <tuple>
#include <utility> // for std::pair, std::tuple
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
  // Extending the range to avoid oscillation
  double RIGHT_EXTEND, LEFT_EXTEND;

  // The following help avoid "narrow" width affordances for the robot
  // A NUM_PEAKS longest ranges will be compared by the SUM of NUM_NEIGHBORS
  // neighboring ranges on both sides. This will allow picking a direction
  // that is most likely to be "wide" enough for the robot to move in.
  const int NUM_PEAKS = 100;
  const int NUM_NEIGHBORS = 30;

  //   // Range of the center of next direction (+/- pi radians) REQUIREMENT
  //   const int DIR_FROM = LEFT;
  //   const int DIR_TO = RIGHT;

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
  enum class State { STOPPED, FORWARD, FIND_NEW_DIR, TURNING, BACK_UP };
  State state_, last_state_;
  class InvalidStateError : public std::runtime_error {
  public:
    InvalidStateError(const std::string &what_arg)
        : std::runtime_error(what_arg) {}
    ~InvalidStateError() = default;
  };

  // Finding safest direction to turn to at an obstacle
  enum class DirSafetyBias { ANGLE, RANGE };
  DirSafetyBias dir_safety_bias_; // to set in STOPPED for FIND_NEW_DIR
  enum class DirPref { RIGHT, LEFT, RIGHT_LEFT, NONE };
  DirPref direction_preference_; // to set in STOPPED for FIND_NEW_DIR

  // setup, tracking, reporting, and handling anomalous situations
  // oscillation, stuck, too_close_to_obstacle
  bool is_oscillating_, is_too_close_to_obstacle_;
  int turns_in_a_row_;
  // num consecutive turns for identifying oscillation
  const int OSCILLATION_THRESHOLD = 6;
  // give robot a wider choice of angles
  bool extended_angle_range_; // should go with DirSafetyBias::ANGLE

  // callbacks
  // publishers
  void velocity_callback();

  // subscribers
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // end callbacks

  // utility functions
  void parametrize_laser_scanner();
  std::tuple<bool, float> obstacle_in_range(int from, int to, double dist);
  double yaw_from_quaternion(double x, double y, double z, double w);
  void find_safest_direction(bool extended = false,
                             DirSafetyBias dir_bias = DirSafetyBias::ANGLE,
                             DirPref dir_pref = DirPref::NONE);
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
  state_ = State::STOPPED;
  laser_scanner_parametrized_ = false;
  dir_safety_bias_ =
      DirSafetyBias::RANGE; // RANGE works better with safety criterion
  direction_preference_ = DirPref::RIGHT_LEFT;
  turns_in_a_row_ = 0;
  is_oscillating_ = false;
  is_too_close_to_obstacle_ = false;
  extended_angle_range_ = false;
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
    RCLCPP_INFO(this->get_logger(),
                "Parametrizing laser scanner. Velocity callback no-op.");
    parametrize_laser_scanner();
    return;
  }

  // for obstacle_in_range
  bool is_obstacle;
  float inf_ratio;

  switch (state_) {
  case State::STOPPED:
    // State::STOPPED is the pivotal state of the state machine

    std::tie(is_obstacle, inf_ratio) =
        obstacle_in_range(FRONT_FROM, FRONT_TO, OBSTACLE_PROXIMITY);

    // The only state that has a switch (last_state_) {}
    switch (last_state_) {
    case State::FORWARD:
    case State::FIND_NEW_DIR:
      // Should not come here from these states
      throw InvalidStateError(
          "ERROR: (pkg: robot_patrol, src: patrol.cpp) came to "
          "State::STOPPED from invalid last state");
      break;
    case State::TURNING:
      if (!is_obstacle) {
        cmd_vel_msg_.linear.x = LINEAR_BASE;
        cmd_vel_msg_.angular.z = 0.0;
        state_ = State::FORWARD;
        RCLCPP_INFO(this->get_logger(), "Going forward...\n");
      } else {
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.angular.z = 0.0;
        if (turns_in_a_row_ >= OSCILLATION_THRESHOLD) {
          is_oscillating_ = true;
          state_ = State::BACK_UP;
        } else {
          cmd_vel_msg_.linear.x = 0.0;
          cmd_vel_msg_.angular.z = 0.0;
          state_ = State::FIND_NEW_DIR;
        }
        RCLCPP_INFO(this->get_logger(), "Stopped. Obstacle in front");
      }
      break;
    case State::BACK_UP:
      // After backing up, find new direction and favor larger angles
      cmd_vel_msg_.linear.x = 0.0;
      cmd_vel_msg_.angular.z = 0.0;

      // zero out the current count
      turns_in_a_row_ = 0;

      // find a larger angle
      extended_angle_range_ = true;
      dir_safety_bias_ = DirSafetyBias::ANGLE;

      state_ = State::FIND_NEW_DIR;
      break;
    case State::STOPPED:
    default:
      if (!is_obstacle) {
        // first call with data after constructor
        RCLCPP_WARN(this->get_logger(),
                    "Warning: (pkg: robot_patrol, src: patrol.cpp) "
                    "defaulted in State::STOPPED or came from State::STOPPED");
        cmd_vel_msg_.linear.x = LINEAR_BASE;
        cmd_vel_msg_.angular.z = 0.0;
        state_ = State::FORWARD;
        RCLCPP_INFO(this->get_logger(), "Going forward...\n");
      } else {
        // sanity check
        throw InvalidStateError(
            "ERROR: (pkg: robot_patrol, src: patrol.cpp) defaulted "
            " in State::STOPPED or came from State::STOPPED with obstacle");
      }
    }

    last_state_ = State::STOPPED;
    break;
  case State::FORWARD:
    // if no obstacle in front, stay in FORWARD
    // if obstacle in front, go to TURNING
    // set cmd_vel_msg_.linear.x = 0.0
    // set cmd_vel_msg_.angular.z = 0.0
    std::tie(is_obstacle, inf_ratio) =
        obstacle_in_range(FRONT_FROM, FRONT_TO, OBSTACLE_PROXIMITY);

    if (is_obstacle) {
      cmd_vel_msg_.linear.x = 0.0;
      cmd_vel_msg_.angular.z = 0.0;
      state_ = State::FIND_NEW_DIR;
      RCLCPP_INFO(this->get_logger(), "Obstacle in front, stopping");
    } else {
      cmd_vel_msg_.linear.x = LINEAR_BASE;
      cmd_vel_msg_.angular.z = 0.0;

      // clear all anomaly tracking and reporting variables
      turns_in_a_row_ = 0;
      is_oscillating_ = false;
      is_too_close_to_obstacle_ = false;
    }

    last_state_ = State::FORWARD;
    break;
  case State::FIND_NEW_DIR:

    // find new direction
    // do not change cmd_vel_msg_
    // go to TURNING

    // DEBUG: try extended ranges to avoid getting stuck and/or oscillating
    // the following combination of arguments was pretty robust

    find_safest_direction(false, DirSafetyBias::RANGE,
                          DirPref::NONE); // true = extend side ranges
    // end DEBUG

    state_ = State::TURNING;
    RCLCPP_INFO(this->get_logger(), "Found new direction (robot frame) %f",
                direction_);
    RCLCPP_DEBUG(this->get_logger(), "Starting yaw %f", yaw_);

    last_state_ = State::FIND_NEW_DIR;
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
      ++turns_in_a_row_; // track for oscillation
      state_ = State::STOPPED;
    }

    last_state_ = State::TURNING;
    break;
  case State::BACK_UP:

    // TODO

    last_state_ = State::BACK_UP;
    break;
  default:
    // Should not come here!
    InvalidStateError err("ERROR: (pkg: robot_patrol, src: patrol.cpp) default "
                          "statement executed in state machine");
    throw err;
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
  RCLCPP_INFO(this->get_logger(), "RANGES_SIZE = %d", RANGES_SIZE);
  RCLCPP_INFO(this->get_logger(), "RANGE_MIN = %f", RANGE_MIN);
  RCLCPP_INFO(this->get_logger(), "RANGE_MAX = %f", RANGE_MAX);
  RCLCPP_INFO(this->get_logger(), "ANGLE_INCREMENT = %f\n", ANGLE_INCREMENT);

  // Amount of angle added on both sides of a direction
  // to give the robot some "peripheral" vision.
  double HALF_SPREAD = DIRECTION_SPREAD_DEG * DEG2RAD / 2.0; // in rad
  RCLCPP_INFO(this->get_logger(), "DIRECTION_SPREAD_DEGREES = %f",
              DIRECTION_SPREAD_DEG);
  RCLCPP_INFO(this->get_logger(), "HALF_SPREAD = %f", HALF_SPREAD);

  // Indices from HALF_SPREAD and angle_increment
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
  RCLCPP_INFO(this->get_logger(), "BACK_TO = %d", BACK_TO);

  // extend to the maximum that will not overrun or underrun
  RIGHT_EXTEND = NUM_NEIGHBORS + 1;
  LEFT_EXTEND = RANGES_SIZE - 1 - NUM_NEIGHBORS;

  laser_scanner_parametrized_ = true;
  RCLCPP_INFO(this->get_logger(), "Laser scan parametrized\n");
}

// yaw in radians
double Patrol::yaw_from_quaternion(double x, double y, double z, double w) {
  return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}

std::tuple<bool, float> Patrol::obstacle_in_range(int from, int to,
                                                  double dist) {
  bool is_obstacle = false;
  // get a stable local version
  // while it may still have values from several callbacks,
  // it is likely less inconsistent than the variable which
  // each callback assigns
  std::vector<double> ranges(laser_scan_data_.ranges.begin(),
                             laser_scan_data_.ranges.end());
  int num_inf = 0;
  for (int i = from; i <= to; ++i) {
    if (std::isinf(ranges[i]))
      ++num_inf; // count `inf` values in the range
    if (!std::isinf(ranges[i]))
      if (ranges[i] <= dist)
        is_obstacle = true;
  }
  RCLCPP_DEBUG(this->get_logger(), "Inf: %d/%d", num_inf,
               to - from - 1); // DEBUG line
  return std::make_tuple(is_obstacle, num_inf);
}

// A rather overengineered function implementing
// a complicated safety criterion and sorting of
// directions
void Patrol::find_safest_direction(bool extended, DirSafetyBias dir_bias,
                                   DirPref dir_pref) {
  RCLCPP_INFO(this->get_logger(), "Looking for safest direction");
  std::vector<double> ranges(laser_scan_data_.ranges.begin(),
                             laser_scan_data_.ranges.end());

  // 1. (optional) Extended range
  double right = (extended) ? RIGHT_EXTEND : RIGHT;
  double left = (extended) ? LEFT_EXTEND : LEFT;

  // 2. (fixed) Filter by angle and sort by range
  // put ranges and indices into a vector for sorting
  std::vector<std::pair<int, float>> v_indexed_ranges;
  for (int i = 0; i < static_cast<int>(ranges.size()); ++i)
    // include only ray indices between RIGHT and LEFT (REQUIREMENT)
    // and not those in the FRONT spread (which are checked for obstacles)
    if ((i >= right && i < FRONT_FROM) || (i >= FRONT_TO && i <= left))
      if (!std::isinf(ranges[i]))
        v_indexed_ranges.push_back(std::make_pair(i, ranges[i]));

  // sort by ranges in descending order to get the peak ranges first
  std::sort(v_indexed_ranges.begin(), v_indexed_ranges.end(),
            [](const std::pair<int, float> &a, const std::pair<int, float> &b) {
              return a.second > b.second;
            });

  // 3. (choice of DirSafetyBias) Setup safety criterion
  // for the top NUM_PEAKS, get the average of their neighboring ranges
  // at three levels, each level on both sides:
  // NUM_NEIGHBORS, NUM_NEIGHBORS / 2.0, NUM_NEIGHBORS / 4.0
  // form tuples (int, double, double, double) for the index and avgs
  // sort by avg_qtr > avg_half && avg_half > avg_full
  // this is likely to pick the safest, and not the longest range, as
  // it will be smoothed out on both sides, away from obstacles
  std::vector<std::tuple<int, double, double, double, double>>
      v_indexed_averages;

  double sum = 0.0;
  double divisor = 0.00001; // guard against accidental divide-by-zero
  double avg_qtr, avg_half, avg_full;
  int num_neighbors = NUM_NEIGHBORS;
  int peak_index;
  double peak_range;
  //   float highest_sum = 0, sum;
  //   int highest_sum_index = -1, peak_index;
  for (int i = 0; i < NUM_PEAKS; ++i) {
    peak_index = v_indexed_ranges[i].first;
    peak_range = v_indexed_ranges[i].second;

    // avg_full
    for (int j = peak_index - num_neighbors; j <= peak_index + num_neighbors;
         ++j)
      if (!std::isinf(ranges[j])) {
        sum += ranges[j];
        divisor += 1.0;
      }
    // // we don't want the highest peak but the widest direction
    // // so remove the peak value from the sum
    sum -= peak_range;
    divisor -= 1.0;
    avg_full = sum / divisor;

    // avg_half
    sum = 0.0;
    divisor = 0.00001;
    num_neighbors = static_cast<int>(floor(num_neighbors / 2.0));
    for (int j = peak_index - num_neighbors; j <= peak_index + num_neighbors;
         ++j)
      if (!std::isinf(ranges[j])) {
        sum += ranges[j];
        divisor += 1.0;
      }
    // // we don't want the highest peak but the widest direction
    // // so remove the peak value from the sum
    sum -= peak_range;
    divisor -= 1.0;
    avg_half = sum / divisor;

    // avg_qtr
    sum = 0.0;
    divisor = 0.00001;
    num_neighbors = static_cast<int>(floor(num_neighbors / 2.0));
    for (int j = peak_index - num_neighbors; j <= peak_index + num_neighbors;
         ++j)
      if (!std::isinf(ranges[j])) {
        sum += ranges[j];
        divisor += 1.0;
      }
    // // we don't want the highest peak but the widest direction
    // // so remove the peak value from the sum
    sum -= peak_range;
    divisor -= 1.0;
    avg_qtr = sum / divisor;

    // 3.1 Set up the DirSafetyBias

    double bias_variable;
    switch (dir_bias) {
    case DirSafetyBias::ANGLE:
      // favor larger angles
      bias_variable = abs((peak_index - FRONT) * ANGLE_INCREMENT);
      break;
    case DirSafetyBias::RANGE:
      bias_variable = peak_range;
      break;
    default:
      // default on DirSafetyBias::RANGE
      bias_variable = peak_range;
    }

    // insert in vector for sorting
    // v_indexed_averages.push_back(
    //     std::make_tuple(peak_index, peak_range, avg_qtr, avg_half,
    //     avg_full));

    // favor larger angles
    // (peak_index - FRONT) * laser_scan_data_.angle_increment
    // v_indexed_averages.push_back(
    //     std::make_tuple(peak_index, abs((peak_index - FRONT) *
    //     ANGLE_INCREMENT),
    //                     avg_qtr, avg_half, avg_full));

    v_indexed_averages.push_back(std::make_tuple(peak_index, bias_variable,
                                                 avg_qtr, avg_half, avg_full));

    // restore loop vars
    sum = 0.0;
    divisor = 0.00001;
    num_neighbors = NUM_NEIGHBORS;
  }

  // 4. (fixed) Sort by safety (applying DirSafetyBias)
  std::sort(v_indexed_averages.begin(), v_indexed_averages.end(),
            [](const std::tuple<int, double, double, double, double> &a,
               const std::tuple<int, double, double, double, double> &b) {
              /**
                How to define a total-order sorting criterion for
                "safer"?
                ---
                The idea is that the safest is the one the distribution of
                the neighboring ranges of which is closest to an
                upside-down quadratic or a normal. The three levels of
                averages are defined to approximate this idea. For example,
                if a.avg_qtr > a.avg_half and a.avg_half > a.avg_full and
                !(b.avg_qtr > b.avg_half and b.avg_half > b.avg_full), a is
                safer than b.
              */
              if ((std::get<2>(a) > std::get<3>(a) &&
                   std::get<3>(a) > std::get<4>(a)) &&
                  !(std::get<2>(b) > std::get<3>(b) &&
                    std::get<3>(b) > std::get<4>(b)))
                return true;
              else if (!(std::get<2>(a) > std::get<3>(a) &&
                         std::get<3>(a) > std::get<4>(a)) &&
                       (std::get<2>(b) > std::get<3>(b) &&
                        std::get<3>(b) > std::get<4>(b)))
                return false;
              else
                return std::get<1>(a) > std::get<1>(b);
            });

  // 5. (optional, choice of DirPref) Pick direction from the sorting
  // any but DirPref::NONE potentially defeats the safety criterion
  int ix; // ray index to pick
  int num_right = 0, num_left = 0;
  int top_right_ix = -1, top_left_ix = -1;
  // count right dir and left dir
  // keep track of the top ("safest") of each
  // pick the top of the right or left, whichever is more numerous
  // technically, both cannot be zero and of whichever there
  // are more, their top index cannot be invalid (-1)
  // slight bias to the right due to >=
  for (auto &d : v_indexed_averages) {
    ix = std::get<0>(d);
    if ((ix - FRONT) * ANGLE_INCREMENT < 0.0) { // to the right
      ++num_right;
      if (top_right_ix < 0)
        top_right_ix = ix;
    } else {
      ++num_left;
      if (top_left_ix < 0)
        top_left_ix = ix;
    }
  }
  switch (dir_pref) {
  case DirPref::LEFT:
    ix = top_left_ix;
    break;
  case DirPref::RIGHT:
    ix = top_right_ix;
    break;
  case DirPref::RIGHT_LEFT:
    ix = (top_right_ix >= top_left_ix) ? top_right_ix : top_left_ix;
    break;
  case DirPref::NONE:
  default:
    auto top_sorted = v_indexed_averages[0];
    ix = std::get<0>(top_sorted);
  }

  // 6. (fixed) Set the new direction
  // for the selected ray ix, calculate the angle relative to angle zero (FRONT)
  // negative - CW, positive - CCW, angle_increment is in radians
  direction_ = (ix - FRONT) * ANGLE_INCREMENT;
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