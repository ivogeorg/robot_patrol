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
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  sensor_msgs::msg::LaserScan laser_scan_data_;
  geometry_msgs::msg::Twist vel_cmd_msg_;
  double direction_; // angle in radians
  double yaw_;       // current orientation

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
  const int FRONT_FROM = FRONT - 46, FRONT_TO = FRONT + 46; // ~50 deg angle

  // The following is done to avoid "narrow" width affordances for the robot
  // A NUM_PEAKS longest ranges will be compared by the SUM or NUM_NEIGHBORS
  // neighboring ranges on both sides. This will approximate fitting a normal
  // distribution and allow picking a direction that is most likely to be
  // "wide" enough for the robot to move in.
  const int NUM_PEAKS = 5;
  const int NUM_NEIGHBORS = 5;

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

  // publisher
  void velocity_callback();

  // subscriber
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // utility functions
  bool obstacle_in_range(int from, int to, double dist);
  double yaw_from_quaternion(double x, double y, double z, double w);
  void find_safest_direction();
  void turn_safest_direction();
  double normalize_angle(double angle);
};

// constructors

Patrol::Patrol() : Node("robot_patrol_node") {
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_ = this->create_wall_timer(100ms,
                                   std::bind(&Patrol::velocity_callback, this));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Patrol::laser_scan_callback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Patrol::odometry_callback, this, _1));
}

// callbacks

// publisher
void Patrol::velocity_callback() {
  // TODO:
  // Forward until obstacle.
  // Stop when obstacle.
  // Find safest direction.
  // Rotate. Wait until done!!! (if turning, don't change the Twist msg)
  // repeat...

  if (!obstacle_in_range(FRONT_FROM, FRONT_TO, OBSTACLE_PROXIMITY)) {
    vel_cmd_msg_.linear.x = LINEAR_BASE;
    vel_cmd_msg_.angular.z = 0.0;
  } else {
    // stop forward motion
    vel_cmd_msg_.linear.x = 0.0;

    bool obstacle_left =
        obstacle_in_range(LEFT_FROM, LEFT_TO, OBSTACLE_PROXIMITY);
    bool obstacle_right =
        obstacle_in_range(RIGHT_FROM, RIGHT_TO, OBSTACLE_PROXIMITY);
    bool turning_left = vel_cmd_msg_.angular.z > 0.0;
    bool turning_right = vel_cmd_msg_.angular.z < 0.0;

    // This algorithm is robust but for a corner case of going straight very
    // near a wall. Can correct with some logic to anticipate obstacles in
    // front.
    if (obstacle_left && turning_right) {
      vel_cmd_msg_.angular.z += -VELOCITY_INCREMENT;
    } else if (obstacle_right && turning_left) {
      vel_cmd_msg_.angular.z += VELOCITY_INCREMENT;
    } else {
      vel_cmd_msg_.angular.z = 0.5;
    }
  }

  publisher_->publish(vel_cmd_msg_);
}

// subscriber
void Patrol::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  laser_scan_data_ = *msg;
  RCLCPP_DEBUG(this->get_logger(), "Distance to the left is %f",
               laser_scan_data_.ranges[LEFT]);
}

// subscriber
void Patrol::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
      if (laser_scan_data_.ranges[i] <= dist)
        is_obstacle = true;
  return is_obstacle;
}

void Patrol::find_safest_direction() {
  // put ranges and indices into a vector for sorting
  std::vector<std::pair<int, float>> v_indexed_ranges;
  for (int i = 0; i < static_cast<int>(laser_scan_data_.ranges.size()); ++i)
    // exclude all lower than RIGHT and higher than LEFT (REQUIREMENT)
    if (i >= RIGHT && i <= LEFT)
      v_indexed_ranges.push_back(std::make_pair(i, laser_scan_data_.ranges[i]));

  // sort by ranges in descending order
  std::sort(v_indexed_ranges.begin(), v_indexed_ranges.end(),
            [](const std::pair<int, float> &a, const std::pair<int, float> &b) {
              return a.second > b.second;
            });

  // for the top NUM_PEAKS, sum up their neighboring ranges
  // (by NUM_NEIGHBORS) and keep track of the index with the
  // largest sum
  float highest_sum = 0, sum;
  int highest_sum_index = -1, peak_index;
  for (int i = 0; i < NUM_PEAKS; ++i) {
    peak_index = v_indexed_ranges[i].first;
    for (int j = peak_index - NUM_NEIGHBORS; j < peak_index; ++j)
      sum += laser_scan_data_.ranges[j];
    for (int j = peak_index + 1; j <= peak_index + NUM_NEIGHBORS; ++j)
      sum += laser_scan_data_.ranges[j];
    if (sum > highest_sum) {
      highest_sum = sum;
      highest_sum_index = peak_index;
    }
    sum = 0;
  }

  // for the highest_sum_index, calculate the angle
  // negative CW, positive CCW
  // angle_increment is in radians
  direction_ = (highest_sum_index - FRONT) * laser_scan_data_.angle_increment;
}

// TODO: Can this not block but pass through, only starting the rotation
//       and stopping it when it completes?
//       Slightly different logic...
void Patrol::turn_safest_direction() {
  // TODO: port _rotate from checkpoint2 branch of my_rb1_robot
  // NOTE: use direction_ as goal yaw and to set the
  //       angular velocity
  // NOTE: stop the robot before exiting, the caller will set the
  //       forward velocity

  // TODO: how does this loop interact with the 10 Hz callback
  //       which is the caller?

  double last_angle double turn_angle = 0.0;
  double goal_angle = direction_;

  vel_cmd_msg_.linear.x = 0.0;
  if (goal_angle > 0)
    vel_cmd_msg_.angular.z = goal_angle / 2.0;
  else
    vel_cmd_msg_.angular.z = -goal_angle / 2.0;

  if (goal_angle > 0) {
    while (rclcpp::ok() &&
           (abs(turn_angle + ANGULAR_TOLERANCE) < abs(goal_angle))) {
      publisher_.pub(vel_cmd_msg_);
      // what about sleep()?

      double temp_yaw = yaw_; // yaw_ is changing
      double delta_angle = normalize_angle(temp_yaw - last_angle);

      turn_angle += delta_angle;
      last_angle = temp_yaw;
    }
  } else {
    while (rclcpp::ok() &&
           (abs(turn_angle - ANGULAR_TOLERANCE) < abs(goal_angle))) {
      publisher_.pub(vel_cmd_msg_);
      // what about sleep()?

      double temp_yaw = yaw_; // yaw_ is changing
      double delta_angle = normalize_angle(temp_yaw - last_angle);

      turn_angle += delta_angle;
      last_angle = temp_yaw;
  }

  vel_cmd_msg_.angular.z = 0.0;
  publisher_.publish(vel_cmd_msg_);
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