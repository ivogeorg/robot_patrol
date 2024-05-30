#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath> // for std::isinf(), for atan2(y, x)

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
  float direction_; // angle in radians
  float yaw_;       // current orientation

  // Laser scanner parametrization

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

  // Range of the center of next direction
  const int DIR_FROM = LEFT;
  const int DIR_TO = RIGHT;

  const double VELOCITY_INCREMENT = 0.1;
  const double ANGULAR_BASE = 0.5;
  const double LINEAR_BASE = 0.1; // REQUIREMENT
  const double OBSTACLE_PROXIMITY = 0.35;

  // publisher
  void velocity_callback();

  // subscriber
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // utility functions
  bool obstacle_in_range(int from, int to, double dist);
  float find_safest_direction();
  double yaw_from_quaternion(double x, double y, double z, double w);
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

    // this one is robus but for a corner case of going straight very near a
    // wall can correct with some logic to anticipate obstacles in front
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

// TODO: this should also be a range
//       can we fit a gaussian?
float Patrol::find_safest_direction() {
  float dir = 0; // angle in rads

  // TODO
  // 1. Use laser_scan_data_ to find the longest range +/- pi/2 of fwd.
  // 2.

  return dir;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}