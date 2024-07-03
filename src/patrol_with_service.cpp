#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm> // for std::sort
#include <cassert>
#include <chrono>
#include <cmath> // for std::isinf(), atan2(y, x)
#include <math.h>
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
  sensor_msgs::msg::LaserScan last_laser_;
  nav_msgs::msg::Odometry odom_data_;
  geometry_msgs::msg::Twist cmd_vel_msg_;

  bool have_laser_;
  bool have_odom_;
  bool laser_scanner_parametrized_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

//   double direction_; // angle in radians
  double yaw_;       // current orientation
  bool turning_;     // supports pass-through code for turning

  // Motion parameters
  const double VELOCITY_INCREMENT = 0.1;
  const double ANGULAR_BASE = 0.5;
  const double LINEAR_BASE = 0.1; // REQUIREMENT
  const double LINEAR_TURN = LINEAR_BASE * 0.5;
  const double OBSTACLE_FWD_PROXIMITY = 0.35;
  const double ANGULAR_TOLERANCE_DEG = 1.5;
  const double ANGULAR_TOLERANCE = ANGULAR_TOLERANCE_DEG * DEG2RAD;
  const double DIRECTION_TURN_DIVISOR = 1.5; // REQUIREMENT 2.0

  // Misc. parameters
  const double FLOAT_COMPARISON_TOLERANCE = 1e-9;

  // callbacks
  // publishers
  void velocity_callback(); // TODO: add future parameter

  // subscribers
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // end callbacks

  // utility functions
  double yaw_from_quaternion(double x, double y, double z, double w);
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
}

// callbacks

// publisher
void Patrol::velocity_callback() { // TODO: future param

  // TODO: get the direction

  // single point of publishing
  publisher_->publish(cmd_vel_msg_);
}

// subscriber
void Patrol::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  //   RCLCPP_DEBUG(this->get_logger(), "Laser scan callback");
  last_laser_ = *msg;

  have_laser_ = true;
}

// subscriber
void Patrol::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  //   RCLCPP_DEBUG(this->get_logger(), "Odometry callback");
  odom_data_ = *msg;
  have_odom_ = true;
  yaw_ = yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //   RCLCPP_DEBUG(this->get_logger(), "Current orientation is %f", yaw_);
}

// end callbacks

// utility functions

// yaw in radians
double Patrol::yaw_from_quaternion(double x, double y, double z, double w) {
  return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto patroller_with_service = std::make_shared<Patrol>();

//   auto logger = rclcpp::get_logger("robot_patrol_node");

//   // Set the log level to DEBUG
//   if (rcutils_logging_set_logger_level(
//           logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
//     // Handle the error (e.g., print an error message or throw an exception)
//     RCLCPP_ERROR(logger, "Failed to set logger level for robot_patrol_node.");
//   } else {
//     RCLCPP_INFO(logger, "Successfully set logger level for robot_patrol_node.");
//   }
  
  // TODO: take out node, init, then spin

  rclcpp::spin(patroller_with_service);
  rclcpp::shutdown();
  return 0;
}