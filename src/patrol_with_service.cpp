#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rcutils/logging.h"
#include "robot_patrol/srv/get_direction.hpp"
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

using GetDirection = robot_patrol::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;
using Odometry = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;

class Patrol : public rclcpp::Node {
public:
  Patrol();
  ~Patrol() = default;

  void init();

private:
  LaserScan last_laser_;
  Odometry odom_data_;
  Twist cmd_vel_msg_;

  bool have_laser_ = false;
  bool have_odom_ = false;
  bool got_data = false;

  std::string direction_service_name_;
  rclcpp::Client<GetDirection>::SharedPtr client_;

  std::string laser_scan_topic_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr service_call_timer_;

  //   double direction_; // angle in radians
  double yaw_;   // current orientation
  bool turning_; // supports pass-through code for turning

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
  // service client

  // service response callback
  void velocity_callback(rclcpp::Client<GetDirection>::SharedFuture future);

  // subscribers
  void laser_scan_callback(const LaserScan::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // end callbacks

  // patrolling with service
  void start();        // start timed service call
  void service_call(); // send request to direction service

  // utility functions
  void wait_for_laser_scan_publisher();
  void wait_for_direction_server();

  double yaw_from_quaternion(double x, double y, double z, double w);
  // end utility functions
};

// constructors

Patrol::Patrol() : Node("robot_patrol_node") {
  direction_service_name_ = "direction_service";
  client_ = this->create_client<GetDirection>(direction_service_name_);

  laser_scan_topic_ = "scan";
  scan_sub_ = this->create_subscription<LaserScan>(
      laser_scan_topic_, 10, std::bind(&Patrol::laser_scan_callback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Patrol::odometry_callback, this, _1));
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  have_laser_ = false;
  have_odom_ = false;
  turning_ = false;
}

void Patrol::init() {
  wait_for_laser_scan_publisher();
  wait_for_direction_server();
  start(); // timer for service call
}

// callbacks

// publisher
void Patrol::velocity_callback(
    rclcpp::Client<GetDirection>::SharedFuture future) {

  auto status = future.wait_for(1s);
  if (status != std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Service '%s' in progress...",
                direction_service_name_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Service '%s' response",
                direction_service_name_.c_str());
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "Direction '%s'",
                result->direction.c_str());
  }

  // TODO: linear.x and angular.z depending on the direction result

  // DEBUG
  // just move in circles
  cmd_vel_msg_.linear.x = 0.05;
  cmd_vel_msg_.angular.z = -0.3;

  // single point of publishing
  publisher_->publish(cmd_vel_msg_);
}

// subscriber
void Patrol::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  //   RCLCPP_DEBUG(this->get_logger(), "Laser scan callback");
  last_laser_ = *msg;

  have_laser_ = true;
  got_data = true;
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

// utilities
void Patrol::wait_for_laser_scan_publisher() {
  // ROS 2 does't have an equivalent to wait_for_publisher
  // this is one way to solve the problem
  while (this->count_publishers("scan") == 0) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for '%s' topic publisher. Exiting.",
          laser_scan_topic_.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "'%s' topic publisher not available, waiting...",
                laser_scan_topic_.c_str());
  }
}

void Patrol::wait_for_direction_server() {
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for '%s' server. Terminating...",
          direction_service_name_.c_str());
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Waiting for '%s' server...",
                  direction_service_name_.c_str());
    }
  }
}

void Patrol::service_call() {
  if (!got_data) {
    RCLCPP_INFO(this->get_logger(), "No laser data yet...");
  } else {
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = last_laser_;

    auto result_future = client_->async_send_request(
        request,
        std::bind(&Patrol::velocity_callback, this, std::placeholders::_1));
  }
}

// call the service at a frequency of 10 Hz
void Patrol::start() {
  service_call_timer_ =
      this->create_wall_timer(100ms, std::bind(&Patrol::service_call, this));
}

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
  //           logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK)
  //           {
  //     // Handle the error (e.g., print an error message or throw an
  //     exception) RCLCPP_ERROR(logger, "Failed to set logger level for
  //     robot_patrol_node.");
  //   } else {
  //     RCLCPP_INFO(logger, "Successfully set logger level for
  //     robot_patrol_node.");
  //   }

  patroller_with_service->init();

  rclcpp::spin(patroller_with_service);
  rclcpp::shutdown();
  return 0;
}