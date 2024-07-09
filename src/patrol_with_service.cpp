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

  // laser scanner parametrization
  bool laser_scanner_parametrized_ = false;
  double angle_increment;

  // arc indices
  int right_start, right_end;
  int front_start, front_end;
  int left_start, left_end;
  int front;

  // Motion parameters
  const double VELOCITY_INCREMENT = 0.1;
  const double ANGULAR_BASE = 0.5;
  const double LINEAR_BASE = 0.1;
  const double LINEAR_TURN = LINEAR_BASE * 0.5;
  const double OBSTACLE_FWD_PROXIMITY = 0.35;
  const double ANGULAR_TOLERANCE_DEG = 1.5;
  const double ANGULAR_TOLERANCE = ANGULAR_TOLERANCE_DEG * DEG2RAD;
  const double DIRECTION_TURN_DIVISOR = 1.5;
  const int FRONT_FANOUT = 4;

  // Velocity control
  // Tuples of:
  //   times * range_min
  //   linear velocity
  //   times * angular velocity
  // Example:
  // At 6.0 times range_min from an obstacle, set
  // linear.x = 0.08 and angular.z = 1.0 * ang_vel,
  // where ang_vel is the direction angle of the
  // ray with the farthest range in the sector
  // returned from the direction service {"left",
  // "front", "right"}
  double range_min;
  const std::vector<std::tuple<double, double, double>> vel_ctrl{
      {6.0, 0.08, 1.0},
      {3.0, 0.04, 1.5},
      {1.5, 0.02, 2.0},
  };

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

  // nav
  std::tuple<double, bool> obstacle_in_range_linear(int from, int to,
                                                    double dist);
  void adjust_velocities(double linear_base, double angular_base);
  double best_direction(std::string dir);
  // end nav

  // utility functions
  void wait_for_laser_scan_publisher();
  void wait_for_direction_server();

  double yaw_from_quaternion(double x, double y, double z, double w);
  void parametrize_laser_scanner(LaserScan &scan_data);
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

  // TODO
  /***********************************
  1. Reintroduce obstacle_in_range.
  2. Define stages of approach, say {0.6, 0.3, 0.1}.
  3. Use the stages of approach to modify linear and angular.
  4. Reintroduce find_direction and use the direction service
     to either:
     1. Look for direction in the arc/sector returned, or
     2. Filter found directions by the arc/sector returned.
  5. Use the direction found to define angular.
  ***********************************/

  if (!laser_scanner_parametrized_)
    parametrize_laser_scanner(last_laser_);

  std::string direction;

  auto status = future.wait_for(1s);
  if (status != std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Service '%s' in progress...",
                direction_service_name_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Service '%s' response",
                direction_service_name_.c_str());
    auto result = future.get();
    direction = result->direction;
    RCLCPP_INFO(this->get_logger(), "Direction '%s'", direction.c_str());
  }

//   if (direction == "right") {
//     cmd_vel_msg_.linear.x = 0.1;
//     cmd_vel_msg_.angular.z = -0.5;
//   } else if (direction == "left") {
//     cmd_vel_msg_.linear.x = 0.1;
//     cmd_vel_msg_.angular.z = 0.5;
//   } else if (direction == "forward") {
//     cmd_vel_msg_.linear.x = 0.1;
//     cmd_vel_msg_.angular.z = 0.0;
//   } else {
//     cmd_vel_msg_.linear.x = 0.0;
//     cmd_vel_msg_.angular.z = 0.0;
//   }

  // cmd_vel_msg_ is set 
  adjust_velocities(0.1, best_direction(direction));

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

void Patrol::parametrize_laser_scanner(LaserScan &scan_data) {
  angle_increment = scan_data.angle_increment;
  range_min = scan_data.range_min;

  // DEBUG
  RCLCPP_DEBUG(this->get_logger(), "angle_increment = %f", angle_increment);
  // end DEBUG

  int thirty_deg_indices = static_cast<int>((PI_ / 6.0) / angle_increment);

  // DEBUG
  RCLCPP_DEBUG(this->get_logger(), "thirty_deg_indices = %d",
               thirty_deg_indices);
  // end DEBUG

  double front_center_double = PI_ / angle_increment;
  int front_center_ix = static_cast<int>(lround(front_center_double));
  front = front_center_ix;

  // DEBUG
  RCLCPP_DEBUG(this->get_logger(), "front_center_double = %f",
               front_center_double);
  RCLCPP_DEBUG(this->get_logger(), "front_center_ix = %d", front_center_ix);
  // end DEBUG

  // to ensure equal arc lengths (in indices)
  // start with front, then extend to right and left
  // the length of front
  front_start = front_center_ix - thirty_deg_indices - 1;
  front_end = front_center_ix + thirty_deg_indices;

  int sixty_deg_indices = front_end - front_start - 1;

  right_end = front_start - 1;
  right_start = right_end - sixty_deg_indices - 1;

  left_start = front_end + 1;
  left_end = left_start + sixty_deg_indices + 1;

  // // right:
  // // Relative to front_start [-pi/2.0, -pi/6.0]
  // // Relative to ranges array index 0 angle [pi/2.0, 5.0 * pi/6.0]
  // right_start = static_cast<int>(lround((PI_ / 2.0) / angle_increment));
  // right_end = static_cast<int>(lround((5.0 * PI_ / 6.0) / angle_increment));

  // DEBUG
  RCLCPP_DEBUG(this->get_logger(), "right_start = %d", right_start);
  RCLCPP_DEBUG(this->get_logger(), "right_end = %d", right_end);
  // end DEBUG

  // // forward:
  // // Relative to front_start [0.0, pi/3.0]
  // // Relative to ranges array index 0 angle [5.0 * pi/6.0, 7.0 * pi/6.0]
  // front_start = right_end + 1;
  // front_end = static_cast<int>(lround((7.0 * PI_ / 6.0) / angle_increment));

  // DEBUG
  RCLCPP_DEBUG(this->get_logger(), "front_start = %d", front_start);
  RCLCPP_DEBUG(this->get_logger(), "front_end = %d", front_end);
  // end DEBUG

  // // left:
  // // Relative to front_start [pi/3.0, 2.0 *pi/3.0]
  // // Relative to ranges array index 0 angle [7.0 * pi/6.0, 3.0 * pi/2.0]
  // left_start = front_end + 1;
  // left_end = static_cast<int>(lround((3.0 * PI_ / 2.0) / angle_increment));

  // DEBUG
  RCLCPP_DEBUG(this->get_logger(), "left_start = %d", left_start);
  RCLCPP_DEBUG(this->get_logger(), "left_end = %d", left_end);
  // end DEBUG

  // DEBUG
  // inclusive counting, hence the -1
  RCLCPP_DEBUG(this->get_logger(), "right_end - right_start - 1 = %d",
               right_end - right_start - 1);
  RCLCPP_DEBUG(this->get_logger(), "front_end - front_start - 1 = %d",
               front_end - front_start - 1);
  RCLCPP_DEBUG(this->get_logger(), "left_end - left_start - 1 = %d",
               left_end - left_start - 1);
  // end DEBUG

  laser_scanner_parametrized_ = true;
}

// navigation

// linear, not circular as we only look 180 deg forward
std::tuple<double, bool> Patrol::obstacle_in_range_linear(int from, int to,
                                                          double dist) {
  // is there an obstacle between from and to
  bool is_obstacle = false;
  for (int i = from; i <= to; ++i) { // "linear" span/arc/sector
    if (!std::isinf(last_laser_.ranges[i]) && last_laser_.ranges[i] <= dist)
      is_obstacle = true;
  }

  // how close are obstacles in front
  // fanaout to smooth out inf values
  double front_dist = 0.0;
  for (int i = front - FRONT_FANOUT; i <= front + FRONT_FANOUT; ++i) {
    if (!std::isinf(last_laser_.ranges[i]) &&
        last_laser_.ranges[i] > front_dist)
      front_dist = last_laser_.ranges[i];
  }

  return std::make_tuple(front_dist, is_obstacle);
}

// Velocity control
// Tuples of:
//   times * range_min
//   linear velocity
//   times * angular velocity
// Example:
// At 6.0 times range_min from an obstacle, set
// linear.x = 0.08 and angular.z = 1.0 * ang_vel,
// where ang_vel is the direction angle of the
// ray with the farthest range in the sector
// returned from the direction service {"left",
// "front", "right"}
void Patrol::adjust_velocities(double linear_base, double angular_base) {
  /*
    const std::vector<std::tuple<double, double, double>> vel_ctrl{
        {6.0, 0.08, 1.0},
        {3.0, 0.04, 1.5},
        {1.5, 0.02, 2.0},
    };
  */

  // how close are obstacles in front
  // fanaout to smooth out inf values
  double front_dist = 0.0;
  for (int i = front - FRONT_FANOUT; i <= front + FRONT_FANOUT; ++i) {
    if (!std::isinf(last_laser_.ranges[i]) &&
        last_laser_.ranges[i] > front_dist)
      front_dist = last_laser_.ranges[i];
  }

  // apply vel control vector
  if (front_dist <= std::get<0>(vel_ctrl[2]) * range_min) {
    cmd_vel_msg_.linear.x = std::get<1>(vel_ctrl[2]);
    cmd_vel_msg_.angular.z = std::get<2>(vel_ctrl[2]) * angular_base;
  } else if (front_dist <= std::get<0>(vel_ctrl[1]) * range_min) {
    cmd_vel_msg_.linear.x = std::get<1>(vel_ctrl[1]);
    cmd_vel_msg_.angular.z = std::get<2>(vel_ctrl[1]) * angular_base;
  } else if (front_dist <= std::get<0>(vel_ctrl[0]) * range_min) {
    cmd_vel_msg_.linear.x = std::get<1>(vel_ctrl[0]);
    cmd_vel_msg_.angular.z = std::get<2>(vel_ctrl[0]) * angular_base;
  } else {
    cmd_vel_msg_.linear.x = linear_base;
    cmd_vel_msg_.angular.z = angular_base;
  }
}

double Patrol::best_direction(std::string dir) {
  int from, to;

  if (dir == "left") {
    from = left_start;
    to = left_end;
  } else if (dir == "forward") {
    from = front_start;
    to = front_end;
  } else if (dir == "right") {
    from = right_start;
    to = right_end;
  } else { // for sanity
    from = front;
    to = front;
  }

  double largest_range = 0.0;
  int largest_range_ix = -1;
  for (int i = from; i <= to; ++i)
    if (!std::isinf(last_laser_.ranges[i]) &&
        last_laser_.ranges[i] > largest_range) {
      largest_range = last_laser_.ranges[i];
      largest_range_ix = i;
    }

  return (largest_range_ix - front) * angle_increment;
}

// end navigation

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto patroller_with_service = std::make_shared<Patrol>();

  //   auto logger = rclcpp::get_logger("robot_patrol_node");

  //   // Set the log level to DEBUG
  //   if (rcutils_logging_set_logger_level(
  //           logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG) !=
  //           RCUTILS_RET_OK)
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