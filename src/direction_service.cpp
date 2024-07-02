#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <string>

// #include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionServerNode : public rclcpp::Node {
public:
  DirectionServerNode() : Node("direction_server_node") {

    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionServerNode::direction_callback, this, _1, _2));
  }

private:
  // direction service
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  // laser scanner parametrization
  bool laser_scanner_parametrized_ = false;
  const double PI_ = 3.14159265359;
  // arc indices
  int right_start, right_end;
  int forward_start, forward_end;
  int left_start, left_end;

  // callbacks
  void
  direction_callback(const std::shared_ptr<GetDirection::Request> request,
                     const std::shared_ptr<GetDirection::Response> response) {
    LaserScan scan_data = request->laser_data;

    // DEBUG
    // RCLCPP_INFO(this->get_logger(), "(direction_callback) angle_increment =
    // %f",
    //             scan_data.angle_increment);
    RCLCPP_INFO(this->get_logger(), "(direction_callback) range(330) = %f",
                scan_data.ranges[330]);
    // end DEBUG

    if (!laser_scanner_parametrized_)
      parametrize_laser_scanner(scan_data);

    double total_dist_sec_right = 0.0;
    double total_dist_sec_front = 0.0;
    double total_dist_sec_left = 0.0;

    for (int i = right_start; i <= right_end; ++i)
      if (!std::isinf(scan_data.ranges[i]))
        total_dist_sec_right += scan_data.ranges[i];

    for (int i = forward_start; i <= forward_end; ++i)
      if (!std::isinf(scan_data.ranges[i]))
        total_dist_sec_front += scan_data.ranges[i];

    for (int i = left_start; i <= left_end; ++i)
      if (!std::isinf(scan_data.ranges[i]))
        total_dist_sec_left += scan_data.ranges[i];

    // DEBUG
    RCLCPP_INFO(this->get_logger(), "right: %f, forward: %f, left: %f",
                total_dist_sec_right, total_dist_sec_front,
                total_dist_sec_left);
    // end DEBUG

    std::vector<std::pair<std::string, double>> v;
    v.push_back(std::make_pair("right", total_dist_sec_right));
    v.push_back(std::make_pair("forward", total_dist_sec_front));
    v.push_back(std::make_pair("left", total_dist_sec_left));

    std::sort(v.begin(), v.end(),
              [](const std::pair<std::string, double> &a,
                 const std::pair<std::string, double> &b) {
                return a.second > b.second;
              });

    response->direction = v[0].first;

    // DEBUG
    RCLCPP_INFO(this->get_logger(), "direction: %s",
                response->direction.c_str());
    // end DEBUG
  }

  // utilities
  void parametrize_laser_scanner(LaserScan &scan_data) {
    double angle_increment = scan_data.angle_increment;

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(), "angle_increment = %f", angle_increment);
    // end DEBUG

    // right:
    // Relative to forward_start [-pi/2.0, -pi/6.0]
    // Relative to ranges array index 0 angle [pi/2.0, 5.0 * pi/6.0]
    right_start = static_cast<int>(lround((PI_ / 2.0) / angle_increment));
    right_end = static_cast<int>(lround((5.0 * PI_ / 6.0) / angle_increment));

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(), "right_start = %d", right_start);
    RCLCPP_DEBUG(this->get_logger(), "right_end = %d", right_end);
    // end DEBUG

    // forward:
    // Relative to forward_start [0.0, pi/3.0]
    // Relative to ranges array index 0 angle [5.0 * pi/6.0, 7.0 * pi/6.0]
    forward_start = right_end + 1;
    forward_end = static_cast<int>(lround((7.0 * PI_ / 6.0) / angle_increment));

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(), "forward_start = %d", forward_start);
    RCLCPP_DEBUG(this->get_logger(), "forward_end = %d", forward_end);
    // end DEBUG

    // left:
    // Relative to forward_start [pi/3.0, 2.0 *pi/3.0]
    // Relative to ranges array index 0 angle [7.0 * pi/6.0, 3.0 * pi/2.0]
    left_start = forward_end + 1;
    left_end = static_cast<int>(lround((3.0 * PI_ / 2.0) / angle_increment));

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(), "left_start = %d", left_start);
    RCLCPP_DEBUG(this->get_logger(), "left_end = %d", left_end);
    // end DEBUG

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(), "right_end - right_start = %d",
                 right_end - right_start);
    RCLCPP_DEBUG(this->get_logger(), "forward_end - forward_start = %d",
                 forward_end - forward_start);
    RCLCPP_DEBUG(this->get_logger(), "left_end - left_start = %d",
                 left_end - left_start);
    // end DEBUG

    laser_scanner_parametrized_ = true;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("direction_server_node");

  // Set the log level to DEBUG
  if (rcutils_logging_set_logger_level(
          logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
    // Handle the error (e.g., print an error message or throw an exception)
    RCLCPP_ERROR(logger, "Failed to set logger level for robot_patrol_node.");
  } else {
    RCLCPP_INFO(logger, "Successfully set logger level for robot_patrol_node.");
  }

  rclcpp::spin(std::make_shared<DirectionServerNode>());
  rclcpp::shutdown();
  return 0;
}