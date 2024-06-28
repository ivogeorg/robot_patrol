#include "rclcpp/rclcpp.hpp"
// #include "std_srvs/srv/empty.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using GetDirection = robot_patrol::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // node
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("service_client_direction");

  // direction client
  rclcpp::Client<GetDirection>::SharedPtr client =
      node->create_client<GetDirection>("direction_service");

  LaserScan laser_scan_data;

  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub =
      node->create_subscription<LaserScan>(
          "scan", 10, [&laser_scan_data](const LaserScan::SharedPtr data) {
            laser_scan_data = *data;
          });

  while (node->count_publishers("scan") == 0) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for topic publisher. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Topic publisher not available, waiting again...");
  }

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto request = std::make_shared<GetDirection::Request>();
  request->laser_data = laser_scan_data;
  auto result_future = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Direction: %s", result->direction.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call direction service");
  }

  rclcpp::shutdown();
  return 0;
}