#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using GetDirection = robot_patrol::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;

LaserScan laser_scan_data;
bool have_scan_data = false;

void laser_scan_callback(const LaserScan::SharedPtr data) {
  laser_scan_data = *data;
  have_scan_data = true;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "(laser_scan_callback) Got data.");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  laser_scan_data.angle_increment = 8.0;

  // node
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("direction_service_client");

  // direction client
  rclcpp::Client<GetDirection>::SharedPtr client =
      node->create_client<GetDirection>("direction_service");

  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub =
      node->create_subscription<LaserScan>(
          "scan", 10, laser_scan_callback
          //   [&laser_scan_data, &have_scan_data](const LaserScan::SharedPtr
          //   data) {
          //     laser_scan_data = *data;
          //     have_scan_data = true;
          //     // DEBUG
          //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got scan data from
          //     lambda callback");
          //     // end DEBUG
          //   }
      );

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

  // DEBUG
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "(test_service) angle_increment = %f",
              laser_scan_data.angle_increment);
  // end DEBUG

  if (have_scan_data) {

    request->laser_data = laser_scan_data;
    auto result_future = client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Direction: %s",
                  result->direction.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call direction service");
    }
  }

  rclcpp::shutdown();
  return 0;
}