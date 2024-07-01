#include "rclcpp/create_subscription.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <string>

using std::placeholders::_1;

using namespace std::chrono_literals;
using GetDirection = robot_patrol::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;

class LaserScanSub : public rclcpp::Node {
public:
  LaserScanSub() : Node("laser_scan_sub");
  {
    sub_ = this->rclcpp::create_subscription<LaserScan>(
        "scan", 10, std::bind(&LaserScanSub::laser_scan_cb, this, _1));

    while (this->count_publishers("scan") == 0) {
    //   if (!rclcpp::ok()) {
    //     RCLCPP_ERROR(this->get_logger(),
    //                  "Interrupted while waiting for topic publisher. Exiting.");
    //     return 0;
    //   }
      RCLCPP_INFO(this->get_logger(),
                  "Topic publisher not available, waiting...");
    }
  }

  LaserScan get_data() { return data_; }

private:
  rclcpp::Subscription<LaserScan>::SharedPtr sub_;
  LaserScan data_;
  bool got_data = false;

  void laser_scan_cb(const LaserScan::SharedPtr data) {
    data_ = *data;
    got_data = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "(LaserScanSub::laser_scan_cb) Got data");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  laser_scan_data.angle_increment = 8.0;

  // nodes
  std::shared_ptr<rclcpp::Node> scan_node =
      rclcpp::Node::make_shared("scan_topic_subscriber");

  std::shared_ptr<rclcpp::Node> dir_node =
      rclcpp::Node::make_shared("direction_service_client");

  // executor
  rclcpp::executors::MultiThreadedExecutor exec;

  exec.add_node(scan_node);
  exec.add_node(dir_node);

  // direction client
  rclcpp::Client<GetDirection>::SharedPtr client =
      dir_node->create_client<GetDirection>("direction_service");

  //   rclcpp::Subscription<LaserScan>::SharedPtr scan_sub =
  //       dir_node->create_subscription<LaserScan>(
  //           "scan", 10, laser_scan_callback
  //           //   [&laser_scan_data, &have_scan_data](const
  //           LaserScan::SharedPtr
  //           //   data) {
  //           //     laser_scan_data = *data;
  //           //     have_scan_data = true;
  //           //     // DEBUG
  //           //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got scan data
  //           from
  //           //     lambda callback");
  //           //     // end DEBUG
  //           //   }
  //       );

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  // send request with response cb
  auto request = std::make_shared<GetDirection::Request>();
  request->laser_data = scan_node->get_data();
  auto result_future = client->async_send_request(
      request, [&result_future](rclcpp::Client<LaserScan>::SharedFuture fut) {
        auto status = fut.wait_for(1s);
        if (status == std::future_status::ready) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                      "Got direction service response");
          auto result = result_future.get();
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Direction: %s",
                      result->direction.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Direction service in progress...");
        }
      });

  //   // wait for result
  //   if (rclcpp::spin_until_future_complete(dir_node, result_future) ==
  //       rclcpp::FutureReturnCode::SUCCESS) {
  //     auto result = result_future.get();
  //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Direction: %s",
  //                 result->direction.c_str());
  //   } else {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
  //                  "Failed to call direction service");
  //   }

  rclcpp::shutdown();
  return 0;
}