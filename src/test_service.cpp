#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "robot_patrol/robot_patrol/srv/detail/get_direction__struct.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using namespace std::chrono_literals;

using GetDirection = robot_patrol::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;

class ServiceTest : public rclcpp::Node {
private:
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  LaserScan laser_scan_data_;
  bool service_done_ = false; // TODO

  auto request = std::make_shared<GetDirection::Request>(laser_scan_data_);
  auto result_future = client_->async_send_request(
      request,
      std::bind(&ServiceTest::response_callback, this, std::placeholders::_1));

  // callbacks
  void timer_callback() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }
  }

  void laser_scan_callback(const LaserScan::SharedPtr data) {
    //   RCLCPP_DEBUG(this->get_logger(), "Laser scan callback");
    laser_scan_data_ = *data;
  }

  void
  response_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Result: success");
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

public:
  ServiceTest() : Node("service_test_node") {
    scan_sub_ = this->create_subscription<LaserScan>(
        "scan", 10, std::bind(&Patrol::laser_scan_callback, this, _1));

    client_ = this->create_client<GetDirection>("direction_service");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServiceTest::timer_callback, this));
  }

  bool is_service_done() const { return this->service_done_; }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<ServiceTest>();
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();
  return 0;
}
