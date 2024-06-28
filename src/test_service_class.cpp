#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>

using namespace std::chrono_literals;

using GetDirection = robot_patrol::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;

class ServiceTest : public rclcpp::Node {
private:
  std::string laser_scan_topic_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;

  std::string direction_service_name_;
  rclcpp::Client<GetDirection>::SharedPtr client_;

  rclcpp::TimerBase::SharedPtr service_call_timer_;

  LaserScan laser_scan_data_;

  // TODO:
  // 1. Create init()
  // 2. Create service_call()
  // 3. In service_call():
  //    1. Create a request for GetDirection, initializing with laser_scan_data_
  //    2. Send request and bind velocity_callback()
  // 4. In init():
  //    1. Wait for scan data publisher
  //    2. Wait for direction service
  //    3. Start service_call_timer_, binding service_call()
  // 5. In velocity_callback(), wait for the response and print "direction"
  // 6. Call init() from main

  void service_call() {
    auto request = std::make_shared<GetDirection::Request>(laser_scan_data_);

    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceTest::velocity_callback, this,
                           std::placeholders::_1));
  }

  // callbacks
  void laser_scan_callback(const LaserScan::SharedPtr data) {
    //   RCLCPP_DEBUG(this->get_logger(), "Laser scan callback");
    laser_scan_data_ = *data;
  }

  void velocity_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status != std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Service '%s' in progress...",
                  direction_service_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Service '%s' response",
                  direction_service_name_.c_str());
    }
    std::string dir = future.get();
    RCLCPP_INFO(this->get_logger(), "Direction '%s'", dir.c_str());
  }

  // utilities
  void wait_for_laser_scan_publisher() {
    while (this->wait_for_publisher(laser_scan_topic_, 10s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for '%s' publisher. Terminating...",
            laser_scan_topic_.c_str());
        return 1;
      } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for '%s' publisher...",
                    laser_scan_topic_.c_str());
      }
    }
  }

  void wait_for_direction_server() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for '%s' server. Terminating...",
            direction_service_name_.c_str());
        return 1;
      } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for '%s' server...",
                    direction_service_name_.c_str());
      }
    }
  }

  void start() {
    service_call_timer_ = this->create_wall_timer(
        100ms, std::bind(&ServiceTest::service_call, this));
  }

public:
  ServiceTest() : Node("service_test_node") {
    laser_scan_topic_{"scan"};
    scan_sub_ = this->create_subscription<LaserScan>(
        laser_scan_topic_, 10,
        std::bind(&ServiceTest::laser_scan_callback, this, _1));

    direction_service_name_{"direction_service"};
    client_ = this->create_client<GetDirection>(direction_service_name_);

    // TODO: bind velocity_callback
    // service_call_timer_ = this->create_wall_timer(
    //     1s, std::bind(&ServiceTest::service_call_timer_callback, this));
  }

  void init() {
    wait_for_laser_scan_publisher();
    wait_for_direction_server();
    start();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<ServiceTest>();

  service_client.init();

  rclcpp::spin(service_client);
  rclcpp::shutdown();
  return 0;
}
