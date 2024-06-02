#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class LaserScanSubscriber : public rclcpp::Node {
public:
  LaserScanSubscriber() : Node("laser_scan_subscriber_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&LaserScanSubscriber::laser_scan_callback, this, _1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  bool printed_scan_info_ = false;

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

void LaserScanSubscriber::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "Laser scanner has %ld scan beams",
  // msg->ranges.size());
  if (!printed_scan_info_) {
    RCLCPP_INFO(this->get_logger(), "angle_min = %f", msg->angle_min);
    RCLCPP_INFO(this->get_logger(), "angle_max = %f", msg->angle_max);
    RCLCPP_INFO(this->get_logger(), "angle_increment = %f",
                msg->angle_increment);
    RCLCPP_INFO(this->get_logger(), "range_min = %f", msg->range_min);
    RCLCPP_INFO(this->get_logger(), "range_max = %f", msg->range_max);
    RCLCPP_INFO(this->get_logger(), "ranges.size() = %ld\n", msg->ranges.size());

    RCLCPP_INFO(this->get_logger(), "ranges[0] = %f", msg->ranges[0]);
    RCLCPP_INFO(this->get_logger(), "ranges[179] = %f", msg->ranges[179]);
    RCLCPP_INFO(this->get_logger(), "ranges[359] = %f", msg->ranges[359]);
    RCLCPP_INFO(this->get_logger(), "ranges[539] = %f", msg->ranges[539]);
    RCLCPP_INFO(this->get_logger(), "ranges[719] = %f", msg->ranges[719]);

    /**
        This shows that the turtlebot3 laser scan is parameterized thus:
                 329
              _________
            |||   ^   |||
            |||  fwd  |||
        493   |l     r|   164
              |       |
              |___b___|
                  0 (659)
        These are the indices of the ranges array:
        329 - forward (also approximately 359)
        493 - left
          0 - backward
        164 - right
    */


    // print only once
    printed_scan_info_ = true;
  }
}
/*
float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your
scanner # is moving, this will be used in interpolating position # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
*/

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}