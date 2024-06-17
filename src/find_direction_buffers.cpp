#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iterator>
#include <ostream>

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
  enum class DiscontinuityType { NONE, DROP, RISE };
  enum class LaserTargetType { CLEAR, OBSTACLE };
  const double F2B_RATIO_THRESHOLD = 0.5; // foreground to background
//   const double F2B_DIFF_THRESHOLD = 0.75; // foreground to background
//   const double F2B_DIFF_THRESHOLD = 0.70; // foreground to background
  const double F2B_DIFF_THRESHOLD = 0.65; // foreground to background
  sensor_msgs::msg::LaserScan laser_scan_data_;
  double direction_;

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void find_direction_buffers();
};

void LaserScanSubscriber::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "Laser scanner has %ld scan beams",
  // msg->ranges.size());
  if (!printed_scan_info_) {
    // RCLCPP_INFO(this->get_logger(), "angle_min = %f", msg->angle_min);
    // RCLCPP_INFO(this->get_logger(), "angle_max = %f", msg->angle_max);
    // RCLCPP_INFO(this->get_logger(), "angle_increment = %f",
    //             msg->angle_increment);
    // RCLCPP_INFO(this->get_logger(), "range_min = %f", msg->range_min);
    // RCLCPP_INFO(this->get_logger(), "range_max = %f", msg->range_max);
    // RCLCPP_INFO(this->get_logger(), "ranges.size() = %ld\n",
    //             msg->ranges.size());

    // RCLCPP_INFO(this->get_logger(), "ranges[0] = %f", msg->ranges[0]);
    // RCLCPP_INFO(this->get_logger(), "ranges[164] = %f", msg->ranges[164]);
    // RCLCPP_INFO(this->get_logger(), "ranges[329] = %f", msg->ranges[329]);
    // RCLCPP_INFO(this->get_logger(), "ranges[494] = %f", msg->ranges[494]);
    // RCLCPP_INFO(this->get_logger(), "ranges[659] = %f", msg->ranges[659]);

    // const double THRESHOLD = 0.5;
    // bool is_obstacle = false;
    // // bool is_inf = false;
    // int size = static_cast<int>(msg->ranges.size());
    // double last_range = msg->ranges[0], range;
    // double ratio;
    // for (int i = 1; i < size; ++i) {
    // //   std::cout << i << ": " << msg->ranges[i] << " (";
    //   range = msg->ranges[i];

    //   // NOTE:
    //   // No need to check for inf, because
    //   // (1) The fluke inf are going to be removed in scan_callback
    //   // (2) If too close, will back up first, which will remove the rest

    //   //   if (std::isinf(range)) {
    //   //     is_inf = true;
    //   //   } else {
    //   //     is_inf = false;
    //   // what if std::isinf(last_range) is true ???
    //   ratio = range / last_range;
    //   if (ratio < THRESHOLD)
    //     is_obstacle = true;
    //   else if (ratio > (1.0 / THRESHOLD))
    //     is_obstacle = false;
    //   //   }
    //   //   if (is_inf)
    //   //     std::cout << "inf)\n";
    //   //   else
    // //   std::cout << is_obstacle << ")\n";
    //   last_range = range;
    // }

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

    // TODO
    // 1. Assign to laser_scan_data_
    laser_scan_data_ = *msg;

    // 2. Filter inf ranges
    // Clean up stray inf (E.g. 0.5, inf, 0.45)
    int size = static_cast<int>(laser_scan_data_.ranges.size());
    double one, two, tri;
    int k;                           // for circular array wraparound
    for (int i = 0; i < size; ++i) { // [0, size-1]
      one = laser_scan_data_.ranges[i];
      k = (i + 1) % size;
      two = laser_scan_data_.ranges[k]; // prevent out-of-bound access
      tri = laser_scan_data_.ranges[k + 1];
      if (!std::isinf(one) && std::isinf(two) && !std::isinf(tri))
        // assign the previous value, preserving discontinuity
        laser_scan_data_.ranges[k] = laser_scan_data_.ranges[i];
    }

    // DEBUG
    int inf_ct = 0;
    for (auto &d : laser_scan_data_.ranges)
      if (std::isinf(d))
        ++inf_ct;
    std::cout << "Num inf: " << inf_ct << '\n' << std::flush;
    // end DEBUG

    // 3. Call find_direction_buffers
    find_direction_buffers();

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

void LaserScanSubscriber::find_direction_buffers() {
  RCLCPP_INFO(this->get_logger(), "Looking for safest direction");

  std::vector<double> ranges(laser_scan_data_.ranges.begin(),
                             laser_scan_data_.ranges.end());

  // 1. In a circular array, find the first discontinuity
  int size = static_cast<int>(ranges.size());

  double range, next_range;
  DiscontinuityType first_disc_type = DiscontinuityType::NONE;
  int i = 0;
  for (; i < size; ++i) {
    range = ranges[i];
    next_range = ranges[(i + 1) % size];
    if (range - next_range > F2B_DIFF_THRESHOLD) {
      first_disc_type = DiscontinuityType::DROP;
      break;
    } else if (next_range - range > F2B_DIFF_THRESHOLD) {
      first_disc_type = DiscontinuityType::RISE;
      break;
    }
  }

  int index_zero = (i + 1) % size;
  // DEBUG
  RCLCPP_INFO(this->get_logger(), "index_zero: %d", index_zero);
  switch (first_disc_type) {
  case DiscontinuityType::DROP:
    RCLCPP_INFO(this->get_logger(), "first_disc_type: DiscontinuityType::DROP");
    break;
  case DiscontinuityType::RISE:
    RCLCPP_INFO(this->get_logger(), "first_disc_type: DiscontinuityType::RISE");
    break;
  case DiscontinuityType::NONE:
  default:
    RCLCPP_INFO(this->get_logger(), "first_disc_type: DiscontinuityType::NONE");
    break;
  }
  // end DEBUG

  // 2. In a circular array, mark obstacles and clear
  LaserTargetType marker = (first_disc_type == DiscontinuityType::DROP)
                               ? LaserTargetType::OBSTACLE
                               : LaserTargetType::CLEAR;

  // DEBUG
  switch (marker) {
  case LaserTargetType::CLEAR:
    RCLCPP_INFO(this->get_logger(), "marker: LaserTargetType::CLEAR");
    break;
  case LaserTargetType::OBSTACLE:
    RCLCPP_INFO(this->get_logger(), "marker: LaserTargetType::OBSTACLE");
    break;
  }
  // end DEBUG

  std::vector<bool> obstacles(size);
  for (i = index_zero;; i = (i + 1) % size) {
    obstacles[i] = (marker == LaserTargetType::OBSTACLE) ? true : false;
    range = ranges[i];
    next_range = ranges[(i + 1) % size];
    // When to change the marker:
    // when DROP and CLEAR, or
    // when RISE and OBSTACLE
    if ((range - next_range > F2B_DIFF_THRESHOLD &&
         marker == LaserTargetType::CLEAR) ||
        (next_range - range > F2B_DIFF_THRESHOLD &&
         marker == LaserTargetType::OBSTACLE))
      switch (marker) {
      case LaserTargetType::OBSTACLE:
        marker = LaserTargetType::CLEAR;
        break;
      case LaserTargetType::CLEAR:
        marker = LaserTargetType::OBSTACLE;
        break;
      }

    if (i == (index_zero - 1 + size) % size)
      break;
  }

  // DEBUG
  std::cout << "Obstacles marked:\n";
  for (i = 0; i < size; ++i)
    std::cout << i << ": " << obstacles[i] << " (" << ranges[i] << ")\n";
  std::cout << '\n' << std::flush;
  // end DEBUG

  // 3. In a circular array, identify the clear spans

  // TODO: Add buffer application, width filtering, and search range cutoff

  int start_ix, end_ix;
  // starting at index_zero
  // whether clear or obstacle depends on the
  // marker type at index_zero (that is, there
  // is or isn't an obstacle at index_zero)
  bool in_clear_span = !obstacles[index_zero];
  int num_clear_spans = 0;

  for (i = index_zero;; i = (i + 1) % size) {
    if (!in_clear_span && !obstacles[i]) { // obstacle is over
      in_clear_span = true;
      start_ix = i;
    } else if (in_clear_span && obstacles[i]) { // obstacle starts
      in_clear_span = false;
      end_ix = i - 1; // don't count the beginning of the obstacle
      // DEBUG
      RCLCPP_INFO(this->get_logger(), "Clear span: [%d, %d]", start_ix, end_ix);
      // end DEBUG
      ++num_clear_spans;
    }

    // We start at a discontinuity, so need to count the last span
    if (i == (index_zero - 1 + size) % size) {
      if (in_clear_span) {
        end_ix = i;
        // DEBUG
        RCLCPP_INFO(this->get_logger(), "Clear span: [%d, %d]", start_ix,
                    end_ix);
        // end DEBUG
        ++num_clear_spans;
      }
      break;
    }
  }

  // DEBUG
  RCLCPP_INFO(this->get_logger(), "Num clear spans: %d", num_clear_spans);
  // end DEBUG
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}