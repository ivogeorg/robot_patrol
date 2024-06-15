#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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
  const double F2B_RATIO_THRESHOLD = 0.5; // foreground to background
  const double F2B_DIFF_THRESHOLD = 0.75; // foreground to background
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
    std::cout << "Num inf: " << inf_ct << '\n';
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

  // Mark obstacles in a circular array:
  // The algorithm marks 1s and 0s, switching on a discontinuity,
  // either drop or jump. Foreground obstacles are marked by a drop
  // and after that a jump. If the first discontinuity is a drop,
  // obstacles are the 1s; if the first discontinuity is a rise,
  // obstacles are the 0s.
  int size = static_cast<int>(ranges.size());
  std::vector<bool> obstacle_or_clear(size);
  bool obs_or_clr = false; // which is which afterwards...

  DiscontinuityType disc_type = DiscontinuityType::NONE;
  double range, next_range;
  //   double ratio;
  double diff;
  // The obstacles will be between the lower-indexed
  // sides of the discontinuities.
  int first_disc_ix; // Need for the open span extraction.
  for (int i = 0; i < size; ++i) {
    std::cout << i << ": " << ranges[i] << " (";
    range = ranges[i];
    next_range = ranges[(i + 1) % size]; // circular array
    // ratio = next_range / range;
    diff = next_range - range;
    if (-diff > F2B_DIFF_THRESHOLD) {
      // if (ratio < F2B_RATIO_THRESHOLD) {
      if (disc_type == DiscontinuityType::NONE) {
        disc_type = DiscontinuityType::DROP; // open obst span
        first_disc_ix = i;
        // DEBUG
        std::cout << "DROP, ";
        // end DEBUG
      }
      obs_or_clr = true;
    } else if (diff > F2B_DIFF_THRESHOLD) {
      // } else if (ratio > (1.0 / F2B_RATIO_THRESHOLD)) {
      if (disc_type == DiscontinuityType::NONE) {
        disc_type = DiscontinuityType::RISE; // close obst span
        first_disc_ix = i;
        // DEBUG
        std::cout << "RISE, ";
        // end DEBUG
      }
      obs_or_clr = false;
    }
    obstacle_or_clear[i] = obs_or_clr;
    std::cout << obs_or_clr << ")\n" << std::flush;
  }

  // TODO
  // ----
  // ERROR - off-by-one
  // ------------------
  // [laser_scan_subscriber_node-1] 247: 0.704709 (1)
  // [laser_scan_subscriber_node-1] 248: 0.71184 (1)
  // [laser_scan_subscriber_node-1] 249: 0.729146 (0)
  // [laser_scan_subscriber_node-1] 250: 2.13156 (0)
  // [laser_scan_subscriber_node-1] 251: 2.14458 (0)

  // 3. Extract open spans (between obstacles) in circular array
  //    Apply right and left endpoints
  //    Filter by width???

  // If the first discontinuity is a DROP, obstacles are the 1s;
  // if the first discontinuity is a RISE, obstacles are the 0s.
  bool obstacle_marker = (disc_type == DiscontinuityType::DROP) ? true : false;
  // DEBUG
  std::cout << "obstacle_marker: " << obstacle_marker << '\n';
  // end DEBUG

  // Extract clear spans
  //   std::vector<std::pair<int, int>> clear_spans;
  //   /*
  // width, middle_ix, distance
  std::vector<std::tuple<int, int, double>> clear_spans;
  //   */
  int start_ix, end_ix, middle_ix, width;
  double mid_range;
  bool is_clear_span = false;

  // Circular array
  for (int i = first_disc_ix;; i = (i + 1) % size) {

    if (obstacle_or_clear[i] == obstacle_marker) { // obstacle
      // if open clear span, close and add clear span
      if (is_clear_span) {
        end_ix = i;

        // compute width and middle index
        if (start_ix < end_ix) {
          // normal case
          width = end_ix - start_ix - 1; // TODO: Verify!!!
          middle_ix = static_cast<int>(lround((end_ix + start_ix) / 2.0));
        } else {
          // accross the discontinuity [size - 1, 0]
          width = size - start_ix - 1 + end_ix; // TODO: Verify!!!
          middle_ix = static_cast<int>(start_ix + width / 2.0) % size;
        }
        mid_range = ranges[middle_ix];

        // append to vector
        // width, middle_ix, distance
        // NOTE: This leaves the door open to adding
        //       buffer angles to foreground obstacles.
        clear_spans.push_back(std::make_tuple(width, middle_ix, mid_range));

        // DEBUG
        std::cout << "Clear span:\n";
        std::cout << "start_ix: " << start_ix << '\n';
        std::cout << "end_ix: " << end_ix << '\n';
        std::cout << "width: " << width << '\n';
        std::cout << "middle_ix: " << middle_ix << '\n';
        std::cout << "mid_range: " << mid_range << "\n\n" << std::flush;
        // end DEBUG

        is_clear_span = false;
      }      // else, do nothing
    } else { // clear
      // if no clear span open, start a new one
      if (!is_clear_span) {
        is_clear_span = true;
        start_ix = i;
      } // else, do nothing
    }

    if (i == first_disc_ix - 1)
      break;
  }

  // DEBUG
  std::cout << "Num clear spans in vector: " << clear_spans.size() << '\n'
            << std::flush;
  // end DEBUG

  // 4. Sort by width
  std::sort(clear_spans.begin(), clear_spans.end(),
            [](const std::tuple<int, int, double> &a,
               const std::tuple<int, int, double> &b) {
              return std::get<0>(a) > std::get<0>(b);
            });

  // 6. Take the top candidate
  std::tie(width, middle_ix, mid_range) = clear_spans[0];
  direction_ = middle_ix;

  // DEBUG
  std::cout << "direction_: " << direction_ << '\n' << std::flush;
  // end DEBUG
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}