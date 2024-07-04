#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"

#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "robot_patrol/action/go_to_pose.hpp"

/*
Inside the go_to_pose_action.cpp file, create an action server named
"/go_to_pose". In the callback, store the desired position received into a class
variable named: Pose2D desired_pos_

Inside the go_to_pose_action.cpp file, you need to subscribe to the odometry
topic "/odom". In the callback of "/odom", store the current odometry x, y and
theta into the class var: Pose2D current_pos_

Important: remember that the odometry is received in quaternion format. To get
the theta value you will need to convert the quaternion into Euler angles.
*/

#define PI_ 3.14159265359

class GoToPoseActionServerNode : public rclcpp::Node {
public:
  using GoToPose = robot_patrol::action::GoToPose;
  using GoalHandlePose = rclcpp_action::ServerGoalHandle<GoToPose>;
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;

  explicit GoToPoseActionServerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("gotopose_server", options) {

    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoToPose>(
        this, "go_to_pose",
        std::bind(&GoToPoseActionServerNode::handle_goal, this, _1, _2),
        std::bind(&GoToPoseActionServerNode::handle_cancel, this, _1),
        std::bind(&GoToPoseActionServerNode::handle_accepted, this, _1));

    vel_pub_ = this->create_publisher<Twist>("cmd_vel", 10);
    odom_sub_ = this->create_subscription<Odometry>(
        "odom", 10, std::bind(&GoToPoseActionServerNode::odom_cb, this, _1));
  }

private:
  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Publisher<Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  Odometry odom_data_;
  Twist twist_;

  void odom_cb(const Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Odometry callback");

    odom_data_ = *msg;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with pose [x=%f, y=%f, theta=%f]",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandlePose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePose> goal_handle) {
    using namespace std::placeholders;

    // needs to return quickly to avoid blocking the executor
    // so spin up a new thread
    std::thread{std::bind(&GoToPoseActionServerNode::execute, this, _1),
                goal_handle}
        .detach();
  }

  // utility functions for execute
  //  - assume the robot is stopped (lin = ang = 0.0)
  //  - assume no obstacles
  //  - they operate at 10 Hz for accuracy
  //  - they publish feedback (current pos) at 1 Hz
  //  - they read odom_data_
  //  - they publish Twist (vel_pub_)
  //  - they use predefined tolerances
  //  - they check for cancellation
  //  - they stop the robot when done
  //  - they return a boolean for success
  //    - cancellation: false
  //    - within tolerance: true
  //    - else: false

  double normalize_angle(double angle) {
    double res = angle;
    while (res > PI_)
      res -= 2.0 * PI_;
    while (res < -PI_)
      res += 2.0 * PI_;
    return res;
  }

  double yaw_from_quaternion(double x, double y, double z, double w) {
    return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
  }

  // in-place rotation (linear.x = 0.0)
  // assumes normalized angle [-pi/2.0, pi/2.0]
  bool rotate(double goal_norm_angle_rad) {}

  // linear motion (angular.z = 0.0)
  bool go_to(double goal_x_m, double goal_y_m) {}

  void execute(const std::shared_ptr<GoalHandlePose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose::Feedback>();
    // auto &message = feedback->current_pos;
    // message = "Starting movement...";
    auto result = std::make_shared<GoToPose::Result>();
    auto move = Twist();
    rclcpp::Rate loop_rate(1);

    // TODO: implement
    /**************************************
    1. Compute the line-of-sight vector in (x, y).
    2. Compute the angle between robot front and vector.
    3. Rotate robot around the computed angle.
    4. Go forward until the goal (x, y) pos is reached.
    5. Compute the angle between robot front and goal theta.
    6. Rotate robot around the computed angle.
    **************************************/

    // 1. Stop the robot
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
    vel_pub_.publish(twist_);

    // 2. Compute angle to goal pose vector
    double angle = normalize_angle(
        atan2(goal->goal_pos.y - odom_data_.pose.pose.position.y,
              goal->goal_pos.x - odom_data_.pose.pose.position.x));

    // 3. Rotate toward goal
    bool rotation_1_res = rotate(angle);

    // 4. Go to pose
    bool forward_res = go_to(goal->goal_pos.x, goal->goal_pos.y);

    // 5. Normalize theta
    angle = normalize_angle(goal->goal_pos.theta);

    // 6. Rotate to theta
    bool rotation_2_res = rotate(angle);

    // TODO: check if goal is done and stop the robot
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPoseActionServerNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(action_server);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}