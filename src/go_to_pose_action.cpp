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

    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
    subscriber_ = this->create_subscription<Odometry>(
        "odom", 10, std::bind(&GoToPoseActionServerNode::odom_cb, this, _1));
  }

private:
  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscriber_;
  Odometry odom_data_;

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