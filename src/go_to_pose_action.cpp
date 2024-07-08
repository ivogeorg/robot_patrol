#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
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

using GoToPose = robot_patrol::action::GoToPose;
using GoalHandlePose = rclcpp_action::ServerGoalHandle<GoToPose>;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using Pose2D = geometry_msgs::msg::Pose2D;

class GoToPoseActionServerNode : public rclcpp::Node {
public:
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

    wait_for_odom_publisher();
  }

private:
  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Publisher<Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  Odometry odom_data_;
  double yaw_rad_;
  Twist twist_;

  void odom_cb(const Odometry::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Odometry callback");

    odom_data_ = *msg;
    // NOTE: dynamic yaw
    yaw_rad_ = yaw_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
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

  void wait_for_odom_publisher() {
    // ROS 2 does't have an equivalent to wait_for_publisher
    // this is one way to solve the problem
    while (this->count_publishers("odom") == 0) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Interrupted while waiting for 'odom' topic publisher. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "'odom' topic publisher not available, waiting...");
    }
  }

  // in-place rotation (linear.x = 0.0)
  // assumes normalized angle [-pi/2.0, pi/2.0]
  // TODO: add goal_handle and result for
  // cancellation checks and feedback publishing
  enum class Frame { ROBOT, WORLD };
  bool rotate(double goal_norm_angle_rad, Frame frame,
              const std::shared_ptr<GoalHandlePose> goal_handle,
              std::shared_ptr<GoToPose::Result> result,
              std::shared_ptr<GoToPose::Feedback> feedback) {
    const double VELOCITY = 0.5;
    const double ANGULAR_TOLERANCE = 0.05;

    twist_.angular.z = (goal_norm_angle_rad > 0) ? VELOCITY : -VELOCITY;
    twist_.linear.x = 0.0;

    double last_angle = yaw_rad_;
    double turn_angle = 0.0;

    // if WORLD frame, subtract current robot yaw
    double goal_angle =
        (frame == Frame::ROBOT)
            ? goal_norm_angle_rad
            : normalize_angle(goal_norm_angle_rad - get_current_yaw());

    // DEBUG
    switch (frame) {
    case Frame::ROBOT:
      RCLCPP_DEBUG(this->get_logger(), "(rotate) Robot frame");
      break;
    case Frame::WORLD:
      RCLCPP_DEBUG(this->get_logger(), "(rotate) World frame");
      break;
    }
    RCLCPP_DEBUG(this->get_logger(), "(rotate) Goal angle (robot frame): %f",
                 goal_angle * 180 / PI_);
    // end DEBUG

    // Necessary code duplication to avoid inaccuracy
    // depending on the direction of rotation.
    // Notice the condition in the while loops.
    rclcpp::Rate rate(10);     // 10 Hz
    const int FB_DIVISOR = 10; // 10 for 1 Hz feedback
    int feedback_counter = 0;
    if (goal_angle > 0) {
      while (rclcpp::ok() &&
             (abs(turn_angle + ANGULAR_TOLERANCE) < abs(goal_angle))) {
        vel_pub_->publish(twist_);

        // check for cancellation
        if (goal_handle->is_canceling()) {
          // stop the robot
          twist_.linear.x = 0.0;
          twist_.angular.z = 0.0;
          vel_pub_->publish(twist_);

          // report cancellation
          result->status = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");

          return false;
        }

        // send feedback
        if (feedback_counter % FB_DIVISOR == 0) {
          Pose2D pose;
          pose.x = odom_data_.pose.pose.position.x;
          pose.y = odom_data_.pose.pose.position.y;
          pose.theta = get_current_yaw() * 180.0 / PI_;
          feedback->current_pos = pose;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "Publish feedback");
        }

        ++feedback_counter;
        rate.sleep();

        double temp_yaw = yaw_rad_;
        double delta_angle = normalize_angle(temp_yaw - last_angle);

        turn_angle += delta_angle;
        last_angle = temp_yaw;
      }
    } else {
      while (rclcpp::ok() &&
             (abs(turn_angle - ANGULAR_TOLERANCE) < abs(goal_angle))) {
        // TODO: check for cancellation
        // TODO: give feedback (current pos)
        vel_pub_->publish(twist_);

        // check for cancellation
        if (goal_handle->is_canceling()) {
          // stop the robot
          twist_.linear.x = 0.0;
          twist_.angular.z = 0.0;
          vel_pub_->publish(twist_);

          // report cancellation
          result->status = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");

          return false;
        }

        // send feedback
        if (feedback_counter % FB_DIVISOR == 0) {
          Pose2D pose;
          pose.x = odom_data_.pose.pose.position.x;
          pose.y = odom_data_.pose.pose.position.y;
          pose.theta = get_current_yaw() * 180.0 / PI_;
          feedback->current_pos = pose;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "Publish feedback");
        }

        ++feedback_counter;
        rate.sleep();

        double temp_yaw = yaw_rad_;
        double delta_angle = normalize_angle(temp_yaw - last_angle);

        turn_angle += delta_angle;
        last_angle = temp_yaw;
      }
    }
    // stop robot
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
    vel_pub_->publish(twist_);

    // DEBUG
    RCLCPP_INFO(this->get_logger(), "(rotate) Angular difference with goal: %f",
                abs(goal_norm_angle_rad - turn_angle));
    // end DEBUG
    
    return true;
  }

  // NOTE: dynamic distance
  double linear_distance(double goal_x, double goal_y) {
    return sqrt(pow(goal_x - odom_data_.pose.pose.position.x, 2.0) +
                pow(goal_y - odom_data_.pose.pose.position.y, 2.0));
  }

  // linear motion (angular.z = 0.0)
  // required speed linear.x = 0.2
  // TODO: add goal_handle and result for
  // cancellation checks and feedback publishing
  bool go_to(double goal_x_m, double goal_y_m,
             const std::shared_ptr<GoalHandlePose> goal_handle,
             std::shared_ptr<GoToPose::Result> result,
             std::shared_ptr<GoToPose::Feedback> feedback) {

    // TODO: To avoid indefinite motion due to inaccuracies
    //       in the rotation to the goal, calculate the
    //       distance to travel and stop when the distance
    //       has been travelled.

    const double VELOCITY = 0.2; // 0.2 is pretty high
    const double LINEAR_TOLERANCE = 0.005;

    twist_.linear.x = VELOCITY;

    rclcpp::Rate rate(10);     // 10 Hz
    const int FB_DIVISOR = 10; // 10 for 1 Hz feedback
    int feedback_counter = 0;

    double start_x = odom_data_.pose.pose.position.x, dx;
    double start_y = odom_data_.pose.pose.position.y, dy;
    double start_z = odom_data_.pose.pose.position.z, dz;

    double distance_to_goal =
        sqrt(pow(goal_x_m - start_x, 2.0) + pow(goal_y_m - start_y, 2.0));

    dx = odom_data_.pose.pose.position.x - start_x;
    dy = odom_data_.pose.pose.position.y - start_y;
    dz = odom_data_.pose.pose.position.z - start_z;

    while (sqrt(dx * dx + dy * dy + dz * dz) + LINEAR_TOLERANCE <=
           distance_to_goal) {
      vel_pub_->publish(twist_);

      // check for cancellation
      if (goal_handle->is_canceling()) {
        // stop the robot
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.0;
        vel_pub_->publish(twist_);

        // report cancellation
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");

        return false;
      }

      // send feedback
      if (feedback_counter % FB_DIVISOR == 0) {
        Pose2D pose;
        pose.x = odom_data_.pose.pose.position.x;
        pose.y = odom_data_.pose.pose.position.y;
        pose.theta = get_current_yaw() * 180.0 / PI_;
        feedback->current_pos = pose;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");
      }

      ++feedback_counter;
      rate.sleep();

      dx = odom_data_.pose.pose.position.x - start_x;
      dy = odom_data_.pose.pose.position.y - start_y;
      dz = odom_data_.pose.pose.position.z - start_z;
    }

    // stop robot
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
    vel_pub_->publish(twist_);

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(), "(go_to) Linear distance with goal: %f",
                linear_distance(goal_x_m, goal_y_m));
    // end DEBUG
    return true;
  }

  void INFO_current_position() {
    RCLCPP_INFO(this->get_logger(), "Current pos [%f, %f, %f]",
                odom_data_.pose.pose.position.x,
                odom_data_.pose.pose.position.y,
                yaw_from_quaternion(odom_data_.pose.pose.orientation.x,
                                    odom_data_.pose.pose.orientation.y,
                                    odom_data_.pose.pose.orientation.z,
                                    odom_data_.pose.pose.orientation.w));
  }

  void DEBUG_current_position() {
    RCLCPP_DEBUG(this->get_logger(), "Current pos [%f, %f, %f]",
                 odom_data_.pose.pose.position.x,
                 odom_data_.pose.pose.position.y,
                 yaw_from_quaternion(odom_data_.pose.pose.orientation.x,
                                     odom_data_.pose.pose.orientation.y,
                                     odom_data_.pose.pose.orientation.z,
                                     odom_data_.pose.pose.orientation.w));
  }

  // uses odom directly
  double get_current_yaw() {
    return yaw_from_quaternion(
        odom_data_.pose.pose.orientation.x, odom_data_.pose.pose.orientation.y,
        odom_data_.pose.pose.orientation.z, odom_data_.pose.pose.orientation.w);
  }

  void execute(const std::shared_ptr<GoalHandlePose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();

    // Goal is achieved in three main steps:
    // 1. The robot computes the angle to and rotates to face the goal pos.
    // 2. The robot moves forward to reach the goal pos.
    // 3. The robot rotates to face the angle theta in the world frame.

    // 1. Stop the robot
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
    vel_pub_->publish(twist_);

    // DEBUG
    DEBUG_current_position();
    RCLCPP_DEBUG(this->get_logger(), "Goal pos [%f, %f, %f]", goal->goal_pos.x,
                 goal->goal_pos.y, goal->goal_pos.theta);
    // end DEBUG

    // 2. Compute angle to goal pose vector
    // TODO: is this the right orientation?
    double delta_y = goal->goal_pos.y - odom_data_.pose.pose.position.y;
    double delta_x = goal->goal_pos.x - odom_data_.pose.pose.position.x;
    // double delta_y = odom_data_.pose.pose.position.y - goal->goal_pos.y;
    // double delta_x = odom_data_.pose.pose.position.x - goal->goal_pos.x;
    double world_angle = atan2(delta_y, delta_x);
    double robot_angle = world_angle - get_current_yaw();
    double norm_angle =
        normalize_angle(robot_angle); // TODO: shouldn't be necessary

    RCLCPP_DEBUG(this->get_logger(), "delta_y = %f", delta_y);
    RCLCPP_DEBUG(this->get_logger(), "delta_x = %f", delta_x);
    RCLCPP_DEBUG(this->get_logger(),
                 "Dir to face = atan2(dy, dx) = %f (world frame)",
                 world_angle * 180 / PI_);
    RCLCPP_DEBUG(this->get_logger(),
                 "Target robot yaw = atan2(dy, dx) = %f (robot frame)",
                 robot_angle * 180.0 / PI_);
    RCLCPP_DEBUG(this->get_logger(), "Norm robot yaw = %f",
                 norm_angle * 180.0 / PI_);

    // 3. Rotate toward goal
    bool rotation_1_res =
        rotate(robot_angle, Frame::ROBOT, goal_handle, result, feedback);

    // 4. Go to pose
    bool forward_res = go_to(goal->goal_pos.x, goal->goal_pos.y, goal_handle,
                             result, feedback);

    // 5. Convert from degrees to radians
    RCLCPP_DEBUG(this->get_logger(), "World angle to turn to = %f",
                 goal->goal_pos.theta);

    world_angle = goal->goal_pos.theta * PI_ / 180.0;

    // 6. Rotate to theta
    bool rotation_2_res =
        rotate(world_angle, Frame::WORLD, goal_handle, result, feedback);

    // check if goal is done and stop the robot
    if (rclcpp::ok() && rotation_1_res && forward_res && rotation_2_res) {
      // stop robot
      twist_.linear.x = 0.0;
      twist_.angular.z = 0.0;
      vel_pub_->publish(twist_);

      // report success
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    } // else it was cancelled in rotate or go_to
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPoseActionServerNode>();

  auto logger = rclcpp::get_logger("gotopose_server");

  // Set the log level to DEBUG
  if (rcutils_logging_set_logger_level(
          logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
    // Handle the error (e.g., print an error message or throw an exception)
    RCLCPP_ERROR(logger, "Failed to set logger level for 'gotopose_server'.");
  } else {
    RCLCPP_INFO(logger, "Successfully set logger level for 'gotopose_server'.");
  }

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(action_server);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}