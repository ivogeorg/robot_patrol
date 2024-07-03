#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/action/go_to_pose.hpp"

/*
Inside the go_to_pose_action.cpp file, create an action server named "/go_to_pose".
In the callback, store the desired position received into a class variable named:
Pose2D desired_pos_

Inside the go_to_pose_action.cpp file, you need to subscribe to the odometry topic "/odom".
In the callback of "/odom", store the current odometry x, y and theta into the class var:
Pose2D current_pos_

Important: remember that the odometry is received in quaternion format. To get the theta value you will need to convert the quaternion into Euler angles.
*/



class GoToPose {};

int main() {

    return 0;
}