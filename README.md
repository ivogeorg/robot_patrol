### robot_patrol

A turtlebot3 patrolling the simulated and real robot pen/polygon. Patrolling means constantly moving and avoiding obstacles.

#### Laser scan orientation and parameterization

```
[laser_scan_subscriber_node]: angle_min = -3.141600
[laser_scan_subscriber_node]: angle_max = 3.141600
[laser_scan_subscriber_node]: angle_increment = 0.009534
[laser_scan_subscriber_node]: range_min = 0.120000
[laser_scan_subscriber_node]: range_max = 30.000000
[laser_scan_subscriber_node]: ranges.size() = 660

[laser_scan_subscriber_node]: ranges[0] = 1.610980
[laser_scan_subscriber_node]: ranges[164] = 1.600037
[laser_scan_subscriber_node]: ranges[329] = 0.226959
[laser_scan_subscriber_node]: ranges[493] = 0.470259
[laser_scan_subscriber_node]: ranges[659] = 1.572943
```

```
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
```

#### Requirements & todo

1. Port (from ROS 1 to ROS 2) the [`_rotate`](https://github.com/ivogeorg/my_rb1_robot/blob/ece261459d47d661b5d7ccb5789d8b71e6de308c/my_rb1_ros/src/rotate_service.cpp#L96) code into a private function. Call it `rotate_`.  
2. Update `rotate_` to work with radians, not degrees.  
3. Utilize the **required** `direction_` private variable, the angle between the current forward direction of the robot and the direction of the longest range from the scanner, within the +/- pi/2 radians relative to the forward direction, in two ways:
   1. This will be the argument of `rotate_`, explicitly or implicitly.
   2. This will determine the angular velocity to set in `vel_cmd_msg_`.    
5. Subscribe to `/odom` and get the _current yaw_ as a starting point of the rotation. The odometry callback will be just one line.  
