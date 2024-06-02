### robot_patrol

A turtlebot3 patrolling the simulated and real robot pen/polygon. Patrolling means constantly moving and avoiding obstacles.

#### Submission notes

`colcon build --packages-select robot_patrol`  
`ros2 launch turtlebot3_gazebo main_turtlebot3_lab.launch.xml`  
`ros2 launch robot_patrol start_patrolling.launch.py`

1. 

#### Laser scan orientation and parameterization

##### Simulator

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


##### Lab

![Turtlebot lab scanner has 720 rays](assets/turtlebot-lab-scanner-parameters.jpg)  


#### Current yaw

1. The current yaw of the robot is extracted from the odometry message:
   ```c++
   yaw_ = yaw_from_quaternion(
       msg->pose.pose.orientation.x, 
       msg->pose.pose.orientation.y, 
       msg->pose.pose.orientation.z,
       msg->pose.pose.orientation.w);`
2. Using the formula, this works fine in the Gazebo simulator:
   `atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);`
3. The values are +/- pi radians, equivalent to +/- 180 degrees.


#### Requirements & todo

![View of the lab](assets/turtlebot-lab-camera-views.jpg)  

1. Scanner values are hardcoded. Need dynamic self-parameterization. Note that indices always grow counterclockwise (CCW) from 0 to the max. _Index `0` seems to be in the backward orientation of the robot. **To verify!**_
2. Lab scanner returns `inf` values for some ranges. Need to guard and either ignore or set values to one of two alternative numbers:
   1. `range_max`. or 
   2. A suitable average value for the space which is well above the obstacle threshold (**0.35 m**) but also won't throw off the calculations, esp. the sums of neighboring ray-ranges for the peak ranges. The space is about **2.0 m x 1.8 m**.  
3. The robot got stuck very quickly in the lab, where the obstacles are farther apart and harder to detect than the simulator. Need to:
   1. Monitor for the state of getting stuck and possibly oscillating without positive outcome. These will be:
      1. Count of finding new directions without movement.
      2. New directions that are too close to current orientation and/or which automatically fall within tolerance of current orientation.
      3. Exactly how close is the robot to an obstacle "in front".
   2. Add an `SOS` state that would relax the requirements and allow the robot to extricate itself when it detects that it is stuck. Some of the parameters to relax:
      1. Angle to look for new directions > +/- pi. Add `extended = false` parameter to `find_safest_direction`. _Watch for wraparound. May need to normalize at the edges._
      2. Slow backward movement (`cmd_vel_msg_.linear.x = -0.05;`) with close 360-degree monitoring of obstacles.
   3. (_advanced_) Add buffer space around obstacles to avoid the wheels catching the bases of the traffic signs. See lab camera views above.
      1. Measure arc-width of the detected obstacles.  
      2. Calculate the if one of the wheels might catch the obstacle.  
      3. Modify the direction of the robot to avoid the obstacle.  
      4. Do this dynamically throughout as obstacle widths will vary depending on the (changing) orientation relative to the robot (point of view).  

