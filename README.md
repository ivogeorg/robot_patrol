### robot_patrol

A turtlebot3 patrolling the simulated and real robot pen/polygon. Patrolling means _constantly moving and avoiding obstacles_.

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

#### Scanner orientation

In short, **angle zero** being "forward along the x-axis" doesn't mean that **index zero** of the `ranges` vector coincides. It is actually diametrically opposite.

1. In `tutrlebot3_burger.urdf`, there is no rotation in the joints between `base_footprint` and `base_link`, nor between `base_link` and `scan_base`.
2. In `empty_world.launch.py`, the robot appears in default pose with the forward direction along the x-axis. The LIDAR is therefore aligned the same way.
   ![Turtlebot3 in default pose in empty world](assets/turtlebot-empty-world-default-pose.png)  
3. In the simulator, I positioned the robot in such a way that it was unambiguous which index was 0, and it was the exact opposite of what `ros2 interface show sensor_msgs/msg/LaserScan` states (below). My index 0 is _backward_ (not _forward_) along the x-axis.
   ![Turtlebot3 positioned to make range readings unambiguous](assets/turtlebot-near-wall-for-scan-readings.png)
   Readings:
   ```  
   angle_min = -3.141600
   angle_max = 3.141600
   angle_increment = 0.009534
   range_min = 0.120000
   range_max = 30.000000
   ranges.size() = 660

   ranges[0] = 0.498451
   ranges[164] = 1.056713
   ranges[329] = 0.195327
   ranges[494] = 0.789187
   ranges[659] = 0.485331
   ```
4. `ros2 interface show sensor_msgs/msg/LaserScan` reads:
   ```
   std_msgs/Header header # timestamp in the header is the acquisition time of
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

   float32 angle_min            # start angle of the scan [rad]
   float32 angle_max            # end angle of the scan [rad]
   float32 angle_increment      # angular distance between measurements [rad]

   float32 time_increment       # time between measurements [seconds] - if your scanner
                                # is moving, this will be used in interpolating position
                                # of 3d points
   float32 scan_time            # time between scans [seconds]

   float32 range_min            # minimum range value [m]
   float32 range_max            # maximum range value [m]

   float32[] ranges             # range data [m]
                                # (Note: values < range_min or > range_max should be discarded)
   float32[] intensities        # intensity data [device-specific units].  If your
                                # device does not provide intensities, please leave
                                # the array empty.
   ```

#### Implementation notes

![View of the lab](assets/turtlebot-lab-camera-views.jpg)  
The actual TurtleBot3 lab.  

1. The hardest spots for the robot, where it tens to either start oscillating or (if the oscillation is actually only in the code) get stuck, are:
   1. Corners.
   2. Corner-like spots with traffic-sign obstacles.
   3. Finding itself too close (around or under `min_range`) to an obstacle.

2. Remediation measures:
   1. Direction safety (like convex-fitting) (default/fixed).
   2. **Bias** (optional):
      1. Larger angles.
      2. Larger ranges.
   3. Randomize (optional):
      1. Direction (left, right).
   4. **Extend** (optional):
      1. New direction span (from +/- pi to under +/- 2 * pi). 
   5. **Back up** (move backward) (optional);

3. Detecting problematic cases:
   1. Oscillation: count calls to `find_safest_direction` with no significant `x, y` movement.
   2. Too close and stuck: count `inf` values in the range `[FRONT_FROM, FRONT_TO]`.
   3. New directions too close to current orientation: within tolerance of current orientation.

4. `SOS` state that would relax the requirements and allow the robot to extricate itself when it detects that it is stuck. Some of the parameters to relax:
   1. Angle to look for new directions > +/- pi. Add `extended = false` parameter to `find_safest_direction`. _Watch for wraparound. May need to normalize at the edges._
   2. Slow backward movement (`cmd_vel_msg_.linear.x = -0.05;`) with close 360-degree monitoring of obstacles.  
   3. Note that if the robot is somehow stuck too close to an obstacle, it might be closes than `range_min` and therefore show `inf` values!  

5. (_advanced_) Add buffer space around obstacles to avoid the wheels catching the bases of the traffic signs. See lab camera views above.
   1. Measure arc-width of the detected obstacles.  
   2. Calculate the if one of the wheels might catch the obstacle.
   3. Modify the direction of the robot to avoid the obstacle.  
   4. Do this dynamically throughout as obstacle widths will vary depending on the (changing) orientation relative to the robot (point of view).  
   5. This will require the dimensions of the robot.

6. Multithreading protection.
   1. `laser_scan_data_` or `last_laser_scan_data_`.

##### TODO

1. `find_safest_direction`
   1. bias angle or range (in tuple vector for "safety" sorting)
      1. `enum class DirSafetyBias { ANGLE, RANGE };`
      2. `ANGLE` favors larger angles, `RANGE` favors longer ranges. Both after safety!!!
      3. new function parameter `dir_safety_bias = DirSafetyBias::ANGLE`
   2. direction bias
      1. `enum class DirBias { RIGHT, LEFT, RIGHT_LEFT, NONE };`
      2. `NONE` returns the index of the safest dir `v_indexed_averages[0]`, `LEFT_RIGHT` is based on the count of directions to the left and right.
      3. new function parameter `dir_bias = DirBias::NONE`
   3. deep review of the core "safety" criterion and strength of biases
2. oscillation 
   1. count in `State::FIND_NEW_DIR` and `State::TURNING` calls to `find_safest_direction` without significant `x, y` movement
   2. oscillation is usually just rotational so check for no linear progress in `State::TURNING`
   3. check the angle between the two directions _CONSIDER!!!_
   3. set private `is_oscillating_` to `true` and set `state` to `State::SOS`
3. too close to obstacle (possibly under `range_min`)
   1. count the `inf` in `obstacle_in_range`
   2. a std::tuple<bool, float> return value for `is_obstacle` and the ratio of `inf` (to all)
   3. check in `State::STOPPED` or `State::FORWARD` _CONSIDER!!!_
4. anomalous states _CONSIDER!!!_
   1. one `State::SOS` 
      1. pro: catch-all for anomalous situations, a unified strategy for extrication
      2. con: potentially too complex for pass-through
   2. `State::OSCILLATION` AND `State::TOO_CLOSE`
      1. pro: easier for pass-through
      2. con: different strategies might be redunant and/or error-prone