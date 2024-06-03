# `brownian_motion`
This ROS package provides a node that makes a robot doing Brownian movements while avoiding obstacles. It was originally developed in C language by Rui P. Rocha for ROS1 Indigo distribution in 2015 and has been migrated to ROS2 Humble distribution through this version.

#### Description
The package `brownian_motion` contains a node that makes a wheeled robot executing Brownian movements in a 2D planar workspace using either sonar or laser rangefinder data (exclusive or). The robot's navigation provided by the node does not involve any kind of path planning and is fully reactive. The algorithm can be briefly summarized as follows:
- Robot goes straight at maximum speed (and null angular speed) if no obstacles are detected or if they are all beyond a predefined distance equal to `safe_dist_th`;
- If obstacles are detected at a distance `d` such as `stop_dist_th < d <= safe_dist_th`, the robot's linear speed is reduced proportionally; it becomes zero for `d <= stop_dist_th`.
- The angular speed is different from zero for `d < safe_dist_th` so that the robot turns to avoid the nearest obstacle; the robot's rotation is more agressive for `d <= detour_obst_th`; note that `detour_obst_th > stop_dist_th`.
- The robot resumes straight motion at maximum speed after finding a direction free of obstacles.


#### Nodes

##### `brownian_motion`

###### Subscribed Topics
- `sonar` (`sensor_msgs::msg::PointCloud` message type)
    - Point cloud provided by an array of sonars.
- `laser` (`geometry_msgs::msg::PoseWithCovarianceStamped` message type)
    - Scan provided by a 2D laser rangefinder.

###### Published Topics
- `cmd_vel` (`geometry_msgs::msg::Twist` message type)
    - Velocity commands to the robot.


##### Parameters
- `max_linear_speed` (double, default: 0.20)
    - meter/second
- `max_angular_speed` (double, default: 1.25)
    - radian/second
- `safe_dist_th` (double, default: 0.75)
    - meter
- `detour_obst_th` (double, default: 0.60)
    - meter
- `stop_dist_th` (double, default: 0.40)
    - meter
- `diff_th` (double, default: 0.02)
    - minimum contribution of lateral objects to angular speed to not be considered negligible
    - radian/second
- `verbose` (boolean, default: false)
    - Display verbose logging messages
- `useLRF` (boolean, default: true)
    - If true, use laser rangefinder data rather than sonar data. If false, use sonar data instead.

##### `c_brownian_motion`
The node `c_brownian_motion` is identical to `brownian_motion`: same topics, parameter and  behavior.

The source code of `brownian_motion` uses C++ classes (which is more compliant with ROS2, especially in the node shutdown), whereas the source code of `c_brownian_motion` doesn't. The source code of the latter is therefore more similar to the original version developed for ROS Indigo distribution.

The use of the node `brownian_motion` is _recommended_. The other node was included just for historic reasons.