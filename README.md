# BUTIA-Bots Navigation and Mapping System

This repository contains a collection of ROS (Robot Operating System) packages needed for Simultaneous Localization and Mapping (SLAM), navigating and also performing simulations.

### Nodes

  - [gmapping](http://wiki.ros.org/gmapping):
        Provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called `slam_gmapping`. Using `slam_gmapping`, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.
  - [amcl](http://wiki.ros.org/amcl):
        `amcl` is a probabilistic localization system for a robot moving in 2D. It implements the adaptive (or KLD-sampling) Monte Carlo localization approach (as described by Dieter Fox), which uses a particle filter to track the pose of a robot against a known map. 
  - [move_base](http://wiki.ros.org/move_base):
        Provides an implementation of an action (see the actionlib package) that, given a goal in the world, will attempt to reach it with a mobile base. The move_base node links together a global and local planner to accomplish its global navigation task. It supports any global planner adhering to the nav_core::BaseGlobalPlanner interface specified in the nav_core package and any local planner adhering to the nav_core::BaseLocalPlanner interface specified in the nav_core package. The move_base node also maintains two costmaps, one for the global planner, and one for a local planner (see the costmap_2d package) that are used to accomplish navigation tasks.
  - [urg_node](http://wiki.ros.org/urg_node):
        Driver for Hokuyos laser range-finders.

### Topics

  - /initialpose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)):
        Useful for setting the start pose of the robot. Used by `amcl`.
  - /map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)):
        A latched topic (meaning that the data is sent once to each new subscriber) that contains information about the occupancy grid. Usually created by a SLAM mapping service and saved to disk by the `map_server`.
  - /move_base_simple/goal ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)):
        Provides a non-action interface to `move_base` for users that don't care about tracking the execution status of their goals. Useful for setting a goal through Rviz, for example. The `move_base` package subscribes to this topic and use it to set new goals.
  - /move_base/goal ([move_base_msgs/MoveBaseActionGoal](http://docs.ros.org/api/move_base_msgs/html/msg/MoveBaseActionGoal.html)):
        A goal for `move_base` to pursue in the world. Can be used if tracking status is needed.
  - /move_base/cancel [actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html)
        A request to cancel a specific goal.
  - /move_base/status ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html)): 
        Provides status information on the goals that are sent to the `move_base` action. 
  - /move_base/result [move_base_msgs/MoveBaseActionResult](http://docs.ros.org/api/move_base_msgs/html/msg/MoveBaseActionResult.html):
        Publishes only the final status of the goal. Status code 3 means it was sucessful in reaching the goal, code 4 means it was aborted due to some failure.
  - /tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html)):
        Contains coordinate frames for moving parts.
  - /tf_static ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html)):
        Contains transform information about static components (laser, kinetic, robot model, etc.). Used mostly internally.
  - /scan ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)):
        Laser scans to create the map from and to create the costmap. 
  - /cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)):
        Used to control the robot motors. Used internally by the nodes. Used by `move_base` and `teleop_twist_joy`.

### Bagfiles
Recorded bags can be found on [Google Drive](https://drive.google.com/drive/folders/1SaCmBkyTRnN6mYBaKE2m19mB6H5DNCoK)