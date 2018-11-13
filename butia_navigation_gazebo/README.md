

This directory contains some example launch files and other configuration that can
be used with the Gazebo simulator.

These are intended to serve as examples. Make copies to modify further for your
own projects.

For more information about using Gazebo with ROS, start at <http://wiki.ros.org/gazebo_ros_pkgs>,
and also read the documentation at <http://gazebosim.org>, especially <http://gazebosim.org/tutorials?cat=connect_ros>

Note: The launch files add the "models" subdirectory found here to the
`GAZEBO_MODEL_PATH` and `GAZEBO_RESOURCE_PATH` environment variables before
running Gazebo, so you can reference the example models there.  If you want to
keep your own separate models directory, either use `~/.gazebo/models`, or
remove these `<env ... />` tags and set `GAZEBO_MODEL_PATH` and
`GAZEBO_RESOURCE_PATH` in your shell. For example:

    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/my_gazebo_models
    export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/my_gazebo_models

You will need to structure your models directory as described in
<http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot>.

example-world.launch 
--------------------
An example roslaunch file that runs Gazebo with a simple empty world.
Use one of the "spawn" launch files (see below) to add robots.  Specify an alternate
world via `world` argument or modify the example world to add or change any of the objects (models) in the 
world.  Models can also be interactively added to a world from Gazebo: any models
found in ~/.gazebo/models, or installed on the system, or available from the 
gazebosim.world website will be listed in the "Models" list on the left panel in the
Gazbeo client GUi.

spawn.launch
------------
An example launch file for spawning simulated robots from urdf
files in gazebo if already running.  specify arguments with roslaunch:
  urdf  Path to URDF file
  name  Name of robot, only used an label in GUI

In addition to creating the robot in Gazebo, a `joint_state_publisher` and `robot_state_publisher` 
are launched as well.

e.g.:

  roslaunch spawn.launch urdf:=../description/urdf/pioneer3at.urdf.xacro name:=pioneer3at

If not specified, defaults to pioneer3at robot model.

Other example launch files default to other robot models:
* `spawn-pioneer3at.launch`: Spawn a Pioneer 3-AT

empty.world
-----------
This is just a copy of Gazebo's default "empty" world, containing only a plain
ground plane.

navigate-pioneer3at.launch
--------------------------
Simulate the navigation capabilities with the settings from `butia_navigation_description` using the Pioneer 3-AT. Either the Willow Garage World or Playground World can be used. Choose which one to use changing the "world" and "map" arguments in the launchfile.

slam-pioneer3at.launch
----------------------
Simulate SLAM capabilites with the settings from `butia_navigation_description`. The argument "world" can be changed to select either playground.world or willowgarage.world.