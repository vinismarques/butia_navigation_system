
This directory contains URDF model descriptions of some AMR robots.

config
        Contains configuration files for the nodes of the navigation stack
config/param
        Contains parameters for each specific node of the navigation stack

urdf    Contains URDF descriptions (xacro and URDF xml formats) as well as some launch
        files that display them in rviz.

model   Contains visual models

meshes  Contains lower resolution meshes for simulation, collision, etc.

src     Contains some urdf publisher source code (not normally needed, since 
        robot_state_publisher can usually be used, or some ROS tools such as
        rviz can load the URDF directly)


How to compile URDF from XACRO files
------------------------------------

If you modify any XACRO files, the you must
re-generate the plain-XML URDF file from it.
You can use the Makefile in the urdf directory,
or do it manually using xacro.py.

Manually convert XACRO file to plain-XML URDF file:

    rosrun xacro xacro.py file.urdf.xacro >file.urdf

Validate URDF file syntax:

    check_urdf <urdffile>

e.g.:

    check_urdf urdf/pioneer-lx.urdf.xacro

You will need to install liburdfdom-tools to get `check_urdf`:
  
    sudo apt-get install liburdfdom-tools

There is a Makefile in the urdf directory that converts all xacro files
to plain-xml urdf and also runs `check_urdf` on them:

    cd src/amr-ros-config/description/urdf
    make

