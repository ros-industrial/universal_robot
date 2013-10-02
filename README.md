universal_robot
===============

This repository provides ROS support for the universal robots.  This repo holds source code for all versions > groovy.  For those versions <= groovy see: hg https://kforge.ros.org/ros_industrial/universal_robot

To check that the package works with a UR5, set up a catkin workspace and clone the repository into the src/ folder. It should look like ~/catkin_ws/src/universal_robot. Don't forget to source the setup file ($ source ~/catkin_ws/devel/setup.*sh), then use catkin_make to compile.
You can then start the driver with the following commands (start new terminals, don't forget to source the setup shell files):

$ roslaunch ur_bringup ur5.launch robot_ip:=IP_OF_THE_ROBOT

$ roscd ur_driver; ./test_move.py


In order to start moveit! s.t. it can directly control the robot, close ur_bringup and type this:

$ roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=IP_OF_THE_ROBOT

Remember that you should always have your hands on the big red button in case there is something in the way, or anything unexcpected happens.
