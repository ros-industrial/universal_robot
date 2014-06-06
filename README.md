universal_robot
===============

This repository provides ROS support for the universal robots.  This repo holds source code for all versions > groovy.  For those versions <= groovy see: hg https://kforge.ros.org/ros_industrial/universal_robot

To check that the package works with a UR5, set up a catkin workspace and clone the repository into the src/ folder. It should look like ~/catkin_ws/src/universal_robot. Don't forget to source the setup file ($ source ~/catkin_ws/devel/setup.*sh), then use catkin_make to compile.
You can then start the driver with the following commands (start new terminals, don't forget to source the setup shell files):

$ roslaunch ur_bringup ur5.launch robot_ip:=IP_OF_THE_ROBOT

$ roscd ur_driver; ./test_move.py

BEWARE:
Remember that you should always have your hands on the big red button in case there is something in the way, or anything unexcpected happens.



Additionally, you can use MoveIt! to control the robot.
For setting up the MoveIt! nodes to allow motion planning, run in a new terminal:

$ roslaunch ur5_moveit_config move_group.launch

In order to be able to use RViz to trigger Planning Request using the MoveIt! Plugin for RViz, run in a new terminal:

$ roslaunch ur5_moveit_config moveit_rviz.launch config:=true

NOTE: 
As MoveIt! seems to have difficulties with finding plans for the UR with joint limits [-2pi, 2pi], there is a joint_limited version using joint limits [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited' for both ur_bringup robot.launch and urX_moveit_conifg move_group.launch:

$ roslaunch ur_bringup ur5.launch limited:=true robot_ip:=IP_OF_THE_ROBOT

$ roslaunch ur5_moveit_config move_group.launch limited:=true

$ roslaunch ur5_moveit_config moveit_rviz.launch config:=true




