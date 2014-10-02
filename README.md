universal_robot
===============

This repository provides ROS support for the universal robots.  This repo holds source code for all versions > groovy.  For those versions <= groovy see: hg https://kforge.ros.org/ros_industrial/universal_robot

To check that the package works with a UR5, set up a catkin workspace and clone the repository into the src/ folder. It should look like ~/catkin_ws/src/universal_robot. Don't forget to source the setup file ($ source ~/catkin_ws/devel/setup.*sh), then use catkin_make to compile.


The following will show the commands needed to bringup either REAL or SIMULATED robots.
Both robots (ur5 and ur10) can be used in the same way. Simply replace the prefix accordingly.
For each command use a new terminal (don't forget to source the setup shell files)!

To bring up the REAL robot, run:

```roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT reverse_port:=REVERSE_PORT```

To bring up the SIMULATED robots, run:

```roslaunch ur_gazebo ur5.launch```

A simple test script that moves the robot to predefined positions can be executed like this:

```roscd ur_driver; ./test_move.py```


BEWARE:
Remember that you should always have your hands on the big red button in case there is something in the way, or anything unexcpected happens.



Additionally, you can use MoveIt! to control the robot.
For setting up the MoveIt! nodes to allow motion planning with the REAL robot, run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch```

For setting up the MoveIt! nodes to allow motion planning with the SIMULATED robot, run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true```

In order to be able to use RViz to trigger Planning Request using the MoveIt! Plugin for RViz, run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```

NOTE: 
As MoveIt! seems to have difficulties with finding plans for the UR with joint limits [-2pi, 2pi], there is a joint_limited version using joint limits [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited':

```roslaunch ur_bringup ur5_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT reverse_port:=REVERSE_PORT```

OR

```roslaunch ur_gazebo ur5.launch limited:=true```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true```

OR

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true sim:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```




