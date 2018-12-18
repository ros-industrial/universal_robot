# Universal Robot

[![Build Status](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

[ROS-Industrial](http://wiki.ros.org/Industrial) Universal Robot meta-package. See the [ROS wiki](http://wiki.ros.org/universal_robot) page for compatibility information and other more information.

This repository provides ROS support for the universal robots.  This repo holds source code for all versions > groovy.  For those versions <= groovy see: hg https://kforge.ros.org/ros_industrial/universal_robot

__Installation__  
On supported Linux distributions (Ubuntu, up to 16.04 (Xenial), i386 and amd64):

```sudo apt-get install ros-kinetic-universal-robot```


__Installation/Building from Source__  
There are releases available for ROS Hydro, Indigo and kinetic. However, for the latest features and developments you might want to install from source.

The following instructions assume that a [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) has been created at $HOME/catkin_ws and that the source space is at $HOME/catkin_ws/src. Update paths appropriately if they are different on the build machine.

In all other cases the packages will have to be build from sources in a Catkin workspace: 

```
cd /path/to/catkin_ws/src

# retrieve the sources (replace '$DISTRO' with the ROS version you are using)
git clone -b $DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd /path/to/catkin_ws

# checking dependencies (replace '$DISTRO' with the ROS version you are using)
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $DISTRO

# building
catkin_make

# source this workspace (careful when also sourcing others)
source /path/to/catkin_ws/devel/setup.bash
```

---

__Usage with real Hardware__  
There are launch files available to bringup a real robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the real robot, run:

```roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]```


CAUTION:  
Remember that you should always have your hands on the big red button in case there is something in the way or anything unexpected happens.


__MoveIt! with real Hardware__  
Additionally, you can use MoveIt! to control the robot.  
There exist MoveIt! configuration packages for both robots.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


NOTE:  
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:  

```roslaunch ur_bringup ur5_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


---

__Usage with Gazebo Simulation__  
There are launch files available to bringup a simulated robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the simulated robot in Gazebo, run:

```roslaunch ur_gazebo ur5.launch```


__MoveIt! with a simulated robot__  
Again, you can use MoveIt! to control the simulated robot.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


NOTE:  
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:  

```roslaunch ur_gazebo ur5.launch limited:=true```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


