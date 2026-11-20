## Overview

This package is part of the [ROS-Industrial](Industrial) program. It is
the MoveIt! configuration for the UR3 arm, generated automatically by
the MoveIt Setup Assistant.

## Getting Started



```wiki
The current binary releases of the UR MoveIt configuration packages do not install all required dependencies upon installation (see [[https://github.com/ros-industrial/universal_robot/issues/225|universal_robot/issues/225]]). Be sure to install the `moveit_simple_controller_manager` package in addition to those mentioned below.
```



Install the package from package management, and run the !MoveIt!
planning demo: 

```
$ sudo apt-get install ros-$ROS_DISTRO-ur3-moveit-config

$ roslaunch ur3_moveit_config demo.launch
```



This is not a real simulation, just a demonstration of the planning
capability and the !MoveIt! and RViz integration. For true simulation of
a UR3, see the [ur_gazebo](https://index.ros.org/p/ur_gazebo) package.

## Additional information

See also the relevant sections in the
[readme](https://github.com/ros-industrial/universal_robot/blob/indigo-devel/README.md)
on Github.
