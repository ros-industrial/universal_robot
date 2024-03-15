# Universal Robot

[![Build Status: Ubuntu Bionic (Actions)](https://github.com/ros-industrial/universal_robot/workflows/CI%20-%20Ubuntu%20Bionic/badge.svg?branch=melodic-devel)](https://github.com/ros-industrial/universal_robot/actions?query=workflow%3A%22CI+-+Ubuntu+Bionic%22)
[![Build Status: Ubuntu Focal (Actions)](https://github.com/ros-industrial/universal_robot/workflows/CI%20-%20Ubuntu%20Focal/badge.svg?branch=melodic-devel)](https://github.com/ros-industrial/universal_robot/actions?query=workflow%3A%22CI+-+Ubuntu+Focal%22)

[![License - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![License](https://img.shields.io/badge/License-Universal%20Robots%20A/S%E2%80%99%20Terms%20and%20Conditions%20for%20Use%20of%20Graphical%20Documentation-blue.svg)](https://www.universal-robots.com/legal/terms-and-conditions/terms_and_conditions_for_use_of_graphical_documentation.txt)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

[ROS-Industrial](https://wiki.ros.org/Industrial) Universal Robot meta-package. See the [ROS wiki](https://wiki.ros.org/universal_robots) page for compatibility information and other more information.

__License__

The [UR20 meshes](ur_description/meshes/ur20) and [UR30 meshes](ur_description/meshes/ur30) constitute “Graphical Documentation” from Universal Robots which is subject to and governed by Universal Robots' "[Terms and Conditions for use of Graphical Documentation](https://www.universal-robots.com/legal/terms-and-conditions/terms_and_conditions_for_use_of_graphical_documentation.txt)".

Universal Robots' [Terms and Conditions for use of Graphical Documentation](https://www.universal-robots.com/legal/terms-and-conditions/terms_and_conditions_for_use_of_graphical_documentation.txt) do not fully comply with [OSI's definition of Open Source](https://opensource.org/osd/), but they do allow you to use, modify and share “Graphical Documentation”, including [UR20 meshes](ur_description/meshes/ur20) and [UR30 meshes](ur_description/meshes/ur30), subject to certain restrictions.\
If you have any questions regarding this license or if this license doesn't fit your use-case, please contact [legal@universal-robots.com](mailto:legal@universal-robots.com).

All other content is licensed under the BSD-3-Clause license or Apache-2.0 respectively.



__Installation__

There are two different ways to install the packages in this repository. The following sections detail installing the packages using the binary distribution and building them from source in a Catkin workspace.


___Using apt (Ubuntu, Debian)___

On supported Linux distributions (Ubuntu, 18.04 (Bionic) and 20.04 (Focal), `i386` and `amd64`) and ROS versions:

```
sudo apt-get install ros-$ROS_DISTRO-universal-robots
```

replace `$ROS_DISTRO` with `melodic` or `noetic`, depending on which ROS version you have installed.


___Building from Source___

There *will soon be* releases available for ROS Melodic and Noetic. However, for the latest features and developments you might want to build the packages from source.

**NOTE**: please prefer using the binary release (see previous section) over building from source where possible. Source installs will not be automatically updated by new package releases and require more work to setup.

The following instructions assume that a [Catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace) has been created at `$HOME/catkin_ws` and that the source space is at `$HOME/catkin_ws/src`. Update paths appropriately if they are different on the build machine.

In all other cases the packages will have to be build from sources in a Catkin workspace:

```
cd $HOME/catkin_ws/src

# retrieve the sources (replace '$ROS_DISTRO' with the ROS version you are using)
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd $HOME/catkin_ws

# checking dependencies (again: replace '$ROS_DISTRO' with the ROS version you are using)
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```


__Usage__

___With real Hardware___

For using real hardware, please use the
[`ur_robot_driver`](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver). Installation
and startup instructions are available there separately.

If you have a particular old robot running Software CB1 or CB2, please use the
[`ur_modern_driver`](https://github.com/ros-industrial/ur_modern_driver) instead.

CAUTION:
Remember that you should always have your hands on the big red button in case there is something in the way or anything unexpected happens.

___MoveIt! with real Hardware___
Additionally, you can use MoveIt! to control the robot.
There exist MoveIt! configuration packages for all robots.

In the following the commands for the UR5 are given. For other robots, simply replace the prefix accordingly.

For setting up the MoveIt! nodes to allow motion planning run e.g.:

```roslaunch ur5_moveit_config moveit_planning_execution.launch```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch```

___Usage with Gazebo Simulation___
There are launch files available to bringup a simulated robot.

Don't forget to source the correct setup shell files and use a new terminal for each command!

To bring up the simulated robot in Gazebo, run:

```roslaunch ur_gazebo ur5_bringup.launch```


___MoveIt! with a simulated robot___
Again, you can use MoveIt! to control the simulated robot.

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch```
