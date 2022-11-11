^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur3_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2022-11-11)
------------------

1.3.0 (2022-11-10)
------------------
* Merge melodic-devel-staging `#617 <https://github.com/ros-industrial/universal_robot/issues/617>`_ from ros-industrial/melodic-devel-staging
* Use effort controllers for Gazebo `#619 <https://github.com/ros-industrial/universal_robot/issues/619>`_ from fmauch/gazebo_effort_controllers
* [moveit_configs] Change default sim controller to effort controller
* Update ur kinematics (`#616 <https://github.com/ros-industrial/universal_robot/issues/616>`_)
  * Add ur_kinematics parameter sets for all e-Series robots
  * Add a README with a notice to ur_kinematics
  * Set correct IKFast plugin name in kinematics.yaml
  * Add ignore files for releasing ur_kinematics
* Update MoveIt! support (`#538 <https://github.com/ros-industrial/universal_robot/issues/538>`_)
  Update MoveIt! configurations to new description structure.
  Co-authored-by: Luke Dennis <luke.j.dennis@gmail.com>
  Co-authored-by: gavanderhoorn <g.a.vanderhoorn@tudelft.nl>
  Co-authored-by: RobertWilbrandt <wilbrandt@fzi.de>
* Increase minimum CMake version to allow builds on Focal Fossa (Noetic) (`#586 <https://github.com/ros-industrial/universal_robot/issues/586>`_)
* migrated all package.xml files to format=2 (`#439 <https://github.com/ros-industrial/universal_robot/issues/439>`_)
* Merge branch 'kinetic-devel' into patch-1
* Contributors: Felix Exner, Felix Exner (fexner), Felix Mauch, Nadia Hammoudeh García, Qiang Qiu, RobertWilbrandt

1.2.5 (2019-04-05)
------------------
* Update maintainer listing: add Miguel (`#410 <https://github.com/ros-industrial/universal_robot/issues/410>`_)
* MoveGroupExecuteService is Deprecated by MoveIt! (`#391 <https://github.com/ros-industrial/universal_robot/issues/391>`_)
* Update maintainer and author information.
* Add roslaunch tests (`#362 <https://github.com/ros-industrial/universal_robot/issues/362>`_)
* Contributors: gavanderhoorn, Nadia Hammoudeh García, 薯片半价

1.2.1 (2018-01-06)
------------------
* Reduce longest valid segment fraction to accomodate non-limited version of the UR5 (`#266 <https://github.com//ros-industrial/universal_robot/issues/266>`_)
* Contributors: Scott Paulin

1.2.0 (2017-08-04)
------------------
* Fix Deprecated warning in MoveIt: parameter moved into namespace 'trajectory_execution'
* Contributors: Dave Coleman

1.1.9 (2017-01-02)
------------------
* use '--inorder' for jade+ xacro as well.
* make RViz load MoveIt display by default.
* Contributors: gavanderhoorn

1.1.8 (2016-12-30)
------------------
* all: update maintainers.
* Contributors: gavanderhoorn

1.1.7 (2016-12-29)
------------------
* Don't depend on moveit_plugins metapackage
* Fix xacro warnings in Jade
* Contributors: Dave Coleman, Jon Binney

1.1.6 (2016-04-01)
------------------
* add missing dependency for moveit_simple_controller_manager
* apply default RRTConnect to ur3
* add moveit_config for ur3
* Contributors: ipa-fxm
