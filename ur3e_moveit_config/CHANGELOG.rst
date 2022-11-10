^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur3e_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'indigo-devel' of github.com:ros-industrial/universal_robot into ur3_moveit_config
* apply latest setup assistant changes to ur5 and ur3e
* Adding comment explaining the choice of default planning algorithm
* Use RRTConnect by default for UR10
  Fixes bug `#193 <https://github.com/ros-industrial/universal_robot/issues/193>`_ about slow planning on Indigo
  LBKPIECE1 (the previous default) looks to be the wrong planning algorithm for the robot
  See https://groups.google.com/forum/#!topic/moveit-users/M71T-GaUNgg
* crop ik solutions wrt joint_limits
* set planning time to 0
* reduce planning attempts in moveit rviz plugin
* Contributors: Marco Esposito, ipa-fxm

1.0.2 (2014-03-31)
------------------

1.0.1 (2014-03-31)
------------------
* changes due to file renaming
* update moveit_configs: include ee_link and handle limited robot
* new moveit_configs for ur5 and ur3e
* Contributors: ipa-fxm
