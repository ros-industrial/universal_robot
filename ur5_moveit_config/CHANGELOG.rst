^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur5_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* apply latest setup assistant changes to ur5 and ur10
* Adding comment explaining the choice of default planning algorithm
* Use RRTConnect by default for UR5
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
* Merge branch 'hydro-devel' of github.com:ros-industrial/universal_robot into hydro
* changes due to file renaming
* update moveit_configs: include ee_link and handle limited robot
* new moveit_configs for ur5 and ur10
* remove old ur5_moveit_config
* Contributors: Florian Weisshardt, ipa-fxm

* ur5_moveit_cfg: add missing run_depend ind_rob_simulator. Fix `#38 <https://github.com/ros-industrial/universal_robot/issues/38>`_.
* update moveit_configs to use moveit_simple_controller_manager
* Added config files missed on last commit
* Added launch/configuration files for using real robot.  Updated joint limits to velocity limits of the driver (all of which can be configured to make the robot move faster)
* Removed ur5_joint_limited_moveit_config.  ur5_moveit_config now has limited joint ranges to plus/minus 180 degrees.
* Added ur5 moveit library.  The Kinematics used by the ur5 move it library is unreliable and should be replaced with the ur_kinematics
* Contributors: Jeremy Zoss, Shaun Edwards, gavanderhoorn
