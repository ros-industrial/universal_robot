^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur5_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
