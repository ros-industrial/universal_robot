^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2022-11-11)
------------------

1.3.0 (2022-11-10)
------------------
* Merge melodic-devel-staging `#617 <https://github.com/ros-industrial/universal_robot/issues/617>`_ from ros-industrial/melodic-devel-staging
* Merge pull request `#619 <https://github.com/ros-industrial/universal_robot/issues/619>`_ from fmauch/gazebo_effort_controllers
  Gazebo effort controllers
* [ur_gazebo] update effort_controllers' pid parameters
* [ur_gazebo] Switch to effort controllers
* [ur_gazebo] Switch common files to effort controllers
* Add tool0_controller frame for gazebo simulation (`#611 <https://github.com/ros-industrial/universal_robot/issues/611>`_)
  In order to make the simulation more consistent to the ur_robot_driver
  (and ur_modern_driver) this adds a tool0_controller frame.
  For the driver this corresponds to the tool transformation published by
  the robot controller directly. As this doesn't exist for the simulated
  robot, this adds the frame using an identity transform from tool0.
* Make ur_gazebo robot name consistent with ur_description (`#615 <https://github.com/ros-industrial/universal_robot/issues/615>`_)
* Increase minimum CMake version to allow builds on Focal Fossa (Noetic) (`#586 <https://github.com/ros-industrial/universal_robot/issues/586>`_)
* Merge pull request `#562 <https://github.com/ros-industrial/universal_robot/issues/562>`_ from fmauch/convenience_macros
  Add convenience macros for description files
* ur_gazebo: Update path ur ur_macro.xacro
  This was changed in e4eb54718edc6ff414d3c046f2589b798b15f9b2
* Merge pull request `#518 <https://github.com/ros-industrial/universal_robot/issues/518>`_ from gavanderhoorn/gazebo_updates
  Main improvements:
  - follow ROS-I naming and package layout conventions
  - clean up xacros (naming, whitespace, etc)
  - treat (the virtual robot in) Gazebo as much as possible as just another UR variant (so we get to reuse workflow, settings and `.launch` files as much as possible)
  - improve comments in `.xacro` files
  - clean up "public API" of `ur_gazebo` (ie: `.launch` and `_macro.xacro` files )
* gazebo: values less than 0.1 for spawn_z will be problematic.
  So explain this in the arg's documentation.
* gazebo: in-order is the default on Melodic.
  And we've broken Kinetic-compatibility quite a few commits ago.
* gazebo: list valid values for 'transmission_hw_interface'.
  For some reason this was not yet documented.
* gazebo: clarify we load a position joint trajectory controller.
  So include that in the name.
* gazebo: move include to scope of ur_robot_gazebo macro.
  Prevent potential clashes with symbols outside of that scope.
* gazebo: update comments in xacros.
* gazebo: run e-Series controllers at 500 Hz.
  Same rate as the real controller.
* gazebo: give e-Series variants their own controller configs.
* gazebo: restructure bringup entry points.
  By lifting some of the nodes and parameters to the bringup files (from ur_common.launch.xml), they should be easier to use (and maintain).
  Anything specific to the use of a robot in a particular configuration (ie: a workcell or simulation) should exist at the bringup level, not in a common (ie: shared) file, as that will make it harder to change (as changes will be shared by many other setups).
  This restructuring makes ur_common.launch.xml redundant, so we also remove it.
* gazebo: ur_control: allow more control over launch args.
  Over:
  - which world to load
  - whether to start Gazebo or not
  - from which parameter to (attempt to) load the urdf
  - what name to give to the spawned robot model (inside the simulation)
* gazebo: rostopic is no longer used.
* gazebo: update tests to load non-entry-point loaders.
* gazebo: include load launch file with relative include.
* gazebo: hide loaders as well.
* gazebo: update paths to included launch files.
  Use 'dirname' substitution arg to express relative include.
* gazebo: hide non-entry-point launch files.
  These launch files are not meant to be started directly, so 'hide' them roslaunch's auto-complete.
* gazebo: there is no 'vel_based_pos_traj_controller', so remove.
* gazebo: fix comments in bringup launch files.
* gazebo: load URDF in bringup launch files.
  The main 'driver' launch file should not load the URDF, as it's not the task of the driver.
  This also removes one level of argument-forwarding (namely: the arguments for the xacro macro).
  Finally, it provides a nicer template for users to base their own launch files on, as they 'only' have to deal with these top-level bringup launch files.
* gazebo: test variants separately and also test bringup.
* gazebo: fix install rule.
* gazebo: set controller pub rates to be like real hw.
  As we don't have separate controller config files for e-Series robots, they will still 'only' publish at 125 Hz.
  We might change this in the future.
* gazebo: use new controller config files.
* gazebo: rename controller config files.
  Align with names in ur_robot_driver.
* gazebo: clarify 'world' frame utility in top-level xacro.
* gazebo: refactor gazebo and gazebo_ros_control integration.
  This changes all bringup launch files in this package to be like those of a regular driver. In essence, Gazebo (and the UR(s) simulated by it) are treated as just-another-robot, with the associated launch file infrastructure.
  As ur_robot_driver is currently the driver most users will be exposed to and have experience with, the launch file structure of that driver has been used as a template. But instead of starting the actual ur_robot_driver node, the 'lowest' launch file (ie: 'ur_control.launch') starts Gazebo and loads and spawns the controllers. After this launch file has completed its work, users should see a ROS API (ie: topics, services and actions) similar to what they'd see when using the a real UR with ur_robot_driver.
  Main differences would be (at this point, this may change in the future):
  - no force-torque sensor output (ie: no 'wrench' topic)
  - no URScript topic (as the controller is not simulated by Gazebo)
  - no IO topics nor services
  - no MainBoard or similar topics
  - no Dashboard services
  - no 'tool communication' topics nor services
  - no 'tool0_controller' TF frame
  Parameter files used with the xacro macro for the real robot (ie: joint limits, kinematics, visual and physical parameters) can be used with the Gazebo simulation as well.
* gazebo: cleanup ros_control config yamls.
  Use the same set of controllers for all robots, use the same names for controllers as used by `ur_robot_driver` and include the JointStateController in the same file.
  NOTE: this is all position control only (and thus open-loop or 'forward command' control (using ros_control vernacular)).
* gazebo: don't default to UR3 parameter files.
  Top-level xacro should not default to any robot.
* gazebo: use 'bringup' launch file name.
  These files will serve the same purpose as those provided by a/the driver (but instead of a driver, they will launch Gazebo). Give them the same name to make them recognisable.
* gazebo: introduce 'load\_*.launch' helpers.
  These mimic the files with the same names as in ur_description, but load the Gazebo model instead of the real robot onto the parameter server.
* gazebo: controller_utils isn't re-used anywhere else.
  So merge contents into 'ur_common.launch' and remove the file.
* gazebo: nothing to calibrate.
  This is most likely a relic from the PR2 simulation.
* gazebo: 2-spaces per indent level.
* gazebo: formatting and comments of xacros.
* gazebo: use new robot and macro name.
  Align with filenames and other ROS-Industrial Gazebo support packages.
* gazebo: merge relevant content from common.gazebo.xacro into macro.
  Only the ros_control elements are retained.
  The UR doesn't have a battery, so no need to include that plugin.
* gazebo: use new filenames.
* gazebo: follow description xacro macro naming.
  Rename file to reflect name of macro or top-level entity.
  Include 'gazebo' reference as these files host content specific to Gazebo.
* Merge pull request `#520 <https://github.com/ros-industrial/universal_robot/issues/520>`_ from gavanderhoorn/fix_yaml_loading
  Load yaml files in read_model_data(..), nowhere else
* gazebo: pass parameter filenames, not their content.
  Aligns with similar changes in ur_description.
* Merge pull request `#516 <https://github.com/ros-industrial/universal_robot/issues/516>`_ from gavanderhoorn/convert_limits_files
  Use 'ros_control style' joint limit files
* gazebo: use new arg names for joint limits.
  Renamed in ur_description, so use the new names here as well and pass the right values.
* gazebo: update joint limit filename references.
  They were renamed (in c0f71ebb), so use the new names.
* Merge pull request `#497 <https://github.com/ros-industrial/universal_robot/issues/497>`_ from gavanderhoorn/desc_updates
  Misc updates to ur_description
* gazebo: use new xacro macro file.
* Merge pull request `#477 <https://github.com/ros-industrial/universal_robot/issues/477>`_ from fmauch/ur16e
  Add Ur16e support
* Updated ur16 files for calibrated URDF
* Preliminary model version of UR16 added
  This model is only correct in the kinematics structure. Meshes and dynamics
  parameters have to be corrected.
* Use full kinematics parameters in description (`#495 <https://github.com/ros-industrial/universal_robot/issues/495>`_)
  the kinematics parameters can be retrieved from a calibration mechanism
  to precisely represent the robot's kinematics.
* Merge pull request `#371 <https://github.com/ros-industrial/universal_robot/issues/371>`_ from ipa-led/ur_description_args
  Urdf with args and yaml configuration
* change gazebo launch file and tests
  * added e-series
* create urdf files for ur_gazebo
* migrated all package.xml files to format=2 (`#439 <https://github.com/ros-industrial/universal_robot/issues/439>`_)
* Load the JointGroupPositionController so jog commands can be sent (`#422 <https://github.com/ros-industrial/universal_robot/issues/422>`_)
  * Load the JointGroupPositionController so jog commands can be sent
  * Load new controllers for UR5/UR10, too
  * Add other controllers in launch file
  * Add JointGroupPositionController to UR e-series
* Merge branch 'kinetic-devel' into patch-1
* Contributors: AndyZe, Felix Exner, Felix Exner (fexner), Felix Mauch, G.A. vd. Hoorn, Lucchi, Matteo, Ludovic Delval, Nadia Hammoudeh García, Qiang Qiu, RobertWilbrandt, gavanderhoorn, kut

1.2.5 (2019-04-05)
------------------
* Update maintainer listing: add Miguel (`#410 <https://github.com/ros-industrial/universal_robot/issues/410>`_)
* UR-E Series (`#380 <https://github.com/ros-industrial/universal_robot/issues/380>`_)
* Update maintainer and author information.
* Add roslaunch tests (`#362 <https://github.com/ros-industrial/universal_robot/issues/362>`_)
* Using the 'doc' attribute on 'arg' elements.
* Contributors: Dave Niewinski, gavanderhoorn, Harsh Deshpande, Nadia Hammoudeh García

1.2.1 (2018-01-06)
------------------

1.2.0 (2017-08-04)
------------------
* Remove dependency on ros_controllers metapackage.
  As per http://www.ros.org/reps/rep-0127.html, packages are not allowed to
  depend on metapackages.
* Contributors: Miguel Prada

1.1.9 (2017-01-02)
------------------
* No changes.

1.1.8 (2016-12-30)
------------------
* ur_gazebo: escape underscore in changelog (`#279 <https://github.com/ros-industrial/universal_robot/issues/279>`_).
* all: update maintainers.
* Contributors: gavanderhoorn

1.1.7 (2016-12-29)
------------------
* ur_gazebo: add controller_manager as run dependency.
* Contributors: Hans-Joachim Krauch

1.1.6 (2016-04-01)
------------------
* provide launch files for ur3
* use controller_manager spawn
* allow to start gazebo without gui
* adjust controllers to new hardwareInterface - affects simulation only
* Contributors: ipa-fxm

1.0.2 (2014-03-31)
------------------

1.0.1 (2014-03-31)
------------------
* adapt launch files in order to be able to use normal/limited xacro
* updates for latest gazebo under hydro
* Contributors: ipa-fxm

* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Updated to catkin.  ur_driver's files were added to nested Python directory for including in other packages.
* removed ``arm_`` prefix from joint names in gazebo controller config
* Renamed packages and new groovy version
* Added ur10 and renamed packages
* Contributors: IPR-SR2, Kelsey, Mathias Lüdtke, ipa-nhg, robot
