^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2024-04-08)
------------------
* Fix default calibration file for UR30 (`#677 <https://github.com/ros-industrial/universal_robot/issues/677>`_)
* UR30 description and meshes (`#674 <https://github.com/ros-industrial/universal_robot/issues/674>`_)
* Contributors: Felix Exner, Vincenzo Di Pentima

1.3.2 (2023-12-18)
------------------
* UR20 description and meshes (`#657 <https://github.com/ros-industrial/universal_robot/issues/657>`_)
* Contributors: Rune Søe-Knudsen

1.3.1 (2022-11-11)
------------------

1.3.0 (2022-11-10)
------------------
* Merge melodic-devel-staging `#617 <https://github.com/ros-industrial/universal_robot/issues/617>`_ from ros-industrial/melodic-devel-staging
* ur_description: Make robot name configurable on load_ur.launch (`#612 <https://github.com/ros-industrial/universal_robot/issues/612>`_)
  In 9d02ccadc8fb9b5cfeaf0d5b0c0c7708d2461447 robot launchfiles were forwarded
  to their specialized launchfiles instead of setting the robot model in the
  load_ur.launch file.
* Fixed ur5 base height (`#607 <https://github.com/ros-industrial/universal_robot/issues/607>`_)
  The visual and collision mesh of the base of the ur5 has to be moved 3 mm upwards in order to close the gap between the shoulder and align its bottom in the xy-plane. Otherwise the ur5 collides with the surface it is directly placed on.
* Merge pull request `#602 <https://github.com/ros-industrial/universal_robot/issues/602>`_ from fmauch/fix_deprecation_warning
  fixing deprecation warning
* fixing deprecation warning
* ur_description: Use the per-model xacro files for correct robot name (`#588 <https://github.com/ros-industrial/universal_robot/issues/588>`_)
  With the common xacro file the robot name will always be "ur".
  However, the moveit config packages expect the robot name to be e.g. "ur10e".
  This commit will use the macros containing the model name already.
* Increase minimum CMake version to allow builds on Focal Fossa (Noetic) (`#586 <https://github.com/ros-industrial/universal_robot/issues/586>`_)
* Merge pull request `#562 <https://github.com/ros-industrial/universal_robot/issues/562>`_ from fmauch/convenience_macros
  Add convenience macros for description files
* Replace _\_ prefixes (`#579 <https://github.com/ros-industrial/universal_robot/issues/579>`_)
  With newer versions of xacro this leads to errors otherwise.
* description: provide convenience top-levels and macros.
  These avoid users having to provide values for *all* arguments of the 'ur_robot' macro.
  In many cases, users will want to keep the default files for the joint limits, physical and visual parameters, and only override the default kinematics (to use extracted calibration fi).
  With the provided wrapper macros, this is possible, as variant-specific defaults are provided for all arguments, requiring only to override the required ones.
  Users looking to include a UR into a larger scene or composite xacro macro should 'xacro:include' these '_macro.xacro' files.
  The top-levels are only useful when loading a stand-alone UR in an otherwise empty scene. They do not allow access to any arguments and only use the defaults.
* description: main macro xacro is not a top-level.
  So move to include directory.
* Merge pull request `#533 <https://github.com/ros-industrial/universal_robot/issues/533>`_ from gavanderhoorn/yaml_ctors_rosparam_joint_limits
  description: spec limits in degrees.
* description: spec limits in degrees.
  Use rosparam's and xacro's support for these special constructors to convert to radians on-the-fly.
* Merge pull request `#520 <https://github.com/ros-industrial/universal_robot/issues/520>`_ from gavanderhoorn/fix_yaml_loading
  Load yaml files in read_model_data(..), nowhere else
* description: delegate loading yaml files to read_model_data(..).
  This avoids having to 'remember' to load_yaml(..) the contents of those files in every user of the 'ur_robot' macro, and seems to conform to the idea behind the '\*_parameters_file' naming of the parameters (they're paths to files, not yaml strings).
* Merge pull request `#516 <https://github.com/ros-industrial/universal_robot/issues/516>`_ from gavanderhoorn/convert_limits_files
  Use 'ros_control style' joint limit files
* description: load values from new params.
  'ros_control format' uses different names, so use them.
* description: they're joint limits.
  Not joints limits.
* description: load new joint limit files.
* description: convert joint limit files to 'ros_control format'.
  This allows re-using them with MoveIt and ros_control hw interfaces.
* description: rename limit files.
  Bring in-line with other joint limit files.
* description: add missing safety_ctrlr for shoulder pan. (`#514 <https://github.com/ros-industrial/universal_robot/issues/514>`_)
* description: correct joint limits for all supported variants. (`#513 <https://github.com/ros-industrial/universal_robot/issues/513>`_)
  Based on values from the User Manuals.
* description: fix main macro docs. (`#512 <https://github.com/ros-industrial/universal_robot/issues/512>`_)
* Merge pull request `#505 <https://github.com/ros-industrial/universal_robot/issues/505>`_ from gavanderhoorn/fix_base_link_orientation
  Remove incorrect base_link rotation.
* description: correct orientation of Base mesh.
  Connector should be aligned with Y+ of 'base' frame.
* description: fix orientation of 'base' frame.
  So it's coincident with the robot's/controller's 'Base' frame. It's actually 180 degrees rotated over Z wrt 'base_link'.
* description: rotate for UR internal frames.
* description: introduce base_link_inertia link.
  To work around the root-link-with-inertia limitation in KDL.
* description: remove the 'base_correction' property.
  There is an offset between 'base_link' and 'base', but it's a fixed one, known and does not need to be a property.
  Rectify the problem with the 'base_link' mesh in a follow-up commit.
* description: assume -0==0. (`#511 <https://github.com/ros-industrial/universal_robot/issues/511>`_)
* description: use mat def from yaml for base_link. (`#509 <https://github.com/ros-industrial/universal_robot/issues/509>`_)
* description: reduce verbosity: pass all args. (`#508 <https://github.com/ros-industrial/universal_robot/issues/508>`_)
  These convenience wrappers only setup defaults, and all args are needed, so pass all of them.
* Merge pull request `#506 <https://github.com/ros-industrial/universal_robot/issues/506>`_ from gavanderhoorn/add_flange_frame
  Add flange frame
* description: use standard comments for base and tool0 frames.
  Aligned with other ROS-Industrial robot support packages.
* description: fix orientation of tool0.
  Now that flange is its parent.
* description: introduce flange frame.
  This is the correct EEF attachment point: REP-103 aligned and no unwanted rotations (as tool0).
* description: remove ee_link link.
  Also remove the tiny collision object, as it's no longer needed now that MoveIt has been fixed.
* description: clarify status of 'inc' xacro files. (`#507 <https://github.com/ros-industrial/universal_robot/issues/507>`_)
  As the way we model the robots may change in the future, users should consider these files as private.
  No guarantees are given as to the existence of these files or their contents.
* Merge pull request `#497 <https://github.com/ros-industrial/universal_robot/issues/497>`_ from gavanderhoorn/desc_updates
  Misc updates to ur_description
* description: clarify note on base_correction property.
* description: typo.
* description: explain function of common load_ur launch file.
* description: recognise Mathias Lüdtke as contributor.
* Update ur_description/urdf/ur_macro.xacro
  Co-authored-by: Felix Exner <felix_mauch@web.de>
* description: format top-level xacro.
* description: build script cleanup.
* description: minor manifest cleanup.
* description: add roslaunch check tests.
* description: use new load launch files everywhere.
* description: rename load launch files.
  Align them with other ROS-Industrial support packages.
* description: explain why we limit elbow joints to +- 1 pi.
* description: ur16e: limit elbow to +- 1 pi.
  Align with other models. For https://github.com/ros-industrial/universal_robot/issues/265.
* description: ur16e: state_publisher is deprecated.
  Use new name.
* description: use JSP GUI everywhere.
  avoid deprecation warnings.
* description: remove whitespace.
* description: use new xacro macro filenames.
* description: dots to underscores.
* description: fix xacro filenames.
  Align them with other ROS-I support packages.
* description: formatting and layout of xacro macro.
* description: align link and joint order with other ROS-I pkgs.
  Links first, then joints.
* Merge pull request `#477 <https://github.com/ros-industrial/universal_robot/issues/477>`_ from fmauch/ur16e
  Add Ur16e support
* Updated ur16 files for calibrated URDF
* Updated kinetmatic and physical parameters according to the current PR
* Added missing meshes for ur16e
* Preliminary model version of UR16 added
  This model is only correct in the kinematics structure. Meshes and dynamics
  parameters have to be corrected.
* Use full kinematics parameters in description (`#495 <https://github.com/ros-industrial/universal_robot/issues/495>`_)
  the kinematics parameters can be retrieved from a calibration mechanism
  to precisely represent the robot's kinematics.
* Merge pull request `#371 <https://github.com/ros-industrial/universal_robot/issues/371>`_ from ipa-led/ur_description_args
  Urdf with args and yaml configuration
* used robot_state_publisher instead of state_publisher
* removed --inorder for common launch
  * default in melodic
* update ur_description launch files
  * use of yaml files parameters
  * added e_series
  * create a common launch file to avoir duplicated
* made common macro for ur_robot urdf
  * removed each model specific xacro
  * use of yaml files
  * pass yaml files as parameters
  * common ur_robo macro
  * remved ur_gazebo specific parts
* create parameters yaml files
  * one for each models
* deleted ur_e specific repositories
  * moved ur_e_description meshes files to ur_description
* Merge pull request `#437 <https://github.com/ros-industrial/universal_robot/issues/437>`_ from ipa-nhg/safetylimits
  Add optional safety_controller tags to all joints in xacro macros
* migrated all package.xml files to format=2 (`#439 <https://github.com/ros-industrial/universal_robot/issues/439>`_)
* Merge pull request `#426 <https://github.com/ros-industrial/universal_robot/issues/426>`_ from fmauch/inertia
  corrected dimensions and positions of inertias
* Add optional safety_controller tags to all joints in xacro macros
* Merge pull request `#435 <https://github.com/ros-industrial/universal_robot/issues/435>`_ from fmauch/add_description_view_files
  Add description view files
* Add dependencies for view_x.launch files to the description packages
  As we use the joint_state_publisher, the robot_state_publisher and rviz
  inside the launch files, I added them as run-dependencies.
* Added view_x.launch files for all descriptions to easily check them.
  This resolves `#432 <https://github.com/ros-industrial/universal_robot/issues/432>`_
  To avoid introducing another dependency, I copied the rviz configuration
  from industrial_robot_client.
* corrected dimensions and positions of inertias
  I'm by far not an expert in working with gazebo or inertias, but it seemed wrong to me:
  - The upper arm inertia of the ur10 is not centered in the visual arm segment
  - CoM in the wrist links don't sit inside the correct links. E.g. wrist1 has its CoM inside the end of the forearm for all robots.
  - Because of the second point the inertia's geometry of wrist3 is matching wrist2 instead of the actual moving part of wrist3.
  - Wrist dimensions of ur5 were completely off.
  - On the ur5e the arm inertias weren't centered in the visuals.
* Merge branch 'kinetic-devel' into patch-1
* Contributors: BobbyCephy, Felix Exner, Felix Exner (fexner), Felix Mauch, G.A. vd. Hoorn, JeremyZoss, Ludovic Delval, Miguel Prada, Nadia Hammoudeh García, Qiang Qiu, RobertWilbrandt, gavanderhoorn, georgiablanco

1.2.5 (2019-04-05)
------------------
* Add transmission_hw_interface to UR xacro and expose everywhere (`#392 <https://github.com/ros-industrial/universal_robot/issues/392>`_)
* Update maintainer listing: add Miguel (`#410 <https://github.com/ros-industrial/universal_robot/issues/410>`_)
* Updated xacro namespace.
* Update maintainer and author information.
* Updated mesh ambience so the model isn't so dark in Gazebo
* Fix overlapping variable names between robot definition files (`#356 <https://github.com/ros-industrial/universal_robot/issues/356>`_)
* Improve meshes shading (`#233 <https://github.com/ros-industrial/universal_robot/issues/233>`_)
* Added run_depend for xacro
* Using the 'doc' attribute on 'arg' elements.
* Enable self collision in gazebo
* Contributors: Dave Niewinski, Felix von Drigalski, Harsh Deshpande, Joe, Marcel Schnirring, Miguel Prada, MonteroJJ, ipa-fxm

1.2.1 (2018-01-06)
------------------
* Merge pull request `#329 <https://github.com//ros-industrial/universal_robot/issues/329>`_ from tecnalia-medical-robotics/joint_limits
  Homogenize xacro macro arguments.
* Merge pull request `#332 <https://github.com//ros-industrial/universal_robot/issues/332>`_ from davetcoleman/kinetic_hw_iface_warning
  Remove UR3 ROS Control Hardware Interface warning
* Remove UR3 ROS Control Hardware Interface warning
* Extend changes to '_robot.urdf.xacro' variants as well.
* Homogenize xacro macro arguments.
  Joint limits for the limited version could be set using arguments for the UR10
  but not for the UR3 and UR5. Same lower and upper limit arguments are added to
  the UR3 and UR5 xacro macros.
* Fix elbow joint limits (`#268 <https://github.com//ros-industrial/universal_robot/issues/268>`_)
* Remove warning 'redefining global property: pi' (Jade+) (`#315 <https://github.com//ros-industrial/universal_robot/issues/315>`_)
* Contributors: Beatriz Leon, Dave Coleman, Felix Messmer, Miguel Prada

1.2.0 (2017-08-04)
------------------

1.1.9 (2017-01-02)
------------------
* reintroduce 'pi', unbrake dependent xacros.
* use '--inorder' to trigger use of jade+ xacro on Indigo.
* Contributors: gavanderhoorn

1.1.8 (2016-12-30)
------------------
* all: update maintainers.
* Contributors: gavanderhoorn

1.1.7 (2016-12-29)
------------------
* Fix xacro warnings in Jade (`#251 <https://github.com/ros-industrial/universal_robot/issues/251>`_)
* added default values to xacro macro
* tested joint limits modification
* Contributors: Dave Coleman, G.A. vd. Hoorn, philip 14.04

1.1.6 (2016-04-01)
------------------
* unify mesh names
* add color to avoid default color 'red' for collision meshes
* use correct DH parameter + colored meshes
* introducing urdf for ur3 - first draft
* unify common xacro files
* remove obsolete urdf files
* description: add '_joint' suffix to newly introduced joint tags.
  This is more in-line with naming of existing joint tags.
* description: add ROS-I base and tool0 frames. Fix `#49 <https://github.com/ros-industrial/universal_robot/issues/49>`_ and `#95 <https://github.com/ros-industrial/universal_robot/issues/95>`_.
  Note that 'base' is essentially 'base_link' but rotated by 180
  degrees over the Z-axis. This is necessary as the visual and
  collision geometries appear to also have their origins rotated
  180 degrees wrt the real robot.
  'tool0' is similar to 'ee_link', but with its orientation such
  that it coincides with an all-zeros TCP setting on the UR
  controller. Users are expected to attach their own TCP frames
  to this frame, instead of updating it (see also [1]).
  [1] http://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages#Standardised_links\_.2BAC8_frames
* description: minor whitespace cleanup of UR5 & 10 xacros.
* regenerate urdf files
* use PositionJointInterface as hardwareInterface in transmissions - affects simulation only
* Contributors: gavanderhoorn, ipa-fxm

1.0.2 (2014-03-31)
------------------

1.0.1 (2014-03-31)
------------------
* changes due to file renaming
* generate urdfs from latest xacros
* file renaming
* adapt launch files in order to be able to use normal/limited xacro
* fixed typo in limits
* add joint_limited urdf.xacros for both robots
* (re-)add ee_link for both robots
* updates for latest gazebo under hydro
* remove ee_link - as in ur10
* use same xacro params as ur10
* use new transmission interfaces
* update xml namespaces for hydro
* remove obsolete urdf file
* remove obsolete urdf file
* Contributors: ipa-fxm

* Update ur10.urdf.xacro
  Corrected UR10's urdf to faithfully represent joint effort thresholds, velocity limits, and dynamics parameters.
* Update ur5.urdf.xacro
  Corrected effort thresholds and friction values for UR5 urdf.
* added corrected mesh file
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Corrected warning on xacro-files in hydro.
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Updated to catkin.  ur_driver's files were added to nested Python directory for including in other packages.
* fixed name of ur5 transmissions
* patched gazebo.urdf.xacro to be compatible with gazebo 1.5
* fixed copy&paste error (?)
* prefix versions of gazebo and transmission macros
* Added joint limited urdf and associated moveit package.  The joint limited package is friendlier to the default KLD IK solution
* Added ur5 moveit library.  The Kinematics used by the ur5 move it library is unreliable and should be replaced with the ur_kinematics
* Updated urdf files use collision/visual models.
* Reorganized meshes to include both collision and visual messhes (like other ROS-I robots).  Modified urdf xacro to include new models.  Removed extra robot pedestal link from urdf (urdfs should only include the robot itself).
* minor changes on ur5 xacro files
* Removed extra stl files and fixed indentions
* Renamed packages and new groovy version
* Added ur10 and renamed packages
* Contributors: Denis Štogl, IPR-SR2, Kelsey, Mathias Lüdtke, Shaun Edwards, ipa-nhg, jrgnicho, kphawkins, robot
