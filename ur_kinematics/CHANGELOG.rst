^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.5 (2019-04-05)
------------------
* Update maintainer listing: add Miguel (`#410 <https://github.com/ros-industrial/universal_robot/issues/410>`_)
* Removed non-existent moveit KDL libraries from ur_kinematics includes
* Update maintainer and author information.
* Missed python module definition and setup.py script (`#364 <https://github.com/ros-industrial/universal_robot/issues/364>`_)
* Use the new proposed API to query params from correct namespaces (`#334 <https://github.com/ros-industrial/universal_robot/issues/334>`_)
* Setting default ik_weights to 1.0 (`#346 <https://github.com/ros-industrial/universal_robot/issues/346>`_)
* ur_moveit_plugin: fix compile error with GCC6
* Contributors: Alexander Rössler, gavanderhoorn, Henning Kayser, Mike Lautman, Nadia Hammoudeh García, jwhendy

1.2.1 (2018-01-06)
------------------
* Merge pull request `#303 <https://github.com//ros-industrial/universal_robot/issues/303>`_ from marcoesposito1988/pr-urdf-pointer-type
  Updated urdf::ModelInterface pointer type for ROS Lunar
* Updated urdf::ModelInterface pointer type for ROS Lunar
* Contributors: G.A. vd. Hoorn, Marco Esposito

1.2.0 (2017-08-04)
------------------
* Fixup for MoveIt! Kinetic
* Contributors: Dave Coleman

1.1.9 (2017-01-02)
------------------
* No changes.

1.1.8 (2016-12-30)
------------------
* all: update maintainers.
* Contributors: gavanderhoorn

1.1.7 (2016-12-29)
------------------
* Depend on new moveit_kinematics package (`#274 <https://github.com/ros-industrial/universal_robot/issues/274>`_).
* Contributors: Isaac I.Y. Saito

1.1.6 (2016-04-01)
------------------
* apply ur-kin-constants fix for ur3
* Merge remote-tracking branch 'origin-rosi/indigo-devel' into ur-kin-constants
* ur_kinematics: Move #defines to constants in source file.
* ur_kinematics for ur3
* crop ik solutions wrt joint_limits
* Contributors: Maarten de Vries, ipa-fxm

1.0.2 (2014-03-31)
------------------

1.0.1 (2014-03-31)
------------------

* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Updated to catkin.  ur_driver's files were added to nested Python directory for including in other packages.
* added IKfast compatibility functions
* Ported ur_kinematics package from Georgia Tech library.  Added ability to create ur5 & ur10 kinematics libraries.  Python libaries not untested.  Kinematics still needs to be wrapped within Kinematics plugin interface
* Contributors: IPR-SR2, Kelsey, Mathias Lüdtke, Shaun Edwards
