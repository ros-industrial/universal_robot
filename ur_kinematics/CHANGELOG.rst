^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2022-11-11)
------------------

1.3.0 (2022-11-10)
------------------
* Merge melodic-devel-staging `#617 <https://github.com/ros-industrial/universal_robot/issues/617>`_ from ros-industrial/melodic-devel-staging
* Update ur kinematics (`#616 <https://github.com/ros-industrial/universal_robot/issues/616>`_)
  * Add ur_kinematics parameter sets for all e-Series robots
  * Add a README with a notice to ur_kinematics
  * Set correct IKFast plugin name in kinematics.yaml
  * Add ignore files for releasing ur_kinematics
  Since the state of this package questionable at the current point, let's
  disable releasing it for now.
* Increase minimum CMake version to allow builds on Focal Fossa (Noetic) (`#586 <https://github.com/ros-industrial/universal_robot/issues/586>`_)
* Merge pull request `#501 <https://github.com/ros-industrial/universal_robot/issues/501>`_ from gavanderhoorn/port_kinetic-devel_missing
  Cherry-pick missing commits from kinetic-devel into melodic-devel
* Use print() function in both Python 2 and Python 3
  Legacy __print_\_ statements are syntax errors in Python 3 but __print()_\_ function works as expected in both Python 2 and Python 3.
* Merge pull request `#372 <https://github.com/ros-industrial/universal_robot/issues/372>`_ from qqfly/patch-1
  simplify FK equations
* migrated all package.xml files to format=2 (`#439 <https://github.com/ros-industrial/universal_robot/issues/439>`_)
* Merge branch 'kinetic-devel' into patch-1
* simplify FK equations
* Contributors: Felix Exner (fexner), Felix Mauch, G.A. vd. Hoorn, Levi Armstrong, Nadia Hammoudeh García, Qiang Qiu, cclauss

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
