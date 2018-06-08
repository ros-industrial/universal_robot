^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
