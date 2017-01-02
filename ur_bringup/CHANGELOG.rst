^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.9 (2017-01-02)
------------------
* No changes.

1.1.8 (2016-12-30)
------------------
* all: update maintainers.
* Contributors: gavanderhoorn

1.1.7 (2016-12-29)
------------------
* Add prefix parameter in common launch
* Contributors: Yannick Jonetzko

1.1.6 (2016-04-01)
------------------
* provide launch files for ur3
* Contributors: ipa-fxm

1.0.2 (2014-03-31)
------------------

1.0.1 (2014-03-31)
------------------
* adapt launch files in order to be able to use normal/limited xacro
* Contributors: ipa-fxm

* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Updated to catkin.  ur_driver's files were added to nested Python directory for including in other packages.
* Updated urdf files use collision/visual models.
* Added launch directory to bringup package.  Left the old launch files to be backwards compatable for now.  Updated driver with SwRI internal changes.  Changed driver to accept ip address to match ROS-Industrial standard.  IP addresses for industrial robots are typically fixed.  Hostnames are not typically used (this is what the driver expected)
* Removed extra stl files and fixed indentions
* Renamed packages and new groovy version
* Added ur10 and renamed packages
* Contributors: IPR-SR2, Kelsey, Shaun Edwards, ipa-nhg, robot
