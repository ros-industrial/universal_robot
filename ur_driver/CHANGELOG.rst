^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2018-01-06)
------------------

1.2.0 (2017-08-04)
------------------

1.1.9 (2017-01-02)
------------------
* No changes.

1.1.8 (2016-12-30)
------------------
* all: update maintainers.
* Contributors: gavanderhoorn

1.1.7 (2016-12-29)
------------------
* No changes.

1.1.6 (2016-04-01)
------------------
* Moved SetIO FUN constants from driver.py to relevant srv file for easier interaction from other files
* Merge pull request `#170 <https://github.com/ros-industrial/universal_robot/issues/170>`_ from ipa-fxm/prevent_programming
  [Indigo] enable reconfigure and set_param side by side
* Merge pull request `#189 <https://github.com/ros-industrial/universal_robot/issues/189>`_ from abubeck/157_for_indigo
  port of PR `#157 <https://github.com/ros-industrial/universal_robot/issues/157>`_ to indigo
* catkin_lint
* enable reconfigure and set_param side by side
* port of PR `#157 <https://github.com/ros-industrial/universal_robot/issues/157>`_ to indigo
* driver: Factor out __send_message method.
* porting PR `#165 <https://github.com/ros-industrial/universal_robot/issues/165>`_ to indigo
* dynamic reconfigure server for prevent_programming
* RobotStateRT V15 added to indigo_devw
* Changed variable params_mult to unique name in each socket_read instance.
* Fixed unpacking of MasterboardData_V30.
* Contributors: Alexander Bubeck, Dan Solomon, Maarten de Vries, Thomas Timm Andersen, Wouter van Oijen, abubeck, ipa-fxm, jeppewalther

1.0.2 (2014-03-31)
------------------

1.0.1 (2014-03-31)
------------------

* Merge pull request `#25 <https://github.com/ros-industrial/universal_robot/issues/25>`_ from ros-industrial/groovy-devel
  Merging latest changed from groovy to hydro devel branch
* completed adding the workarounds from the original driver script
* ported modifications made to original driver script
* ur_driver: run_depend on trajectory_msgs.
* ur_driver: run_depend on beautifulsoup. Fix `#29 <https://github.com/ros-industrial/universal_robot/issues/29>`_.
* Added keywords to arguments for new message instance of JointTrajectoryPoint().
  Arguments were previously "in order" but the keyword arguments are recommended as they are more resilient to msg changes.
* Add 1 sec throttle to unknown pkt warning.
* Fix formatting and sort list of unknown ptypes.
* Ignore unknown pkt types, instead of erroring out.
  This change modifies the behaviour of the driver in case an
  unknown packet type is encountered. Previously, the deserialisation
  routine would raise an exception, which would cause the driver
  to print an error and exit.
  In the current implementation the unknown packet type(s) are
  stored, their payload ignored, and the driver reports the fact that
  unknown type(s) was/were received (with a request to report the
  warning, ideally to the package maintainer).
  This is not a solution for `#16 <https://github.com/ros-industrial/universal_robot/issues/16>`_, although that issue prompted this
  change.
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Merge branch 'groovy-devel' into groovy-dev
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Merge remote-tracking branch 'origin/master' into hydro-dev
* Fixed reference from ur5_driver to ur_driver in test_move.py
* patch for ur_driver/deserialize.py
* Updated to catkin.  ur_driver's files were added to nested Python directory for including in other packages.
* Added launch directory to bringup package.  Left the old launch files to be backwards compatable for now.  Updated driver with SwRI internal changes.  Changed driver to accept ip address to match ROS-Industrial standard.  IP addresses for industrial robots are typically fixed.  Hostnames are not typically used (this is what the driver expected)
* Renamed packages and new groovy version
* Added ur10 and renamed packages
* Contributors: Christina Gomez, Damien Smeets, IPR-SR2, Kelsey, ROS-Industrial, Shaun Edwards, gavanderhoorn, ipa-nhg, robot, ros-industrial
