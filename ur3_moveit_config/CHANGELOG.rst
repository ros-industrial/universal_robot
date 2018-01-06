^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur3_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2018-01-06)
-------------------
* Merge pull request `#321 <https://github.com//ros-industrial/universal_robot/issues/321>`_ from gavanderhoorn/bp_266_indigo-devel
  Backport `#266 <https://github.com//ros-industrial/universal_robot/issues/266>`_ to Indigo.
* Reduce longest valid segment fraction to accomodate non-limited version of the UR5 (`#266 <https://github.com//ros-industrial/universal_robot/issues/266>`_)
* Contributors: G.A. vd. Hoorn, Scott Paulin

1.1.10 (2017-08-04)
-------------------
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
* apply default RRTConnect to ur3
* add moveit_config for ur3
* Contributors: ipa-fxm
