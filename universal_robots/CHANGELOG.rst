^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package universal_robots
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.3 (2024-04-08)
------------------
* UR30 description and meshes (`#674 <https://github.com/ros-industrial/universal_robot/issues/674>`_)
* Contributors: Vincenzo Di Pentima

1.3.2 (2023-12-18)
------------------
* UR20 description and meshes (`#657 <https://github.com/ros-industrial/universal_robot/issues/657>`_)
* Contributors: Rune Søe-Knudsen

1.3.1 (2022-11-11)
------------------
* [meta]: Remove ur_kinematics package for now (`#620 <https://github.com/ros-industrial/universal_robot/issues/620>`_)
  With the dependency we cannot release the metapackage for now, since it is not released.
* Contributors: Felix Exner (fexner)

1.3.0 (2022-11-10)
------------------
* Merge melodic-devel-staging `#617 <https://github.com/ros-industrial/universal_robot/issues/617>`_ from ros-industrial/melodic-devel-staging
* Update MoveIt! support (`#538 <https://github.com/ros-industrial/universal_robot/issues/538>`_)
  Update dependencies of meta package
* Increase minimum CMake version to allow builds on Focal Fossa (Noetic) (`#586 <https://github.com/ros-industrial/universal_robot/issues/586>`_)
* Remove ur_msgs package from the metapackage (`#542 <https://github.com/ros-industrial/universal_robot/issues/542>`_)
  There are no direct dependants any more in this repository.
* Merge pull request `#498 <https://github.com/ros-industrial/universal_robot/issues/498>`_ from gavanderhoorn/remove_ur_driver
  Remove obsolete driver and bringup pkgs
* meta: remove driver and bringup.
* Merge pull request `#371 <https://github.com/ros-industrial/universal_robot/issues/371>`_ from ipa-led/ur_description_args
  Urdf with args and yaml configuration
* deleted ur_e specific repositories
  * moved ur_e_description meshes files to ur_description
* Contributors: Felix Exner, Felix Exner (fexner), G.A. vd. Hoorn, Ludovic Delval, Nadia Hammoudeh García, gavanderhoorn

1.2.5 (2019-04-05)
------------------
* First release (of this package)
* Contributors: gavanderhoorn, Nadia Hammoudeh García
