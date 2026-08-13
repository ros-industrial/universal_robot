cmake_minimum_required(VERSION 3.0.2)
project(ur5_moveit_tutorials)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/joint_group_pos_ctrl_wrapper.py
  scripts/keyboard_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)