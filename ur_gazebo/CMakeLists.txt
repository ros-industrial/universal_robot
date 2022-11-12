cmake_minimum_required(VERSION 3.0.2)

project(ur_gazebo)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/test_ur3.launch)
  roslaunch_add_file_check(launch/test_ur3e.launch)
  roslaunch_add_file_check(launch/test_ur5.launch)
  roslaunch_add_file_check(launch/test_ur5e.launch)
  roslaunch_add_file_check(launch/test_ur10.launch)
  roslaunch_add_file_check(launch/test_ur10e.launch)
  roslaunch_add_file_check(launch/test_ur16e.launch)
endif()

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
