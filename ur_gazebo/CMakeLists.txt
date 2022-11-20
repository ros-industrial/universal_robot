cmake_minimum_required(VERSION 3.0.2)
project(ur_gazebo)
find_package(catkin REQUIRED)
catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  foreach(model ur3 ur3e ur5 ur5e ur10 ur10e ur16e)
    roslaunch_add_file_check(launch/test.launch model=${model})
  endforeach()
endif()

install(DIRECTORY controllers launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
