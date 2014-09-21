cmake_minimum_required(VERSION 2.8.3)
project(ur_kinematics)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs moveit_core
  moveit_ros_planning pluginlib tf_conversions)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur10_kin ur5_kin ur10_moveit_plugin ur5_moveit_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_ros_planning
    pluginlib tf_conversions
  DEPENDS boost
)


###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ur10_kin src/ur_kin.cpp)
set_target_properties(ur10_kin PROPERTIES COMPILE_DEFINITIONS "UR10_PARAMS")

add_library(ur5_kin src/ur_kin.cpp)
set_target_properties(ur5_kin PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")

add_library(ur10_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur10_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR10_PARAMS")
target_link_libraries(ur10_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur10_kin)

add_library(ur5_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur5_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")
target_link_libraries(ur5_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur5_kin)


#############
## Install ##
#############

install(TARGETS ur5_kin ur10_kin ur5_moveit_plugin ur10_moveit_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install moveit plugin description file
install(FILES ur_moveit_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
