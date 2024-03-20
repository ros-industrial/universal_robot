cmake_minimum_required(VERSION 3.0.2)
project(ur_kinematics)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs moveit_core moveit_kinematics
  moveit_ros_planning pluginlib tf_conversions)

find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur3_kin ur5_kin ur10_kin ur3_moveit_plugin ur5_moveit_plugin ur10_moveit_plugin ur3e_kin ur5e_kin ur10e_kin ur16e_kin ur20_kin ur20_moveit_plugin ur30_kin ur30_moveit_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_kinematics moveit_ros_planning
    pluginlib tf_conversions
  DEPENDS Boost
)


###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ur3_kin src/ur_kin.cpp)
set_target_properties(ur3_kin PROPERTIES COMPILE_DEFINITIONS "UR3_PARAMS")

add_library(ur3e_kin src/ur_kin.cpp)
set_target_properties(ur3e_kin PROPERTIES COMPILE_DEFINITIONS "UR3e_PARAMS")

add_library(ur5_kin src/ur_kin.cpp)
set_target_properties(ur5_kin PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")

add_library(ur5e_kin src/ur_kin.cpp)
set_target_properties(ur5e_kin PROPERTIES COMPILE_DEFINITIONS "UR5e_PARAMS")

add_library(ur10_kin src/ur_kin.cpp)
set_target_properties(ur10_kin PROPERTIES COMPILE_DEFINITIONS "UR10_PARAMS")

add_library(ur10e_kin src/ur_kin.cpp)
set_target_properties(ur10e_kin PROPERTIES COMPILE_DEFINITIONS "UR10e_PARAMS")

add_library(ur16e_kin src/ur_kin.cpp)
set_target_properties(ur16e_kin PROPERTIES COMPILE_DEFINITIONS "UR16e_PARAMS")

add_library(ur20_kin src/ur_kin.cpp)
set_target_properties(ur20_kin PROPERTIES COMPILE_DEFINITIONS "UR20_PARAMS")

add_library(ur30_kin src/ur_kin.cpp)
set_target_properties(ur30_kin PROPERTIES COMPILE_DEFINITIONS "UR30_PARAMS")

add_library(ur3_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur3_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR3_PARAMS")
target_link_libraries(ur3_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur3_kin)

add_library(ur3e_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur3e_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR3e_PARAMS")
target_link_libraries(ur3e_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur3e_kin)

add_library(ur5_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur5_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")
target_link_libraries(ur5_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur5_kin)

add_library(ur5e_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur5e_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR5e_PARAMS")
target_link_libraries(ur5e_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur5e_kin)

add_library(ur10_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur10_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR10_PARAMS")
target_link_libraries(ur10_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur10_kin)

add_library(ur10e_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur10e_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR10e_PARAMS")
target_link_libraries(ur10e_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur10e_kin)

add_library(ur16e_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur16e_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR16e_PARAMS")
target_link_libraries(ur16e_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur16e_kin)

add_library(ur20_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur20_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR20_PARAMS")
target_link_libraries(ur20_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur20_kin)

add_library(ur30_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur30_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR30_PARAMS")
target_link_libraries(ur30_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur30_kin)


#############
## Install ##
#############

install(TARGETS
    ur3_kin
    ur3e_kin
    ur5_kin
    ur5e_kin
    ur10_kin
    ur10e_kin
    ur3_moveit_plugin
    ur3_moveit_plugin
    ur5_moveit_plugin
    ur5e_moveit_plugin
    ur10_moveit_plugin
    ur10e_moveit_plugin
    ur16e_moveit_plugin
    ur20_moveit_plugin
    ur30_moveit_plugin
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
