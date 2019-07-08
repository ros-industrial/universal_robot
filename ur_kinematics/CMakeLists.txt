cmake_minimum_required(VERSION 2.8.3)
project(ur_kinematics)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs moveit_core moveit_kinematics
  moveit_ros_planning pluginlib tf_conversions)

find_package(Boost REQUIRED COMPONENTS system)
find_package(benchmark REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur3_kin ur3_moveit_plugin ur5_moveit_plugin ur10_moveit_plugin ur3_e_moveit_plugin ur5_e_moveit_plugin ur10_e_moveit_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_kinematics moveit_ros_planning
    pluginlib tf_conversions
  DEPENDS Boost
)


###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ur3_kin src/ur_kin.cpp)
set_target_properties(ur3_kin PROPERTIES COMPILE_DEFINITIONS "UR3_PARAMS")

add_library(ur3_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur3_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR3_PARAMS")
target_link_libraries(ur3_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

add_library(ur5_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur5_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")
target_link_libraries(ur5_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

add_library(ur10_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur10_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR10_PARAMS")
target_link_libraries(ur10_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

  add_library(ur3_e_moveit_plugin src/ur_moveit_plugin.cpp)
  set_target_properties(ur3_e_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR3_E_PARAMS")
  target_link_libraries(ur3_e_moveit_plugin
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})
  
  add_library(ur5_e_moveit_plugin src/ur_moveit_plugin.cpp)
  set_target_properties(ur5_e_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR5_E_PARAMS")
  target_link_libraries(ur5_e_moveit_plugin
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})
  
  add_library(ur10_e_moveit_plugin src/ur_moveit_plugin.cpp)
  set_target_properties(ur10_e_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR10_E_PARAMS")
  target_link_libraries(ur10_e_moveit_plugin
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

add_executable(benchmark_kinematics src/test_kinematics.cpp)
set_target_properties(benchmark_kinematics PROPERTIES COMPILE_DEFINITIONS "UR3_PARAMS")
target_link_libraries(benchmark_kinematics
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  benchmark
  ur3_kin)

#############
## Install ##
#############

install(TARGETS ur3_moveit_plugin ur5_moveit_plugin ur10_moveit_plugin ur3_e_moveit_plugin ur5_e_moveit_plugin ur10_e_moveit_plugin benchmark_kinematics
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
