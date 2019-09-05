cmake_minimum_required(VERSION 3.1)
project(ur_kinematics)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_BENCHMARK "Build the ur_kinematics benchmarks (requires google benchmark)" OFF)
option(BUILD_TESTS "Build tests (requires GTest)" OFF)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs moveit_core moveit_kinematics
  moveit_ros_planning pluginlib tf_conversions)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PythonLibs REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur3_kin ur3_moveit_plugin ur5_moveit_plugin ur10_moveit_plugin ur3_e_moveit_plugin ur5_e_moveit_plugin ur10_e_moveit_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_kinematics moveit_ros_planning
    pluginlib tf_conversions
  DEPENDS Boost
)

include_directories(SYSTEM ${PYTHON_NUMPY_INCLUDE_DIR})
include_directories(SYSTEM ${PYTHON_INCLUDE_DIRS})
add_subdirectory(pybind11)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

define_property( GLOBAL PROPERTY UR_TARGETS_PROP
  BRIEF_DOCS "Targets list"
  FULL_DOCS "Targets list")

function(ADD_ROBOT ROBOT)
  string(TOUPPER ${ROBOT} UPPER_ROBOT)
  string(TOLOWER ${ROBOT} LOWER_ROBOT)

  set(ROBOT_PARAMS "${UPPER_ROBOT}_PARAMS")
  set(TARGET_NAME "${LOWER_ROBOT}_moveit_plugin")

  add_library(${TARGET_NAME} src/ur_moveit_plugin.cpp)
  set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_DEFINITIONS ${ROBOT_PARAMS})
  target_link_libraries(${TARGET_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

  set_property(GLOBAL APPEND PROPERTY UR_TARGETS_PROP ${TARGET_NAME})
endfunction()



ADD_ROBOT("ur3")
ADD_ROBOT("ur5")
ADD_ROBOT("ur10")

ADD_ROBOT("ur3_e")
ADD_ROBOT("ur5_e")
ADD_ROBOT("ur10_e")

pybind11_add_module(ur_kin_py src/ur_kin_py.cpp)
target_link_libraries(ur_kin_py PRIVATE ${catkin_LIBRARIES})
set_target_properties(ur_kin_py PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

if(BUILD_BENCHMARK)
  find_package(benchmark REQUIRED)
  add_executable(benchmark_kinematics src/benchmark_kinematics.cpp)
  target_link_libraries(benchmark_kinematics
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    benchmark)
  set_property(GLOBAL APPEND PROPERTY UR_TARGETS_PROP benchmark_kinematics)
endif()

if(BUILD_TESTS)
find_package(GTest REQUIRED)
  add_executable(ur_kinematics_tests src/test_kinematics.cpp )
  # Link test executable against gtest & gtest_main
  target_link_libraries(ur_kinematics_tests GTest::Main GTest::GTest)
  gtest_discover_tests(ur_kinematics_tests)
endif()

#############
## Install ##
#############
get_property(UR_TARGETS GLOBAL PROPERTY UR_TARGETS_PROP)
install(TARGETS ${$UR_TARGETS}
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
