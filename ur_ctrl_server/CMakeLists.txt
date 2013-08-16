cmake_minimum_required(VERSION 2.8.3)
project(ur_ctrl_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS simple_message)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur_ctrl_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# add_definitions(-DROS=1)  # Build using ROS libraries
# add_definitions(-DLINUXSOCKETS=1)  # Use linux sockets for communication
# add_definitions(-DFLOAT64=1)  # Use 64-bit floats for communication
# add_definitions(-DROBOTDISABLED=1)  # Robot communication is disabled

add_library(ur_ctrl_msgs
    src/simple_msgs/ur_joint_cmd.cpp
    src/simple_msgs/ur_config_cmd.cpp
    src/simple_msgs/ur_state_data.cpp)

add_library(ur_ctrl_base
    src/ur_joint_cmd_handler.cpp
    src/ur_config_cmd_handler.cpp
    src/ur_controller_iface.cpp
)
target_link_libraries(ur_ctrl_base 
    simple_message_float64
    ur_ctrl_msgs
)

# build test controller
add_executable(ur_test_server
    src/ur_test_server.cpp
    src/ur_test_controller.cpp
    src/run_controller.cpp
)
target_link_libraries(ur_test_server ur_ctrl_base)

# build actual controller
find_package(UR-C-API)
if( ${UR-C-API_FOUND} )
    include_directories(${UR-C-API_INCLUDE_DIRS})
    link_directories(${UR-C-API_INCLUDE_DIRS})
    add_executable(ur_ctrl_server
        src/ur_ctrl_server.cpp
        src/ur_hardware_controller.cpp
        src/run_controller.cpp
    )
    target_link_libraries(ur_ctrl_server ${UR-C-API_LIBRARIES} ur_ctrl_base)
endif()

## Declare a cpp library
# add_library(ur_ctrl_server
#   src/${PROJECT_NAME}/ur_ctrl_server.cpp
# )

## Declare a cpp executable
# add_executable(ur_ctrl_server_node src/ur_ctrl_server_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ur_ctrl_server_node ur_ctrl_server_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ur_ctrl_server_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ur_ctrl_server ur_ctrl_server_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_ctrl_server.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
