cmake_minimum_required(VERSION 2.8.3)
project(ur_ctrl_client)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  ur_ctrl_server 
  controller_manager controller_interface control_toolbox realtime_tools sensor_msgs)

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
#   JointTrajPtFull32.msg
#   JointTrajPtFull64.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
#generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS controller_manager controller_interface control_toolbox
    INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(ur_ctrl_client
#   src/${PROJECT_NAME}/ur_ctrl_client.cpp
# )

## Declare a cpp executable
# add_executable(ur_ctrl_client 
#   src/ur_ctrl_client.cpp 
#   src/ur_msg_handlers.cpp
# )
# target_link_libraries(ur_ctrl_client 
#   simple_message_float64 
#   industrial_robot_client
#   ur_ctrl_msgs
# )

add_executable(ur_ctrl_man
  src/ur_ctrl_man.cpp 
  src/ur_robot_hw.cpp
  src/ur_msg_handlers.cpp
)
target_link_libraries(ur_ctrl_man 
  simple_message_float64 
  controller_manager
  control_toolbox
  ur_ctrl_msgs
)

# add_library(ur_test_controller
#   include/ur_ctrl_client/ur_test_controller.h
#   src/ur_test_controller.cpp
# )
# target_link_libraries(ur_test_controller
#   ${catkin_LIBRARIES}
# )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ur_ctrl_client_node ur_ctrl_client_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ur_ctrl_client_node
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
# install(TARGETS ur_ctrl_client ur_ctrl_client_node
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

  install(FILES test_controllers_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_ctrl_client.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
