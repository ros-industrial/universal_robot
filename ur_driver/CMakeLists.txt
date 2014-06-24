cmake_minimum_required(VERSION 2.8.3)
project(ur_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED message_generation)

catkin_python_setup()

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
add_service_files(
  FILES
  URSetPayload.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   #control_msgs#   sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package()


#############
## Install ##
#############
install(PROGRAMS test_move.py test_io.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/ur_driver/driver.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES prog prog_reset
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
