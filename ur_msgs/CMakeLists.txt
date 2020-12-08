cmake_minimum_required(VERSION 2.8.3)
project(ur_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Analog.msg
   Digital.msg
   IOStates.msg
   RobotStateRTMsg.msg
   MasterboardDataMsg.msg
   RobotModeDataMsg.msg
   ToolDataMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SetPayload.srv
   SetSpeedSliderFraction.srv
   SetIO.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_msgs
   CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############
