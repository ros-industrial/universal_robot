cmake_minimum_required(VERSION 2.8.3)
project(ur_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation ur_msgs)

catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
catkin_package()


###########
## Build ##
###########
catkin_python_setup()

#############
## Install ##
#############
install(PROGRAMS test_move.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/ur_driver/driver.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES test.launch prog prog_reset
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
