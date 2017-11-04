cmake_minimum_required(VERSION 2.8.3)
project(ur_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

catkin_python_setup()


generate_dynamic_reconfigure_options(
  cfg/URDriver.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure
)

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
