#!/bin/bash

# this assumes industrial_core and universal_robot are checked out to the same directory
# run ./link_simple_message.sh to generate symbolic links needed for scons installation

ln -s ../../../industrial_core/simple_message/src/ ../src/simple_message
ln -s ../../../industrial_core/simple_message/include/simple_message ../include/simple_message
ln -s ../../../ur-c-api-example-1.8 ../include/ur_c_api
