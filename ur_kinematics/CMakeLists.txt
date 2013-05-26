cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE MinSizeRel)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

##find_package(PythonLibs REQUIRED)
##include_directories(${PYTHON_INCLUDE_DIRS})

##rosbuild_add_executable(ur_kin src/ur_kin.cpp)
rosbuild_add_library(ur10_kin src/ur_kin.cpp)
set_target_properties (ur10_kin PROPERTIES COMPILE_DEFINITIONS "UR10_PARAMS")


rosbuild_add_library(ur5_kin src/ur_kin.cpp)
set_target_properties (ur5_kin PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")

##rosbuild_add_library(ur_kin_py src/ur_kin_py.cpp)
##target_link_libraries(ur_kin_py ${PYTHON_LIBRARIES} boost_numpy ur_kin)
##set_target_properties(ur_kin_py PROPERTIES PREFIX "")
