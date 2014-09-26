cmake_minimum_required(VERSION 2.6)
 
project(UR-C-API)
 
set(CPACK_PACKAGE_VERSION "1.8.14035")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "8")
set(CPACK_PACKAGE_VERSION_PATCH "14035")

export(PACKAGE UR-C-API)
 
set(INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR})
configure_file(UR-C-APIConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/UR-C-APIConfig.cmake" @ONLY)
set(INSTALL_CMAKE_DIR lib/cmake/ur-c-api)
install(FILES ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/UR-C-APIConfig.cmake 
    DESTINATION "${INSTALL_CMAKE_DIR}" 
    COMPONENT dev)

set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Kelsey Hawkins")
 
include(CPack)
 
# To use this:
# Move these files into a UR-C-API directory
# mkdir build && cd build
# cmake ..
# make package
# sudo dpkg -i UR-C-API-1.8.14035-Linux.deb
# Now the location of the C-API files is installed such that it can be found with find_package
