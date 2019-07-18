cmake_minimum_required(VERSION 2.6.4)

project(soem)

message([=[
=====================================
SOEM ROS Package Upgrade Announcement
=====================================
To any consumers of the SOEM ROS package:

This package will be upgraded to the new Release of the upstream SOEM repo v1.4.0.
The upgrade via this PR on GitHub https://github.com/mgruhler/soem/pull/24 will happen on the

    ===================
    31st of August 2019
    ===================

This upgrade will bring not only the new release, but also changes to the `catkin` plumbing.
This allows you to use soem from within your regular ROS workspace. No more jumping through
the hoops required, that are described in the Usage section of the Readme: https://github.com/mgruhler/soem#usage.

The changes to the build system are backwards compatible.

Changes in upstream SOEM, however, might not be! Please test the above linked PR and provide
feedback until the merge deadline, if you are a heavy user of this library.

The current state of the SOEM ROS package has been tagged with v1.3.0 if you cannot use the new version.
]=])

find_package(catkin QUIET)
if(catkin_FOUND)
  catkin_package( INCLUDE_DIRS soem osal oshw/linux
                  LIBRARIES soem
                  )
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR} 
                    ${CMAKE_CURRENT_SOURCE_DIR}/osal
                    ${CMAKE_CURRENT_SOURCE_DIR}/oshw/linux
                    ${CMAKE_CURRENT_SOURCE_DIR}/soem )

add_definitions(-DEC_VER1)
add_subdirectory(osal)
add_subdirectory(oshw)
add_subdirectory(soem)
add_subdirectory(test)

install(FILES package.xml DESTINATION share/soem)

# Add all targets to the build-tree export set
export(TARGETS soem osal oshw simple_test red_test slaveinfo eepromtool ebox firm_update FILE "${PROJECT_BINARY_DIR}/SoemTargets.cmake")
 
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE soem)
 

configure_file(soem-config.cmake.in ${PROJECT_BINARY_DIR}/soem-config.cmake @ONLY)
configure_file(soem-config-version.cmake.in ${PROJECT_BINARY_DIR}/soem-config-version.cmake @ONLY)
 
install(FILES ${PROJECT_BINARY_DIR}/soem-config.cmake ${PROJECT_BINARY_DIR}/soem-config-version.cmake
  DESTINATION share/soem)
 
# Install the export set for use with the install-tree
install(EXPORT SoemTargets DESTINATION share/soem)



