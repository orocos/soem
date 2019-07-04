cmake_minimum_required(VERSION 2.8.3)
project(soem)

find_package(catkin REQUIRED)

# create directory during CMake build system creation step
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/soem)

# Copy header to ROS-standard location to satisfy catkin_package
add_custom_target(CopyHeaders ALL
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/soem/*.h
                          ${CMAKE_CURRENT_SOURCE_DIR}/include/soem
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/osal/osal.h
                          ${CMAKE_CURRENT_SOURCE_DIR}/include/soem
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/osal/linux/osal_defs.h
                          ${CMAKE_CURRENT_SOURCE_DIR}/include/soem
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/oshw/linux/nicdrv.h
                          ${CMAKE_CURRENT_SOURCE_DIR}/include/soem
                  COMMENT "Copying SOEM headers to ROS-standard location.")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES soem
)

# SOEM is, by default, built with position-dependent code as a static library.
# To be able to include SOEM in a dynamic library (default ROS convention),
# it needs to be built with position-independent code.
# Thus, we add -fPIC here.
# This allows us to still include this in any shared libraries we create.
add_definitions(-fPIC)
add_subdirectory(SOEM)
