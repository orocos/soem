cmake_minimum_required(VERSION 2.8.3)
project(soem)

find_package(catkin REQUIRED)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/soem)

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

add_subdirectory(SOEM)
