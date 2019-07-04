cmake_minimum_required(VERSION 2.8.12)
project(soem)

find_package(catkin REQUIRED)

catkin_destinations() # set-up destination variables
# create directory during CMake build system creation step
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Copy header to ROS-standard location in devel space to satisfy catkin_package
add_custom_target(CopyHeaders ALL
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/soem/*.h
                          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/osal/osal.h
                          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/osal/linux/osal_defs.h
                          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/oshw/linux/nicdrv.h
                          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                  COMMENT "Copying SOEM headers to ROS-standard in devel space location.")

catkin_package(
  CFG_EXTRAS soem-ros-extras.cmake
  LIBRARIES soem
)

# SOEM is, by default, built with position-dependent code as a static library.
# To be able to include SOEM in a dynamic library (default ROS convention),
# it needs to be built with position-independent code.
# Thus, we add -fPIC here.
# This allows us to still include this in any shared libraries we create.
add_compile_options(-fPIC)
add_subdirectory(SOEM)
