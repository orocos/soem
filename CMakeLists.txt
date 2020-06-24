cmake_minimum_required(VERSION 3.0.2)
project(soem)

find_package(catkin REQUIRED)

# set-up destination variables
catkin_destinations()

# create directory during CMake build system creation step
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(WIN32)
  set(OS "win32")
else()
  set(OS "linux")
endif()

file(GLOB SOEM_COMMON_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/SOEM/soem/*.h")
file(GLOB SOEM_OSAL_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/SOEM/osal/${OS}/*.h")
file(GLOB SOEM_OSHW_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/SOEM/oshw/${OS}/*.h")

# Copy header to ROS-standard location in devel space to satisfy catkin_package
add_custom_target(CopyHeaders ALL
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${SOEM_COMMON_HEADERS}
                          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/osal/osal.h
                          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${SOEM_OSAL_HEADERS}
                          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                  COMMAND ${CMAKE_COMMAND} -E copy_if_different
                          ${SOEM_OSHW_HEADERS}
                          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                  COMMENT "Copying SOEM headers to ROS-standard in devel space location.")

if(WIN32)
  add_custom_target(CopyWpcapHeaders ALL
                  COMMAND ${CMAKE_COMMAND} -E copy_directory
                          ${CMAKE_CURRENT_SOURCE_DIR}/SOEM/oshw/${OS}/wpcap/Include/
                          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                  COMMENT "Copying Wpcap headers to ROS-standard in devel space location.")
endif()

# explicitely find pthread to be able to explicitely export the pthread dependency via catkin_package
# see https://github.com/ros/ros_comm/issues/135
set(PTHREAD_LIB "")
find_package(Threads)
if(CMAKE_THREAD_LIBS_INIT)
  string(LENGTH ${CMAKE_THREAD_LIBS_INIT} _length)
  if(_length GREATER 2)
    string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 0 2 _prefix)
    if(${_prefix} STREQUAL "-l")
      math(EXPR _rest_len "${_length} - 2")
      string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 2 ${_rest_len} PTHREAD_LIB)
    endif()
  endif()
endif()

# export dependencies and targets to catkin
# explicitely export pthread, as stated above
catkin_package(
  CFG_EXTRAS soem-ros-extras.cmake
  LIBRARIES soem ${PTHREAD_LIB}
)

# SOEM is, by default, built with position-dependent code as a static library.
# To be able to include SOEM in a dynamic library (default ROS convention),
# it needs to be built with position-independent code.
# Thus, we add -fPIC here.
# This allows us to still include this in any shared libraries we create.
add_compile_options(-fPIC)
# removed any pre-defined `WIN32_LEAN_AND_MEAN`.
# Otherwise, many symbols will be reported as missing.
remove_definitions(-DWIN32_LEAN_AND_MEAN)
#catkin_lint: ignore duplicate_cmd
#catkin_lint: ignore subproject
add_subdirectory(SOEM)
