cmake_minimum_required(VERSION 2.8.12)
project(soem)

find_package(catkin REQUIRED)

message([=[
        
          SOEM ROS Package Upgrade Announcement
          =====================================
          
          Prior, due to the code layout in soem, a workaround
          has been necessary in users of this library.
          This version solves this issue now by
          providing proper cmake configuration files.
          
          If you've used the workaround of specifying
          `${soem_INCLUDE_DIRS}/soem` in the include_directories section
          in your code/library, you should remove this now.
          
          Also, this package has been upgraded to the upstream
          SOEM release 1.4.0.
          If you experience any problems, you can revert to
          release v1.3.0 of this package to retrieve the old state.
        ]=]
       )

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
#catkin_lint: ignore duplicate_cmd
#catkin_lint: ignore subproject
add_subdirectory(SOEM)
