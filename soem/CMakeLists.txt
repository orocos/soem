#common commands for building c++ executables and libraries
find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
option(ENABLE_RTNET "Enable RTnet over Xenomai." ON)
if(RTNET_INCLUDE_DIR AND ENABLE_RTNET)
  message("Found RTNET: ${RTNET_INCLUDE_DIR}")
  include_directories(${RTNET_INCLUDE_DIR})
  add_definitions(-DHAVE_RTNET)
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -lnative")
endif()

add_library(soem SHARED ethercatbase.c ethercatcoe.c ethercatconfig.c ethercatfoe.c ethercatsoe.c ethercatmain.c ethercatprint.c nicdrv.c ethercatdc.c) 

add_executable(simple_test simple_test.c)
target_link_libraries(simple_test soem m rt c pq pthread)

add_executable(red_test red_test.c)
target_link_libraries(red_test soem m rt c pq pthread)

add_executable(slaveinfo slaveinfo.c)
target_link_libraries(slaveinfo soem m rt c pq pthread)

add_executable(eepromtool eepromtool.c)
target_link_libraries(eepromtool soem m rt c pq pthread)

add_executable(ebox ebox.c)
target_link_libraries(ebox soem m rt c pq pthread)

install(TARGETS soem simple_test red_test slaveinfo eepromtool ebox
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib)

install(FILES ethercatbase.h ethercatcoe.h ethercatconfig.h 
  ethercatconfiglist.h ethercatdc.h ethercatfoe.h 
  ethercatmain.h ethercatprint.h ethercatsoe.h 
  ethercattype.h nicdrv.h
  DESTINATION include/soem )



