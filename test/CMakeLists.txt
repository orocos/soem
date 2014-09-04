
add_executable(simple_test linux/simple_test/simple_test.c)
target_link_libraries(simple_test soem m c pthread)

add_executable(red_test linux/red_test/red_test.c)
target_link_libraries(red_test soem m c pthread)

add_executable(slaveinfo linux/slaveinfo/slaveinfo.c)
target_link_libraries(slaveinfo soem m c pthread)

add_executable(eepromtool linux/eepromtool/eepromtool.c)
target_link_libraries(eepromtool soem m c pthread)

add_executable(ebox linux/ebox/ebox.c)
target_link_libraries(ebox soem m c pthread)

add_executable(firm_update linux/firm_update/firm_update.c)
target_link_libraries(firm_update soem m c pthread)

install(TARGETS simple_test red_test slaveinfo eepromtool ebox firm_update
  EXPORT SoemTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib )


