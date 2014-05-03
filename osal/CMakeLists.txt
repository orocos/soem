#common commands for building c++ executables and libraries

add_library(osal SHARED linux/osal.c)
target_link_libraries(osal rt)

install(TARGETS osal
  EXPORT SoemTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib )

install(FILES osal.h
        DESTINATION include/soem )

