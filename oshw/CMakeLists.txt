#common commands for building c++ executables and libraries

add_library(oshw SHARED linux/nicdrv.c linux/oshw.c)
target_link_libraries(oshw pthread)
# Add all targets to the build-tree export set
export(TARGETS oshw FILE "${PROJECT_BINARY_DIR}/SoemTargets.cmake")

install(TARGETS oshw
  EXPORT SoemTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib )


install(FILES linux/oshw.h linux/nicdrv.h
        DESTINATION include/soem )
