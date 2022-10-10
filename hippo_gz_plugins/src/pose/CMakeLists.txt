set(lib_name pose)
add_library(${lib_name} SHARED
  pose.cpp
)
ament_target_dependencies(${lib_name}
  ignition-plugin${GZ_PLUGIN_VER}
  ignition-gazebo${GZ_GAZEBO_VER}
  ignition-common${GZ_COMMON_VER}
)

install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
