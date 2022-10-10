cmake_minimum_required(VERSION 3.5)
project(hippo_gz_plugins)

find_package(ament_cmake REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)
set(GZ_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})

function(add_plugin plugin_name)
  add_library(${plugin_name} SHARED
  src/${plugin_name}/${plugin_name}.cpp
  src/${plugin_name}/${plugin_name}_private.cpp
  )
  target_include_directories(${plugin_name} PRIVATE include)
  ament_target_dependencies(${plugin_name}
  ignition-plugin${GZ_PLUGIN_VER}
  ignition-gazebo${GZ_GAZEBO_VER}
  ignition-common${GZ_COMMON_VER}
  )

  install(TARGETS ${plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
endfunction()

add_plugin(barometer)
add_plugin(buoyancy)
add_plugin(odometry)
add_plugin(thruster)
# add_subdirectory(src/barometer)
# add_subdirectory(src/buoyancy)
add_subdirectory(src/hydrodynamics)
# add_subdirectory(src/odometry)
add_subdirectory(src/pose)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
