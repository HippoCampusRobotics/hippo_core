cmake_minimum_required(VERSION 3.5)
project(hippo_gz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# needed for clang-tidy to work
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2)
add_compile_options(${CXX_FLAGS})


find_package(ament_cmake REQUIRED)
find_package(rclcpp_components)
find_package(hippo_gz_msgs REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)
set(GZ_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})

find_package(sdformat12 REQUIRED)
set(GZ_SDF_VER ${sdformat12_VERSION_MAJOR})

find_package(ignition-transport11 REQUIRED)
set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

# find_package(ignition-msgs8 REQUIRED)
# set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

# find_package(Protobuf REQUIRED)
# set(Protobuf_IMPORT_DIRS ${ignition-msgs8_INCLUDE_DIRS})
# add_subdirectory(proto/msgs)
# set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)
# ament_export_include_directories(${CMAKE_BINARY_DIR}/proto/msgs)


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
  sdformat${GZ_SDF_VER}
  hippo_gz_msgs
  )

  install(TARGETS ${plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
endfunction()

################################################################################
## range_sensor_bridge composable node
################################################################################
set(component_name range_sensor_bridge)
add_library(${component_name}
  SHARED
  src/range_sensor_bridge/range_sensor_bridge.cpp
)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${component_name}
  rclcpp
  rclcpp_components
  hippo_common
  hippo_msgs
  hippo_gz_msgs
  ignition-transport${GZ_TRANSPORT_VER}
)
rclcpp_components_register_node(${component_name} PLUGIN "hippo_gz_plugins::range_sensor_bridge::RangeSensor" EXECUTABLE ${component_name}_node)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_plugin(barometer)
add_plugin(buoyancy)
add_plugin(odometry)
add_plugin(thruster)
add_plugin(range_sensor)
# add_subdirectory(src/barometer)
# add_subdirectory(src/buoyancy)
add_subdirectory(src/hydrodynamics)
# add_subdirectory(src/odometry)
add_subdirectory(src/pose)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
