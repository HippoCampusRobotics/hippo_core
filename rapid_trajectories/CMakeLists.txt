cmake_minimum_required(VERSION 3.5)
project(rapid_trajectories)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)

################################################################################
# install headers
################################################################################
install(
  DIRECTORY include/
  DESTINATION include
)
set(lib_name trajectory_generator)
add_library(${lib_name}
  src/trajectory_generator/generator.cpp
  src/trajectory_generator/single_axis.cpp
)
target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${lib_name} rclcpp)
ament_export_targets(${lib_name}Target HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(TARGETS ${lib_name}
  EXPORT ${lib_name}Target
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


ament_package()
