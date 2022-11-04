cmake_minimum_required(VERSION 3.5)
project(rapid_trajectories)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

################################################################################
# install headers
################################################################################
install(
  DIRECTORY include/
  DESTINATION include
)

################################################################################
# trajectory_generator library
################################################################################

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

################################################################################
# single_tracker node
################################################################################
set(exec_name single_tracker_node)
add_executable(${exec_name} src/nodes/single_tracker_node.cpp)
ament_target_dependencies(${exec_name}
  rclcpp
  hippo_msgs
  hippo_common
  nav_msgs
  visualization_msgs
)
target_link_libraries(${exec_name} trajectory_generator)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
