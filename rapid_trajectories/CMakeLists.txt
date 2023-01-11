cmake_minimum_required(VERSION 3.5)
project(rapid_trajectories)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# needed for clang-tidy to work
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(rapid_trajectories_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

################################################################################
# install headers
################################################################################
install(
  DIRECTORY include/
  DESTINATION include
)

################################################################################
# trajectory library
################################################################################

set(lib_name trajectory)
add_library(${lib_name}
  src/trajectory/generator.cpp
  src/trajectory/single_axis.cpp
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
# rviz helper library
################################################################################

set(lib_name rviz_helper)
add_library(${lib_name}
  src/rviz_helper.cpp
)
target_include_directories(${lib_name}
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${lib_name} rclcpp visualization_msgs hippo_common)
ament_export_targets(${lib_name}Target HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp visualization_msgs)

set_property(TARGET ${lib_name} PROPERTY POSITION_INDEPENDENT_CODE ON)

install(TARGETS ${lib_name}
EXPORT ${lib_name}Target
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

################################################################################
# simple_tracker component
################################################################################
set(component_name simple_tracker)
add_library(${component_name}
  SHARED
  src/tracking/simple_tracker.cpp
  src/tracking/simple_tracker_params.cpp
)
target_link_libraries(${component_name} trajectory rviz_helper)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${component_name}
  rclcpp
  rclcpp_components
  hippo_msgs
  hippo_common
  nav_msgs
  visualization_msgs
  rapid_trajectories_msgs
  Eigen3
)
rclcpp_components_register_node(${component_name} PLUGIN "rapid_trajectories::tracking::SimpleTracker" EXECUTABLE ${component_name}_node)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# simulator component
################################################################################
set(component_name simulator)
add_library(${component_name}
  SHARED
  src/simulator/simulator.cpp
  src/simulator/simulator_params.cpp
)
# target_include_directories(${component_name}
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )
ament_target_dependencies(${component_name}
  rclcpp
  rclcpp_components
  hippo_msgs
  hippo_common
  nav_msgs
  Eigen3
)
rclcpp_components_register_node(${component_name} PLUGIN "rapid_trajectories::simulator::Simulator" EXECUTABLE ${component_name}_node)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
