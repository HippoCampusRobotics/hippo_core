# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(path_planning)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2 -fPIC)
add_compile_options(${CXX_FLAGS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hippo_common REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gflags REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

set(lib_name path_planning)
add_library(${lib_name}
  SHARED
  src/path.cpp
  src/static_generation.cpp
  src/rviz_helper.cpp
)
ament_target_dependencies(${lib_name}
  Eigen3
  hippo_common
  rclcpp
  visualization_msgs
)
ament_export_dependencies(Eigen3 hippo_common rclcpp visualization_msgs)
target_link_libraries(${lib_name}
  gflags
)
ament_export_targets(${lib_name}Target HAS_LIBRARY_TARGET)
target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
install(TARGETS ${lib_name}
  EXPORT ${lib_name}Target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

set(exec_name generate_waypoints)
add_executable(${exec_name} src/generate_waypoints.cpp)
ament_target_dependencies(${exec_name}
  Eigen3
)
target_link_libraries(${exec_name}
  gflags
  path_planning
)
install(TARGETS
  generate_waypoints
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# path_follower component
################################################################################
set(component_name path_follower)
add_library(${component_name}
  SHARED
  src/path_follower/path_follower_node.cpp
  src/path_follower/path_follower_node_params.cpp
)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${component_name} yaml-cpp path_planning)
ament_target_dependencies(${component_name}
  rclcpp
  rclcpp_components
  hippo_common
  hippo_msgs
  geometry_msgs
  nav_msgs
  yaml-cpp
)
rclcpp_components_register_node(${component_name}
  PLUGIN "path_planning::PathFollowerNode"
  EXECUTABLE ${component_name}_node
)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/set_path.py
  DESTINATION lib/${PROJECT_NAME}
)
################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
