cmake_minimum_required(VERSION 3.5)
project(hippo_common)
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
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(hippo_common
  SHARED
  src/convert.cpp
  src/param_utils.cpp
  src/tf2_utils.cpp
)
ament_target_dependencies(hippo_common
  Eigen3
  geometry_msgs
  rclcpp
)
ament_export_targets(hippo_commonTarget HAS_LIBRARY_TARGET)
target_include_directories(hippo_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS hippo_common
  EXPORT hippo_commonTarget
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
################################################################################
## tf_publisher composable node
################################################################################
add_library(tf_publisher
  SHARED
  src/tf_publisher.cpp
)
rclcpp_components_register_nodes(tf_publisher "hippo_common::TfPublisher")
target_include_directories(tf_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(tf_publisher 
  hippo_common
)
ament_target_dependencies(tf_publisher
  rclcpp
  geometry_msgs
  px4_msgs
  nav_msgs
  tf2_ros
)
install(
  TARGETS tf_publisher
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
add_executable(tf_publisher_node src/nodes/tf_publisher_node.cpp)
target_include_directories(tf_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(tf_publisher_node tf_publisher)
install(TARGETS tf_publisher_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
## rviz_mesh_publisher
################################################################################
set(exec_name rviz_mesh_publisher)
add_executable(${exec_name} src/nodes/rviz_mesh_publisher.cpp)
ament_target_dependencies(${exec_name}
  rclcpp
  visualization_msgs
  geometry_msgs
  tf2_ros
)
target_link_libraries(${exec_name}
  hippo_common
)

install(TARGETS
  ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
