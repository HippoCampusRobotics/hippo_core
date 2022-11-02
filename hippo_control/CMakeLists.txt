cmake_minimum_required(VERSION 3.5)
project(hippo_control)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
)
################################################################################
# attitude_control library
################################################################################
set(lib_name attitude_control)
add_library(${lib_name} src/geometric_attitude_control.cpp)
target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${lib_name} rclcpp Eigen3)
ament_export_targets(${lib_name}Target HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3)

install(TARGETS ${lib_name}
  EXPORT ${lib_name}Target
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

################################################################################
# mixer library
################################################################################
set(lib_name mixer)
add_library(${lib_name} src/simple_mixer.cpp)
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
# attitude_control_node
################################################################################
set(exec_name attitude_control_node)
add_executable(${exec_name} src/nodes/attitude_control_node.cpp)
ament_target_dependencies(${exec_name} rclcpp hippo_msgs hippo_common nav_msgs Eigen3)
target_link_libraries(${exec_name} attitude_control)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# actuator_command_mixer_node
################################################################################
set(exec_name actuator_command_mixer_node)
add_executable(${exec_name} src/nodes/actuator_command_mixer_node.cpp)
ament_target_dependencies(${exec_name} rclcpp hippo_msgs hippo_common hippo_msgs)
target_link_libraries(${exec_name} mixer)
install (TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)


################################################################################
# install headers
################################################################################
install(
  DIRECTORY include/
  DESTINATION include
)

################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
