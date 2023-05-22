cmake_minimum_required(VERSION 3.5)
project(remote_control)
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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)
################################################################################
# joystick component
################################################################################
set(component_name joystick_component)
add_library(${component_name}
  SHARED
  src/joystick/joystick_component.cpp
  src/joystick/joystick_component_params.cpp
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
  sensor_msgs
)
rclcpp_components_register_node(${component_name}
  PLUGIN "remote_control::joystick::JoyStick"
  EXECUTABLE joystick_node
)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
