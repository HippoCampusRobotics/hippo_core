# Copyright (C) 2023 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA

cmake_minimum_required(VERSION 3.5)
project(hippo_control)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# needed for clang-tidy to work
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2 -fPIC)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hippo_control_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(path_planning REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

################################################################################
# tracking_control library
################################################################################
set(lib_name tracking_control)
add_library(${lib_name} src/trajectory_tracking/tracking_control.cpp)
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
# tracking_controller component
################################################################################
set(component_name tracking_controller)
add_library(${component_name}
  SHARED
  src/trajectory_tracking/tracking_controller.cpp
  src/trajectory_tracking/tracking_controller_params.cpp
)
target_link_libraries(${component_name} tracking_control)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${component_name}
  rclcpp
  rclcpp_components
  hippo_common
  hippo_control_msgs
  geometry_msgs
  nav_msgs
)
rclcpp_components_register_node(${component_name}
  PLUGIN "hippo_control::trajectory_tracking::TrackingControllerNode"
  EXECUTABLE ${component_name}_node
)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# attitude_control library
################################################################################
set(lib_name attitude_control)
add_library(${lib_name}
  src/attitude_control/geometric_controller.cpp
  src/attitude_control/quaternion_controller.cpp
)
target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${lib_name} rclcpp Eigen3 hippo_common)
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
# rate_controller library
################################################################################
set(lib_name rate_controller)
add_library(${lib_name} src/rate_control/rate_controller.cpp)
target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${lib_name} Eigen3)
ament_export_targets(${lib_name}Target HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

install(TARGETS ${lib_name}
  EXPORT ${lib_name}Target
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

################################################################################
# rate_control component
################################################################################
set(component_name rate_control)
add_library(${component_name}
  SHARED
  src/rate_control/rate_control_node/rate_control_node.cpp
  src/rate_control/rate_control_node/rate_control_node_params.cpp
)
target_link_libraries(${component_name} rate_controller)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${component_name}
  rclcpp
  rclcpp_components
  hippo_common
  hippo_control_msgs
  geometry_msgs
  nav_msgs
)
rclcpp_components_register_node(${component_name}
  PLUGIN "hippo_control::rate_control::RateControlNode"
  EXECUTABLE ${component_name}_node
)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# mixer library
################################################################################
set(lib_name mixer)
add_library(${lib_name} src/mixer/simple_mixer.cpp)
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
# mixer bluerov library
################################################################################
set(lib_name mixer_bluerov)
add_library(${lib_name} src/mixer_bluerov/simple_mixer_bluerov.cpp)
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
# geometric_attitude_control component
################################################################################
set(component_name geometric_attitude_control)
add_library(${component_name}
  SHARED
  src/attitude_control/geometric_control_node/geometric_control_node.cpp
  src/attitude_control/geometric_control_node/geometric_control_node_params.cpp
)
target_link_libraries(${component_name} attitude_control)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${component_name}
  rclcpp rclcpp_components hippo_control_msgs hippo_common nav_msgs Eigen3
)
rclcpp_components_register_node(${component_name}
  PLUGIN "hippo_control::attitude_control::GeometricControlNode"
  EXECUTABLE ${component_name}_node
)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# quaternion_attitude_control component
################################################################################
set(component_name quaternion_attitude_control)
add_library(${component_name}
  SHARED
  src/attitude_control/quaternion_control_node/quaternion_control_node.cpp
  src/attitude_control/quaternion_control_node/quaternion_control_node_params.cpp
)
target_link_libraries(${component_name} attitude_control)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${component_name}
  rclcpp rclcpp_components hippo_control_msgs hippo_common nav_msgs Eigen3
)
rclcpp_components_register_node(${component_name}
  PLUGIN "hippo_control::attitude_control::QuaternionControlNode"
  EXECUTABLE ${component_name}_node
)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# motor_failure library
################################################################################
set(lib_name motor_failure)
add_library(${lib_name} src/motor_failure/motor_failure.cpp)
target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${lib_name} Eigen3)
ament_export_targets(${lib_name}Target HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

install(TARGETS ${lib_name}
  EXPORT ${lib_name}Target
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

################################################################################
# motor_failure component
################################################################################
set(component_name motor_failure_control)
add_library(${component_name}
  SHARED
  src/motor_failure/control_node/control_node.cpp
  src/motor_failure/control_node/control_node_params.cpp
)
target_link_libraries(${component_name} motor_failure)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${component_name}
  rclcpp rclcpp_components hippo_control_msgs hippo_common nav_msgs Eigen3 std_srvs
)
rclcpp_components_register_node(${component_name}
  PLUGIN "hippo_control::motor_failure::ControlNode"
  EXECUTABLE ${component_name}_node
)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# actuator_mixer component
################################################################################
set(component_name actuator_mixer)
add_library(${component_name}
  SHARED
  src/mixer/actuator_mixer_node.cpp
  src/mixer/actuator_mixer_node_params.cpp
)
target_link_libraries(${component_name} mixer)
ament_target_dependencies(${component_name}
  rclcpp rclcpp_components hippo_control_msgs hippo_common
)
rclcpp_components_register_node(${component_name}
  PLUGIN "hippo_control::mixer::ActuatorMixerNode"
  EXECUTABLE ${component_name}_node
)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


################################################################################
# actuator_mixer_bluerov_node
################################################################################
set(exec_name actuator_mixer_bluerov_node)
add_executable(${exec_name}
  src/mixer_bluerov/actuator_mixer_bluerov_node.cpp
  src/mixer_bluerov/actuator_mixer_bluerov_node_params.cpp
)
ament_target_dependencies(${exec_name} rclcpp hippo_control_msgs hippo_common hippo_control_msgs)
target_link_libraries(${exec_name} mixer_bluerov)
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
