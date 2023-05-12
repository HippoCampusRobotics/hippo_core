# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(esc)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2)
add_compile_options(${CXX_FLAGS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   hippocampus_common
#   hippocampus_msgs
#   path_planning
#   dynamic_reconfigure
# )

################################################################################
# esc_serial library
################################################################################
set(lib_name esc_serial)
add_library(${lib_name}
  STATIC
  src/esc_serial/src/cobs.cpp
  src/esc_serial/src/crc.cpp
  src/esc_serial/src/esc_serial.cpp
)

target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/esc_serial/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_libraries(${lib_name}_export HAS_LIBRARY_TARGET)

install(TARGETS ${lib_name}
  EXPORT ${lib_name}_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY src/esc_serial/include/
  DESTINATION include
)

include_directories(
  include
  src/esc_serial/include
)

################################################################################
# teensy_commander component
################################################################################
set(component_name teensy_commander)
add_library(${component_name}
  SHARED
  src/teensy/teensy_commander.cpp
  src/teensy/teensy_commander_params.cpp
)
target_link_libraries(${component_name} esc_serial)
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
  std_msgs
  std_srvs
)
rclcpp_components_register_node(${component_name} PLUGIN "esc::teensy::TeensyCommander" EXECUTABLE ${component_name}_node)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

add_executable(esc_commander_node src/esc_commander_node.cpp src/afro_esc.cpp)
ament_target_dependencies(esc_commander_node rclcpp hippo_msgs std_srvs)

# Install Python executables
install(PROGRAMS
  nodes/esc_commander
  DESTINATION lib/${PROJECT_NAME}
)

install (
  TARGETS
  esc_commander_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
