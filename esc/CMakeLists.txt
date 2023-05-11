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
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   hippocampus_common
#   hippocampus_msgs
#   path_planning
#   dynamic_reconfigure
# )

add_library(esc_serial
  STATIC
  src/esc_serial/src/cobs.cpp
  src/esc_serial/src/crc.cpp
  src/esc_serial/src/esc_serial.cpp
)

ament_export_targets(esc_serialTarget HAS_LIBRARY_TARGET)
target_include_directories(esc_serial
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS esc_serial
  EXPORT esc_serialTarget
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY src/esc_serial/include/
  DESTINATION include/esc_serial
)

include_directories(
  include
  src/esc_serial/include
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
