# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(hardware)
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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)

################################################################################
# i2c library
################################################################################
set(lib_name i2c)
add_library(${lib_name} src/i2c/i2c.cpp)
target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# ament_target_dependencies(${lib_name} rclcpp)
ament_export_targets(${lib_name}Target HAS_LIBRARY_TARGET)
# ament_export_dependencies(rclcpp)

install(TARGETS ${lib_name}
  EXPORT ${lib_name}Target
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

################################################################################
# ms5837 library
################################################################################
set(lib_name ms5837)
add_library(${lib_name} src/barometer/ms5837.cpp)
target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${lib_name} i2c)
# ament_target_dependencies(${lib_name} rclcpp)
ament_export_targets(${lib_name}Target HAS_LIBRARY_TARGET)
# ament_export_dependencies(rclcpp)

install(TARGETS ${lib_name}
  EXPORT ${lib_name}Target
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

################################################################################
# barometer node
################################################################################
set(exec_name barometer)
add_executable(${exec_name}
  src/barometer/barometer_node.cpp
  src/barometer/barometer.cpp
  src/barometer/barometer_params.cpp
)
target_link_libraries(${exec_name} ms5837)
ament_target_dependencies(${exec_name}
  rclcpp
  rclcpp_components
  hippo_common
  hippo_msgs
  sensor_msgs
  std_srvs
)
install(TARGETS ${exec_name}
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/camera_servo_node
  nodes/spotlight_node
  nodes/newton_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# install headers
################################################################################
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
