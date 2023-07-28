# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(depth_estimator)
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
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)


# Install Python modules
# ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  nodes/depth_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
