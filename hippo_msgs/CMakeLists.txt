# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(hippo_msgs)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorControls.msg"
  "msg/AttitudeTarget.msg"
  "msg/EscRpms.msg"
  "msg/EscVoltages.msg"
  "msg/EstimatorInnovation.msg"
  "msg/EstimatorSensorBias.msg"
  "msg/EstimatorState.msg"
  "msg/Float64Stamped.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
