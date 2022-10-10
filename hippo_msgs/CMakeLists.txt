# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(hippo_msgs)

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
