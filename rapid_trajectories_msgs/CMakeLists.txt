cmake_minimum_required(VERSION 3.5)
project(rapid_trajectories_msgs)

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
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CurrentStateDebug.msg"
  "msg/TargetState.msg"
  "msg/TrackingDebug.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryArray.msg"
  "msg/TrajectoryResult.msg"
  "msg/TrajectoryStamped.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
