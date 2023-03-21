cmake_minimum_required(VERSION 3.5)
project(hippo_sim)
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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gflags REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(hippo_gz_plugins REQUIRED)
find_package(hippo_gz_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
find_package(ignition-math6 REQUIRED)
set(GZ_MATH_VER ${ignition-math6_VERSION_MAJOR})
find_package(ignition-transport11 REQUIRED)
set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})

set(exec_targets "")

set(exec_name fake_vision)
add_executable(${exec_name} src/fake_vision.cpp)
ament_target_dependencies(${exec_name} rclcpp nav_msgs geometry_msgs hippo_common)
list(APPEND exec_targets ${exec_name})

set(exec_name spawn)
add_executable(${exec_name} src/spawn.cpp)
ament_target_dependencies(${exec_name}
  rclcpp
)
target_link_libraries(${exec_name}
  gflags
  ignition-math${GZ_MATH_VER}::core
  ignition-msgs${GZ_MSGS_VER}::core
  ignition-transport${GZ_TRANSPORT_VER}::core
  ignition-gazebo${GZ_GAZEBO_VER}::core
)
list(APPEND exec_targets ${exec_name})

set(exec_name bridge)
add_executable(${exec_name} src/static_bridge.cpp)
ament_target_dependencies(${exec_name}
  rclcpp
  hippo_gz_msgs
  ros_gz_bridge
  ignition-transport${GZ_TRANSPORT_VER}
  geometry_msgs
  nav_msgs
  hippo_msgs
)
list(APPEND exec_targets ${exec_name})

set(exec_name fake_state_estimator)
add_executable(${exec_name} src/fake_state_estimator.cpp)
ament_target_dependencies(${exec_name} rclcpp nav_msgs)
list(APPEND exec_targets ${exec_name})

set(exec_name simulated_baro)
add_executable(${exec_name} src/simulated_baro.cpp)
ament_target_dependencies(${exec_name}
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  hippo_common
)
list(APPEND exec_targets ${exec_name})

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/create_robot_description.py
  scripts/generate_tag_poses.py
  scripts/generate_pool.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${exec_targets}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
