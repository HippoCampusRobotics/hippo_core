cmake_minimum_required(VERSION 3.5)
project(hippo_common)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# needed for clang-tidy to work
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(hippo_common
  SHARED
  src/convert.cpp
  src/param_utils.cpp
  src/tf2_utils.cpp
)
ament_target_dependencies(hippo_common
  Eigen3
  geometry_msgs
  rclcpp
)
ament_export_targets(hippo_commonTarget HAS_LIBRARY_TARGET)
target_include_directories(hippo_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS hippo_common
  EXPORT hippo_commonTarget
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
