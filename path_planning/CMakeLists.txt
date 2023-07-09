# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(path_planning)
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
find_package(Eigen3 REQUIRED)
find_package(hippo_common REQUIRED)
find_package(gflags REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

set(lib_name path_planning)
add_library(${lib_name}
  SHARED
  src/path.cpp
  src/static_generation.cpp
)
ament_target_dependencies(${lib_name}
  Eigen3
  hippo_common
)
target_link_libraries(${lib_name}
  gflags
)
ament_export_targets(${lib_name}Target HAS_LIBRARY_TARGET)
target_include_directories(${lib_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
install(TARGETS ${lib_name}
  EXPORT ${lib_name}Target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

set(exec_name generate_waypoints)
add_executable(${exec_name} src/generate_waypoints.cpp)
ament_target_dependencies(${exec_name}
  Eigen3
)
target_link_libraries(${exec_name}
  gflags
  path_planning
)
install(TARGETS
  generate_waypoints
  DESTINATION lib/${PROJECT_NAME}
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/set_path.py
  DESTINATION lib/${PROJECT_NAME}
)
################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
