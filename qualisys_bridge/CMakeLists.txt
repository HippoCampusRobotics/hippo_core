cmake_minimum_required(VERSION 3.5)
project(qualisys_bridge)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# needed for clang-tidy to work
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2 -fPIC)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(px4_msgs REQUIRED)

add_library(qualisys
  src/qualisys/Markup.cpp
  src/qualisys/Network.cpp
  src/qualisys/RTPacket.cpp
  src/qualisys/RTProtocol.cpp
)
target_include_directories(qualisys
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS qualisys
  EXPORT qualisysTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
# ################################################################################
# ## bridge composable node
# ################################################################################
set(component_name bridge)
add_library(${component_name}
  SHARED
  src/bridge/bridge.cpp
  src/ekf.cpp
)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${component_name}
  qualisys
)
ament_target_dependencies(${component_name}
  rclcpp
  rclcpp_components
  geometry_msgs
  hippo_common
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  px4_msgs
)
rclcpp_components_register_node(${component_name} PLUGIN "qualisys_bridge::Bridge" EXECUTABLE ${component_name}_node)

install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(
  DIRECTORY include/
  DESTINATION include
)
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
