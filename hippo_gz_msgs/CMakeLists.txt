# Guide to create mixed C++ and Python package
# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/

cmake_minimum_required(VERSION 3.5)
project(hippo_gz_msgs)
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
find_package(Protobuf REQUIRED)
find_package(ignition-msgs8 REQUIRED)
set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

set(PROTOBUF_IMPORT_DIRS ${ignition-msgs${GZ_MSGS_VER}_INCLUDE_DIRS})
set(msgs
  msg/range_measurement.proto
  msg/range_measurement_array.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER
  ${msgs}
)
set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)

# include_directories(${PROTOBUF_IMPORT_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
add_library(hippo_gz_msgs SHARED ${PROTO_SRC})
ament_target_dependencies(hippo_gz_msgs Protobuf)
ament_export_targets(hippo_gz_msgsTarget HAS_LIBRARY_TARGET)
target_include_directories(hippo_gz_msgs
  PUBLIC
    $<BUILD_INTERFACE:${PROTOBUF_IMPORT_DIRS}>
    $<INSTALL_INTERFACE:include>
    ${PROTOBUF_IMPORT_DIRS}
)
# ament_export_libraries(hippo_gz_msgs)
install(TARGETS hippo_gz_msgs
  EXPORT hippo_gz_msgsTarget
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
install(FILES ${PROTO_HEADER} DESTINATION include/${PROJECT_NAME}/msg/)


ament_package()
