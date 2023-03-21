

# foreach(ITR ${ignition-msgs8_INCLUDE_DIRS})
#   if(ITR MATCHES ".*/ignition/msgs8$")
#     set(Protobuf_IMPORT_DIRS ${Protobuf_IMPORT_DIRS} "${ITR}/ignition/msgs")
#   endif()
# endforeach()

set(msgs
  range_measurement.proto
  range_measurement_array.proto
#   ${Protobuf_IMPORT_DIRS}/header.proto
)
# set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
# set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
set(PROTO_SRC ${PROTO_SRC} PARENT_SCOPE)
set(PROTO_HEADER ${PROTO_HEADER} PARENT_SCOPE)
# message("${Protobuf_IMPORT_DIRS}")

PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER
    ${msgs}
)


add_library(hippo_gz_plugins_msgs ${PROTO_SRC})
include_directories(${Protobuf_IMPORT_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(hippo_gz_plugins_msgs ${PROTOBUF_LIBRARY})
message(${CMAKE_CURRENT_BINARY_DIR})
install(TARGETS hippo_gz_plugins_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.h"
)
# target_include_directories(hippo_gz_plugins_msgs
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
#     $<INSTALL_INTERFACE:>
# )
# add_custom_target(protobuf_compilation DEPENDS ${PROTO_SRC})
