find_package(Protobuf REQUIRED)

# Obtain Protobuf import directories
set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# Obtain set of proto messages definitions
set (msgs
  # Custom messages
  gripper.proto
  hand.proto
  # Grasp target messages
  target_request.proto
  target_response.proto
  # Camera messages
  camera_request.proto
  camera_response.proto
  # Contact sensor messages
  contact_sensor_request.proto
  contact_sensor_response.proto
  # Contact messages
  contact_request.proto
  contact_response.proto
  # Message dependencies
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  ${PROTOBUF_IMPORT_DIRS}/pose.proto
  ${PROTOBUF_IMPORT_DIRS}/contact.proto
  ${PROTOBUF_IMPORT_DIRS}/joint_wrench.proto
  ${PROTOBUF_IMPORT_DIRS}/wrench.proto
  ${PROTOBUF_IMPORT_DIRS}/friction.proto
  ${PROTOBUF_IMPORT_DIRS}/surface.proto
)

# Generate Protobuf sources
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
# Compile shared library
add_library(grasp_msgs SHARED ${PROTO_SRCS})
# Link Protobuf library
target_link_libraries(grasp_msgs ${PROTOBUF_LIBRARY})

# Obtain header include string
set (headers_list)
foreach(FIL ${msgs})
  get_filename_component(FIL_WE ${FIL} NAME_WE)
  set(headers_list ${headers_list} "\n\#include \"${FIL_WE}.pb.h\"")
endforeach()
string(REPLACE ";" "" message_headers "${headers_list}")

# Configure header for message types
configure_file (${PROJECT_SOURCE_DIR}/msgs/MessageTypes.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/MessageTypes.hh )
# Configure header for default message topics 
configure_file (${PROJECT_SOURCE_DIR}/msgs/MessageTopics.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/MessageTopics.hh )
