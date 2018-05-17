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
  # Message dependencies
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  ${PROTOBUF_IMPORT_DIRS}/pose.proto
)

# Generate Protobuf sources
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
# Compile shared library
add_library(grasp_msgs SHARED ${PROTO_SRCS})
# Link Protobuf library
target_link_libraries(grasp_msgs ${PROTOBUF_LIBRARY})
