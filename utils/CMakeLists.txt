# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

# Find packages
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
# Include and link Gazebo and SDF libraries
include_directories(${GAZEBO_INCLUDE_DIRS} ${SDF_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Include custom messages
include_directories(${PROJECT_BINARY_DIR}/msgs)
link_directories(${PROJECT_BINARY_DIR}/msgs)

# Gripper plugin example client
add_executable (gripper_remote gripper_remote.cc)
target_link_libraries(gripper_remote
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(
    gripper_remote
    grasp_msgs)

# Hand plugin example client
add_executable (hand_remote hand_remote.cc)
target_link_libraries(hand_remote
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(
    hand_remote
    grasp_msgs)

# Target plugin example client
add_executable (target_example target_example.cc)
target_link_libraries(target_example
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(
    target_example
    grasp_msgs)

# Contact world plugin example client
add_executable (contacts_example contacts_example.cc)
target_link_libraries(contacts_example
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(
    contacts_example
    grasp_msgs)

# Configure header-only debug utilities 
configure_file (${PROJECT_SOURCE_DIR}/utils/debug.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/debug.hh )
