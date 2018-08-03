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

find_package(yaml-cpp REQUIRED)

# Include and link Gazebo and SDF libraries
include_directories(${GAZEBO_INCLUDE_DIRS} ${SDF_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Include and link yaml-cpp library
include_directories(${YAML_CPP_INCLUDE_DIR})
link_directories(${YAML_CPP_INCLUDE_DIR})

# Include custom messages
include_directories(${PROJECT_BINARY_DIR}/msgs)
link_directories(${PROJECT_BINARY_DIR}/msgs)
# Include custom utilities
include_directories(${PROJECT_BINARY_DIR}/utils)
link_directories(${PROJECT_BINARY_DIR}/utils)

# Shared library for hand plugin interface
add_library(grasp_interface SHARED
    GraspShape.cc
    Interface.cc
    utils.cc
)
target_link_libraries(grasp_interface
    grasp_msgs)
add_dependencies(grasp_interface
    grasp_msgs)

# Shared library for domain randomization plugin interface
add_library(dr_interface SHARED
    DRInterface.cc
    utils.cc
)
target_link_libraries(dr_interface
    grasp_msgs)
add_dependencies(grasp_interface
    grasp_msgs)

# Hand plugin example client
add_executable (hand_remote
    hand_remote.cc
    utils.cc)
target_link_libraries(hand_remote
    grasp_msgs
    grasp_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
    ${YAML_CPP_LIBRARIES})
add_dependencies(hand_remote
    grasp_msgs
    grasp_interface)

# Target plugin example client
add_executable (target_example target_example.cc)
target_link_libraries(target_example
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(target_example
    grasp_msgs)

# Contact world plugin example client
add_executable (contacts_example contacts_example.cc)
target_link_libraries(contacts_example
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(contacts_example
    grasp_msgs)

# Domain randomization plugin example client
add_executable (dr_example dr_example.cc)
target_link_libraries(dr_example
    grasp_msgs
    dr_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(dr_example
    grasp_msgs
    dr_interface)

# Configure interface header
configure_file (${PROJECT_SOURCE_DIR}/utils/GraspShape.hh
  ${CMAKE_CURRENT_BINARY_DIR}/GraspShape.hh )
configure_file (${PROJECT_SOURCE_DIR}/utils/Interface.hh
  ${CMAKE_CURRENT_BINARY_DIR}/Interface.hh )
configure_file (${PROJECT_SOURCE_DIR}/utils/utils.hh
  ${CMAKE_CURRENT_BINARY_DIR}/utils.hh )
configure_file (${PROJECT_SOURCE_DIR}/utils/DRInterface.hh
  ${CMAKE_CURRENT_BINARY_DIR}/DRInterface.hh )

# Configure header-only debug utilities
configure_file (${PROJECT_SOURCE_DIR}/utils/debug.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/debug.hh )
