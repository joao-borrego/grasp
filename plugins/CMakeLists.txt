# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

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

# Hand Gazebo plugin
add_library(HandPlugin SHARED HandPlugin.cc )
target_link_libraries(HandPlugin
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(HandPlugin
    grasp_msgs)

# Grasp target Gazebo plugin
add_library(TargetPlugin SHARED TargetPlugin.cc )
target_link_libraries(TargetPlugin
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(TargetPlugin
    grasp_msgs)

# RGBD Camera Gazebo plugin
add_library(RGBDCameraPlugin SHARED
    RGBDCameraPlugin.cc)
target_link_libraries(RGBDCameraPlugin
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(RGBDCameraPlugin
    grasp_msgs)

# Contact manager Gazebo World plugin
add_library(ContactWorldPlugin SHARED ContactWorldPlugin.cc )
target_link_libraries(ContactWorldPlugin
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(ContactWorldPlugin
    grasp_msgs)

# Contact Sensor Gazebo plugin
add_library(ContactSensorPlugin SHARED ContactSensorPlugin.cc )
target_link_libraries(ContactSensorPlugin
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(ContactSensorPlugin
    grasp_msgs)
