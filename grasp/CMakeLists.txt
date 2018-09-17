# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# Find packages
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
  pkg_check_modules(YAML yaml-cpp)
endif()

find_package(yaml-cpp REQUIRED)
find_package(gap REQUIRED VERSION 1.2)

# Link GAP
include_directories(${GAP_INCLUDE_DIRS})
link_directories(${GAP_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAP_CXX_FLAGS}")

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

# Obtain target object resting pose script
add_executable (gen_rest_poses
  gen_rest_poses.cc
  object_utils.cc)
target_link_libraries(gen_rest_poses
  grasp_msgs
  ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
  ${YAML_CPP_LIBRARIES})
add_dependencies(gen_rest_poses
  grasp_msgs)

# Run grasping trials script
add_executable (run_trials
  run_trials.cc
  gen_grasps.cc
  object_utils.cc
  Grasp.cc)
target_link_libraries(run_trials
  grasp_msgs
  grasp_interface
  gap_msgs
  dr_interface
  ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
  ${YAML_CPP_LIBRARIES})
add_dependencies(run_trials
  grasp_msgs
  grasp_interface)

# Obtain camera frame per grasp trial
add_executable (gen_renders
  gen_renders.cc
  gen_grasps.cc
  object_utils.cc
  Grasp.cc)
target_link_libraries(gen_renders
  grasp_msgs
  ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
  ${YAML_CPP_LIBRARIES})
add_dependencies(gen_renders
  grasp_msgs)
