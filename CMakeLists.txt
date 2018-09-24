# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

# Project name
project(Grasp)

# Project version
set (GRASP_MAJOR_VERSION 0)
set (GRASP_MINOR_VERSION 1)
set (GRASP_PATCH_VERSION 0)
set (GRASP_VERSION
	${GRASP_MAJOR_VERSION}.${GRASP_MINOR_VERSION})
set (GRASP_FULL_VERSION
	${GRASP_MAJOR_VERSION}.${GRASP_MINOR_VERSION}.${GRASP_PATCH_VERSION})

# Additional cmake modules
set(CMAKE_MODULE_PATH
	${CMAKE_MODULE_PATH}
	${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# CMake safeguards e.g. prevent in-source builds
include(safeguards)

# Check dependencies
find_package(gazebo REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

# Build project

# Gazebo plugin messages
add_subdirectory(msgs)
# Gazebo plugins
add_subdirectory(plugins)
# Utilities
add_subdirectory(utils)
# Main scripts
add_subdirectory(grasp)

# Examples
add_subdirectory(examples)

# Documentation
add_subdirectory(doc)
