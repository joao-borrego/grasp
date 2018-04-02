#!/bin/bash

# Source Gazebo environment
source /usr/local/share/gazebo-9/setup.sh

# Export path to model meshes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/hands
# Export path to model meshes
export GAZEBO_MEDIA_PATH=${GAZEBO_MEDIA_PATH}:`pwd`/hands
# Export path to model meshes
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:`pwd`/hands


echo "Successfully initialised."