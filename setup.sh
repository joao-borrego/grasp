#!/bin/bash

# Source Gazebo environment
source /usr/local/share/gazebo-9/setup.sh

# Export location of gazebo plugins
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/build/plugins

# Export location of Gazebo models
# export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/DATA/Datasets/YCB/gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/datasets/ycb/gazebo

echo "Successfully initialised."
