#!/bin/bash

# Source Gazebo environment
source /usr/local/share/gazebo-9/setup.sh

# Export location of gazebo plugins
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/build/plugins

# Export location of Gazebo models

# Datasets
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/datasets/kit
# export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/datasets/3dnet
# export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/datasets/ycb
# export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/datasets/ShapeNetSem

# Other
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:`pwd`/models

echo "Successfully initialised."
