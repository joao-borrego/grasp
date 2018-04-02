#!/bin/bash

# Source Gazebo environment
source /usr/local/share/gazebo-9/setup.sh

# Export location of gazebo plugins
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:`pwd`/build/plugins

echo "Successfully initialised."
