#!/bin/bash

# Script to install dependencies

# Stop on error
set -e
# Create directories
mkdir -p ~/workspace/deps/
mkdir -p ~/catkin_ws/

# libccd

sudo apt install libccd-dev

# FCL from source

cd ~/workspace/deps &&
git clone https://github.com/flexible-collision-library/fcl/ ~/workspace/deps &&
cd fcl/ &&
git checkout fcl-0.5 &&
mkdir build &&
cd build &&
cmake .. &&
make -j8 &&
sudo make install

# SDF 6

sudo apt install libsdformat6-dev

# DART from source

sudo apt remove libdart*
sudo apt install build-essential cmake pkg-config git
sudo apt install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev
sudo apt install libopenscenegraph-dev
cd ~/workspace/deps &&
git clone https://github.com/dartsim/dart &&
cd dart/ &&
git checkout tags/v6.3.0 &&
mkdir build &&
cd build &&
cmake .. &&
make -j8 &&
sudo make install

# Gazebo from source

# Setup OSRF keys
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# Dependencies
ROS_DISTRO=kinetic
BASE_DEPS="build-essential \ cmake \ debhelper \ mesa-utils \ cppcheck \ xsltproc \ python-lxml \ python-psutil \ python \ bc \ netcat-openbsd \ gnupg2 \ net-tools \ locales"
GAZEBO_BASE_DEPS="libfreeimage-dev \ libprotoc-dev \ libprotobuf-dev \ protobuf-compiler \ freeglut3-dev \ libcurl4-openssl-dev \ libtinyxml-dev \ libtar-dev \ libtbb-dev \ libogre-1.9-dev \ libxml2-dev \ pkg-config \ qtbase5-dev \ libqwt-qt5-dev \ libltdl-dev \ libgts-dev \ libboost-thread-dev \ libboost-signals-dev \ libboost-system-dev \ libboost-filesystem-dev \ libboost-program-options-dev \ libboost-regex-dev \ libboost-iostreams-dev \ libbullet-dev \ libsimbody-dev \ \ libignition-transport3-dev \ libignition-math3-dev \ libignition-msgs0-dev \ libtinyxml2-dev \ libignition-msgs-dev \ libignition-transport4-dev"
sudo apt install $(sed 's:\\ ::g' <<< $BASE_DEPS) $(sed 's:\\ ::g' <<< $GAZEBO_BASE_DEPS)
# Build
cd ~/workspace/deps &&
hg clone https://bitbucket.org/osrf/gazebo &&
cd gazebo &&
hg checkout gazebo9_9.0.0 &&
mkdir build &&
cd build &&
cmake .. &&
make -j8 &&
sudo make install
# Fix startup bug
echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.
sudo ldconfig

# Gazebo ROS packages

# Dependencies
sudo apt install ros-kinetic-hardware-interface ros-kinetic-polled-camera ros-kinetic-control-toolbox ros-kinetic-controller-manager ros-kinetic-transmission-interface ros-kinetic-camera-info-manager ros-kinetic-joint-limits-interface
# Build from source
cd catkin_ws/src &&
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel &&
cd .. && 
catkin_make

echo "Success! Dependencies installed!"
