### Installation

Check [setup] for details on the intended software setup and program versions.
The following tutorial assumes you have Ubuntu 16.04 LTS, and that you want a catkin workspace in `~/catkin_ws` and build dependencies from source in `~/deps`.
Start by creating this directory and installing `git` and mercurial:
```bash
mkdir ~/deps &&
sudo apt install git mercurial
```

#### ROS Kinetic Kame

1.1 Install ROS Kinetic Kame, via the official binaries ([official link](http://wiki.ros.org/kinetic/Installation/Ubuntu)):
```bash
# Setup sources and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
# Install ROS desktop (NOT full installation!)
sudo apt update
sudo apt install ros-kinetic-desktop
```

1.2 Setup and create catkin workspace
```bash
# Initialise rosdep
sudo rosdep init && rosdep update
# Source ROS environment variables (for zsh check original link)
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && source ~/.bashrc
# Install tools for installing package dependencies
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
# Create catkin workspace in ~/catkin_ws
mkdir -p ~/catkin_ws/src &&
cd ~/catkin_ws/ &&
catkin_make
```

#### DART Physics engine

2.1 Install FCL from source
```bash
# Install FCL dependencies
sudo apt install libccd-dev
# Install FCL 0.5 from source
cd ~/deps &&
git clone https://github.com/flexible-collision-library/fcl/ &&
cd fcl/ && git checkout fcl-0.5 &&
mkdir build && cd build && cmake .. && make -j8 && sudo make install
```

2.2 Install DART physics engine ([official link](https://dartsim.github.io/install_dart_on_ubuntu.html))
```bash
# Dependencies
sudo apt install build-essential cmake pkg-config git
sudo apt install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev
sudo apt install libopenscenegraph-dev
# Install DART v6.6.2 from source
cd ~/deps &&
git clone git://github.com/dartsim/dart.git
cd dart/ && git checkout tags/v6.6.2 &&
mkdir build && cd build && cmake .. && make -j8 && sudo make install
```

#### GAZEBO simulator

3.1 Install GAZEBO dependencies
```bash
# SDF library
sudo apt install libsdformat6-dev
# Setup OSRF keys
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' &&
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - &&
# Dependencies
ROS_DISTRO=kinetic &&
BASE_DEPS="build-essential \ cmake \ debhelper \ mesa-utils \ cppcheck \ xsltproc \ python-lxml \ python-psutil \ python \ bc \ netcat-openbsd \ gnupg2 \ net-tools \ locales" &&
GAZEBO_BASE_DEPS="libfreeimage-dev \ libprotoc-dev \ libprotobuf-dev \ protobuf-compiler \ freeglut3-dev \ libcurl4-openssl-dev \ libtinyxml-dev \ libtar-dev \ libtbb-dev \ libogre-1.9-dev \ libxml2-dev \ pkg-config \ qtbase5-dev \ libqwt-qt5-dev \ libltdl-dev \ libgts-dev \ libboost-thread-dev \ libboost-signals-dev \ libboost-system-dev \ libboost-filesystem-dev \ libboost-program-options-dev \ libboost-regex-dev \ libboost-iostreams-dev \ libbullet-dev \ libsimbody-dev \ \ libignition-transport3-dev \ libignition-math3-dev \ libignition-msgs0-dev \ libtinyxml2-dev \ libignition-msgs-dev \ libignition-transport4-dev" &&
sudo apt install $(sed 's:\\ ::g' <<< $BASE_DEPS) $(sed 's:\\ ::g' <<< $GAZEBO_BASE_DEPS) &&
```

3.2 Install GAZEBO from source  ([official link](http://gazebosim.org/tutorials?tut=install_from_source))
```bash
cd ~/deps && hg clone https://bitbucket.org/osrf/gazebo &&
cd gazebo && hg checkout gazebo9_9.4.1 &&
mkdir build && cd build && cmake .. && make -j8 &&
sudo make install
# Fix bug
echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo. &&
sudo ldconfig
```

#### gazebo_ros_pkgs

4.1 Install GAZEBO ROS pkgs from source  ([official link](https://dartsim.github.io/install_dart_on_ubuntu.html))
```bash
# Dependencies
sudo apt install ros-kinetic-hardware-interface ros-kinetic-polled-camera ros-kinetic-control-toolbox ros-kinetic-controller-manager ros-kinetic-transmission-interface ros-kinetic-camera-info-manager ros-kinetic-joint-limits-interface
# Build from source
cd ~/catkin_ws/src &&
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel &&
cd ~/catkin_ws && catkin_make
```

#### GAP

5.1 Install custom Gazebo plugins  ([official link](https://github.com/jsbruglie/gap))
```bash
cd ~/deps && git clone https://github.com/jsbruglie/gap.git
cd ~/workspace/gap/ && git checkout v1.5-beta &&
mkdir build && cd build && cmake ../ && make -j8 &&
sudo make install
```

#### Robotic manipulators

6.1 In order to use the provided robotic manipulator description files, Gazebo needs to have access to the required mesh files for the hand models.
Simply clone the necessary repositories to a properly sourced catkin workspace.
There is no need to compile them.
```bash
cd ~/catkin_ws/src &&
# Shadow dexterous hand
git clone https://github.com/shadow-robot/sr_common &&
# Vizzy's (in-house robot) right hand
git clone https://github.com/vislab-tecnico-lisboa/vizzy &&
# Baxter gripper
git clone https://github.com/RethinkRobotics/baxter_common.git
```


[setup]: setup.md

