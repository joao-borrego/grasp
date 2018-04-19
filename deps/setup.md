### Overall desired setup

1. Ubuntu 16.04.4 LTS (Xenial Xerus)
1. Gazebo 9.0.0
1. DART 6.3.0
1. gazebo_ros_pks for Gazebo-ROS bridge

You can either follow the installation of each of the required dependencies, or simply run the provided [installation script] (needs to be tested).

### Libraries

#### [libccd] - Collision between convex surfaces

Version: 2.0-1 (Stable)

Package: `libccd-dev`

Install with `apt`:
``` bash
sudo apt install libccd-dev
```

Installed to: `/usr/local/include/fcl`

#### [FCL] - Flexible Collision Library 

Version: 0.5 (Stable)

Dependencies:
1. [libccd](#libccd---collision-between-convex-surfaces)
1. [Boost](#boost) (Optional: required for building tests)

Install from source:
``` bash
cd ~/workspace &&
git clone https://github.com/flexible-collision-library/fcl/ &&
cd fcl/ &&
git checkout fcl-0.5 &&
mkdir build &&
cd build &&
cmake .. &&
make -j8 &&
sudo make install
```

Installed to:
1. Headers: `/usr/local/include/fcl/`
1. Binaries: `/usr/local/lib/`

#### [SDF]

Version: 6.0.0

Package: `libsdformat6-dev`

Install with `apt`:
```
sudo apt install libsdformat6-dev
```


### Programs

#### [DART] - Physics simulator

Version: 6.3.0

Dependencies:
1. [FCL](#fcl---flexible-collision-library)
1. [Boost](#boost)
1. [Eigen3](#eigen)

Install from source:
``` bash
sudo apt remove libdart* &&
sudo apt install build-essential cmake pkg-config git &&
sudo apt install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev &&
sudo apt install libopenscenegraph-dev &&
cd ~/workspace &&
git clone https://github.com/dartsim/dart &&
cd dart/ &&
git checkout tags/v6.3.0 &&
mkdir build &&
cd build &&
cmake .. &&
make -j8 &&
sudo make install
```

Installed to:
1. Headers: `/usr/local/include/dart/`
1. Binaries: `/usr/local/lib/`

#### [GAZEBO]

Version: 9.0.0

Dependencies:
1. Too many to count; Thankfully they provide an installer script for them.
1. [DART](#dart---physics-simulator) (Optional)

Install from source:
``` bash
# Setup OSRF keys
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' &&
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - &&
# Dependencies
ROS_DISTRO=kinetic &&
BASE_DEPS="build-essential \ cmake \ debhelper \ mesa-utils \ cppcheck \ xsltproc \ python-lxml \ python-psutil \ python \ bc \ netcat-openbsd \ gnupg2 \ net-tools \ locales" &&
GAZEBO_BASE_DEPS="libfreeimage-dev \ libprotoc-dev \ libprotobuf-dev \ protobuf-compiler \ freeglut3-dev \ libcurl4-openssl-dev \ libtinyxml-dev \ libtar-dev \ libtbb-dev \ libogre-1.9-dev \ libxml2-dev \ pkg-config \ qtbase5-dev \ libqwt-qt5-dev \ libltdl-dev \ libgts-dev \ libboost-thread-dev \ libboost-signals-dev \ libboost-system-dev \ libboost-filesystem-dev \ libboost-program-options-dev \ libboost-regex-dev \ libboost-iostreams-dev \ libbullet-dev \ libsimbody-dev \ \ libignition-transport3-dev \ libignition-math3-dev \ libignition-msgs0-dev \ libtinyxml2-dev \ libignition-msgs-dev \ libignition-transport4-dev" &&
sudo apt install $(sed 's:\\ ::g' <<< $BASE_DEPS) $(sed 's:\\ ::g' <<< $GAZEBO_BASE_DEPS) &&
# Build
cd ~/workspace/ &&
hg clone https://bitbucket.org/osrf/gazebo &&
cd gazebo &&
hg checkout gazebo9_9.0.0 &&
mkdir build &&
cd build &&
cmake .. &&
make -j8 &&
sudo make install &&
# Fix bug
echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo. &&
sudo ldconfig
```

Installed to:
1. Headers: `/usr/local/include/gazebo-9/`
1. Binaries: `/usr/local/lib/gazebo-9/`
1. Resources: `/usr/local/share/gazebo-9/`

#### [gazebo_ros_pkgs]

Branch: `kinetic-devel`

Dependencies:
1. (Not sure since also compiled Baxter)

Install from source:
``` bash
# Dependencies
sudo apt install ros-kinetic-hardware-interface ros-kinetic-polled-camera ros-kinetic-control-toolbox ros-kinetic-controller-manager ros-kinetic-transmission-interface ros-kinetic-camera-info-manager ros-kinetic-joint-limits-interface &&
# Build from source
cd catkin_ws/src &&
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel &&
cd .. &&
catkin_make
```

<!-- Links -->

[DART]: https://github.com/dartsim/dart/tree/v6.3.0
[FCL]: https://github.com/flexible-collision-library/fcl/tree/fcl-0.5
[GAZEBO]: http://gazebosim.org/tutorials?tut=install_from_source
[gazebo_ros_pkgs]: https://github.com/ros-simulation/gazebo_ros_pkgs
[libccd]: https://github.com/danfis/libccd
[SDF]: http://sdformat.com/tutorials?tut=install

[installation script]: install_deps.sh
