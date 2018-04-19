# grasp (working title)

This project consists of a set of tools for performing grasping experiments within [Gazebo] using [DART] physics engine.

### Setup environment and dependencies

Refer to [setup].

### Compilation

Clone the repository to your workspace directory and build from source.

``` bash
cd ~/workspace/grasp/ &&
mkdir build &&
cd build &&
cmake ../ &&
make
```

### Initialisation

Make sure you properly initialise the required environment variables.
We provide a simple script for this:

``` bash
cd ~/workspace/grasp &&
source setup.sh
```

### Simple example

Provided you have Vizzy's repository in a sourced catkin workspace, you can run an example client with the following commands:

``` bash
# In terminal 1
cd ~/workspace/grasp &&
source setup.sh &&
roscore & rosrun gazebo_ros gazebo /DATA/Dropbox/Workspace/grasp/worlds/dart.world --verbose

# In terminal 2
rosrun gazebo_ros spawn_model -file models/vizzy_right_hand.urdf -z 0.05 -roll 1.57 -urdf -model vizzy_hand
./build/utils/hand_remote
# Issue commands like "grasp" or "pose"
```

[DART]: https://dartsim.github.io/
[Gazebo]: http://gazebosim.org/
[setup]: deps/setup.md
