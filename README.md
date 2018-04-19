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

[DART]: https://dartsim.github.io/
[Gazebo]: http://gazebosim.org/
[setup]: deps/setup.md
