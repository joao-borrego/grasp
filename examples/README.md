### Running

## Examples

Open up two terminals in the root directory of the repository.

On terminal 1 launch ROS and gazebo server:
```bash
cd ~/workspace/grasp/ &&
source setup.sh &&
roscore & rosrun gazebo_ros gazebo worlds/dart.world --verbose
```

On terminal 2 run:
```bash
cd ~/workspace/grasp/ &&
# Spawn shadow hand
rosrun gazebo_ros spawn_model -file models/shadowhand.urdf -urdf -model shadowhand
# Launch hand remote example
./build/bin/hand_remote -c cfg/robots.yml -r shadow
