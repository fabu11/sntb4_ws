

# Dual Robot Simulation
ROS2 Humble workspace for simulating a TurtleBot4 and Hello Robot Stretch in a shared Gazebo environment.

## Developed on
- Ubuntu 22.04 LTS
- ROS2 Humble
- Ignition Fortress v6.x

## Dependencies
### [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
#### Once installed:
```
source /opt/ros/humble/setup.bash
```

Or if using zsh:
```
source /opt/ros/humble/setup.zsh
```
### [TurtleBot4](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
```
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
```

#### Ignition Fortress
```
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```

Verify installed:
```
ign gazebo --version
```
#### Verify Turtlebot4 packages are installed:
##### Run Simulation:
```
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```

## Build
```
cd $HOME
git clone https://github.com/fabu11/sntb4_ws.git
cd $HOME/sntb4_ws
colcon build --symlink-install --packages-up-to dual_robot
```

### Source
```
source $HOME/sntb4_ws/install/setup.sh
```

Or if using zsh:
```
source $HOME/sntb4_ws/install/setup.zsh
```

### Run Simulation (Once sourced)
```
ros2 launch dual_robot dual_robot.launch.py
```

#### Show args
```
ros2 launch dual_robot dual_robot.launch.py --show-args
```

## Structure
### dual_robot
#### Launch Files
This project uses two launch files:
1. stretch_spawn_only.launch.py
2. dual_robot.launch.py

##### stretch_spawn_only.launch.py
This launch file spawns in the Hello Robot stretch in any existing gazebo world.
Dependent on stretch_gz_sim which can be found in the package *stretch_ros2*. 

##### dual_robot.launch.py
This launch file utilizes the Turtlebot4 debian packages to launch the Turtlebot4 and Gazebo at the same time. Once these are loaded, stretch_spawn_only.launch.py is used to spawn the stretch in the same world. 

#### Worlds
By default, the world is set to *warehouse.sdf*
##### List of worlds
1. warehouse.sdf
2. depot.sdf
3. maze.sdf

## Acknowledgements
This repository utilizes gazebo port made by [Cardiff University](https://github.com/CardiffUniversityComputationalRobotics/stretch_ros2). Their port was built for Gazebo Sim Harmonic. To utilize the simulation on Ignition Fortress, the URDF was ported.
