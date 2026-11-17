# CS193-final-project
robot that avoids moving people in a crowded area build in ROS2 and gazebo

## Problem
i want to learn more about ros2 and gazebo and this project will solve that problem

## How it works
robot code and control algorithm written in C++ using the ROS 2 framework
crowd simulation code also in C++ and ROS 2
physics and sensor simulation done in Gazebo

the system diagram is in docs/cs193 project flow diagram.png

## Tech stack
- **Language:** C++20, Python (launch)
- **Libraries:** ROS 2 Jazzy, Gazebo Harmonic, Eigen3

## Overview
The project has three main pieces:
 - **Simulation**: Gazebo Harmonic handles physics, collisions, robot model, lidar, odometry
 - **Crowd node**: ROS2 node that creates people and updates them with social force model to avoid each other while reaching their goal
 - **Controller node**: ROS2 node that takes in the gazebo lidar data and plans the path the robot should take

## Package layout
```
crowd_interfaces/        custom messages
  msg/Person.msg         id, position, velocity, radius
  msg/PersonArray.msg    header + Person[]
crowd_sim/               nodes, launch, config, world
  src/crowd.cpp          crowd node: spawns agents, runs the social force model
  src/agent.cpp          agent update + marker publishing
  src/controller.cpp     controller node: lidar in, cmd_vel out
  include/crowd_sim/     headers
  launch/sim.launch.py   starts Gazebo, bridge, nodes, RViz
  config/sim_params.yaml crowd node parameters
  config/world.yaml      wall geometry used to generate the Gazebo world
  gazebo_sim/            world template + build_sdf.py
  rviz/                  RViz config
```

## Setup
1. Install ROS 2, a C++ compiler, and Gazebo
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- Ubuntu 24.04 (tested on WSL2)
- Gazebo Harmonic
2. Clone this repo into `~/ros2_ws/src`
3. Make sure ROS is sourced in every terminal:
```bash
source /opt/ros/jazzy/setup.bash
```
4. Build and source the workspace from `~/ros2_ws`:
```bash
colcon build --packages-up-to crowd_sim
source install/setup.bash
```

## Usage
Run the python launch file to launch all ROS2 nodes, the gazebo sim, the ROS2/gazebo bridge, and RViz2
```bash
ros2 launch crowd_sim sim.launch.py
```
If the visualization seems jittery there may be multiple instances of the ROS2 nodes running. Run
```bash
pkill -f -e 'crowd_sim/|rviz2|gz sim|parameter_bridge|ros2 launch'
```
To delete all instances of the simulation. Then rerun
```bash
ros2 launch crowd_sim sim.launch.py
```
to launch the simulation again

## Roadmap
1. build my robot SDF and make sure it drives in gazebo with proper inertia and collisions with basic terminal 2. command to set speed for testing
3. add a 2D lidar to the robot and build a world with walls and ground planes
4. create a ROS package with a launch file that starts the gazebo sim and  opens RVis
5. create a pedestrian model and a node that controls the pedestrian to move in a straight line and avoid other pedestrians (social force model)
6. write the algorithm for the robot to pathfind around the moving people nodes
