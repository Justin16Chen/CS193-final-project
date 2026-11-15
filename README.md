# CS193-final-project
robot that avoids moving people in a crowded area build in ROS2 and gazebo

## Problem
i want to learn more about ros2 and gazebo and this project will solve that problem

## How it works
robot code and control algorithm written in C using ROS2 framework
crowd control code also in C and ROS2
physics and sensor simulation done in gazebo

the system diagram is in docs/cs193 project flow diagram.png

## Tech stack
- **Language:** C
- **Libraries:** ROS2, Gazebo

## Overview
The project has three main pieces:
 - **Simulation**: Gazebo Harmonic handles physics, collisions, robot model, lidar, odometry
 - **Crowd node**: ROS2 node that creates people and updates them with social force model to avoid each other while reaching their goal
 - **Controller node**: ROS2 node that takes in the gazebo lidar data and plans the path the robot should take

## Setup
1. Need to download ROS2 and C and Gazebo
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- Ubuntu 24.04 (tested on WSL2)
- Gazebo Harmonic
2. Clone this repo into WSL ros2_ws folder
4. Make sure ROS is sourced in every terminal:
```bash
source /opt/ros/jazzy/setup.bash
```

## Usage
Run the python launch file to launch all ROS2 nodes, the gazebo sim, the ROS2/gazebo bridge, and RViz2
```bash
ros2 launch cs193project sim.launch.py
```
If the visualization seems jittery there may be multiple instances of the ROS2 nodes running. Run
```bash
pkill -f -e 'cs193project/|rviz2|gz sim|parameter_bridge|ros2 launch'
```
To delete all instances of the simulation. Then rerun
```bash
ros2 launch cs193project sim.launch.py
```
to launch the simulation again

## Roadmap
1. build my robot SDF and make sure it drives in gazebo with proper inertia and collisions with basic terminal 2. command to set speed for testing
3. add a 2D lidar to the robot and build a world with walls and ground planes
4. create a ROS package with a launch file that starts the gazebo sim and  opens RVis
5. create a pedestrian model and a node that controls the pedestrian to move in a straight line and avoid other pedestrians (social force model)
6. write the algorithm for the robot to pathfind around the moving people nodes
