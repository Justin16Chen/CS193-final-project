# CS193-final-project
Creating a robot with a lidar and IMU that can pathfind through an area crowded with people behaving according to a social force model. It will be able to navigate through the crowds of moving people without hitting anyone, going from some starting location to some target location.

## Problem
ROS2 is the industry standard framework for deploying code for robotics system. It consists of a system to publish and subscribe to topics. Gazebo is a widely used simulation that simulates realistic collisions, physics, and sensors. Using ROS2 and Gazebo is a common practice to develop initial software for robotic systems to rapidly test the control systems first in simulation, then on the robot. There are many advantages to this, as you can reproduce many hard to create scenarios in real life very easily in simulation, and you can rapidly develop and debug the code. I want to go into robotics in the future, but the problem is I have never used ROS2 or Gazebo before. This project solves the problem because I will learn a lot more about the 2 systems and how to use them in an actual project.

## How it works
Crowd simulation, math helpers, and robot code written in C++
Robot model (wheels, chassis, lidar sensor, IMU sensor) and walls written in a SDF file
Crowd geometry dynamically inserted into SDF file
physics and sensor simulation done in Gazebo (lidar detects where the people are, IMU tracks where robot is)
C++ code communicates with Gazebo simulation through ROS2-GAZEBO bridge (ROS2 library node)
parameters I can tune (crowd sim parameters, pathfinding algorithm parameters) stored in .YAML files that the C++ files read
dependencies and compilation done with XML and CmakeLists.txt files
RViz (part of the ROS2 library) for visualizing debug information (estimated velocity of each person, for example)
launch file (to launch ROS2 nodes, gazebo simulation, RViz, and ROS2-GAZEBO bridge) written in python
bash scripts to simplify process of compiling and running everything

the system diagram is in docs/cs193 project flow diagram.png

## Tech stack
- **Language:** C++20, Python, CMake, bash
- **Libraries:** ROS 2 Jazzy, Gazebo Harmonic, Eigen3, and standard libraries C++ libraries like std

## Overview
The project has 4 main pieces:
 - **Simulation**: Gazebo Harmonic handles physics, collisions, robot model, lidar, odometry
 - **Crowd node**: ROS2 node that creates people and updates them with social force model to avoid each other while reaching their goal
 - **Controller node**: ROS2 node that takes in the gazebo lidar data and plans the path the robot should take
 - **Visualization**: gazebo is just a simulation. For proper visualization of what is going on, RViz2 is used. The ROS2 nodes send whatever data that needs to be displayed to RViz2

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
4. Build the workspace (from anywhere; the script finds `~/ros2_ws` relative to itself):
```bash
~/ros2_ws/src/cs193project/scripts/build_sim.sh
```
This runs `colcon build --packages-up-to crowd_sim --symlink-install`, so later edits to
the YAML, launch file and world template take effect on the next launch without rebuilding.
C++ changes still need a rebuild.

## Usage
All helpers live in `scripts/`. They source ROS and the workspace themselves.

| script | what it does |
|---|---|
| `scripts/run_sim.sh` | kills any leftover sim processes, then `ros2 launch crowd_sim sim.launch.py` (Gazebo, bridge, nodes, RViz) |
| `scripts/build_sim.sh` | builds `crowd_interfaces` + `crowd_sim`; extra args are passed to `colcon build` |
| `scripts/build_run_sim.sh` | build, then run |
| `scripts/kill_sim.sh` | stops Gazebo (including the server, which survives Ctrl-C), the bridge, the nodes and RViz |

Typical loop after a code change:
```bash
~/ros2_ws/src/cs193project/scripts/build_run_sim.sh
```

Optional: add short aliases to `~/.bashrc` so you can type `sim` instead of the full path:
```bash
alias sim='~/ros2_ws/src/cs193project/scripts/build_run_sim.sh'
alias simrun='~/ros2_ws/src/cs193project/scripts/run_sim.sh'
alias simkill='~/ros2_ws/src/cs193project/scripts/kill_sim.sh'
```

## Roadmap
1. build my robot SDF and make sure it drives in gazebo with proper inertia and collisions with basic terminal 2. command to set speed for testing
3. add a 2D lidar to the robot and build a world with walls and ground planes
4. create a ROS package with a launch file that starts the gazebo sim and opens RVis
5. create a pedestrian model and a node that controls the pedestrian to move in a straight line and avoid other pedestrians (social force model)
6. write the algorithm for the robot to pathfind around the moving people nodes
