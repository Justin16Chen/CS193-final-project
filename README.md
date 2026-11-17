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
scripts/                 build / run / kill helpers (see Usage)
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

If Gazebo opens as a white window, or the agents in RViz do not move, a Gazebo server from a
previous run is still alive. `run_sim.sh` already clears it; to do it by hand:
```bash
~/ros2_ws/src/cs193project/scripts/kill_sim.sh
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
4. create a ROS package with a launch file that starts the gazebo sim and  opens RVis
5. create a pedestrian model and a node that controls the pedestrian to move in a straight line and avoid other pedestrians (social force model)
6. write the algorithm for the robot to pathfind around the moving people nodes
