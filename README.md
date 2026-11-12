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
| **Simulation** | Gazebo Harmonic | Physics, collisions, robot model, lidar, odometry |
| **Crowd node** | ROS 2 | Spawns pedestrians and moves them with a Social Force Model (goal attraction + person/wall/robot repulsion) |
| **Planner node** | ROS 2 | Reads sensor data and pedestrian states, publishes velocity commands to reach the goal safely |

## Setup
First need to download ROS2 and C and Gazebo
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- Ubuntu 24.04 (tested on WSL2)
- Gazebo Harmonic + ROS–Gazebo integration:
Make sure ROS is sourced in every terminal:
```bash
source /opt/ros/jazzy/setup.bash
```
can test everything is downloaded properly with "gz sim shapes.sdf" in the terminal

## Usage
run "gz sim simulation" to run the actual thing

## Roadmap
1. build my robot SDF and make sure it drives in gazebo with proper inertia and collisions with basic terminal 2. command to set speed for testing
3. add a 2D lidar to the robot and build a world with walls and ground planes
4. create a ROS package with a launch file that starts the gazebo sim and  opens RVis
5. create a pedestrian model and a node that controls the pedestrian to move in a straight line and avoid other pedestrians (social force model)
6. write the algorithm for the robot to pathfind around the moving people nodes
