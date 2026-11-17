#!/usr/bin/env bash
# Build crowd_interfaces + crowd_sim.
# --symlink-install links config/, launch/, gazebo_sim/ into install/, so
# YAML, launch and template edits take effect on the next launch without a
# rebuild. C++ changes still need this script.
set -eo pipefail   # no -u: ROS setup.bash reads optional unset vars

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # scripts/ -> cs193project -> src -> ros2_ws

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR"
colcon build --packages-up-to crowd_sim --symlink-install "$@"
