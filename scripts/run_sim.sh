#!/usr/bin/env bash
# Kill any leftover sim processes, then launch the crowd simulation.
set -eo pipefail   # no -u: ROS setup.bash reads optional unset vars

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # scripts/ -> cs193project -> src -> ros2_ws

"$SCRIPT_DIR/kill_sim.sh"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR"
if [[ ! -f install/setup.bash ]]; then
  echo "install/setup.bash not found: run $SCRIPT_DIR/build_sim.sh first" >&2
  exit 1
fi
# shellcheck disable=SC1091
source install/setup.bash

exec ros2 launch crowd_sim sim.launch.py
