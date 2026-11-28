#!/usr/bin/env bash
# Snapshot the planner cost grid to crowd_sim/tuning/. Any arguments are
# passed through to plot_costs.py (for example --surface or -o path.png).
set -eo pipefail   # no -u: ROS setup.bash reads optional unset vars

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # scripts/ -> cs193project -> src -> ros2_ws

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
  echo "install/setup.bash not found: run $SCRIPT_DIR/build_sim.sh first" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

exec python3 "$SCRIPT_DIR/plot_costs.py" "$@"