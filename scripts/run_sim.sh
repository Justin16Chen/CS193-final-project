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

# Only has an effect on a sanitizer build (build_sim.sh --asan). Leak detection
# is off because rclcpp's shutdown path reports leaks that are not ours; the
# sanitizer report goes to stderr, under the [controller-N] launch prefix.
export ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}"
export UBSAN_OPTIONS="${UBSAN_OPTIONS:-print_stacktrace=1}"

exec ros2 launch crowd_sim sim.launch.py
