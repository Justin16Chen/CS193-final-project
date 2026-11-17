#!/usr/bin/env bash
# Kill any running sim, build, then launch. Extra arguments are passed to colcon build.
set -eo pipefail   # no -u: ROS setup.bash reads optional unset vars
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/kill_sim.sh"
"$SCRIPT_DIR/build_sim.sh" "$@"
exec "$SCRIPT_DIR/run_sim.sh"
