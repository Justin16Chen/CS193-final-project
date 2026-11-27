#!/usr/bin/env bash
# Build crowd_interfaces + crowd_sim.
#
#   build_sim.sh          normal build, sanitizer explicitly OFF
#   build_sim.sh --asan   build with AddressSanitizer + UBSan (slow: ~3x compile time)
#
# The sanitizer flag is always passed explicitly so the CMake cache cannot keep
# a stale value from a previous build. Any other arguments go to colcon build.
#
# --symlink-install links config/, launch/, gazebo_sim/ into install/, so
# YAML, launch and template edits take effect on the next launch without a
# rebuild. C++ changes still need this script.
set -eo pipefail   # no -u: ROS setup.bash reads optional unset vars

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # scripts/ -> cs193project -> src -> ros2_ws

ASAN=OFF
ARGS=()
for arg in "$@"; do
  case "$arg" in
    --asan) ASAN=ON ;;
    *) ARGS+=("$arg") ;;
  esac
done

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR"
# --cmake-args must come last: colcon treats everything after it as a cmake argument.
colcon build --packages-up-to crowd_sim --symlink-install "${ARGS[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCROWD_SIM_ASAN="$ASAN"
