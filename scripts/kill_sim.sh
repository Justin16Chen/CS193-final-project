#!/usr/bin/env bash
# Stop every process from a crowd_sim launch, including the Gazebo server,
# which survives Ctrl-C and otherwise leaves the next launch with a white
# Gazebo window and a frozen /clock.
#
# Patterns are anchored to the start of the command line so they only match
# the real executables, never a shell or editor whose arguments mention them.
# -9 because the stuck gz server ignores SIGTERM.

pkill -9 -f -e '^ruby [^ ]*/gz sim'                       || true  # gz sim wrapper / GUI
pkill -9 -f -e '^gz sim'                                  || true  # gz sim server
pkill -9 -f -e '^[^ ]*/crowd_sim/lib/crowd_sim/'          || true  # crowd, controller nodes
pkill -9 -f -e '^[^ ]*/parameter_bridge'                  || true  # ros_gz_bridge
pkill -9 -f -e '^[^ ]*/rviz2'                             || true
pkill -9 -f -e '^[^ ]*python3[^ ]* [^ ]*/ros2 launch crowd_sim' || true

sleep 1   # let the gz server release its transport ports
exit 0
