#!/bin/bash
# start.sh - container main process.
# With Zenoh (default) it first starts the Zenoh router on tcp/7447: remote
# machines (the Mac VM) connect to it with ONE outgoing TCP connection, which
# works through NAT and on networks that block multicast (campus Wi-Fi).
set -e
if [ "${RMW_IMPLEMENTATION}" = "rmw_zenoh_cpp" ]; then
    ros2 run rmw_zenoh_cpp rmw_zenohd &
    sleep 2
fi
exec ros2 launch so101_bridge bridge.launch.py
