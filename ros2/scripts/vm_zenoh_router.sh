#!/bin/bash
# vm_zenoh_router.sh - run IN THE VM (terminal 1): a local Zenoh router that
# connects to the router in the Pi's container. VM nodes (RViz, robot_state_publisher,
# ros2 topic ...) talk to this local router; it forwards over ONE TCP connection
# to the Pi, so it works with Parallels "Shared" (NAT) and without multicast.
#   ./vm_zenoh_router.sh [pi_ip]          (default 10.195.22.101)
PI_IP=${1:-10.195.22.101}
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/${PI_IP}:7447\"]"
echo "Zenoh router: local tcp/7447 <-> Pi tcp/${PI_IP}:7447  (Ctrl+C to stop)"
exec ros2 run rmw_zenoh_cpp rmw_zenohd
