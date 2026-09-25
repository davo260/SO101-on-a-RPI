#!/bin/sh
# run_bridge.sh - build and (re)start the so101_bridge container on the Pi.
#   ./ros2/docker/run_bridge.sh                 Zenoh (default): router on tcp/7447
#   RMW=rmw_fastrtps_cpp ./ros2/docker/run_bridge.sh    classic DDS (same LAN, multicast)
# --net host : router/DDS reachable from the LAN   --ipc host : sees /dev/shm/so101
set -e
cd "$(dirname "$0")/../.."
docker build -t so101_bridge -f ros2/docker/Dockerfile .
docker rm -f so101_bridge >/dev/null 2>&1 || true
docker run -d --name so101_bridge --restart unless-stopped \
    --net host --ipc host \
    -v /etc/so101:/etc/so101:ro \
    -e RMW_IMPLEMENTATION="${RMW:-rmw_zenoh_cpp}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    so101_bridge
echo "logs: docker logs -f so101_bridge"
