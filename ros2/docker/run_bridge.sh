#!/bin/sh
# run_bridge.sh - build and (re)start the so101_bridge container on the Pi.
#   ./ros2/docker/run_bridge.sh            (from the repo root, with so101d running)
# --net host : DDS discovery on the LAN     --ipc host : sees /dev/shm/so101 + semaphores
set -e
cd "$(dirname "$0")/../.."
docker build -t so101_bridge -f ros2/docker/Dockerfile .
docker rm -f so101_bridge >/dev/null 2>&1 || true
docker run -d --name so101_bridge --restart unless-stopped \
    --net host --ipc host \
    -v /etc/so101:/etc/so101:ro \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    so101_bridge
echo "logs: docker logs -f so101_bridge"
