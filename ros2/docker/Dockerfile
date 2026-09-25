# ROS 2 Jazzy container for so101_bridge on the Raspberry Pi (arm64).
# Build from the REPO ROOT:  docker build -t so101_bridge -f ros2/docker/Dockerfile .
FROM ros:jazzy-ros-base
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pip ros-jazzy-rmw-zenoh-cpp \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir --break-system-packages posix_ipc
COPY clients/python/so101.py /opt/so101/so101.py
ENV PYTHONPATH=/opt/so101
WORKDIR /ws
COPY ros2/src/so101_bridge src/so101_bridge
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select so101_bridge
COPY ros2/docker/entrypoint.sh /entrypoint.sh
COPY ros2/docker/start.sh /start.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start.sh"]
