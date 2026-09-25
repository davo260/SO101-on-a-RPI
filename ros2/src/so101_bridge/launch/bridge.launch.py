"""Run on the Pi (or its container):  ros2 launch so101_bridge bridge.launch.py"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("so101_bridge"), "config", "bridge.yaml")
    return LaunchDescription([
        Node(package="so101_bridge", executable="bridge", name="so101_bridge",
             parameters=[cfg], output="screen"),
    ])
