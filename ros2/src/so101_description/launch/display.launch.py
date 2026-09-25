"""RViz digital twin: follower (solid) + leader (next to it), both driven by so101_bridge.

    ros2 launch so101_description display.launch.py              # live, from the Pi
    ros2 launch so101_description display.launch.py gui:=true    # sliders, no hardware
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def urdf(name):
    path = os.path.join(get_package_share_directory("so101_description"), "urdf", name)
    with open(path) as f:
        return f.read()


def generate_launch_description():
    share = get_package_share_directory("so101_description")
    gui = LaunchConfiguration("gui")
    leader = LaunchConfiguration("leader")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="joint_state_publisher_gui sliders instead of the robot"),
        DeclareLaunchArgument("leader", default_value="true",
                              description="also show the leader arm (0.35 m to the side)"),
        # follower: /joint_states -> TF base_link...
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="follower_state_publisher",
             parameters=[{"robot_description": urdf("so101_follower.urdf")}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--frame-id", "world", "--child-frame-id", "base_link"]),
        # leader: /so101/leader/joint_states -> TF leader/base_link...
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="leader_state_publisher", namespace="so101/leader",
             condition=IfCondition(leader),
             parameters=[{"robot_description": urdf("so101_leader.urdf"),
                          "frame_prefix": "leader/"}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             condition=IfCondition(leader),
             arguments=["--y", "0.35", "--frame-id", "world",
                        "--child-frame-id", "leader/base_link"]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(gui),
             parameters=[{"robot_description": urdf("so101_follower.urdf")}]),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "so101.rviz")]),
    ])
