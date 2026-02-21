"""
ROS2 Launch 文件 —— 一键启动所有节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ensoul",
            executable="g1_awakening_fsm",
            name="g1_awakening_fsm",
            output="screen",
            parameters=["config/params.yaml"],
        ),
        Node(
            package="ensoul",
            executable="wipe_controller",
            name="wipe_controller",
            output="screen",
        ),
        Node(
            package="ensoul",
            executable="tts_service",
            name="tts_service",
            output="screen",
        ),
    ])
