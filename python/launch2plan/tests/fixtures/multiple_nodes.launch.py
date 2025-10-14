"""Launch file with multiple nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="talker",
                remappings=[("chatter", "/demo/chatter")],
            ),
            Node(
                package="demo_nodes_cpp",
                executable="listener",
                name="listener",
                remappings=[("chatter", "/demo/chatter")],
            ),
        ]
    )
