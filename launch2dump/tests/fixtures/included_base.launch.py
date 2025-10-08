"""Base launch file to be included by others."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with a base node."""
    return LaunchDescription(
        [
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="included_talker",
                output="screen",
            ),
        ]
    )
