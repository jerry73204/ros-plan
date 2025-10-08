"""Launch file with multiple nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple nodes."""
    return LaunchDescription(
        [
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="talker",
                output="screen",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="listener",
                name="listener",
                output="screen",
            ),
            Node(
                package="demo_nodes_py",
                executable="talker",
                name="py_talker",
                namespace="/python",
                output="screen",
            ),
        ]
    )
