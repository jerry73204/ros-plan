"""Simple launch file with a single talker node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with a single talker node."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "node_name",
                default_value="talker",
                description="Name of the talker node",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name=LaunchConfiguration("node_name"),
                output="screen",
            ),
        ]
    )
