"""Test launch file with multiple nodes and parameters."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple nodes."""
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node',
            parameters=[{
                'publish_rate': 5,
            }],
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener_node',
        ),
    ])
