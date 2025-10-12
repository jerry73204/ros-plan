"""Test launch file with launch arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with configurable parameters."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='configurable_node',
            description='Name of the node',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='10',
            description='Publishing rate',
        ),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name=LaunchConfiguration('node_name'),
            parameters=[{
                'publish_rate': LaunchConfiguration('rate'),
            }],
        ),
    ])
