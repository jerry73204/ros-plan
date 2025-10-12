"""Simple test launch file with a single node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a simple launch description with one node."""
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='test_talker',
            namespace='/test',
            parameters=[{
                'use_sim_time': False,
                'publish_rate': 10,
            }],
            remappings=[
                ('/chatter', '/test_topic'),
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
