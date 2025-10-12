"""Launch file with a talker node for testing link resolution."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with a single talker node."""
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='launch_talker',
            parameters=[{'use_sim_time': False}],
        ),
    ])
