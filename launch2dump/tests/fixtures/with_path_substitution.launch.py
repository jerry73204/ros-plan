"""Launch file with nested path substitutions for testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with path join substitution."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_dir", default_value="/opt/config", description="Config directory"
            ),
            DeclareLaunchArgument(
                "config_file", default_value="params.yaml", description="Config filename"
            ),
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="talker",
                parameters=[
                    PathJoinSubstitution(
                        [LaunchConfiguration("config_dir"), LaunchConfiguration("config_file")]
                    )
                ],
            ),
        ]
    )
