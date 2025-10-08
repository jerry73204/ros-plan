"""Launch file with node parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with parameters."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_rate",
                default_value="10",
                description="Publishing rate in Hz",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="talker",
                parameters=[
                    {
                        "use_sim_time": False,
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "qos.depth": 10,
                    }
                ],
                output="screen",
            ),
        ]
    )
