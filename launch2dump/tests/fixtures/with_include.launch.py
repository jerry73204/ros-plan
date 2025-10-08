"""Launch file that includes another launch file."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with include."""
    fixtures_dir = Path(__file__).parent

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(fixtures_dir / "included_base.launch.py"))
            ),
            Node(
                package="demo_nodes_cpp",
                executable="listener",
                name="main_listener",
                output="screen",
            ),
        ]
    )
