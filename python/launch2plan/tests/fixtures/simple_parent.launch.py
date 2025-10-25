"""Simple parent launch file for integration testing."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get path to included file
    current_dir = Path(__file__).parent
    included_file = current_dir / "simple_included.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_device", default_value="/dev/video0", description="Sensor device path"
            ),
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="talker",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(included_file)),
                launch_arguments={"device": LaunchConfiguration("sensor_device")}.items(),
            ),
        ]
    )
