"""Driver launch file for medium hierarchy testing."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    current_dir = Path(__file__).parent
    sensor_file = current_dir / "medium_sensor.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_fps", default_value="30"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sensor_file)),
                launch_arguments={
                    "device": LaunchConfiguration("camera_device"),
                    "fps": LaunchConfiguration("camera_fps"),
                }.items(),
            ),
        ]
    )
