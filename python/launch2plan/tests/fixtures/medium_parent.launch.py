"""Medium hierarchy parent launch file for integration testing."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    current_dir = Path(__file__).parent
    driver_file = current_dir / "medium_driver.launch.py"
    sensor_file = current_dir / "medium_sensor.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument("front_camera", default_value="/dev/video0"),
            DeclareLaunchArgument("rear_camera", default_value="/dev/video1"),
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="main_node",
            ),
            # Include driver which includes sensor
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(driver_file)),
                launch_arguments={
                    "camera_device": LaunchConfiguration("front_camera"),
                    "camera_fps": "30",
                }.items(),
            ),
            # Include sensor directly (should be deduplicated)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sensor_file)),
                launch_arguments={
                    "device": LaunchConfiguration("rear_camera"),
                    "fps": "15",
                }.items(),
            ),
        ]
    )
