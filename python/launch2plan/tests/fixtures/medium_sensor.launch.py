"""Sensor launch file for medium hierarchy testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="/dev/video0"),
            DeclareLaunchArgument("fps", default_value="30"),
            Node(
                package="camera_pkg",
                executable="camera_node",
                name="camera",
                parameters=[
                    {
                        "device": LaunchConfiguration("device"),
                        "fps": LaunchConfiguration("fps"),
                    }
                ],
            ),
        ]
    )
