"""Simple included launch file for integration testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device", default_value="/dev/ttyUSB0", description="Device path"
            ),
            Node(
                package="sensor_pkg",
                executable="sensor_node",
                name="sensor",
                parameters=[{"device": LaunchConfiguration("device")}],
            ),
        ]
    )
