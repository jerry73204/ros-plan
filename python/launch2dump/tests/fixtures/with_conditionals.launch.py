"""Launch file with conditionals."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with conditionals."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_talker",
                default_value="true",
                description="Whether to start the talker node",
            ),
            DeclareLaunchArgument(
                "use_listener",
                default_value="false",
                description="Whether to start the listener node",
            ),
            GroupAction(
                [
                    Node(
                        package="demo_nodes_cpp",
                        executable="talker",
                        name="talker",
                        output="screen",
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("use_talker")),
            ),
            GroupAction(
                [
                    Node(
                        package="demo_nodes_cpp",
                        executable="listener",
                        name="listener",
                        output="screen",
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("use_listener")),
            ),
        ]
    )
