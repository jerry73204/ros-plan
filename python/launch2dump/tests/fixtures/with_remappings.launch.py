"""Launch file with topic remappings."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with remappings."""
    return LaunchDescription(
        [
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="talker",
                remappings=[
                    ("chatter", "/my_topic"),
                    ("/rosout", "/custom_rosout"),
                ],
                output="screen",
            ),
        ]
    )
