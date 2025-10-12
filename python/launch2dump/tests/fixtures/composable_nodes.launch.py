"""Launch file with composable node container."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with composable nodes."""
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="my_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="composition",
                        plugin="composition::Talker",
                        name="talker_component",
                        parameters=[{"use_sim_time": False}],
                    ),
                    ComposableNode(
                        package="composition",
                        plugin="composition::Listener",
                        name="listener_component",
                        remappings=[("chatter", "/my_chatter")],
                    ),
                ],
                output="screen",
            )
        ]
    )
