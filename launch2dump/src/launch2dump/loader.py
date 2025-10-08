"""
Launch file loader - extracts node metadata without spawning processes
"""

from typing import Dict, Optional

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

from .inspector import LaunchInspector
from .result import LaunchResult


class LaunchLoader:
    """Loads ROS 2 launch files and extracts node metadata."""

    def __init__(self):
        """Create a LaunchLoader instance."""
        pass

    def load_launch_file(
        self,
        launch_file_path: str,
        launch_arguments: Optional[Dict[str, str]] = None,
    ) -> LaunchResult:
        """
        Load a launch file and extract all nodes without spawning processes.

        :param launch_file_path: Path to .launch.py file
        :param launch_arguments: Arguments to pass to launch file (key-value pairs)
        :return: LaunchResult containing nodes, containers, errors, and dependencies
        """
        if launch_arguments is None:
            launch_arguments = {}

        # Convert arguments dict to list of (name, value) tuples
        launch_args = list(launch_arguments.items())

        # Prepare argv for launch arguments
        argv = [f"{name}:={value}" for name, value in launch_args]

        # Create inspector
        inspector = LaunchInspector(argv=argv, noninteractive=True, debug=False)

        # Run the inspector (visits all entities without spawning processes)
        try:
            # Create launch description that includes the target launch file
            launch_description = LaunchDescription(
                [
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(launch_file_path),
                        launch_arguments=launch_args,
                    )
                ]
            )

            # Include the launch description
            inspector.include_launch_description(launch_description)

            # Run the inspector
            inspector.run(shutdown_when_idle=True)
        except Exception as e:
            # Collect error
            inspector.session.add_error(f"Error loading launch file: {str(e)}")

        # Extract results from session
        session = inspector.session

        return LaunchResult(
            nodes=session.nodes,
            containers=session.containers,
            lifecycle_nodes=session.lifecycle_nodes,
            errors=session.errors,
            parameter_dependencies=session.parameter_dependencies,
        )
