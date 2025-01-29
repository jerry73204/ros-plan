from typing import Text, List, Union, Optional
from dataclasses import dataclass

from launch.utilities import is_a
from launch.launch_description import LaunchDescription
from launch.launch_description_source import LaunchDescriptionSource
from launch.actions.opaque_function import OpaqueFunction
from launch.actions.group_action import GroupAction
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch_ros.actions.push_ros_namespace import PushRosNamespace
from launch_ros.actions.node import Node
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.load_composable_nodes import LoadComposableNodes
from launch_ros.actions.set_use_sim_time import SetUseSimTime

from .include_launch_description import IncludeLaunchDescriptionDump
from .substitution import serialize_text_or_substitution, TextOrSubstitutionExpr
from .node import NodeDump, serialize_node
from .declare_launch_argument import (
    serialize_declare_launch_argument,
)
from .group_action import GroupActionDump, serialize_group_action
from .set_launch_configuration import serialize_set_launch_configuration
from .set_use_sim_time import serialize_set_use_sim_time
from .declare_launch_argument import DeclareLaunchArgumentDump
from .set_launch_configuration import SetLaunchConfigurationDump


@dataclass
class LaunchDescriptionDump:
    file_path: Text
    push_namespace: Optional[TextOrSubstitutionExpr]
    use_sim_time: Optional[TextOrSubstitutionExpr]
    var_list: List[Union[DeclareLaunchArgumentDump, SetLaunchConfigurationDump]]
    node_list: List[NodeDump]
    group_list: List[GroupActionDump]
    include_list: List[IncludeLaunchDescriptionDump]


def serialize_launch_description(
    desc: LaunchDescription,
    source: LaunchDescriptionSource,
) -> LaunchDescriptionDump:
    file_path = source.location
    dump = LaunchDescriptionDump(
        file_path=file_path,
        push_namespace=None,
        use_sim_time=None,
        var_list=list(),
        group_list=list(),
        node_list=list(),
        include_list=list(),
    )
    print(file_path)

    for entity in desc.entities:
        # Check non-action cases
        if is_a(entity, PushRosNamespace):
            namespace = serialize_text_or_substitution(entity.namespace)

            if dump.push_namespace is None:
                dump.push_namespace = namespace
            else:
                raise ValueError("PushRosNamespace is done more than once")

        # Process the action
        elif is_a(entity, DeclareLaunchArgument):
            arg = serialize_declare_launch_argument(entity)
            dump.var_list.append(arg)

        elif is_a(entity, SetLaunchConfiguration):
            var = serialize_set_launch_configuration(entity)
            dump.var_list.append(var)

        elif is_a(entity, OpaqueFunction):
            # TODO
            pass

        elif is_a(entity, GroupAction):
            group = serialize_group_action(entity, dump)
            dump.group_list.append(group)

        elif is_a(entity, IncludeLaunchDescription):
            # Late import due to circular import
            from .include_launch_description import serialize_include_launch_description

            include = serialize_include_launch_description(entity)
            dump.include_list.append(include)

        elif is_a(entity, ComposableNodeContainer):
            # TODO
            pass

        elif is_a(entity, LoadComposableNodes):
            # TODO
            pass

        elif is_a(entity, Node):
            node = serialize_node(entity)
            dump.node_list.append(node)

        elif is_a(entity, SetUseSimTime):
            yes = serialize_set_use_sim_time(entity).value

            if dump.use_sim_time is not None:
                raise ValueError("SetUseSimTime is done more than once")

            dump.use_sim_time = yes

        else:
            # The following entity types are excluded
            # - PushLaunchConfigurations
            # - PopLaunchConfigurations
            # - PushEnvironment
            # - PushRosNamespace
            # - PopEnvironment
            # - SetParameter
            # - SetParametersFromFile
            # - SetRemap
            raise ValueError(f"unknown entity type; {type(entity)}")

    return dump
