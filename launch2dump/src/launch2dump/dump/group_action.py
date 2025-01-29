from dataclasses import dataclass
from typing import List, Optional, Union

from launch.utilities import is_a
from launch.actions.group_action import GroupAction
from launch.actions.push_launch_configurations import PushLaunchConfigurations
from launch.actions.pop_launch_configurations import PopLaunchConfigurations
from launch.actions.reset_launch_configurations import ResetLaunchConfigurations
from launch.actions.push_environment import PushEnvironment
from launch.actions.pop_environment import PopEnvironment
from launch.actions.opaque_function import OpaqueFunction
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions.reset_environment import ResetEnvironment
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch_ros.actions.set_parameter import SetParameter
from launch_ros.actions.set_parameters_from_file import SetParametersFromFile
from launch_ros.actions.set_remap import SetRemap
from launch_ros.actions.push_ros_namespace import PushRosNamespace
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.load_composable_nodes import LoadComposableNodes
from launch_ros.actions.node import Node
from launch_ros.actions.set_use_sim_time import SetUseSimTime

from .substitution import serialize_text_or_substitution, TextOrSubstitutionExpr
from .node import NodeDump, serialize_node
from .declare_launch_argument import (
    serialize_declare_launch_argument,
    DeclareLaunchArgumentDump,
)
from .set_launch_configuration import (
    serialize_set_launch_configuration,
    SetLaunchConfigurationDump,
)
from .include_launch_description import IncludeLaunchDescriptionDump


@dataclass
class GroupActionDump:
    scoped: bool
    forwarding: bool
    push_namespace: Optional[TextOrSubstitutionExpr]
    var_list: List[Union[DeclareLaunchArgumentDump, SetLaunchConfigurationDump]]
    node_list: List[NodeDump]
    include_list: List[IncludeLaunchDescriptionDump]
    group_list: List["GroupActionDump"]


def serialize_group_action(
    group: GroupAction,
    dump: "LaunchDescriptionDump",
) -> GroupActionDump:
    all_entities = group.get_sub_entities()

    # Determine "scoped" and "forwarding" properties
    if len(all_entities) >= 4:
        if is_a(all_entities[0], PushLaunchConfigurations):
            assert is_a(all_entities[1], PushEnvironment)
            assert is_a(all_entities[-2], PopEnvironment)
            assert is_a(all_entities[-1], PopLaunchConfigurations)
            scoped = True

            if is_a(all_entities[2], ResetEnvironment):
                assert is_a(all_entities[3], ResetLaunchConfigurations)
                forwarding = False
                action_entities = all_entities[4:-2]
            else:
                forwarding = True
                action_entities = all_entities[2:-2]

        else:
            scoped = False
            forwarding = False
            action_entities = all_entities
    else:
        scoped = False
        forwarding = False
        action_entities = all_entities

    # Process action entities
    push_namespace = None
    var_list = list()
    node_list = list()
    group_list = list()
    include_list = list()

    for entity in action_entities:
        from .entity import serialize_action  # Due to circular import

        # Check actions that are specific to groups
        if is_a(entity, SetParameter):
            # TODO
            pass

        elif is_a(entity, SetParametersFromFile):
            # TODO
            pass

        elif is_a(entity, SetRemap):
            # TODO
            pass

        elif is_a(entity, PushRosNamespace):
            if push_namespace is not None:
                raise ValueError("PushRosNamespace is done more than once")
            if not scoped:
                raise ValueError(
                    "PushRosNamespace in non-scoped group is not supported"
                )
            push_namespace = serialize_text_or_substitution(entity.namespace)

        elif is_a(entity, DeclareLaunchArgument):
            arg = serialize_declare_launch_argument(entity)
            var_list.append(arg)

        elif is_a(entity, SetLaunchConfiguration):
            var = serialize_set_launch_configuration(entity)
            var_list.append(var)

        elif is_a(entity, OpaqueFunction):
            # TODO
            pass

        elif is_a(entity, GroupAction):
            group = serialize_group_action(entity, dump)
            group_list.append(group)

        elif is_a(entity, IncludeLaunchDescription):
            # Late import due to circular import
            from .include_launch_description import serialize_include_launch_description

            include = serialize_include_launch_description(entity)
            include_list.append(include)

        elif is_a(entity, ComposableNodeContainer):
            # TODO
            pass

        elif is_a(entity, LoadComposableNodes):
            # TODO
            pass

        elif is_a(entity, Node):
            node = serialize_node(entity)
            node_list.append(node)

        elif is_a(entity, SetUseSimTime):
            raise ValueError("SetUseSimTime in group is not supported")

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

    return GroupActionDump(
        scoped=scoped,
        forwarding=forwarding,
        push_namespace=push_namespace,
        var_list=var_list,
        node_list=node_list,
        group_list=group_list,
        include_list=include_list,
    )
