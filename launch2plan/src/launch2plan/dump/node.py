from dataclasses import dataclass
from typing import List, Optional

from launch.utilities import is_a
from launch_ros.actions.node import Node
from launch_ros.descriptions import ParameterFile

from .substitution import (
    TextOrSubstitutionExpr,
    serialize_text_or_substitution,
)
from .parameter import (
    ParameterFileDump,
    ParameterValueOrSubstitutionExpr,
    serialize_parameter_value,
    serialize_parameters_file,
)


@dataclass
class RemapEntryDump:
    orig: TextOrSubstitutionExpr
    new: TextOrSubstitutionExpr


@dataclass
class ParameterEntryDump:
    name: TextOrSubstitutionExpr
    value: ParameterValueOrSubstitutionExpr


@dataclass
class NodeDump:
    executable: TextOrSubstitutionExpr
    package: TextOrSubstitutionExpr
    name: Optional[TextOrSubstitutionExpr]
    namespace: Optional[TextOrSubstitutionExpr]
    params: List[ParameterEntryDump]
    params_files: List[ParameterFileDump]
    remaps: List[RemapEntryDump]
    ros_args: Optional[List[TextOrSubstitutionExpr]]
    args: Optional[List[TextOrSubstitutionExpr]]


def serialize_node(node: Node) -> NodeDump:
    executable = serialize_text_or_substitution(node._Node__node_executable)
    package = serialize_text_or_substitution(node._Node__package)

    name = None
    if node._Node__node_name is not None:
        name = serialize_text_or_substitution(node._Node__node_name)

    namespace = None
    if node._Node__node_namespace is not None:
        namespace = serialize_text_or_substitution(node._Node__node_namespace)

    ros_args = None
    if node._Node__ros_arguments is not None:
        ros_args = [
            serialize_text_or_substitution(subst) for subst in node._Node__ros_arguments
        ]

    args = None
    if node._Node__arguments is not None:
        args = [
            serialize_text_or_substitution(subst) for subst in node._Node__arguments
        ]

    # Extract parameters
    params = list()
    params_files = list()

    for entry in node._Node__parameters:
        if is_a(entry, ParameterFile):
            params_file = serialize_parameters_file(entry)
            params_files.append(params_file)
        else:
            assert is_a(entry, dict)
            for param_name, value in entry.items():
                param_name = serialize_text_or_substitution(param_name)
                param_value = serialize_parameter_value(value)
                params.append(ParameterEntryDump(name=param_name, value=param_value))

    remaps = list()
    for orig, new in node._Node__remappings:
        orig = serialize_text_or_substitution(orig)
        new = serialize_text_or_substitution(new)
        remaps.append(RemapEntryDump(orig=orig, new=new))

    # Store a node record
    dump = NodeDump(
        executable=executable,
        package=package,
        name=name,
        namespace=namespace,
        remaps=remaps,
        params=params,
        params_files=params_files,
        ros_args=ros_args,
        args=args,
    )
    return dump
