from dataclasses import dataclass
from typing import Text, List

from launch.actions.include_launch_description import IncludeLaunchDescription

from .substitution import TextOrSubstitutionExpr, serialize_text_or_substitution


@dataclass
class ArgAssignDump:
    name: TextOrSubstitutionExpr
    value: TextOrSubstitutionExpr


@dataclass
class IncludeLaunchDescriptionDump:
    file_path: Text
    args: List[ArgAssignDump]


def serialize_include_launch_description(
    include: IncludeLaunchDescription,
) -> IncludeLaunchDescriptionDump:
    source = include.launch_description_source
    file_path = source.location

    args = list()

    for name, value in include.launch_arguments:
        name = serialize_text_or_substitution(name)
        value = serialize_text_or_substitution(value)
        args.append(ArgAssignDump(name=name, value=value))

    return IncludeLaunchDescriptionDump(file_path=file_path, args=args)
