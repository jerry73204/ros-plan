from typing import Text, List, Optional
from dataclasses import dataclass

from launch.actions.declare_launch_argument import DeclareLaunchArgument
from .substitution import serialize_text_or_substitution, TextOrSubstitutionExpr


@dataclass
class DeclareLaunchArgumentDump:
    name: Text
    default_value: Optional[TextOrSubstitutionExpr]
    description: Optional[Text]
    choices: Optional[List[Text]]


def serialize_declare_launch_argument(
    entity: DeclareLaunchArgument,
) -> DeclareLaunchArgumentDump:
    name = entity.name

    default_value = None
    if entity.default_value is not None:
        default_value = serialize_text_or_substitution(entity.default_value)

    description = entity.description
    choices = entity.choices

    return DeclareLaunchArgumentDump(name, default_value, description, choices)
