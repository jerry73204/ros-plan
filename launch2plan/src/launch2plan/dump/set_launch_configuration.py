from dataclasses import dataclass

from launch.actions.set_launch_configuration import SetLaunchConfiguration

from .substitution import serialize_text_or_substitution, TextOrSubstitutionExpr


@dataclass
class SetLaunchConfigurationDump:
    name: TextOrSubstitutionExpr
    value: TextOrSubstitutionExpr


def serialize_set_launch_configuration(
    action: SetLaunchConfiguration,
) -> SetLaunchConfigurationDump:
    name = serialize_text_or_substitution(action.name)
    value = serialize_text_or_substitution(action.value)
    return SetLaunchConfigurationDump(name=name, value=value)
