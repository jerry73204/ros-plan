"""
Substitution tracker for detecting parameter dependencies
"""

from typing import Any

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.parameter_descriptions import ParameterFile

from .session import CollectionSession


def track_substitutions_in_value(value: Any, session: CollectionSession) -> None:
    """
    Recursively track LaunchConfiguration substitutions in a value.

    :param value: The value to scan for substitutions
    :param session: The collection session to record dependencies
    """
    if isinstance(value, ParameterFile):
        # ParameterFile wraps substitutions in __param_file
        if hasattr(value, "_ParameterFile__param_file"):
            track_substitutions_in_value(value._ParameterFile__param_file, session)
    elif isinstance(value, IfCondition):
        # IfCondition has a private __predicate_expression attribute
        if hasattr(value, "_IfCondition__predicate_expression"):
            track_substitutions_in_value(value._IfCondition__predicate_expression, session)
    elif isinstance(value, PathJoinSubstitution):
        # PathJoinSubstitution contains nested substitutions
        if hasattr(value, "substitutions"):
            for sub in value.substitutions:
                track_substitutions_in_value(sub, session)
    elif isinstance(value, LaunchConfiguration):
        # Found a parameter dependency
        # variable_name can be a list of substitutions or a string
        if isinstance(value.variable_name, (list, tuple)):
            for item in value.variable_name:
                if isinstance(item, str):
                    session.add_parameter_dependency(item)
                elif isinstance(item, TextSubstitution):
                    # TextSubstitution has a 'text' attribute
                    if hasattr(item, "text"):
                        session.add_parameter_dependency(item.text)
                else:
                    track_substitutions_in_value(item, session)
        elif isinstance(value.variable_name, str):
            session.add_parameter_dependency(value.variable_name)
        else:
            track_substitutions_in_value(value.variable_name, session)
    elif isinstance(value, dict):
        # Recursively track in dictionary values and keys
        for k, v in value.items():
            track_substitutions_in_value(k, session)
            track_substitutions_in_value(v, session)
    elif isinstance(value, (list, tuple)):
        # Recursively track in lists/tuples
        for item in value:
            track_substitutions_in_value(item, session)
    elif hasattr(value, "__iter__") and not isinstance(value, (str, bytes)):
        # Recursively track in other iterables
        try:
            for item in value:
                track_substitutions_in_value(item, session)
        except TypeError:
            pass
