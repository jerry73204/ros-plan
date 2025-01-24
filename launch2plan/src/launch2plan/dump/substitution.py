from dataclasses import dataclass
from enum import Enum
from typing import List, Union, Text, Optional, List, Tuple

from launch.utilities import is_a
from launch.substitution import Substitution
from launch.substitutions.text_substitution import TextSubstitution
from launch.substitutions.environment_variable import EnvironmentVariable
from launch.substitutions.anon_name import AnonName
from launch.substitutions.boolean_substitution import (
    NotSubstitution,
    AndSubstitution,
    OrSubstitution,
)
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.substitutions.local_substitution import LocalSubstitution
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.substitutions.python_expression import PythonExpression
from launch.substitutions.this_launch_file_dir import ThisLaunchFileDir
from launch.substitutions.this_launch_file import ThisLaunchFile
from launch_ros.substitutions.find_package import FindPackageShare
from launch_ros.substitutions.parameter import Parameter
from launch_ros.substitutions.executable_in_package import ExecutableInPackage


class SubstitutionTokenKind(Enum):
    TEXT = "text"
    ENV = "env"
    ANON = "anon"
    NOT = "not"
    AND = "and"
    OR = "or"
    COMMAND = "command"
    FIND_EXEC = "find_exec"
    VAR = "var"
    EVAL = "eval"
    PATH_JOIN = "path_join"
    LOCAL = "local"
    FILENAME = "filename"
    DIRNAME = "dirname"
    FIND_PKG_SHARE = "find_pkg_share"
    PARAM = "param"
    EXEC_IN_PKG = "exec_in_pkg"


@dataclass
class TextToken:
    text: Text
    kind: Text = "text"


@dataclass
class EnvironmentVariableToken:
    name: "TextOrSubstitutionExpr"
    default_value: Optional["TextOrSubstitutionExpr"]
    kind: Text = "env"


@dataclass
class AnonNameToken:
    name: "TextOrSubstitutionExpr"
    kind: Text = "anon"


@dataclass
class AndToken:
    lhs: "TextOrSubstitutionExpr"
    rhs: "TextOrSubstitutionExpr"
    kind: Text = "and"


@dataclass
class OrToken:
    lhs: "TextOrSubstitutionExpr"
    rhs: "TextOrSubstitutionExpr"
    kind: Text = "or"


@dataclass
class NotToken:
    value: "TextOrSubstitutionExpr"
    kind: Text = "not"


@dataclass
class CommandToken:
    command: "TextOrSubstitutionExpr"
    on_stderr: Optional["TextOrSubstitutionExpr"]
    kind: Text = "command"


@dataclass
class FindExecutableToken:
    name: "TextOrSubstitutionExpr"
    kind: Text = "find_exec"


@dataclass
class LaunchConfigurationToken:
    variable_name: "TextOrSubstitutionExpr"
    kind: Text = "var"


@dataclass
class LocalToken:
    expression: "TextOrSubstitutionExpr"
    description: Optional["TextOrSubstitutionExpr"]
    kind: Text = "local"


@dataclass
class PathJoinToken:
    substitutions: List["SubstitutionToken"]
    kind: Text = "path_join"


@dataclass
class PythonExpressionToken:
    expression: "TextOrSubstitutionExpr"
    kind: Text = "eval"


@dataclass
class ThisLaunchFileDirToken:
    kind: Text = "dirname"


@dataclass
class ThisLaunchFileToken:
    kind: Text = "filename"


@dataclass
class FindPackageShareToken:
    package: "TextOrSubstitutionExpr"
    kind: Text = "find_pkg_share"


@dataclass
class ParameterToken:
    name: "TextOrSubstitutionExpr"
    kind: Text = "param"


@dataclass
class ExecutableInPackageToken:
    executable: "TextOrSubstitutionExpr"
    kind: Text = "exec_in_pkg"


SubstitutionToken = Union[
    TextToken,
    EnvironmentVariableToken,
    AnonNameToken,
    AndToken,
    OrToken,
    NotToken,
    CommandToken,
    FindExecutableToken,
    LaunchConfigurationToken,
    LocalToken,
    PathJoinToken,
    PythonExpressionToken,
    ThisLaunchFileToken,
    ThisLaunchFileDirToken,
    ParameterToken,
    ExecutableInPackageToken,
]


SubstitutionExpr = List[SubstitutionToken]
TextOrSubstitutionExpr = Union[Text, SubstitutionExpr]


def serialize_text_or_substitution(
    text_or_subst: Union[Text, List[Substitution], Tuple[Substitution]]
) -> TextOrSubstitutionExpr:
    if is_a(text_or_subst, str):
        return text_or_subst
    else:
        assert is_a(text_or_subst, list) or is_a(text_or_subst, tuple)
        assert len(text_or_subst) >= 1

        is_pure_text = all(is_a(token, TextSubstitution) for token in text_or_subst)

        if is_pure_text:
            return "".join(token.text for token in text_or_subst)
        else:
            return serialize_substitution(text_or_subst)


def serialize_parameter_value_or_substitution(
    value_or_subst: Union[bool, int, float, Text, List[Substitution]]
) -> "ParameterValueOrSubstitutionExpr":
    if is_a(value_or_subst, int):
        return value_or_subst

    elif is_a(value_or_subst, bool):
        return value_or_subst

    elif is_a(value_or_subst, float):
        return value_or_subst

    elif is_a(value_or_subst, str):
        return value_or_subst

    else:
        assert is_a(value_or_subst, list)

        if len(value_or_subst) == 0:
            return list()

        first = value_or_subst[0]

        if is_a(first, int):
            return value_or_subst

        elif is_a(first, bool):
            return value_or_subst

        elif is_a(first, float):
            return value_or_subst

        elif is_a(first, str):
            return value_or_subst
        else:
            return serialize_text_or_substitution(value_or_subst)


def serialize_substitution(subst_list: List[Substitution]) -> SubstitutionExpr:
    tokens = []

    for token in subst_list:
        if is_a(token, TextSubstitution):
            value = TextToken(text=token.text)

        elif is_a(token, EnvironmentVariable):
            name = serialize_text_or_substitution(token.name)

            default_value = None
            if token.default_value is not None:
                default_value = serialize_text_or_substitution(token.default_value)

            value = EnvironmentVariableToken(
                name=name,
                default_value=default_value,
            )

        elif is_a(token, AnonName):
            name = serialize_text_or_substitution(token.name)
            value = AnonNameToken(name=name)

        elif is_a(token, NotSubstitution):
            val = serialize_text_or_substitution(token.value)
            value = NotToken(value=val)

        elif is_a(token, AndSubstitution):
            lhs = serialize_text_or_substitution(token.left)
            rhs = serialize_text_or_substitution(token.right)
            value = AndToken(lhs, rhs)

        elif is_a(token, OrSubstitution):
            lhs = serialize_text_or_substitution(token.left)
            rhs = serialize_text_or_substitution(token.right)
            value = AndToken(lhs, rhs)

        elif is_a(token, Command):
            command = serialize_text_or_substitution(token.command)
            on_stderr = serialize_text_or_substitution(token.on_stderr)
            value = CommandToken(
                command=command,
                on_stderr=on_stderr,
            )

        elif is_a(token, FindExecutable):
            name = serialize_text_or_substitution(token.name)
            value = FindExecutableToken(name=name)

        elif is_a(token, LaunchConfiguration):
            variable_name = serialize_text_or_substitution(token.variable_name)
            value = LaunchConfigurationToken(variable_name=variable_name)

        elif is_a(token, LocalSubstitution):
            expression = serialize_text_or_substitution(token.expression)

            description = None
            if token.description is not None:
                description = serialize_text_or_substitution(token.expression)

            value = LocalToken(
                expression=expression,
                description=description,
            )

        elif is_a(token, PathJoinSubstitution):
            substitutions = serialize_text_or_substitution(token.substitutions)
            value = PathJoinToken(substitutions=substitutions)

        elif is_a(token, PythonExpression):
            expression = serialize_text_or_substitution(token.expression)
            value = PythonExpressionToken(expression=expression)

        elif is_a(token, ThisLaunchFileDir):
            value = ThisLaunchFileDirToken()

        elif is_a(token, ThisLaunchFile):
            value = ThisLaunchFileToken()

        elif is_a(token, FindPackageShare):
            package = serialize_text_or_substitution(token.package)
            value = FindPackageShareToken(package=package)

        elif is_a(token, Parameter):
            name = serialize_text_or_substitution(token.name)
            value = ParameterToken(name=name)

        elif is_a(token, ExecutableInPackage):
            executable = serialize_text_or_substitution(token.executable)
            value = ExecutableInPackageToken(executable=executable)

        else:
            raise ValueError(f"unexpected substitution token type `{type(token)}`")

        tokens.append(value)

    return tokens
