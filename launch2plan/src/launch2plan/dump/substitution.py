from dataclasses import dataclass
from enum import Enum
from typing import List, Union, Text, Optional, List

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
    FIND_EXEC = "find-exec"
    VAR = "var"
    EVAL = "eval"
    PATH_JOIN = "path_join"
    LOCAL = "local"
    FILENAME = "filename"
    DIRNAME = "dirname"
    FIND_PKG_SHARE = "find-pkg-share"
    PARAM = "param"
    EXEC_IN_PKG = "exec-in-pkg"


@dataclass
class TextToken:
    text: Text


@dataclass
class EnvironmentVariableToken:
    name: "SubstitutionExpr"
    default_value: Optional["SubstitutionExpr"]


@dataclass
class AnonNameToken:
    name: "SubstitutionExpr"


@dataclass
class AndToken:
    lhs: "SubstitutionExpr"
    rhs: "SubstitutionExpr"


@dataclass
class OrToken:
    lhs: "SubstitutionExpr"
    rhs: "SubstitutionExpr"


@dataclass
class NotToken:
    value: "SubstitutionExpr"


@dataclass
class CommandToken:
    command: "SubstitutionExpr"
    on_stderr: "SubstitutionExpr"


@dataclass
class FindExecutableToken:
    name: "SubstitutionExpr"


@dataclass
class LaunchConfigurationToken:
    variable_name: "SubstitutionExpr"


@dataclass
class LocalToken:
    expression: Text
    description: Optional[Text]


@dataclass
class PathJoinToken:
    substitutions: List["SubstitutionToken"]


@dataclass
class PythonExpressionToken:
    expression: "SubstitutionExpr"


@dataclass
class ThisLaunchFileDirToken:
    # empty
    pass


@dataclass
class ThisLaunchFileToken:
    # empty
    pass


@dataclass
class FindPackageShareToken:
    package: "SubstitutionExpr"


@dataclass
class ParameterToken:
    name: "SubstitutionExpr"


@dataclass
class ExecutableInPackageToken:
    executable: "SubstitutionExpr"


@dataclass
class SubstitutionToken:
    kind: SubstitutionTokenKind
    value: Union[TextToken, EnvironmentVariableToken]


@dataclass
class SubstitutionExpr:
    tokens: List[SubstitutionToken]


ParameterValueOrSubstitutionExpr = Union["ParameterValueType", SubstitutionExpr]
TextOrSubstitutionExpr = Union[Text, SubstitutionExpr]


def serialize_text_or_substitution(
    text_or_list: Union[Text, List[Substitution]]
) -> TextOrSubstitutionExpr:
    if is_a(text_or_list, str):
        return text_or_list
    else:
        assert is_a(text_or_list, list)
        return serialize_substitution(text_or_list)


def serialize_parameter_value_or_substitution(
    value_or_list: Union[bool, int, float, Text, List[Substitution]]
) -> ParameterValueOrSubstitutionExpr:
    if is_a(value_or_list, int):
        return value_or_list

    elif is_a(value_or_list, bool):
        return value_or_list

    elif is_a(value_or_list, float):
        return value_or_list

    elif is_a(value_or_list, str):
        return value_or_list

    else:
        assert is_a(value_or_list, list)

        if len(value_or_list) == 0:
            return list()

        first = value_or_list[0]

        if is_a(first, int):
            return value_or_list

        elif is_a(first, bool):
            return value_or_list

        elif is_a(first, float):
            return value_or_list

        elif is_a(first, str):
            return value_or_list
        else:
            return serialize_substitution(value_or_list)


def serialize_substitution(subst_list: List[Substitution]) -> SubstitutionExpr:
    tokens = []

    for token in subst_list:
        if is_a(token, TextSubstitution):
            kind = SubstitutionTokenKind.TEXT
            value = TextToken(text=token.text)

        elif is_a(token, EnvironmentVariable):
            default_value = None
            if token.default_value is not None:
                default_value = serialize_substitution(token.default_value)

            kind = SubstitutionTokenKind.ENV
            value = EnvironmentVariableToken(
                name=serialize_substitution(token.name),
                default_value=default_value,
            )

        elif is_a(token, AnonName):
            kind = SubstitutionTokenKind.ANON
            value = AnonNameToken(name=serialize_substitution(token.name))

        elif is_a(token, NotSubstitution):
            kind = SubstitutionTokenKind.NOT
            value = NotToken(value=serialize_substitution(token.value))

        elif is_a(token, AndSubstitution):
            kind = SubstitutionTokenKind.AND
            value = AndToken(
                lhs=serialize_substitution(token.left),
                rhs=serialize_substitution(token.right),
            )

        elif is_a(token, OrSubstitution):
            kind = SubstitutionTokenKind.OR
            value = AndToken(
                lhs=serialize_substitution(token.left),
                rhs=serialize_substitution(token.right),
            )

        elif is_a(token, Command):
            kind = SubstitutionTokenKind.COMMAND
            value = CommandToken(
                command=serialize_substitution(token.command),
                on_stderr=serialize_substitution(token.on_stderr),
            )

        elif is_a(token, FindExecutable):
            kind = SubstitutionTokenKind.FIND_EXEC
            value = FindExecutableToken(name=serialize_substitution(token.name))

        elif is_a(token, LaunchConfiguration):
            kind = SubstitutionTokenKind.VAR
            value = LaunchConfigurationToken(
                variable_name=serialize_substitution(token.variable_name)
            )

        elif is_a(token, LocalSubstitution):
            kind = SubstitutionTokenKind.LOCAL

            description = None
            if token.description is not None:
                description = serialize_substitution(token.expression)

            value = LocalToken(
                expression=serialize_substitution(token.expression),
                description=description,
            )

        elif is_a(token, PathJoinSubstitution):
            kind = SubstitutionTokenKind.PATH_JOIN
            value = PathJoinToken(
                substitutions=serialize_substitution(token.substitutions)
            )

        elif is_a(token, PythonExpression):
            kind = SubstitutionTokenKind.EVAL
            value = PythonExpressionToken(
                expression=serialize_substitution(token.expression)
            )

        elif is_a(token, ThisLaunchFileDir):
            kind = SubstitutionTokenKind.DIRNAME
            value = ThisLaunchFileDirToken()

        elif is_a(token, ThisLaunchFile):
            kind = SubstitutionTokenKind.FILENAME
            value = ThisLaunchFileToken()

        elif is_a(token, FindPackageShare):
            kind = SubstitutionTokenKind.FIND_PKG_SHARE
            value = FindPackageShareToken(package=serialize_substitution(token.package))

        elif is_a(token, Parameter):
            kind = SubstitutionTokenKind.PARAM
            value = ParameterToken(name=serialize_substitution(token.name))

        elif is_a(token, ExecutableInPackage):
            kind = SubstitutionTokenKind.EXEC_IN_PKG
            value = ExecutableInPackageToken(
                executable=serialize_substitution(token.executable)
            )

        else:
            raise ValueError(f"unexpected substitution token type `{type(token)}`")

        tokens.append(
            SubstitutionToken(
                kind=kind,
                value=value,
            )
        )

    return SubstitutionExpr(tokens=tokens)
