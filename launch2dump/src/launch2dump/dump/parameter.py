from dataclasses import dataclass
from typing import Union, Text, List

from launch.utilities import is_a
from launch.utilities.typing_file_path import FilePath
from launch_ros.descriptions import ParameterValue, ParameterFile

from .substitution import (
    SubstitutionExpr,
    TextOrSubstitutionExpr,
    serialize_substitution,
    serialize_text_or_substitution,
    serialize_parameter_value_or_substitution,
)


ParameterValueType = Union[
    bool,
    int,
    float,
    Text,
    List[bool],
    List[int],
    List[float],
    List[Text],
]
ParameterValueOrSubstitutionExpr = Union["ParameterValueType", SubstitutionExpr]


@dataclass
class ParameterFileDump:
    path: Union[Text, TextOrSubstitutionExpr]
    allow_substs: Union[bool, SubstitutionExpr]


def serialize_parameter_value(param_value: ParameterValue) -> ParameterValueOrSubstitutionExpr:
    value = serialize_parameter_value_or_substitution(
        param_value._ParameterValue__value
    )
    return value


def serialize_parameters_file(param_file: ParameterFile) -> ParameterFileDump:
    path = param_file._ParameterFile__param_file
    if is_a(path, list):
        path = serialize_text_or_substitution(path)
    else:
        assert is_a(path, FilePath)
        path = str(path)

    allow_substs = param_file._ParameterFile__allow_substs
    if is_a(allow_substs, list):
        allow_substs = serialize_substitution(allow_substs)
    else:
        assert is_a(allow_substs, bool)

    return ParameterFileDump(path=path, allow_substs=allow_substs)
