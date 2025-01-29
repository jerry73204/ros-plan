import decimal
from typing import Text, Optional, Tuple, Optional
from enum import Enum
import dataclasses

from rcl_interfaces.msg import Parameter
from ruamel.yaml import YAML
from ruamel.yaml.compat import StringIO
from ruamel.yaml.comments import TaggedScalar

from .dump.substitution import SubstitutionToken


MY_YAML = YAML(typ="safe")
# MY_YAML.default_flow_style = True
# MY_YAML.width = 65536

DECIMAL_CONTEXT = decimal.Context()
DECIMAL_CONTEXT.prec = 20


def float_to_str(f):
    """
    Convert the given float to a string,
    without resorting to scientific notation
    """
    global DECIMAL_CONTEXT
    d = DECIMAL_CONTEXT.create_decimal(repr(f))
    return format(d, "f")


def log_level_code_to_text(code: int) -> Optional[Text]:
    match code:
        case 0:
            return None
        case 10:
            return "DEBUG"
        case 20:
            return "INFO"
        case 30:
            return "WARN"
        case 40:
            return "ERROR"
        case 50:
            return "FATAL"
        case _:
            raise ValueError(f"unknown log level code {code}")


def text_to_kv(expr: Text):
    return tuple(expr.split(":=", 1))


def param_to_kv(param: Parameter) -> Tuple[Text, Text]:
    pvalue = param.value

    match pvalue.type:
        case 1:
            value = dump_yaml(pvalue.bool_value)
        case 2:
            value = dump_yaml(pvalue.integer_value)
        case 3:
            # NOTE: ruamle.yaml serializes small floats into scientific
            # notations like 1e-7. It is treated as a string by `ros2`
            # command. It uses an alternative way to work around.

            # value = dump_yaml(pvalue.double_value)
            value = float_to_str(pvalue.double_value)
        case 4:
            value = dump_yaml(pvalue.string_value)
        case 5:
            value = dump_yaml(list(pvalue.byte_array_value))
        case 6:
            value = dump_yaml(list(pvalue.bool_array_value))
        case 7:
            value = dump_yaml(list(pvalue.integer_array_value))
        case 8:
            # NOTE: ruamle.yaml serializes small floats into scientific
            # notations like 1e-7. It is treated as a string by `ros2`
            # command. It uses an alternative way to work around.

            # value = dump_yaml(list(pvalue.double_array_value))
            value = (
                "["
                + ", ".join(float_to_str(f) for f in pvalue.double_array_value)
                + "]"
            )
        case 9:
            value = dump_yaml(list(pvalue.string_array_value))
        case _:
            raise ValueError(f"unknown parameter type code {param.type}")

    return param.name, value


def dump_yaml(value) -> Text:
    global MY_YAML

    def asdict_factory(data):
        def convert_value(obj):
            if isinstance(obj, Enum):
                return obj.value
            else:
                return obj

        return dict((k, convert_value(v)) for k, v in data)

    stream = StringIO()
    value_dict = dataclasses.asdict(value, dict_factory=asdict_factory)
    MY_YAML.dump(data=value_dict, stream=stream)
    return stream.getvalue()
