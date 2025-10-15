"""
Argument type inference for launch2plan.

This module infers ROS-Plan argument types from DeclareLaunchArgument default values.
"""

from typing import Optional, Tuple


def infer_argument_type(default_value: Optional[str]) -> Tuple[str, Optional[str]]:
    """
    Infer the argument type from its default value.

    Args:
        default_value: The default value string (or None)

    Returns:
        Tuple of (type_tag, converted_value):
        - type_tag: One of "!bool", "!i64", "!f64", "!str", or "!todo"
        - converted_value: The value converted to appropriate type (or None for !todo)

    Examples:
        >>> infer_argument_type("true")
        ("!bool", "true")
        >>> infer_argument_type("42")
        ("!i64", "42")
        >>> infer_argument_type("3.14")
        ("!f64", "3.14")
        >>> infer_argument_type("hello")
        ("!str", "hello")
        >>> infer_argument_type(None)
        ("!todo", None)
    """
    if default_value is None:
        return ("!todo", None)

    # Try to infer type from string value
    value = default_value.strip()

    # Check for boolean
    if value.lower() in ("true", "false"):
        return ("!bool", value.lower())

    # Check for integer
    try:
        int(value)
        return ("!i64", value)
    except ValueError:
        pass

    # Check for float
    try:
        float(value)
        return ("!f64", value)
    except ValueError:
        pass

    # Default to string
    return ("!str", value)
