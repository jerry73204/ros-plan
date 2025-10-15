"""
Tests for argument type inference module.

Phase 5: Argument & Parameter Conversion
"""

import pytest

from launch2plan.arg_inference import infer_argument_type


def test_infer_bool_true():
    """Test boolean inference for 'true' value."""
    type_tag, value = infer_argument_type("true")
    assert type_tag == "!bool"
    assert value == "true"


def test_infer_bool_false():
    """Test boolean inference for 'false' value."""
    type_tag, value = infer_argument_type("false")
    assert type_tag == "!bool"
    assert value == "false"


def test_infer_bool_case_insensitive():
    """Test boolean inference is case-insensitive."""
    # Test various cases
    for val in ["True", "TRUE", "False", "FALSE"]:
        type_tag, value = infer_argument_type(val)
        assert type_tag == "!bool"
        assert value in ("true", "false")


def test_infer_integer_positive():
    """Test integer inference for positive values."""
    type_tag, value = infer_argument_type("42")
    assert type_tag == "!i64"
    assert value == "42"


def test_infer_integer_negative():
    """Test integer inference for negative values."""
    type_tag, value = infer_argument_type("-100")
    assert type_tag == "!i64"
    assert value == "-100"


def test_infer_integer_zero():
    """Test integer inference for zero."""
    type_tag, value = infer_argument_type("0")
    assert type_tag == "!i64"
    assert value == "0"


def test_infer_float_decimal():
    """Test float inference for decimal values."""
    type_tag, value = infer_argument_type("3.14")
    assert type_tag == "!f64"
    assert value == "3.14"


def test_infer_float_scientific():
    """Test float inference for scientific notation."""
    type_tag, value = infer_argument_type("1.23e-4")
    assert type_tag == "!f64"
    assert value == "1.23e-4"


def test_infer_float_negative():
    """Test float inference for negative values."""
    type_tag, value = infer_argument_type("-2.5")
    assert type_tag == "!f64"
    assert value == "-2.5"


def test_infer_string_path():
    """Test string inference for path values."""
    type_tag, value = infer_argument_type("/path/to/file")
    assert type_tag == "!str"
    assert value == "/path/to/file"


def test_infer_string_namespace():
    """Test string inference for namespace values."""
    type_tag, value = infer_argument_type("/robot/sensors")
    assert type_tag == "!str"
    assert value == "/robot/sensors"


def test_infer_string_general():
    """Test string inference for general text."""
    type_tag, value = infer_argument_type("hello_world")
    assert type_tag == "!str"
    assert value == "hello_world"


def test_infer_none_default():
    """Test TODO marker when no default value provided."""
    type_tag, value = infer_argument_type(None)
    assert type_tag == "!todo"
    assert value is None


def test_infer_empty_string():
    """Test empty string inference."""
    type_tag, value = infer_argument_type("")
    assert type_tag == "!str"
    assert value == ""


def test_infer_whitespace_handling():
    """Test that whitespace is stripped before inference."""
    type_tag, value = infer_argument_type("  42  ")
    assert type_tag == "!i64"
    assert value == "42"


def test_infer_mixed_alphanumeric():
    """Test mixed alphanumeric strings are treated as strings."""
    type_tag, value = infer_argument_type("robot123")
    assert type_tag == "!str"
    assert value == "robot123"


def test_infer_numeric_string_with_units():
    """Test numeric-like strings with units are treated as strings."""
    type_tag, value = infer_argument_type("10m")
    assert type_tag == "!str"
    assert value == "10m"
