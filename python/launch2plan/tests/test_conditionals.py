"""
Tests for conditional branch exploration.

Phase 6: Conditional Branch Exploration
"""

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch2plan.visitor import (
    BranchExplorerSession,
    extract_condition_expression,
)


def test_extract_ifcondition_with_launch_configuration():
    """Test IfCondition with LaunchConfiguration extraction."""
    # Setup: IfCondition(LaunchConfiguration('use_sim_time'))
    condition = IfCondition(LaunchConfiguration("use_sim_time"))

    # Execute: Extract condition expression
    expr = extract_condition_expression(condition)

    # Verify: Converts to $(use_sim_time)
    assert expr == "$(use_sim_time)"


def test_extract_unlesscondition_with_launch_configuration():
    """Test UnlessCondition with LaunchConfiguration extraction."""
    # Setup: UnlessCondition(LaunchConfiguration('debug'))
    condition = UnlessCondition(LaunchConfiguration("debug"))

    # Execute: Extract condition expression
    expr = extract_condition_expression(condition)

    # Verify: Converts to $(not debug)
    assert expr == "$(not debug)"


def test_extract_ifcondition_with_text_true():
    """Test IfCondition with literal 'true' text."""
    # Setup: IfCondition with TextSubstitution('true')
    condition = IfCondition([TextSubstitution(text="true")])

    # Execute: Extract condition expression
    expr = extract_condition_expression(condition)

    # Verify: Returns 'true'
    assert expr == "true"


def test_extract_ifcondition_with_text_false():
    """Test IfCondition with literal 'false' text."""
    # Setup: IfCondition with TextSubstitution('false')
    condition = IfCondition([TextSubstitution(text="false")])

    # Execute: Extract condition expression
    expr = extract_condition_expression(condition)

    # Verify: Returns 'false'
    assert expr == "false"


def test_extract_unlesscondition_with_text_true():
    """Test UnlessCondition with literal 'true' text."""
    # Setup: UnlessCondition with TextSubstitution('true')
    condition = UnlessCondition([TextSubstitution(text="true")])

    # Execute: Extract condition expression
    expr = extract_condition_expression(condition)

    # Verify: Returns 'false' (negation of true)
    assert expr == "false"


def test_extract_unlesscondition_with_text_false():
    """Test UnlessCondition with literal 'false' text."""
    # Setup: UnlessCondition with TextSubstitution('false')
    condition = UnlessCondition([TextSubstitution(text="false")])

    # Execute: Extract condition expression
    expr = extract_condition_expression(condition)

    # Verify: Returns 'true' (negation of false)
    assert expr == "true"


def test_extract_none_condition():
    """Test that None condition returns None."""
    expr = extract_condition_expression(None)
    assert expr is None


def test_condition_stack_single_level():
    """Test condition stack with single condition."""
    # Setup: Session with one condition
    session = BranchExplorerSession()

    # Execute: Add condition to stack
    with session.condition_context("$(use_sim_time)"):
        current = session.get_current_condition()

    # Verify: Single condition returned as-is
    assert current == "$(use_sim_time)"

    # Verify: Stack is empty after context exit
    assert session.get_current_condition() is None


def test_condition_stack_nested_conditions():
    """Test condition stack with nested conditions."""
    # Setup: Session with nested conditions
    session = BranchExplorerSession()

    # Execute: Nest multiple conditions
    with session.condition_context("$(use_sim_time)"):
        with session.condition_context("$(debug_mode)"):
            current = session.get_current_condition()

    # Verify: Conditions combined with 'and'
    assert current == "($(use_sim_time)) and ($(debug_mode))"


def test_condition_stack_three_levels():
    """Test condition stack with three nested levels."""
    # Setup: Session with three nested conditions
    session = BranchExplorerSession()

    # Execute: Nest three conditions
    with session.condition_context("$(cond1)"):
        with session.condition_context("$(cond2)"):
            with session.condition_context("$(cond3)"):
                current = session.get_current_condition()

    # Verify: All three conditions combined with 'and'
    assert current == "($(cond1)) and ($(cond2)) and ($(cond3))"


def test_condition_context_none():
    """Test that None condition doesn't affect stack."""
    # Setup: Session
    session = BranchExplorerSession()

    # Execute: Try to add None condition
    with session.condition_context(None):
        current = session.get_current_condition()

    # Verify: Stack remains empty
    assert current is None


def test_condition_stack_mixed_if_unless():
    """Test condition stack with mixed IfCondition and UnlessCondition."""
    # Setup: Session
    session = BranchExplorerSession()

    # Execute: Mix IfCondition and UnlessCondition
    with session.condition_context("$(enable_feature)"):
        with session.condition_context("$(not debug_mode)"):
            current = session.get_current_condition()

    # Verify: Both conditions combined
    assert current == "($(enable_feature)) and ($(not debug_mode))"


def test_ifcondition_with_numeric_text():
    """Test IfCondition with numeric text ('1', '0')."""
    # Test with '1' - should be treated as true
    condition = IfCondition([TextSubstitution(text="1")])
    expr = extract_condition_expression(condition)
    assert expr == "true"

    # Test with '0' - should be treated as false
    condition = IfCondition([TextSubstitution(text="0")])
    expr = extract_condition_expression(condition)
    assert expr == "false"


def test_unlesscondition_with_numeric_text():
    """Test UnlessCondition with numeric text."""
    # Test with '1' - should be negated to false
    condition = UnlessCondition([TextSubstitution(text="1")])
    expr = extract_condition_expression(condition)
    assert expr == "false"

    # Test with '0' - should be negated to true
    condition = UnlessCondition([TextSubstitution(text="0")])
    expr = extract_condition_expression(condition)
    assert expr == "true"


def test_ifcondition_with_arbitrary_text():
    """Test IfCondition with arbitrary text (not true/false)."""
    # Setup: IfCondition with custom text
    condition = IfCondition([TextSubstitution(text="custom_value")])

    # Execute: Extract condition expression
    expr = extract_condition_expression(condition)

    # Verify: Returns as string literal
    assert expr == '"custom_value"'


def test_unlesscondition_with_arbitrary_text():
    """Test UnlessCondition with arbitrary text."""
    # Setup: UnlessCondition with custom text
    condition = UnlessCondition([TextSubstitution(text="custom_value")])

    # Execute: Extract condition expression
    expr = extract_condition_expression(condition)

    # Verify: Returns negated string literal
    assert expr == 'not "custom_value"'


def test_extract_empty_condition():
    """Test extraction of condition with empty predicate list."""
    # IfCondition with empty list
    condition = IfCondition([])
    expr = extract_condition_expression(condition)
    assert expr == "true"

    # UnlessCondition with empty list
    condition = UnlessCondition([])
    expr = extract_condition_expression(condition)
    assert expr == "false"


def test_case_insensitive_boolean_text():
    """Test that boolean text is case-insensitive."""
    # Test various cases of 'true'
    for text in ["true", "True", "TRUE", "TrUe"]:
        condition = IfCondition([TextSubstitution(text=text)])
        expr = extract_condition_expression(condition)
        assert expr == "true", f"Failed for text: {text}"

    # Test various cases of 'false'
    for text in ["false", "False", "FALSE", "FaLsE"]:
        condition = IfCondition([TextSubstitution(text=text)])
        expr = extract_condition_expression(condition)
        assert expr == "false", f"Failed for text: {text}"
