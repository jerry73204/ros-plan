"""
Tests for progress tracking module.
"""

import io

import pytest

from launch2plan.progress import ProgressTracker


def test_progress_disabled():
    """Test that disabled progress tracker produces no output."""
    output = io.StringIO()
    progress = ProgressTracker(enabled=False, file=output)

    progress.start_phase("Test Phase", total=3)
    progress.update("item 1")
    progress.update("item 2")
    progress.end_phase("Done")
    progress.message("Standalone message")

    assert output.getvalue() == ""


def test_progress_with_total():
    """Test progress tracking with known total."""
    output = io.StringIO()
    progress = ProgressTracker(enabled=True, file=output)

    progress.start_phase("Processing items", total=3)
    progress.update("item 1")
    progress.update("item 2")
    progress.update("item 3")
    progress.end_phase("✓ Complete")

    result = output.getvalue()
    assert "Processing items... (0/3)" in result
    assert "[1/3] item 1" in result
    assert "[2/3] item 2" in result
    assert "[3/3] item 3" in result
    assert "✓ Complete" in result


def test_progress_without_total():
    """Test progress tracking without known total."""
    output = io.StringIO()
    progress = ProgressTracker(enabled=True, file=output)

    progress.start_phase("Discovering files")
    progress.update("file1.py")
    progress.update("file2.py")
    progress.end_phase()

    result = output.getvalue()
    assert "Discovering files..." in result
    assert "- file1.py" in result
    assert "- file2.py" in result


def test_progress_standalone_message():
    """Test standalone message output."""
    output = io.StringIO()
    progress = ProgressTracker(enabled=True, file=output)

    progress.message("Important message")

    result = output.getvalue()
    assert "Important message" in result


def test_progress_multiple_phases():
    """Test multiple sequential phases."""
    output = io.StringIO()
    progress = ProgressTracker(enabled=True, file=output)

    progress.start_phase("Phase 1", total=2)
    progress.update("item 1")
    progress.update("item 2")
    progress.end_phase("✓ Phase 1 done")

    progress.start_phase("Phase 2", total=1)
    progress.update("item 1")
    progress.end_phase("✓ Phase 2 done")

    result = output.getvalue()
    assert "Phase 1... (0/2)" in result
    assert "Phase 2... (0/1)" in result
    assert "✓ Phase 1 done" in result
    assert "✓ Phase 2 done" in result


def test_progress_update_without_phase():
    """Test that update without active phase is safe (no crash)."""
    output = io.StringIO()
    progress = ProgressTracker(enabled=True, file=output)

    # Should not crash
    progress.update("orphan item")
    progress.end_phase("orphan end")

    # No output expected since no phase was started
    result = output.getvalue()
    assert result == ""
