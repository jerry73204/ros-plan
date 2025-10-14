"""Tests for CLI functionality (Phase 12.1)."""

import subprocess
from pathlib import Path

import pytest


@pytest.fixture
def fixtures_dir():
    """Return the fixtures directory."""
    return Path(__file__).parent / "fixtures"


def test_convert_command(fixtures_dir):
    """Test basic convert command invocation."""
    launch_file = fixtures_dir / "simple.launch.py"

    result = subprocess.run(
        ["uv", "run", "launch2plan", "convert", str(launch_file)],
        capture_output=True,
        text=True,
        cwd=Path(__file__).parent.parent,
    )

    assert result.returncode == 0
    assert "demo_nodes_cpp::talker" in result.stdout
    assert "✓ Discovered 1 nodes" in result.stdout
