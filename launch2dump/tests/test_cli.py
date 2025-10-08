"""Tests for the CLI tool."""

import json
import sys
from pathlib import Path

import pytest

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from launch2dump.__main__ import main, parse_launch_arguments


def test_parse_launch_arguments():
    """Test parsing launch arguments."""
    args = ["key1:=value1", "key2:=42", "path:=/some/path"]
    result = parse_launch_arguments(args)

    assert result == {"key1": "value1", "key2": "42", "path": "/some/path"}


def test_parse_launch_arguments_empty():
    """Test parsing empty launch arguments."""
    result = parse_launch_arguments(None)
    assert result == {}

    result = parse_launch_arguments([])
    assert result == {}


def test_cli_help():
    """Test CLI help output."""
    # argparse raises SystemExit for --help
    with pytest.raises(SystemExit) as exc_info:
        main(["--help"])
    assert exc_info.value.code == 0


def test_cli_missing_file(capsys):
    """Test CLI with missing launch file."""
    exit_code = main(["nonexistent.launch.py"])

    assert exit_code == 1

    captured = capsys.readouterr()
    # Error message changes based on whether it's interpreted as package or file
    assert "error" in captured.err.lower()


def test_cli_simple_launch_yaml(capsys):
    """Test CLI with simple launch file, YAML output."""
    launch_file = Path(__file__).parent / "fixtures" / "simple_talker.launch.py"
    exit_code = main([str(launch_file)])

    assert exit_code == 0

    captured = capsys.readouterr()
    output = captured.out

    # Should contain YAML-formatted output
    assert "nodes:" in output
    assert "package:" in output
    assert "demo_nodes_cpp" in output
    assert "talker" in output


def test_cli_simple_launch_json(capsys):
    """Test CLI with simple launch file, JSON output."""
    launch_file = Path(__file__).parent / "fixtures" / "simple_talker.launch.py"
    exit_code = main([str(launch_file), "-f", "json"])

    assert exit_code == 0

    captured = capsys.readouterr()
    output = captured.out

    # Should be valid JSON
    data = json.loads(output)
    assert "nodes" in data
    assert "containers" in data
    assert "parameter_dependencies" in data
    assert "errors" in data

    # Check node data
    assert len(data["nodes"]) >= 1
    node = data["nodes"][0]
    assert node["package"] == "demo_nodes_cpp"
    assert node["executable"] == "talker"


def test_cli_with_arguments(capsys):
    """Test CLI with launch arguments (positional syntax)."""
    launch_file = Path(__file__).parent / "fixtures" / "with_parameters.launch.py"
    exit_code = main([str(launch_file), "publish_rate:=20", "node_name:=test"])

    assert exit_code == 0

    captured = capsys.readouterr()
    output = captured.out

    # Should contain the launch output
    assert "nodes:" in output


def test_cli_output_to_file(tmp_path):
    """Test CLI with output to file."""
    launch_file = Path(__file__).parent / "fixtures" / "simple_talker.launch.py"
    output_file = tmp_path / "output.yaml"

    exit_code = main([str(launch_file), "-o", str(output_file)])

    assert exit_code == 0
    assert output_file.exists()

    # Read and verify output
    content = output_file.read_text()
    assert "nodes:" in content
    assert "demo_nodes_cpp" in content


def test_cli_output_json_to_file(tmp_path):
    """Test CLI with JSON output to file."""
    launch_file = Path(__file__).parent / "fixtures" / "simple_talker.launch.py"
    output_file = tmp_path / "output.json"

    exit_code = main([str(launch_file), "-f", "json", "-o", str(output_file)])

    assert exit_code == 0
    assert output_file.exists()

    # Read and verify JSON
    content = output_file.read_text()
    data = json.loads(content)
    assert "nodes" in data
    assert len(data["nodes"]) >= 1


def test_cli_invalid_argument_format(capsys):
    """Test CLI with invalid argument format."""
    launch_file = Path(__file__).parent / "fixtures" / "simple_talker.launch.py"

    # parse_launch_arguments calls sys.exit(1) on invalid format
    with pytest.raises(SystemExit) as exc_info:
        main([str(launch_file), "-a", "invalid_format"])

    assert exc_info.value.code == 1

    captured = capsys.readouterr()
    assert "invalid argument format" in captured.err.lower()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
