"""Tests for the CLI entry point."""

import json
import subprocess
import sys

import pytest

from tests.conftest import FIXTURES_DIR


@pytest.fixture
def cli():
    """Run play-launch-parser CLI and return (stdout, stderr, returncode)."""

    def _cli(*args):
        # Call _cli_main via a one-liner to avoid entry_point discovery issues
        code = (
            "import sys; from play_launch_parser import _cli_main; "
            "_cli_main(['play-launch-parser'] + sys.argv[1:])"
        )
        result = subprocess.run(
            [sys.executable, "-c", code] + list(args),
            capture_output=True,
            text=True,
        )
        return result.stdout, result.stderr, result.returncode

    return _cli


@pytest.fixture
def args_fixture():
    return str(FIXTURES_DIR / "test_args.launch.xml")


class TestCliJson:
    """Test JSON output (default format)."""

    def test_json_to_stdout(self, cli, args_fixture):
        stdout, _, rc = cli("file", args_fixture)
        assert rc == 0
        data = json.loads(stdout)
        assert len(data["node"]) == 1

    def test_json_valid(self, cli, args_fixture):
        stdout, _, rc = cli("file", args_fixture)
        assert rc == 0
        json.loads(stdout)


class TestCliFormats:
    """Test --format flag."""

    def test_summary(self, cli, args_fixture):
        stdout, _, rc = cli("file", args_fixture, "--format", "summary")
        assert rc == 0
        assert "Nodes:" in stdout
        assert "my_talker" in stdout

    def test_names(self, cli, args_fixture):
        stdout, _, rc = cli("file", args_fixture, "--format", "names")
        assert rc == 0
        lines = stdout.strip().split("\n")
        assert len(lines) == 1
        assert "my_talker" in lines[0]


class TestCliExitCodes:
    """Test exit codes."""

    def test_file_not_found(self, cli):
        _, _, rc = cli("file", "/nonexistent.launch.xml")
        assert rc == 2

    def test_package_not_found(self, cli):
        _, _, rc = cli("launch", "nonexistent_pkg", "foo.xml")
        assert rc == 2


class TestCliArgs:
    """Test launch argument passing."""

    def test_override_arg(self, cli, args_fixture):
        stdout, _, rc = cli("file", args_fixture, "node_name:=overridden")
        assert rc == 0
        data = json.loads(stdout)
        assert data["node"][0]["name"] == "overridden"
