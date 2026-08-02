"""Tests for parse_package()."""

import os

import pytest

from play_launch_parser import parse_package


@pytest.fixture
def fake_package(tmp_path):
    """Create a fake ROS package with a launch file in a temp directory.

    Sets AMENT_PREFIX_PATH to include the temp directory so parse_package()
    can find it.
    """
    pkg_name = "test_fake_pkg"
    launch_dir = tmp_path / "share" / pkg_name / "launch"
    launch_dir.mkdir(parents=True)

    launch_file = launch_dir / "test.launch.xml"
    launch_file.write_text(
        '<launch>\n'
        '  <node pkg="demo_nodes_cpp" exec="talker" name="test_node" />\n'
        '</launch>\n'
    )

    old_ament = os.environ.get("AMENT_PREFIX_PATH")
    if old_ament:
        os.environ["AMENT_PREFIX_PATH"] = f"{tmp_path}:{old_ament}"
    else:
        os.environ["AMENT_PREFIX_PATH"] = str(tmp_path)

    yield pkg_name

    # Restore
    if old_ament:
        os.environ["AMENT_PREFIX_PATH"] = old_ament
    else:
        os.environ.pop("AMENT_PREFIX_PATH", None)


class TestParsePackage:
    """Test parse_package() with a fake ROS package."""

    def test_parse_fake_package(self, fake_package):
        result = parse_package(fake_package, "test.launch.xml")
        assert len(result["node"]) == 1
        assert result["node"][0]["name"] == "test_node"

    def test_returns_dict(self, fake_package):
        result = parse_package(fake_package, "test.launch.xml")
        assert isinstance(result, dict)
        assert "node" in result
        assert "container" in result

    def test_package_not_found(self):
        with pytest.raises(FileNotFoundError, match="not found"):
            parse_package("nonexistent_pkg_12345", "foo.launch.xml")

    def test_file_not_found_in_package(self, fake_package):
        with pytest.raises(FileNotFoundError, match="not found"):
            parse_package(fake_package, "nonexistent.launch.xml")


class TestNoAment:
    """Test behavior when AMENT_PREFIX_PATH is not set."""

    def test_missing_ament_prefix_path(self, monkeypatch):
        monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)
        with pytest.raises(FileNotFoundError, match="AMENT_PREFIX_PATH"):
            parse_package("some_pkg", "some.launch.xml")
