"""Phase 92: which ROS distribution payload the wheel's entry point runs."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from play_launch import cli, payload  # noqa: E402

PY = f"{sys.version_info.major}.{sys.version_info.minor}"


def make_pkg(tmp_path: Path, *distros: str) -> Path:
    pkg = tmp_path / "play_launch"
    for d in distros:
        prefix = pkg / "dist" / d
        (prefix / "bin").mkdir(parents=True)
        (prefix / "lib").mkdir()
        (prefix / "share" / "ament_index").mkdir(parents=True)
        exe = prefix / "bin" / "play_launch"
        exe.write_text("#!/bin/sh\n")
        exe.chmod(0o755)
    pkg.mkdir(exist_ok=True)
    return pkg


def make_opt(tmp_path: Path, *distros: str) -> Path:
    opt = tmp_path / "opt_ros"
    opt.mkdir()
    for d in distros:
        (opt / d).mkdir()
        (opt / d / "setup.bash").write_text("")
    return opt


def test_carried_lists_payloads(tmp_path):
    assert payload.carried(make_pkg(tmp_path, "kilted", "jazzy")) == ["jazzy", "kilted"]


def test_override_beats_ros_distro(tmp_path):
    pkg = make_pkg(tmp_path, "jazzy", "kilted")
    env = {"PLAY_LAUNCH_ROS_DISTRO": "kilted", "ROS_DISTRO": "jazzy"}
    p = payload.select(env, pkg, make_opt(tmp_path))
    assert (p.distro, p.reason) == ("kilted", "PLAY_LAUNCH_ROS_DISTRO=kilted")
    assert p.prefix == pkg / "dist" / "kilted"


def test_ros_distro_selects(tmp_path):
    pkg = make_pkg(tmp_path, "jazzy", "kilted")
    p = payload.select({"ROS_DISTRO": "jazzy"}, pkg, make_opt(tmp_path, "jazzy", "kilted"))
    assert p.distro == "jazzy"


def test_ros_distro_not_carried_names_python(tmp_path):
    pkg = make_pkg(tmp_path, "humble")
    other = next(d for d, v in payload.DISTRO_PYTHON.items() if v != PY)
    with pytest.raises(payload.PayloadError) as e:
        payload.select({"ROS_DISTRO": other}, pkg, make_opt(tmp_path))
    msg = str(e.value)
    assert f"ROS_DISTRO is '{other}'" in msg
    assert "carries humble" in msg
    assert f"needs Python {payload.DISTRO_PYTHON[other]}" in msg


def test_unknown_distro(tmp_path):
    pkg = make_pkg(tmp_path, "humble")
    with pytest.raises(payload.PayloadError, match="does not build for 'foxy'"):
        payload.select({"ROS_DISTRO": "foxy"}, pkg, make_opt(tmp_path))


def test_single_opt_ros_without_ros_distro(tmp_path):
    pkg = make_pkg(tmp_path, "jazzy", "kilted")
    # humble is installed too but not carried, so jazzy is the only usable one.
    p = payload.select({}, pkg, make_opt(tmp_path, "humble", "jazzy"))
    assert p.distro == "jazzy"
    assert "only distribution" in p.reason


def test_several_opt_ros_is_an_error(tmp_path):
    pkg = make_pkg(tmp_path, "jazzy", "kilted")
    with pytest.raises(payload.PayloadError) as e:
        payload.select({}, pkg, make_opt(tmp_path, "jazzy", "kilted"))
    msg = str(e.value)
    assert "several" in msg and "source /opt/ros/jazzy/setup.bash" in msg


def test_nothing_installed_is_an_error(tmp_path):
    pkg = make_pkg(tmp_path, "humble")
    with pytest.raises(payload.PayloadError) as e:
        payload.select({}, pkg, make_opt(tmp_path))
    msg = str(e.value)
    assert "carries humble" in msg and "found under" in msg and "no payload" in msg


def test_empty_ros_distro_falls_through(tmp_path):
    pkg = make_pkg(tmp_path, "humble")
    p = payload.select({"ROS_DISTRO": " "}, pkg, make_opt(tmp_path, "humble"))
    assert p.distro == "humble"


def test_payload_env_points_at_the_payload(tmp_path, monkeypatch):
    pkg = make_pkg(tmp_path, "humble")
    prefix = pkg / "dist" / "humble"
    monkeypatch.setenv("LD_LIBRARY_PATH", "/opt/ros/humble/lib")
    monkeypatch.setenv("AMENT_PREFIX_PATH", "/opt/ros/humble")
    env = cli.payload_env(str(prefix / "bin" / "play_launch"))
    assert env["LD_LIBRARY_PATH"] == f"{prefix / 'lib'}:/opt/ros/humble/lib"
    assert env["AMENT_PREFIX_PATH"] == f"{prefix}:/opt/ros/humble"


def test_payload_env_leaves_a_colcon_binary_alone(tmp_path, monkeypatch):
    monkeypatch.setenv("LD_LIBRARY_PATH", "/x")
    exe = tmp_path / "install" / "play_launch" / "lib" / "play_launch" / "play_launch"
    exe.parent.mkdir(parents=True)
    exe.write_text("")
    assert cli.payload_env(str(exe))["LD_LIBRARY_PATH"] == "/x"


def test_find_binary_refuses_to_fall_through(tmp_path, monkeypatch):
    pkg = make_pkg(tmp_path, "humble")
    monkeypatch.setattr(payload, "package_dir", lambda: pkg)
    monkeypatch.setenv("ROS_DISTRO", "foxy")
    with pytest.raises(payload.PayloadError):
        cli._find_binary("play_launch")
    monkeypatch.setenv("ROS_DISTRO", "humble")
    assert cli._find_binary("play_launch") == str(pkg / "dist" / "humble" / "bin" / "play_launch")
