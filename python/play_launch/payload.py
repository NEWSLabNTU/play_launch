"""Pick the ROS distribution payload a wheel carries.

The binaries link one ROS distribution's libraries (rcl, rclcpp, message
typesupport) and its libpython, so a wheel holds one payload per
distribution it serves, under ``play_launch/dist/<distro>/{bin,lib,share}``.
Wheel tags already separate distributions that need a different Python or
glibc (cp310 for Humble, cp312 for Jazzy and Kilted, cp314 for Lyrical and
Rolling); the payload is the part a tag cannot express, chosen here when the
tool runs.

Order (phase 92):

1. ``PLAY_LAUNCH_ROS_DISTRO``, an override for tests and unusual setups;
2. ``ROS_DISTRO``, set by sourcing any ROS setup file;
3. the only ``/opt/ros/<d>`` present that the wheel carries;
4. otherwise an error naming what the wheel carries, what was found, and the
   fix.

Run ``python3 -m play_launch.payload`` to see what the wheel carries and why
one payload is chosen.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path

# The Python each distribution's binaries link. A distribution the wheel does
# not carry is usually one built for another Python, and the message says so.
DISTRO_PYTHON = {
    "humble": "3.10",
    "jazzy": "3.12",
    "kilted": "3.12",
    "lyrical": "3.14",
    "rolling": "3.14",
}

OVERRIDE_VAR = "PLAY_LAUNCH_ROS_DISTRO"


class PayloadError(RuntimeError):
    """No payload in this wheel can serve the environment."""


@dataclass(frozen=True)
class Payload:
    distro: str
    prefix: Path
    reason: str

    @property
    def bin_dir(self) -> Path:
        return self.prefix / "bin"

    @property
    def lib_dir(self) -> Path:
        return self.prefix / "lib"


def package_dir() -> Path:
    return Path(__file__).resolve().parent


def carried(pkg_dir: Path | None = None) -> list[str]:
    """Distributions this installation carries a payload for, sorted."""
    root = (pkg_dir or package_dir()) / "dist"
    if not root.is_dir():
        return []
    return sorted(d.name for d in root.iterdir() if (d / "bin" / "play_launch").exists())


def installed(opt_ros: Path = Path("/opt/ros")) -> list[str]:
    """Distributions installed under /opt/ros, sorted."""
    if not opt_ros.is_dir():
        return []
    return sorted(d.name for d in opt_ros.iterdir() if (d / "setup.bash").exists())


def _not_carried(distro: str, source: str, have: list[str]) -> PayloadError:
    py = f"{sys.version_info.major}.{sys.version_info.minor}"
    msg = f"{source} is '{distro}', but this play_launch carries {_list(have)} (Python {py})."
    need = DISTRO_PYTHON.get(distro)
    if need and need != py:
        msg += (
            f"\n  ROS {distro} needs Python {need}: install play_launch with that"
            f" Python (python{need} -m pip install play_launch)."
        )
    elif not need:
        msg += f"\n  play_launch does not build for '{distro}'."
    return PayloadError(msg)


def _list(items: list[str]) -> str:
    return ", ".join(items) if items else "no payload"


def select(
    env: dict[str, str] | None = None,
    pkg_dir: Path | None = None,
    opt_ros: Path = Path("/opt/ros"),
) -> Payload:
    """The payload to run, or PayloadError saying why there is none."""
    env = os.environ if env is None else env
    pkg = pkg_dir or package_dir()
    have = carried(pkg)

    def make(distro: str, reason: str) -> Payload:
        return Payload(distro, pkg / "dist" / distro, reason)

    for var in (OVERRIDE_VAR, "ROS_DISTRO"):
        distro = env.get(var, "").strip()
        if distro:
            if distro not in have:
                raise _not_carried(distro, var, have)
            return make(distro, f"{var}={distro}")

    found = installed(opt_ros)
    usable = [d for d in found if d in have]
    if len(usable) == 1:
        return make(usable[0], f"the only distribution under {opt_ros} this wheel carries")
    if not usable:
        detail = f"found under {opt_ros}: {_list(found)}"
    else:
        detail = f"several under {opt_ros} it carries: {_list(usable)}"
    hint = usable[0] if usable else (have[0] if have else "<distro>")
    raise PayloadError(
        f"No ROS distribution selected. This play_launch carries {_list(have)}; {detail}.\n"
        f"  Source one first: source /opt/ros/{hint}/setup.bash"
        f" (or set {OVERRIDE_VAR})."
    )


def report() -> str:
    """What the installation carries and which payload this shell would get."""
    lines = [
        f"package:   {package_dir()}",
        f"python:    {sys.version_info.major}.{sys.version_info.minor}",
        f"carries:   {_list(carried())}",
        f"/opt/ros:  {_list(installed())}",
    ]
    try:
        p = select()
        lines.append(f"selected:  {p.distro} ({p.reason})")
        lines.append(f"binary:    {p.bin_dir / 'play_launch'}")
    except PayloadError as e:
        lines.append(f"selected:  none\n{e}")
    return "\n".join(lines)


if __name__ == "__main__":
    print(report())
