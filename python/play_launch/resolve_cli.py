"""CLI entry point for ros-launch-resolve that dispatches to the Rust binary.

`ros-launch-resolve` is layer 2: it resolves launch trees into a SystemModel
and needs NO ROS install, so — unlike `play_launch` — this shim deliberately
does not touch `LD_LIBRARY_PATH` or `AMENT_PREFIX_PATH`.
"""

import os
import shutil
import subprocess
import sys
from pathlib import Path


def _find_binary() -> str:
    """Find the bundled ros-launch-resolve binary, falling back to PATH.

    Resolution order:
    1. Package bin directory (pip install layout)
    2. Repo build output (running from a source checkout)
    3. PATH search (fallback)
    """
    name = "ros-launch-resolve"

    pkg_dir = Path(__file__).parent
    bundled = pkg_dir / "bin" / name
    if bundled.exists() and os.access(bundled, os.X_OK):
        return str(bundled)

    # Source checkout: its own cargo workspace, outside colcon.
    for build_path in (
        Path("src/ros-launch-resolve/target/release") / name,
        Path("src/ros-launch-resolve/target/debug") / name,
    ):
        if build_path.exists() and os.access(build_path, os.X_OK):
            return str(build_path)

    path_binary = shutil.which(name)
    if path_binary:
        return path_binary

    raise FileNotFoundError(
        f"{name} not found. Ensure play_launch is properly installed.\n"
        f"Checked locations:\n"
        f"  - {bundled}\n"
        f"  - src/ros-launch-resolve/target/{{release,debug}}/{name}\n"
        f"  - PATH"
    )


def main():
    """Main entry point - delegates to the Rust binary."""
    try:
        binary = _find_binary()
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    result = subprocess.run([binary] + sys.argv[1:])
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
