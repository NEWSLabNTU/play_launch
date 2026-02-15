"""CLI entry point that dispatches to Rust binary."""

import os
import shutil
import subprocess
import sys
from pathlib import Path


def _find_binary(name: str) -> str:
    """Find bundled binary with fallback to PATH.

    Resolution order:
    1. Package bin directory (pip install layout)
    2. ROS2 install paths (colcon build)
    3. PATH search (fallback)
    """
    # 1. Check package bin directory (pip install)
    pkg_dir = Path(__file__).parent
    bundled = pkg_dir / "bin" / name
    if bundled.exists() and os.access(bundled, os.X_OK):
        return str(bundled)

    # 2. Check ROS2 install locations (colcon build)
    ros2_paths = [
        Path("/opt/ros/humble/lib/play_launch") / name,
        Path("/usr/lib/play_launch") / name,
        Path("install/play_launch/lib/play_launch") / name,
    ]
    for ros2_path in ros2_paths:
        if ros2_path.exists() and os.access(ros2_path, os.X_OK):
            return str(ros2_path)

    # 3. Fall back to PATH
    path_binary = shutil.which(name)
    if path_binary:
        return path_binary

    raise FileNotFoundError(
        f"{name} not found. Ensure play_launch is properly installed.\n"
        f"Checked locations:\n"
        f"  - {bundled}\n"
        f"  - /opt/ros/humble/lib/play_launch/{name}\n"
        f"  - /usr/lib/play_launch/{name}\n"
        f"  - install/play_launch/lib/play_launch/{name}\n"
        f"  - PATH"
    )


def main():
    """Main entry point - delegates to Rust binary."""
    try:
        binary = _find_binary("play_launch")
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    # Add bundled libraries to LD_LIBRARY_PATH
    pkg_dir = str(Path(__file__).parent)
    lib_dir = str(Path(__file__).parent / "lib")
    env = os.environ.copy()
    ld_path = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = f"{lib_dir}:{ld_path}" if ld_path else lib_dir

    # Add bundled directory to AMENT_PREFIX_PATH so ament_index finds play_launch_container
    ament_path = env.get("AMENT_PREFIX_PATH", "")
    env["AMENT_PREFIX_PATH"] = f"{pkg_dir}:{ament_path}" if ament_path else pkg_dir

    # Pass through all arguments
    result = subprocess.run([binary] + sys.argv[1:], env=env)
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
