"""CLI entry point that dispatches to Rust binary."""

import os
import shutil
import signal
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

    # Pass through all arguments.
    #
    # Ctrl-C delivers SIGINT to the whole foreground process group: the Rust
    # binary receives it directly and runs its staged shutdown (SIGINT, then
    # SIGTERM, then SIGKILL on repeat). This wrapper's only job is to keep
    # waiting for it -- subprocess.run() instead raises KeyboardInterrupt out
    # of the wait, dumping a Python traceback over the launcher's own shutdown
    # messages. Loop so a second or third Ctrl-C (force-terminate) is equally
    # quiet.
    #
    # But group delivery is a terminal behaviour, not a general one. Under
    # `systemd`, `KillMode=mixed` -- the default for a simple service -- signals
    # only the MainPID, which is THIS wrapper, not the binary it spawned. With
    # nothing forwarding, the Rust binary never learns it should shut down: the
    # unit sits until TimeoutStopSec, gets SIGKILLed, and ends in `failed`.
    # Measured on an aarch64 robot: every `systemctl --user stop` took the full
    # 90 s and the nodes were killed rather than asked to exit, which for a
    # vehicle means skipping the shutdown path that stops its motors.
    #
    # So forward explicitly instead of relying on the group. Forwarding is
    # idempotent with group delivery -- in a terminal the binary gets the signal
    # twice, and its staged shutdown already treats a repeat as
    # "force-terminate", which is what a second Ctrl-C means anyway.
    proc = subprocess.Popen([binary] + sys.argv[1:], env=env)

    def _forward(signum, _frame):
        try:
            proc.send_signal(signum)
        except (ProcessLookupError, OSError):
            pass  # already gone; nothing to forward to

    previous = {}
    for _sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            previous[_sig] = signal.signal(_sig, _forward)
        except (ValueError, OSError):
            pass  # not the main thread, or the platform lacks it

    try:
        while True:
            try:
                rc = proc.wait()
                break
            except KeyboardInterrupt:
                continue
    finally:
        for _sig, _handler in previous.items():
            try:
                signal.signal(_sig, _handler)
            except (ValueError, OSError):
                pass
    # A child killed by signal N reports -N; map it to the conventional
    # 128+N shell exit status rather than passing a negative to sys.exit().
    sys.exit(128 - rc if rc < 0 else rc)


if __name__ == "__main__":
    main()
