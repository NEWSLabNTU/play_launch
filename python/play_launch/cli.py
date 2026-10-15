"""CLI entry point that dispatches to Rust binary."""

import os
import shutil
import signal
import subprocess
import sys
from pathlib import Path


def _is_self(path: str) -> bool:
    """Whether `path` is the script running now (the console entry point)."""
    try:
        return os.path.samefile(path, sys.argv[0])
    except OSError:
        return False


def _find_binary(name: str) -> str:
    """Find bundled binary with fallback to PATH.

    Resolution order:
    1. The wheel's payload for the selected ROS distribution (pip install
       layout, ``dist/<distro>/bin``; see ``payload.py``). A wheel that
       carries payloads but none for this environment is an error, never a
       fall-through to whatever ``play_launch`` is on PATH.
    2. ROS2 install paths (colcon build)
    3. PATH search (fallback)
    """
    # Imported here, not at module level: the package's __init__ imports this
    # module, and `python -m play_launch.payload` would otherwise find the
    # payload module already loaded and warn.
    from play_launch import payload

    # 1. The wheel's payload (pip install)
    if payload.carried():
        bundled = payload.select().bin_dir / name
        if bundled.exists() and os.access(bundled, os.X_OK):
            return str(bundled)
        raise FileNotFoundError(f"{name} is missing from the wheel's payload: {bundled}")

    # 2. Check ROS2 install locations (colcon build)
    ros2_paths = [
        Path("/opt/ros/humble/lib/play_launch") / name,
        Path("/usr/lib/play_launch") / name,
        Path("install/play_launch/lib/play_launch") / name,
    ]
    for ros2_path in ros2_paths:
        if ros2_path.exists() and os.access(ros2_path, os.X_OK):
            return str(ros2_path)

    # 3. Fall back to PATH, skipping this wrapper's own entry point: the
    # console script is also called `play_launch`, and exec'ing it re-enters
    # here until the environment overflows ("Argument list too long").
    path_binary = shutil.which(name)
    if path_binary and not _is_self(path_binary):
        return path_binary

    raise FileNotFoundError(
        f"{name} not found. Ensure play_launch is properly installed.\n"
        f"Checked locations:\n"
        f"  - {Path(__file__).parent / 'dist' / '<distro>' / 'bin' / name}\n"
        f"  - /opt/ros/humble/lib/play_launch/{name}\n"
        f"  - /usr/lib/play_launch/{name}\n"
        f"  - install/play_launch/lib/play_launch/{name}\n"
        f"  - PATH"
    )


def payload_env(binary: str) -> dict[str, str]:
    """The environment to run a bundled binary in.

    The payload's ``lib`` goes on LD_LIBRARY_PATH and its prefix on
    AMENT_PREFIX_PATH. Both reach the nodes play_launch spawns, and both have
    to: the container is found through the ament index, and rosidl finds
    play_launch_msgs' typesupport libraries through LD_LIBRARY_PATH in a
    process (the container) that play_launch did not link. An RPATH on our
    own binaries cannot serve either lookup.
    """
    env = os.environ.copy()
    prefix = Path(binary).resolve().parent.parent
    if not (prefix / "share" / "ament_index").is_dir():
        return env  # not a wheel payload (colcon install or PATH)
    lib_dir = str(prefix / "lib")
    ld_path = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = f"{lib_dir}:{ld_path}" if ld_path else lib_dir
    ament_path = env.get("AMENT_PREFIX_PATH", "")
    env["AMENT_PREFIX_PATH"] = f"{prefix}:{ament_path}" if ament_path else str(prefix)
    return env


def main():
    """Main entry point - delegates to Rust binary."""
    from play_launch.payload import PayloadError

    try:
        binary = _find_binary("play_launch")
    except (FileNotFoundError, PayloadError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    env = payload_env(binary)

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
