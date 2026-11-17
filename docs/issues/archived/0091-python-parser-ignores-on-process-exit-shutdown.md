---
id: 91
title: "Under `--parser python`, `OnProcessExit(target_action=X, on_exit=Shutdown)` is still ignored (#0081 holds for the Rust parser only)"
status: resolved
type: defect
severity: medium
---

# 0091 - `--parser python` ignores the #0081 shutdown handler

**Repo:** `play_launch` 0.35.0 and 0.36.0 (PyPI wheel), `--parser python`
**Affects:** the Python dumper's handling of `RegisterEventHandler(OnProcessExit(...))`;
#0081's resolution reached the Rust parser (`pyexec`, `visit.rs` sets `on_exit_shutdown`)
**Found:** scenario-world-bridge, roadmap 077 (its issue 044), 2026-11-17, checking #0081
on both parsers.

## Principle

play_launch should behave as `ros2 launch` does, and its two parsers should agree.

## What `ros2 launch` does (Humble)

`RegisterEventHandler(OnProcessExit(target_action=X, on_exit=[EmitEvent(event=Shutdown())]))`:
when X exits the launch shuts down, whatever X's `respawn`.

## What play_launch does

With the Rust parser (0.35.0, 0.36.0) X is marked required and the launch ends after one
exit, as #0081 intended. With `--parser python` the handler is dropped and X respawns.

## Repro

```python
# exit2_unconditional.launch.py
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    stub = ExecuteProcess(cmd=["python3", "-c", "import sys; sys.exit(2)"], name="stub",
                          respawn=True, respawn_delay=2.0, output="screen")
    return LaunchDescription([stub, RegisterEventHandler(OnProcessExit(
        target_action=stub, on_exit=[EmitEvent(event=Shutdown())]))])
```

```bash
timeout 20 play_launch launch --disable-web-ui --log-dir log ./exit2_unconditional.launch.py
timeout 20 play_launch launch --parser python --disable-web-ui --log-dir log ./exit2_unconditional.launch.py
```

| launcher | starts in 20 s |
|---|---|
| `ros2 launch` (Humble) | 1, then the launch shuts down |
| play_launch 0.35.0 / 0.36.0, Rust parser | 1, then the launch shuts down |
| play_launch 0.35.0 / 0.36.0, `--parser python` | 10 |

## Why it matters

`--parser python` is the fallback the Rust parser's own errors recommend; a launch that ends
correctly under one parser loops under the other.

## Proposed fix

Apply #0081's rule in the Python dumper: an `OnProcessExit` whose `on_exit` is, or emits, a
shutdown sets the target's `on_exit_shutdown` in the dump. Add the repro to the parity tests
that run both parsers.

## Resolution (2026-11-17)

The Python dumper has applied #0081's rule since 0.31.0
(`dump/visitor/action.py::_note_exit_shutdown_handler`), and
`tests/tests/on_exit_shutdown.rs` covers it under `--parser python`, both
orders. The repro starts the stub once on main and with the PyPI 0.36.0
wheel installed fresh in a venv.

What the reporter ran was a 0.30.0 dumper. This account has a play_launch
0.30.0 Python package in `~/.local` (`pip install --user`), and Python puts
the user site ahead of the system's site-packages, so a newer wheel's binary
imported `play_launch.dump` from 0.30.0, which predates #0081. The import-time
check (#0020) compares only the dump FORMAT version, which no release
between 0.30.0 and 0.36.0 changed: the record's shape was the same and only
its behaviour differed.

Fix: a binary installed from a wheel (`<site-packages>/play_launch/dist/
<distro>/bin/`) puts that `site-packages` first on `sys.path` and drops any
`play_launch` module imported before, so it runs the Python package it was
shipped with (`resolve/src/python/python_bridge.rs::wheel_package_root`).
A colcon `install/` is unaffected; there `PYTHONPATH` decides, as before.

Verified with a wheel-layout copy of the binary and the stale 0.30.0
package in `~/.local`: the install-layout binary refuses it ("too old"),
the wheel-layout binary resolves the repro with `on_exit: shutdown`.
Unit tests: `python_bridge::tests` (the wheel layout is found; a colcon
layout, or the shape without the package beside it, is not).
