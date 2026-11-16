---
id: 91
title: "Under `--parser python`, `OnProcessExit(target_action=X, on_exit=Shutdown)` is still ignored (#0081 holds for the Rust parser only)"
status: open
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
