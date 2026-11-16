---
id: 92
title: "Rust parser: `SetEnvironmentVariable` earlier in a launch is not visible to `os.environ` in a later `OpaqueFunction`; `ros2 launch` and `--parser python` see it"
status: open
type: defect
severity: medium
---

# 0092 - `SetEnvironmentVariable` does not reach `os.environ` during the Rust parse

**Repo:** `play_launch` 0.35.0 and 0.36.0 (PyPI wheel), Rust parser (`pyexec`)
**Affects:** the `SetEnvironmentVariable` mock and the environment an `OpaqueFunction` runs in
**Found:** scenario-world-bridge, roadmap 077 (its issue 044), 2026-11-17: a launch-time check
of `DISPLAY` read the environment after the launch had set it, and saw the old value.

## Principle

play_launch should behave as `ros2 launch` does wherever `ros2 launch` defines the behaviour.

## What `ros2 launch` does (Humble)

`SetEnvironmentVariable(name, value)` writes `os.environ` (and `context.environment`) when it
is executed, so an `OpaqueFunction` later in the same launch reads the new value with
`os.environ.get(name)`.

## What play_launch does

Under the Rust parser the later `OpaqueFunction` reads `None`; `--parser python` reads the
value. (Not to be confused with #0077, which was about the dumper passing `set_env` to nodes.)

## Repro

```python
# env_set_then_read.launch.py
import os
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, SetEnvironmentVariable

def _read(_context):
    return [LogInfo(msg="PL_SET=" + str(os.environ.get("PL_SET")))]

def generate_launch_description():
    return LaunchDescription([SetEnvironmentVariable("PL_SET", "7"), OpaqueFunction(function=_read)])
```

| launcher | logs |
|---|---|
| `ros2 launch` (Humble) | `PL_SET=7` |
| play_launch 0.35.0 / 0.36.0, Rust parser | `PL_SET=None` |
| play_launch 0.35.0 / 0.36.0, `--parser python` | `PL_SET=7` |

## Why it matters

Launch files commonly set a variable and branch on it later (a display, an RMW, a domain).
Under the Rust parser such a branch takes the other path, silently.

## Proposed fix

When the parser executes `SetEnvironmentVariable` (and `UnsetEnvironmentVariable`,
`AppendEnvironmentVariable`), apply it to the parser process's `os.environ` as well as
`context.environment`, scoped as launch scopes it (restored at the end of a `GroupAction`
with `scoped=True`). Test: the repro logs `PL_SET=7` under both parsers.
