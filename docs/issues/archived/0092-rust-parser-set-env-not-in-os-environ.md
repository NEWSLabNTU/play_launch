---
id: 92
title: "Rust parser: `SetEnvironmentVariable` earlier in a launch is not visible to `os.environ` in a later `OpaqueFunction`; `ros2 launch` and `--parser python` see it"
status: resolved
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

## Resolution (2026-11-17)

Under `launch`, `context.environment` is `os.environ`: an environment action
writes it, and a launch file's own `os.environ[...] = ...` is part of the
environment later processes start with. The Python frontend now keeps the
two as one (`pyexec/src/api/visit.rs::sync_os_environ`), before each entity
executes and before a file's own code runs:

- what Python wrote to `os.environ` since the last sync is recorded in the
  context, in the scope in effect;
- every set and unset the context holds, scoped as it scopes them, is written
  to `os.environ`, so an `OpaqueFunction` reads what a
  `SetEnvironmentVariable` before it set, a `.launch.py` reads at its top
  level what an including XML file's `<set_env>` set, and a variable whose
  `GroupAction` ended goes back to its old value;
- the outermost execution snapshots `os.environ` first and restores it at
  the end, since the parser runs in play_launch's own process.

Two more defects surfaced in the fixture:

- **`env_unset` read the live process environment.** `resolve_spawn` kept
  an unset only for a name `std::env` had, and the sync had just removed it.
  It now reads the environment the launch started with
  (`actions::process::set_launcher_environment`).
- **A dropped action's address was reused.** The walk recognised an action
  it had already executed by its object address and did not hold the
  object. An action an `OpaqueFunction` returned was freed after it ran,
  and the next one built at that address was discarded as "executed more
  than once" (logged as `Unsupported action type: timer`). Three
  `OpaqueFunction`s calling one function gave one node where stock gives
  three. The frame now holds every object whose address it keys on.

Tests: `execution_semantics.rs::environment_actions_write_os_environ`
(fixture `execution_semantics/env_os_environ`: an XML includer's
`<set_env>`, a set before an `OpaqueFunction`, a scoped set and unset of an
inherited `HOME`, a direct `os.environ` write; it also checks nothing leaks
into the test process) and the stock-parity case
`tests/stock_parity/cases/env_os_environ`, which compares each node's `env`
and `env_unset` with stock Humble. With the source change reverted, the test
fails with `["/a1_none_home_d", "/top_none"]`.
