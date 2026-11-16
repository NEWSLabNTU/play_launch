---
id: 86
title: "The Rust parser's mock substitutions have no `perform()`, so `context.perform_substitution(EnvironmentVariable(...))` fails"
status: open
type: bug
severity: high
---

# 0086 - Mock substitutions lack `perform()`; Autoware 2.1.0 does not load

**Repo:** `play_launch` at `ed2b53f8` (installed 0.20.0)
**Affects:** `pyexec`, the mock `launch` module, and every Autoware 2.1.0
launch under the default Rust parser
**Found by:** the golf cart's Autoware 1.5.0 → 2.1.0 migration, 2026-11-17

## Symptom

    Error: Rust parser error while parsing <file>: Python error: Python error:
    AttributeError: 'launch.substitutions.EnvironmentVariable' object has no
    attribute 'perform'

Autoware 2.1.0's stock `autoware_launch planning_simulator.launch.xml` fails
this way before anything spawns, as does any stack built on 2.1.0. The
triggers are 2.1.0's new agnocast files, which almost every 2.1.0 launch
includes:

- `autoware_component_state_monitor/launch/component_state_monitor.launch.py:64`:
  `context.perform_substitution(EnvironmentVariable("ENABLE_AGNOCAST", default_value="0"))`
- `autoware_agnocast_wrapper/launch/agnocast_env.launch.py`: `EnvironmentVariable("LD_PRELOAD", default_value="")`
  and `EnvironmentVariable("ENABLE_AGNOCAST", ...)` as a `DeclareLaunchArgument` default.

`--parser python` loads the same launch: the stock planning simulator ran
97 entities (48 nodes, 8 containers, 41 composable nodes) for 4 minutes.

## Reproduction

```python
# env_perform.launch.py
from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from launch.substitutions import EnvironmentVariable


def _setup(context):
    enabled = context.perform_substitution(
        EnvironmentVariable("ENABLE_AGNOCAST", default_value="0"))
    return [LogInfo(msg="agnocast=" + enabled)]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
```

```
$ play_launch resolve env_perform.launch.py -o out.yaml            # rc=1, the error above
$ play_launch resolve --parser python env_perform.launch.py -o out.yaml   # rc=0
```

## Cause

`LaunchContextView::perform_substitution`
(`src/ros-launch-resolve/parser/crates/pyexec/src/api/utils.rs`) calls
`sub.perform(context)`. The mock `EnvironmentVariable` has no `perform`
method; it is only converted to a string where `pyobject_to_string` meets it.
The other mock substitutions (`LaunchConfiguration`, `FindPackageShare`,
`PathJoinSubstitution`, `TextSubstitution`, ...) probably have the same gap
when handed to `perform_substitution` or `perform_substitutions`: worth
checking all of them, not just this one.

Related: #0073 (user-defined `launch.Substitution` subclasses) is the other
half of "a substitution performed from Python".

## Direction

Give every mock substitution a `perform(context)` that resolves it the way
`pyobject_to_string` already does, the environment read from the
traverser's environment, including any `SetEnvironmentVariable` in scope.
Then add a test that loads the snippet above plus 2.1.0's
`agnocast_env.launch.py` pattern: a `DeclareLaunchArgument` whose default is
an `EnvironmentVariable`.

## Workaround

`--parser python`. The golf cart has to use it for 2.1.0 until this is fixed.
