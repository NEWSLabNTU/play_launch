---
id: 78
title: "The Rust parser refuses LaunchDescription() with no arguments and has no add_action: nav2_bringup's launch files stop it"
status: open
type: defect
severity: high
---

# 0078 - `LaunchDescription()` and `add_action` are missing from the mock

**Repo:** `play_launch` at `3e6eba93` (seen on 0.20.0)
**Affects:** `src/ros-launch-resolve/parser/crates/pyexec/src/api/launch.rs`

## Symptom

```
Error: Rust parser error while parsing ceilbot_operations: Python error:
Python error: TypeError: LaunchDescription.__new__() missing 1 required
positional argument: 'actions'
```

A robot's launch that includes `nav2_bringup`'s `navigation_launch.py`
(Jazzy) stops the default parser; `--parser python` and `ros2 launch` load it.

## Cause

The mock `launch.LaunchDescription` takes one required `actions` list and has
no other method. Stock `launch`'s signature is
`LaunchDescription(initial_entities=None, *, deprecated_reason=None)`, and a
common way to write a launch file builds the description empty and fills it:

```python
ld = LaunchDescription()
ld.add_action(declare_namespace_cmd)
...
return ld
```

In Jazzy's `/opt/ros/jazzy/share/*/launch/*.py`, 27 files construct
`LaunchDescription()`, `nav2_bringup`'s `bringup_launch.py`,
`navigation_launch.py`, `localization_launch.py` and `slam_launch.py` among
them.

## Fix direction

Match stock: `initial_entities` optional (any iterable), keyword
`deprecated_reason`, `add_action`, `add_entity`, and the `entities` property;
the visitor keeps reading the same list. A fixture under
`tests/fixtures/execution_semantics/` that builds its description with
`add_action`, checked in `execution_semantics.rs`.

## Workaround

`--parser python`.
