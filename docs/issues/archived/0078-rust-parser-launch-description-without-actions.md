---
id: 78
title: "The Rust parser refuses LaunchDescription() with no arguments and has no add_action: nav2_bringup's launch files stop it"
status: resolved
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

## A second cause in the same files

With `LaunchDescription()` fixed, the same three files stopped at

```
ValueError: invalid condition expression: Unexpected value 'not False',
expected one of: ['true', '1', 'false', '0']
```

The mock `PythonExpression` joined its parts into a string when it was
BUILT. `navigation_launch.py` builds
`IfCondition(PythonExpression(['not ', use_composition]))` inside
`generate_launch_description()`, before `use_composition` is declared, so
the string was `not $(var use_composition)`. `eval` refused it, and
`pyobject_to_string` fell back to `__str__`, which resolves the token: the
condition read `not False` as text. `launch` performs a `PythonExpression`'s
substitutions when the expression is performed.

## Fix

- `LaunchDescription(initial_entities=None, *, deprecated_reason=None)`:
  any iterable or none; `add_action`, `add_entity`, `entities`. The visitor
  reads the same list.
- `PythonExpression` keeps its parts (a list, a tuple, one substitution or a
  string; `python_modules` accepted) and performs them in `perform` and
  `__str__`.

Tests, each with the node set read off stock `launch` (Jazzy,
`tests/stock_parity/stock_oracle.py`), each failing before its fix:
`execution_semantics.rs` `a_description_filled_with_add_action_executes`
(fixture `add_action/`: `add_action`, `add_entity`, a tuple, a generator,
the keywords, a condition) and
`a_python_expression_is_performed_when_its_condition_is` (fixture
`python_expression/`). The parser and pyexec suites pass, clippy and
rustfmt are clean.

Jazzy's `nav2_bringup` through the fixed parser: `navigation_launch.py` 11
nodes, `localization_launch.py` 3, `bringup_launch.py` 1 container with 14
composable nodes, the same sets as stock.

## Workaround

`--parser python`.
