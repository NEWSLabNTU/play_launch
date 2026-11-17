---
id: 96
title: "The Rust parser accepts an untyped substituted parameter value that ros2 launch refuses as YAML"
status: resolved
type: bug
severity: medium
---

# 0096 - an untyped substituted parameter value YAML cannot read is accepted

**Repo:** `play_launch` 0.39.0
**Reported:** 2026-11-18, AutoSDV on Autoware 2.1.0 (car A)

## What happened

`tier4_vehicle_launch/vehicle.launch.xml` passes
`robot_description="$(command 'xacro ...')"` with no `type`. AutoSDV's car A
URDF had a comment holding `: `. `ros2 launch` and `--parser python` refused
the launch:

```
Failed to convert '<?xml ...' using yaml rules: yaml.safe_load() failed
mapping values are not allowed here
```

The default parser produced a model, so `play_launch` would have started a
launch that `ros2 launch` refuses.

## Cause

`launch` coerces a performed parameter value with
`coerce_to_type(..., can_be_str=False)`: with no type, a YAML error is an
error. Only literal text in an XML or YAML launch file is read with
`can_be_str=True` and kept as a string. `param_value::yaml_value` treated
every value as the literal case.

Checked against stock `ros2 launch` (Humble), each case:

| value | stock |
|---|---|
| XML `value="$(command 'cat r.urdf')"`, no type | refused |
| the same with `type="str"` | accepted |
| the same text written literally in XML | accepted |
| Python `{'robot_description': Command([...])}` | refused (`Unable to parse the value of parameter robot_description as yaml`) |
| Python `ParameterValue(Command([...]))`, no type | refused |
| Python plain string, in a dict or `ParameterValue` | accepted |

## Fix

`param_value::yaml_value_strict` fails on a YAML error. It is used for an
untyped XML/YAML `<param value>` holding any substitution
(`actions/node.rs`), a substitution in a Python parameter dict, and a
`ParameterValue` with no `value_type` around a substitution. A
`ParameterValue` in a parameter dict is now performed directly, since
`pyobject_to_string` falls back on `__str__` when `perform` fails and hid
the refusal. Tests: `play_launch_parser/tests/param_yaml_errors.rs`, which
fail without the fix.

The YAML reader is `serde_yaml_ng` (YAML 1.2), not PyYAML, so a text one of
them refuses and the other accepts would still diverge; `mapping values are
not allowed here`, the reported case, is refused by both.
