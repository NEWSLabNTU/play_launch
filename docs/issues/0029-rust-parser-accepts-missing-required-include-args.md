---
id: 29
title: "Rust parser satisfies an include's required argument from the parent scope; ros2 launch and the Python parser refuse"
status: resolved
type: parity
severity: low
---

# 0029 — required include arguments are not required under the Rust parser

**Repo:** `play_launch` (`src/ros-launch-resolve/parser`, include handling)
**Affects:** the Rust parser (the default) on any `<include>` of a file that declares
an `<arg>` without a default
**Resolved:** 2026-09-11

## What happens

`autoware_launch/launch/components/tier4_simulator_component.launch.xml` declares
thirteen arguments without defaults and forwards them with `$(var ...)`. A file that
includes it and passes only five, but *declares* the rest at its own top level:

- stock `ros2 launch`: `Included launch description missing required argument
  'perception/enable_detection_failure' (description: ...), given: [...]`
- `--parser python`: the same error, since it drives the real launch frontend
- `--parser rust`: resolves, 124 nodes, no diagnostic

Found 2026-09-11 on `NEWSLabNTU/2026-golf-cart`'s `aruco_planning_sim.launch.xml`
(fixed there by passing every argument explicitly, known-config-defects #9). The Rust
parser found each name in the enclosing scope and carried on.

## Why it matters

The repo's rule is that Python's behaviour is the reference and Rust is fixed to
match. Here the divergence is in the lenient direction: a launch file that the Rust
parser resolves will fail under `ros2 launch`, and `play_launch launch` would bring up
a stack that the stock tool refuses to start. A user who validates with play_launch
alone ships a broken launch file.

`launch`'s rule (`IncludeLaunchDescription.execute`): every `DeclareLaunchArgument`
of the included description that has no default and is not `conditionally_declared`
must appear in the include's own `launch_arguments`; the parent's launch
configurations do not count.

## Fix

`traverser/include.rs`: before an included XML or YAML file is traversed, its
required arguments are collected — every `<arg>` (or `- arg:`) without a default,
walking into elements that carry no `if`/`unless` and never into a nested `<include>`,
which is launch's `get_launch_arguments` rule: a declaration under a condition is
*conditionally included* and checked only if it executes, and a nested include is its
own description. Each must be named among the include's own `<arg>`s; the parent's
scope is not consulted, because launch does not consult it. The error is
`ParseError::MissingIncludeArgument`, worded as launch words it:

```
Included launch description missing required argument 'required' (description: 'must be passed on the include'), given: [] — in .../test_required_arg_inner.launch.xml
```

Fixtures `test_required_arg_{inner,outer_missing,outer_passing}.launch.xml` and
`test_required_arg_inner.launch.yaml` with `test_required_arg_outer_missing_yaml.launch.xml`;
three tests in `xml_tests.rs` cover missing (with the parent declaring the name,
which must not rescue it), passed, and the YAML frontend. The conditionally declared
argument in the inner fixture is asserted absent from the message.

Verified on the golf cart: the pre-fix `aruco_planning_sim.launch.xml` (before
`c584b5b`) is refused with the message above naming `perception/enable_detection_failure`,
the fixed one and every other entry point resolve as before.

Not covered: a `.launch.py` include. Its `IncludeLaunchDescription` runs inside the
embedded launch API mocks, which do not implement this check either; a separate item
if it bites.
