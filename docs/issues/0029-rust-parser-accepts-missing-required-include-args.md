---
id: 29
title: "Rust parser satisfies an include's required argument from the parent scope; ros2 launch and the Python parser refuse"
status: open
type: parity
severity: low
---

# 0029 — required include arguments are not required under the Rust parser

**Repo:** `play_launch` (`src/ros-launch-resolve/parser`, include handling)
**Affects:** the Rust parser (the default) on any `<include>` of a file that declares
an `<arg>` without a default

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

## Suggested fix

In the include traverser, after loading the included file, collect its `<arg>`
declarations without `default=` and error unless each is named in the include's
`<arg>` list, with the same message shape as launch's. Fixture: a two-file pair, the
inner declaring `<arg name="required"/>`, the outer declaring `required` at top level
and including without passing it; assert the Rust parser errors and names the
argument. The differential attribute tests already know how to run the same file
through `ros2 launch` as the oracle.
