---
id: 21
title: "The Rust parser gives a composable node the CONTAINER's namespace; ROS 2 gives it the launch context's"
status: open
type: correctness
severity: high
---

# 0021 — composable nodes land in the wrong namespace when a container declares its own

**Repo:** `play_launch`
**Affects:** `src/ros-launch-resolve/parser/crates/play_launch_parser/src/actions/container.rs:464`
**Applies to:** `--parser rust`, which is the DEFAULT
**Related:** #0017 (model FQN ≠ ROS graph name)

## Symptom

First cross-parser comparison on a real corpus (the golf cart stack, 145
nodes, made runnable by #0020) matches 137/137 nodes functionally, and finds
8 whose FQNs disagree:

    Rust:   /system/system_monitor/system_monitor/cpu_monitor
    Python: /system/system_monitor/cpu_monitor

`system_monitor` twice. Same for `gpu_monitor`, `hdd_monitor`, `mem_monitor`,
`net_monitor`, `ntp_monitor`, `process_monitor`, `voltage_monitor` — every
composable node in `autoware_system_monitor`'s container, and the only
container in the stack that declares a `namespace=` attribute of its own.

## Ground truth

Not decided by the parity rule alone. `tmp/nsprobe/nsprobe.launch.xml`
reproduces the shape — a container with its own `namespace=`, nested inside a
pushed namespace, holding a composable node that declares none — and stock
`ros2 launch` was asked directly:

    $ ros2 node list
    /outer/probe/child_one
    /outer/probe/probe/probe_container

So ROS 2 applies the container's `namespace=` to **the container process
only**. A composable node with no namespace of its own takes the LAUNCH
CONTEXT namespace (`/outer/probe`), not the container's
(`/outer/probe/probe`). The Python parser agrees. The Rust parser emits
`/outer/probe/probe/child_one`.

That the container itself is right in both parsers is what kept this hidden:
container counts match, so entity-count parity passes and only an FQN-level
comparison shows it.

## Cause

`actions/container.rs:464` — when a `<composable_node>` declares no
`namespace=`, it inherits the container's already-resolved namespace:

- `container.rs:215` resolves the container's namespace correctly (context +
  its own attribute).
- `container.rs:449-465` then uses that resolved value as the composable
  node's namespace when the node declares none.
- The container never pushes its namespace onto the context
  (`traverser/entity.rs:240` passes the context through unchanged), so the
  context still holds the right value — it is simply not the one consulted.

Where a container has no `namespace=` of its own, the two are equal and
nothing is visibly wrong, which is every other container in this stack.

## Why it matters

- The **default** parser produces wrong node names, silently. Nothing errors;
  the model is internally consistent and merely describes a different system
  than the one that will run.
- Anything joining model identity to the live ROS graph — the startup gate,
  the web UI, contract and scheduling application keyed by FQN — misses these
  8 nodes. This is very likely a subset of #0017's 19 unmatched nodes, and
  unlike the rest of that issue it has a definite cause.
- A per-node scheduling override or contract written against the true name
  would silently apply to nothing.

## Reproducing

    ros2 launch tmp/nsprobe/nsprobe.launch.xml     # then: ros2 node list
    ros-launch-resolve resolve --parser rust -o /tmp/m.yaml tmp/nsprobe/nsprobe.launch.xml

## Fix sketch

A composable node with no `namespace=` should take the launch-context
namespace, not the container's resolved namespace. An explicit `namespace=`
on the composable node keeps resolving against the context as it does now.

Wanted with it: the probe promoted out of `tmp/` into a fixture, so the case
is covered by the differential test rather than by a comparison someone
happens to run.
