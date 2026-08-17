---
id: 21
title: "The Rust parser gives a composable node the CONTAINER's namespace; ROS 2 gives it the launch context's"
status: resolved
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
- A per-node scheduling override or contract written against the true name
  silently applies to nothing.
- **Not** a subset of #0017's 19 unmatched nodes, which was the first guess and
  is wrong — see below. It is worse than that: nothing was unmatched, because
  the running system was moved to agree with the model.

## It changed the running system, not just the artifact

Worth stating plainly, because "the model has a wrong label" and "the node
comes up in the wrong namespace" are very different severities.

`LoadNodeRecord.namespace` is what play_launch puts in the LoadNode request,
so the composable really did come up at the doubled name. The evidence is
#0017's own measurement: on a live golf cart run it compared all 144 model
FQNs against `ros2 node list` and found exactly 19 unmatched — 17 renamed
un-named nodes and 2 genuinely absent. None of the 8 `system_monitor`
composables were among them. Their doubled FQNs matched the graph because the
graph had the doubled names too.

So under play_launch those nodes ran at
`/system/system_monitor/system_monitor/cpu_monitor` while `ros2 launch` puts
them at `/system/system_monitor/cpu_monitor` — a different topology, with
every relative topic, parameter and remapping shifted with them. Anything
outside the container addressing them by name found nothing. This is also why
no graph-based check could catch it: the model and the runtime were wrong
together, and only stock ROS 2 disagreed.

## Fix

`actions/container.rs` — a composable node with no `namespace=` now takes
`context.current_namespace()`. An explicit `namespace=` keeps resolving
against the context, as before, and `container_namespace` is still used for
`target_container_name`, which is genuinely the container's own name.

`tests/fixtures/launch/test_container_namespace_scope.launch.xml` covers both
shapes (relative under pushed namespaces, absolute at root) with the
expectations read off stock `ros2 launch` rather than derived.

One existing expectation had to change:
`test_load_composable_node` asserted `/test` for a composable in a container
declared `namespace="/test"` at the root context. Real ROS 2 says `/`
(`Loaded node '/abs_child' in container '/test/abs_container'`), so the test
had encoded the bug.

## Verified

- Parser suite: 470 tests, plus IR and the resolve crate — clean.
- End-to-end under play_launch, same probe as the ground-truth run:
  `/outer/probe/child_one` and `/outer/probe/probe/probe_container`, identical
  to stock `ros2 launch`.
- **Golf cart stack, cross-parser: 145/145 nodes equivalent, PASS** — up from
  137/145. First clean parity result on a real corpus.
