---
id: 22
title: "`<extra_arg>` is accepted as a child of `<composable_node>` and then discarded"
status: open
type: correctness
severity: medium
---

# 0022 — `<extra_arg>` parses, validates, and reaches nothing

**Repo:** `play_launch`
**Affects:** `src/ros-launch-resolve/parser/crates/play_launch_parser/src/actions/container.rs:362`

## What happens

`extra_arg` is listed as an allowed child of `<composable_node>` in
`xml/attr_spec.rs`, so a launch file using it validates cleanly. The composable
action then initialises

```rust
let extra_args = HashMap::new();
```

and never fills it from those children — the child loop handles `param` and
`remap` only. The field is carried faithfully the rest of the way (action →
record → `NodeInstance.extra_args` → LoadNode `extra_arguments`), so everything
downstream is correct and every value is empty.

Measured: a launch file with

```xml
<composable_node pkg="composition" plugin="composition::Talker" name="t1">
  <extra_arg name="executor_threads" value="4"/>
</composable_node>
```

resolves to a model with no `extra_args` for that node.

## Why it matters

This is the shape of issue #0007: an attribute the parser accepts, and then
ignores, with no warning. The author gets no signal, and the only way to learn
it did nothing is to inspect the running system.

`use_intra_process_comms` is the case worth checking first. CLAUDE.md documents
the container as extracting it from `extra_arguments` and forwarding
`--use-intra-process-comms`, and the container end does exactly that — but if
the XML path never populates `extra_args`, a launch file cannot turn
intra-process comms on through `<extra_arg>` either. Whether that is reachable
by some other route (the Python parser, the launch API) has not been checked.

## Found by

Phase 66 follow-up work. `component_node` gained `--executor-threads` as an
override for a pool it otherwise derives from callback groups; forwarding it
from `<extra_arg>` was written, found to be a branch that could never execute,
and removed rather than shipped.

## Fix sketch

Read `extra_arg` children in the same loop as `param`/`remap`, keyed
`name` → `value`. The model and the container already handle the rest. Add a
fixture asserting a launch-file `<extra_arg>` reaches
`NodeInstance.extra_args`, since every layer except the first already claims to
support it.
