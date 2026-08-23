---
id: 22
title: "`<extra_arg>` is accepted as a child of `<composable_node>` and then discarded"
status: resolved
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

## Resolution

Fixed. The sketch above was wrong about the size of it — "the model and the
container already handle the rest" was true of the *types* and false of the
*code*. The value was dropped at **four** separate hops, each of which had to
be found by testing rather than reading:

1. **The XML child loop** (`actions/container.rs`) — `extra_args` initialised
   empty, `extra_arg` falling through to a debug-level "skipping" arm. The one
   hop the sketch predicted.
2. **The YAML frontend** (`traverser/yaml.rs`) — the same `HashMap::new()`,
   independently. Also a cross-parser divergence: the Python parser has always
   carried `extra_arguments`.
3. **`LoadNodeCapture`** (`captures.rs`) — no field at all. `<node_container>`
   builds its records directly, but a top-level `<load_composable_node>` goes
   through captures, so fixing hops 1 and 2 left that shape still dropping
   everything.
4. **`bridge.rs`'s capture → record conversion** — `extra_args: HashMap::new()`
   even once the capture carried them.

Hops 3 and 4 were found only because the test asserted BOTH container shapes.
Asserting one and assuming the other is how issue #7 shipped half-fixed, and
the same assumption would have shipped this one at 50%.

`<extra_arg>` also gained an `AttrSpec` (`name`, `value`), since without one
`spec_for` returns `None` and `check` exits before validating anything —
`<extra_arg nmae="x"/>` was accepted in silence.

Verified end to end: a launch file setting
`<extra_arg name="executor_threads" value="4"/>` now reaches the running
process, which starts with a 4-thread executor pool (14 threads) beside a
sibling that did not ask and gets 11.

`use_intra_process_comms`, the case this issue flagged as worth checking
first, was indeed unreachable from an XML or YAML launch file. It is now.
