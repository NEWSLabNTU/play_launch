---
id: 70
title: "The Python frontend drops `respawn` / `respawn_delay` on `Node` and `ExecuteProcess`"
status: resolved
type: defect
severity: medium
---

# 0070 - The Python frontend drops `respawn`

**Repo:** `play_launch` at `150536c4`
**Affects:** `src/ros-launch-resolve/parser/crates/play_launch_parser/src/bridge.rs`
(captures to records), the pyexec mocks for `Node` and `ExecuteProcess`
**Found:** 2026-10-13, while fixing #0069.

## What happens

A `.launch.py` declaring `Node(..., respawn=True, respawn_delay=1.0)` or
`ExecuteProcess(..., respawn=True)` resolves to a model with no `respawn`,
so play_launch does not restart the process when it dies. `ros2 launch`
does. `bridge.rs` writes `respawn: None` for every Python-captured node and
executable; nothing in the pyexec mocks reads the argument.

The XML and YAML frontends carry it for `<node>` and, since #0069, for
`<executable>`.

## Fix direction

Record `respawn` / `respawn_delay` in the `Node` and `ExecuteProcess` mocks'
captures (as substitutions, resolved when the action executes) and carry them
through `NodeCapture` and `bridge.rs` into the record. Test against stock
`ros2 launch`, as the rest of the Python semantics are.

## Fix

The `Node`, `LifecycleNode`, `ComposableNodeContainer` and `ExecuteProcess`
mocks read `respawn` / `respawn_delay` from their `**kwargs` when constructed
(`RespawnArgs` in `pyexec/src/api/launch_ros/node.rs`) and resolve them when
the action executes, as `ExecuteLocal` does: `respawn` is
`perform_typed_substitution(context, respawn, bool)` (a bool as is, a
substitution performed and coerced, a value launch would refuse raises), and
`respawn_delay` is a float kept as given. `NodeCapture` and `ContainerCapture`
carry both, and `bridge.rs` puts them on the record instead of `None` (and,
for containers, instead of a hard-coded `Some(false)`).

What is recorded follows the Python dumper (`dump/visitor/node.py`,
`bare_execute_process.py`, `composable_node_container.py`), which the parity
gate compares against exactly: `respawn` is `true` or absent (false and
unset are both absent), and a bare `ExecuteProcess` keeps its delay only
alongside `respawn`.

The fields are serde-defaulted, so a v9 pyexec object would answer without
them and the bug would come back in silence: **C ABI 9 -> 10**, both sides
(`pyexec/src/c_abi.rs`, `pyload/src/lib.rs`).

Tests: `tests/executable_respawn_tests.rs` in the parser crate, a `Node` with
a literal, an `ExecuteProcess` with a `LaunchConfiguration`, a container, and
false/unset staying absent; a value launch cannot coerce is refused. Checked
as a negative control (the kwargs read disabled: both Python tests fail).
End to end, `play_launch resolve` on such a `.launch.py` writes
`respawn: true` / `respawn_delay` onto the node and the process in the model.
