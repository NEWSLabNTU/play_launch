---
id: 70
title: "The Python frontend drops `respawn` / `respawn_delay` on `Node` and `ExecuteProcess`"
status: open
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
