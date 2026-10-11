---
id: 64
title: "`on_exit` shutdown is honoured only for `Node`/`ExecuteProcess` in a `.launch.py`; XML, YAML, `LifecycleNode` and containers still drop it"
status: resolved
type: defect
severity: medium
---

# 0064 - `on_exit` shutdown is honoured on one path only

**Repo:** `play_launch` at `785cddb7`
**Affects:** `src/ros-launch-resolve/parser/crates/play_launch_parser/src/xml/attr_spec.rs:94`
and `:168`; `actions/node.rs:301`, `actions/executable.rs:151` and
`actions/container.rs:430` (all hard-code `on_exit_shutdown: None`); pyexec's
`LifecycleNode` and `ComposableNodeContainer` captures
**Found:** review of the commits pulled 2026-10-11, confirmed by reading.

## What happens

fcd2188a's title is "the Rust parser honours on_exit=Shutdown, as ros2 launch
does". It does so for `launch_ros.actions.Node` and `ExecuteProcess` in a
`.launch.py` (pyexec `node.rs`, `on_exit_kwarg_shuts_down`). Everywhere else
the declaration is still dropped:

- **XML and YAML** `<node on_exit="shutdown">`, `<executable on_exit=...>` and
  `<node_container on_exit=...>`: `attr_spec.rs` lists `on_exit` as
  known-unsupported, so a warning is printed and the attribute is ignored.
  Stock launch honours it (`ExecuteProcess.parse`, reached from `Node.parse`
  via `super().parse(...)`, `launch_ros/actions/node.py:309`). The comment at
  `attr_spec.rs:87` records exactly that and says "Not implemented here".
- **`LifecycleNode(on_exit=Shutdown())`** in a `.launch.py`: the capture goes
  through `capture`, not `capture_with`, so the flag never reaches the record.
- **`ComposableNodeContainer(on_exit=...)`**: not read.

A launch written in XML that marks its test runner as required keeps running
after the runner exits. That is the bug fcd2188a fixed, on the frontends it
did not touch.

## Fix

Parse `on_exit` in the XML/YAML actions, accepting the frontend's value
`shutdown`. Any other value stays a warning, as stock launch only defines that
one. Set `on_exit_shutdown` from it, and remove `on_exit` from the
known-unsupported lists. In pyexec, route `LifecycleNode` and
`ComposableNodeContainer` through `capture_with`. Containers also need the
builder to register them: `add_container` has no equivalent of the check at
`builder.rs:145`.

Depends on #0065 for correct matching, and on #0063 for `up`.

## Verification, when fixed

Extend `tests/tests/on_exit_shutdown.rs` with an XML fixture, a YAML fixture
and a `LifecycleNode`/container case, each under the existing four checks.
Add a stock-parity case so the differential gate covers the attribute.

## Resolution (2026-10-11)

- **XML and YAML:** `<node>`, `<executable>` and `<node_container>` read
  `on_exit` through one `parse_on_exit`. `shutdown` sets the flag; any other
  value is an error with stock launch's own message
  (`ParseError::UnsupportedOnExit`). `on_exit` moved from known-unsupported to
  supported in `attr_spec.rs`.
- **`.launch.py`:** `LifecycleNode` and `ComposableNodeContainer` pass their
  `on_exit` through `capture_with`. The flag-less `capture` is deleted, so no
  path can drop it again.
- **Containers:** `ContainerCapture`, `ComposableNodeContainerRecord` and the
  resolver's mirror carry it. The builder registers a container whose record
  has it, as it does a node.
- **pyexec C ABI 7 -> 8:** the captures' new serde-defaulted field would let
  a v7 object answer silently wrong.

Tests: `play_launch_parser/tests/on_exit_tests.rs` (XML and YAML, all three
elements, and the refusal of other values), and `on_exit_shutdown.rs::xml_on_exit_shutdown`
end to end.
