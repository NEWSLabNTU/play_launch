---
id: 81
title: "`RegisterEventHandler(OnProcessExit(target_action=X, on_exit=Shutdown))` is ignored; `ros2 launch` ends when X exits"
status: resolved
type: defect
severity: medium
---

# 0081 - an `OnProcessExit` shutdown handler is ignored

**Repo:** `play_launch` at `0cba7d39` (0.30.0)
**Affects:** pyexec `api/actions/configuration.rs` (`RegisterEventHandler`),
`api/event_handlers.rs` (`OnProcessExit`); the Python dumper's
`visitor/action.py`
**Found:** carla-scenario-bridge, 2026-11-08. Also GitHub
NEWSLabNTU/play_launch#19.

## What happens

```python
short = ExecuteProcess(cmd=["bash", "-c", "sleep 2"], name="short")
LaunchDescription([
    ExecuteProcess(cmd=["bash", "-c", "sleep 30"], name="long"),
    short,
    RegisterEventHandler(OnProcessExit(
        target_action=short,
        on_exit=[EmitEvent(event=Shutdown(reason="short ended"))])),
])
```

`ros2 launch` ends after 2 s, when `short` exits. `play_launch launch`
under either parser runs until `long` ends by itself.

This is the event-handler spelling of `short(on_exit=Shutdown())`, which
#0063-#0065 made work. pyexec's `RegisterEventHandler` executed only the
start-up handlers (`OnProcessStart`, `OnStateTransition`) and dropped
`OnProcessExit`; the Python dumper ran the real `RegisterEventHandler`,
which registers the handler in a context that never sees a process exit,
and recorded nothing.

## Resolution (2026-11-08)

An `OnProcessExit` handler whose `on_exit` shuts the launch down
(`launch.actions.Shutdown`, or an `EmitEvent` of `launch.events.Shutdown`
such as SSv2's `ShutdownOnce`, the same test as `on_exit=`) sets its
target's `on_exit_shutdown`, so the target is required exactly as with
`on_exit=Shutdown()`. Everything downstream (model `on_exit: shutdown`,
`launch`, `up`) is unchanged.

- **Rust parser (pyexec):** the walk notes the capture range each process
  action (`ExecuteProcess`, `Node`, `LifecycleNode`,
  `ComposableNodeContainer`) produces, by object identity. A handler for a
  target already executed marks that range; one registered first is
  remembered and applied when the target executes. If an include came
  between the target and the handler, the captures have been handed to
  the traverser and a warning says the exit will not end the launch.
  `OnProcessExit.on_exit` now accepts a single action, as `launch` does
  (it was extracted as a list).
- **Python dumper:** the same two-sided bookkeeping on `LaunchDump`
  (`note_process`, `shutdown_on_exit_of`), driven from
  `visit_action_by_class`. `ComposableNodeContainerRecord` gains
  `on_exit_shutdown`, which the Rust side's record already had.
- An `OnProcessExit` without a `target_action` (any process exit shuts
  down) is warned about and not modelled. Other `on_exit` actions, and a
  callable `on_exit`, remain unmodelled, as before.

Tests (`tests/tests/on_exit_shutdown.rs`, the same four checks as the
`on_exit=` cases): `rust_parser_on_process_exit_handler`,
`..._registered_first`, the two `python_parser_` equivalents, and
`up_from_a_resolved_on_process_exit_handler`. Negative control: with the
dumper change reverted, both Python cases time out at 60 s.
