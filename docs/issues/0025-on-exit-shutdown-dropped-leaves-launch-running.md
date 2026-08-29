---
id: 25
title: "`on_exit=Shutdown()` is dropped, so a required node's exit never ends the launch"
status: resolved
type: correctness
severity: high
---

# 0025 — a required node exits and the launch keeps running

**Repo:** `play_launch`
**Affects:** the dump visitor (`dump/visitor/node.py`), the record types, and the
member coordinator (`member_actor/coordinator/{builder,runner}.rs`,
`commands/up.rs`)

## What happened

`on_exit=Shutdown()` is the launch idiom for "this process is required; when it
finishes, take the whole launch down with it". play_launch detected these handlers at
dump time and deliberately discarded them, warning once:

```
One or more nodes have on_exit handlers which are NOT supported by play_launch.
Only respawn functionality is supported. on_exit handlers will be ignored during replay.
```

For an optional node that is harmless. For an orchestrator it is not. SSv2's
`scenario_test_runner` runs a scenario, exits 0, and expects to take the interpreter,
preprocessor and visualization with it. Replayed without the handler, those three ran
forever: `scenario_test_runner/status` read `0` while the other three had no `status`
file at all, and their `play_launch` stayed up holding a ROS domain.

The visible symptom was not a stuck launch but a stuck *node*. The interpreter's `main`
is `while (rclcpp::ok()) { status_monitor.touch(); executor.spin_once(); }`, and
`spin_once()` has no timeout, so an idle inactive node blocks in `rcl_wait` and its own
watchdog reports it as broken:

```
[ERROR] [status_monitor]: main of openscenario_interpreter_node unresponsive for 284079 ms.
```

That message reads like an interpreter deadlock and is not one — it is the launch never
being torn down. One interpreter was found still spinning **40 hours** after its scenario
had passed, alongside an ego stack of ~90 nodes. Every scenario run in `play_log/` shows
the same, passing and failing alike.

## Root cause

Three separate gaps, each of which had to be closed:

1. **The dump discarded the handler.** `visit_node` warned and moved on, so nothing
   downstream could know the node was required.
2. **The record was rebuilt from the model.** On the model path `context.record` comes
   from `node_record_from_instance`, and `NodeInstance` (in the external
   `ros-launch-manifest` crate) carries no launch-level handlers — so putting the flag
   only on `NodeRecord` was not enough. The set is now read from the `LaunchDump`, which
   does carry it.
3. **Names did not match.** Members are keyed by canonical id (`node:/ns/name`,
   `member_id.rs`), which is what actors report in `StateEvent::Exited` — matching on
   the display name silently never fired.

And once it did fire, pulling the actor shutdown lever alone left every launched process
running: the processes are killed by signalling the process **group**, which only the
command layer can do.

## Fix

- `_classify_on_exit` splits a node's `on_exit` actions into "shuts the launch down" and
  everything else. Both `launch.actions.Shutdown` and SSv2's `ShutdownOnce` are
  `EmitEvent` actions carrying a `launch.events.Shutdown`, so the match is on the event,
  not the class. `on_exit` may be a single action, a list, or a callable — assuming a
  list made the dump die with `'Shutdown' object is not iterable` on exactly the launch
  files this is for. The warning survives for handlers that are still unsupported.
- `on_exit_shutdown: Option<bool>` on `NodeRecord` (both record types), `#[serde(default)]`
  so older records still deserialize.
- `up.rs` collects the required members from the `LaunchDump`, keyed like `member_name`;
  the builder translates those to canonical ids as it allocates them.
- `MemberRunner::honour_on_exit_shutdown` fires on `StateEvent::Exited` for a required
  member and runs a hook installed by `up.rs` that mirrors the signal path exactly — the
  replay-level watch, `member_handle.shutdown()`, and SIGTERM to the process group. This
  is the same sequence `--on-startup-failure exit` uses.

## Verified

Synthetic launch (`orchestrator` with `on_exit=Shutdown()` plus a long-running `worker`):
before, play_launch ran until killed at the 90 s test timeout and the worker survived;
after, it logs

```
[node:/orchestrator] exited (no exit code) and was declared on_exit=Shutdown: shutting down the launch
```

and returns in ~2 s with no leftover processes.

Real scenario (`carla-scenario-bridge`, SSv2 `town01_unmanaged.xosc`): the scenario stack
now tears itself down when `scenario_test_runner` exits 0 — its `play_launch` is gone and
`pgrep openscenario_int` returns nothing, where previously both survived indefinitely.

## Not covered

The Rust-native parser does not model `on_exit` at all; only the Python dump path carries
it. A launch replayed through `--parser rust` still ignores the handler.
