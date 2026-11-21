---
id: 108
title: "A launch file driving its LifecycleNode with ChangeState could not be parsed"
status: resolved
type: defect
severity: high
---

# 0108 - A launch file driving its `LifecycleNode` with `ChangeState` failed

**Repo:** `play_launch` up to 0.41.2
**Affects:** the Python frontend's stand-ins (pyexec `api/mod.rs`,
`api/visit.rs`, `api/launch_ros/lifecycle_node.rs`) and the lifecycle fields
of the record and model
**Found:** amr 175, 2026-11-22, on Jazzy: Nav2's `bringup_launch.py
slam:=True` includes slam_toolbox's `online_sync_launch.py`

## What happened

    ERROR Python execution failed: ImportError: Blocked filesystem import of
    launch.events.matches_action (use mocks only)

slam_toolbox's launch files on Jazzy start their node as a `LifecycleNode`
and bring it up themselves:
1. `EmitEvent(ChangeState(lifecycle_node_matcher=matches_action(node),
   transition_id=TRANSITION_CONFIGURE))`, right after the node;
2. a `RegisterEventHandler(OnStateTransition(start_state='configuring',
   goal_state='inactive', entities=[EmitEvent(ChangeState(…ACTIVATE))]))`.

Both are gated on `autostart`. The stand-ins had no `launch.events.matches_action`,
no `launch_ros.events.lifecycle.ChangeState` and no `launch_ros.event_handlers`,
so the import fell through to the real modules and was blocked, and the whole
file failed. Humble's slam_toolbox does not do this, which is why Humble
parsed.

## Fix

- **Stand-ins:** `launch.events.matches_action` (and `matches_name`),
  `launch_ros.events.lifecycle.ChangeState` and `StateTransition`, and
  `launch_ros.event_handlers.OnStateTransition`.
- **The walk executes `EmitEvent(ChangeState(…))`.** A `LifecycleNode`
  remembers the capture it made, and `matches_action(node)` names it.
  - A configure and an activate of one node make it `lifecycle_autostart`,
    which `up` drives after spawn (`Autostart::Active`).
  - A configure alone makes it `lifecycle_configure_only`, which becomes
    `Autostart::Configure` in the model. The executor already drove that
    case; nothing could produce it before.
  - An activate alone does nothing, as in launch, since the node is
    unconfigured.
  - A matcher other than `matches_action(node)` is reported, not guessed.
  - The handler's entities were already walked at startup, so the activate
    event inside one reaches the walk.
- **`EmitEvent` keeps its `condition=`.** The stand-in swallowed it, so a
  conditional event ran unconditionally. That made `autostart:=false`
  configure the node anyway.
- **pyexec C ABI 15 to 16**, since the captures carry a new
  serde-defaulted field.

Verified on Humble with a launch file of the same shape around
`lifecycle_talker`. Stock `ros2 launch` and play_launch both leave the
configured-and-activated node `active` and the configure-only one
`inactive`; before the configure-only field, play_launch left the second
one `unconfigured`. Resolving slam_toolbox's own Jazzy `online_sync_launch.py`
gives `/slam_toolbox` with `lifecycle_autostart: active`, and with
`autostart:=false` none.

## Not done

- **`--parser python`:** the dumper runs launch's own action classes and
  records nodes, not events, so a model it writes for this file has no
  autostart. Not checked.
- **`LifecycleTransition`** (`launch_ros.actions`), the other way a launch
  file drives lifecycle nodes, is still a stand-in that records nothing.

## Tests

`execution_semantics.rs::a_lifecycle_node_the_launch_configures_and_activates_autostarts`
(fixture `lifecycle_change_state`): active, configure-only, and neither under
`autostart:=false`. It fails on the previous code at the import.
