---
id: 23
title: "Six of 84 composable load requests never reached the container, and nothing anywhere recorded it"
status: open
type: correctness
severity: high
---

# 0023 — loads vanish between play_launch and the container, silently

**Repo:** `play_launch`
**Affects:** the LoadNode dispatch path
(`member_actor/container_actor/supervisor.rs::dispatch_pending_loads`,
`container_control`, `ros_client::call_load_node_service`) and the log bundle
layout (`commands/up.rs`)
**Observed under:** `--container-mode isolated`, the DEFAULT
**Distinct from #0019:** there a load was accepted and the child was killed
later. Here the container never printed `Accepted load request` at all, so the
request did not arrive.

## What was measured

Two runs of the golf cart Autoware stack on the 12-core AGX Orin, same machine,
28 minutes apart, same launch. The first ran `isolated`, the second `observable`.
Log bundles `2026-08-25_16-45-52` and `2026-08-25_17-33-40`.

Counting `Accepted load request` and `Spawned isolated child` in each container's
`err` against `metadata.json`'s `composable_node_count`:

```
container                                decl accepted spawned
container                                  16       10      10   <-- 6 lost
behavior_planning_container                 3        3       3
container_2                                19       19      19
container_3                                 1        1       1
control_check_container                     5        5       5
control_container                           7        7       7
map_container                               4        4       4
mission_planner_container                   3        3       3
motion_planning_container                   4        4       4
mrm_comfortable_stop_operator_container     1        1       1
mrm_emergency_stop_operator_container       1        1       1
parking_container                           3        3       3
planning_validator_container                2        2       2
pointcloud_container                        5        5       5
system_monitor_container                    8        8       8
velocity_smoother_container                 2        2       2
TOTAL                                      84       78      78
```

The six that vanished, all from `/system/component_state_monitor/container`:

- `/system/component_state_monitor/component`
- `/system/topic_state_monitor_initialpose3d`
- `/system/topic_state_monitor_object_recognition_objects`
- `/system/topic_state_monitor_system_emergency_control_cmd`
- `/system/topic_state_monitor_vector_map`
- `/system/topic_state_monitor_vehicle_status_steering_status`

The `observable` run 28 minutes later loaded 16 of 16 in the same container
(`Instantiate class` x16), and all five of those `topic_state_monitor` names
appear in `diagnostics.csv` for the first time. Whatever this is, it did not
reproduce in the other mode on the same machine that afternoon. One occurrence,
not a rate.

## Three ways it stayed invisible

**1. The container is not the one dropping them.** Its ten accepts land in a
57 ms burst and then stop:

```
[1787647553.889475737] Accepted load request ... (pre-assigned id 1)
[1787647553.926821720] ... id 2
    ... ids 3..9 ...
[1787647553.946109890] ... id 10
[1787647554.569414628] Spawned isolated child PID 10840 ...
```

The container answers LoadNode by pre-assigning an id and returning immediately
(`clone_isolated_component_manager.cpp`), so an accept costs it nothing and it
has nothing to be busy with. It was not overloaded, and it never saw requests
11 through 16.

**2. It is not the global load cap.** `max_concurrent_load_node_spawn` defaults
to 10, which matches the count suspiciously well, but the cap is global rather
than per-container, and `container_2` accepted 19 in the same window. Every one
of the 16 containers dispatched inside a single 1.3 s span and nothing sets this
container apart by start order or by size:

```
container                                decl  acc  proc_t0  1st acc last acc
container_2                                19   19    0.199    0.894    1.041
container                                  16   10    0.339    1.107    1.164
map_container                               4    4    0.351    1.059    1.208
```

Since phase 61 the cap is documented as disabled for `isolated`
(`docs/design/composable-load-admission.md`). If it were somehow active and
leaking permits, the ceiling would be 10 across the launch, not 78.

**3. Nothing in the bundle can say what happened.** `dispatch_pending_loads`
drains its queue completely and spawns a task per request, so a request lost
after that point died inside `call_load_node_service` — where the only witness
is play_launch's own log. **play_launch writes no log file into `log_dir`.**
The bundle has `diagnostics.csv`, `system_stats.csv`, `node/`, `load_node/`,
`params_files/` and `plot/`; the launcher's own stdout exists only in the
terminal that ran it, which is gone.

So the bundle from a failed launch cannot answer why the launch failed. That is
the part worth fixing first, because it is what makes the rest of this issue
guesswork.

## Why the six mattered

They are not incidental. Five of them are `topic_state_monitor` instances, which
are leaves of the Autoware diagnostic graph. The graph ran for the whole session
with five inputs that never published, and the aggregator has no way to
distinguish "this leaf is absent" from "this leaf is quiet": a missing leaf and a
healthy silent one look identical downstream. The sixth,
`component_state_monitor/component`, is the node that decides whether a
subsystem is ready at all.

A launch that drops nodes and reports success is bad. A launch that drops the
nodes whose job is to notice things are missing is worse, because it removes the
mechanism that would have caught it.

## Suggested order of work

1. **Write the launcher's own log into `log_dir`.** Everything below is
   diagnosable in one run once this exists, and not diagnosable without it.
2. **Reconcile declared against loaded at startup-complete**, per container,
   and fail loudly on a shortfall. `metadata.json` already carries
   `composable_node_count`; the check is a comparison nobody makes. Note that
   #0019 already forced `Loaded` to require ListNodes confirmation, so the
   number to compare against is available and trustworthy.
3. Then find the drop. Candidates, in the order I would test them: the LoadNode
   client not yet matched when the call is made (this container starts 0.339 s
   in and its first accept is at 1.107 s, the longest such gap in the table);
   a `call_load_node_service` error path that reports failure only through a
   channel whose consumer had already moved on; and the `load_completion_tx`
   send whose result is discarded with `let _ =`.

## Reproduction

Not reproduced on demand. To try: `--container-mode isolated` on a launch with
many containers coming up at once, then compare `Accepted load request` counts
against `composable_node_count` per container. On the golf cart stack that is
84 composables across 16 containers, all dispatched within 1.3 s.

The version of play_launch installed on the vehicle
(`/home/ubuntu/.local/lib/python3.10/site-packages/play_launch/`) is not
recorded in the bundle, which is a second thing worth writing into `log_dir`.
