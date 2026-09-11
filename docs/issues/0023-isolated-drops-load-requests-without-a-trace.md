---
id: 23
title: "Six of 84 composable load requests never reached the container, and nothing anywhere recorded it"
status: resolved
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

## Resolution

Worked in the issue's own order, on `main` after phase 64 — which is
**not** the code the vehicle ran. The wheel on the golf cart was `v0.9.0`
(tagged 2026-08-18): it carried #0019's fix, so `Loaded` already required
ListNodes confirmation, but **not** phase 64, so every one of the 84 loads
went through the rmw `LoadNode` service. That matters for part 3 below.

### 1. The launcher writes its own log into the bundle

`play_log/<ts>/` now holds three new files, from all three run verbs
(`launch`, `up`, `run`):

- **`play_launch.log`** — play_launch's own tracing output. It opens with a
  header (`# play_launch <version>`, start time, pid, cwd, argv quoted so it
  can be pasted back, the config path, the filter), and then carries
  **`play_launch=debug,ros_launch_resolve=debug,info` regardless of the
  terminal's `RUST_LOG`** (`PLAY_LAUNCH_LOG_FILE` overrides it with `RUST_LOG`
  syntax). Measured on the `container_events` fixture with the web UI,
  monitoring and diagnostics all on, 20 s: INFO alone is ~21 lines,
  `play_launch=debug` 174 lines / 30 KB, a global `debug` 179 lines / 31 KB.
  Debug is ~8x the lines of INFO and still tens of kilobytes a run — nothing
  beside the per-node `out`/`err` already there — and it is the level at
  which the load path narrates every dispatch, acceptance and timeout, which
  is precisely what this bundle lacked. Crate-scoped rather than global so a
  chatty dependency can never turn the file into its own log; `EnvFilter`
  matches by prefix, so `play_launch=` covers the parser too.
- **`run_info.json`** — version, argv, cwd, pid, start time, config path,
  and the name of the config copy. The version was the second thing this
  bundle could not answer.
- **`config.yaml`** — a verbatim copy of `--config`, when one was given.

The subscriber is installed in `main()` long before `play_log/<ts>/` exists
— `launch` parses the launch file first, `up` loads and validates the model
— and those early lines are worth keeping (a parser warning is diagnostic).
Rather than create the directory earlier (which would move `create_log_dir`
out of the shared `play()` engine into three callers and leave an empty
timestamped directory plus a moved `latest` symlink behind every failed
parse), the file layer is registered at startup with a writer that
**buffers** (`util::run_log`, 4 MiB cap, overflow counted and noted) until
`attach` is called right after the directory is created; the buffer drains
into the file ahead of everything that follows. The integration test pins
that ordering: `Step 1/3: Parsing launch file` precedes `Log directory
created` in the file. A log that cannot be opened is a warning, never a
failed run.

### 2. Declared against loaded, per container, at startup-complete

`commands::startup_reconcile::composable_shortfall` walks the member list
and, per container, counts the composables the launch declared for it and
the ones that reached `Loaded`; everything else is missing, with its FQN and
its state. At startup-complete the launcher now prints, at **`error`**, one
line per short container:

```
Startup shortfall in /shortfall_container: 1/2 composables loaded — 1 MISSING: /system/ghost (failed: ...)
```

and `STARTUP_SUMMARY` carries a `composable_shortfall` array with the same
facts. `Startup complete: all nodes ready` is printed only when the
shortfall is empty AND nothing failed. The comparison is against the
**declaration**, not the failure count: a composable that is blocked because
its container never came up, or still loading, is missing too, where
`HealthSummary` counts it as nothing at all. Related gap closed on the way:
a container that fails to spawn leaves its composables `Blocked`, which
never settles, so startup never completed and the periodic line named
nothing — the "still incomplete after Ns" report now names blocked
composables with their reason, at `error`.

**Exit behaviour is deliberately unchanged.** At startup-complete a missing
composable is a `Failed` member (the completion test requires every
composable to be loaded or failed), so `--on-startup-failure exit` already
exits on a shortfall; a second knob would decide the same thing twice.

### 3. The drop

**Could the drop the vehicle saw still happen on `main`?** Not on the path
our own container uses. On `main`, `isolated` loads go over the phase 64
socketpair: the frame is written to a stream that is ordered and lossless,
the container answers `accepted` with the id, and phase 64 W2's ack timer
turns silence into a *question* (`query` → `status`) rather than a guess.
The rmw `LoadNode` path where the six requests plausibly died is now only
the fallback (stock containers, an old container binary, `control_socket:
false`). Every place on the socket path where a request could be lost
without a log line was walked, and each now says so:

| Where | Before | Now |
|---|---|---|
| `ControlChannel::send` — enqueue to the writer task fails | swallowed; `send_load` returned `Ok(seq)`, entry went `Loading` with a tracking record for a frame that never existed | `send` returns `Err`; `send_load` propagates it and the entry is marked `Failed` with `control-channel: …` (unit test: a channel whose peer is gone must answer `Err`, not a seq) |
| `write_loop` — the socket write fails | `debug!`, and every frame still queued behind it vanished uncounted | `warn!` naming the container and the number of abandoned frames |
| `read_loop` — an unparsable frame from the container | `debug!` | `warn!` with the first 200 bytes — if it was an `accepted` or a `loaded`, this line is the only trace of where that fact went |
| `send_query`/`send_cancel` — the probe itself cannot be sent | silent | `warn!` |
| `send_load_over_socket` — a name the supervisor does not know | silent `return` | `warn!` naming the composable |
| `Accepted` for a seq no entry is waiting on | `debug!` | `warn!` — the container will fork a child nobody supervises |
| LoadNode path: `let _ = tx.send(LoadCompletion)` | outcome discarded when the actor is gone | `warn!` naming the composable and the outcome it had |

Also examined and found sound: a load queued before the hello handshake
(`negotiate_control_channel` settles before `handle_load_all_composables`,
and `await_hello` buffers anything that arrives early); the fallback engaging
mid-dispatch (`closed` flips only when the peer's end is gone, which
`child.wait()` sees too, and a tracked entry that then falls to the
ListNodes sweeps is named by the `rescue_lost_loads` "left alone" warning);
and `emit()` to the coordinator, which already logs at `error` when its
receiver is gone.

**What actually happened on 2026-08-25 cannot be recovered**, and the
current code makes that loss the last of its kind rather than explaining it.
On v0.9.0 the request left `client.call()` and never reached the container's
service — the `Accepted` count says so — which puts the loss inside DDS,
where play_launch has no witness: a request published before the service's
reader has matched the client's writer is simply not delivered, and
`service_is_ready()` plus the warmup delay narrow that window without
closing it (the issue's first candidate; `container` had the longest
start-to-first-accept gap in the table). Thirty seconds later that version
would have logged `LoadNode service call timed out … deferring to
ComponentEvent`, then `rescue_lost_loads`' verdict — every one of those
lines to the terminal only. The bundle now carries them.

**Still open**: the rmw fallback path keeps its ambiguity for our own
container — a load accepted (id known) that ListNodes then reports absent
stays `Loading` (`check_loading_timeouts` cannot tell "constructing" from
"gone"), named only by the periodic "waiting on" line; phase 64's answer is
to not use that path. And `docs/issues/README.md` says resolved issues move
to `archived/`, but no such directory exists and none of #0007–#0029 was
ever moved, so this file stays where its predecessors are.
