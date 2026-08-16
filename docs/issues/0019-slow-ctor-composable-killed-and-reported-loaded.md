---
id: 19
title: "A composable whose constructor takes >30s is SIGKILLed mid-construction, and play_launch reports it as loaded"
status: resolved
type: correctness
severity: high
---

# 0019 — slow-constructing composables are killed, and the failure is reported as success

**Repo:** `play_launch`
**Affects:** `play_launch_container/src/clone_isolated_component_manager.cpp`
(the 30 s ready-pipe timeout), `member_actor/container_actor/ros_client.rs`
(the ComponentEvent fallback)
**Applies to:** `--container-mode isolated`, which is the DEFAULT
**Explains:** the unresolved third case in #0017 (`motion_velocity_planner`)

## Two defects that compound

**A. The container kills any composable whose constructor exceeds 30 s.**

`clone_isolated_component_manager.cpp` fork+execs `component_node`, then waits
on a ready pipe:

```cpp
constexpr int kReadyTimeoutMs = 30000;  // 30s matches LoadNode service timeout
...
int ret = poll(&pfd, 1, kReadyTimeoutMs);
if (ret <= 0) { break; }          // timeout
...
kill(child_pid, SIGKILL);
waitpid(child_pid, nullptr, 0);
throw std::runtime_error("component_node did not respond (timeout or crash)");
```

The child writes `OK <name>` only *after* `factory->create_node_instance(options)`
returns (`component_node.cpp:181-187`) — i.e. after the node's **constructor**
has run. So the 30 s budget covers construction.

**B. play_launch reports the killed node as loaded.**

The container pre-assigns a unique_id and returns from LoadNode *before* the
child is ready (`Accepted load request … spawning async`). play_launch takes
that service response as evidence, and when the `ComponentEvent LOADED` never
arrives it falls back to the response after `loading_event_timeout_secs`:

    WARN  ComponentEvent LOADED not received for '…' (unique_id: 1) after 10s
          -- falling back to service response
    INFO  Startup complete: all nodes ready (…, composable 2/2)

So a load that FAILS twenty seconds later is already counted as a success, and
`Startup complete` is printed over the top of it.

## Why this matters: TensorRT

First-run TensorRT engine builds compile an `.onnx` into a `.engine` inside the
node's constructor and take **minutes to tens of minutes**. The `.engine` is
written at the end, so a kill at 30 s discards all of it and the same thing
happens on the next launch: **the node can never complete its first run.** No
retry helps — each of the three attempts forks a fresh child that is killed at
30 s again.

CLAUDE.md already warns that such constructors "hold a blocking container's
executor" and advises raising `load_total_budget_secs`. That advice does not
help here: the 30 s limit is a hardcoded C++ constant on the isolated path,
below every configurable timeout around it (first call 30 s, retry 60 s, budget
600 s), and it is the one that fires.

## Reproduction

`play_launch_container` ships `SlowLoader`, whose constructor sleeps
`load_delay_ms`. `tmp/slow/slow.launch.xml`:

```xml
<node_container pkg="rclcpp_components" exec="component_container"
                name="slow_container" namespace="/probe">
  <composable_node pkg="play_launch_container"
                   plugin="play_launch_container::SlowLoader" name="slow_one">
    <param name="load_delay_ms" value="45000"/>
  </composable_node>
  <composable_node pkg="play_launch_container"
                   plugin="play_launch_container::SlowLoader" name="fast_one">
    <param name="load_delay_ms" value="1000"/>
  </composable_node>
</node_container>
```

    play_launch launch --disable-web-ui --container-mode isolated tmp/slow/slow.launch.xml

Container stderr:

    [INFO ] Accepted load request for 'SlowLoader' (pre-assigned id 1), spawning async...
    [INFO ] Spawned isolated child PID 3760889 for node '/probe/fast_one' (id 2)
    [INFO ] Component 'SlowLoader' loaded as '/probe/fast_one' (id 2)
    [ERROR] Failed to spawn component 'SlowLoader' (id 1):
            component_node did not respond (timeout or crash)

Accept at `1786883357.480`, failure at `1786883387.505` — **30.03 s**. And
play_launch, meanwhile:

    INFO  Startup complete: all nodes ready (nodes 0/0, containers 1/1, composable 2/2)

## Confirmed on the real stack

This is the `motion_velocity_planner` case #0017 could not explain — a
composable play_launch counted as loaded, whose child process was alive, and
which never appeared in `ros2 node list`. The golf cart container log says why:

    Accepted load request for 'autoware::motion_velocity_planner…'   @ 1786881093.933
    Failed to spawn component 'autoware::motion_velocity_planner…'   @ 1786881124.586

30.65 s. Same timeout. The node was killed and the run reported `84/84 loaded`.

## Notes on scope

- Perception is **off by default** in the golf cart launch
  (`launch_perception` defaults to `false`), so the TensorRT nodes are absent
  from the stack measured in phase 61. With `launch_perception:=true` the model
  grows 144 → 157 nodes, and exactly one TensorRT composable appears:
  `/perception/object_recognition/detection/centerpoint/lidar_centerpoint`
  (in `pointcloud_container`). `shape_estimation` also uses TensorRT but is a
  plain node, so it is not subject to any of this — it simply takes a long time
  to start.
- In `--container-mode observable` the failure mode is different and also bad:
  the composable is constructed on the container's executor, so a multi-minute
  constructor blocks every other load in that container. That is the hazard
  CLAUDE.md describes.

## Fix

Two halves, fixed independently and needed together.

**A — the ready timeout is raisable.** Fixed in the `play_launch_container`
submodule by `480f5fc` ("let a slow component constructor be waited for"),
which replaces the hardcoded 30 s with `PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS`
(bounded 1..3600000 ms, warning on an unparseable value). That commit carries
the measurement this issue lacked: Autoware's traffic light classifier takes
**~33 s to build its TensorRT engine from ONNX cold, and ~45 s of construction
even with the engine cached**.

**The default deliberately stays 30 s** — "so nothing changes for anyone not
asking for it". Note the consequence, since it follows from that same
measurement: a stack with the traffic light classifier still loses that node on
every launch unless the variable is set. The parent repo's submodule pointer is
bumped to `480f5fc` here, which is what makes the knob exist at all.

**B — a composable is `Loaded` only when ListNodes confirms it.**
`check_loading_timeouts` promoted on the LoadNode response alone once
`loading_event_timeout` (10 s) elapsed. Under `isolated` that response means
only "spawn requested", so a node still constructing — or already killed — was
counted as loaded. It now verifies through the same `verify_component_loaded`
path the budget-expiry code already used:

- `Present` → promote, with the confirmed unique_id.
- `Absent` → leave `Loading`; `rescue_lost_loads` owns the give-up decision.
- `Unavailable` → leave alone. The important one: a container too busy to
  answer is the NORMAL state while a composable constructs, and reading it as
  success is exactly what produced the false `2/2`.

A missing event is not evidence of success.

Also fixed alongside: `load_total_budget_secs` was measured from dispatch,
which after phase 61 is *before* the new load-slot wait — so time spent queued
was charged against the time allowed for the load itself. Harmless when every
load was dispatched immediately; a real hazard once there is a queue, and worst
exactly where loads are slowest. The clock now starts when the slot is held.

### Verified — the two halves do different jobs

45 s-constructor fixture, `--container-mode isolated`.

Default (30 s), so the container still kills it — but the failure is now
**reported**:

    container:  Failed to spawn component 'SlowLoader' (id 1)
    play_launch: Startup: … composable 1/2 loaded (1 pending)
    play_launch: Startup complete with failures: … 1 composable failed

Before B, that same run printed `Startup complete: all nodes ready (composable
2/2)`.

With `PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS=120000`, it loads:

    container:  Component 'SlowLoader' loaded as '/probe/slow_one' (id 2)
    play_launch: Startup complete: all nodes ready (…, composable 2/2)

So A decides whether the node gets to finish; B decides whether you are told
the truth either way.

## Options considered


1. **Make the ready timeout configurable and default it far higher.** It is a
   `constexpr` today; play_launch already plumbs `LoadTimings` and could pass
   one more value. Simplest fix for the immediate breakage. Choosing a number
   is the hard part — a first TensorRT build has no useful upper bound.
2. **Make the wait open-ended but liveness-based.** Do not time out on a child
   that is alive and making progress; the container already has a `pidfd` for
   it (`epoll_wait` on child death). Fail on child *death*, not on elapsed
   time. Strictly better than a bigger constant, since a hung child is still
   detected and a slow one is not punished.
3. **Fix the reporting regardless of 1/2.** A `LoadNode` response that only
   means "spawn requested" must not be promoted to `Loaded`. Either the
   container should not return until the child is ready, or play_launch must
   treat the pre-assigned id as provisional and require the `ComponentEvent`
   (or a `ListNodes` verification) before counting it. This is what turned a
   loud failure into `Startup complete: all nodes ready`, and it is worth
   fixing on its own — a silent success is worse than a slow load.

Recommendation: **2 + 3**.
