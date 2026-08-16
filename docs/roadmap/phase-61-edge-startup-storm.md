# Phase 61 — the edge startup storm

Status: **W1 done** (diagnosis + admission control + OOM bias + two defect
fixes). W2 (process-count reduction as a first-class choice) not started.

## The report

A golf cart Autoware stack (`~/repos/2026-golf-cart`) was launched on an AGX
Orin with sensors attached. play_launch started every node at once, the machine
went into a resource storm, and the **kernel OOM killer took the operator's
GNOME session**. The same stack is fine on a workstation.

## Reproduction

Bench AGX Orin, 12 cores, 64 GiB + 32 GiB swap — the same hardware as the
vehicle, confirmed with the reporter. No sensors attached.

    play_launch launch golfcart_launch golfcart.launch.yaml \
        host:=master rviz:=false imu_source:=xsens camera_model:=gscam

play_launch's own line: `Parsed: 44 node(s), 16 container(s), 84 composable
node(s)`. Under the default `--container-mode isolated` each composable is
fork+exec'd into its own process, so **144 processes** start.

Harness: [`scripts/edge-storm/`](../../scripts/edge-storm/README.md).
`memwatch.sh` samples `/proc/meminfo`, `/proc/stat` and `/proc/vmstat` at
100 ms; `cpuwatch.py` samples every `/proc/<pid>/stat` for per-process
attribution; `oom_guard.sh` kills the launch process group if `MemAvailable`
falls below a floor, so the experiment could not repeat the vehicle's outcome on
the machine running it; `analyze_repro.py` summarises a trace.

### What reproduced, and what did not

| | measured |
|---|---|
| peak runnable tasks | **484** on 12 cores |
| peak load1 | **203** |
| CPU during startup | **85% of all 12 cores for 49 s** |
| MemAvailable consumed | 4.0 GiB |
| OOM kills | none |

The **memory** storm did not reproduce; the **CPU** storm did, hard. 4 GiB
against 64 GiB is nothing, and that gap is informative rather than a failed
repro: on the vehicle the sensors keep publishing into nodes that cannot get
CPU for ~50 s, and DDS and callback queues absorb the backlog. Here there is
nothing to queue. The CPU storm is the cause; the OOM is its consequence.

**This is the one part of the diagnosis that is inferred rather than measured**,
because it needs sensors. Confirming it means re-running on the vehicle with
`memwatch.sh` attached.

### Attribution

Per-process CPU over the 49 s startup window (`cpuwatch.py`, weighted by real
sample intervals — the sampler was itself starved and its nominal 500 ms
interval stretched to 2.5 s, which invalidated a first naive pass):

| process | cpu-s | share |
|---|---|---|
| `component_node` (84 procs) | 252.9 | **50.5%** |
| `component_container` (19 procs) | 61.4 | 12.3% |
| everything else | ~186 | 37% |
| **total** | **500.8** | 85% of 588 available |

Nearly two thirds of the storm is the composable-node machinery, and the 84
`component_node` processes exist only because `isolated` is the default.

## Findings in the code

1. **No admission control existed at all.** `builder.rs:215` spawns one tokio
   task per node immediately; `regular_node_actor.rs` `handle_pending` called
   `command.spawn()` with no gate.
2. **`max_concurrent_load_node_spawn` was dead config** — parsed, defaulted,
   asserted in a unit test, read by nothing. `dispatch_pending_loads` drained
   its queue and spawned a task per request, so all 84 composables went into
   LoadNode at once. `delay_load_node_millis` was dead the same way.
3. **`startup_complete` was `composable_pending == 0`**, which is true at t=0:
   a composable whose container has not started yet is `Blocked`, counting as
   neither pending nor loaded nor failed. It worked only by a race — every
   container used to spawn within 100 ms — and pacing made the race lose every
   time, printing `Startup complete: all nodes ready (nodes 12/44, containers
   0/16, composable 0/84)` two tenths of a second in, then falling silent
   because the progress task exits on completion. Pre-existing; would misreport
   on any slow-starting container.
4. **play_launch 0.9.0 could not parse the launch file at all.** `<choice>` is
   a child entity of `<arg>` in real ROS 2
   (`declare_launch_argument.py:176` reads it as
   `get_attr('choice', data_type=List[Entity])`), but `attr_spec.rs` gave `arg`
   no children, so the YAML frontend saw an unknown attribute. A regression
   against the 0.5.1 the project runs today. Fixed; the choice LIST is still
   not enforced, which is laxness rather than a parse divergence.

## The fix that did not work

The obvious inference is that the storm is contention and that pacing the
spawns will fix it. It was implemented (`execution/startup_governor.rs`) and
**measured to be wrong**. Same binary, same launch:

| | pacing off | 12 at a time |
|---|---|---|
| spawn → startup complete | 10.6 s | 23.8 s |
| peak runnable tasks | 525 | 471 |
| peak load1 | 181 | 184 |
| MemAvailable consumed | 3.8 GiB | 3.6 GiB |

Doubled the startup for about ten percent. The storm is not contention over a
fixed amount of work — it *is* the work; those processes cost what they cost by
existing. Serialising the spawns stretches the same CPU over a longer window.

An earlier variant was worse still: a runnable-task ceiling of `2 * ncpu`
cannot distinguish "busy because we are starting things" from "busy because the
things we started are running", so nodes already up held `procs_running` above
the ceiling and nothing further could be admitted until the 30 s bypass fired
each time — startup unfinished at 190 s with peak load1 **217 against the
unpaced 183**, i.e. worse than doing nothing.

Both are now off by default, with the measurements recorded at the fields they
govern so the defaults are not re-derived from first principles by the next
reader.

## The lever that does work

Both arms below are the phase-61 binary with its shipped defaults, so the
completion figures are comparable (the pre-fix detector could not report
completion at all — finding 3).

| | `isolated` (default) | `observable` |
|---|---|---|
| processes | 144 | 60 |
| spawn → startup complete | 10.7 s | **8.4 s** |
| composables loaded | 84/84 | 84/84 |
| peak load1 | 190 | **45** |
| peak runnable tasks | 506 | **128** |
| MemAvailable consumed | 3.5 GiB | **1.4 GiB** |
| CPU during startup¹ | 10.2 of 12 cores | 3.9 cores |
| CPU in steady state¹ | 4.0 cores | 2.8 cores |

¹ from the earlier pre-fix pair, which is where per-process attribution was
sampled; the ratio is what matters and the load/memory columns above confirm it
on the shipped binary.

Same system up, **2.6x less CPU, 4x less peak load, 2.5x less memory — and
faster**. There is no throughput argument for `isolated` here: what it buys is
per-composable crash isolation, and it charges a process, an executor and a DDS
participant per node for it. On a 12-core box carrying 84 composables that
charge dominates everything else.

This is a **warning, not an automatic downgrade**: the isolation is real and
only the operator knows whether they need it.

## What shipped (W1)

- `execution/startup_governor.rs` — admission control with three independent
  gates, each with a bypass so a launch can never fail to start:
  - concurrency limit — **off by default**, see above;
  - runnable-task ceiling — **off by default**, see above;
  - `MemAvailable` floor — **on**, 10% of RAM clamped to [512 MiB, 4 GiB]. Not
    a throughput control: it never blocks while memory is plentiful and only
    serialises admissions once memory falls through the floor, which is the
    condition that ended with a dead desktop.
  - a separate semaphore for composable loads, because a load slot is held
    until LoadNode returns (minutes, under the existing retry budgets) and
    sharing one semaphore would let a stuck container stall every plain node.
- `oom_score_adj = +300` on every spawned child (`node_cmdline.rs` `pre_exec`,
  `PLAY_LAUNCH_OOM_SCORE_ADJ` to override). The kernel chose GNOME because it
  was the largest thing on the box and nothing said the 144 new processes were
  expendable. Raising the score needs no privilege; only lowering does, which
  is why play_launch cannot protect the desktop directly.
- `max_concurrent_load_node_spawn` made real; `delay_load_node_millis` removed
  (`post_service_ready_warmup_ms` already does what it described).
- `HealthSummary::startup_complete()` replacing the broken test.
- The `isolated`-mode advisory, fired when composables exceed `4 * ncpu`.
- `attr_spec.rs`: `<arg>` accepts `<choice>`.

## Open (W2)

- **Confirm the OOM mechanism on the vehicle.** Everything above is measured;
  the link from CPU storm to OOM is the one inference, and it needs sensors.
- **Make the process-count decision first-class** rather than an advisory.
  Options: per-container isolation (isolate the containers that host
  crash-prone nodes, not all of them), or a resource-aware default.
- **Start order.** Nothing here changes *what* starts first. Starting consumers
  before producers would bound the queue backlog that is the suspected path
  from CPU storm to OOM, and the SystemModel already carries the chain
  information needed to derive an order.
- `delay_load_node_millis` still exists in layer 2's copy of the config
  (`src/ros-launch-resolve/cli/src/config.rs`), which is a duplicate of this
  struct and a place for the two to drift.
