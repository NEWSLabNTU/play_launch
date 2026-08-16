# Phase 61 — the edge startup storm

Status: **W1 done** (diagnosis + admission control + OOM bias + two defect
fixes). **W2 done** (staged startup: start consumers before producers).
W3 — per-node isolation granularity — not started, and it is the item that
would actually let a vehicle keep crash containment and lose the storm.

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

Nearly two thirds of the storm is the composable-node machinery: the 84
`component_node` processes that `--container-mode isolated` forks. They exist
for a reason (see "The lever that does work" below) — this is what fault
isolation costs, not a defect.

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
faster**.

**But this is not a recommendation, and the first draft of this document was
wrong to read it as one.** The cost is the documented price of fault isolation
(`docs/design/container-isolation.md`), not waste:

- **A SIGSEGV cannot be contained inside a process.** In a shared container one
  segfaulting composable takes down every node loaded beside it — 3–10 in an
  Autoware container — and cascades from there. That document surveys the
  alternatives (signal handler + `pthread_exit`, `siglongjmp`, seccomp-bpf) and
  concludes every production system that isolates native-code crashes uses
  processes.
- **`oom_score_adj` is process-only** — so the per-node OOM bias W1 added is
  per-*node* under `isolated` and per-*container* under `observable`. The
  cheaper mode is the one that makes this phase's own OOM protection coarser.
- **`memory` cgroups, `kill(pid)` and per-node restart are process-only too.**
- `observable` additionally gives up intra-process **zero-copy IPC**, put at
  ~1–5 ms per pipeline stage for LiDAR/camera paths.

So the mode is a **safety decision, not a performance one**: `observable` on a
bench, `isolated` on a vehicle. play_launch reports the cost and names what the
alternative forfeits; it does not downgrade anything.

The real lever for a vehicle that cannot afford the full cost is to reduce *what
has to be isolated* — per-node rather than per-run granularity — which
play_launch does not support today. That is the W2 item below, and it is the
one that would let a vehicle keep containment and lose the storm.

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
- **Per-node isolation granularity.** `--container-mode` is all-or-nothing for
  a whole run, so an integrator who needs containment for one perception node
  pays for it on all 84. Being able to isolate a named set — and load the rest
  as threads — is what would let a vehicle keep crash containment where it
  matters and drop the storm. This is the single most valuable follow-up.
- ~~**Start order**~~ — done, see W2 below.
- `delay_load_node_millis` still exists in layer 2's copy of the config
  (`src/ros-launch-resolve/cli/src/config.rs`), which is a duplicate of this
  struct and a place for the two to drift.

## W2 — staged startup

`execution/startup_order.rs`. Members are assigned a stage; stage 0 starts
immediately and each later stage waits until every earlier member is up. The
point is to start sensor drivers LAST, so nothing publishes into subscribers
that do not exist yet.

**Correction to the W1 write-up:** it claimed "the SystemModel already carries
the chain information needed to derive an order". It does not, for a launch like
this one. `structure.topics` is built from authored manifests
(`model_builder.rs` walks `index.topics`), not from the launch tree, and the
golf cart resolve reports **0 topics**. So automatic derivation is inert here
and the working path is explicit configuration. Both ship:

- `startup.order` — glob groups against node FQNs, in start order. Works on any
  launch.
- `startup.defer_sources` — derives a final group from the topic graph
  (publishes and never subscribes → last). Inert without manifests. Explicit
  groups win.

Off by default: reordering startup is a semantic change, and a system where
something waits on a driver being up early would break.

### A stage waits for the ROS graph, not for `spawn()`

`spawn()` returns long before a node has constructed its subscriptions, so a
spawn-counting gate would order the `fork()` calls and nothing that matters. A
member is accounted for when it appears in the graph, is `Loaded` (composable),
or is dead — that last one because a driver whose hardware is absent never
appears and must not hold up the system. The timeout names whatever is still
missing.

### The naming gap this surfaced

The first staged run sat on its full 25 s timeout reporting 16 healthy nodes as
missing. Dumping the live `ros2 node list` against the model explained it: **19
of 144 model FQNs never appear in the graph**. For a node the launch file did
not name, the model keys it by its EXECUTABLE while the node registers its
compiled-in name —

    model:  /localization/pose_twist_fusion_filter/autoware_ekf_localizer_node
    graph:  /localization/pose_twist_fusion_filter/ekf_localizer

— and the emitted cmdline confirms play_launch passes `__ns` but **no `__node`
remap**, so it never imposes the model's name. (CLAUDE.md says `exec_name` is
used as a `__node` remap; it is used as the FQN map *key*. Worth an issue —
changing node naming would move params and remaps, so it is not a side effect to
take on here.)

`NodeInstance.node_name.is_none()` is an exact discriminator for the case: 17
nodes, plus the 2 lidar drivers legitimately absent without hardware, accounts
for all 19. The gate now requires a graph match only where the model knows the
name and falls back to process-running otherwise — weaker for those, but the
strongest signal available and no worse than not staging at all.

### Measured

| | unstaged | staged |
|---|---|---|
| stage 0 up | — | 24.7 s |
| peak load1 | 201 | 212 |
| peak runnable tasks | 473 | 462 |
| MemAvailable consumed | 3.8 GiB | 3.7 GiB |

No resource benefit, as expected: **the benefit needs sensors**, and there are
none on the bench. What is measured is that the ordering works and what it
costs — the drivers start ~14 s later, because stage 0 completing means the
consumers are genuinely in the graph rather than merely spawned. On the vehicle
that is 14 s of LiDAR frames not published into nothing.

Confirming the benefit is the same open vehicle run as W1's.
