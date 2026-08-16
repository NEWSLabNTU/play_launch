# Running on an edge machine

play_launch's defaults assume a workstation. On a Jetson-class board — an AGX
Orin, say, with twelve cores instead of thirty-two — one of those defaults is
expensive enough to make a large launch unusable, and this page is about that
one plus the safety nets around it.

Everything here was measured on a 12-core AGX Orin bringing up a 44-node /
16-container / 84-composable Autoware stack. Full numbers and method:
[phase-61-edge-startup-storm.md](../roadmap/phase-61-edge-startup-storm.md).

## The one decision that dominates everything else

`--container-mode isolated` is the default, and on a small machine carrying
many composable nodes it costs more than every other setting on this page
combined. Whether you should change it is **not** a performance question.

### What it costs

| | `isolated` (default) | `observable` |
|---|---|---|
| processes | 144 | 60 |
| spawn → startup complete | 10.7 s | **8.4 s** |
| peak load1 | 190 | **45** |
| peak runnable tasks | 506 | **128** |
| memory consumed during startup | 3.5 GiB | **1.4 GiB** |
| CPU during startup | 10.2 of 12 cores | **3.9 cores** |

### What it buys

`isolated` fork+execs each composable into its own process. That is the only
way to get any of the following, because every one of them is process-granular
in Linux — see [container-isolation.md](../design/container-isolation.md):

- **Crash containment.** A SIGSEGV cannot be contained inside a process. In a
  shared container one segfaulting composable takes down every node loaded
  beside it — 3–10 nodes in an Autoware container, cascading from there.
- **Per-node OOM accounting.** `oom_score_adj` is process-only, so the OOM bias
  described below applies *per node* under `isolated` and *per container* under
  `observable`. Under `observable`, one composable's memory makes the whole
  container the victim.
- **Per-node kill, restart and cgroup limits.** Also process-only.

And what `observable` gives up beyond that: intra-process **zero-copy IPC**.
The design note puts serialisation at ~1–5 ms per pipeline stage, which is
material for LiDAR and camera paths with a latency budget.

### So which?

| | prefer |
|---|---|
| Bench, desktop, CI, bring-up, replay | `observable` — the CPU back is worth more than crash containment you are watching anyway |
| Vehicle, or anything where a node crash must not cascade | **`isolated`** — pay the CPU |

If you are on a vehicle and the cost hurts, the answer is not to give up
isolation globally; it is to reduce what has to be isolated. play_launch does
not yet support per-node isolation (it is all-or-nothing per run) — that is
open work, tracked in the phase-61 roadmap.

play_launch reports the cost when a launch would fork more than four processes
per core, so the trade is visible rather than implicit:

    WARN --container-mode isolated starts 84 extra processes (one per composable
    node) on a 12-core machine; ... That buys per-node fault isolation, OOM
    accounting and restart, which observable gives up along with intra-process
    zero-copy IPC ...

`stock` gives up play_launch's `ComponentEvent` visibility as well; prefer
`observable` over `stock` unless you specifically need the launch file's own
container binary.

## What happens if you run out of memory anyway

Two mechanisms, both on by default, neither of which slows a healthy launch.

**Children are marked as the preferred OOM victims.** Every spawned node gets
`oom_score_adj = +300`. If the machine does run out of memory, the kernel kills
a node rather than whatever else you had running — which is not academic: the
report behind this page had the OOM killer take the operator's GNOME session,
because the desktop was simply the largest thing on the box and nothing told the
kernel that the 144 processes which had just appeared were the expendable ones.

Set `PLAY_LAUNCH_OOM_SCORE_ADJ=0` to leave children at the kernel default.

Note that play_launch can only raise its own children's badness; *lowering*
another process's score needs privilege it does not have. So it can volunteer
its nodes, but it cannot protect your desktop directly.

**Spawning pauses when free memory gets low.** Before starting each process
play_launch checks `MemAvailable` against a floor of **1 GiB**, capped at a
quarter of RAM on machines too small for that. Above the floor this costs
nothing and never blocks. Below it, admissions serialise until memory recovers,
and after 30 s a node is let through anyway with a warning naming the gate,
because a launch that never starts is worse than one that starts under pressure.

The floor is deliberately **absolute rather than a share of RAM**: what it
guards against is one more process allocating before the next sample notices,
and a process needs what it needs regardless of how big the machine is. Measured
on this stack, the largest launch-owned process peaked at 274 MiB and the 99th
percentile across every process on the box was 116 MiB, so 1 GiB is about four
times the largest single demand, with room for the CUDA/TensorRT allocations a
sensor-attached run adds. (Scaling it at 10% of RAM, as an earlier version did,
gave a 64 GiB box a 4 GiB cushion it did not need and a 4 GiB board 512 MiB —
less than two of the processes it was meant to protect against, on the machine
least able to absorb the miss.) The quarter-of-RAM cap exists only so the floor
cannot demand half of a very small board and spend every startup bouncing off
its own gate.

```yaml
startup:
  min_available_mb: 2048   # explicit floor
  # min_available_mb: ~    # default: 1 GiB, capped at RAM/4
```

## Pacing the startup

There is a concurrency limit, and it is **off by default because it was measured
and did not pay**: capping at one process per core doubled a 10.6 s startup and
reduced peak runnable tasks by about ten percent. The reason is that a wide
launch costs what it costs by having those processes *exist*; starting them in
batches stretches the same CPU over a longer window rather than removing any of
it.

It is still the right knob in one situation: when the machine must stay
responsive to something else *while* the launch comes up, and you would rather
pay startup latency than take the whole box for fifty seconds.

```yaml
startup:
  max_concurrent: 4        # at most 4 processes starting at once
  max_settle_secs: 15      # cap on how long one slot may be held
```

A slot is held until the child's CPU usage decays, not for a fixed delay —
`spawn()` returns in microseconds, so a delay-based gate would have to be tuned
per machine and per launch file to be either safe or fast.

There is also a runnable-task ceiling (`max_runnable_factor`), off for a
separate measured reason: it cannot tell "busy because we are starting things"
from "busy because the things we started are running", and at `2 * ncpu` it made
startup *worse* than no pacing at all.

## Start order

Nothing above changes *what* starts first. For however long the stack takes to
come up, sensor drivers publish into nodes that do not exist yet, and that
backlog — DDS history caches plus the callback queues of subscribers appearing
part-way through — is the suspected path from "the machine is busy" to "the OOM
killer took the desktop".

Starting the drivers last bounds it: by the time anything is published, the
things that consume it are listening.

```yaml
startup:
  order:
    - name: sensor-drivers
      match:
        - "/sensing/lidar/falcon/seyond_node"
        - "/sensing/lidar/vlp32/velodyne_ros_wrapper_node"
        - "/sensing/camera/**"
        - "/sensing/imu/xsens/**"
  stage_timeout_secs: 30
```

Anything not matched is stage 0 and starts immediately; each later group waits
until every earlier member is up. Name the drivers rather than matching a whole
namespace — `/sensing/**` also catches consumers like `imu_corrector` and
`concatenate_data`, which belong in stage 0 with everything else they feed.

A stage waits for members to appear in the **ROS graph**, not merely to have
been spawned: `spawn()` returns long before a node has constructed its
subscriptions. Members that die are counted as accounted for — a driver whose
hardware is absent never appears, and must not hold up the rest of the system —
and the timeout names whatever is still missing when it fires.

One limitation worth knowing: for a node the launch file did not give a `name=`,
the model keys it by its *executable* while the running node registers its own
compiled-in name, so no graph match is possible. play_launch detects that case
and falls back to "the process is running" for those nodes. On the golf cart
stack that is 17 of 144.

There is also `defer_sources: true`, which derives the last group from the
model's topic graph (publishes but never subscribes → last). It is inert unless
manifests have been authored, since a plain launch file resolves to zero topics.

Ordering is **off by default**: it is a semantic change, and a system where
something waits on a driver being up early would break.

## Composable loading

`composable_node_loading.max_concurrent_load_node_spawn` (default 10) bounds how
many LoadNode calls are in flight across all containers at once. Before phase-61
this field existed but nothing read it, so every composable in the launch was
dispatched simultaneously. Set it to `0` for the old unbounded behaviour.

## Full config reference

```yaml
startup:
  enabled: true            # false restores pre-phase-61 spawn-immediately
  max_concurrent: ~        # ~ or 0 = unlimited (default)
  min_available_mb: ~      # ~ = 1 GiB absolute, capped at RAM/4
  max_runnable_factor: 0.0 # 0 = off; otherwise multiplied by core count
  max_gate_wait_secs: 30   # then admit anyway, with a warning
  max_settle_secs: 15
  settle_threshold_pct: 20.0
  order: []                # [] = no staging (default)
  defer_sources: false     # derive a final group from the topic graph
  stage_timeout_secs: 30

composable_node_loading:
  max_concurrent_load_node_spawn: 10   # 0 = unlimited
```
