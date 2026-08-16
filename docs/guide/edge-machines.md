# Running on an edge machine

play_launch's defaults assume a workstation. On a Jetson-class board — an AGX
Orin, say, with twelve cores instead of thirty-two — one of those defaults is
expensive enough to make a large launch unusable, and this page is about that
one plus the safety nets around it.

Everything here was measured on a 12-core AGX Orin bringing up a 44-node /
16-container / 84-composable Autoware stack. Full numbers and method:
[phase-61-edge-startup-storm.md](../roadmap/phase-61-edge-startup-storm.md).

## The short version

```sh
play_launch launch --container-mode observable <pkg> <launch_file>
```

If your launch has many composable nodes and your machine has few cores, that
one flag is worth more than everything else on this page combined.

## Why

`--container-mode isolated` is the default. It fork+execs every composable node
into its own process, so a node that crashes cannot take down the siblings
sharing its container. That is a real property and some systems need it.

The price is a process, an executor and a DDS participant per composable node
instead of per container. A launch with 16 containers and 84 composables runs
144 processes under `isolated` and 60 under `observable`, and on a 12-core board
the difference is not subtle:

| | `isolated` | `observable` |
|---|---|---|
| processes | 144 | 60 |
| spawn → startup complete | 10.7 s | **8.4 s** |
| peak load1 | 190 | **45** |
| peak runnable tasks | 506 | **128** |
| memory consumed during startup | 3.5 GiB | **1.4 GiB** |
| CPU during startup | 10.2 of 12 cores | **3.9 cores** |

Same 84/84 composables loaded, 2.6x less CPU, and faster. There is no
throughput argument for `isolated` on a machine this size — what it buys is
crash isolation and nothing else, so the question to ask is whether you need
that, not whether you can afford it.

`observable` keeps play_launch's `ComponentEvent` visibility, so the web UI and
the per-composable state tracking work exactly as before. `stock` gives up that
too; prefer `observable` unless you specifically want the launch file's own
container binary.

play_launch warns about this on its own when a launch would fork more than four
processes per core:

    WARN --container-mode isolated will start 84 extra processes (one per
    composable node) on a 12-core machine. ...

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
play_launch checks `MemAvailable` against a floor — 10% of RAM, clamped to
between 512 MiB and 4 GiB. Above the floor this costs nothing and never blocks.
Below it, admissions serialise until memory recovers, and after 30 s a node is
let through anyway with a warning naming the gate, because a launch that never
starts is worse than one that starts under pressure.

```yaml
startup:
  min_available_mb: 2048   # explicit floor
  # min_available_mb: ~    # default: 10% of RAM, clamped
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
  min_available_mb: ~      # ~ = 10% of RAM, clamped to [512, 4096]
  max_runnable_factor: 0.0 # 0 = off; otherwise multiplied by core count
  max_gate_wait_secs: 30   # then admit anyway, with a warning
  max_settle_secs: 15
  settle_threshold_pct: 20.0

composable_node_loading:
  max_concurrent_load_node_spawn: 10   # 0 = unlimited
```
