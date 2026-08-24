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

## When the launch makes your desktop unusable

A common report from a developer bench: bring up an Autoware stack under
`--container-mode isolated`, and the desktop goes to treacle — windows lag,
video stutters, the mouse skips. It looks like the runqueue is saturated, and it
is, but not in the way that first suggests.

### What is actually starved

Not CPU *share* — CFS still hands the compositor its slice. What it loses is
**milliseconds inside a 16.6 ms frame**. A compositor wakes 60 times a second
and needs several milliseconds of CPU each time to build a frame; with a few
hundred runnable tasks it gets that CPU too late, and a frame that lands after
its deadline is a frame you see drop.

The distinction is not academic, and it is easy to measure the wrong thing. A
probe that wakes at 60 Hz and does *nothing* is barely affected by 144 CPU
burners (median wakeup 10 us), because the scheduler serves a sleeper promptly
however deep the queue. That result is true and useless. Model a compositor that
does real per-frame work and the same load gives:

| load | frame p50 | frames late |
|---|---|---|
| none | 11.5 ms | 0 / 360 |
| 144 processes, `nice 0` | **4226 ms** | **299 / 300 (99.7%)** |
| 144 processes, `nice 10` | 3.85 ms | 0 / 300 (0.0%) |

Three repetitions, same result each time. The two probes disagree by four orders
of magnitude on identical load
(`scripts/cgroup-probes/frame_budget.c`).

### The fix, in order of effort

**1. Renice the launch.** No setup, no privilege, no config:

```bash
nice -n 10 play_launch launch autoware_launch planning_simulator.launch.xml
```

`nice` is inherited across `fork()` and `exec()`, so every one of those hundreds
of processes gets it. In the measurement above this is the whole difference
between dropping essentially every frame and dropping none.

Why it works, concretely: a compositor sharing a core with ~12 launch processes
gets `1024/(12x1024+1024)` = 7.7% of that CPU at `nice 0` — under the ~24% it
needs — and `1024/(12x110+1024)` = 44% at `nice 10`, which fits.

**This is a bench recommendation, not a vehicle one.** It is a deliberate trade:
you are making the whole launch yield, including its sensor drivers, to protect
a desktop. On a vehicle there is no desktop to protect and driver latency is the
thing you care about, so do not carry this into a deployment.

**2. Use fewer processes.** `--container-mode observable` measured 3.9 cores
against `isolated`'s 10.2 on the same stack — see the container-mode decision at
the top of this page, including what fault isolation you give up for it.

**3. The principled version, if you have root once.** `nice` has one structural
weakness: weights **sum**. 144 processes at `nice 10` still out-weigh a handful
of desktop tasks in aggregate, and doubling the launch halves the desktop's
share again. A cgroup `cpu.weight` on the launch *group* is invariant to process
count — the group gets its share whether it holds 60 processes or 1000 — and it
leaves scheduling **inside** the launch untouched, so a sensor driver still
preempts a compute node exactly as it normally would.

That needs the CPU controller delegated to your user session, which by default
it is not:

```
/sys/fs/cgroup              subtree='cpuset cpu io memory pids'
/sys/fs/cgroup/system.slice subtree='cpuset cpu io memory pids'
/sys/fs/cgroup/user.slice   subtree='memory pids'          <- stops here
```

Controllers are enabled one level at a time, so nothing in your session has a
`cpu.weight` knob. The fix is a root-side drop-in:

```ini
# /etc/systemd/system/user@.service.d/delegate.conf
[Service]
Delegate=cpu cpuset io memory pids
```

Then `systemctl daemon-reload` and **re-login**. This is a documented, standard
practice — it is how rootless Podman and Docker get resource limits — but read
the three caveats in
[phase 66 W4](../roadmap/phase-66-cgroup-per-container.md) before applying it,
particularly the one about real-time tasks, which interacts directly with
play_launch's own RT scheduling.

## Bounding a container, and choosing what a failure takes with it

Everything above is best-effort: `oom_score_adj` volunteers a victim, the
memory floor delays a spawn. Neither is a limit. cgroup v2 gives you real ones,
per container, plus something a ROS container cannot offer at all — a choice
about **what a failure destroys**.

### It costs no configuration, and one wrapper

The grouping needs nothing from you. Your launch file already says which
composables belong to which container, and that *is* the group: play_launch
creates a cgroup per node and per container, and because the container forks
its composables, they land in its group automatically.

What it does need is somewhere it is allowed to create cgroups. A login shell
lives in a `session-N.scope` that systemd owns and no ordinary user can write,
so started normally, play_launch cannot do any of this and says nothing about
it. Start it inside a scope of your own and it can:

```bash
systemd-run --user --scope play_launch up system_model.yaml
```

You will see one line confirming it:

```
INFO cgroups: per-container grouping active under /sys/fs/cgroup/user.slice/...
```

Two things improve immediately, with no config at all:

- **Per-container memory becomes correct.** play_launch used to report a
  container's memory by summing its own RSS with each composable's, which counts
  every shared page once per process — and under `--container-mode isolated`
  every child is the same `component_node` binary plus the same rclcpp/rmw/DDS
  libraries. Measured on one six-composable container: **173 344 kB reported
  against 82 172 kB actually charged, 210% of truth.** The group's own figure
  counts shared pages once.
- **Teardown becomes atomic.** `cgroup.kill` removes every member at once,
  including a composable whose container died first — which a process-group kill
  can miss.

### Limits, if you want them

```yaml
cgroups:
  limits:
    - match: ["/perception/**"]
      memory_high_mb: 4096
      oom_group: true
    - match: ["**"]
      pids_max: 2048
```

Globs match node FQNs, exactly like `startup.order`, and the **first matching
rule wins** — put the specific pattern above the general one. Everything
defaults unset, deliberately: a wrong `memory_max_mb` turns a slow launch into a
killed one, and the right number is a property of your vehicle that nobody else
can guess.

**Prefer `memory_high_mb` to `memory_max_mb`.** `high` is a throttle: exceeding
it puts the group under reclaim pressure and slows it down, and nothing is
killed. `max` is a hard ceiling that ends in an OOM. `high` is the principled
version of what `oom_score_adj` gropes at — instead of nominating a victim in
advance, you make the launch yield memory rather than take it.

`pids_max` bounds tasks including threads. A ROS node is 11–22 threads measured,
so a ten-composable container is a couple of hundred; the ceiling turns runaway
thread creation into a clean `EAGAIN` instead of a board you cannot log into.

### `oom_group` — the setting worth thinking about

This one decides whether a container is a **fault unit**.

| | what an OOM kills |
|---|---|
| `oom_group: false` *(default)* | only the offending process |
| `oom_group: true` | every member of the container, together |

Neither a real ROS container nor plain separate processes lets you choose. A ROS
container shares one address space, so its composables die together whether or
not that was anyone's intent; `--container-mode isolated` forks a process each,
so they die alone. This is the choice made explicit.

The default is `false`, because per-node survival is what isolation is *for*.
Set `true` where partial survival is the more dangerous outcome — a pipeline
container that loses one stage keeps publishing stale data or nothing at all,
and a supervisor will restart a dead thing while never noticing a degraded one.
A clean failure you can see beats a quiet one you cannot.

Note this is about **memory** only. A segfault is still contained to one process
either way; no cgroup setting changes that.

### What it reports

At shutdown, any group whose memory counters moved is named — and only those,
because a line of zeroes per container per launch is how a real signal gets
scrolled past:

```
WARN cgroups: container mt_container: hit memory.max 139x
```

Those are the kernel's own counters, not an inference. A throttle is reported at
INFO (the limit holding the line); a refused allocation or a kill at WARN (the
kernel having already acted).

That example is also worth reading as a warning about `max`: 139 refusals and
**zero** kills. The kernel reclaimed instead, because those composables held
little anonymous memory. `memory.max` throttles well before it kills — which is
another reason to reach for `memory_high_mb` first.

### If you configure limits and they do not apply

You get told:

```
WARN cgroups.limits: 2 rule(s) configured but NONE were applied — play_launch
cannot create cgroups here. Start it under `systemd-run --user --scope`.
```

Configured limits are never dropped silently. If you configured nothing and
grouping is unavailable, play_launch says nothing at all — that is the ordinary
case and not worth a line on every launch.

### What this does not do

It does not make the launch faster or lighter. Process count, thread count and
runqueue depth are untouched, and on this hardware the process count is what
dominates — see the container-mode decision at the top of this page. What you
get here is accounting that is correct, teardown that is atomic, and limits that
exist.

CPU limits (`cpu.weight`, `cpu.max`) are **not** available: systemd delegates
only `memory` and `pids` to a user session by default. Enabling them needs a
root-side drop-in:

```ini
# /etc/systemd/system/user@.service.d/delegate.conf
[Service]
Delegate=cpu cpuset io memory pids
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

Under the default `--container-mode isolated`, loads do not use the ROS
`LoadNode` service at all (phase 64). play_launch hands its own container one
end of a socketpair before forking it and sends load requests down that; the
container answers with the id it pre-assigned, a liveness line every 15 s while
a constructor runs, and the final result.

This exists because of a measurement, not a preference. On a 12-core Orin
launching 144 processes, one run produced **14** `LoadNode service call timed
out after 30s` warnings and **8** `ComponentEvent LOADED not received after
10s` warnings — with **zero** composables actually failing. The container had
answered each request in microseconds; what was congested was the rmw service
layer, during precisely the window when load status matters.

You will see this line once per container:

```
container:/perception_container: loading composables over the private control
channel (no LoadNode service calls)
```

and, for a component whose constructor takes a while (a TensorRT engine build):

```
container:/tl_container: 'composable:/traffic_light_classifier' still
constructing after 30s (pid 41213, alive — reported by the container)
```

That line is the container reporting, not play_launch guessing. It is what
distinguishes a slow constructor from a wedged one.

Nothing here changes a container play_launch does not own: `--container-mode
stock` keeps the LoadNode service exactly as before, and so does `observable`
(where loading has to run on the container's executor thread). If a container
binary is older than play_launch it simply never answers the hello, and the
service path takes over after `control_hello_timeout_ms`. To turn the channel
off entirely, set `composable_node_loading.control_socket: false`.

### When a load goes quiet

play_launch never fails a composable for taking too long. If reporting stops it
ASKS the container, and acts only on the answer:

```
container:/pointcloud_container: '/perception/.../lidar_centerpoint' is
constructing at 47s (pid 41213) — still in flight, not resending
```

A resend happens only when the container states it has no record of the load —
which means nothing was forked, so there is nothing to duplicate. Everything
else waits and is reported. `max_load_attempts` (default 2) caps the total.

Stall detection is **off** by default. Turning it on requires saying what a
stall means on your hardware, because there is no number that is right on every
board:

```yaml
composable_node_loading:
  stall_after_secs: 120      # 0 (default) = never declare a stall
  stall_action: report       # report | fail | restart
```

Even with a budget set, a constructor counts as stalled only if it is alive,
past that budget, AND burning no CPU — and `report` (the default) just says so,
because a node blocked on a service that has not come up yet looks identical to
a wedged one. `fail` and `restart` route through a cancel: the container kills
the child and confirms it is gone before anything is loaded again.

`composable_node_loading.max_concurrent_load_node_spawn` (default 10) bounds how
many LoadNode calls are in flight across all containers at once. Before phase-61
this field existed but nothing read it, so every composable in the launch was
dispatched simultaneously. Set it to `0` for the old unbounded behaviour. It
governs the service path only — on the socket path there is no call to be in
flight, and the container paces its own spawns against `MemAvailable`.

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
  max_concurrent_load_node_spawn: 10   # 0 = unlimited; service path only
  control_socket: true                 # false = always use the LoadNode service
  control_hello_timeout_ms: 10000      # then fall back to LoadNode
  ack_timeout_ms: 5000                 # unacknowledged load -> ask the container
  report_timeout_secs: 45              # silence -> ask the container
  probe_interval_secs: 15              # re-ask cadence when a query is unanswered
  max_load_attempts: 2                 # 2 = one resend, after a CONFIRMED absence
  stall_after_secs: 0                  # 0 = never declare a stall
  stall_cpu_threshold_pct: 1.0
  stall_action: report                 # report | fail | restart
  composable_respawn: off              # off | on-crash

cgroups:
  # Inert unless play_launch can create cgroups — start it under
  # `systemd-run --user --scope`. First matching rule wins.
  limits: []
    # - match: ["/perception/**"]   # globs over node FQNs, as in startup.order
    #   memory_high_mb: 4096        # throttle by reclaim; nothing is killed
    #   memory_max_mb: ~            # hard ceiling, ends in an OOM. Prefer high.
    #   pids_max: 2048              # tasks including threads
    #   oom_group: false            # true = the container dies as a unit
```
