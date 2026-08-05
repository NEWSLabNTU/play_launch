# Phase 57 — Mixed-criticality RT demo + Chrome timeline export

**Design:** brainstormed 2026-08-05; this doc is the design of record.
**Motivation:** the RT apply path has never executed on any machine in this
repo. `per_tid_sched_fifo_launch_privileged_only` reports PASS while returning
early, because the host lacks `CAP_SYS_NICE`. Everything around RT is tested;
the part a user depends on is not.

## Problem

Three gaps, in order of how much they matter:

1. **No evidence RT does anything.** `--sched-apply strict` fails loudly
   without privilege and `warn` warns — both verified. Whether SCHED_FIFO,
   priorities and affinity change *observable timing* has never been shown.
2. **No timeline.** `metrics.csv` samples every 2 s — three orders of
   magnitude too coarse for scheduling. The interception layer already
   records every `rcl_publish`/`rcl_take` with a `CLOCK_MONOTONIC` timestamp,
   but only ever writes summary JSON.
3. **No mixed-criticality workload.** `rt_demo`'s three nodes are
   pass-through: they consume no CPU, so the scheduler has nothing to
   arbitrate and priority cannot matter.

## The claim under test

The safety chain `lidar → detector → brake` holds a **60 ms end-to-end
deadline** on an oversubscribed CPU set. With `--sched-apply strict` it holds;
with `off` the mission and best-effort load pushes it into misses.

Same binaries, same load, one flag.

## Two constraints this machine imposes

Both were measured, and both change the design.

**32 cores.** Six nodes on 32 cores never contend, so RT-on and RT-off produce
identical timelines and the honest reading is "scheduling changed nothing".
Contention is therefore *manufactured*: the workspace is confined with
`taskset` so runnable threads outnumber cores.

**Calibration (measured, supersedes the estimate above).** Two CPUs is not
enough to manufacture contention here. Measured on the RT-off baseline: on two
cores the chain missed **zero** deadlines at every load tried, up to 60 ms
best-effort burns (p99 stayed at 36 ms). The reason is CFS sleeper fairness —
the chain is a low-duty-cycle task that sleeps between frames, accrues low
vruntime, and is picked first on each wake. CFS is genuinely good at this, and
no amount of added best-effort load changes it while a spare core exists.

The calibrated operating point is therefore **one core** (`cpus := "2"`) with
four extra best-effort "tool" nodes at a 5 ms burn. Measured baseline over
three repetitions: p50 42-48 ms, p99 90-93 ms, **17-30% of frames past the
60 ms deadline, with no message loss** (~1015 of ~1015 delivered). Confining
to a single CPU makes the run sensitive to whatever else the host puts on that
core, so the miss rate is noisy across runs — 17% to 77% observed. It never
approached zero, which is all the guard requires, but it is not a stable
number and should not be quoted as one. Heavier
loads were rejected as dishonest: at a 10 ms tool burn the chain stops keeping
up and *drops* messages, which is system collapse rather than a deadline miss.

Note `sched_rt_runtime_us` = 950000, so on a single core the RT-on run leaves
best-effort work 5% of the CPU. The tool nodes will crawl. That is the
intended demonstration, not a fault.

**`PREEMPT_DYNAMIC`, not `PREEMPT_RT`**, and `/sys/devices/system/cpu/isolated`
is empty — note the existing fixture *declares* `isolated_cpus: [0]`, which is
a statement in a platform file, not kernel isolation. Worst-case latency is
therefore milliseconds. Deadlines are set at **60 ms**: comfortably
demonstrable, and a miss means scheduling rather than kernel jitter.

**The harness fails if the RT-off baseline does NOT miss deadlines.** A
baseline that passes means the experiment did not contend, so the result is
inconclusive — not a success. This is the vacuous-pass shape that issues 0008,
0012 and 0014 all had, and it is the single most likely way this demo lies.

**The check must judge steady state.** The first ~4 s of any run miss the
deadline by 150 ms from node discovery and DDS matching alone — misses that
scheduling cannot fix and that RT-on will show too. A first cut of this
harness counted them, which meant a workload that never contended at all
still satisfied "the baseline misses deadlines". The comparison therefore
discards the first `warmup` (default 200) samples on both sides, and the
baseline must miss *after* that.

## W1 — Chrome trace export from interception

The reusable half. Works on any header-bearing stack, Autoware included; it is
not specific to the demo workspace.

- `ChildConsumer` gains the owning node's name. It currently carries only
  `consumer`/`event_fd`/`shm_fd`, so events cannot be attributed to a node —
  the enabling change.
- New `interception.trace` setting (default off) writes
  `play_log/<ts>/interception/trace.json` in Chrome Trace Event Format,
  alongside the existing frontier/stats summaries.
- Event mapping:
  - `ph:"X"` complete events — a node's take→publish span
  - `ph:"s"`/`ph:"f"` flow events — link a publish to the matching take, so
    Chrome draws the chain as arrows between nodes
  - `ph:"i"` instant events — deadline misses
- Correlation key is the message header stamp. **Messages without a
  `std_msgs/Header` cannot be linked** — `rt_demo` uses `std_msgs/String` and
  is therefore unlinkable, which is why the new workspace uses stamped
  messages.

## W2 — `examples/rt_av_demo/` — a standalone ROS 2 workspace

Placed under `examples/`, not `tests/fixtures/`: it is a runnable
demonstration, not a fixture any test depends on. Plain `rclcpp` nodes with no
play_launch-specific code — the timeline comes from interception, so the nodes
stay ordinary ROS.

| Node | Rate | Criticality | Work |
|---|---|---|---|
| `/safety/lidar_driver` | 50 Hz | safety | publish stamped scan |
| `/safety/obstacle_detector` | on msg | safety | ~8 ms bounded |
| `/safety/brake_controller` | on msg | safety | emit brake command |
| `/planning/path_planner` | 10 Hz | mission | ~40 ms burst |
| `/perception/map_loader` | 2 Hz | best_effort | ~120 ms burst |
| `/telemetry/logger` | continuous | best_effort | steady burn |
| `/tools/{rosbag_recorder, diagnostics_aggregator, visualizer, health_monitor}` | 30 Hz | best_effort | 5 ms each |

The four `tools` nodes were added during calibration. They are the support
software any real stack runs alongside the driving code, and they matter
because CFS stretches a task by roughly (runnable threads / cores): thread
*count*, not just total load, is what pushes the chain past its deadline.
They need no platform-file entry — with no timing facts the mapper leaves
them `SCHED_OTHER`, which is correct for them.

Plus `bringup.launch.xml`, `bringup.contract.yaml` (declaring the
`lidar_to_brake` chain with its 60 ms budget and per-node `criticality`), and
`bringup.system.posix.yaml` (`chain_aware` mapper, priority band, the 2-CPU
confinement).

Burn durations are parameters, so the workload can be retuned without
recompiling when a different machine needs more or less pressure.

## W3 — `examples/rt_av_demo/justfile`

- `just build` — colcon build of the workspace
- `just trace` — one run, RT applied, dumps `trace.json`
- `just ab` — the experiment: same launch twice (`off`, then `strict`), two
  traces, and a comparison table
- `just check` — contract + schedule validation with `--explain`

Reported per run: chain latency p50/p99/max and deadline misses out of total.

## Not in scope

Kernel-level tracing (LTTng is not installed here, and `ros2_tracing` without
it produces nothing), CPU-occupancy timelines (interception sees messages, not
thread run-time), and any change to the scheduling mappers themselves.

## Prerequisite

`just setcap` — without `CAP_SYS_NICE` on `play_launch_rt_helper`, the RT-on
run is identical to RT-off and the whole experiment is inconclusive. The
harness must detect and refuse to report in that case rather than emit a
comparison showing no difference.

**Status: this prerequisite is unmet on the development host, so the RT-on
half has never run.** Measured: `ulimit -r` is 0 and `chrt -f 10 true` fails
with `Operation not permitted`, so `SCHED_FIFO` is unreachable for this user
by any route. `getcap` on the built helper returns nothing. Everything else in
this phase is verified end to end — the trace exporter, the workspace, the
baseline calibration above, and `just ab`'s refusal — but the central claim
("with `strict` the chain holds its deadline") is **unverified**. `just setcap`
needs a password and cannot be run unattended.

One consequence worth stating plainly: the 60 ms deadline is an a-priori
choice, not one calibrated against a measured RT-on distribution. The
principled version sets the deadline above RT-on p99 and below RT-off p99;
that requires the capability. If RT-on p99 lands near 60 ms, the deadline
should be revisited before this demo is used as evidence of anything.
