# Real-Time Scheduling Guide

`play_launch` can apply Linux real-time scheduling — `SCHED_FIFO`/`SCHED_RR`
policy, RT priority, and CPU affinity — to every process it launches, driven by
a portable scheduling spec you write once. It works **without root**: the
privileged syscalls are delegated to a small capability-holding helper.

This guide explains the configuration structure and, precisely, how each piece
of configuration maps onto the processes and threads the kernel actually
schedules.

```
you write                play_launch resolves            the kernel sees
─────────                ────────────────────            ───────────────
system.toml   ──┐
                ├──►  SchedPlan: FQN → tier  ──►  per-THREAD SCHED_FIFO/RR
launch file   ──┘     (via record.json)           + priority + CPU affinity
                                                  on every launched process
```

## Quick start

A runnable copy of this example lives at `tests/fixtures/rt_workspace/` — a
real colcon workspace (`rt_demo`: a 100 Hz sensor publisher, a FIFO-pinned
control node, and one composable filter) with the launch file, contract, and
`system.toml` committed:

```bash
just build && just setcap        # one-time: grant the helper its capability
cd tests/fixtures/rt_workspace
just build                       # colcon build of the rt_demo package
just check                       # validate contract + system.toml statically
just run                         # launch with --sched system.toml
```

While it runs, prove every thread of the control node got the policy
(class `FF`, rtprio 20, cpu 0):

```bash
ps -eLo tid,cls,rtprio,psr,comm | grep control_node
```

(`just verify-sched-rt` at the repo root demonstrates the same check
end-to-end against a generated spec.)

Its `system.toml` (control tier pinned to FIFO 20 on core 0, perception
tier at FIFO 10):

```toml
[tiers.control]
class = "real_time"

[tiers.control.posix]
priority    = 20
sched_class = "SCHED_FIFO"
core        = 0

[tiers.perception]
class = "real_time"

[tiers.perception.posix]
priority    = 10
sched_class = "SCHED_FIFO"

[[assign]]
tier  = "control"
nodes = ["control_node"]

[[assign]]
tier  = "perception"
scope = "/perception"
```

No `sudo` anywhere. Nothing runs as root.

---

## 1. Configuration structure

The spec has three layers, deliberately separated:

| layer | section | portable? | contains |
|---|---|---|---|
| **Tier (generic head)** | `[tiers.<name>]` | yes — identical across Linux and RTOS targets | scheduling *class* and timing *requirements* |
| **Tier (placement)** | `[tiers.<name>.<platform>]` | per-platform values, same shape everywhere | concrete priority / affinity numbers |
| **Binding** | `[[assign]]` | yes | which nodes belong to which tier |

The same file drives `play_launch` on Linux (`posix` placement) and nano-ros on
RTOS targets (`freertos`, `zephyr`, `threadx`, `nuttx` placements). That is why
**no priority number may appear in the generic head** — priorities are not
portable (FreeRTOS counts up, ThreadX counts down, Zephyr has negative
cooperative priorities). The schema rejects a stray `priority` at the head
(`deny_unknown_fields`).

### 1.1 Tiers — `[tiers.<name>]`

A **tier** is a named class of work with common scheduling properties
("control loop", "perception", "logging"). The generic head describes *what
the tier requires*, never *how a specific OS delivers it*:

```toml
[tiers.control]
class           = "real_time"      # best_effort | real_time | time_triggered | interrupt
deadline_us     = 50000            # relative deadline (µs)
period_us       = 20000            # activation period, for periodic work
budget_us       = 5000             # execution-time budget (EDF/sporadic)
deadline_policy = "warn"           # ignore | warn | skip | fault (on deadline miss)
spin_period_us  = 1000             # executor spin period
```

Every field is optional. On Linux today, `class`, `deadline_us`, `period_us`,
`budget_us` are **validated but not enforced** — they map to `SCHED_DEADLINE`,
which is deferred (a `time_triggered` tier logs a warning). They matter for
the RTOS targets and for future Linux support.

### 1.2 Placement — `[tiers.<name>.posix]`

The per-platform sub-table holds the numbers. `posix` is the Linux target:

```toml
[tiers.control.posix]
priority    = 20             # RT priority: 1..=99 (validated at startup)
sched_class = "SCHED_FIFO"   # SCHED_FIFO | SCHED_RR | SCHED_OTHER (or omit)
core        = 0              # pin to this CPU (omit = unpinned)
deadline_us = 40000          # optional: tighten the generic deadline per-platform
```

`stack_bytes` and `preempt_threshold` exist in the schema for RTOS targets and
are ignored on Linux.

Rules enforced **at startup, before anything spawns**, in every mode:
- `SCHED_FIFO`/`SCHED_RR` priority must be in `1..=99`.
- `core` must be a CPU that exists.
- Every tier that has members must have a sub-table for the resolve target
  (`posix` on Linux).
A violation aborts with an error naming the tier — a malformed spec is a spec
bug, not a runtime condition.

### 1.3 Binding — `[[assign]]`

Sparse selector rules bind nodes to tiers. You do not enumerate every node;
you promote the ones that matter and let the rest default:

```toml
[[assign]]
tier  = "control"
nodes = ["ndt_localizer", "ekf_localizer"]   # explicit: FQN or bare name

[[assign]]
tier  = "perception"
scope = "/perception/lidar"                   # subtree: every node under this namespace
```

Selector semantics:

| selector | matches | precedence |
|---|---|---|
| `nodes = [...]` | full FQN (`/ns/name`) or bare last segment (`name`) | highest |
| `scope = "..."` | every node whose namespace equals or is under the path | middle |
| *(unmatched)* | synthesized `default` tier — **no scheduling applied** | lowest |

- An explicit `nodes` rule silently beats a `scope` rule for the same node —
  that's the intended override mechanism.
- Two rules at the **same** precedence level claiming one node for different
  tiers is an error (`NodeMatchedByMultipleTiers`).
- A selector that matches nothing is an error (typos fail loudly, not
  silently).

### 1.4 CLI

```bash
play_launch launch <pkg> <file> --sched system.toml [--sched-apply off|warn|strict]
play_launch check <pkg> <file> --contracts contracts/ --sched system.toml   # validate only
```

| `--sched-apply` | behavior |
|---|---|
| `off` | resolve + report the tier table; **no syscalls** |
| `warn` *(default)* | apply; on failure log a warning, node keeps default scheduling |
| `strict` | any privilege or apply failure **aborts the run** — all-or-nothing RT |

Without `--sched`, the entire subsystem is inert.

---

## 2. How configuration maps to the RT execution model

### 2.1 Launch entities → OS processes

Everything `play_launch` launches is an OS process, and the process is the
unit your `[[assign]]` selectors target:

| launch entity | OS process | scheduled? |
|---|---|---|
| `<node>` | one process per node | ✅ |
| `<node_container>` | one container process | ✅ (its own tier) |
| composable node (`isolated` mode, default) | one `component_node` process, fork+exec'd by the container | ✅ (its own tier) |
| composable node (`observable`/`stock` mode) | a *thread group inside* the container — no process of its own | ❌ only the container is schedulable |

### 2.2 Resolution: spec + launch → plan

At startup, before any process spawns:

1. The launch file is parsed to `record.json` (nodes, containers, composables,
   and the namespace **scope table**).
2. Every entity is flattened to a fully-qualified name — its own namespace,
   falling back to the launch scope's namespace: `/perception/lidar/voxel_grid`.
3. Your `[[assign]]` selectors run against those FQNs (explicit > scope >
   default), producing the **SchedPlan**: `FQN → {policy, priority, core}`.
4. Config validation happens here (priority range, core bounds, missing
   placement) — bad specs fail before anything runs, in every mode.

Each spawned member is handed its own resolved entry; nothing is re-resolved
at runtime.

### 2.3 Apply timing — and why it's per-thread

Linux scheduling attributes belong to **threads**, not processes. A ROS node
starts as one thread and grows to ~11 (DDS/rmw spawn ~10) within half a second
of exec. `play_launch` therefore applies the tier to **every thread**
(`/proc/<pid>/task/*`), and threads created afterwards inherit the policy from
their creator (glibc `PTHREAD_INHERIT_SCHED`):

| entity | applied when | threads at that moment |
|---|---|---|
| node / container | immediately after spawn (~1 ms) | 1 — later threads inherit |
| composable | on its `LOADED` event (after init) | ~11 — the per-thread sweep covers them all |
| respawned process | re-applied automatically on each respawn | — |

Two safety details on the composable path:
- The `ComponentEvent` carries the child's `pid` **and its start time** (the
  PID's kernel birth tick). The apply is skipped with a warning if the live
  process no longer matches — a recycled PID can never be scheduled by
  mistake.
- A composable whose event carries `pid = 0` (non-isolated modes) is skipped.

> Verification caveat: `chrt -p <pid>` reads **only the main thread**. To see
> what actually happened, check every TID — `just verify-sched-rt` does.

### 2.4 Privilege: the RT helper

An unprivileged process cannot request RT scheduling (`RLIMIT_RTPRIO` is 0 for
normal users). `play_launch` solves this without running anything as root:

```
play_launch  (uncapped, links ROS, runs as you)
    │  ApplySched{pid, tier}   — pipe IPC, once per process at spawn/load
    ▼
play_launch_rt_helper  (ROS-free, holds CAP_SYS_NICE only)
    → sched_setscheduler / sched_setaffinity on every thread of that pid
```

- `just setcap` (or `play_launch setcap`) grants `cap_sys_nice+ep` to the
  helper (and `cap_sys_ptrace+ep` to the I/O-monitoring helper). File
  capabilities do not change the user — the helper runs as *you*, with exactly
  one extra permission. **Capabilities live on the file inode: re-run
  `just setcap` after every rebuild.**
- The main `play_launch` binary must **never** be capability-granted: a file
  capability triggers secure-execution mode (`AT_SECURE`), the loader ignores
  `LD_LIBRARY_PATH`, and the binary can no longer find its ROS libraries.
  `setcap` strips a stray capability from it automatically; `verify` flags one
  as a fault. This is precisely why the helpers exist.
- The helper's lifetime is managed: it dies with the parent (`PDEATHSIG`), is
  killed on drop, shuts down gracefully on normal exit, and every request has
  a 5 s timeout — a wedged helper degrades per `--sched-apply`, never hangs
  the launch.
- Running as **root** still works (`sudo -E`, with `LD_LIBRARY_PATH`
  re-injected): the syscalls are then made directly, no helper needed. This is
  the fallback, not the recommended path.
- IPC cost is control-plane only: one round-trip per process at
  spawn/respawn/load. Steady-state cost is zero — the kernel does the
  scheduling from then on.

### 2.5 What the kernel ends up with

For a node in `[tiers.control]` with `posix = { priority = 20, sched_class =
"SCHED_FIFO", core = 0 }`:

- every thread: policy `SCHED_FIFO`, `rt_priority 20` — preempts all
  `SCHED_OTHER` work on its CPU; among RT threads, higher priority wins;
  `SCHED_RR` additionally time-slices equal priorities
- every thread: affinity mask `{0}` — the whole process is pinned to CPU 0
- unassigned (`default`-tier) nodes: untouched — normal `SCHED_OTHER`

Kernel-level notes:
- **RT throttling**: by default Linux caps RT execution at 95% of each second
  (`sched_rt_runtime_us`), so a runaway FIFO loop cannot wedge the machine.
- On kernels with `CONFIG_RT_GROUP_SCHED`, a cgroup with `rt_runtime = 0` can
  still refuse RT even with the capability — you'll see `EPERM` despite a
  correct setup.
- `SCHED_DEADLINE` (from `deadline_us`/`period_us`/`budget_us`) is not applied
  on Linux yet.

---

## 3. Verifying

```bash
play_launch verify        # capability state of all three binaries
just verify-sched-rt      # launches a fixture UNPRIVILEGED and checks EVERY thread
```

Expected `verify-sched-rt` output:

```
  talker    pid=12345  threads=11  fifo=11/11  prio=20  cpu=0
  listener  pid=12346  threads=11  fifo=11/11  prio=20  cpu=0
```

`fifo=11/11` is the number that matters: all threads, not just the main one.

Manual, for a live system:

```bash
pid=$(cat play_log/latest/node/<name>/pid)
for t in /proc/$pid/task/*; do chrt -p ${t##*/}; done   # per-thread policy
taskset -cp $pid                                        # affinity
```

## 4. Troubleshooting

| symptom | cause | fix |
|---|---|---|
| warn: `no privilege to apply ... cap_sys_nice` | helper not capability-granted | `just setcap` |
| worked yesterday, EPERM today | rebuild replaced the binaries — caps are per-inode | `just setcap` again |
| `error while loading shared libraries: lib...rosidl...` | someone ran `setcap` on the **main** binary (`AT_SECURE` drops `LD_LIBRARY_PATH`) | `just setcap` (it strips it) |
| `tier 'X': RT priority N out of range 1..=99` at startup | bad placement value | fix the spec — this is deliberate fail-fast |
| assign rule error: selector matches nothing | typo in node name / scope path | check FQNs with `play_launch context record.json --tree` |
| composable skipped: `pid no longer matches its start_time` | the composable died right after loading; its PID may have been recycled | benign — nothing to schedule |
| `chrt -p <pid>` shows `SCHED_OTHER` but the node "should" be RT | `chrt -p` reads only the main thread — or the node is in the `default` tier | check all TIDs; check your `[[assign]]` coverage |
| EPERM with caps correct, inside a container/slice | cgroup `rt_runtime = 0` gate (`CONFIG_RT_GROUP_SCHED`) | provision RT bandwidth for the cgroup |
| strict run aborts at startup | that's strict working: no privilege or bad spec = no launch | `just setcap`, or use `warn` |

## 5. Related documents

- Spec schema in depth: [`src/ros-launch-manifest/docs/scheduling.md`](../../src/ros-launch-manifest/docs/scheduling.md)
- Design of record (apply-layer): [`docs/superpowers/specs/2026-07-06-linux-sched-apply-layer-design.md`](../superpowers/specs/2026-07-06-linux-sched-apply-layer-design.md)
- Design of record (RT helper, per-TID, capabilities): [`docs/superpowers/specs/2026-07-14-rt-helper-design.md`](../superpowers/specs/2026-07-14-rt-helper-design.md)
- Roadmap and implementation history: [`docs/roadmap/phase-38-linux_rt_scheduling.md`](../roadmap/phase-38-linux_rt_scheduling.md)
