# Phase 66 — cgroup-per-container: the properties, without the container

Status: **W1 in progress**. Design study, with every claim measured on the
bench Orin (`5.15.148-tegra`, 12 cores, 61 GiB):
[docs/design/cgroup-per-container.md](../design/cgroup-per-container.md).
Probes are in `tmp/` (`cgroup_probe.sh`, `oom_group_probe.sh`,
`delegation_detect.sh`, `freeze_probe.sh`, `accounting_probe.sh`,
`mechanics_probe.sh`).

## Why

`--container-mode isolated` forks a process per composable to buy fault
isolation, and in doing so has already surrendered everything that makes a ROS
container a container — shared address space, shared executor, one DDS
participant. What survives is a launcher wearing a container's costume: the
isolated `on_unload_node` is, in full, deregister a pidfd, kill the child,
erase a map entry.

cgroup v2 gives the *resource and lifecycle* half of a container back to a set
of separate processes, and gives it back **better**, because the failure model
stops being implied and becomes a choice:

```
oom.group=1  bystander=dead   hog=dead   oom_kill=3     <- container semantics
oom.group=0  bystander=ALIVE  hog=dead   oom_kill=1     <- fault isolation
```

A real ROS container gives you the first row and no say in it. Separate
processes give you the second and no say in it. `memory.oom.group` makes it a
per-container decision by whoever knows the system — and a segfault stays
per-process either way, which no cgroup setting changes.

The IPC and executor half is **not** recoverable. `cgroup.type=threaded` fails
under a subtree with the memory controller enabled, because `memory` is
domain-only: memory is attributable to a process, never to a thread. So
per-node memory accounting *requires* per-node processes, while zero-copy IPC
*requires* one shared process. Nothing bridges that, and this phase does not
pretend to.

## Scope boundary — no performance claim

Nothing in this phase reduces process count, thread count or runqueue depth.
Phase 61 measured process count as the driver of the startup storm and that is
untouched here. What this phase delivers is **semantics a design had given up**
— grouped limits, a chosen failure model, atomic teardown — plus one number
that is wrong today and becomes right.

Any performance effect is unmeasured and should be treated as absent until it
is not. This is the discipline phase 61 learned the hard way (spawn pacing
doubled startup for a 10% gain) and that W1's own `_mt` investigation
re-learned (a 2x thread reduction moved peak runnable not at all).

## Work items

### W1 — probe, place, account, tear down

The whole mechanism, no limits. Only two visible changes: memory numbers
become correct, and teardown becomes atomic.

- Probe at startup by attempting `mkdir` + self-move. The capability
  **cannot** be read off the cgroup path or the controller list — a login
  shell and a `systemd-run --user --scope` both report
  `controllers: memory pids` and differ only on the `mkdir`. Same shape as
  phase 60's `partition root` writing successfully and reading back
  `root invalid`: the write is not the test, the effect is.
- Build `supervisor/`, `node/<dir>/`, `container/<dir>/`. play_launch steps
  into `supervisor/` first — a cgroup holding processes cannot enable
  controllers for its children.
- Place at spawn: `to_command` takes a cgroup path, the existing `pre_exec`
  (`node_cmdline.rs:829`) writes `"0\n"` to `cgroup.procs` beside
  `bias_oom_score()`. Composables need no work — they inherit the container's
  group at fork.
- Read `memory.current` for the group instead of summing child RSS
  (`resource_monitor.rs:237`), which over-counts shared pages **2.4x** on six
  processes and far worse across a container of identical `component_node`
  binaries.
- `cgroup.kill` on teardown, alongside the existing PGID kill.
- Degrade to exactly today's behaviour, with one log line, when unavailable —
  which is the **default** path, since play_launch normally starts from a
  terminal.

**Acceptance:** a launch under `systemd-run --user --scope` reports container
memory that matches `memory.current` rather than a sum of RSS; the same launch
from a plain terminal behaves identically to today and says so once; no test
requires delegation to pass.

### W2 — limits and the failure model

`pids.max`, `memory.high`, `memory.max` and `memory.oom.group` per container,
from config. Defaults stay unset/0 — W2 makes them *expressible*, not
mandatory.

`memory.high` is the interesting one: it throttles by reclaim pressure instead
of killing, which is the principled replacement for guessing with
`oom_score_adj`. `pids.max` bounds the thread explosion directly — a ROS node
is 11–22 threads measured, and 93 composables is roughly 2000.

**Acceptance:** a container with `oom.group=1` loses all members together and
reports it as one event; with `0` it loses only the offender. `memory.events`
counters are surfaced rather than inferred.

### W3 — `cgroup.freeze` for staged startup

Phase 61 W2 holds sensor drivers by waiting for consumers to appear in the ROS
graph. `cgroup.freeze` is strictly stronger: a frozen driver *cannot* publish,
where a spawned-but-unsubscribed one merely shouldn't. Verified working
(`frozen 1`, no progress until thawed).

**Blocked on a measurement, not on code.** A frozen process holding DDS
sockets may be timed out of discovery by its peers, which would make this worse
than the graph wait it replaces. Measure that before writing any of it.

### W4 — CPU, only if the delegation ships

`cpu`, `io` and `cpuset` are enabled at the cgroup root and dropped at
`user.slice`; `DelegateControllers` reports `memory pids`. One root-side
drop-in changes it:

```ini
# /etc/systemd/system/user@.service.d/delegate.conf
[Service]
Delegate=cpu cpuset io memory pids
```

Same class of provisioning as `scripts/provision_rt_cpuset.sh`. If it ships,
`cpu.weight` (proportional share), `cpu.max.burst` (bounded steady state,
unbounded startup spike) and `cpu.idle` (whole group at `SCHED_IDLE`, and
present on this kernel) become available — and `cpu.idle` would replace the
per-process `SCHED_BATCH` work with one group setting.

**Not started, and lower priority than it looks:** the unprivileged
`SCHED_BATCH`/nice path needs no delegation at all and protects an operator's
session on any machine.

## Not doing

- **Per-composable groups nested under the container.** It would give per-node
  limits *and* a container-wide ceiling — genuinely both properties at once —
  but it is the only change here that requires `play_launch_container` to place
  each child. Wait until someone wants per-composable limits.
- **Self-re-exec under `systemd-run`.** Drags tty, signal, exit-code and
  subreaper handling into an optional feature.
- **Anything built on PSI.** `/proc/pressure` does not exist on
  `5.15.148-tegra`; `memory.pressure` is absent from every cgroup. Pressure
  stall information is the right startup-governor signal and it is not on the
  target hardware, so a design leaning on it would work on a developer desktop
  and degrade silently on a vehicle.
