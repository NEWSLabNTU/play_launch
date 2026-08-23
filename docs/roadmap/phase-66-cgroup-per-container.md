# Phase 66 — cgroup-per-container: the properties, without the container

Status: **W1–W2 done** (`5d8618a`, W2 below). Design study, with every claim measured on the
bench Orin (`5.15.148-tegra`, 12 cores, 61 GiB):
[docs/design/cgroup-per-container.md](../design/cgroup-per-container.md).
Probes are in `tmp/` (`cgroup_probe.sh`, `oom_group_probe.sh`,
`delegation_detect.sh`, `freeze_probe.sh`, `accounting_probe.sh`,
`mechanics_probe.sh`, `freeze_discovery.sh`).

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

### W1 — probe, place, account, tear down ✅

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

**Result.** Met, on a live 6-composable container. `metrics.csv` recorded
173344 kB where the group was charged 82172 kB — **210% of truth** — and now
records 80120–80292 kB against a live `memory.current` of 80168 kB. Seven
members landed in the container's group with no change to
`play_launch_container`, and `cgroup.kill` reached the grandchildren.

Two defects that measurement caught, both of which would have shipped as
silent wrongness rather than as errors:

- **`cgroup.subtree_control` is enabled one level at a time.** The tree is two
  deep, so enabling `+memory` on the root alone left the leaf with no
  controllers — surfacing not as a failure but as `memory.current: 0` on a
  group with seven live members. The intermediate level now enables its own,
  and `memory_current()` treats a zero charge as *absent* rather than as a
  memory figure: replacing an over-count with an under-count is worse than
  either.
- **Availability is not readable, only probeable.** A login shell and a
  `systemd-run --user --scope` both report `memory pids` and differ only on
  whether `mkdir` succeeds.

### W2 — limits and the failure model ✅

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

**Result.** `cgroups.limits` is a glob list in the shape `startup.order`
already uses, first match wins. Verified on a live container:

```
INFO cgroups: /mt_container: memory.high=4294967296, pids.max=4096, oom.group=1
```

and read back from the kernel as `memory.high 4294967296`, `pids.max 4096`,
`memory.oom.group 1`.

Limits are written **before** the member is forked into the group. A limit
applied afterwards is a window the process spent without it, and for
`memory.max` that window is exactly when a constructor allocates most.

`memory.events` is reported at shutdown, only for groups whose counters moved,
split by severity — a kill or a refused allocation is the kernel having acted,
a throttle is it holding the line. Observed on a deliberately tight
`memory_max_mb: 40`:

```
WARN cgroups: container mt_container: hit memory.max 139x
```

That run is also the useful negative: 139 `max` events and **zero**
`oom_kill`. The kernel reclaimed rather than killed, because the six
talker/listener composables hold little anonymous memory. It is a clean
demonstration that `memory.max` throttles before it kills — and the reason
`memory.high` is the knob to reach for first.

**One acceptance clause is not closed end to end.** That `oom.group=1` makes a
container die as a unit is verified as a *mechanism*
(`tmp/oom_group_probe.sh`: `bystander=dead` at `1`, `bystander=ALIVE` at `0`),
and the bit is verified reaching the kernel through play_launch. The two have
not been composed, because no fixture composable allocates enough to force an
OOM. Closing it needs a composable that does — `tests/fixtures/governor_stress/
mem_hog.cpp` is the shape, but it is a standalone binary, not a plugin.

### W3 — `cgroup.freeze` for staged startup

Phase 61 W2 holds sensor drivers by waiting for consumers to appear in the ROS
graph. `cgroup.freeze` is strictly stronger: a frozen driver *cannot* publish,
where a spawned-but-unsubscribed one merely shouldn't. Verified working
(`frozen 1`, no progress until thawed).

**The blocking measurement is done, and the risk did not materialise.**

The worry was that a frozen process sends no SPDP announcements, so past its
peers' lease duration they purge the participant — and a driver thawed into a
graph that has forgotten it would be worse off than one that was merely told to
wait. Measured with `tmp/freeze_discovery.sh`: a publisher in its own cgroup,
an unfrozen subscriber as the peer whose view matters, freeze, thaw, and time
the recovery.

| freeze | thaw to first message | messages in the next 10 s |
|---|---|---|
| 2 s | 116 ms | 10/10 |
| 5 s | 752 ms | 10/10 |
| 10 s | 115 ms | 10/10 |
| 20 s | 224 ms | 10/10 |
| 45 s | 229 ms | 10/10 |
| 90 s | 225 ms | 10/10 |

**A 90-second freeze recovers in ~225 ms with full throughput.** No threshold
appeared, and recovery time does not grow with freeze duration — so whatever
rediscovery happens is not on the critical path. Staged startup can hold a
stage for as long as it needs to.

Two things the harness got wrong first, both worth keeping in mind for the next
one:

- **`ros2 node list` measures the wrong thing.** It spawns a FRESH participant
  that must complete discovery from scratch, so a "0" conflates "the peer
  purged it" with "my brand-new observer had not found it yet" — it read 0
  during a 2 s freeze and 1 during a 5 s freeze, which is backwards. The
  subscriber's actual data flow is the ground truth; those columns are noise
  and are not reported above.
- **`grep -c` already prints 0 on no match**, so `|| echo 0` printed a second
  zero and corrupted the first run's table.

**Scope of the result.** Cyclone DDS (the Humble default), single host,
loopback, two participants, and **default QoS** — `KeepLast(10)`, reliable, no
liveliness or deadline contract. Not tested, and the case most likely to
behave differently: a publisher whose QoS carries `LIVELINESS_AUTOMATIC` with a
lease, or a `DEADLINE`, where a freeze longer than the lease should fire a
not-alive or missed-deadline event at the subscriber even though data recovers.
An Autoware stack has such nodes. Measure that before freezing anything on a
vehicle.

W3 is therefore unblocked for the plain case and can be written.

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
