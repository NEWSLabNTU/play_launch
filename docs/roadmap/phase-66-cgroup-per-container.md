# Phase 66 — cgroup-per-container: the properties, without the container

Status: **W1–W2 done** (`5d8618a`, W2 below). **W3 closed by measurement** — the primitive shipped, the staging gate was not built and should not be. Design study, with every claim measured on the
bench Orin (`5.15.148-tegra`, 12 cores, 61 GiB):
[docs/design/cgroup-per-container.md](../design/cgroup-per-container.md).
Probes are in [`scripts/cgroup-probes/`](../../scripts/cgroup-probes/) —
tracked, because `tmp/` is gitignored and a design whose evidence cannot be
re-run is one asking to be believed.

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
(`scripts/cgroup-probes/oom_group_probe.sh`: `bystander=dead` at `1`, `bystander=ALIVE` at `0`),
and the bit is verified reaching the kernel through play_launch. The two have
not been composed, because no fixture composable allocates enough to force an
OOM. Closing it needs a composable that does — `tests/fixtures/governor_stress/
mem_hog.cpp` is the shape, but it is a standalone binary, not a plugin.

### W3 — `cgroup.freeze` for staged startup — CLOSED, primitive only

Phase 61 W2 holds sensor drivers by waiting for consumers to appear in the ROS
graph. `cgroup.freeze` is strictly stronger: a frozen driver *cannot* publish,
where a spawned-but-unsubscribed one merely shouldn't. Verified working
(`frozen 1`, no progress until thawed).

**The blocking measurement is done, and the risk did not materialise.**

The worry was that a frozen process sends no SPDP announcements, so past its
peers' lease duration they purge the participant — and a driver thawed into a
graph that has forgotten it would be worse off than one that was merely told to
wait. Measured with `scripts/cgroup-probes/freeze_discovery.sh`: a publisher in its own cgroup,
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

**W3.a landed** (`fcf2303`): `set_frozen`/`is_frozen` on `CgroupTree`, reading
state back from `cgroup.events` rather than echoing the request, because
freezing is asynchronous.

**W3.b — the staging gate — is NOT written, because measuring the mechanism
broke this wave's premise.**

The premise was that freeze is "strictly stronger" than phase 61 W2's graph
wait. It is not the right comparison: `StartupGovernor::admit` blocks the
**spawn**, so a late stage is not forked at all until its dependencies are
accounted for. It publishes nothing — exactly as strong as a freeze, and with
no race. Freezing at spawn would buy nothing either, since a frozen process
does not construct, and deferring construction is what holding the fork already
does.

That leaves one argument for a freeze-mode gate — spawn everything so
constructors overlap, freeze each node once accounted-for, thaw in stage order,
paying ~225 ms per release instead of a 33–45 s constructor. Before building
it, the question underneath it was measured: **what does freezing a constructed
node actually return?**

```
settled (running)     130 ticks/10s   mean_runnable 2.1
frozen                  0 ticks/10s   mean_runnable 2.8
thawed                 86 ticks/10s   mean_runnable 3.2
```

Freezing works completely — 0 ticks, the container stops dead. What it stopped
was **0.13 cores across 30 composables**, about 0.4% of a core each, and mean
runnable did not fall at all. A constructed ROS node with little data flowing
is blocked in `rcl_wait`, and a blocked task is already off the runqueue, so
suspending it returns what it was not using. Extrapolated to phase 61's 144
processes that is ~0.6 cores against a measured **10.2-core** storm.

So freeze does not address the startup storm by quiescing what is already up,
and the remaining case for a freeze-mode gate rests entirely on the
release-latency argument — which is real, but is a trade (a small publish
window for less serialised startup), not the strict improvement W3 claimed.

**Decided: the gate is not built.** Phase 61 already spent a wave on a clever
default that measured worse than nothing, and this one would ship a semantic
weakening — a driver may publish between constructing and being frozen — in
exchange for a benefit that the storm measurement says is not there. If the
release-latency case is ever wanted it should be opened on its own terms, as
"staged startup releases in milliseconds instead of a constructor", not as
storm relief.

What survives is the primitive. `set_frozen`/`is_frozen` are a correct,
tested way to quiesce a container, and the two measurements behind this
decision — that a freeze costs a node nothing to recover from, and that a
constructed node costs nothing to suspend — are exactly what a future
"quiesce this container" feature would otherwise have to re-establish.

Scope: talker/listener composables with trivial callbacks in a 31-participant
graph. Autoware nodes running real callbacks at 10–100 Hz in a 144-participant
graph would cost more when idle-but-running; the direction is established, the
magnitude on a real stack is not.

### W4 — CPU, only if the delegation ships ✅ (code side)

`cpu`, `io` and `cpuset` are enabled at the cgroup root and dropped at
`user.slice`; `DelegateControllers` reports `memory pids`. One root-side
drop-in changes it:

```ini
# /etc/systemd/system/user@.service.d/delegate.conf
[Service]
Delegate=cpu cpuset io memory pids
```

**The drop-in is standard practice. Of the three caveats, one is resolved.**
It is how rootless Podman and Docker obtain resource limits, and systemd
documents `Delegate=` as the supported hand-off: the tree at and above the
unit's cgroup is the host manager's, the tree below it is the unit's, and
systemd "will refrain from manipulating control groups or moving processes
below" it. So this is not fighting systemd.

1. **It is not scoped to one user.** `systemd.resource-control(5)`: *"any
   controllers that are delegated will be enabled for the parent and sibling
   units of the unit with delegation."* Enabling `cpu` for `user@.service`
   turns the controller on across `user.slice` and its siblings — every user
   session on the machine, not just the one running a launch.
2. **Real-time tasks may block it, and this project creates them.** The v2 CPU
   controller is reported not to enable while userspace RT tasks exist. This
   bench has them in the user session already — `pipewire` and
   `pipewire-media-session` at `SCHED_RR 20`, `rtkit-daemon` at `RR 99`,
   pulseaudio's ALSA threads at `RR 5` — and play_launch itself applies
   `SCHED_FIFO`/`SCHED_DEADLINE` to nodes (phase 60).

   **Settled, and the answer is that it does not apply here.** The Tegra image
   ships no kernel config, so `CONFIG_RT_GROUP_SCHED` cannot be read directly —
   but it can be inferred from a cgroup that already violates the restriction
   if it were on. `rtkit-daemon` runs a `SCHED_RR 99` thread and lives in
   `/system.slice/rtkit-daemon.service`, which carries `cpu.weight 100` and
   `cpu.max max 100000`, with `cpu` in its parent's `subtree_control`. So on
   this kernel the CPU controller is *already* enabled for a cgroup holding a
   userspace RT task. The RHEL 8 restriction is not in force here, and the
   interaction with phase 60's RT scheduling is not a blocker.

   Worth re-checking on a different kernel rather than carrying forward as a
   general fact: it is a build-time option, and the probe above is the way to
   ask.
3. **Takes effect only after re-login or reboot**, and `cpuset` delegation
   needs systemd 244 or later.

**One argument in favour, specific to the target hardware.** systemd declines to
delegate `cpu` by default partly because the kernel's autogroup feature already
gives per-session scheduling isolation. It is not there:

```
/proc/sys/kernel/sched_autogroup_enabled -> does not exist
```

`CONFIG_SCHED_AUTOGROUP` is not compiled into `5.15.148-tegra`, so on a Jetson
the fallback systemd assumes is **absent**, and the default leaves nothing in
its place.

Sources: [rootlesscontaine.rs cgroup v2](https://rootlesscontaine.rs/getting-started/common/cgroup2/),
[systemd.resource-control(5)](https://www.man7.org/linux/man-pages/man5/systemd.resource-control.5.html),
[systemd CGROUP_DELEGATION](https://systemd.io/CGROUP_DELEGATION/),
[Red Hat: cpu controller in cgroup v2 on RHEL 8](https://access.redhat.com/solutions/6582021).

Same class of provisioning as `scripts/provision_rt_cpuset.sh`. If it ships,
`cpu.weight` (proportional share), `cpu.max.burst` (bounded steady state,
unbounded startup spike) and `cpu.idle` (whole group at `SCHED_IDLE`, and
present on this kernel) become available — and `cpu.idle` would replace the
per-process `SCHED_BATCH` work with one group setting.

**Not started.** An earlier note here recommended the unprivileged
`SCHED_BATCH`/nice path instead, on the grounds that it needs no delegation and
protects an operator's session on any machine. **That recommendation is
withdrawn — measured, it penalises exactly the nodes an AV stack most needs to
be prompt.**

`SCHED_BATCH`'s entire content is that the task does not preempt on wakeup
(`check_preempt_wakeup`: *"Batch and idle tasks do not preempt non-idle tasks
— their preemption is driven by the tick"*). For something that computes, that
is free. For something that sleeps on a device and must act on waking — every
sensor driver — it is added latency, and the name of the policy is the warning.

A 100 Hz driver woken through a pipe, competing with 24 external un-niced
burners on 12 cores (`scripts/cgroup-probes/batch_wakeup.c`):

| arm | p50 | p99 | max |
|---|---|---|---|
| `SCHED_OTHER`, nice 0 | 9.2 us | 13.8 us | 156 us |
| `SCHED_OTHER`, nice +10 | 9.3 us | **4005 us** | 4010 us |
| `SCHED_BATCH` | **3741 us** | 3900 us | 4074 us |

`SCHED_BATCH` moves the **median** wakeup to 3.7 ms — not a tail, every sample.
`nice +10` keeps the median but puts 1% of wakeups 4 ms late, because CFS
scales the wakeup-preemption comparison by weight, so a niced task loses
preemption in practice as well as share.

The CPU-bound half is the mirror image and confirms the mechanism: a fixed
compute workload under 24 competitors took 4652/4587 ms at `SCHED_OTHER` and
4896/4632 ms at `SCHED_BATCH` — no meaningful cost, because a task that never
sleeps never uses wakeup preemption.

So the policy is safe for an NDT-style compute node and unsafe for an I/O-bound
driver, and **play_launch cannot tell which is which**. The model would have to
say, and it does not: a plain launch resolves to zero contracts, so "carries no
timing fact" means "nobody wrote one down", not "timing does not matter" — the
same absent-versus-zero confusion phase 60 removed from the chain checker.
Applying it by default would be phase 61's clever-default mistake with a
different mechanism.

**Landed.** `cpu_weight` and `cpu_max_percent` join the per-container limit
surface, and the controller set is no longer hardcoded.

That last part was the actual work. `cgroup.subtree_control` accepts a write
only if the parent granted every controller named in it — asking for one it did
not enable fails the WHOLE write — so the previous `+memory +pids` constant was
not conservatism, it was the only string that could not break. It now
intersects `USABLE_CONTROLLERS` with what the host actually reports, which is
what lets `cpu` appear on a delegated machine without taking `memory` down with
it everywhere else.

`cpu_weight` is a share (default 100, contended only); `cpu_max_percent` is a
hard ceiling written as `"<quota> <period>"` in microseconds, converted from
percent-of-one-CPU so the config reads in the units a person thinks in. Weight
is the one to reach for: a cap wastes CPU nobody else wants.

**Scope worth stating: this weighs containers against EACH OTHER, inside the
launch.** To weigh the whole launch against a desktop, the weight belongs on
the scope — `systemd-run --user --scope -p CPUWeight=20 play_launch …` — which
needs no play_launch code at all.

A configuration asking for CPU control on a session that was never delegated
the controller is **reported, not dropped**, at the finer granularity the rest
of the wave uses. Measured on this bench, where the memory half still applies:

```
INFO cgroups: per-container grouping active under …/run-….scope [memory pids]
WARN cgroups.limits: cpu_weight/cpu_max_percent are configured but the CPU
     controller was not delegated to this session, so only the memory and pids
     limits apply.
INFO cgroups: /mt_container: memory.high=4294967296
```

**Not verified against a real CPU-enabled cgroup.** `cpu` is not delegated on
this box and granting it needs root, so the write path is covered by unit tests
(the percent→quota conversion, the controller intersection) and by the
end-to-end warning above, but no `cpu.weight` has been read back from a live
group here. That is the one claim in this phase resting on unit tests rather
than a kernel.

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
