# cgroup-per-container: recovering container properties without a container

**Status:** design study. Every claim below was measured on the bench Orin
(Tegra `5.15.148-tegra`, 12 cores, 61 GiB) — the same class of machine phase 61
measured the startup storm on, so the negative results are on-target rather
than desktop artifacts. Probes live in [`scripts/cgroup-probes/`](../../scripts/cgroup-probes/).

## The question

`--container-mode isolated` fork+execs a process per composable to buy fault
isolation. In doing so it has already given up everything that makes a ROS
container a container: shared address space, shared executor, one DDS
participant. What is left of the container is a **launcher wearing a
container's costume** — a process whose `on_unload_node` is, in full,
deregister a pidfd, kill the child, erase a map entry.

So: which container properties can Linux give back to a set of separate
processes, and which are irreducibly tied to sharing an address space?

## What a ROS container actually bundles

Four unrelated things under one name:

1. **IPC optimization** — `shared_ptr` handoff between publisher and
   subscriber in one address space.
2. **Scheduling consolidation** — one executor, one thread pool, N nodes.
3. **Resource and lifecycle grouping** — the composables are accounted,
   limited and torn down as a unit.
4. **A runtime load/unload API** — `ros2 component list/load/unload`.

`isolated` kept 3 (degraded, via the container process) and 4, and threw away
1 and 2. The interesting question is only about 3: cgroups can do it, and can
do it *better*, because they let the fault model be chosen rather than
implied.

## Measured: what cgroup v2 gives back

Inside a user-owned scope, unprivileged:

| container property | mechanism | result |
|---|---|---|
| memory limit per group | `memory.max` / `memory.high` | writable, reads back |
| **OOM as a unit** | `memory.oom.group=1` | **measured** |
| **per-node OOM (isolation)** | `memory.oom.group=0` | **measured** |
| atomic teardown | `cgroup.kill` | 3 members → 0 |
| thread/pid ceiling | `pids.max` | writable |
| hold a group before start | `cgroup.freeze` | `frozen 1`, no progress until thawed |
| CPU share per group | `cpu.weight` | **unavailable** — `cpu` not delegated |
| stall signal | PSI (`memory.pressure`) | **unavailable** — not compiled in |

### The one bit that decides the failure model

This is the finding the whole design rests on. Same cgroup, same limit, one
bit:

```
oom.group=1  bystander=dead   hog=dead   oom_kill=3     <- container semantics
oom.group=0  bystander=ALIVE  hog=dead   oom_kill=1     <- fault isolation
```

A real ROS container gives you the first row and no choice about it: an OOM
inside a container takes the whole process, every composable in it. Separate
processes give you the second row and no choice about it. `memory.oom.group`
makes it **per group, per launch, decided by whoever knows the system**.

That matters because the second row is not always what you want. A container
whose members form a pipeline can be *worse* half-dead than dead: a supervisor
restarts a dead thing and does not notice a degraded one. Where partial
survival is a silent hazard, `oom.group=1` makes the group fail cleanly and
visibly — and it is still per-process for a *segfault*, which no cgroup
setting changes.

## The design

### Grouping needs no new configuration

The launch file already declares it. A composable names its `target`
container; that is the group. Nothing to author, nothing to maintain — which
is the objection that stalled phase 65's per-node policy lists.

```
<logical container "pointcloud_container">/     memory.max, oom.group, pids.max
    ├── component_node (composable A)           own address space
    ├── component_node (composable B)           own address space
    └── component_node (composable C)           own address space
```

### Placement: fork into the group, never migrate into it

Migration is the trap. Moving an existing process into a sibling cgroup needs
write access to the **common ancestor** of source and destination — the same
rule `execution/cpuset.rs` already documents for the RT partition. Measured
directly:

```
move a running process into a sibling cgroup
  -> write error: Permission denied
```

What works is placing the process at birth. Measured: a grandchild forked
inside a group lands in that group, no migration involved. So the mechanism
is the `pre_exec` hook that **already exists** in
`execution/node_cmdline.rs:924` to write `oom_score_adj`:

```
fork()
  child (single-threaded, pre-exec):
      write own pid to <group>/cgroup.procs     <- async-signal-safe open/write/close
      write oom_score_adj                        <- already there
  exec()
```

play_launch must not move *itself* per spawn: `cgroup.procs` moves every
thread of a process, and play_launch is a tokio runtime with many. The child
placing itself is both correct and race-free.

The kernel offers a better primitive — `clone3(CLONE_INTO_CGROUP)` (5.7+)
places the child atomically at fork, with no window in the wrong group — but
it is not reachable through `std::process::Command`, so it is the ideal rather
than the plan.

### Two structural rules that bite

- **A cgroup that holds processes cannot enable controllers for its
  children.** play_launch must step down into a `supervisor/` leaf of its own
  before it can create sibling groups. Measured; skipping it makes
  `cgroup.subtree_control` fail.
- **Delegation is not what the path looks like.** See below.

## Where it breaks

### Started from a terminal, play_launch cannot do any of this

The single most important negative result. Three contexts, same binary, same
user:

```
plain shell        /user.slice/.../session-1179.scope         mkdir child: EPERM
systemd-run scope  /user@1001.service/app.slice/run-*.scope   mkdir child: ok
  + Delegate=yes   /user@1001.service/app.slice/run-*.scope   mkdir child: ok
```

A login session lands in `session-N.scope` under `user.slice`, which systemd
owns and the user cannot write. Anything started through `systemd-run --user`
lands under `user@1001.service/app.slice`, which the user's own systemd
manages and where `mkdir` succeeds. `Delegate=yes` turns out **not** to be the
discriminator — the slice is.

Both contexts report `controllers: memory pids` identically. So the capability
**cannot be read off the path or the controller list**; it has to be probed by
attempting the `mkdir` and the self-move. This is the same shape as phase 60's
`partition root` writing successfully and reading back `root invalid`: the
write is not the test, the effect is.

Consequence: this is opt-in via a wrapper (`systemd-run --user --scope
play_launch up …`) or it degrades to no grouping with one clear line, the way
`cpuset.rs` reports a missing partition. Self-re-exec is possible and not
recommended — it drags in tty, signal, exit-code and subreaper handling for a
feature that is an enhancement, not a requirement.

### No CPU control without root

`cpu` is absent from the delegated controller set, so `cpu.weight` and
`cpu.max` are unavailable — measured earlier by `systemd-run --user --scope -p
CPUWeight=20` running happily and having no effect. Group CPU share needs a
root-side delegation, the same class as `scripts/provision_rt_cpuset.sh`.

For protecting an operator's desktop during startup, the unprivileged
substitute is `SCHED_BATCH` + nice on children, which needs no cgroups at all.

### PSI is not on this kernel

`/proc/pressure` does not exist on `5.15.148-tegra`; `memory.pressure` and
`cpu.pressure` are absent from every cgroup. A design that leaned on pressure
stall information — a far better startup-governor signal than phase 61's
global `MemAvailable` floor, because it measures actual stalling rather than
free bytes — would work on a developer desktop and silently degrade on the
exact hardware that needs it. Do not build on it without a fallback.

## The trade no Linux feature bridges

Attempting `cgroup.type=threaded` under a subtree with the memory controller
enabled **fails**, and that failure is the architectural fact of this whole
area:

```
cgroup.type=threaded: FAILED
controllers in that cgroup: 'memory pids'
```

`memory` is a domain-only controller. Memory cannot be attributed to a thread,
only to a process. Therefore:

- **per-node memory accounting and limits require per-node processes**
- **zero-copy intra-process IPC requires one shared process**

These pull in opposite directions and nothing reconciles them. Shared memory
transport narrows the gap but does not close it: `rclcpp::LoanedMessage` needs
plain bounded types, and `sensor_msgs/PointCloud2` carries an unbounded
`uint8[] data`, so the heavy Autoware topics are precisely the non-loanable
ones. Cyclone is the Humble default and has no iceoryx installed here.

So the honest split of the four bundled properties:

| property | recoverable across processes? |
|---|---|
| resource + lifecycle grouping | **yes, and better** — the fault model becomes a choice |
| runtime load/unload API | yes, by keeping a container process only where needed |
| IPC zero-copy | **no** — needs a shared address space |
| shared executor | **no** — an application construct, no OS analogue |

## How it is employed on play_launch's containers and nodes

### The tree falls out of the process topology

```
<delegated scope>/
├── supervisor/              play_launch itself
├── node/<dir_name>/         one per plain <node>
└── container/<dir_name>/    the container process AND all its composables
```

The third row is what makes this cheap. play_launch **never spawns a
composable**: `prepare_composable_node_contexts_from_model`
(`execution/context.rs:468`) builds LoadNode *requests*, and the C++ container
fork+execs `component_node` itself. Since a fork child inherits its parent's
cgroup, placing the container process in `container/<name>/` puts every
composable there with **no change to `play_launch_container`**. Measured:

```
write 0: ok
now in: ctr
grandchild in: ctr          <- the composable case
```

Teardown reaches them too — `cgroup.kill` on the container group killed both
the member and its fork child, `members after: 0`. That is exactly the
play_launch → container → composable shape.

### One insertion point

`execution/context.rs:118` is the single spawn chokepoint for nodes *and*
containers. `to_command(long_args, pgid)` takes a `cgroup: Option<CString>`,
and the `pre_exec` hook that already exists at `execution/node_cmdline.rs:829`
gains one line beside `bias_oom_score()`:

```rust
command.pre_exec(move || {
    set_pdeathsig(SIGKILL)?;
    join_cgroup(&cgroup);   // open, write "0\n", close
    bias_oom_score();
    Ok(())
});
```

Writing **`0`** rather than a pid is deliberate and verified: it means "the
calling process", so the hook needs no integer formatting after `fork()` and is
`open`/`write`/`close` — async-signal-safe by construction, like the
`oom_score_adj` write it sits next to.

play_launch must **not** move itself per spawn: a write to `cgroup.procs`
moves every thread of the process, and play_launch is a tokio runtime. The
child places itself.

Lifecycle bounds already exist: build the tree beside `PR_SET_CHILD_SUBREAPER`
(`commands/up.rs:87`), tear it down in the `CleanupGuard` (`commands/up.rs:175`).

### What each group carries, and why the defaults are empty

| knob | node group | container group | default |
|---|---|---|---|
| `memory.current` | read | read | — (accounting) |
| `memory.high` | yes | yes | **unset** |
| `memory.max` | yes | yes | **unset** |
| `memory.oom.group` | n/a (one process) | yes | **0** |
| `pids.max` | yes | yes | **unset** |
| `cgroup.kill` | teardown | teardown | — |

The first value here is **accounting and teardown, not limiting**. A limit
needs per-system knowledge nobody has yet, and a wrong `memory.max` converts a
slow launch into a killed one. `memory.oom.group=0` preserves today's
semantics — fault isolation is the reason to fork at all — and `=1` is opt-in
per container, for pipelines where half-dead is worse than dead.

### What it replaces

- **`monitoring/resource_monitor.rs:237`** — `rss_bytes += child_process.memory()`
  sums a container and its composables, counting every shared page once per
  child. Under `isolated` every child is the *same binary* plus the same
  rclcpp/rmw/DDS libraries. Measured over six processes: **49660 kB summed vs
  20644 kB actual, 2.4x**. `memory.current` on the group replaces the loop with
  one file read and is correct. This number reaches `metrics.csv` and the web
  UI today. (Phase 61's headline figures are unaffected — they came from
  `MemAvailable` deltas, which count shared pages once.)
- **Teardown** — `cgroup.kill` is atomic and catches a composable whose parent
  died first, which a PGID kill can miss.
- **`oom_score_adj +300` stays.** It works with no delegation and is the only
  lever when this whole path is unavailable.

### Degradation is the load-bearing part

play_launch normally starts from a terminal, where this is **unavailable** —
so the unavailable path is the default one, not the exception. Probe at
startup by attempting the `mkdir` and the self-move; on failure log ONE line
naming the wrapper (`systemd-run --user --scope play_launch up …`) and
continue with today's behaviour. This must never fail a launch.

## Relation to existing work

- **Phase 61.** `oom_score_adj +300` on children is a blunt instrument —
  play_launch can only volunteer its own children. A per-container
  `memory.max` bounds the launch's footprint directly, and `memory.high`
  throttles rather than kills. Neither helps the runqueue; the process count
  is still the measured driver.
- **Phase 61 W2 (staged startup).** `cgroup.freeze` is a stronger primitive
  than waiting for a node to appear in the ROS graph: a frozen driver *cannot*
  publish. Untested risk: freezing a process holding DDS sockets may make
  peers time it out of discovery. Measure before adopting.
- **Phase 64** (landed, `e763a2a`). Unaffected — the private load channel and
  cgroup grouping are orthogonal: one carries the load conversation, the other
  bounds and accounts the processes it produces.
- **Phase 65.** Partly reframed. Grouping was one of the reasons to want a
  shared process, and cgroups supply it without one. What remains as a real
  reason to share a process is IPC and executor consolidation — which is a
  question about dataflow, not a hand-written node list.

## What this does not claim

No performance claim. Nothing here reduces process count, thread count or
runqueue depth, and phase 61 measured process count as the driver of the
startup storm. This recovers *semantics* — grouped limits, a chosen failure
model, atomic teardown — for a design that had given them up. Any performance
effect is unmeasured and should be treated as absent until it is not.
