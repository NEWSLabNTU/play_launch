# Completing the Linux scheduling realizer — the full policy surface

Status: Accepted (2026-08-10). Design of record for extending the `posix`
realizer beyond `SCHED_FIFO`.
Roadmap: [phase-60](../../roadmap/phase-60-linux-sched-surface.md) — W1–W8.
Absorbs: [phase-58](../../roadmap/phase-58-scheduling-derivation.md) **W1**
(make cost authorable) and **W4** (reservations), which cannot be separated once
`SCHED_DEADLINE` is derived — a reservation's `runtime` has no source until W1
lands.
Depends on: nothing. *Blocks* on nothing.
Deferred to: [`criticality-from-hazards.md`](../../design/criticality-from-hazards.md)
for the best-effort partition rule.

## Problem

The mapper is a priority calculator that emits one Linux policy.
`chain_aware_mapper.rs:498` and `mapper.rs:224` both hardcode
`sched_class: Some("SCHED_FIFO")` for every ranked node; every unranked node
falls into `DEFAULT_TIER` with `sched_class: None`, which `SchedPolicy::
from_sched_class` maps to `SCHED_OTHER` at nice 0, no affinity, no
differentiation. That is the entire Linux scheduling surface in use.

Everything else Linux offers is either unreachable or dead in the schema:

| feature | mechanism | fact that would drive it | today |
|---|---|---|---|
| `SCHED_RR` | `sched_setscheduler` | equal-priority tie needing a slice | `SchedPolicy::Rr` parses; nothing emits it |
| `SCHED_DEADLINE` | `sched_setattr` | rate → period, cost → runtime | `sched_plan.rs` warns *"SCHED_DEADLINE not applied in v1"* |
| `SCHED_BATCH` / `SCHED_IDLE` | `sched_setattr` | no timing fact at all | absent |
| `nice` | `sched_setattr` | order inside best-effort | absent |
| uclamp | `SCHED_FLAG_UTIL_CLAMP_*` | DVFS floor/ceiling | absent |
| reset-on-fork | `SCHED_FLAG_RESET_ON_FORK` | safety | absent |
| overrun signal | `SCHED_FLAG_DL_OVERRUN` | `deadline_policy` | `deadline_policy` parses; never applied |
| CPU **set** | `sched_setaffinity` mask | partitioning | `core: Option<u32>` — exactly one CPU |
| exclusive cpuset | cgroup v2 `cpuset.cpus.partition` | `resources.isolated_cpus` | documented *"advisory … not enforced"* |
| RT throttling | `sched_rt_runtime_us` | bounds every RT claim | never read, never checked |

Two of those — `budget_us` and `deadline_policy` — already flow through
`ResolvedTier` into `system_model.yaml` and are consumed by nothing. That is the
same defect class the phase-58 study named for cost: a field that looks live and
is not.

## Scope

**In.**

1. `budget_us` authorable in v2 platform `overrides` (phase-58 W1).
2. A typed `PosixSched` / `PosixPlacement` sub-block in `ros-launch-manifest`,
   **additive** beside a deprecated `sched_class`.
3. Apply layer: `sched_setscheduler` → `sched_setattr`; `core: Option<u32>` →
   `CpuSet`.
4. Derived policies: `SCHED_FIFO` (unchanged), `SCHED_RR` on ties,
   `SCHED_DEADLINE` under an opt-in, all-or-nothing rule.
5. Override-only policies: `SCHED_OTHER`, `SCHED_BATCH`, `SCHED_IDLE`; uclamp;
   explicit policy pins.
6. Exclusive cgroup v2 cpuset partition, provisioned by a one-time privileged
   verb, **required** for `SCHED_DEADLINE` — no unpinned fallback.
7. Admission-control (`EBUSY`) and overrun (`SIGXCPU`) surfacing.

**Out.**

- **Best-effort *derivation*.** Which nodes become `BATCH`/`IDLE` is a partition
  question, and the partition comes from criticality, which is being replaced by
  a derivation from hazards. Deriving a demotion from today's meaningless
  `criticality:` label would bake in the exact defect that design exists to
  remove. `BATCH`/`IDLE` are reachable by override this round and derived when
  hazards land.
- Deadline decomposition (phase-58 W3), cost measurement (W2),
  `deadline_policy: warn|skip`, `SCHED_FLAG_RECLAIM`.

**Non-goals.** Hard real-time guarantees (the host is `PREEMPT_DYNAMIC`);
static WCET analysis; changing `chain_aware`'s ranking semantics; ISO 26262
freedom-from-interference claims.

## Findings that shape the design

Each was found while tracing the existing code or checking kernel documentation,
and each changes what can be built.

### Terminology

Two different things are called a "partition" below, so they get distinct names
throughout:

- **RT band** — the set of *nodes* the mapper ranked, placed within
  `rt_priority_band`. A grouping of nodes.
- **cpuset partition** — the cgroup v2 exclusive root partition holding the CPUs
  in `isolated_cpus`. A grouping of CPUs.

### F1 — an exclusive cpuset turns *explicit* pins into hard errors

Setting `cpuset.cpus.partition = root` **removes** those CPUs from the parent
cgroup's effective set. A process that is *not* in the cpuset partition can then
no longer be pinned to those CPUs — `sched_setaffinity` fails, because the CPUs
are not in its effective set.

The check is therefore on **explicit affinity overrides**, not on placement in
general: a node with `cpus:`/`core:` intersecting `resources.isolated_cpus` that
is not itself placed in the cpuset partition is a hard error, naming both sides.
Cgroup membership remains the sanctioned way into those CPUs — which is exactly
how a `SCHED_DEADLINE` node and its F2 sibling threads legitimately end up
running there.

This is also why `isolated_cpus` has been "advisory" until now: nothing ever
made it bite, so nothing ever had to resolve the conflict.

### F2 — a per-TID sweep multiplies a reservation by the thread count

`sched::apply_tier` deliberately walks `/proc/<pid>/task/` and applies identical
attributes to every TID, because Linux scheduling attributes are per-thread and
"a ROS node/composable can go from 1 thread to ~11 threads within half a second
of exec". Correct and cheap for a fixed priority. **Catastrophic for a
reservation**: one declared 8 ms / 100 ms budget becomes eleven 8 % reservations
— 88 % of a CPU presented to admission control, which will either reject it or
grant an order of magnitude more bandwidth than was declared.

**Decision.** The thread-group leader receives the reservation; every other TID
receives `SCHED_FIFO` at the node's derived priority. On a single-threaded
rclcpp executor the leader *is* the callback thread, so the reservation covers
the work. DDS threads stay ahead of best-effort without consuming reservation
bandwidth, and admission control sees exactly the declared utilization.

**Stated limit.** With a multi-threaded executor the reservation covers one of
several executor threads, and the guarantee is unsound. Where we can detect it
(`--use_multi_threaded_executor` on a container) this is a hard error. For a
plain node we cannot detect it at all; the limit is documented and warned about
rather than hidden — see resolved decision 3.

The literature agrees this is the outside-in approximation, not the real answer:
ROSRT and Wilson et al. set `SCHED_DEADLINE` parameters on a **per-callback**
thread created by the executor. That requires executor cooperation we do not
have on vanilla `rclcpp`, and is exactly side-track G ("callback granularity")
in the phase-58 study.

### F3 — `SCHED_DEADLINE` outranks every fixed-priority thread

Linux orders scheduling classes `stop > deadline > rt > fair > idle`. A deadline
task preempts every `SCHED_FIFO` thread regardless of RT priority. The mapper's
entire output is an *order* over the RT band, so granting one node a reservation
silently lifts it above nodes the mapper ranked higher, and the inversion is
invisible in `system_model.yaml`.

**Decision — all-or-nothing per band.** If any node in the RT band derives
`SCHED_DEADLINE`, every node in it must, or it is a hard error naming the nodes
that lack a budget. One ordering discipline per band; no inversions to explain.

**The rule is over nodes, not threads.** F2 deliberately leaves a reserved
node's sibling threads on `SCHED_FIFO`, inside the same band and the same cpuset
partition. That is not a violation: those threads are DDS and executor plumbing,
they are *meant* to sit below every reservation, and the ordering the mapper
computed is an ordering over nodes. Only a node with no reservation while its
peers have one produces the invisible inversion F3 is about.

**And only over nodes carrying timing facts.** A component container declares no
rate and no deadline — it is a supervisor, not a task. Restricting the rule to
nodes the mapper could actually rank keeps it aimed at what it protects: an
inversion is only invisible between nodes that were ranked *against each other*.
Containers, and any node with no timing fact, are exempt by construction.

**Container policy.** A container takes `SCHED_FIFO` at the maximum of its
members' derived priorities, and never derives a reservation. Reserving a
supervisor wastes admission bandwidth on a process that does almost nothing
steady-state, and F4 shows a reserved container could not `fork()` its
components at all were reset-on-fork ever dropped. Demoting it instead is not an
option either: the container's service thread handles `LoadNode` under a 30 s
ready-pipe timeout, and starving it under RT load reproduces a load-failure mode
phase 57 already traced once.

Without this scoping the design would be unusable: `--container-mode isolated`
is the **default**, so under `reservations: required` nearly every real system
would hit a hard error its author could not fix.

**Consequence, and the knob that makes it workable.** A bare "any budget ⇒ all
must have one" rule means adding a single `budget_us` breaks launch, while
phase-58 W1's own premise is that budgets arrive incrementally. Reservations are
therefore **opt-in**:

```yaml
resources:
  reservations: off        # off (default) | required
```

- `off` — budgets are parsed, carried into the model and shown by `--explain`,
  but never select a policy. Adding one budget changes nothing.
- `required` — every node in the RT band must carry a budget, or hard
  error naming those that do not. This is where the all-or-nothing rule bites.

### F4 — a `SCHED_DEADLINE` task cannot `fork()`, and our containers fork

`fork(2)` from a thread scheduled `SCHED_DEADLINE` fails **`EAGAIN`** — the
kernel refuses so a child cannot silently duplicate a reservation and defeat
admission control — *unless the thread has reset-on-fork set*.

`CloneIsolatedComponentManager` is a `fork()`+`exec()` design: it forks the
container process once per composable node, and `--container-mode isolated` is
the **default**. A container holding a reservation without reset-on-fork could
not load a single component.

**This makes `SCHED_FLAG_RESET_ON_FORK` load-bearing, not hygiene.** It is set
unconditionally for every RT policy, and the two effects compose exactly the way
we want: the fork succeeds, the forked `component_node` starts at
`SCHED_OTHER`/nice 0 rather than inheriting the container's reservation, and
play_launch then applies that composable's *own* tier to the child PID. Any
future change that makes reset-on-fork conditional breaks isolated containers,
so it carries a test that fails if the flag is ever dropped for a policy that
can fork.

Separately: a container that only loads and forks is a supervisor, not a
periodic task, and should never derive a reservation of its own. See open
question 4.

### F5 — uclamp for RT tasks is a ceiling, not a boost

RT tasks default to `uclamp_min = uclamp_max = 1024` because they "must always
run at a constant frequency to combat undeterministic DVFS rampup delays". Under
`schedutil` an RT task already requests maximum frequency. So `uclamp_min` on a
`SCHED_FIFO` node is a **no-op** — the useful RT knob is `uclamp_max`, which
lowers a node off the maximum performance point to save power, and `uclamp_min`
is meaningful for `SCHED_OTHER`/`BATCH`.

The design's first draft had `uclamp_min: 512` on an RT node as its worked
example, which would have done nothing. Both knobs are accepted on both sides,
and `check --explain` warns when `uclamp_min` is set on an RT policy, naming
`sched_util_clamp_min_rt_default` as the system-wide alternative.

### F6 — cpuset membership must precede `sched_setattr`

"Deadline tasks cannot have a cpu affinity mask smaller than the root domain
they are created on", and `sched_setaffinity(2)` cannot narrow it afterwards.
The apply sequence is therefore fixed: **move the process into the partition
first, set `SCHED_DEADLINE` second.** Reversing it yields `EPERM`, and the error
would read as a permission problem rather than an ordering one.

### F7 — deadline bandwidth is capped by the RT throttle, per root domain

Admission control admits while `Σ(runtime_i / period_i) < M × (sched_rt_runtime_us
/ sched_rt_period_us)` — 95 % by default, over the **root domain**, shared with
`SCHED_FIFO`/`RR`. An exclusive partition of one CPU therefore offers 0.95 CPU
of total reservable bandwidth, not 1.0, and every FIFO thread in that partition
(including the F2 sibling threads) draws on the same budget. `check --sched
--explain` prints the ceiling it read from the running kernel, the computed sum,
and the headroom — never a figure derived from the default, since an integrator
may have changed it.

### F8 — `map_errno` reports every `EINVAL` as a bad priority

`sched.rs::map_errno` maps `EINVAL` to `SchedApplyError::InvalidPriority`. That
was defensible when the only syscall was `sched_setscheduler`. `sched_setattr`
returns `EINVAL` for an unsupported flag and `E2BIG` for a `sched_attr` the
kernel does not recognise, and a uclamp attempt on a pre-5.3 kernel is exactly
that. Reporting it as "invalid RT priority 34" would send an integrator hunting
a priority that is fine. The error type gains `UnsupportedFeature { feature,
errno }`, resolved by a one-time capability probe on self rather than by
guessing from the errno alone.

### F9 — `sched_class` fails open on a typo

`SchedPolicy::from_sched_class` maps anything unrecognised to `Other`. A platform
file saying `SCHED_FIF0` silently drops a node out of real-time with no
diagnostic. Same failure mode the criticality design calls out for
`parse_criticality`, and it is fixed here for the same reason: a scheduling
declaration must not fail open.

### F10 — three existing consumers derive meaning from the `sched_class` string

Adding `posix:` beside `sched_class` is only additive if every place that reads
the string learns about the new block. Three do, and each breaks differently if
missed:

- **`sched_loader.rs::is_explicitly_non_rt`** decides whether to emit
  `class: real_time` into `system_model.yaml`'s execution layer. Phase 57 fixed
  exactly this — `class: real_time` shipping on best-effort tiers. `Deadline`
  and `Rr` must both count as RT, `Batch`/`Idle` must not, and the decision has
  to move onto `PosixSched` rather than pattern-matching a string.
- **`band_violations`** checks derived priorities against `rt_priority_band`.
  Phase 57 fixed it comparing `SCHED_OTHER` tiers against the RT band and
  "clamping" them into it, producing `SCHED_OTHER 10` — a state Linux cannot
  represent. `Deadline` has **no priority at all**, so it must be skipped
  entirely rather than compared against the band as a zero.
- **The legacy `system.toml` bridge** produces `sched_class` strings and nothing
  else. It maps to `PosixSched` by the same table as any other file, gains no
  new expressiveness (no `budget_us`, so no reservations), and keeps working
  unchanged until nano-ros migrates.

### F11 — cgroup v2 migration rules constrain *where* the slice may live

This one can invalidate the provisioning design if it is got wrong, so it is
recorded as a finding rather than a detail.

Writing a PID into a destination `cgroup.procs` requires write access to the
destination **and to the common ancestor of source and destination**. A slice
created at `/sys/fs/cgroup/play_launch.slice` has the cgroup root as its common
ancestor with the user's login session — and the user cannot write there. The
obvious placement therefore does not work, no matter what is chowned.

The slice must live **inside the user's own delegated subtree**
(`…/user.slice/user-$UID.slice/user@$UID.service/…`), so that the common
ancestor is itself delegated. Two further constraints apply there:

- `cpuset` must be enabled in `cgroup.subtree_control` down the whole chain, and
  **systemd delegates `cpuset` only from v244 and only when configured** —
  `Delegate=cpuset` is not on by default.
- A nested partition root needs exclusive CPUs available from its ancestors.
  `cpuset.cpus.exclusive` (and `cpuset.cpus.exclusive.effective`, which must be
  a subset of the parent's) is what makes that possible, and remote partitions
  were added specifically to allow a partition root below a non-partition
  ancestor. On older kernels the nested case may simply not be constructible.

**Consequence.** The provisioning verb must detect and report which of these
conditions the host fails, with a specific remedy for each, rather than
attempting the writes and reporting `EACCES`/`EINVAL`.

### F11a — measured: the partition is unreachable unprivileged on stock Ubuntu

The spike was run rather than scheduled. On the development host
(Ubuntu 22.04, kernel 6.8.0, systemd 249, cgroup2 unified, 32 CPUs,
`PREEMPT_DYNAMIC`):

| probe | result |
|---|---|
| `user@UID.service` `cgroup.controllers` | `memory pids` — **no cpuset** |
| `user-UID.slice` / `session-N.scope` | `root:root 0755`; `subtree_control` not writable |
| `mkdir` under the session scope | `EACCES` |
| play_launch's own cgroup | `/user.slice/user-UID.slice/session-N.scope` |
| root `cgroup.subtree_control` | *has* `cpuset` |
| `cpuset.cpus.isolated` | empty — no `isolcpus=` at boot |
| `sched_rt_runtime_us` / `sched_rt_period_us` | 950000 / 1000000 → 95 % (F7) |
| `sched_rr_timeslice_ms` | **100** — confirms the RR degeneracy |
| `sched_util_clamp_min_rt_default` | **1024** — confirms F5 |

So a top-level `play_launch.slice` *can* hold cpuset, and nothing in the user
tree can ever migrate into it: the common ancestor is the cgroup root. The
standard remedy — `Delegate=cpuset` on `user@.service` — does not reach us
either, because play_launch runs in `session-N.scope`, a **sibling** of
`user@.service`, not a descendant. `clone3(CLONE_INTO_CGROUP)` is not an escape
hatch: *"all of the usual restrictions … on placing a process into a version 2
cgroup apply"*.

**This is not a gap in the design; it is what delegation containment is for.**
An unprivileged session is *supposed* to be unable to carve exclusive CPUs out
of the machine.

**Decision — split the product path from the evidence path.**

- **Product.** The rule stands unchanged: `reservations: required` with no
  partition is a hard error naming the setup command. Correct, loud, and on a
  stock desktop simply unreachable until an operator provisions the machine —
  which is how production RT deployments are set up anyway.
- **Evidence.** The `rt_av_demo` DEADLINE arm runs **as root**, in a
  root-provisioned slice over the demo's single CPU. A measurement harness may
  demand privilege the product does not. `just ab` already refuses rather than
  reports when the baseline meets its deadline or the helper lacks
  `CAP_SYS_NICE`; it gains a third refusal when not run with the privilege the
  DEADLINE arm needs.

The shipped default therefore stays `SCHED_FIFO` until an operator provisions,
and the A/B claim stays measurable. Neither half pretends to be the other.

## Schema

### Platform file (`<stem>.system.posix.yaml`)

```yaml
target: posix
mapper: chain_aware
reservations: off              # off (default) | required — POLICY, not a fact  [F3]
resources:
  rt_priority_band: { min: 10, max: 40 }
  isolated_cpus: [2, 3]        # semantics CHANGE: advisory -> the exclusive partition
  cpuset_root: /sys/fs/cgroup/play_launch.slice     # optional; discovered  [F11]
overrides:
  obstacle_detector:
    budget_us: 8000            # NEW (W1) — declared cost; the DEADLINE runtime source
    uclamp_max: 1024           # NEW — 0..=1024                    [F5]
  telemetry_logger:
    sched_class: SCHED_BATCH   # policy reachable only by override this round
    nice: 10                   # NEW
    uclamp_min: 256            # NEW — meaningful on OTHER/BATCH   [F5]
  planner:
    cpus: [4, 5]               # NEW — CpuSet; `core:` kept as a single-element alias
```

`reservations` is a **top-level** key beside `mapper:`, not a member of
`resources:`. `resources:` holds platform *facts* — what the machine has;
`mapper:` holds *policy* — what to do with them. Whether to reserve is a policy
choice, and burying it among the facts is the same mixing that phase-58 W1 had
to unpick from `TierDef`.

`PosixResources` gains `cpuset_root`. `PosixOverride` gains
`budget_us`, `nice`, `cpus`, `uclamp_min`, `uclamp_max`, and keeps `priority`,
`core`, `sched_class`.

**`isolated_cpus` changes meaning** from advisory to enforced. That is a
behaviour change for any existing file that declares it, so `check` reports the
new interpretation explicitly rather than letting it take effect silently.

### Parse-time validation — all of it new

- `sched_class` restricted to the six real policies; **unknown is an error**
  (F9).
- `priority` only with `FIFO`/`RR`; `nice` only with `OTHER`/`BATCH`; neither on
  `IDLE` or `DEADLINE`.
- `nice ∈ -20..=19`; `uclamp_* ∈ 0..=1024` with `min ≤ max`; `budget_us > 0`.
- `core` and `cpus` mutually exclusive.
- `reservations: required` with no `isolated_cpus` is an error — a reservation
  needs a partition (F6).

### Typed sub-block (`ros-launch-manifest`)

```rust
pub enum PosixSched {
    Idle,
    Batch    { nice: i32 },
    Other    { nice: i32 },
    Fifo     { priority: i32 },
    Rr       { priority: i32 },
    Deadline { runtime_ns: u64, deadline_ns: u64, period_ns: u64, overrun: bool },
}

pub struct PosixPlacement {
    pub sched:         PosixSched,
    pub cpus:          Option<CpuSet>,   // never Some for Deadline          [F6]
    pub cpuset:        Option<String>,   // only Some for Deadline
    pub uclamp:        Option<(u32, u32)>,
    pub reset_on_fork: bool,             // always true for RT policies      [F4]
}
```

`ResolvedTier` gains `posix: Option<PosixPlacement>`. `sched_class`, `core`,
`priority` stay readable-but-deprecated for one release so the coordinated tag
bump does not break nano-ros mid-flight; both `Cargo.toml`s move together, per
the standing rule that a split revision turns `SystemModel` into two
incompatible types.

Illegal states stop being representable: no `priority` on `Batch`, no
`runtime_ns` on `Fifo`, no `cpus` on `Deadline`.

## Derivation

A policy-selection step is added to `realize_posix` after step 6 (node
projection). The agnostic ranker `chain_aware_rank` is untouched — nano-ros
writes its own realizer over the same `RankedPlan`, and none of this leaks into
it.

```
for each ranked node with derived priority p:

  reservations == required
    ∧ budget_us present
    ∧ period derivable            → Deadline {
                                      runtime  = budget_us,
                                      period   = 1 / min_rate_hz,
                                      deadline = declared, else period (implicit),
                                      overrun  = deadline_policy == "fault" }
  p equals another ranked node's p
    ∧ rr_timeslice < min tied period → Rr   { priority: p }   [see below]
  otherwise                          → Fifo { priority: p }

unranked → DEFAULT_TIER, no PosixPlacement, NO apply at all   (unchanged)
```

**`DEFAULT_TIER` keeps producing no syscalls.** `sched_plan.rs` skips it
outright today (*"the default tier's members have no meaningful scheduling
assignment"*), and this design does not change that. Emitting
`Other { nice: 0 }` for it would newly issue a `sched_setattr` per thread for
every best-effort node in the system, which is a behaviour change with no
benefit — `SCHED_OTHER`/nice 0 is already what those threads have. Best-effort
placement starts being applied only when the criticality-derived partition
arrives and there is something to say about it.

Three rules stated rather than assumed:

- **Absent ≠ zero.** No budget ⇒ no reservation ⇒ `Fifo`, and `--explain` prints
  `budget absent`. A regression test fails if `max_latency_ms` is ever read as a
  cost again — that substitution in `sched_derive.rs` is the conflation phase-58
  W1 exists to delete, and feeding it into a reservation's `runtime` would make
  a declared *deadline* into an enforced *budget*.
- **RR is derived only when the slice is small enough to matter.** The slice is
  global (`/proc/sys/kernel/sched_rr_timeslice_ms`, default **100 ms**), not
  per-task — `TierPlatformSpec.time_slice_us` cannot express it on Linux. At the
  default, two tied 10 Hz nodes get a slice as long as their entire period, so
  `SCHED_RR` degenerates to `SCHED_FIFO` while *looking* like it solved
  starvation. Deriving it unconditionally would be a cosmetic change presented as
  a real one. The rule is therefore conditional: emit `Rr` only when the kernel's
  actual slice is shorter than the shortest period among the tied nodes;
  otherwise stay `Fifo` and warn that the tie is unmitigated, naming the slice
  and the periods. `--explain` prints the value read from the running kernel —
  never a figure assumed from the default, and never phrased as something we set.
- **Provenance for every choice**, following the existing
  `ChainAwareDetail.provenance` convention:

```
brake_controller     derived(chain_aware: safety boundary DM 60ms) -> prio 40 -> SCHED_FIFO
obstacle_detector    ... -> prio 40 -> SCHED_RR (priority tied with brake_controller)
lidar_driver         ... -> SCHED_DEADLINE runtime=8ms(declared) period=100ms(rate_hz=10)
                                          deadline=100ms(implicit) overrun=on(fault)
telemetry_logger     override -> SCHED_BATCH nice=10
```

## Apply layer

**Syscall.** `libc` exposes no `sched_setattr` wrapper; a local `struct
sched_attr` and a raw `syscall(SYS_sched_setattr, …)` are required. One
capability probe on self at startup establishes what the running kernel accepts
(`uclamp` needs ≥ 5.3, `DL_OVERRUN` ≥ 4.16), so an unsupported flag is reported
as such and not as a bad priority (F8).

**Per-node sequence.**

```
non-DEADLINE:  for tid in /proc/<pid>/task:
                   sched_setattr(tid, policy, prio|nice, uclamp, RESET_ON_FORK)
               sched_setaffinity(tid, cpus)

DEADLINE:      1. write pid to <partition>/cgroup.procs         [F6 — first]
               2. sched_setattr(leader, DEADLINE, runtime/deadline/period,
                                RESET_ON_FORK | DL_OVERRUN?)     [F2, F4]
               3. for every other tid: sched_setattr(tid, FIFO, derived prio)
               4. no sched_setaffinity call at all               [F6]
```

**cpuset module** (`execution/cpuset.rs`, new). Membership moves the **process**
via `cgroup.procs`, not individual threads — no threaded-domain complication,
and F2 already places the whole node on the isolated CPUs. Creation writes
`cpuset.cpus`, sets `cpuset.cpus.partition = root`, and **verifies it reads back
`root` and not `root invalid`** — the kernel accepts the write and reports
invalidity through the same file. Teardown restores `member` and removes the
directory; the provisioning verb also cleans up leftovers from a crash.

**Provisioning** follows the existing `setcap`/`verify` precedent — one-time,
out of band, checked at launch:

```
once, as root — see F11a for why an unprivileged session cannot do this:
    play_launch setup --cpuset          # `setcap` becomes a hidden alias  [#7]
      SLICE=/sys/fs/cgroup/play_launch.slice
      preflight: cgroup v2 unified? cpuset in the parent's subtree_control?
                 exclusive CPUs reachable by the partition root?
      mkdir  $SLICE
      enable cpuset in $SLICE/cgroup.subtree_control
      chown  $USER  $SLICE  and its cgroup.procs / cgroup.subtree_control
                            / cpuset.cpus / cpuset.cpus.partition

at launch, unprivileged:
      move play_launch itself into $SLICE/main         <- required
      mkdir $SLICE/rt-<runid>; write cpus; partition = root
      verify cpuset.cpus.partition reads back `root`, not `root invalid`
      move DEADLINE nodes in; teardown on shutdown
```

play_launch must place **itself** under the slice, or moving a spawned child into
a sibling cgroup violates cgroup v2's common-ancestor rule: once both source and
destination sit inside the slice, the common ancestor is the slice itself and the
user has write on it. Children inherit `$SLICE/main` and DEADLINE nodes migrate
to `$SLICE/rt-<runid>`.

The move of play_launch *into* `$SLICE/main` is the one step that cannot be
unprivileged (F11a), which is why the launch path for `reservations: required`
either runs privileged or is started already inside the slice. The verb
preflights each precondition and names the failing one rather than attempting
writes and surfacing `EACCES`.

**The RT helper does not grow.** cgroup writes are ordinary file permissions,
not capabilities; after provisioning the user already holds them. `CAP_SYS_NICE`
stays exactly "may call `sched_setattr`", and the wire protocol gains only the
new placement payload. Widening a capability-holding binary to `CAP_DAC_OVERRIDE`
to save an out-of-band setup step would be a bad trade.

Note issue #0015 applies unchanged and now has a second victim: any rebuild
replaces `play_launch_rt_helper` and drops its capability. `just setcap` after
every build; `verify` reports both the capability and the partition.

## Errors

Hard errors, unconditional — schema and check-time problems are spec bugs, not
runtime conditions, so `--sched-apply warn` does not downgrade them. Each names
both sides.

| condition | message content |
|---|---|
| `cpus:`/`core:` intersects `isolated_cpus` (F1) | the node, its CPUs, the partition's CPUs |
| DEADLINE derived, partition missing | the exact `play_launch setcap --cpuset` invocation |
| DEADLINE on a multi-threaded-executor container (F2) | the node and the `--use_multi_threaded_executor` flag |
| `reservations: required`, node without a budget (F3) | every node lacking one |
| unknown `sched_class` (F9) | the value and the six legal ones |
| `reservations: required` with no `isolated_cpus` | both fields |

Apply-time failures honour `--sched-apply off\|warn\|strict`:

| condition | strict | warn |
|---|---|---|
| `EBUSY` from admission control | abort; print per-CPU `Σ(runtime/period)` against the kernel's ceiling (F7) and name the node that exceeded it | same figures; leave the task at its prior policy |
| `EPERM` | abort | warn, continue |
| unsupported flag (F8) | abort naming the feature and kernel requirement | drop the flag, warn, continue |

Overrun maps onto the existing, currently-dead `deadline_policy`:

| value | Linux |
|---|---|
| `fault` | `SCHED_FLAG_DL_OVERRUN` on — `SIGXCPU` default-terminates, which is what `fault` declares |
| `ignore` | flag off |
| `skip` | flag off + diagnostic: needs executor cooperation, not expressible on `posix` |
| `warn` | diagnostic: needs a `SIGXCPU` handler inside the child; not implemented this round |

`warn` and `skip` produce an explicit unsupported-on-this-target diagnostic
rather than a silent no-op — a field that parses and does nothing is the defect
this design is removing, so it must not be reintroduced by omission.

## Testing and evidence

**Manifest crate.** Parse/validate units for every new field and every rejection
above; `posix:` golden roundtrip; derivation units covering FIFO / RR-on-tie /
DEADLINE selection and their provenance strings; the `max_latency_ms`-as-cost
regression test.

**play_launch.** `sched.rs` syscall tests reuse the existing `with_sleep_child`
pattern, gated on `has_sched_privilege()`. `cpuset.rs` tests gate on a
provisioned partition. Both gates go into `test-all`'s **silently-skipped
summary** — the 27-of-108 incident is precisely a guard that started always
skipping while reporting green, and these two guards are more fragile than most
because a rebuild silently drops the capability.

A test asserts `RESET_ON_FORK` is set for every fork-capable RT policy (F4),
since dropping it breaks isolated containers in a way that surfaces only as a
component that fails to load.

**Evidence.** `rt_av_demo` gains a third arm: `off` / `fifo` / `deadline`.
`just ab` already refuses rather than reports when the baseline meets its
deadline or the helper lacks `CAP_SYS_NICE`; it gains a third refusal when the
partition is absent. The claim under test is whether reservations hold the
chain's deadline **while returning** the 26–37 % best-effort throughput that
`SCHED_FIFO` cost in phase 57. If throughput does not recover, the report says
so — phase 57's rule stands that a harness which cannot fail is not evidence.

## Related work

**Reservations for ROS 2 callbacks.** Blaß et al. give a response-time analysis
for ROS 2 processing chains under reservation-based scheduling, which is the
analysis our `budget`/`period` pair would feed. ROSRT and Wilson et al. go
further and have the *executor* create one thread per callback and set
`SCHED_DEADLINE` parameters on it — the granularity F2 cannot reach from
outside the process, and side-track G in the phase-58 study. Wilson's design is
noted to inherit `SCHED_DEADLINE`'s sporadic release model, which is the same
assumption our implicit-deadline fallback makes.

**Fixed priority in practice.** `ros2_control`'s Controller Manager asks for
`SCHED_FIFO` priority 50, and `ros2-realtime-examples` exposes `--sched
SCHED_FIFO --priority 80` per process. Both are per-node manual choices; the
gap this design addresses is that nothing derives the number from a system-level
declaration, which is what the mapper does.

**`SCHED_DEADLINE` limitations.** Tang & Anderson document tardiness and
affinity defects in `SCHED_DEADLINE` and offer a partial fix — directly relevant
because our partition design constrains affinity through cpusets, which is the
mechanism their analysis concerns. Worth reading before claiming any bound from
a reservation.

**Priority/EDF assignment over ROS graphs.** Recent work assigns fixed-priority
and EDF schedules to ROS 2 graphs on a uniprocessor, and ReDAG-RT does global
rate-priority scheduling for multi-DAG execution. Both are the derivation half
(phase-58's option families), not the mechanism half this document covers.

**Partitioning above the OS.** Jiao reaches freedom-from-interference with a
static partitioning hypervisor (Jailhouse) — dedicated cores, memory and
devices — cutting jitter σ from 12.58 µs to 1.95 µs. We operate one tier down at
OS scheduling; an exclusive cpuset isolates CPU time and nothing else. The
honest claim stays "temporal budget, not freedom from interference".

**uclamp tooling.** `uclampset(1)` is the per-task CLI equivalent of what we set
via `sched_setattr`; systemd has an open request for `cpu.uclamp.*` support at
the unit level. Neither derives values from a system description.

### Sources

- [Deadline Task Scheduling — Linux kernel documentation](https://docs.kernel.org/scheduler/sched-deadline.html)
- [Utilization Clamping — Linux kernel documentation](https://docs.kernel.org/scheduler/sched-util-clamp.html)
- [sched(7) — Linux manual page](https://man7.org/linux/man-pages/man7/sched.7.html)
- [uclampset(1) — Linux manual page](https://www.man7.org/linux/man-pages/man1/uclampset.1.html)
- [cgroup/cpuset: support remote partitions (LWN)](https://lwn.net/Articles/943607/)
- [Add utilization clamping support (LWN)](https://lwn.net/Articles/751361/)
- [ROSRT: Enabling Flexible Scheduling in ROS 2 (RTSS'25)](http://www.cs.unc.edu/~anderson/papers/rtss25b.pdf)
- [Response-Time Analysis of ROS 2 Processing Chains Under Reservation-Based Scheduling](https://www.researchgate.net/publication/334459173_Response-Time_Analysis_of_ROS_2_Processing_Chains_Under_Reservation-Based_Scheduling)
- [On the Defectiveness of SCHED_DEADLINE w.r.t. Tardiness and Affinities, and a Partial Fix](https://dl.acm.org/doi/fullHtml/10.1145/3453417.3453440)
- [A Survey of Real-Time Support, Analysis, and Advancements in ROS 2](https://arxiv.org/html/2601.10722v1)
- [Fixed-Priority and EDF Schedules for ROS2 Graphs on Uniprocessor](https://arxiv.org/pdf/2512.16926)
- [ReDAG-RT: Global Rate-Priority Scheduling for Real-Time Multi-DAG Execution in ROS 2](https://arxiv.org/pdf/2603.18238)
- [ros2_control Controller Manager — real-time configuration](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html)
- [ros2-realtime-examples — minimal_scheduling](https://github.com/ros-realtime/ros2-realtime-examples/blob/rolling/minimal_scheduling/README.md)
- [Jiao: mixed-criticality robotics isolation](https://arxiv.org/html/2605.03641)

## Resolved decisions

The eight questions this design opened are settled. Recorded with their
reasoning, because a decision without one gets re-litigated.

1. **Cost is per-node this round.** Inherited from phase-58 open question 1.
   `overrides` is node-keyed and the apply layer is node-keyed — one thread
   group, one reservation — so per-node is exactly enough for reservations. Per
   (node, path) matters only for the chain feasibility math in W3, which is out
   of scope. The `costs:` section stays deferred.

2. **A rate propagates along the chain from its source.** A node triggered by an
   input whose publisher declares no rate has no period of its own and could not
   take a reservation. But a chain has one triggering rate by construction, so
   inheriting the source's rate is a statement of fact rather than an invention.
   "No rate anywhere on the chain" remains a hard error under
   `reservations: required`.

3. **Undetectable multi-threaded executors are accepted with a warning.** F2's
   refusal is enforceable for containers, where `--use_multi_threaded_executor`
   is on the command line, and not for a plain node that constructs a
   `MultiThreadedExecutor` internally. Requiring the contract to declare its
   executor kind was rejected: an executor is a platform implementation detail,
   and the contract is the platform-*agnostic* file. Putting it there is the
   same misplacement that `criticality-from-hazards` and phase-58 W1 both spent
   effort undoing. The limit is documented and warned about, not hidden.

4. **The all-or-nothing rule is scoped to nodes carrying timing facts**, and
   containers take `SCHED_FIFO` at the max of their members' priorities. See F3.

5. **Respawn admission accounting is measured, not designed.** One test:
   reserve, kill, immediately re-reserve, and see whether `EBUSY` appears —
   i.e. whether the kernel releases a dead task's bandwidth synchronously with
   process exit. Retry-with-backoff only if the measurement says it is needed.
   Designing for a race that may not exist would be inventing a number, which
   this phase's global constraint forbids.

6. **Product and evidence paths split.** See F11a — the product keeps the
   no-fallback rule; the `rt_av_demo` DEADLINE arm runs privileged.

7. **The verb is `play_launch setup [--caps] [--cpuset]`**, with `setcap` kept
   as a hidden alias. `setcap` names capabilities and now covers a cgroup too.

8. **`reservations:` sits beside `mapper:`, not inside `resources:`.** Facts and
   policy stay separated. See the schema section.

### Still genuinely open

Nothing blocks implementation. Two items are follow-on work, recorded so they
are not rediscovered:

- **`SCHED_FLAG_RECLAIM` (GRUB)** is deferred. It bears directly on whether
  best-effort throughput recovers, since without it a reservation idles its
  unused bandwidth rather than yielding it. Revisit once the A/B has a number.
- **Callback-granularity reservations** (phase-58 side-track G) are the real
  answer to F2 and need an executor we control. ROSRT and Wilson et al. show
  what it looks like; nothing on vanilla `rclcpp` gets there.

## Risks

- **Behaviour change on `isolated_cpus`.** Any existing platform file declaring
  it gets enforcement it did not have. Surfaced by `check`, but it is a real
  break for anyone who declared it decoratively.
- **The tag bump is cross-repo.** `ros-launch-manifest` is pinned by tag in two
  manifests that must move together. The additive `posix:` block is designed to
  let nano-ros land its side afterwards rather than in lockstep, but the tag
  itself is still coordinated work.
- **A reservation is not a proof.** `budget_us` is a declared high-percentile
  observed cost used *as* an upper bound, on a `PREEMPT_DYNAMIC` kernel, with a
  reservation covering one thread of a multi-threaded process. `check --explain`
  shows provenance so nobody reads it as WCET, and no output of this work claims
  a hard real-time guarantee.
- **Reservations are unavailable to an unprivileged session, measured — not
  predicted.** F11a is not a risk to watch for; it is the state of the
  development host and of any stock Ubuntu desktop. Nothing in the user's cgroup
  tree is writable and `cpuset` is not delegated, so `reservations: required`
  ends in a hard error until an operator provisions the machine. That is the
  intended behaviour, and it means the entire usability of the feature rests on
  the preflight diagnostic naming the right missing precondition. A vague error
  here is indistinguishable from a broken feature.
- **The product's most-tested path will be the one nobody ships.** The A/B arm
  runs privileged while the product path does not, so the code exercised by the
  evidence harness is not the code an integrator runs. Whatever differs between
  them — provisioning, migration, teardown — is untested by the demo and needs
  its own coverage.
- **Reservations may not return the throughput they cost.** The phase-57 measured
  26–37 % best-effort loss is the number W4 exists to recover. Without
  `SCHED_FLAG_RECLAIM` (deferred), a reservation idles its unused bandwidth
  rather than yielding it, so the recovery may be partial. The A/B says which.
