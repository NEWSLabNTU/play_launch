# Phase 60 — Completing the Linux scheduling realizer

**Status:** 📋 planned
**Design of record:**
[`docs/superpowers/specs/2026-08-10-linux-sched-feature-surface-design.md`](../superpowers/specs/2026-08-10-linux-sched-feature-surface-design.md)
**Absorbs:** [Phase 58](./phase-58-scheduling-derivation.md) **W1** (make cost
authorable) and **W4** (reservations). They cannot be separated: a reservation's
`runtime` has no source until cost is authorable, so deriving `SCHED_DEADLINE`
drags W1 in with it. Phase 58 keeps W2 (measure cost), W3 (deadline
decomposition) and W5 (synthesis).
**Predecessor:** [Phase 57](./phase-57-rt-mixed-criticality-demo.md) — supplies
the A/B harness every claim here is checked against.
**Independent of:** [Phase 59](./phase-59-timing-vocabulary.md). New fields here
follow today's `*_us` convention; Phase 59 renames them whenever it lands.

## Why

The `posix` realizer emits exactly one Linux policy. `chain_aware_mapper.rs:498`
and `mapper.rs:224` both hardcode `sched_class: Some("SCHED_FIFO")` for every
ranked node; every unranked node lands in `DEFAULT_TIER`, which `sched_plan.rs`
skips outright. That is the whole surface in use.

The rest of Linux is either unreachable or already dead in the schema:
`SCHED_RR` parses and nothing emits it; `SCHED_DEADLINE` has a *"not applied in
v1"* warning sitting in `sched_plan.rs`; `budget_us` and `deadline_policy` flow
through `ResolvedTier` into `system_model.yaml` and are consumed by nothing.
Two fields that look live and are not — the same defect class the Phase 58 study
named for cost.

This phase makes the full policy surface expressible, appliable and — where a
fact supports it — derived.

## Global constraints

- **Every claim is A/B measured.** Phase 57's rule holds: a harness that cannot
  fail is not evidence.
- **No invented numbers.** Absent and zero stay distinct. Where a cost is
  unknown the model says absent, and a deadline is never substituted for it.
- **`ros-launch-manifest` is a git dependency pinned by tag.** Schema changes
  need a coordinated bump in both `src/play_launch/Cargo.toml` and
  `src/ros-launch-resolve/Cargo.toml`. Never split the revision — one instance
  of `SystemModel`, or the type becomes two.
- **Nothing here claims hard real-time.** The host is `PREEMPT_DYNAMIC`. A
  declared budget is a high-percentile observed cost used *as* an upper bound,
  not a proven WCET.
- **Best-effort derivation is out of scope.** Which nodes become `BATCH`/`IDLE`
  is a partition question, and the partition comes from criticality — which
  [`criticality-from-hazards.md`](../design/criticality-from-hazards.md) is
  replacing. Deriving a demotion from today's meaningless `criticality:` label
  would bake in the defect that design exists to remove. Reachable by override
  here; derived when hazards land.

## Findings this phase is built on

Eleven are recorded in the design; four change what can be built and are
repeated here because every wave touches them.

**F2 — a per-TID sweep multiplies a reservation by the thread count.**
`sched::apply_tier` applies identical attributes to every TID because Linux
scheduling is per-thread. Fine for a priority; for a reservation, one declared
8 ms / 100 ms budget becomes eleven of them — 88 % of a CPU at admission
control. The thread-group leader is reserved; siblings take `SCHED_FIFO` at the
node's derived priority.

**F3 — `SCHED_DEADLINE` outranks every fixed-priority thread**, regardless of
priority (`stop > deadline > rt > fair > idle`). A reservation silently lifts a
node above ones the mapper ranked higher. Hence all-or-nothing per band, scoped
to nodes carrying timing facts, with containers exempt.

**F4 — a `SCHED_DEADLINE` task cannot `fork()`** (`EAGAIN`) unless reset-on-fork
is set, and `CloneIsolatedComponentManager` forks per composable node under the
**default** container mode. `SCHED_FLAG_RESET_ON_FORK` is load-bearing there,
not hygiene, and the two effects compose: the fork succeeds, the child starts
clean, play_launch applies the composable's own tier to it.

**F12 — but the flag must NOT be set on `SCHED_FIFO`/`SCHED_RR`.** Measured
while implementing W1: the kernel resets scheduling in `sched_fork()`, which
runs for **thread** creation too, so the flag stops every thread spawned after
the per-TID sweep from inheriting the policy via `PTHREAD_INHERIT_SCHED` —
leaving an arbitrary subset of a node's ~11 threads at `SCHED_OTHER`, which is
the exact failure the sweep exists to prevent. Setting it uniformly for RT
failed `per_tid_sched_fifo_launch_privileged_only`; removing it fixed the test.
`RESET_ON_FORK` is therefore set only for policies the kernel refuses to fork
from without it. Consequence for W7: a reserved leader carries the flag, so
threads it creates *after* apply land on `SCHED_OTHER` rather than joining the
FIFO siblings — acceptable, since only the leader is reserved by design, but
the sibling-FIFO claim holds only for threads live at apply time.

**F11a — the cpuset partition is unreachable unprivileged. Measured, not
predicted.** On the development host (Ubuntu 22.04, kernel 6.8, systemd 249,
cgroup2, 32 CPUs): `user@UID.service` delegates `memory pids` only,
`user-UID.slice` and `session-N.scope` are root-owned with unwritable
`subtree_control`, `mkdir` under the session scope is `EACCES`, and play_launch
runs in `session-N.scope` — a *sibling* of `user@.service`, so `Delegate=cpuset`
does not reach it. `clone3(CLONE_INTO_CGROUP)` inherits the same common-ancestor
rule. This is delegation containment working as designed.

The same probe pinned two constants this phase depends on:
`sched_rr_timeslice_ms = 100` and `sched_util_clamp_min_rt_default = 1024`.

**F11b — but the partition IS constructible, at the top level only. Also
measured.** Five configurations on the same host settled where it can live:
`SCHED_DEADLINE` works unpinned in a plain container with `--cap-add=SYS_NICE`;
any CPU confinement (`--cpuset-cpus`) gives `EPERM`; a partition nested under
`system.slice` is always `root invalid`, because that slice's blank
`cpuset.cpus` leaves its `cpuset.cpus.exclusive.effective` empty; and a
**top-level** cgroup under the true root reads back `root` and a task inside it
takes `SCHED_DEADLINE` with affinity 31.

So the slice must be top-level — which resurrects F11a's migration problem,
since a top-level slice's common ancestor with `session-N.scope` is the cgroup
root. **Processes must be *started* inside the partition, never migrated into
it**: by a container, a systemd unit, or a privileged launcher. W6 and W8 below
are written to that shape.

---

## W1 — `sched_setattr` in the apply layer

The enabling change, and deliberately behaviour-neutral.

`sched_setscheduler(2)` cannot express `SCHED_DEADLINE`, uclamp, or any
`sched_flags` value. Nothing else in this phase is reachable while it is the
only syscall. `libc` has no wrapper, so this means a local `struct sched_attr`
and a raw `syscall(SYS_sched_setattr, …)`.

`AppliedTier` becomes a lowering of the design's `PosixPlacement`: `CpuSet`
replaces `core: Option<u32>`, uclamp is carried, and `RESET_ON_FORK` gets a
single per-policy home (`reset_on_fork_flag`) that returns nothing today —
see F12 for why setting it for RT would be a regression, not hygiene. A one-time capability probe on self
establishes what the running kernel accepts — `uclamp` needs ≥ 5.3,
`DL_OVERRUN` ≥ 4.16.

**Also in W1: stop reporting every `EINVAL` as a bad priority.** `map_errno`
maps `EINVAL` → `InvalidPriority`, which was defensible when
`sched_setscheduler` was the only caller. `sched_setattr` returns `EINVAL` for
an unsupported flag and `E2BIG` for an unrecognised `sched_attr`, and reporting
"invalid RT priority 34" sends an integrator hunting a priority that is fine.
The error type gains `UnsupportedFeature { feature, errno }`, resolved by the
probe rather than guessed from the errno.

**Done when:** every existing `sched_apply` and `rt_workspace` test passes
unchanged with the new syscall; `chrt -p` on a live node shows the same policy
and priority as before; an unsupported-flag failure reports the feature and the
kernel version it needs, never a priority.

---

## W2 — The typed `posix:` sub-block

`sched_class: Option<String>` is where every new knob would otherwise land as
another loose `Option`, with each consumer re-deriving from a string which
fields are live. `ResolvedTier` already carries thirteen fields mixing a
portable head with `posix` placement.

Introduce `PosixSched` (one variant per policy, carrying only that policy's
parameters) and `PosixPlacement` in `ros-launch-manifest`. `ResolvedTier` gains
`posix: Option<PosixPlacement>`. Illegal states stop being representable: no
`priority` on `Batch`, no `runtime_ns` on `Fifo`, no `cpus` on `Deadline`.

**Additive, not a replacement.** `sched_class`/`core`/`priority` stay
readable-but-deprecated for one release so nano-ros can land its side after the
tag bump rather than in lockstep.

**Also in W2 — F9 and F10, both fail-open defects.**
`SchedPolicy::from_sched_class` maps anything unrecognised to `Other`, so
`SCHED_FIF0` silently drops a node out of real-time. That becomes an error, for
the same reason `criticality-from-hazards` makes an unknown severity an error: a
scheduling declaration must not fail open. And three existing consumers derive
meaning from the string and must move onto the type —
`sched_loader::is_explicitly_non_rt` (which decides whether `class: real_time`
ships, a bug Phase 57 already fixed once), `band_violations` (which Phase 57
also fixed once, and which must now *skip* `Deadline` entirely since it has no
priority), and the legacy `system.toml` bridge.

**Done when:** the `posix:` block round-trips through
`system_model.yaml` golden tests; a typo'd `sched_class` is rejected naming the
six legal values; `band_violations` skips `Deadline` rather than comparing it as
zero; both `Cargo.toml`s name the same new tag.

---

## W3 — The override vocabulary, end to end

The first user-visible capability, and the first that needs no privilege.

`PosixOverride` gains `nice`, `cpus`, `uclamp_min`, `uclamp_max`;
`sched_class` accepts `SCHED_BATCH` and `SCHED_IDLE`. Parse-time validation
lands here, all of it new: `priority` only with FIFO/RR, `nice` only with
OTHER/BATCH, neither on IDLE or DEADLINE; `nice ∈ -20..=19`;
`uclamp ∈ 0..=1024` with `min ≤ max`; `core` and `cpus` mutually exclusive.

**`uclamp_min` on an RT task is a no-op, and `check` says so.** RT tasks default
to `uclamp_min = uclamp_max = 1024` and already request maximum frequency under
`schedutil`; the measured `sched_util_clamp_min_rt_default` on the dev host is
1024. The useful RT knob is `uclamp_max` (lower a node off the maximum
performance point), and `uclamp_min` matters for OTHER/BATCH. `--explain` warns
when `uclamp_min` is set on an RT policy and names
`sched_util_clamp_min_rt_default` as the system-wide alternative.

`DEFAULT_TIER` keeps producing no syscalls. `sched_plan.rs` skips it today, and
emitting `Other { nice: 0 }` would issue a `sched_setattr` per thread for every
best-effort node in the system to set what those threads already have.

**Done when:** a platform file can pin a node to `SCHED_BATCH` with a nice
value and it is observable with `chrt -p`; every validation rejection above has
a test; `DEFAULT_TIER` still issues zero syscalls.

---

## W4 — Make cost authorable (Phase 58 W1)

`TierDef.budget_us` is documented "Execution-time budget (µs) — EDF/sporadic"
and flows into the model, but only the deprecated v1 TOML bridge populates it.
On every v2 path it is `None`, so `sched_derive.rs` passes a path's
`max_latency_ms` in as `exec_ms` — its **deadline**, not its cost.

`overrides.<node>.budget_us` makes it authorable. Cost belongs on the platform
side because execution time is a property of (code, hardware); an earlier draft
put it in the contract with a scale factor, which invents a reference machine
nobody owns.

**Delete the substitution.** With a real source, `sched_derive.rs` reads the
budget or reports absent — never "substitute the deadline". This will change
existing feasibility verdicts, and the diff must show which and why.

**Provenance, so nobody reads it as a proof.** A budget is a declared
high-percentile observed cost used *as* an upper bound. `check --explain` shows
where it came from.

**Done when:** a v2 platform file authors a cost and it reaches
`execution.tiers`; `feasible ON INCOMPLETE EVIDENCE` fires only when cost is
genuinely absent; a test fails if `max_latency_ms` is ever read as a cost again.

---

## W5 — `SCHED_RR` on compressor-created ties

Small, self-contained, and the one derivation that needs no new input.

Band compression already *creates* ties: `assign_priorities_compressed`
collapses adjacent ranks, and `BandTooNarrow` clamps into `band.min`. Two nodes
sharing a priority under `SCHED_FIFO` means one can starve the other; `SCHED_RR`
time-slices instead.

**But the slice is global and enormous.** `/proc/sys/kernel/sched_rr_timeslice_ms`
is 100 on the dev host, not per-task —`TierPlatformSpec.time_slice_us` cannot
express it on Linux. At that value two tied 10 Hz nodes get a slice as long as
their entire period, so `SCHED_RR` degenerates to `SCHED_FIFO` while *looking*
like it solved starvation. Deriving it unconditionally would be a cosmetic
change presented as a real one.

So the rule is conditional: emit `Rr` only when the kernel's actual slice is
shorter than the shortest period among the tied nodes; otherwise stay `Fifo` and
warn that the tie is unmitigated, naming the slice and the periods. `--explain`
prints the value read from the running kernel, never one assumed from the
default, and never phrased as something we set.

**Done when:** a tie at a slice-appropriate period derives `Rr` with provenance;
the same tie on a host with a 100 ms slice derives `Fifo` and warns; neither
path reads a hardcoded default.

---

## W6 — cpuset provisioning

Privileged, no derivation, and shaped entirely by F11a.

`play_launch setup [--caps] [--cpuset]` (with `setcap` kept as a hidden alias)
provisions once, as root: create the slice, enable `cpuset` in its
`subtree_control`, chown it to the invoking user. `execution/cpuset.rs` then
creates `rt-<runid>` at launch, writes `cpuset.cpus`, sets
`cpuset.cpus.partition = root`, and **verifies it reads back `root` and not
`root invalid`** — the kernel accepts the write and reports invalidity through
the same file.

**The slice is top-level, and play_launch is *started* inside it (F11b).** An
earlier draft had play_launch migrate itself in; that cannot work. The partition
must be a child of the true cgroup root — nested under a systemd-managed slice
it is always `root invalid` — and a top-level slice's common ancestor with the
login session is the cgroup root, which no unprivileged writer can use. So there
is no migration step at all: the launcher enters the partition first and every
spawned node inherits it. Once play_launch is inside, moving a node between
`$SLICE/main` and `$SLICE/rt-<runid>` is legal, because the common ancestor is
then the slice itself.

Membership therefore moves the **process** via `cgroup.procs`, not threads: no
threaded-domain complication, and F2 puts the whole node on the isolated CPUs
anyway.

**The RT helper does not grow.** cgroup writes are file permissions, not
capabilities. `CAP_SYS_NICE` stays exactly "may call `sched_setattr`". Widening a
capability-holding binary to `CAP_DAC_OVERRIDE` to save an out-of-band step
would be a bad trade.

**F1 lands here:** once the partition exists, a node with `cpus:`/`core:`
intersecting `isolated_cpus` that is not itself in the partition can no longer
be pinned there. `check --sched` rejects that combination naming both sides,
and `isolated_cpus` stops being advisory — a behaviour change `check` reports
explicitly rather than applying silently.

**The preflight is the deliverable.** Given F11a, most hosts will fail
provisioning, and the difference between a usable feature and an inexplicable
one is whether the verb names the failing precondition — cgroup v2 unified,
`cpuset` in the parent's `subtree_control`, exclusive CPUs reachable by the
partition root — instead of surfacing `EACCES`.

**Done when:** `setup --cpuset` provisions on a host that can support it and
names the exact failing precondition on one that cannot; `verify` reports both
the capability and the partition; teardown restores `member` and removes the
directory, including after a crash; an intersecting pin is rejected before
spawn.

---

## W7 — `SCHED_DEADLINE` derivation and apply

Everything above converges here.

```
reservations == required
  ∧ budget_us present
  ∧ period derivable        → Deadline { runtime  = budget_us,
                                         period   = 1 / min_rate_hz,
                                         deadline = declared, else period,
                                         overrun  = deadline_policy == "fault" }
```

`reservations: off | required` sits **beside `mapper:`**, not inside
`resources:` — `resources:` holds platform facts, `mapper:` holds policy, and
whether to reserve is a policy choice. Default `off`, under which budgets are
parsed and shown by `--explain` but never select a policy. Without the opt-in,
F3's all-or-nothing rule would mean adding a single `budget_us` breaks launch.

**Where the period comes from.** A timer path has one. An input-triggered node
whose publisher declares no rate has none of its own — but a chain has one
triggering rate by construction, so the rate propagates along the chain from its
source. That is a statement of fact rather than an invention. "No rate anywhere
on the chain" stays a hard error under `reservations: required`, since a
reservation without a period is not expressible.

**The multi-threaded executor limit.** F2's reservation covers the thread-group
leader, which on a single-threaded `rclcpp` executor *is* the callback thread.
With a multi-threaded executor it covers one of several, and the guarantee is
unsound. Where we can detect it — `--use_multi_threaded_executor` on a
container — that is a hard error naming the flag. For a plain node that
constructs a `MultiThreadedExecutor` internally we cannot detect it at all, and
that limit is documented and warned about rather than hidden. Requiring the
contract to declare its executor kind was rejected: an executor is a platform
implementation detail and the contract is the platform-*agnostic* file — the
same misplacement `criticality-from-hazards` and Phase 58 W1 both spent effort
undoing.

**Respawn is measured, not designed around.** The actor system respawns nodes,
and a respawned reserved node must re-enter the partition before re-applying.
Whether the kernel releases a dead task's bandwidth synchronously with process
exit decides whether a respawn storm produces spurious `EBUSY`. One test answers
it — reserve, kill, immediately re-reserve — and retry-with-backoff is added
only if the measurement calls for it. Designing for a race that may not exist
would be inventing a number.

**Apply order is fixed** (F6): move the process into the partition *first*,
`sched_setattr` second. Deadline tasks cannot have an affinity mask smaller than
the root domain they were created on, and `sched_setaffinity` cannot narrow it
afterwards — reversing the order yields `EPERM`, which would read as a
permission problem rather than an ordering one. No `sched_setaffinity` call is
made for a `Deadline` node at all.

**F3, scoped.** All-or-nothing applies to nodes carrying timing facts. A
container declares no rate and no deadline — it is a supervisor, not a task — so
it takes `SCHED_FIFO` at the max of its members' priorities and never derives a
reservation. Without this scoping the phase would be unusable: `isolated` is the
default container mode, so nearly every real system would hard-error under
`reservations: required` with no fix available to its author.

**Failure surfacing.** `EBUSY` from admission control is first-class: print the
per-CPU `Σ(runtime/period)` against the ceiling read from the running kernel —
`M × (sched_rt_runtime_us / sched_rt_period_us)`, 95 % on the dev host, shared
with every `SCHED_FIFO` thread in the same root domain including F2's siblings —
and name the node that exceeded it. `--sched-apply strict|warn` governs
apply-time failures; schema and check-time errors are unconditional.

**`deadline_policy` finally does something**, and says so where it cannot:
`fault` → `SCHED_FLAG_DL_OVERRUN` on, since `SIGXCPU` default-terminates, which
is what `fault` declares; `ignore` → off; `skip` and `warn` → an explicit
unsupported-on-`posix` diagnostic. A field that parses and does nothing is the
defect this phase removes; reintroducing it by omission would be worse than
leaving it alone.

**Done when:** a reserved node shows `SCHED_DEADLINE` under `chrt -p` with the
declared parameters; sibling threads show `SCHED_FIFO`; an isolated container
still loads its components (F4); admission rejection names the node and the
arithmetic; a mixed band is rejected before spawn; `deadline_policy: warn`
produces a diagnostic rather than silence; the respawn measurement is recorded
either way.

---

## W8 — Evidence

`rt_av_demo` gains a third arm: `off` / `fifo` / `deadline`.

**The DEADLINE arm runs privileged, in a container, by a measured recipe.**
F11a makes the product path unreachable from an unprivileged session, and a
measurement harness may demand privilege the product does not. F11b's
configuration E *is* the recipe, verified end to end on this host:

```
docker run --rm --privileged --cgroupns=host --cap-add=SYS_NICE ...
  mkdir  /sys/fs/cgroup/play_launch.slice        # top-level: parent = true root
  write  cpuset.cpus = <demo cpu>
  write  cpuset.cpus.exclusive = <demo cpu>
  write  cpuset.cpus.partition = root            # reads back `root`, not `root invalid`
  start  the demo inside it                      # never migrate in
  teardown: partition = member, rmdir
```

`just ab` already refuses rather than reports when the baseline meets its
deadline or the helper lacks `CAP_SYS_NICE`; it gains a third refusal when the
partition does not read back `root`. That check matters more than it looks —
configuration C produced a working-looking `SCHED_DEADLINE` purely *because* its
partition was invalid and the task never left the full root domain. A demo that
silently measures the unpinned case while claiming to measure the partitioned
one is the exact failure Phase 57's logging fix existed to prevent.

The claim under test is whether reservations hold the chain's deadline **while
returning** the 26–37 % best-effort throughput `SCHED_FIFO` cost in Phase 57.
Without `SCHED_FLAG_RECLAIM` — deferred — a reservation idles its unused
bandwidth rather than yielding it, so the recovery may be partial. If throughput
does not recover, the report says so.

**Done when:** `just report` regenerates every figure from the three-arm run
with no hand-typed values; the report states the recovery, partial recovery, or
absence of it.

### Measured, 2026-08-11 — reservations lose this one

| arm | p99 | missed (60 ms) | best-effort vs baseline |
|---|---|---|---|
| RT OFF | 155.0 ms | 217 / 1013 | — |
| `SCHED_FIFO` | 19.9 ms | **9 / 1030** | **−16 %** |
| `SCHED_DEADLINE` | 148.6 ms | 42 / 1038 | **−5 %** |

Reservations return most of the throughput fixed priority costs and give up
most of the determinism. **Fixed priority remains the better trade for this
workload**, and the phase says so rather than tuning until it wins.

The obvious explanation — budgets too small, so CBS throttles — was tested and
**rejected**: a fourth arm at the largest budgets the declared deadlines admit
was *worse* (59 misses, 194.2 ms p99). What remains is a model mismatch the
prior art predicts. `SCHED_DEADLINE` is CBS over a **sporadic release model**;
an `rclcpp::spin()` loop is an event loop whose releases are message arrivals
with no phase relationship to replenishment, so a message arriving after the
runtime is spent waits for the next period rather than for a CPU. That is
exactly the observed shape: p50 improves (31.6 vs 43.0 ms) while p99 barely
moves. Getting the benefit needs per-callback release — phase 58's side-track
G — which the apply layer cannot reach from outside the process.

What the run *does* establish: admission control and the arithmetic are right
(65 % of one CPU admitted against a 95 % ceiling); reservations genuinely bound
the RT class's appetite (the −5 % column); and the derivation is sound end to
end, applied verbatim as `lidar 2000/2000/20000`, `detector 8000/12000/20000`,
`brake 3000/8000/20000` µs.

Full write-up, with caveats:
[`docs/reports/rt-mixed-criticality/reservations-result.md`](../reports/rt-mixed-criticality/reservations-result.md).

---

## Testing posture

Two gates in this phase are more fragile than most: `has_sched_privilege()` and
"is the cpuset partition provisioned". Both must appear in `test-all`'s
**silently-skipped summary** — the 27-of-108 incident was exactly a guard that
started always-skipping while reporting green, and issue #0015 means any rebuild
silently drops the helper's capability and flips the first gate.

One test exists purely to protect F4: `RESET_ON_FORK` must be set for every
fork-capable RT policy. Dropping it breaks isolated containers in a way that
surfaces only as a component that fails to load, which is a long way from the
cause.

## Side tracks — not scheduled

- **`SCHED_FLAG_RECLAIM` (GRUB).** Bears directly on whether best-effort
  throughput recovers, since without it a reservation idles unused bandwidth.
  Revisit once W8 produces a number. Weakens the isolation argument in exchange,
  since a task may run beyond its declared runtime when spare bandwidth exists.
- **Callback-granularity reservations** (Phase 58 side-track G). The real answer
  to F2, and it needs an executor we control. ROSRT and Wilson et al. show what
  it looks like; nothing on vanilla `rclcpp` gets there.
- **`deadline_policy: warn`** needs a `SIGXCPU` handler inside the child. The
  only injection vehicle is the interception `.so`, which is disabled by
  default, so `warn` would silently degrade whenever interception is off.

## Risks

- **The most-tested path is the one nobody ships.** W8 runs privileged and the
  product path does not, so provisioning, migration and teardown are exercised
  by no demo and need their own coverage.
- **Reservations are unavailable to an unprivileged session.** Measured, not
  predicted (F11a). The whole usability of W6/W7 rests on the preflight naming
  the right missing precondition; a vague error there is indistinguishable from
  a broken feature.
- **`isolated_cpus` changes meaning.** Anyone who declared it decoratively gets
  enforcement they did not have.
- **The tag bump is cross-repo.** The additive `posix:` block lets nano-ros land
  its side afterwards, but the tag itself is coordinated work.

## Non-goals

Hard real-time guarantees; static WCET analysis; changing `chain_aware`'s
ranking semantics; ISO 26262 freedom-from-interference claims (an exclusive
cpuset isolates CPU time and nothing else — not cache, not memory bandwidth,
not DDS internals); the best-effort partition derivation; the units migration
(Phase 59).
