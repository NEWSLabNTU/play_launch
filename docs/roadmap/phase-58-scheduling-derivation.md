# Phase 58 — Deriving scheduling from contracts: cost, deadlines, reservations

**Status:** 📋 planned
**Study of record:** [`docs/research/scheduling-derivation-prior-art.md`](../research/scheduling-derivation-prior-art.md)
**Predecessor:** Phase 57 (mixed-criticality RT demo) — supplies the measurement
harness this phase's claims are checked against.
**Absorbed:** **W1** (make cost authorable) and **W4** (reservations) moved to
[Phase 60](./phase-60-linux-sched-surface.md), which cannot derive
`SCHED_DEADLINE` without them — a reservation's `runtime` has no source until
cost is authorable. W2, W3 and W5 stay here.
**Split out:** the vocabulary/units migration is
[Phase 59](./phase-59-timing-vocabulary.md). It touches adjacent fields but is
independent work; coupling it here would stall W1 behind a cross-repo
negotiation.

## Why

Today exactly one derivation exists: `chain_aware` turns an authored `chains:`
block into fixed priorities. The study found four families that need no
authored chain, and one defect that blocks most of them: **nothing a v2 user
can author declares execution cost**. The field exists (`budget_us`) and is
plumbed to the model, but only the deprecated v1 TOML bridge populates it — so
`sched_derive.rs` substitutes a path's `max_latency_ms`, its *deadline*, and no
true cost appears anywhere in the model.

Ordering follows what each option needs, not how interesting it is. W1 unblocks
the rest; B lands before C because it runs on budgets alone; C carries the
largest architectural collision.

## Global constraints

- **Every claim is A/B measured.** Phase 57's rule holds: a harness that cannot
  fail is not evidence. If a baseline does not miss deadlines, the result is
  inconclusive, not a success.
- **No invented numbers.** Where a cost is unknown the model says so — "absent"
  and "zero" stay distinct, as `chain-sampling-feasibility` already insists.
- **`ros-launch-manifest` is a git dependency pinned by tag.** Schema changes
  need a coordinated tag bump in both `src/play_launch/Cargo.toml` and
  `src/ros-launch-resolve/Cargo.toml`. Never split the revision — one instance
  of `SystemModel` or the type becomes two.
- **Nothing here claims hard real-time.** The host is `PREEMPT_DYNAMIC`. These
  are analyses over declared facts, not proofs.

---

## W1 — Make cost authorable

The enabling change. Nothing else in this phase is reachable without it.
Deliberately the smallest change that unblocks the rest: **additive, no
renames, no field moves.**

### Cost belongs on the platform side

Execution time is a property of (code, hardware). The contract is the
platform-*agnostic* file — rates, deadlines, criticality: things that stay true
when you change machine. An earlier draft put an `exec_ms` in the contract with
a platform `exec_scale` to convert it, which invents a reference machine nobody
owns and no one can verify. Rejected.

Cost belongs in the **platform file**, beside `rt_priority_band` and
`isolated_cpus`. That file is already per-target and already ships through the
provider-sidecar + user-overlay channels, so an integrator writing one for
their machine is the existing workflow, not a new one.

### The field already exists — v2 just cannot author it

`TierDef.budget_us` is documented as "Execution-time budget (µs) —
EDF/sporadic". It flows through `ResolvedTier` into the SystemModel's execution
layer, whose portable head is `class, deadline_us, period_us, budget_us`. But
only the v1 `system.toml` bridge populates it; the v2 schema (`mapper` +
`resources` + `overrides`) has no way to set it, so on every v2 path it is
`None`.

And v2 did not simply forget it. `TierDef` mixes *requirements* (`deadline_us`,
`period_us`, `class` — which duplicate the contract's `max_latency`, `rate_hz`
and `criticality`) with *facts* (`budget_us`, `spin_period_us`,
`time_slice_us`). That mixing is historical: v1 was a standalone `system.toml`
predating the contract/platform split, so it carried both. v2 correctly purged
the requirement-shaped fields — and took one fact with them. **Restoring
`budget` is a restoration, not a new idea.**

### Shape

```yaml
target: posix
mapper: chain_aware
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  obstacle_detector: { budget_us: 8000 }
```

`overrides` rather than a new `costs:` section, on purpose: it is the smallest
additive change, and it is enough for W4 (reservations are per-node — one
thread group, one reservation). It is *not* enough for W3, which wants per-path
cost. See open question 1; a `costs:` section is the likely answer, and
deferring it keeps W1 shippable.

Naming follows today's convention (`budget_us` matches `deadline_us` /
`period_us` on the side it lives on). Phase 59 may rename it to `budget: 8ms`;
W1 does not depend on which lands first.

### Also in W1

Stop passing `max_latency_ms` as `exec_ms` in `sched_derive.rs`. That single
line is the conflation; with a real source for cost it becomes "read the
budget, or report absent" — never "substitute the deadline". Fixing it will
change existing feasibility verdicts, so the diff must show which and why.

**Honesty about what a budget is.** Not a proven WCET — there is no static
analysis here. It is a declared high-percentile observed cost, used *as* an
upper bound. `check --explain` must show provenance (declared / overridden /
measured) so nobody reads it as a proof.

**Done when:** a v2 platform file can author a cost and it reaches
`execution.tiers`; `check --explain` distinguishes budget from cost with
provenance; `rt_av_demo` declares real costs; `feasible ON INCOMPLETE
EVIDENCE` fires only when cost is genuinely absent; a test fails if a deadline
is ever used as a cost again.

---

## W2 — Measure cost instead of asking for it

W1 without W2 asks integrators to invent numbers, which is how the budget/cost
conflation happened in the first place.

The interception layer already timestamps every `rcl_publish` and `rcl_take`
with `CLOCK_MONOTONIC` and correlates them by header stamp — that is exactly a
path's cost for an input-triggered path: *take(input) → publish(output)* for
one message. Phase 57's trace exporter already builds those flows.

**Design of record:**
[`docs/superpowers/specs/2026-08-13-measured-cost-design.md`](../superpowers/specs/2026-08-13-measured-cost-design.md).

**Deliverable:** `play_launch measure <run-dir> --model <model.yaml>` turns a
run into a platform-file fragment on stdout — never written back.

**One correction to the sketch above.** Take→publish elapsed time is *response
time*, not execution time: it carries preemption, blocking and DDS wakeup
latency. `budget_us` becomes a CBS **runtime**, which is CPU time, so feeding
response time into it over-declares every reservation — and "measure on an idle
system" is a mitigation, not a fix, because even idle that figure includes
wakeup latency that is not the node's execution. W2 therefore measures **both**
and keeps them apart: `cpu_ns` (new, from `CLOCK_THREAD_CPUTIME_ID` in the
existing hooks) feeds `budget_us`; the wall-clock figure is reported beside it,
because the gap between them *is* the preemption.

`budget_us` takes the observed **maximum**, with p50/p99 as comments. Under CBS
an overrun is throttled to the next replenishment, so a p99 budget converts the
slowest 1% into a full-period stall — on a safety chain, precisely the
invocations that were already slow.

**Limits, each stated in the output rather than omitted** (omission would read
as "no cost", the absent-versus-zero confusion this phase exists to kill):
timer-triggered paths have no input take to anchor against; messages without
`header.stamp` cannot be correlated (the `std_msgs/String` case from Phase 57);
a declared path never exercised is reported as such; and thread CPU time misses
work a node fans out to another thread.

**Validated against ground truth (DONE):** `rt_av_demo`'s nodes busy-burn for
exactly `burn_ms`, so the fixture is an oracle. `just measure` in that
workspace runs it and checks the output. Measured over a 25 s run:

| path | declared burn | measured cost max | response max |
|---|---|---|---|
| `obstacle_detector/detect` | 8.0 ms | **8.08 ms** (n=1202) | 8.08 ms |
| `brake_controller/brake` | 3.0 ms | **3.05 ms** (n=1201) | **40.43 ms** |
| `lidar_driver/sample` | 2.0 ms | timer-triggered — reported, not measured | — |

The brake row is the argument for measuring both quantities in one line: same
invocation, 3.05 ms of CPU and 40.43 ms of wall clock. A design that measured
elapsed time would have declared a 40 ms reservation for a 3 ms callback.

Hot-path overhead measured rather than assumed: `CLOCK_THREAD_CPUTIME_ID`
costs **85 ns/call against 17 ns** for `CLOCK_MONOTONIC` (not in the vDSO) —
~150 ns per message, 0.15% of a core at 10k msg/s.

---

## W3 — Option B: deadline decomposition

The first new derivation, and the one that answers the DAG question.

Split an end-to-end budget across the nodes realising it, then assign priorities
by deadline-monotonic. Once every node carries a deadline there is no
drain-toward-sink rule, no linearisation, and no need for the ranker to walk
the topology in order.

**Policies** (platform file selects, same channel as `mapper:`):
- *fair laxity* — distribute slack equally. Needs budgets only, so it works
  before W1 lands.
- *proportional to cost* — distribute slack in proportion to the declared
  cost. Needs W1. The better default once cost exists.

**Why this answers the DAG question.** Decompose along each path; a node on
several paths takes the **tightest** resulting deadline. Multiple paths between
two nodes need no special ranking rule — they are just several decompositions
over one node. The `ResolvedChain` sequence-only limitation (study finding 2)
still blocks correct *feasibility* math for fork-join (`max` over branches, not
sum), so either restrict W3 to series budgets or add a branch-carrying variant.
Decide before implementing; do not linearise a diamond and sum it.

**Transport cost lands here, not earlier.** `max_transport_ms` is declared on
`TopicDecl`, documented as "worst-case transport latency for this topic hop",
defaults to 0 and is "absorbed into scope residual" — so it is not a
requirement but a **cost term in the latency budget**, the transport half of
what `budget` is for computation. It is deployment-dependent by the same test:
a pointer pass for composables sharing a container with intra-process comms,
DDS shared memory across processes, the network across hosts — and
`--container-mode isolated` moves a system between the first two *with a CLI
flag*, while `docs/guide/multi-host.md` moves it to the third, both with the
contract unchanged.

By the principle it should move to the platform file. By the evidence it should
not move yet: `max_transport_ms` appears in **exactly one file in this repo, a
test fixture**, and in none of the real Autoware contracts. Moving an unused
field costs a schema change, an alias and a cross-repo bump for no present
benefit. W3 is the first work that consumes transport cost in budget
arithmetic, so W3 is when it moves.

**Done when:** a `deadline_decomposition` mapper produces per-node deadlines
with `--explain` provenance; `rt_av_demo` runs A/B against `chain_aware` on the
Phase 57 harness; the comparison reports both latency and the best-effort
throughput cost.

---

## W4 — Option C: reservations (`SCHED_DEADLINE`)

Biggest payoff, biggest collision. Phase 57 measured the problem it solves:
under `SCHED_FIFO` the safety chain's determinism cost best-effort **26–37%**
of its throughput, bounded only by the global `sched_rt_runtime_us` 95%
backstop. Reservations bound each node individually.

**Spike first — this may not be free.** `sched_setattr(2)` returns **EPERM**
when "the CPU affinity mask of the thread does not include all CPUs in the
system." Our apply path sets per-node affinity and the `rt_av_demo` harness
pins with `taskset`; both are exactly the condition that fails. Deadline tasks
are partitioned with **exclusive cpusets** (root domains), not affinity masks.

The spike answers, on this host, before any implementation:
1. Does `sched_setattr(SCHED_DEADLINE)` succeed on an unpinned thread with
   `CAP_SYS_NICE`?
2. Can an exclusive cpuset give a deadline task a CPU subset, replacing
   `taskset`?
3. What does admission control reject, and is the error legible?

**If the spike passes:**
- `AppliedTier` gains `{runtime_ns, deadline_ns, period_ns}` alongside
  `{policy, priority, core}`.
- The apply path moves from `sched_setscheduler` to `sched_setattr` — the
  former cannot express `SCHED_DEADLINE`.
- Affinity handling forks: FIFO/RR keep `sched_setaffinity`; DEADLINE uses
  cpusets. The platform file's `isolated_cpus` finally means something
  operational rather than declarative.
- Derivation: period from `min_rate_hz`, runtime from the declared cost (W1) —
  which is what `budget_us` has meant since v1 ("EDF/sporadic"), so a
  reservation is its first real consumer; deadline from the decomposition (W3)
  or the declared budget.

### Admission control needs a policy — currently unowned

The spike's third question ("what does admission control reject, and is the
error legible?") has a consequence the rest of this phase does not cover. When
the reservation set does not fit, **something must decide who is refused**, and
no timing fact answers "who matters less".

Criticality answers it — and this is where criticality's job moves to. Once W3
supplies deadlines and W4 supplies reservations, criticality stops being a
priority input: `SCHED_DEADLINE` has no priority number, CBS gives every
reserved task isolation by construction, and EDF orders them by deadline. What
remains is exactly Vestal's original premise — criticality as a **degradation
policy**. See
[`criticality-from-hazards.md`](../design/criticality-from-hazards.md) §"The
division of labour", which records the supersession.

Concretely W4 must decide: on an infeasible set, refuse reservations
lowest-criticality-first and let those nodes fall back to `SCHED_OTHER`, or
refuse the launch outright under `--sched-apply strict`. Both are defensible;
neither is designed. Failing to choose means the kernel chooses arbitrarily,
which for a safety classification is the worst option.

**Done when:** `rt_av_demo` runs a third arm — FIFO vs DEADLINE vs off — and
the report shows whether reservations keep the chain's deadline while returning
best-effort throughput. That is the claim; if throughput does not recover, say
so.

---

## W5 — Option E: synthesis, as a timeboxed spike

Teper et al. report up to **50.2%** reduction in the analytical end-to-end bound
by choosing executor assignment, DDS modes, priorities and timer periods
together with constraint programming. The 2026 survey says automated synthesis
from high-level specs is where the field is thin — which is where our contract
+ platform file sits naturally.

Not scheduled as delivery. It presupposes an analysis to optimise, which W3/W4
supply. Spike scope: can we express our decision variables (priority, core,
reservation parameters) and one objective over `rt_av_demo`, and does the
solver beat the heuristic? A negative answer is a fine outcome and should be
written down.

---

## Side tracks — not scheduled

- **D (LET / time-triggered).** Our `trigger:` taxonomy is already the input
  this needs. Requires executor cooperation, so it is a nano-ros track; on
  vanilla rclcpp there is nothing to apply it to.
- **G (callback granularity).** The only route past the mapper design's step-6
  projection (`node priority = max over its paths' ranks`). Callback-per-thread
  + `SCHED_DEADLINE` makes each callback Linux-schedulable. Per-path ranks are
  already carried in the plan for consumers that can act on them, so the data
  is there when an executor can use it.

Both are worth a try when the executor is ours. Neither blocks W1–W4.

---

## Recorded findings — not scheduled

From auditing the contract/platform split with the same test that moved cost
(*does this value survive a change of machine or deployment?*). Kept here so
they are not rediscovered; none has a consumer that makes it worth a schema
change today.

**`criticality:` is a separate track.** The audit found it to be a label whose
only meaning is its enum variant order, which is a design problem rather than a
misplacement — see
[`docs/design/criticality-from-hazards.md`](../design/criticality-from-hazards.md),
which proposes deriving it from declared hazards and reaches the conclusion
that W4's reservations are the ISO 26262 freedom-from-interference mechanism,
not merely a nicer isolation story.

- **`TopicDecl.drop` is a deployment property.** `max_count` /
  `max_consecutive` describe transport loss, which depends on the link. Note
  the asymmetry the spec already draws: `PathDecl.drop` is end-to-end and
  includes a node *internally* skipping messages, which is code behaviour and
  does belong in the contract. Two fields, one type, different natures.
- **`QosDecl` mixes interface with tuning.** `reliability`, `durability`,
  `history` and `liveliness` decide pub/sub compatibility — genuinely
  contract. `depth` is a buffer-size knob and `lifespan_ms` a retention
  policy — deployment. No action proposed: splitting `qos:` would fight ROS 2's
  own presentation of it as one profile, and the cost of the confusion is low.
- **`jitter_ms` is ambiguous.** Jitter is *caused by* scheduling (a platform
  effect), but "I tolerate ≤5 ms jitter" is a requirement. The name does not
  say which, and nothing reads the field today, so nothing forces the question.
  Decide before something does.

## Open questions

1. Per-node or per-path cost? `overrides` is keyed per node, and the apply
   layer is per-node too (one thread group, one reservation). But chain
   feasibility wants per path — `ChainElement::Boundary.exec_ms` is per
   (node, path), which is exactly why the conflation reached for the
   contract's per-path `max_latency_ms`. Per-node is enough for W4 and not
   for W3. A `costs:` section keyed node→path is the likely answer, and it
   would also give transport cost (W3) somewhere to live that is not keyed by
   node — the platform file is node-keyed today and costs are per-node *and*
   per-topic.
2. Should W3 decomposition run over declared chains only, or over a derived
   causal DAG? The DAG extraction is the larger idea; decomposition works
   either way, and shipping it on chains first keeps the two decisions apart.
3. Does `ResolvedChain` grow a branch-carrying variant, or does a DAG get its
   own resolved type? Affects the manifest crate's public API and hence a tag
   bump.

## Non-goals

Hard real-time guarantees; static WCET analysis; changing the `chain_aware`
mapper's semantics (it stays, as the option that needs no cost); replacing
`chains:` (the study found no case for removing it — causality says what *can*
happen, not which path is accountable, what its budget is, or its semantics);
the units migration (Phase 59).
