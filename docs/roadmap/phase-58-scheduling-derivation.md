# Phase 58 — Deriving scheduling from contracts: cost, deadlines, reservations

**Status:** 📋 planned
**Study of record:** [`docs/research/scheduling-derivation-prior-art.md`](../research/scheduling-derivation-prior-art.md)
**Predecessor:** Phase 57 (mixed-criticality RT demo) — supplies the measurement
harness this phase's claims are checked against.

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
- **`ros-launch-manifest` is a git dependency pinned by tag.** Vocabulary
  changes (W1) need a coordinated tag bump in both
  `src/play_launch/Cargo.toml` and `src/ros-launch-resolve/Cargo.toml`. Never
  split the revision — one instance of `SystemModel` or the type becomes two.
- **Nothing here claims hard real-time.** The host is `PREEMPT_DYNAMIC`. These
  are analyses over declared facts, not proofs.

---

## W1 — Cost as a first-class fact, on the platform side

The enabling change. Nothing else in this phase is reachable without it.

**Cost does not belong in the contract.** Execution time is a property of
(code, hardware). The contract is the platform-*agnostic* file — rates,
deadlines, criticality: things that stay true when you change machine. Putting
a cost there and "converting" it with a scale factor invents a reference
platform nobody owns and no one can verify. Rejected.

Cost belongs in the **platform file**, beside the other machine facts
(`rt_priority_band`, `isolated_cpus`). That file is already per-target and
already ships through the same provider-sidecar + user-overlay channels, so an
integrator writing one for their machine is the existing workflow, not a new
one.

**The field already exists — v2 just cannot author it.** `TierDef.budget_us`
is documented as "Execution-time budget (µs) — EDF/sporadic". It flows through
`ResolvedTier` into the SystemModel's execution layer, whose portable head is
documented as `class, deadline_us, period_us, budget_us`. But it is only ever
*populated* from the v1 legacy `system.toml` bridge; nothing in the v2 schema
(`mapper` + `resources` + `overrides`) sets it, so on every v2 path it is
`None`.

So W1 is not "add a cost field". It is **give v2 a way to author `budget_us`**,
and stop the conflation that filled the gap:

```yaml
# platform file — machine facts, including what work costs on THIS machine
target: posix
mapper: chain_aware
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  obstacle_detector: { budget_us: 8000 }
```

**Naming.** The two files already have consistent, *different* conventions,
and the fix is to follow each rather than to unify them:

| side | unit | bound prefix | examples |
|---|---|---|---|
| contract (requirements) | `_ms` | `max_` | `max_latency_ms`, `max_age_ms`, `max_transport_ms`, `max_response_ms` |
| platform / sched (facts) | `_us` | none | `deadline_us`, `period_us`, `budget_us`, `spin_period_us` |

`budget_us` therefore needs no new name: it matches `deadline_us`/`period_us`
on the side it lives on, and it is already this codebase's word for exactly
this quantity. `max_latency_ms` stays untouched.

The ms/µs split across the two files is deliberate, not an oversight:
contracts are human-authored requirements at millisecond granularity, while
scheduling parameters need sub-millisecond precision (a 20 ms period is 20000
µs, and budgets under 1 ms are ordinary). Recorded here so nobody "fixes" it
later.

**Honesty about what `budget_us` is.** Not a proven WCET — there is no static
analysis here. It is a declared high-percentile observed cost, used *as* an
upper bound. `check --explain` must show its provenance (declared / overridden
/ measured) so nobody reads it as a proof.

**Also in W1:** stop passing `max_latency_ms` as `exec_ms` in
`sched_derive.rs`. That single line is the conflation; with a real source for
cost it becomes "read the budget, or report absent" — never "substitute the
deadline". Fixing it will change existing feasibility verdicts, so the diff
must show which and why.

**Done when:** a v2 platform file can author `budget_us` and it reaches
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

**Deliverable:** a verb that turns a run into a platform-file fragment —
per-path observed cost at p50/p99/max, emitted as a platform-file
`budget_us` fragment for the author to review and paste, never silently
written back. Note the unit change: the trace measures in nanoseconds, the
contract thinks in milliseconds, and the field is microseconds.

**Limits to state in the output:** timer-triggered paths have no input take to
anchor against; messages without `header.stamp` cannot be correlated (the
`std_msgs/String` case from Phase 57); and observed cost under contention
includes preemption, so it over-states cost unless measured on an idle system.
That last one matters — measure on the quiet machine, apply on the busy one.

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
- *proportional to cost* — distribute slack in proportion to `budget_us`.
  Needs W1. The better default once cost exists.

**Why this answers the DAG question.** Decompose along each path; a node on
several paths takes the **tightest** resulting deadline. Multiple paths between
two nodes need no special ranking rule — they are just several decompositions
over one node. The `ResolvedChain` sequence-only limitation (study finding 2)
still blocks correct *feasibility* math for fork-join (`max` over branches, not
sum), so either restrict W3 to series budgets or add a branch-carrying variant.
Decide before implementing; do not linearise a diamond and sum it.

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
- Derivation: period from `min_rate_hz`, runtime from `budget_us` (W1) —
  which is what that field has meant since v1 ("EDF/sporadic"), so a
  reservation is its first real consumer,
  deadline from the decomposition (W3) or the declared budget.

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

## Open questions

1. Per-node or per-path cost? `overrides` is keyed per node, and the apply
   layer is per-node too (one thread group, one reservation). But chain
   feasibility wants per path — `ChainElement::Boundary.exec_ms` is per
   (node, path), which is exactly why the conflation reached for the
   contract's per-path `max_latency_ms`. Per-node is enough for W4 and not
   for W3.
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
happen, not which path is accountable, what its budget is, or its semantics).
