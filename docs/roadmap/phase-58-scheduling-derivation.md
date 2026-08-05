# Phase 58 — Deriving scheduling from contracts: cost, deadlines, reservations

**Status:** 📋 planned
**Study of record:** [`docs/research/scheduling-derivation-prior-art.md`](../research/scheduling-derivation-prior-art.md)
**Predecessor:** Phase 57 (mixed-criticality RT demo) — supplies the measurement
harness this phase's claims are checked against.

## Why

Today exactly one derivation exists: `chain_aware` turns an authored `chains:`
block into fixed priorities. The study found four families that need no
authored chain, and one defect that blocks most of them: **our contracts
declare budgets, never costs**. `sched_derive.rs` passes a path's
`max_latency_ms` in as its execution time, so no true cost appears anywhere in
the model.

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

## W1 — Cost as a first-class, platform-agnostic fact

The enabling change. Nothing else in this phase is reachable without it.

**Schema.** Add `exec_ms` to `PathDecl`, distinct from `max_latency_ms`:

```yaml
paths:
  detect:
    trigger: { input: [scan] }
    output: [obstacles]
    max_latency_ms: 12       # budget — what this path is ALLOWED
    exec_ms: 8               # cost — what this path COSTS
```

**Platform-agnostic, via the split that already exists.** Cost is a property of
(code, hardware), but the contract is provider-owned and the platform file is
integrator-owned. So the contract states cost on a *reference* basis and the
platform file converts:

```yaml
# platform file (integrator)
resources:
  exec_scale: 1.4            # this machine is 1.4x slower than the reference
overrides:
  obstacle_detector: { exec_ms: 11 }   # measured here; beats any scaling
```

A single scalar is deliberately crude — it does not model cache, memory
bandwidth, or big.LITTLE asymmetry. It is a stated approximation whose job is
to make the reference basis explicit rather than implied. Per-node overrides are
the escape hatch when the scalar is wrong, and they are the same override
channel scheduling already uses.

**Honesty about what `exec_ms` is.** Not a proven WCET — we have no static
analysis. It is a declared high-percentile observed cost. `check --explain`
must show its provenance (declared / scaled / overridden / measured) so nobody
reads it as a bound.

**Also in W1:** stop passing `max_latency_ms` as `exec_ms` in
`sched_derive.rs`. That single line is the conflation; fixing it will change
existing feasibility verdicts, so the diff must show which and why.

**Done when:** `check --explain` distinguishes budget from cost with
provenance; `rt_av_demo`'s contract declares real costs; the
`feasible ON INCOMPLETE EVIDENCE` path fires only when cost is genuinely
absent; a test fails if budget is ever used as cost again.

---

## W2 — Measure cost instead of asking for it

W1 without W2 asks integrators to invent numbers, which is how the budget/cost
conflation happened in the first place.

The interception layer already timestamps every `rcl_publish` and `rcl_take`
with `CLOCK_MONOTONIC` and correlates them by header stamp — that is exactly a
path's cost for an input-triggered path: *take(input) → publish(output)* for
one message. Phase 57's trace exporter already builds those flows.

**Deliverable:** a verb that turns a run into a contract fragment —
per-path observed cost at p50/p99/max, emitted in `exec_ms` form for the author
to review and paste, never silently written back.

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
- *proportional to cost* — distribute slack in proportion to `exec_ms`. Needs
  W1. The better default once cost exists.

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
- Derivation: period from `min_rate_hz`, runtime from `exec_ms` (W1),
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

1. Does `exec_scale` earn its place, or should the reference basis be recorded
   and left unscaled until someone has two machines to compare?
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
