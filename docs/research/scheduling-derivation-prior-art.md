# Deriving Scheduling Parameters from Contracts: Prior Art and Options

**Status**: Research
**Date**: 2026-08-06
**Prompted by**: whether chains must be authored, or could be derived from the
causal graph the contracts already describe — and, more broadly, whether a
chain is the right intermediate concept at all.
**Feeds**: `docs/roadmap/phase-58-scheduling-derivation.md`
**Related**: [io-contract-prior-art.md](io-contract-prior-art.md),
[caret-analysis.md](caret-analysis.md),
[manifest-prior-art.md](manifest-prior-art.md)

## Summary

Today one mapper (`chain_aware`) derives fixed priorities from an authored
`chains:` block, ranking drain-toward-sink. This survey asks what *else* a
contract can be turned into, and finds four families that do not need an
authored chain at all — plus one local defect that blocks most of them.

The headline: our contracts declare **budgets**, never **costs**. Most of the
real-time literature needs cost.

## Local findings

Established by reading the current implementation, not from the literature.

### 1. A deadline is being used as a cost; the cost field exists but v2 cannot author it

`sched_derive.rs` passes a path's declared latency budget in as its execution
time:

```rust
exec_ms: resolved.max_latency_ms,
```

`max_latency_ms` is documented as "End-to-end budget the chain's segments must
fit within". Budget (*D*) and execution time (*C*) are distinct quantities:
*D* is what a path is allowed, *C* is what it costs. Using *D* for *C* inflates
a chain's computed sampling cost, so the feasibility verdict errs conservative
— but it means **no true cost appears anywhere in the model**.

This is also why `check` reports `feasible ON INCOMPLETE EVIDENCE` when a
boundary omits the field: absent a declared value the hop is counted as *zero*
cost, and "absent" and "zero" are not the same answer.

**Correction (2026-08-06).** This finding first read "there is no WCET in the
vocabulary", which is wrong, and the error mattered: it led to a design that
put a `exec_ms` in the *contract* with a platform `exec_scale` to convert it —
i.e. a platform-dependent quantity in the platform-agnostic file, reconciled by
a reference machine nobody owns.

`TierDef.budget_us` already exists and is documented as "Execution-time budget
(µs) — EDF/sporadic". It is plumbed through `ResolvedTier` into the
SystemModel's execution layer, whose portable head is `class, deadline_us,
period_us, budget_us`. What is missing is narrower: **only the deprecated v1
`system.toml` bridge populates it.** The v2 schema (`mapper` + `resources` +
`overrides`) has no way to author it, so on every v2 path it is `None` — which
is the vacuum `max_latency_ms` was substituted into.

The correct home for cost is the platform file, beside `rt_priority_band` and
`isolated_cpus`, because cost is a property of (code, hardware). The contract
stays agnostic. See `phase-58` W1.

Consequence: response-time analysis, reservation sizing, and optimal priority
assignment are all unreachable until cost is a first-class fact.

### 2. `ResolvedChain` cannot express parallelism

```rust
pub struct ResolvedChain { pub elements: Vec<ChainElement>, ... }
pub enum ChainElement {
    Segment { nodes_in_topo_order: Vec<SegmentNode> },
    Boundary { node, path, period_ms, exec_ms },
}
```

Both levels are sequences. Linearising a diamond (two paths between the same
pair of nodes) preserves enough to *rank* nodes but destroys the branch
structure needed to *cost* them: feasibility sums where fork-join requires
`max(branch_A, branch_B) + L(join)`, a formula our own
`contract-theory.md` §Parallel (Fork-Join) already states. Multiple paths
between two nodes would therefore be over-estimated as series.

### 3. The DAG ranking rule is specified but never exercised

The mapper design (step 4) says "Fan-in (DAG) inside a segment: rank by
longest-path-to-sink, ties by declared deadline, then name", and `SegmentNode`
documents that the caller must supply the linearisation because only it holds
the launch DAG. But `sched_derive::push_segment_node` merges consecutive path
segments **in declaration order** — an authored chain never presents a fan-in,
so the rule has never had an input that would distinguish it from declaration
order.

### 4. `SCHED_DEADLINE` and CPU pinning are mutually exclusive

`sched_setattr(2)`:

> **EPERM** The CPU affinity mask of the thread specified by pid does not
> include all CPUs in the system.

So a thread confined by `taskset`/`sched_setaffinity` **cannot** be moved to
`SCHED_DEADLINE`. Partitioning for deadline tasks is done with exclusive
cpusets (root domains), not affinity masks. Our current apply path sets
affinity per node and the rt_av_demo harness pins with `taskset`; both collide
with option C below. The kernel exposes `sched_deadline_period_{min,max}_us`,
and admission control shares the `sched_rt_runtime_us` bandwidth.

## Option families

Organised by *what you derive*, since that is the axis that decides what facts
you must have.

### A. A priority order (what we do today)

Current: PiCAS-style chain-aware fixed priority — chain ordering by criticality
then slack, drain-toward-sink within a segment, max-over-chains for shared
nodes.

**Audsley's Optimal Priority Assignment** replaces the heuristic with a search
that is optimal wherever a schedulability test exists: assign the lowest
priority to any task schedulable at it, recurse. Polynomial in the number of
tasks. Its applicability condition holds for fixed-priority preemptive
scheduling — a task's worst-case response time depends on the *set* of
higher-priority tasks, not their relative order.

- Needs: a response-time test, hence WCETs (blocked by local finding 1).
- Caveat from the literature: fragile to small parameter changes or
  underestimated interference, because it makes an arbitrary choice among
  tasks schedulable at a given level.

### B. Per-node deadlines from an end-to-end budget

Deadline decomposition splits an E2E budget across the subtasks that realise
it, then schedules by DM or EDF. Mature literature: Fair/Unfair Laxity
Distribution, proportional-to-cost, convex-optimisation and MILP formulations,
surveys comparing methods.

Why it fits us: it consumes a **budget**, which our contracts already declare,
rather than a cost. It also dissolves the topology problem — once each node
carries a deadline, priority assignment is DM with no drain-toward-sink rule,
no linearisation, and a node on several paths simply takes the tightest
decomposed deadline. A DAG becomes an *input* to decomposition instead of a
structure the ranker must walk in order.

### C. Reservations instead of priorities

Casini et al. (ECRTS 2019) is the canonical ROS 2 result: response-time
analysis of processing chains under reservation-based scheduling. Wilson et al.
go further — callback-per-thread under `SCHED_DEADLINE`, bypassing the executor.

Our contracts already carry both reservation parameters: `min_rate_hz` → period,
a cost → runtime. Linux implements this natively (CBS). The gain over
`SCHED_FIFO` is **temporal isolation**: today an overrunning node is bounded
only by the global `sched_rt_runtime_us` 95% backstop, which is why the
rt_av_demo measurement showed best-effort throughput dropping 26–37% under RT.
A reservation bounds each node individually.

Costs: `AppliedTier` carries `{policy, priority, core}` and would need
`{runtime, deadline, period}`; the apply path uses `sched_setscheduler`, which
cannot express `SCHED_DEADLINE` (needs `sched_setattr`); admission control makes
the kernel reject infeasible sets, which is a feature but a behaviour change;
and local finding 4 means affinity-based partitioning must become cpuset-based.

### D. Remove the dependence on scheduling — LET / time-triggered

Lingua Franca now executes unmodified ROS 2 applications deterministically via
logical time; Abaza et al.'s "latency shaping" uses table-driven reservation
servers for near-zero output jitter; micro-ROS's rclc executor offers LET
semantics (read and buffer all inputs at cycle start).

Instead of deriving priorities that *achieve* a deadline, fix logical release
and output times so timing stops depending on the schedule. Our
`trigger: {timer|input|once|spontaneous}` taxonomy is already the input this
needs. Requires executor cooperation — a nano-ros play, not a vanilla-rclcpp
one.

### E. Whole-configuration synthesis

Teper et al. (RTAS 2024) use constraint programming to jointly choose
node→executor assignment, DDS communication modes, priorities and timer
periods, reporting up to **50.2%** reduction in the analytical end-to-end bound
on an Indy Autonomous Challenge stack. Sciangula et al. do the same for the
communication pipeline with analysis-based heuristics.

This is the fullest form of contract-driven derivation: contract + platform file
as the model, an analysis as the objective, the whole scheduling context as
decision variables. It presupposes the analysis from A/B/C.

The 2026 ROS 2 real-time survey locates the gap precisely:

> parameter derivation remains an active research area, with most systems
> offering either manual configuration options or analysis-guided heuristics
> rather than fully automated synthesis from high-level specifications.

### F. Compositional analysis that never enumerates chains

AADL flow specifications (`flow source` / `path` / `sink`) carry per-component
latency and are composed automatically by tooling; Cheddar performs the
schedulability analysis from the same model. `io-contract-prior-art.md` already
calls this "the gold standard for timing composition". Real-Time Calculus and
SymTA/S compose arrival and service curves per component.

Structurally this is what "derive from local contracts" means — end-to-end
behaviour falls out of per-component facts with no chain object. Worth reading
before designing B, because AADL's flow-path composition is the same problem
with two decades of tooling behind it.

### G. Callback granularity

PiCAS, RTeX (lock-free ready list), chain-instance-level EDF, Teper's
multi-threaded executor analysis, and Wilson et al.'s callback-per-thread +
`SCHED_DEADLINE`.

This is orthogonal but it is the only route past the mapper design's step 6
limitation: "a node's priority = max over its paths' ranks… intra-node
discrimination needs callback-level executor cooperation, impossible on vanilla
rclcpp." Callback-per-thread makes each callback a Linux-schedulable entity and
removes the projection entirely.

## Assessment for this stack

| Option | Blocked on | Value here |
|---|---|---|
| B — deadline decomposition | nothing (uses budgets) | removes chain-topology heuristics; handles DAGs |
| C — reservations | cost; `sched_setattr`; cpusets | temporal isolation we currently lack |
| A — Audsley OPA | cost + a response-time test | optimality, but brittle |
| E — CP synthesis | an analysis to optimise (A/B/C) | largest reported gains |
| D — LET | executor control | determinism; nano-ros track |
| G — callback granularity | executor control | unlocks per-path ranks on Linux |

## References

- Davis et al., [A review of priority assignment in real-time systems](https://www.sciencedirect.com/science/article/abs/pii/S1383762116300200); [Robust Priority Assignment](https://www-users.york.ac.uk/~rd17/papers/Davis-R-RobustPriority.pdf)
- Casini, Blaß, Lütkebohle, Brandenburg, [Response-Time Analysis of ROS 2 Processing Chains Under Reservation-Based Scheduling](https://drops.dagstuhl.de/storage/00lipics/lipics-vol133-ecrts2019/LIPIcs.ECRTS.2019.6/LIPIcs.ECRTS.2019.6.pdf), ECRTS 2019
- [A Survey of Real-Time Support, Analysis, and Advancements in ROS 2](https://arxiv.org/html/2601.10722v1), 2026
- Teper et al., [End-To-End Timing Analysis and Optimization of Multi-Executor ROS 2 Systems](https://lamarr-institute.org/publication/end-to-end-timing-analysis-and-optimization-of-multi-executor-ros-2-systems/), RTAS 2024
- [Timing Analysis and Priority-driven Enhancements of ROS 2 Multi-threaded Executors](https://arxiv.org/pdf/2408.08440); [Bridging the Gap between ROS 2 and Classical Real-Time Scheduling for Periodic Tasks](https://arxiv.org/pdf/2408.03696)
- [Deterministic Execution of ROS 2 Applications via Lingua Franca](https://arxiv.org/pdf/2606.09203); [Managing End-to-End Timing Jitters in ROS2 Computation Chains](https://dl.acm.org/doi/10.1145/3696355.3696363)
- Deadline decomposition: [Convex optimization framework for intermediate deadline assignment](https://www.sciencedirect.com/science/article/abs/pii/S0164121212001100); [Analysis of deadline assignment methods in distributed real-time systems](https://www.sciencedirect.com/science/article/abs/pii/S0140366404001811)
- [Flow Latency Analysis with AADL](https://resources.sei.cmu.edu/library/asset-view.cfm?assetid=8229) (SEI); [Cheddar](https://beru.univ-brest.fr/cheddar/)
- [Theory-Guided Adaptive Scheduling for ROS 2](https://intra.ece.ucr.edu/~hyoseung/pdf/RTNS25_LaME.pdf), RTNS 2025
- `sched_setattr(2)`, `sched(7)`, `cpuset(7)` — Linux man-pages 5.10
