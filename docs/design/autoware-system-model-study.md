# Autoware System Model Study → Chain-Aware Scheduling — Design Draft

**Status:** Draft (discussion)
**Date:** 2026-07-16
**Phase:** [phase-42](../roadmap/phase-42-autoware_system_model.md)
**Not to be confused with** [system-model.md](./system-model.md) — the
*SystemModel resolved artifact* (structure/contracts/execution schema shared
with nano-ros). This document studies the *behavioral* model of a real
system. Touchpoints: contract vocabulary extensions proposed here (Q3
cross-scope chains) extend SystemModel layer 2; the chain-aware mapper's
output populates SystemModel layer 3 (`execution:`).

**Prereads:** [data-quality-semantics.md](../research/data-quality-semantics.md)
(cause-effect chains, TIMEX, Lingua Franca, Timely Dataflow),
`src/ros-launch-manifest/docs/launch-manifest.md` (paths, `state:` semantics),
`.superpowers/sdd/autoware-v2-report.md` (Phase 41 exercise findings).

## Why study first

The Phase 41 Autoware exercise showed the mechanics work but the mapper is
starved of structure: 11/13 RT nodes declare the same 10 Hz, criticality is
inert, and per-node deadlines are a lossy projection of what the contracts
actually express — *paths*, i.e. cause-effect chains. PiCAS (Choi et al.,
RTAS 2021) demonstrates that chain-aware priority assignment beats
node-local RM for exactly this workload class. But PiCAS assumes clean
chains; the real Autoware graph is messier:

- **Cyclic topic paths** — e.g. planning consumes localization which consumes
  vehicle state influenced by control output; feedback loops are the norm.
  The manifest's `state: true` (polled, non-causal) is our existing
  cycle-breaking primitive — the study must check whether it is *sufficient*
  and *honestly placed* in real contracts, or merely where authors happened
  to need the causal-dag rule to pass.
- **Non-1-to-1 input→output mapping** — message_filters sync (N inputs → 1
  output), latch/parameter-style inputs, batching, timers that publish
  regardless of input arrival (time-triggered vs event-triggered nodes),
  outputs fanned to many consumers at different rates (over/undersampling
  junctions in the AUTOSAR TIMEX sense: data-age vs reaction-time semantics
  diverge exactly at these junctions).
- **Information flow ≠ topic graph** — what matters for scheduling is which
  *data lineage* must stay fresh end-to-end (sensor → actuation), not the
  raw pub/sub adjacency.

Cutting a mapper before understanding these shapes risks encoding the wrong
model into the shared sched crate. Study first, then design.

## Study method

Three lenses over the same system (Autoware 1.5.0 planning_simulator, the
environment already used by Phase 40/41 validation):

1. **Declared model** — parse the 75 contracts: enumerate all `paths:`,
   their `state:`/causal subscriptions, budgets, rates. Build the declared
   causal graph (we already have `check`'s causal-dag machinery; extend to
   *export* the graph rather than only validate it).
2. **Measured model** — Phase 29 interception on a live planning_simulator
   run: per-topic rates, frontier timestamps, actual pub/take counts. This
   yields the *effective* flow: which declared paths are hot, real rates vs
   declared rates, where fan-in junctions sample and drop.
3. **Source model (sampled)** — for a representative node per pattern
   (timer-driven publisher, sync-driven fusion, polled-state consumer, a
   feedback-loop member): read source in `~/repos/autoware/1.5.0-ws/` and
   classify its trigger semantics (event-triggered / time-triggered / hybrid)
   — the distinction Casini et al.'s executor analysis and Lingua Franca both
   treat as fundamental, and which contracts currently cannot express.

Cross-checks the study must answer:

- **Q1 Cycles:** where do cycles exist in the declared graph; are all broken
  by `state:`; do measured timestamps confirm the polled semantics (no
  causal coupling)?
- **Q2 Junctions:** catalogue of non-1:1 patterns (sync policies, sampling
  rates at fan-in) and which chain semantics (data age vs reaction latency,
  per data-quality-semantics.md §2) each junction needs.
- **Q3 Chains:** can end-to-end chains (sensor→actuation) be *composed* from
  the per-scope `paths:` we have, or do we need first-class cross-scope
  chain declarations in the manifest?
- **Q4 Triggers:** how many nodes are time-triggered vs event-triggered;
  does the mapper need this fact (a time-triggered node's rate is real; an
  event-triggered node's rate is inherited from upstream)?
- **Q5 Rates:** is the 10 Hz monoculture a declaration artifact (authors
  copying the planning rate) or the measured truth?

## Expected outputs

1. **Autoware system-model report** (`docs/research/autoware-system-model.md`):
   the graph, cycle catalogue, junction catalogue, trigger census, declared-vs-
   measured deltas. The empirical grounding for everything after.
2. **Contract vocabulary gaps** — proposed manifest extensions, each justified
   by a study finding. Candidates (to be validated, not assumed): trigger
   semantics on nodes (`trigger: timer|input|either`), cross-scope chain
   declarations with semantics tag (`age` vs `reaction`), junction sampling
   annotations. Additions must stay platform-agnostic and optional.
3. **Mapper design spec** (separate, after 1+2): the chain-aware mapper —
   unit of assignment is the chain (PiCAS-style priority ordering of chains
   by criticality/budget, node priority = max over member chains, cycles
   handled via the validated `state:` cuts). The interim `criticality_rm`
   bucket mapper (41.7) is deliberately gated on the study: if chains prove
   composable, node-bucket criticality may be subsumed by chain criticality.

## Non-goals

- No mapper implementation in this phase (design only, informed by data).
- No nano-ros work (separate track; the sched crate stays additive-stable).
- No Autoware source patching; observation only.
- Not a schedulability analysis (no WCET); the model is structural + rates.

## Relation to existing work

| Concept | Source | Status here |
|---|---|---|
| Cause-effect chains, age vs reaction | AUTOSAR TIMEX; Becker et al. | vocabulary candidate (Q2/Q3) |
| Chain-aware priority assignment | PiCAS (RTAS 2021) | target mapper shape |
| Executor semantics matter | Casini et al. (ECRTS 2019) | trigger census (Q4) |
| Logical time / reactions | Lingua Franca | conceptual reference only |
| Polled vs causal subscription | our manifest `state:` | cycle-cut primitive under test (Q1) |
| Measured flow | Phase 29 interception | measurement lens |
