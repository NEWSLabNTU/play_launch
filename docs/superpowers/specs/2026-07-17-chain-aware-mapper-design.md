# Chain-Aware Mapper (`chain_aware`) — Design

**Date:** 2026-07-17
**Status:** Approved (design), pending implementation
**Phase:** 42.6 (see `docs/roadmap/phase-42-autoware_system_model.md`)
**Consumes:** Contract Vocabulary v2 (`2026-07-17-contract-vocabulary-v2-design.md`) —
path triggers, `state:`/`buffer:`, `sync:`, `chains:`.
**Evidence base:** `docs/research/autoware-system-model.md` (Q1–Q5).
**Related work:** PiCAS (Choi et al., RTAS 2021) — chain-aware priority
assignment for ROS 2; Davare et al. (DAC 2007) — period costs in cause-effect
chains; Becker et al. — data-age over sampled links; AUTOSAR TIMEX semantics.

## Model: clock-segmented chains

PiCAS assumes unbroken event-driven chains. Real Autoware routes are DAGs whose
end-to-end causality is **cut by clock-sampled links**: a timer-triggered path
reading `state:` inputs decouples its upstream from its downstream — latency
across that crossing is dominated by the sampling *period*, and no priority
assignment can reduce it (only meeting the timer's own period matters).

A declared chain therefore decomposes into an alternating sequence of:

- **Event segments** — maximal runs of `trigger: input` path links. Inside a
  segment, scheduling controls latency; PiCAS applies.
- **Clock boundaries** — chain hops through a `trigger: timer` path. Crossing
  cost (reaction semantics): up to one period + the boundary path's declared
  `max_latency_ms` (when present; period alone otherwise — no invented WCETs).

**Decomposition is declaration-driven** (design issue #1): chains reference
paths; each referenced path's `trigger:` says segment-link vs boundary. No
node-kind inference — a hybrid node (e.g. `vehicle_cmd_gate`: input-triggered
`forward` + timer `status_tick`) participates through whichever of its paths
the chain names. Cyclic chain declarations are rejected (`chain-link` error): a
feedback loop is not a chain.

```
S1: lidar_preproc → concat → detector   B1: ekf (50 Hz)   B2: planning_tick (10 Hz)   S2: follower → gate
sampling_cost = (20 + exec_B1) + (100 + exec_B2)
controllable  = budget − sampling_cost      # what scheduling can still influence
```

Out of scope (agreed): cross-boundary phase control (offset/LET-style timer
alignment) — changes node code assumptions. Harmonic-period refinement of the
conservative one-period-per-boundary bound: future work; the conservative bound
is used for checking.

### Boundary consumption rule (implementation note, 2026-07-17)

A `Timer`-triggered path's own trigger fact carries no input endpoint (a
timer callback has no input) — but that does not mean a Boundary consumes
nothing from a preceding `via`. Rule: **a Boundary consumes an incoming
`via` through its NODE's subscription on that topic** — the `state:` sub
(or any ordinary sub) IS the sampling mechanism that makes this a
clock-segmented model in the first place. Concretely, `chain-link`'s
"via consumed by the following segment" check resolves a Timer segment's
`input_topics` as every topic FQN the owning node subscribes to anywhere
(all its `sub:` endpoints across the whole node, not just the endpoints
named by this path's own trigger) — the honest reading of "a boundary
samples whatever it subscribes."

Without this rule, a Boundary's `input_topics` always resolved empty,
which made `Segment → via → Boundary` structurally undeclarable —
`chain-link` would always reject the `via` as "not consumed by the
following segment" — even though this worked example's own `S1 B1 S2 B2
S3` shape requires exactly that transition twice. Implemented in
`chain_checks::resolve_segment` (`src/play_launch/src/ros/chain_checks.rs`);
mirrored nowhere else since the sched-extraction decomposition
(`sched_derive::build_resolved_chain`) only branches on trigger kind
(`Timer` vs. not) to alternate `Segment`/`Boundary` elements — it never
reads `input_topics` — so it already supported the S·B·S shape once
`chain-link` stopped rejecting it.

## Companion checker rules (land regardless of mapper)

- **`chain-sampling-feasibility`**: `sampling_cost(chain) ≥ budget` → the chain
  is structurally infeasible; scheduling cannot fix it (period/architecture
  change required). Diagnostic lists the per-boundary period breakdown.
- **`chain-budget` (extended)**: Σ segment declared `max_latency_ms` +
  sampling_cost ≤ chain budget — pure arithmetic over declarations (no
  response-time analysis is claimed or required; design issue #2).

## Algorithm

Inputs: causal DAG + resolved `chains:` + per-path trigger facts + node
`criticality` + `PlatformFacts` (band). Output: the existing `SchedPlan`;
`overrides:` and band-clamp validation stay downstream, unchanged.

1. **Partition** (O(V+E)): resolve each chain to `S₁ B₁ S₂ … Sₖ` from declared
   path triggers. Paths/nodes not on any chain: classified by their own facts
   for step 5.
2. **Feasibility**: compute `sampling_cost` / `controllable` per chain; emit the
   companion diagnostics. Chains with `controllable ≤ 0` are excluded from
   priority shaping (members keep their local-fact priorities).
3. **Chain order**: criticality desc → controllable-slack asc (tighter = more
   urgent) → name. (PiCAS chain-criticality ordering; slack as the principled
   tiebreaker.)
4. **Within-chain ranks** (each chain claims the next ranks below the previous
   chain's):
   - Segments ranked by position: **later segments above earlier ones** — the
     tail after the LAST boundary is pure scheduling latency and adds 1:1 to
     E2E, while pre-boundary latency is partially absorbed by sampling slack.
   - **Within a segment: drain-toward-sink** (PiCAS): downstream > upstream, so
     in-flight data flushes to the segment sink without preemption from fresh
     arrivals at the head. Fan-in (DAG) inside a segment: rank by
     longest-path-to-sink, ties by declared deadline, then name. `sync:` does
     not affect ranks (it affects the latency math: oldest-matched-input).
   - **Boundary timers**: ranked among themselves by RM (shorter period
     higher), placed below the chain's segments; they inherit the criticality
     of the most critical chain through them. Their obligation is local —
     complete within their own period — because step 2 already charged the
     chain a full period per crossing.
   - **Shared nodes** (multiple chains): take the max rank. Known anomaly
     (design issue #4): this can invert drain order within the
     lower-criticality chain around the lifted node; accepted (PiCAS makes the
     same trade), bounded by the chain processing order.
5. **Non-chain remainder** (below all chain ranks): criticality buckets, then
   within bucket `timer` paths by RM and `input` paths by DM
   (`max_latency_ms`); **equal facts collapse to equal priority** (Phase 41
   decision — no alphabetical spread). `once` / `spontaneous` / fact-less →
   `SCHED_OTHER`. With zero chains declared the mapper IS this step —
   graceful degradation to criticality-bucketed RM/DM.
6. **Node projection** (design issue #3): the apply layer is per-TID-uniform
   per node; a node's priority = **max over its paths' ranks**. This is a lossy
   but honest projection — intra-node discrimination (gate's 5 ms `forward` vs
   its 10 Hz `status_tick`) needs callback-level executor cooperation,
   impossible on vanilla rclcpp. The shared crate's plan SHOULD carry per-path
   ranks so consumers that can act on them (nano-ros executors on RTOS) are
   not limited by the POSIX projection.
7. **Band compression**: dense ranks → `[band.min, band.max]`, order-preserving.
   When the band is narrower than the rank count, collapse adjacent ranks
   **within segments first** (equal-priority SCHED_FIFO approximates pipeline
   order via arrival-order queuing), then across segments of the same chain,
   never across the chain/non-chain divide or across criticality buckets
   (design issue #5).
8. **Warnings**: chain members co-located in a stock (non-isolated) container
   share one process and cannot receive distinct priorities → warning naming
   the container and members (design issue #7). Every rank decision emits an
   `--explain` provenance string:
   `derived(chain_aware: sensing_to_actuation S2 drain 2/2 → prio 44)`.

## Worked example

Chain `sensing_to_actuation`, budget 150 ms, band 5–45; S1 = preproc→concat→
detector (Σ declared 45 ms), B1 = ekf 50 Hz, B2 = planning 10 Hz, S2 =
follower→gate (gate `forward` deadline 5 ms):

- Feasibility: sampling_cost = 120 ms → controllable = 30 ms. Feasible;
  `chain-budget` warns: S1's declared 45 ms alone exceeds the 30 ms residual —
  visible at `check` time, before any run.
- Ranks: gate > follower (S2 drain) > ekf > planning (boundary RM) > detector >
  concat > preproc (S1 drain) > … non-chain: 40 Hz sim timer (RM bucket) → map
  loader (`once`) OTHER.
- Band 5–45: 45,44,43,42,41,40,39 …; sim timer 30; loader OTHER.

## Properties and limits

Deterministic (all orders total); monotonic with criticality; degrades to
criticality-RM/DM with no chains; O(V+E+Σ|chain|); every decision explainable
in one line. Limits, stated: one-period-per-boundary is conservative (harmonic
periods have tighter exact bounds — future refinement); drain-toward-sink is an
empirically-validated heuristic (PiCAS), not an optimality theorem for general
DAGs; no schedulability guarantee is claimed anywhere (no WCET facts — the
Audsley/OPA endgame requires them); the POSIX projection is per-node max
(callback granularity reserved for executor-cooperating consumers).

## Implementation home

`SchedMapper` impl `chain_aware` in the sched crate (registry addition —
Phase 41 trait unchanged); `MapperInput` grows optional per-path trigger facts
and resolved chains (additive). Companion checker rules land with the
vocabulary-v2 implementation. Implementation phasing belongs to a future phase
doc (not 42 — the study phase ends with this spec).
