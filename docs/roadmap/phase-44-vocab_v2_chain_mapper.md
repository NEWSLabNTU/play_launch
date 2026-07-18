# Phase 44: Vocabulary v2 + Chain-Aware Mapper — Linux Implementation

**Status:** 🔄 44.1–44.6, 44.8 done; 44.7 handoff (coordination note delivered)
**Designs of record:**
[contract vocabulary v2](../superpowers/specs/2026-07-17-contract-vocabulary-v2-design.md) ·
[chain-aware mapper](../superpowers/specs/2026-07-17-chain-aware-mapper-design.md)
**Builds on:** Phase 41 (SchedMapper pipeline), Phase 42 (study + designs), Phase 43 (SystemModel — layer-2 coordination point).
**Evidence:** `docs/research/autoware-system-model.md`.

## Overview

Implements the two Phase 42 designs on Linux: the additive contract vocabulary
(explicit path triggers, `sync:`, `buffer:`, integrator `chains:`) and the
`chain_aware` mapper (clock-segmented chains: PiCAS drain-toward-sink within
event segments, RM boundaries, criticality-RM/DM fallback), with their checker
rules and end-to-end tests against rt_workspace and Autoware planning_simulator.
All schema changes are additive; legacy contracts and the Phase 41 mappers keep
working unchanged.

## Work items

- **44.1** ✅ Types + parse (`ros-launch-manifest` `types` crate): `trigger:`
  (timer/input/once/spontaneous), `sync:` (exact/approximate/timeout_any),
  `buffer: latest|queue`, top-level `chains:` (segments/via/semantics/budget).
  Legacy derivation (non-empty `input:` ⇒ input-triggered; empty+no-trigger ⇒
  unclassified). Serde round-trip + golden tests. `EffectiveTrigger`/
  `PathDecl::effective_trigger()` is the single source of truth for every
  downstream consumer.
- **44.2** ✅ Checker rules (`check` crate + play_launch cross-scope layer):
  `explicit-trigger` (info), generalized `state-consistency` (warning; sub
  neither `state:` nor in any `trigger.input`), `inherited-rate` (warning;
  shipped as "stale legacy `input:` list alongside a non-Input explicit
  trigger" — the spec's original `rate_hz`-on-input-path clause has no
  schema field to check, see `p44-w1-report.md`), `once-durability`
  (warning), `sync-feasibility` (warning), `queue-drain-rate` (warning),
  `chain-shape` (error; per-manifest cyclic-chain + missing-`via`-between-
  adjacent-segments), `chain-link` (error; resolves `via:` across scopes,
  including the boundary-consumption rule — see 44.4/W6 fix below),
  `chain-budget` (warning; declared arithmetic incl. sampling cost),
  `chain-sampling-feasibility` (warning; Σ boundary periods vs budget).
- **44.3** ✅ `chain_aware` mapper (sched crate): registry addition; `MapperInput`
  gains optional per-path trigger facts + resolved chains (additive); plan
  carries per-path ranks (POSIX consumer projects per-node max; per-path kept
  for executor-cooperating consumers). Decomposition, chain/segment/boundary
  ordering, tie-collapse, band compression per spec. Unit tests incl. the
  worked example, shared-node anomaly, zero-chain degradation.
- **44.4** ✅ play_launch integration: extraction (contract paths → per-path
  MapperInput facts; chains resolved with scope FQNs), node projection,
  stock-container co-location warning, `--explain` chain provenance
  (`derived(chain_aware: <chain> segment drain N/M) -> prio N`), cyclic-chain
  rejection surfaced in `check`.
- **44.5** ✅ Fixtures + integration tests: rt_workspace's `bringup.contract.yaml`
  adopts explicit triggers on all three nodes + one chain (`points_to_cmd`:
  timer boundary → 2-node event segment), both platform files switched
  `mapper: rate_monotonic` → `chain_aware`; `control_node.cpp` re-wired
  `points_raw` → `points_filtered` so the declared chain matches the running
  code (contract-honesty fix). 35/35 rt_workspace|manifest_check|sched_apply|
  contract_eject tests green.
- **44.6** ✅ Autoware validation (`~/repos/autoware-contract`, branch
  `rt/vocab-v2`, not pushed): annotated the W3-censused nodes' triggers
  (`.superpowers/sdd/p42-w3-report.md` citations), closed the
  `/planning/trajectory` gap by authoring the real bridge node
  (`autoware_planning_validator`, not the study's scenario_selector/
  velocity_smoother guess), authored 3 chains (`planning_to_trajectory`
  8-hop, `control_to_actuation` 2-hop, `planning_to_actuation` their merge
  with an interior boundary). 0 errors / 2 pre-existing warnings across 64
  manifests, unchanged. **Two real defects found at scale, both fixed**
  (`.superpowers/sdd/p44-w6-fixes-report.md`):
  - Interior clock boundaries were structurally undeclarable — a `via:`
    landing on a non-first `Timer` segment always failed `chain-link`
    because `resolve_segment` never populated a boundary's `input_topics`.
    Fixed by the **boundary consumption rule**: a boundary consumes an
    incoming `via` through its node's own subscriptions (any `sub:`
    endpoint, not just the path's own empty trigger endpoints) — spec note
    added to the chain-aware-mapper design.
  - `vehicle_cmd_gate` (behind a bare `<group ns="/control">`, no
    `<include>`) resolved to a phantom contract-side FQN
    (`/vehicle_cmd_gate` instead of `/control/vehicle_cmd_gate`) because
    `ManifestIndex::node_identity` was keyed only by a record's immediate
    scope id, one level too shallow for a node nested behind an
    intervening bare group. Fixed by indexing under both the immediate
    scope id and the nearest ancestor **file** scope id.
- **44.7** 🔄 SystemModel layer-2 embedding — **coordination note delivered,
  embedding itself is the Phase 43/SystemModel track's job**: see
  [`docs/design/system-model-vocab-v2-embedding.md`](../design/system-model-vocab-v2-embedding.md)
  for the exact `types`/`sched`-crate structs to share (no parallel copies)
  and the residual gap it closes (`SchedPlan::from_model` has zero chain
  awareness today — a bare `replay --model <file>` run can't re-derive,
  re-explain, or warn about chain structure because the model's
  `execution:` layer doesn't carry it, per `p44-w4-report.md`'s documented
  gap). No `model`-crate code touched by Phase 44. **Realized by
  [Phase 45](./phase-45-sched_ssot_unification.md)**, item 45.2/45.4: the
  SystemModel-as-scheduling-SSoT decision
  ([system-model-sched-ssot.md](../design/system-model-sched-ssot.md))
  commits this coordination note to a concrete schema + `resolve`-time
  embedding.
- **44.8** ✅ Docs: `launch-manifest.md` vocabulary-v2 section gained the
  boundary-consumption rule, the via-required-between-segments rule, and a
  `Static Validation` table row for every Phase 44 rule with its severity
  (9 new rows, 25 total); RT guide (`docs/guide/rt-scheduling.md`) gained a
  new §1.7 "Chains" section: authoring workflow (declare paths w/ explicit
  triggers → declare chain w/ via links → `check` → `--explain` → pin
  overrides, with the real drain-order warning), `mapper: chain_aware`
  selection guidance vs. `rate_monotonic`/`deadline_monotonic`, the
  clock-segmented model in two paragraphs, and pointers to rt_workspace's
  `points_to_cmd` chain (runnable) and autoware-contract's 3 chains
  (at-scale). Every guide command re-run against the built binary.

## Order and dependencies

44.1 → (44.2, 44.3 parallel) → 44.4 → 44.5 → 44.6 → (44.7, 44.8). 44.7
coordinates with the SystemModel track — additive on both sides. 44.7's
embedding work itself is now a Phase 43/SystemModel-track follow-up; this
phase's own scope (coordination note) is complete.

## Out of scope

Callback-level priorities on rclcpp (per-path ranks are carried, not applied,
on POSIX); harmonic-period bound refinement; mode-conditioned contracts;
`consumes_hz`; LET/offset phase control; nano-ros consumption (other track).
