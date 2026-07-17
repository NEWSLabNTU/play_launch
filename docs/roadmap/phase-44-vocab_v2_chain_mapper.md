# Phase 44: Vocabulary v2 + Chain-Aware Mapper — Linux Implementation

**Status:** 📋 Planned
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

- **44.1** Types + parse (`ros-launch-manifest` `types` crate): `trigger:`
  (timer/input/once/spontaneous), `sync:` (exact/approximate/timeout_any),
  `buffer: latest|queue`, top-level `chains:` (segments/via/semantics/budget).
  Legacy derivation (non-empty `input:` ⇒ input-triggered; empty+no-trigger ⇒
  unclassified). Serde round-trip + golden tests.
- **44.2** Checker rules (`check` crate + play_launch cross-scope layer):
  `explicit-trigger`, generalized `state-consistency` (sub neither `state:` nor
  in any `trigger.input`), `inherited-rate`, `once-durability`,
  `sync-feasibility`, `queue-drain-rate`, `chain-link` (error; resolves `via:`
  across scopes), `chain-budget` (declared arithmetic incl. sampling cost),
  `chain-sampling-feasibility` (Σ boundary periods vs budget).
- **44.3** `chain_aware` mapper (sched crate): registry addition; `MapperInput`
  gains optional per-path trigger facts + resolved chains (additive); plan
  carries per-path ranks (POSIX consumer projects per-node max; per-path kept
  for executor-cooperating consumers). Decomposition, chain/segment/boundary
  ordering, tie-collapse, band compression per spec. Unit tests incl. the
  worked example, shared-node anomaly, zero-chain degradation.
- **44.4** play_launch integration: extraction (contract paths → per-path
  MapperInput facts; chains resolved with scope FQNs), node projection,
  stock-container co-location warning, `--explain` chain provenance
  (`derived(chain_aware: <chain> S2 drain 2/2 → prio N)`), cyclic-chain
  rejection surfaced in `check`.
- **44.5** Fixtures + integration tests: rt_workspace contracts adopt explicit
  triggers + one two-segment chain with a timer boundary (filter as boundary or
  add a tick path — smallest honest topology); tests: vocab parse-through,
  chain rules fire/quiet cases, `chain_aware` derivation asserted via
  `--explain`, sched_apply smoke with the chain mapper, legacy contracts
  regression (Phase 41 mappers + bridge untouched).
- **44.6** Autoware validation: in `~/repos/autoware-contract` annotate the
  W3-censused nodes' triggers (citations exist in
  `.superpowers/sdd/p42-w3-report.md`), add `state: true`/`buffer:` where the
  census proved them, author one `sensing_to_actuation` chain (expect
  `chain-link` to fail on the `/planning/trajectory` gap first — closing it by
  declaring the scenario_selector/velocity_smoother bridge is part of the
  item); run `check --explain` with `chain_aware` at scale; record derived-plan
  review + warning counts.
- **44.7** SystemModel layer-2 embedding: vocab structs shared with the `model`
  crate (no parallel copies) — coordinate with the Phase 43 track; resolved
  chains embed in the artifact.
- **44.8** Docs: `launch-manifest.md` vocabulary-v2 section (rules table rows);
  RT guide: chain authoring workflow (declare → `check` → `--explain` → pin),
  mapper selection (`mapper: chain_aware`).

## Order and dependencies

44.1 → (44.2, 44.3 parallel) → 44.4 → 44.5 → 44.6 → (44.7, 44.8). 44.7
coordinates with the SystemModel track — additive on both sides.

## Out of scope

Callback-level priorities on rclcpp (per-path ranks are carried, not applied,
on POSIX); harmonic-period bound refinement; mode-conditioned contracts;
`consumes_hz`; LET/offset phase control; nano-ros consumption (other track).
