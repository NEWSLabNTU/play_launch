# Phase 45: Scheduling SSoT Unification

**Status:** 📋 Planned
**Design of record:** [docs/design/system-model-sched-ssot.md](../design/system-model-sched-ssot.md)
**Builds on:** Phase 41 (sched v2), Phase 42 (study), Phase 43 (SystemModel runtime), Phase 44 (vocab v2 + chain_aware).
**Cross-track:** the `model`-crate work items (45.2, 45.3) are shared with the
Phase 43/SystemModel/nano-ros track — coordinate; do not land unilaterally.
**Evidence:** `.superpowers/sdd/design-review-ours-theirs.md`, `.superpowers/sdd/warning-diagnosis-study.md`.

## Overview

Makes the SystemModel the single source of truth for scheduling: `resolve`
runs the mapper once and embeds its complete output (resolved chains, per-path
ranks, mapper identity) into the model; every downstream task — runtime apply,
`--explain`, analysis, monitoring, nano-ros — reads scheduling from the model
and never re-derives it. Also unifies the diagnostic renderer, collapses the
Autoware warning flood, and unifies the three FQN builders.

All schema changes are additive (old models still parse); no flag day.

## Work items

- **45.1** Warning diagnosis (independent, ship first — no schema change):
  (a) fix the duplicate-print bug in `sched_loader.rs` (one authoritative
  print site); (b) make `validate.rs::scan_contradictions` chain-aware —
  suppress when the higher-priority side is a chain member, fold
  override-derivative contradictions into the "override pins" warning as a
  consequences sub-list; (c) give sched/runtime diagnostics the same
  structured presentation (rule id, dedup, summary line) the manifest-checker
  half already has via `codespan-reporting`. Target: Autoware 111 lines → ≤3.
- **45.2** Model schema: `execution:` gains `mapper: String`, resolved
  `chains: Vec<ResolvedChain>`, per-path `Vec<ChainAwareDetail>`; `contracts:`
  gains the vocab-v2 authored facts (triggers/sync/buffer/chains) via the
  shared `types` structs. Additive, optional, serde round-trip + golden tests.
  **(model crate — cross-track.)**
- **45.3** Type sharing: retire `sched/src/chain.rs`'s hand-mirror of
  `EffectiveTrigger`/`ChainDecl` in favor of a `types` dependency (or forbid a
  third copy in `model` and document); move/host `resolve_chains` so
  `play_launch` and `model` share one translation. **(sched + model — cross-track.)**
- **45.4** `resolve` embeds: `commands/resolve.rs` calls the existing
  `derive_sched_plan`/`resolve_chains` and writes the full resolved plan +
  chains + per-path ranks into the model's execution/contracts layers, using
  the reconciled launch-dump identity.
- **45.5** `from_model` becomes a pure reader: reconstructs
  `chain_member_nodes`, the container co-location warning, and the applied
  plan from the model's embedded chains — retires the ManifestIndex re-parse
  fallback on the model path.
- **45.6** `--explain` on the model: promote from `CheckArgs`-only to a
  model-reader; `check --explain` renders the model it built, add
  `resolve --explain` / `replay --model --explain` rendering the stored model.
  One renderer.
- **45.7** FQN unification: one builder (`manifest_loader::qualify_name` /
  `sched_loader::fqn_for` / `model_builder::fqn` → one reconciled identity);
  regression test the namespace-override/bare-`<group>` shape against
  `model_builder`.
- **45.8** Platform-file / `system.toml` delineation: document the two front
  ends → one `execution:` layer; both produce identical resolved output for
  the manual case (test). Convergence to one surface is tracked, not done here.
- **45.9** Docs sweep: `system-model.md` (SSoT + sched-layer detail), the two
  sched specs (reachability now via the model), phase-43/44 cross-refs, RT
  guide (`--explain` works on the model / off the launch path).

## Order and dependencies

45.1 ships independently first (pure UX win, no schema change). Then
45.2 → 45.3 → 45.4 → (45.5, 45.6 parallel) → 45.7 → 45.8 → 45.9. 45.2/45.3
are cross-track gates (model crate); coordinate before 45.4 depends on them.

## Out of scope

Spawn-artifact changes (record.json stays); callback-level POSIX apply;
nano-ros's consumption of embedded chains; forcing platform-file/system.toml
convergence.
