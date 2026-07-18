# SystemModel as the Scheduling SSoT — Design

**Date:** 2026-07-18
**Status:** Approved (direction), pending implementation (Phase 45)
**Scope:** spans two tracks — the RT-scheduling track (vocab v2 + chain_aware,
Phases 41/42/44) and the SystemModel track (Phase 43, `model` crate).
**Supersedes / promotes:** `system-model-vocab-v2-embedding.md` (was a
"coordination note, not a design change" — this document makes it the design).
**Related:** `system-model.md` (the artifact), `chain-aware-mapper-design.md`,
`contract-vocabulary-v2-design.md`, `.superpowers/sdd/design-review-ours-theirs.md`,
`.superpowers/sdd/warning-diagnosis-study.md`.

## Decision

**The SystemModel is the single source of truth for scheduling.** The sched
derivation (contracts + platform file → chain_aware/rate/deadline/manual
mapper) runs once, at `resolve` time, and writes its *complete* output into
the model. Every downstream task — runtime apply (`replay --model`),
`--explain`, analysis, monitoring, and the off-host nano-ros consumer — reads
scheduling from the model and never re-derives it. `from_model` becomes a pure
reader; `check`'s derivation exists only to produce and validate the model,
not as a parallel runtime path.

This resolves the design review's central finding: today there are **two
disconnected scheduling systems** — the derived chain_aware plan (reachable
only via `check`/`--sched`) and the model's flat `execution.tiers`/`bindings`
(what `launch`/`replay --model` actually apply). The numbers happen to agree
on the default path because `resolve` runs the mapper, but the *structure*
(chains, per-path ranks, mapper identity, S·B·S decomposition) dies at the
artifact boundary. That is exactly the boundary the SystemModel exists to
make authoritative.

## Why this and not the alternatives

- **Numbers-only (status quo):** the artifact carries flat per-node
  priorities but no chain structure. `--explain` stays a check-time
  re-derivation; nano-ros — the artifact's whole second audience — gets no
  chain reasoning. A "shared truth" that drops the RT structure is not a
  shared truth for RT. Rejected.
- **Store-inputs, re-derive at runtime:** the model references the platform
  file and every consumer re-runs the mapper. Keeps one derivation
  implementation but re-introduces exactly the "the checked artifact is not
  the thing that runs" gap Phase 43 was built to close, and forces every
  off-host consumer to link the mapper. Rejected.
- **Embed the resolved plan (chosen):** the model carries what was derived
  and reviewed. One derivation, at resolve time; every consumer reads. This
  is the same "one canonical input" argument `system-model.md` already makes
  for every other layer-2 fact — chains were simply not yet vocabulary when
  that document was written.

## Target architecture

```
launch tree ─┐
contracts ───┼─► resolve ──(parse→bind→merge→check→DERIVE SCHED)──► SystemModel
platform file┘        (chain_aware / rate_mono / deadline_mono / manual)   │
system.toml ─┘                                                             │
                        ┌──────────────────────┬──────────────┬───────────┤
                    replay --model         check --explain   analysis   nano-ros
                    (apply, pure reader)   (render, reader)  monitoring  (off-host)
```

### What the model gains

**Layer 2 (`contracts:`) — authored facts, additive (vocab v2 §6):**
per-path `trigger:` (timer/input/once/spontaneous), `sync:`, `buffer:`, and
the authored `chains:` (scope-relative). Embedded verbatim from the `types`
crate — no parallel copies (see "Type sharing" below).

**Layer 3 (`execution:`) — the RESOLVED sched plan:** today only
`tiers: BTreeMap<String,TierDef>` + `bindings: FQN→tier`. It gains:

- `mapper: String` — which mapper produced this plan (provenance).
- resolved chains: `Vec<ResolvedChain>` — FQN-qualified `via` topics, the
  S·B·S decomposition (`ChainElement::{Segment,Boundary}`), each chain's
  budget/semantics/criticality. This is `chain_member_nodes` made explicit
  and durable.
- per-path ranks: `Vec<ChainAwareDetail>` — one per (node, path), with the
  provenance string. The POSIX apply layer projects to per-node max (a
  documented lossy compression); embedding the per-path ranks lets an RTOS
  executor (nano-ros) discriminate at callback granularity — the finer fact
  play_launch computes and then throws away at its own POSIX boundary.

`bindings`/`tiers` stay the applied representation (per-node priority/policy/
core); the additions are the *structure* behind them. `from_model`
reconstructs membership, the colocation warning, and `--explain` from these
fields instead of returning `chain_member_nodes: empty`.

### Consumers become readers

- **`replay --model` / `launch`:** `from_model` applies `bindings`+`tiers`
  and reads the resolved chains for the colocation warning — no
  ManifestIndex re-parse fallback needed.
- **`--explain`:** promoted from a `CheckArgs`-only flag to a model-reader.
  `check --explain` renders from the model it just built; a new
  `replay --model --explain` / `resolve --explain` renders from the stored
  model. Same renderer, one code path, works off-host.
- **Analysis / monitoring:** read chains + per-path ranks from the model —
  no contract re-parse. (These tasks are named as future consumers; this
  design only guarantees the model carries what they need.)

## Folded-in decisions

### Warning diagnosis (unify the renderer)

`.superpowers/sdd/warning-diagnosis-study.md`: real Autoware check emits 37
warning events as 111 lines. Three fixes, all part of moving diagnosis onto
the model-consuming layer:

1. **Kill the duplicate-print bug** — the same `Vec<String>` is printed by
   three uncoordinated sites in `sched_loader.rs` (`derive_sched_plan`'s
   `tracing::warn!`, `check_sched`'s loop, `render_explain`'s loop). One
   authoritative site. ~70% of the flood, free.
2. **Make the contradiction detector chain-aware** — `validate.rs::
   scan_contradictions` flags chain members ranked by drain-toward-sink as
   "contradicting" their raw rate/deadline order: 74% of unique
   contradictions are false-positives against the tool's own documented
   design. Suppress the contradiction when the higher-priority side is a
   chain member; fold override-derivative contradictions into the existing
   "override pins chain member" warning as a consequences sub-list. The
   chain-membership data is already in scope at the call site (and, post-
   migration, in the model).
3. **One diagnostic renderer for the whole command** — the manifest-checker
   half of `check` already uses `codespan-reporting` (ids, spans, per-file
   summary, dedup); the sched/runtime half emits bare `String`s. Give sched
   diagnostics the same structure (rule id, dedup, "N warnings emitted"
   summary). Combined: 111 lines → ≤3 with no signal loss.

### FQN identity — one builder

Three FQN-construction functions exist: `manifest_loader::qualify_name`,
`sched_loader::fqn_for`, `model_builder::fqn`. The third lacks the scope-ns
fallback — the **same bug class** as the Phase 44.6 `vehicle_cmd_gate` fix
(node behind a bare `<group>` resolves to a phantom FQN). Unify to one
reconciled identity (the launch-dump identity, `ManifestIndex::node_identity`,
which is what the apply layer keys on). The model's resolve step must use the
same reconciled identity so embedded chain/binding FQNs match the applied
node names.

### Platform file vs `system.toml` — delineate now, converge target

Two integrator config surfaces feed `execution:` today: the sched **platform
file** (`<stem>.system.<target>.yaml`, 3-channel overlay/provider discovery,
`mapper:` + `resources:` + `overrides:`) and **`system.toml`** (bare
`--system` path, manual `[tiers]` + `[[assign]]`). Both resolve *into* the
model's execution layer.

- **Now:** delineate cleanly — `system.toml` = manual-tier authoring (the
  `manual` mapper world), platform file = derived-mapper authoring
  (chain_aware/rate/deadline). `resolve` reads whichever is provided and
  produces the same `execution:` output. Document that they are two front
  ends to one resolved layer.
- **Target:** converge to one authoring surface once the model is the SSoT
  and the two front ends have demonstrably identical resolved output. Not
  required for this phase; tracked.

## Type sharing (no parallel copies)

The `model` crate must reuse the **declaration-side** `types` structs
(`Trigger`, `EffectiveTrigger`, `Sync`, `ChainDecl`, `ChainSegment`,
`Buffer`) and the **resolved-side** `sched` structs (`ResolvedChain`,
`ChainElement`, `MapperPath`, `ChainAwareDetail`) — both already `Serialize`.
The translation `types::ChainDecl` + launch DAG → `sched::ResolvedChain` is
`sched_derive::resolve_chains` (play_launch, Phase 44.4), already tested. It
must not be reimplemented in the `model` crate: either `resolve` calls it and
embeds the output, or the function moves to a crate both `play_launch` and
`model` depend on. One translation, embedded verbatim.

`sched/src/chain.rs` today hand-mirrors `EffectiveTrigger`/`ChainDecl`
(deliberate, dependency-light). The migration must decide: make `sched`
depend on `types` (retire the mirror), or accept the mirror and forbid a
*third* copy in `model`. Recommended: retire the mirror when `model` embeds —
one declaration home (`types`), one resolved home (`sched`).

## Migration

Additive to the model schema (new optional fields; old models still parse).
Phased across both tracks — see [phase-45](../roadmap/phase-45-sched_ssot_unification.md).
No flag day: the `replay --model` path already applies correct numbers today;
the migration adds structure, then flips `--explain`/analysis/monitoring onto
the model, then removes the check-time re-derivation as a separate runtime
path.

## Non-goals

- Rewriting the spawn artifact: `record.json` stays the spawn-info source
  (Phase 43 two-artifact decision); the model owns identity + contracts +
  scheduling only.
- Callback-level apply on POSIX (per-path ranks are carried for RTOS
  consumers, still projected to per-node max on Linux).
- nano-ros's own consumption of the embedded chains (its track; this design
  guarantees the artifact carries the facts).
- Forcing platform-file/`system.toml` convergence in this phase (delineated
  now, convergence tracked).
