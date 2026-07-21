# Phase 49: Retire the last `record.json` reader + close the `replay --model` degradations

**Status:** 📋 Planned.
**Motivated by:** Phase 47 removed `record.json` as a user artifact but left one
CLI reader (`context`), and Phase 47/48/45.10 left `replay --model` (standalone,
no in-memory `LaunchDump`) degrading several best-effort features to empty.
Phase 48 gave `structure.scopes` the file/pkg/parent granularity that makes most
of this now sourceable from the model.

Two independent tracks.

## A. `context` → SystemModel (retire the last `record.json` reader)

`play_launch context <record.json> --tree|--node <FQN>|--launch <PKG> <FILE>`
(`commands/context.rs`) is the ONLY remaining consumer of `record.json`. Since
Phase 47 nothing WRITES a `record.json` on any primary path, so `context` today
only works on an old/hand-produced file — it's effectively dead. Migrate it to
read the SystemModel (`system_model.yaml`), which every `resolve`/`dump`/`launch`
now emits.

**What `context` reads today** (`ScopeEntry`): `pkg()`, `file()`, `ns`
(per-scope namespace), `args` (per-include launch args), plus per-node `scope`
and each record's `ns`/`name`. **What the model now carries** (Phase 48
`ScopeInfo`): `parent`, `manifest`, `package`, `file`, and per-`NodeInstance`
`scope` + `namespace`. Gaps to close:

- **49.A1 — namespace display**: Phase 48 detangled namespace from scope (scope
  is the launch FILE now, not a namespace). So `--tree`/`--node` must show each
  node's namespace from `NodeInstance.namespace`, NOT from a per-scope `ns`
  field (which no longer exists / no longer means "the namespace"). This is a
  semantic improvement — the tree shows include structure + per-node namespaces,
  matching the Phase 48 model.
- **49.A2 — include args**: `context --launch <pkg> <file>` prints the resolved
  launch arguments passed to each include. `ScopeInfo` does NOT carry them.
  Either add `args: BTreeMap<String,String>` to `ScopeInfo` (the resolved
  include args are already in the parser's `ScopeEntry.args`; `model_builder`
  just needs to carry them through — additive, backward-compatible serde) or
  drop the args view. Prefer carrying them: it's the one genuinely-launch-tree
  fact the model still lacks, and cheap to add.
- **49.A3 — CLI surface**: `context` takes a model path (`.yaml`/`.yml`)
  instead of `record.json`; keep a `.json` fallback only if a use for old
  records survives (probably not — cut it). Update `ContextArgs` (`record`
  field → `model`), help text, and CLAUDE.md's `context` references.
- **49.A4 — delete dead code**: once `context` is off `record.json`, the
  `LaunchDump`→`context` record-reading helpers (`scope_chain`, `print_tree`
  over `ScopeEntry`, etc.) move to the model shape. `LaunchDump` itself STAYS
  (in-memory parser intermediate) — only the disk-read + `ScopeEntry`-walking
  in `context` goes.

Net: `record.json` has ZERO readers/writers — fully retired end to end.

## B. Close (or formally accept) the `replay --model` degradations

Standalone `replay <model.yaml>` has no in-memory `LaunchDump` (Phase 47 passes
an empty one), so several best-effort features degrade to empty. Each is
independently either model-sourceable now or a genuine accept-and-document.

- **49.B1 — web launch-tree scope map (model-sourceable)**: `GET
  /api/launch-tree` serves `WebState.scopes` + `node_scope_map`, built from the
  `LaunchDump` → empty on standalone `replay --model`. Source them from
  `model.structure.scopes` (the include tree, Phase 48) + each
  `NodeInstance.scope` instead, so the Launch page works on model replay. The
  web UI's own JS already consumes `{scopes, node_scopes}`; only the server-side
  builder changes. This is the carried-forward 47.C1.
- **49.B2 — composable co-location warning (accept + document, or re-derive)**:
  `SchedPlan::from_model` yields empty `chain_member_nodes` (Phase 45.10 — chains
  are no longer embedded), so the container co-location warning is silent on the
  model path. Chains are NOT in the model (sched-revert: model is INPUT only), so
  the ONLY way to recompute membership is a fresh derive, which needs the
  `ManifestIndex` (`chains:` come from contracts) — unavailable on a bare
  `replay --model`. Options: (a) accept + document (it's a best-effort warning;
  `launch`/`run` with the dump still fire it); (b) if `replay --model` is given
  `--contracts`/a sidecar, re-derive chains from the manifest + the model's
  nodes and populate `chain_member_nodes`. (a) is the honest default; (b) only
  if a user actually wants co-location checks on standalone replay.
- **49.B3 — `replay --model --explain` (accept + document)**: renders a
  tier-based table (class/priority/core from `tiers`+`bindings`) with
  `"tier '<name>' → prio <p>"` provenance — no chain-aware ranks / override
  classification, because those need a fresh derive (not embedded, Phase 45.10).
  This is the correct degradation: the applied schedule is faithful; only the
  *provenance* detail is absent. Document that full provenance needs `check
  --sched --explain` (which re-derives). No code change — just make the note in
  the guide + the `--explain` help.

## Order

A and B are independent. Within B: B1 is a real fix (model-source the scope
map); B2/B3 are mostly documentation of accepted degradations (with an optional
B2 re-derive path). 49.A2 (add `ScopeInfo.args`) is a shared-model additive
field — coordinate with the nano-ros model-ingest track like Phase 48 (nano-ros
ignores scopes, so low risk), but it IS a model schema touch.

## Out of scope

Re-embedding the sched plan in the model (reverted in 45.10 — the model stays
INPUT only); the parser (unchanged); nano-ros's own model consumption.
