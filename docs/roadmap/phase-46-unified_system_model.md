# Phase 46: Unified SystemModel — One Complete Artifact

**Status:** 📋 Planned
**Design of record:** [docs/design/unified-system-model.md](../design/unified-system-model.md)
**Revises:** Phase 43 two-artifact decision (superseded).
**Builds on:** Phase 43 (resolve/replay --model), Phase 45 (execution.sched).
**Cross-track:** 46.1 (model-crate launch fields) shared with the nano-ros track — coordinate first.

## Overview

Makes the SystemModel the one complete artifact: it carries all
launch-derived info (+ optional contract + optional system config); both
play_launch and nano-ros read the full model and each derives its own
platform specifics (exec path, argv, bake). `record.json` retires. The user
perceives one kind of dump.

## Work items

- **46.0** Cross-track coordination: confirm the `NodeInstance` launch-field
  set + semantics with the nano-ros track (RFC-0050) before landing 46.1.
  Note like the sched-SSoT handshake.
- **46.1** `<node machine=>` → `execution.deploy[fqn].host` (nano-ros #236,
  the real cross-track win): the parser already captures `machine` (ir.rs)
  but it's dropped — `LaunchDump::NodeRecord` has no `machine` field, so
  `model_builder` never writes `deploy.host`. Add `machine` to the record
  path + populate `deploy.host`. Unblocks nano-ros's multihost migration.
- **46.1b** Shared launch fields (DECIDED 2026-07-20: option a): add
  `remaps`, `ros_args`, `respawn`/`respawn_delay`, launch-declared `env` to
  the shared `NodeInstance` per the all-launch-info principle (nano-ros
  ignores the ones it doesn't consume — no `deny_unknown_fields`). Additive,
  optional, backward-compat. **(model crate — cross-track.)**
- **46.2** Populate: `resolve`/model_builder fill the new fields from the
  parsed launch (they already flow through the LaunchDump the resolver reads).
- **46.3** Spawn-from-model: relocate cmdline assembly (`node_cmdline.rs`) to
  derive exec path (ament) + argv + injected env + materialized param files
  from the model at spawn. `replay` reads the model, not `record.json`. This
  is the load-bearing item — the Linux spawn path must reproduce today's
  behavior from the model inputs (regression-test against current
  `record.json`-driven spawns).
- **46.4** (Python-parser preservation FIRST — the acceptance gate): make
  `dump` emit a SystemModel for BOTH parsers — `--parser rust` → full model
  (structure+contracts+sched), `--parser python` → the SAME complete model
  (Python parse → LaunchDump → model_builder). CORRECTION 2026-07-20:
  contracts/sched apply on the shared scope table regardless of parser
  (Phase 40.1 `ScopeOrigin.path`) — Python gets FULL parity with Rust
  (verified byte-identical structure/contracts/execution), NOT structure-only.
  Layers are empty only when no sidecar/`--sched` input resolves. CAVEAT for
  46.5: a stale pip install silently drops `ScopeOrigin.path` → silent
  contract/sched loss; dump→model convergence must use current source or fail loud. Remove the now-vestigial
  `meta.record` binding gate (`verify_model_record_binding`) so
  `replay --model` spawns from the self-sufficient model (46.3b) without a
  record companion. **ACCEPTANCE (must pass before 46.5): `dump --parser
  python <standard launch>` → `replay --model` spawns cleanly** (rt_workspace
  + a standard-ROS launch), AND the Rust path keeps its Autoware parity.
  Verified 2026-07-20: Python→record→model bridge already produces a valid
  structure-only model that drives spawning; only the binding gate causes
  friction (removed here).
- **46.5** Drop the dead LaunchDump-only artifacts (`file_data`, `variables`)
  from the emitted artifact; keep whatever the parser needs internally.
- **46.6** Docs: guide + design cross-refs; `dump`/`replay`/`resolve` one-artifact story.

## Order and dependencies

46.0 (coordinate) → **46.1 (machine=/#236 — ships first, quick unblock for
nano-ros)** → 46.1b/46.2 (shared launch fields) → 46.3 (spawn-from-model —
largest/riskiest, regression-gated: Autoware + rt_workspace spawns must match
record.json-era) → **retirement 46.4/46.5 (non-additive, gated on 46.3's
regression suite)** → 46.6 docs. Through 46.3 both paths coexist (additive
model + record.json still written); retirement is the only non-additive step.

**Coordination (46.0):** note nano-ros (issue #236 + RFC-0050) with the
unified-model design + the incoming schema additions; confirm no field they
read is omitted. Same handshake as the sched-SSoT reconciliation.

**Retirement (46.4/46.5), only after 46.3 proves parity:** `dump` emits the
one model; `record.json`, `meta.record` binding, `verify_model_record_binding`,
and the `resolve` record companion are removed; `file_data`/`variables` dropped
from the emitted model.

## Out of scope

Storing platform-resolved outputs in the shared artifact; contract/sched layer
changes (Phases 44/45 stand); nano-ros's consumption of the new fields.
