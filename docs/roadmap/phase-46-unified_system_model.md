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
- **46.1b** Linux-serving launch fields (`remaps`, `ros_args`, `respawn`/
  `respawn_delay`, `env`): the study corrected the earlier assumption —
  nano-ros does NOT need these (it uses resolved `structure.topics`, has no
  argv/respawn model). USER DECISION pending (design §Cross-track (a)/(b)):
  (a) still add to shared `NodeInstance` per the all-launch-info principle
  (nano-ros ignores), or (b) keep play_launch-side. Either is additive.
  **(model crate touch only under (a) — cross-track.)**
- **46.2** Populate: `resolve`/model_builder fill the new fields from the
  parsed launch (they already flow through the LaunchDump the resolver reads).
- **46.3** Spawn-from-model: relocate cmdline assembly (`node_cmdline.rs`) to
  derive exec path (ament) + argv + injected env + materialized param files
  from the model at spawn. `replay` reads the model, not `record.json`. This
  is the load-bearing item — the Linux spawn path must reproduce today's
  behavior from the model inputs (regression-test against current
  `record.json`-driven spawns).
- **46.4** `dump` emits the model; retire `record.json`, `meta.record`
  binding, `verify_model_record_binding`, the record companion in `resolve`.
- **46.5** Drop the dead LaunchDump-only artifacts (`file_data`, `variables`)
  from the emitted artifact; keep whatever the parser needs internally.
- **46.6** Docs: guide + design cross-refs; `dump`/`replay`/`resolve` one-artifact story.

## Order and dependencies

46.0 → 46.1 → 46.2 → 46.3 → 46.4 → 46.5 → 46.6. 46.3 is the largest and
riskiest (core spawn path); gate it behind thorough regression (Autoware +
rt_workspace spawns must match record.json-era behavior). 46.1 is the
cross-track gate.

## Out of scope

Storing platform-resolved outputs in the shared artifact; contract/sched layer
changes (Phases 44/45 stand); nano-ros's consumption of the new fields.
