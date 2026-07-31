# Phase 46: Unified SystemModel — One Complete Artifact

**Status:** ✅ Complete (2026-07-20) — 46.0–46.6 all shipped. `system_model.yaml`
is the one user-facing artifact; `record.json` is retired to a deprecated
compat/dev surface (`dump --format record`, `replay --input-file` without
`--model`).
**Design of record:** [docs/design/unified-system-model.md](../design/unified-system-model.md)
**Revises:** Phase 43 two-artifact decision (superseded).
**Builds on:** Phase 43 (resolve/replay --model), Phase 45 (execution.sched).
**Cross-track:** 46.1 (model-crate launch fields) shared with the nano-ros track — coordinate first.
**Reports:** `.superpowers/sdd/p46-w2-report.md` (46.1b/46.2), `p46-w3b-report.md`
(46.3b), `p46-w4-report.md` (46.4), `p46-w5-report.md` (46.5), `p46-w6-report.md`
(46.6, this docs sweep — closes the phase). 46.0/46.1/46.3 landed via
git history (`8b2f77f` for 46.1/#236) without a separate wave report.

## Overview

Makes the SystemModel the one complete artifact: it carries all
launch-derived info (+ optional contract + optional system config); both
play_launch and nano-ros read the full model and each derives its own
platform specifics (exec path, argv, bake). `record.json` retires. The user
perceives one kind of dump.

## Work items

- **46.0** ✅ Cross-track coordination: confirm the `NodeInstance` launch-field
  set + semantics with the nano-ros track (RFC-0050) before landing 46.1.
  Note like the sched-SSoT handshake.
- **46.1** ⛔ REVERTED 2026-07-31 — `<node machine=>` →
  `execution.deploy[fqn].host` (nano-ros #236). `machine=` is ROS 1
  roslaunch syntax, not ROS 2; both it and `Deploy.host` were removed. See
  `docs/superpowers/specs/2026-07-31-machine-attr-removal-design.md`.
- **46.1b** ✅ Shared launch fields (DECIDED 2026-07-20: option a): add
  `remaps`, `ros_args`, `respawn`/`respawn_delay`, launch-declared `env` to
  the shared `NodeInstance` per the all-launch-info principle (nano-ros
  ignores the ones it doesn't consume — no `deny_unknown_fields`). Additive,
  optional, backward-compat. **(model crate — cross-track.)**
- **46.2** ✅ Populate: `resolve`/model_builder fill the new fields from the
  parsed launch (they already flow through the LaunchDump the resolver reads).
- **46.3** ✅ Spawn-from-model: relocate cmdline assembly (`node_cmdline.rs`) to
  derive exec path (ament) + argv + injected env + materialized param files
  from the model at spawn. `replay` reads the model, not `record.json`. This
  is the load-bearing item — the Linux spawn path must reproduce today's
  behavior from the model inputs (regression-test against current
  `record.json`-driven spawns).
- **46.4** ✅ (Python-parser preservation FIRST — the acceptance gate): make
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
- **46.5** ✅ Retirement: `dump` converges onto `resolve`'s model
  (`--format model` default, `--format record` the deprecated
  parser-parity escape hatch); the `resolve` record companion + `meta.record`
  binding + `verify_model_record_binding` removed; `replay --input-file
  record.json` (no `--model`) kept as a one-release, warning compat path.
  **`file_data`/`variables`**: confirmed (not dropped, because there was
  nothing to drop) — the model was never built by embedding a `LaunchDump`,
  so those LaunchDump-only fields were never in the emitted artifact to
  begin with (`.superpowers/sdd/p46-w5-report.md` §5). Also fixed the
  stale-pip-install silent-degrade gap flagged in 46.4's CAVEAT: a
  pre-Phase-40.1 Python install now fails loud (missing `ScopeOrigin.path`)
  instead of silently emitting a structure-only artifact.
- **46.6** ✅ Docs: guide + design cross-refs; `dump`/`replay`/`resolve`
  one-artifact story. `CLAUDE.md`, `docs/guide/rt-scheduling.md`,
  `docs/guide/cli-interface.md`, `README.md`, this doc, and
  `docs/design/unified-system-model.md` updated; `src/play_launch/src/cli/options.rs`
  help text verified against the built binary and touched up (top-level +
  `replay` examples). See `.superpowers/sdd/p46-w6-report.md`.

## Order and dependencies

This is the order all sub-phases actually landed in:

46.0 (coordinate) → **46.1 (machine=/#236 — ships first, quick unblock for
nano-ros; ⛔ REVERTED 2026-07-31, see 46.1's entry above)** → 46.1b/46.2 (shared launch fields) → 46.3 (spawn-from-model —
largest/riskiest, regression-gated: Autoware + rt_workspace spawns must match
record.json-era) → **retirement 46.4/46.5 (non-additive, gated on 46.3's
regression suite)** → 46.6 docs. Through 46.3 both paths coexist (additive
model + record.json still written); retirement is the only non-additive step.

**Coordination (46.0):** note nano-ros (issue #236 + RFC-0050) with the
unified-model design + the incoming schema additions; confirm no field they
read is omitted. Same handshake as the sched-SSoT reconciliation.

**Retirement (46.4/46.5), only after 46.3 proved parity:** `dump` emits the
one model by default; the `meta.record` binding, `verify_model_record_binding`,
and the `resolve` record companion are removed. `record.json` itself is not
hard-removed — it stays as a deprecated, explicitly-opt-in compat/dev
surface (`dump --format record`, `replay --input-file` without `--model`)
for the parser-parity tooling and one release's rollback grace.
`file_data`/`variables` were confirmed never present in the emitted model
in the first place — nothing to drop.

## Out of scope

Storing platform-resolved outputs in the shared artifact; contract/sched layer
changes (Phases 44/45 stand); nano-ros's consumption of the new fields.
