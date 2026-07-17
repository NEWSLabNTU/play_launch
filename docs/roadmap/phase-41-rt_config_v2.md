# Phase 41: RT Config v2 — Derived Scheduling

**Status:** 🔄 41.1–41.5 done, 41.6 gated on nano-ros migration
**Design of record:** [docs/superpowers/specs/2026-07-16-rt-config-v2-design.md](../superpowers/specs/2026-07-16-rt-config-v2-design.md)
**Builds on:** Phase 38 (apply layer, unchanged), Phase 40 (contract channels, reused for platform files).

## Overview

Revises the three-part RT configuration in response to design feedback:
scheduling context becomes **derived** from launch+contract by a pluggable
`SchedMapper` (rate-monotonic, deadline-monotonic, manual), with the platform
file shrunk to platform facts + explicit overrides. The two files we control
unify on YAML; platform files become per-target
(`<stem>.system.<target>.yaml`) and ship through the same provider-sidecar +
user-overlay channels as contracts. `check --explain` presents the merged
logical view. Legacy `system.toml` keeps working via a dual-format bridge
until nano-ros migrates, then the old path is retired.

## Work items

- **41.1** ✅ Sched crate: new YAML platform-file schema (`target`, `mapper`,
  `resources`, `overrides`), `SchedMapper` trait + registry, built-ins
  (`manual`, `rate_monotonic`, `deadline_monotonic`), `MapperInput`
  extraction types, legacy-TOML bridge (`.toml` → `manual` mapper; extension
  selects parser).
- **41.2** ✅ play_launch integration: contract timing facts → `MapperInput`;
  pipeline derive → override → validate; conflict semantics (band clamp
  warn/strict, rate-vs-priority contradiction check rule); `--target` flag.
- **41.3** ✅ Shipping channels for platform files: provider sidecar + user
  overlay resolution per target; overlay-root discovery order
  (`--contracts` > `$PLAY_LAUNCH_CONTRACTS` > XDG > `/etc/play_launch/contracts`).
- **41.4** ✅ Tooling: `check --sched --explain` provenance view;
  `play_launch contract eject` subcommand.
- **41.5** ✅ Migration: rt_workspace fixture gains `bringup.system.posix.yaml`
  provider sidecar (mapper=rate_monotonic + one override) + a Zephyr stub
  proving per-target coexistence (parses; derive is posix-only today) + a
  user-overlay platform file tweaking the provider's band/pin (keeps
  `system.toml` as the living bridge test); `docs/guide/rt-scheduling.md`
  rewritten around the v2 model (quick start, mapper/platform-file/channels/
  `--explain`/`eject` sections, legacy bridge marked deprecated-but-supported);
  autoware-contract repo untouched (contracts unchanged).
- **41.6** Retirement (LAST, gated on nano-ros migration): drop legacy
  `system.toml` schema, remove bridge, final doc sweep.

## Follow-ups (from the Autoware v2 exercise, 2026-07-16)

Findings report: `.superpowers/sdd/autoware-v2-report.md`. Mechanics validated at
scale (119 entities / 63 manifests, 0.29 s derivation; channels/provenance/band
clamp all correct). Gaps are mapper sophistication + UX:

- **41.7 Criticality-aware mapper** (highest value): Autoware declares 11/13 RT
  nodes at exactly 10 Hz, so `rate_monotonic` orders them by its alphabetical
  tie-break; an untagged perception node outranked a `criticality: high` control
  node; 43/78 node pairs produced contradiction-warning noise from the tied
  rates. New built-in consuming the (already-plumbed, currently inert)
  `criticality` field; equal facts should collapse to equal priority, not
  alphabetical spread. CLOSED by Phase 42.6: the chain-aware
  mapper design subsumes it (its non-chain fallback layer IS criticality-RM/DM
  with tie-collapse); see
  [phase-42-autoware_system_model.md](./phase-42-autoware_system_model.md).
- **41.8 `--explain` polish**: dedupe repeated warning lines; show the full
  default bucket (or a count + `--explain-all`); warn when the requested
  `--target` resolves no platform file even without `--explain`.
- **41.9 Document the two-pass authoring workflow**: overrides are written
  against the derived plan (`--explain` first, then pin) — guide section.
- **autoware-contract**: branch `rt/v2-platform` (platform file + 6 criticality
  facts, committed, unpushed) — push after 41.7 makes the derived numbers
  meaningful.

## Order and dependencies

41.1 → 41.2 → (41.3, 41.4 parallel) → 41.5 → 41.6. 41.6 blocks on the
nano-ros track adopting the new schema; everything before it is
non-breaking (bridge keeps old configs working).

## Out of scope

External/WASM mappers, callback-level scheduling, automatic criticality
inference, the nano-ros migration itself.
