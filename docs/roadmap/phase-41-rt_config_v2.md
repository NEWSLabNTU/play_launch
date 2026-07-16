# Phase 41: RT Config v2 — Derived Scheduling

**Status:** 📋 Planned
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

- **41.1** Sched crate: new YAML platform-file schema (`target`, `mapper`,
  `resources`, `overrides`), `SchedMapper` trait + registry, built-ins
  (`manual`, `rate_monotonic`, `deadline_monotonic`), `MapperInput`
  extraction types, legacy-TOML bridge (`.toml` → `manual` mapper; extension
  selects parser).
- **41.2** play_launch integration: contract timing facts → `MapperInput`;
  pipeline derive → override → validate; conflict semantics (band clamp
  warn/strict, rate-vs-priority contradiction check rule); `--target` flag.
- **41.3** Shipping channels for platform files: provider sidecar + user
  overlay resolution per target; overlay-root discovery order
  (`--contracts` > `$PLAY_LAUNCH_CONTRACTS` > XDG > `/etc/play_launch/contracts`).
- **41.4** Tooling: `check --sched --explain` provenance view;
  `play_launch contract eject` subcommand.
- **41.5** Migration: rt_workspace fixture gains `bringup.system.posix.yaml`
  sidecar + Zephyr stub + overlay example (keeps `system.toml` as living
  bridge test); docs guide rewrite; autoware-contract repo untouched
  (contracts unchanged).
- **41.6** Retirement (LAST, gated on nano-ros migration): drop legacy
  `system.toml` schema, remove bridge, final doc sweep.

## Order and dependencies

41.1 → 41.2 → (41.3, 41.4 parallel) → 41.5 → 41.6. 41.6 blocks on the
nano-ros track adopting the new schema; everything before it is
non-breaking (bridge keeps old configs working).

## Out of scope

External/WASM mappers, callback-level scheduling, automatic criticality
inference, the nano-ros migration itself.
