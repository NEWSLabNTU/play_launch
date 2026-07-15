# Phase 40: Contract Shipping — Provider Sidecars + User Overlay

**Status:** ✅ Complete (40.1–40.7). Live-verified: 63 Autoware contracts load via the overlay channel, 0 errors.
**Design of record:** [docs/superpowers/specs/2026-07-15-contract-shipping-design.md](../superpowers/specs/2026-07-15-contract-shipping-design.md)
**Builds on:** the manifest loader (`ros/manifest_loader.rs`) and Phase 30 scope table.

## Overview

The manifest stays a separate YAML file; its **shipping** changes. Two
channels replace the single `--manifest-dir <dir>/<pkg>/<file>.yaml` lookup:

1. **Provider sidecar** — `<name>.contract.yaml` next to
   `<name>.launch.{xml,py,yaml}`, shipped and installed *by the package*.
2. **User overlay** — `<overlay-root>/<package>/launch/<name>.contract.yaml`
   (`--contracts <dir>`) — the general user-side source. Primary use today:
   supplying contracts for packages that ship none (Autoware); overriding a
   shipped sidecar is the same mechanism.

Precedence: overlay > provider > legacy `--manifest-dir` (transitional; retired in-phase).
Document-level replacement in v1; field-level merge deliberately deferred.
(Embedding contracts inside launch files was investigated —
[research note](../research/manifest-annex-in-launch-files.md) — feasible via
comment-fenced payloads, but rejected in favor of sidecars.)

## Work items

- ✅ **40.1** Record format: add `ScopeOrigin.path` (absolute launch-file path)
  in both parsers + `launch_dump.rs`; additive/serde-default, old records stay
  loadable. Cross-parser scope comparison updated.
- ✅ **40.2** Loader: three-step resolution (overlay → provider → legacy) in
  `manifest_loader`, recording the supplying channel per scope;
  `--contracts <dir>` + `--no-provider-contracts` flags; deprecation warning
  on `--manifest-dir`.
- ✅ **40.3** `check` surfaces channel + path per scope, so the effective
  contract source is always visible.
- ✅ **40.4** Tests: unit (stem rule, precedence, missing-path fallback);
  fixture migration to provider sidecars; overlay-override integration case
  (rt_workspace, Phase 39, ships the new layout from day one).
- ✅ **40.5** **Autoware manifest migration** — relayout `~/repos/autoware-contract`
  (github `NEWSLabNTU/autoware-contract`; 75 manifests, legacy
  `<pkg>/<stem>.yaml`) to `<pkg>/launch/<stem>.contract.yaml`; wire the
  Autoware fixture (justfile + gated tests) to
  `--contracts ~/repos/autoware-contract`. The overlay's flagship use:
  Autoware ships no contracts, so the user overlay is their only source.
- ✅ **40.6** **Retire the legacy channel** — after 40.5, remove
  `--manifest-dir`, the legacy resolution branch, and legacy-layout fixture
  remnants (the Autoware set was the last known user).
- ✅ **40.7** Docs: `launch-manifest.md` lookup section, RT guide file tree,
  CLAUDE.md, README.

## Order and dependencies

40.1 → 40.2 → (40.3, 40.4, 40.5) → 40.6 → 40.7. Phase 39's fixture should land either
after 40.2 or ship both layouts (legacy + sidecar) temporarily — preferred:
land 40.1–40.2 first, then Phase 39 uses only the new channels.

## Out of scope

Field-level overlay merging; embedded contracts; ament-index-based discovery;
any change to the manifest schema or the scheduling spec (`system.toml` stays
a single system-level file — one scheduling authority per deployment).
