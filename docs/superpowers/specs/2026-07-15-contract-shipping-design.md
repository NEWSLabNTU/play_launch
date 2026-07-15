# Contract Shipping — Provider Sidecars + User Overlay

**Date:** 2026-07-15
**Status:** Approved (design), pending implementation
**Repo:** `play_launch` (loader) + `src/ros-launch-manifest` (docs; schema unchanged)

## Decision

The manifest stays a **separate YAML file** (embedding it into launch files —
see `docs/research/manifest-annex-in-launch-files.md` — was investigated and
is *feasible* via comment-fenced payloads, but is **not pursued**). What
changes is **how manifests are shipped and found**. Two channels replace the
single `--manifest-dir <dir>/<pkg>/<file>.yaml` lookup:

1. **Provider sidecar** — the package maintainer ships the contract next to
   the launch file, named by the launch file's stem:

   ```
   share/my_robot/launch/bringup.launch.xml
   share/my_robot/launch/bringup.contract.yaml     ← installed with the package
   ```

   Rule: for `<name>.launch.{xml,py,yaml}` the sidecar is
   `<name>.contract.yaml`, in the same directory. It travels with the package
   through colcon install / release, so a package's contracts version together
   with its launch files.

2. **User overlay** — a *general* user-side contract source, in an overlay
   tree keyed by package + launch path:

   ```
   <overlay-root>/<package>/launch/<name>.contract.yaml
   ```

   e.g. `contracts/autoware_launch/launch/planning_simulator.contract.yaml`.
   The overlay root is given on the CLI (`--contracts <dir>`, working name).

   The overlay is not defined by one purpose — it is simply the user's layer
   in the resolution order. Its **primary use today is supplying contracts
   for packages that ship none** (virtually every third-party package,
   Autoware included, has no `.contract.yaml`). Overriding a provider's
   shipped contract is the same mechanism and needs nothing extra.

**Precedence: overlay > provider**, at whole-document granularity (v1): if an
overlay file exists for a scope, it *replaces* the provider sidecar for that
scope. Document-level replacement is trivially predictable; field-level
merging (overlay extends/patches provider) is a possible v2, deliberately not
in v1 — partial merges reopen every cross-scope consistency question the
checker already handles between *scopes*, and would now have to handle
*within* one scope.

## Why two channels

- The provider channel puts the contract where it belongs: authored by whoever
  authored the launch file, reviewed and released together. Today's
  `--manifest-dir` forces the *user* to maintain a parallel tree for other
  people's packages.
- The overlay channel keeps the user in control, and covers the world as it
  is: today essentially no upstream package ships contracts, so the overlay is
  how contracts exist at all for third-party trees (Autoware being the
  driving example). It equally covers deployment-specific overrides (rates,
  freshness budgets) and read-only installed trees. The mirror layout
  (`<pkg>/launch/<name>.contract.yaml`) makes the correspondence to the
  launch file obvious.

## Resolution (per file scope)

For every file scope in `record.json` (pkg, launch-file name):

```
1. overlay:  <overlay-root>/<pkg>/launch/<stem>.contract.yaml   (if --contracts given)
2. provider: <launch-file-dir>/<stem>.contract.yaml
3. legacy:   <manifest-dir>/<pkg>/<file>.yaml                   (if --manifest-dir given; deprecated)
```

First hit wins; the loader records which channel supplied each scope (shown in
`check` output, so "why is this contract in effect" is always answerable).
`<stem>` = launch file basename minus its `.launch.*` suffix
(`bringup.launch.xml` → `bringup`). Scopes with no contract in any channel are
skipped, as today.

### Record-format prerequisite

Provider lookup needs the launch file's **directory**, which `record.json`
does not carry today (`ScopeOrigin { pkg, file }` stores the basename only —
and includes may reference arbitrary paths outside a package's `launch/`
dir). Extend `ScopeOrigin` with the resolved absolute path:

```rust
pub struct ScopeOrigin {
    pub pkg: Option<String>,
    pub file: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,     // absolute path of the launch file (new)
}
```

Both parsers (Rust and Python dump) populate it; `#[serde(default)]` keeps old
records loadable (provider lookup silently unavailable for them). This is an
additive record-format change — nano-ros reads `record.json` by key and is
unaffected.

## CLI

- `--contracts <dir>` — overlay root (channel 1).
- Provider sidecars (channel 2) need no flag: on by default whenever a
  contract exists next to a launch file. An escape hatch
  `--no-provider-contracts` covers the "installed package ships a broken
  contract" case.
- `--manifest-dir <dir>` — kept working, warns deprecated, removed later.
- `check` gains a `--explain-contracts` style summary (channel + path per
  scope) — exact flag shape decided at implementation.

## Interaction with the scheduling spec

None. `system.toml` remains a single system-level file (`--sched`); contracts
are per-launch-file. The two-channel idea does not apply to scheduling — a
deployment has one scheduling authority, not per-package fragments.

## Migration

1. Loader gains the three-step resolution (legacy channel keeps existing
   setups working unchanged).
2. Fixtures/tests move to provider sidecars (`tests/fixtures/*/launch/*.contract.yaml`);
   the rt_workspace fixture (Phase 39) ships this layout from day one and adds
   an overlay case to its integration test.
3. **Autoware manifests migrate in this phase**: the Autoware manifest set
   (today a `--manifest-dir` tree in `<dir>/<pkg>/<file>.yaml` layout) moves to
   the overlay layout `<overlay>/<pkg>/launch/<name>.contract.yaml` and is
   exercised through `--contracts` by the Autoware fixture's justfile/tests.
   This is the overlay's flagship case — Autoware packages ship no contracts,
   so the user overlay is their only source.
4. Docs: `src/ros-launch-manifest/docs/launch-manifest.md` lookup section,
   `docs/guide/rt-scheduling.md` file-tree, CLAUDE.md.
5. Deprecation of `--manifest-dir` announced in README; removal in a later
   release.

## Non-goals (v1)

- Field-level overlay merging (document-level replacement only).
- Embedding contracts in launch files (rejected; research note stands).
- Contract discovery via ament resource index (directory conventions are
  sufficient and transparent; revisit only if the provider convention proves
  ambiguous).
