# Shared Scheduling Crate — Design

**Date:** 2026-07-01
**Status:** Approved (design), pending implementation plan
**Repos:** `play_launch` (Linux) + `nano-ros` (RTOS)

## Goal

One shared, portable scheduling spec consumed by both `play_launch` (Linux
RT) and `nano-ros` (RTOS-on-bare-metal). Users author **generic** timing /
grouping requirements once; **platform-specific** placement (concrete
priority / stack / core / sched_class) is supplied per target in the same
form on every platform. Portability is the hard requirement: no priority
numbers leak into the generic layer.

## Background

Two artifacts already cross both repos:

- **`record.json`** — output of `play_launch_parser`. The frozen launch graph
  + scope table (Phase 30). `nano-ros` consumes it by shelling out to the
  parser binary and walking the JSON by key (`node[]`, `container[]`,
  `load_node[]`, `scopes[]`, …). It is NOT linked (the parser embeds CPython
  via pyo3). Contract = field-name stability.
- **`ros-launch-manifest-types::Manifest`** — the per-scope YAML contract
  (endpoint rates, freshness `max_age_ms`, `drop`, QoS reliability/durability,
  path `max_latency_ms`). `nano-ros` links the `types` crate (path dep) but
  converts to `serde_json::Value` immediately.

Scheduling currently lives ONLY in `nano-ros`:
`packages/core/nros-orchestration-ir` defines `TierDef` / `TierRtosSpec` /
`CallbackGroupDecl` / `NodeOverride` + the `resolve_tiers` algorithm, lowered
into `nros-plan.json` (`PlanSchedContext` / `SchedClass` / `DeadlinePolicy`).
`play_launch` has no scheduling concept. RFC-0015 §11.1 already states the
intent to co-locate a shared schema crate next to `ros-launch-manifest-types`
("No third repo").

## Key modeling decisions

1. **Tier membership *is* the callback group.** "Callback group" (a
   `nano-ros`-introduced concept: an executor-sharing unit — several nodes
   sharing one executor on Linux, components under one kernel/executor on
   RTOS) is not authored separately. The set of nodes assigned to a tier IS
   the group. `nano-ros` v1 already pins a whole node to one tier
   (`NodeSpansTiers`), so node-level granularity is sufficient. This removes
   the separate callback-group authoring surface entirely for v1.

2. **Two orthogonal axes, kept separate:**
   - generic (portable) vs platform-specific
   - requirement (WHAT must hold) vs placement (HOW scheduled)

3. **Generic layer carries no numbers.** A tier's generic head names a
   scheduling *class* and timing requirements (deadline/period/budget). The
   concrete priority/stack/core/sched_class live only in per-platform
   sub-tables. Writing a priority number in the generic layer would break
   portability and is disallowed by schema (the generic head has no such
   fields).

4. **Binding = sparse selector assignment (UX approach A).** Users author
   sparse `[[assign]]` rules; node names and launch-scope paths are
   selectors. Scopes come from `record.json` and are shared verbatim by both
   repos, so a scope selector means the same thing on both. Automatic
   causality-based grouping is explicitly rejected as a binding mechanism
   (too unreliable on real Autoware graphs); it may return later as a
   NON-binding advisory tool (`suggest-tiers`).

## Crate

**Name:** `ros-launch-manifest-sched`
**Location:** `src/ros-launch-manifest/` workspace in `play_launch`, sibling
to `types` and `check`.
**Deps:** `serde`, `thiserror` only. Pure host code, no `no_std`, no
runtime deps — same constraints `nros-orchestration-ir` satisfies today.
**Distribution:** authored in `play_launch`; `nano-ros` vendors it via git
submodule (same as `ros-launch-manifest` already) and pins the commit in
`nros-sdk-index.toml` (same mechanism as `play_launch_parser`).

`board_path_for` (the `nano-ros` board-key → ZST-path map) is
`nano-ros`-specific and does NOT move; it stays in `nros-orchestration-ir`.

## Schema

On-disk format: **TOML** (matches `nano-ros`'s existing `system.toml`; the
crate is serde-based, so YAML/JSON also deserialize if a consumer prefers).
One system-level file.

```toml
# ===== GENERIC (portable — byte-identical across nano-ros and play_launch) =====
[tiers.control]            # generic head — NO priority numbers
class = "real_time"        # best_effort | real_time | time_triggered | interrupt
deadline_us = 50000        # generic deadline (≈ derived from callback frequency)
period_us   = 20000        # for periodic / time_triggered
budget_us   = 5000         # EDF/sporadic execution budget
deadline_policy = "warn"   # ignore | warn | skip | fault
spin_period_us  = 1000

[[assign]]                 # binding = tier membership (the "callback group")
tier  = "control"
nodes = ["ndt_localizer", "ekf_localizer"]   # explicit node selectors
[[assign]]
tier  = "perception"
scope = "perception/lidar"                    # launch-scope bulk selector
# any node matched by no rule → synthesized "default" tier

# ===== PLATFORM (same shape per target, values differ) =====
[tiers.control.posix]      # Linux RT — sits beside freertos/zephyr/threadx/nuttx
priority    = 80
sched_class = "SCHED_FIFO"
core        = 1
[tiers.control.freertos]
priority    = 12
stack_bytes = 8192
deadline_us = 40000        # optional per-platform deadline tighten
```

### Types

- `SystemSched` — top-level document: `tiers: BTreeMap<String, TierDef>`,
  `assign: Vec<AssignRule>`.
- `TierDef` — generic head (renamed/trimmed from `nros-orchestration-ir`):
  `class`, `deadline_us`, `period_us`, `budget_us`, `deadline_policy`,
  `spin_period_us`, plus per-platform sub-tables
  `posix`/`freertos`/`zephyr`/`threadx`/`nuttx: Option<TierPlatformSpec>`.
  No numeric priority field on the generic head (`serde(deny_unknown_fields)`
  enforces this — a stray `priority` at the head errors).
- `TierPlatformSpec` — platform placement: `priority: i64` (i64 admits
  Zephyr negative coop priorities), `stack_bytes: Option<u32>`,
  `core: Option<u32>`, `sched_class: Option<String>`,
  `preempt_threshold: Option<i64>` (ThreadX), `deadline_us: Option<u64>`
  (per-platform override).
- `AssignRule` — `tier: String`, `nodes: Vec<String>` (default empty),
  `scope: Option<String>`.
- `ResolvedTier` — generic policy fields + resolved platform numbers +
  `members: Vec<String>` (node names, sorted).
- `ResolvedTierTable` — `tiers` ordered highest-priority-first;
  `is_single_tier()` for the degenerate default case.
- `SchedError` (thiserror) — see Validation.

The `posix` key is the Linux-RT platform (reuse the existing `nano-ros`
key; no new `linux` key).

## Resolver

```
resolve(
  tiers: &BTreeMap<String, TierDef>,
  assigns: &[AssignRule],
  scope_table: &ScopeTable,     // from record.json
  nodes: &[NodeName],           // all nodes in the system
  target: &str,                 // "posix" | "freertos" | ...
) -> Result<ResolvedTierTable, SchedError>
```

Reuses the shape of `nros-orchestration-ir::resolve_tiers`. The difference is
how the node→tier map is built: instead of per-node callback-group
declarations from package metadata, it is computed from `[[assign]]`
selectors against the `record.json` scope table.

**Selector precedence:** explicit `nodes` match > `scope` subtree match >
synthesized `default` tier. A node matched by two rules of different tiers is
a conflict error.

Output: for each populated tier, look up the `target` platform sub-table,
carry the generic policy fields through, resolve the platform numbers, sort
members. Order tiers highest-`priority` first. Degenerate case (no assigns) →
single synthesized `default` tier (priority 0, no platform lookup needed).

## Validation (checks)

`SchedError` variants:

- `UnknownTier` — an `[[assign]]` names a tier with no `[tiers.<name>]`.
- `UnknownNodeSelector` — an `assign.nodes` entry matches no node in
  `record.json`.
- `UnknownScopeSelector` — an `assign.scope` matches no scope in the scope
  table.
- `NodeMatchedByMultipleTiers { node, tier_a, tier_b }` — replaces v1
  `NodeSpansTiers`; a node resolved to two different tiers.
- `MissingPlatformSpec { tier, target }` — populated tier lacks the
  `[tiers.<tier>.<target>]` sub-table for the resolve target.

## Consumers

Both link the crate; `nano-ros` also feeds it a scope table it already
obtains from the parser binary.

### nano-ros (RTOS)

`codegen-system` calls `resolve(..., target = "freertos" | "zephyr" | …)`,
bakes the `ResolvedTierTable` into `nros-plan.json`, and emits one
task/`Executor` per tier. The central `[[assign]]` table replaces the
scattered `[package.metadata.nros.node].callback_groups` +
`[[node_overrides]]` authoring. Same resolver, cleaner input source.

### play_launch (Linux) — validate now, apply later

- **Now:** `play_launch check` parses the system TOML + `record.json`,
  resolves for `target = "posix"`, runs all checks, reports diagnostics.
  **No change to how nodes are spawned.**
- **Phase 2 (documented, not built):** an apply-layer consumes the `posix`
  `ResolvedTier` and applies `sched_setscheduler` (SCHED_FIFO/RR from
  `sched_class`) + `sched_setaffinity` (`core`) + priority per spawned node
  process. Requires `CAP_SYS_NICE` / root. The crate exposes the resolved
  `posix` numbers as the hook point; `play_launch` fills the syscall layer
  later.

## Migration

- Move `TierDef` / `TierPlatformSpec` (from `TierRtosSpec`) / resolver out of
  `nros-orchestration-ir` into `ros-launch-manifest-sched`.
- `nros-orchestration-ir` re-exports the shared types and keeps
  `board_path_for` + any `nano-ros`-only glue.
- `nano-ros` switches authoring from per-package callback groups to the
  central `[[assign]]` table; the `resolve_tiers` call site swaps to the new
  `resolve` signature (scope-table-driven).
- `play_launch` gains the crate as a path dep and wires the `check` command.

## Non-goals (v1)

- No sub-node callback-group granularity (node-pinned tiers only).
- No automatic causality-based tier derivation in the binding path.
- No Linux apply-layer syscalls (phase 2).
- No manifest YAML schema changes — the per-scope contract is unchanged.

## Testing

- Resolver unit tests (port + extend `nros-orchestration-ir` tests):
  degenerate default, two-tier ordering, scope selector, node selector,
  precedence, all error variants, missing-platform-spec for each target.
- Cross-repo parity: a fixture `system.toml` resolves identically (generic
  fields) under `posix` in `play_launch` and under an RTOS target in
  `nano-ros`.
- `play_launch check` integration test over a fixture launch + system file.
