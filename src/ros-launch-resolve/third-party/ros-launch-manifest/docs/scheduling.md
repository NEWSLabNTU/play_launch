# Scheduling Specification Crate

**Crate:** `ros-launch-manifest-sched` (sibling to `ros-launch-manifest-types` in the `src/ros-launch-manifest/` workspace)

**Purpose:** Portable scheduling specification shared between `play_launch` (Linux RT) and `nano-ros` (RTOS). Authors specify generic tier definitions, deadline/period/budget requirements, and node-to-tier binding once; platform-specific placement (priority, scheduler class, core affinity, stack) is supplied per target in the same form on every platform.

**Key invariant:** The generic layer carries no priority numbers, ensuring byte-identical portability across platforms.

## Design Principles

1. **Two orthogonal axes, kept separate:**
   - Generic (portable) layer: tier class, deadline, period, budget, deadline policy, spin period
   - Platform-specific layer: priority, scheduler class, core affinity, stack, optional per-platform deadline override

2. **Tier membership is the callback group.** Node-level tier assignment replaces scattered per-package callback-group authoring. A tier's members (the set of nodes assigned to it via the `[[assign]]` table) form the execution group for that platform.

3. **Sparse selector binding (UX).** Users author minimal `[[assign]]` rules; selectors match by explicit node name or launch-scope path. A node matched by no rule synthesizes into a `default` tier.

4. **No priority leakage.** Schema enforcement via `serde(deny_unknown_fields)` on the generic `TierDef` head prevents stray priority numbers from breaking portability.

## TOML Schema

**File format:** one system-level TOML file with two sections:

```toml
# ===== GENERIC (portable — byte-identical across platforms) =====
[tiers.control]            # tier head: naming + generic requirements
class = "real_time"        # scheduling class: best_effort | real_time | time_triggered | interrupt
deadline_us = 50000        # generic deadline (derived from callback frequency)
period_us   = 20000        # for periodic / time_triggered
budget_us   = 5000         # EDF/sporadic execution budget
deadline_policy = "warn"   # deadline breach handling: ignore | warn | skip | fault
spin_period_us  = 1000     # executor spin period

[[assign]]                 # binding = tier membership
tier  = "control"
nodes = ["ndt_localizer", "ekf_localizer"]   # explicit node selectors
[[assign]]
tier  = "perception"
scope = "/perception/lidar"                  # launch-scope subtree selector
# any node matched by no rule → synthesized "default" tier

# ===== PLATFORM (same shape per target, values differ) =====
[tiers.control.posix]      # Linux RT (shared with nano-ros)
priority    = 80
sched_class = "SCHED_FIFO"
core        = 1
[tiers.control.freertos]   # RTOS example
priority    = 12
stack_bytes = 8192
deadline_us = 40000        # optional per-platform deadline tighten
```

### Type Reference

- **`SystemSched`** — top-level document: `tiers: BTreeMap<String, TierDef>`, `assign: Vec<AssignRule>`

- **`TierDef`** — `[tiers.<name>]` generic head:
  - `class: Option<String>` — scheduling class
  - `deadline_us: Option<u64>` — generic deadline (microseconds)
  - `period_us: Option<u64>` — callback period (microseconds)
  - `budget_us: Option<u64>` — execution budget (microseconds)
  - `deadline_policy: Option<String>` — deadline breach policy
  - `spin_period_us: Option<u64>` — executor spin period (microseconds)
  - Per-platform sub-tables: `posix`, `freertos`, `zephyr`, `threadx`, `nuttx` (each `Option<TierPlatformSpec>`; `native` is accepted as an alias for `posix`)

- **`TierPlatformSpec`** — `[tiers.<name>.<target>]` concrete placement:
  - `priority: i64` — OS/RTOS priority (i64 to admit Zephyr negative coop priorities)
  - `stack_bytes: Option<u32>` — task stack size
  - `core: Option<u32>` — CPU core index (SMP pinning; `None` = unpinned)
  - `sched_class: Option<String>` — POSIX scheduler class (e.g. `"SCHED_FIFO"`, `"SCHED_RR"`)
  - `preempt_threshold: Option<i64>` — ThreadX preemption threshold
  - `deadline_us: Option<u64>` — per-platform `deadline_us` overrides the generic head (not validated to be smaller)

- **`AssignRule`** — `[[assign]]` sparse binding:
  - `tier: String` — target tier name
  - `nodes: Vec<String>` — explicit node selectors (default empty)
  - `scope: Option<String>` — launch-scope subtree selector

- **`ResolvedTier`** — one tier after resolution for a target:
  - Generic policy fields: `class`, `period_us`, `budget_us`, `deadline_us` (effective value), `deadline_policy`, `spin_period_us`
  - Platform placement: `priority`, `core`, `sched_class`, `stack_bytes`, `preempt_threshold`
  - `members: Vec<String>` — sorted list of assigned node names

- **`ResolvedTierTable`** — ordered tier table for one platform:
  - `tiers: Vec<ResolvedTier>` — sorted highest-priority-first
  - `is_single_tier()` — true when system collapsed to single `default` tier

## Resolver

### Signature

```rust
pub fn resolve(
    tiers: &BTreeMap<String, TierDef>,
    assigns: &[AssignRule],
    nodes: &[SchedNode],
    target: &str,  // "posix" | "freertos" | "zephyr" | etc.
) -> Result<ResolvedTierTable, SchedError>
```

Where `SchedNode` carries:
- `name: String` — fully-qualified node name (e.g. `/perception/lidar/ndt_localizer`)
- `scope: String` — node's namespace / scope path (e.g. `/perception/lidar`)

### Selector Precedence

1. **Explicit node selectors** — highest precedence. Match by full FQN or bare name (last path segment).
2. **Scope subtree selectors** — middle precedence. Match by exact scope or descendant (nodes under the scope path).
3. **Synthesized `default` tier** — lowest precedence. Unmatched nodes fall here.

A node claimed by two rules at the **same precedence level** (node-vs-node or scope-vs-scope) for different tiers is a `NodeMatchedByMultipleTiers` error. An explicit node rule always wins over a scope rule for the same node — no error is raised.

### Effective Deadline Rule

For each resolved tier, the effective deadline is:
- Platform override (`spec.deadline_us`), if present
- Otherwise, generic head (`def.deadline_us`)

### Output

- Tiers ordered highest-`priority` first
- Degenerate case (no `[[assign]]` rules): single synthesized `default` tier with priority 0 and all nodes as members (no platform lookup needed)
- All other cases: platform sub-table lookup required for each named tier

## Validation (Error Handling)

`SchedError` enum variants:

- **`Parse`** — TOML parse failure
- **`UnknownTier { tier }`** — an `[[assign]]` references a tier with no `[tiers.<tier>]` definition
- **`UnknownNodeSelector { selector }`** — an explicit node selector matches no node in the system
- **`UnknownScopeSelector { selector }`** — a scope selector matches no node's scope
- **`NodeMatchedByMultipleTiers { node, tier_a, tier_b }`** — a node resolved to two different tiers (conflict)
- **`MissingPlatformSpec { tier, target }`** — a populated tier lacks the required `[tiers.<tier>.<target>]` sub-table for the resolve target

## Consumers

### play_launch (Linux RT) — Validate Now, Apply Later

**Now (v1, implemented):** `play_launch check --sched <system.toml>` parses the scheduling spec, loads `record.json`, extracts node names and scope paths, resolves for `target = "posix"`, runs all checks, and reports diagnostics. **No change to how nodes are spawned.**

**Phase 2 (documented, not implemented):** an apply-layer consumes the resolved `posix` `ResolvedTierTable` and applies:
- `sched_setscheduler` (SCHED_FIFO/RR from `sched_class`, priority from `priority`)
- `sched_setaffinity` (core from `core`)

This requires `CAP_SYS_NICE` or root. The crate exposes the resolved `posix` numbers as the hook point; `play_launch` fills the syscall layer later.

### nano-ros (RTOS) — Full Apply

`codegen-system` calls `resolve(..., target = "freertos" | "zephyr" | ...)`, bakes the `ResolvedTierTable` into the system plan, and emits one task/executor per tier with the resolved platform numbers. The central `[[assign]]` table replaces scattered per-package callback-group authoring.

#### Cross-repo design agreement (2026-07-20) — input model; execution modeling per consumer

**Supersedes the 2026-07-19 "SSoT structure" note** (maintainer decision). The scheduling-SSoT direction (embedding a resolved sched plan into the model) **landed** (`model.execution.sched` / `ExecutionSched`, `78f637d`) but is being **reverted**. Settled split:

- **play_launch is a parser.** It gathers **all input** into the SystemModel — launch structure, contracts, system config, and the integrator's **declared** `deploy`/`tiers`/`bindings`. The model is the complete **input**; it carries **no resolved sched plan** (`model.execution.sched` is removed). Phase-46 (unified **input** model — `record.json`/LaunchDump merge) is exactly this parser job and continues.
- **Causality + execution modeling is the consumer's job; the *algorithm* is shared, not the output.** This crate (`ros-launch-manifest-sched`) is already the shared, pure scheduling crate (no parser/`types`/`check` deps — the 2026-07-02 shared-crate plan). The arrangement (2026-07-20):
  - **Split `chain_aware_mapper`** into (a) a **platform-agnostic core** — feasibility + clock-segmentation + chain/segment **ranking** (a priorityless ordered/segmented structure), and (b) the **Linux realizer** — `rt_priority_band` compression → `ResolvedTierTable` (PiCAS priorities). Both live in this crate; (b) is `posix`-tagged.
  - **Derivation is per-consumer**, sharing the `MapperInput` type: play_launch derives `LaunchDump`+manifests → `MapperInput` (`sched_derive`, play_launch-side, parser-coupled); nano-ros derives the SystemModel → `MapperInput`. Each then calls the agnostic core + its own realizer (play_launch → the Linux realizer; nano-ros → its RTOS realizer: EDF / preemption-threshold / sporadic / affinity).
- **Runtime E2E monitoring stays stamp-based — no chain-id.** `age = now − header.stamp` at the sink (`sub_endpoints.max_age_ms`), per `launch-manifest.md` §Timestamps.

**Rework:** (1) ~~revert `model.execution.sched`/`ExecutionSched`~~ **DONE** (`f090400`); (2) split `chain_aware_mapper` into agnostic core + `posix` realizer, exposing a priorityless ranked/segmented output the RTOS realizer also consumes. Cross-refs nano-ros RFC-0050 §"Input model; causality + execution modeling per consumer", RFC-0052 §"nano-ros execution modeling", and play_launch `docs/superpowers/specs/2026-07-01-shared-scheduling-crate-design.md`.

## Distribution & Cross-Repo Sharing

- **Authored in:** `play_launch` (`src/ros-launch-manifest/sched/`)
- **Dependencies:** `serde`, `thiserror`, `toml`. Pure host code, no `no_std`, no runtime deps.
- **Vendoring:** `nano-ros` vendors it via git submodule (same mechanism as `ros-launch-manifest`) and pins the commit in `nros-sdk-index.toml`.
- **Portability:** Generic layer byte-identical across all platforms. Platform-specific placement lives in per-target sub-tables, so the same system TOML (generic fields) resolves identically on all platforms.

## Usage Example

```toml
[tiers.control]
class = "real_time"
deadline_us = 50000
period_us = 20000

[tiers.control.posix]
priority = 80
sched_class = "SCHED_FIFO"
core = 1

[tiers.perception]
class = "real_time"
deadline_us = 100000

[tiers.perception.posix]
priority = 60
core = 2

[[assign]]
tier = "control"
nodes = ["ndt_localizer", "ekf_localizer"]

[[assign]]
tier = "perception"
scope = "/perception/lidar"
```

On `posix`, this resolves to:
- **control tier:** priority 80, SCHED_FIFO, core 1, members `[ndt_localizer, ekf_localizer]`
- **perception tier:** priority 60, no scheduler class/core (inherited from platform sub-table if specified), members include all nodes under `/perception/lidar`
- **default tier:** priority 0, all other nodes

## v2 (derived) model — Phase 41.1

**Status:** schema + mapper + bridge landed in the crate (this section); play_launch
integration (contract → `MapperInput`, `--sched-apply` pipeline, `--target`
flag) is wave 2 (Phase 41.2). The v1 TOML schema documented above keeps
working unchanged via the bridge — this is purely additive.

**Motivation:** the v1 schema is hand-written (tiers + `[[assign]]`), but the
runtime can derive most of the scheduling context from launch + contract
timing facts (rate, deadline, criticality). v2 makes that derivation a named,
pluggable **mapper** and shrinks the platform file to platform facts +
explicit overrides. See `docs/superpowers/specs/2026-07-16-rt-config-v2-design.md`
for the full design (this section summarizes the parts the sched crate
implements).

### Platform-file schema (YAML)

One file names one target (`target:` header). `posix` (Linux RT) is typed
concretely; any other target is kept as a raw passthrough (`nano-ros`
validates its own target vocabularies — see below).

```yaml
target: posix                # required, non-empty; validated by parse_platform_file_yaml
mapper: rate_monotonic       # SchedMapper name, looked up in a MapperRegistry
resources:                   # platform facts, typed per target
  rt_priority_band: { min: 10, max: 40 }
  isolated_cpus: [0]
overrides:                   # explicit per-node pins; beat derived values, always
  control_node: { priority: 20, core: 0 }
```

- **`posix` `resources`** (`PosixResources`): `rt_priority_band: Option<PriorityBand>`
  (`{ min: i64, max: i64 }`), `isolated_cpus: Vec<u32>` (advisory; not enforced
  by this crate). For `target: posix` the band is validated at parse time:
  `min <= max` AND entirely inside Linux's legal `SCHED_FIFO`/`SCHED_RR`
  priority range `1..=99` (`POSIX_RT_PRIORITY_MIN`/`MAX`), else
  `PlatformError::InvalidPriorityBand`. Unknown targets are never
  range-validated (raw passthrough; e.g. Zephyr's negative coop priorities).
- **`posix` `overrides.<node>`** (`PosixOverride`): `priority: Option<i64>`,
  `core: Option<u32>`, `sched_class: Option<String>`.
- **Unknown target** (e.g. `zephyr`, `freertos`): `resources`/`overrides` parse
  as raw `serde_yaml_ng::Value` (`PlatformResources::Raw` /
  `PlatformOverrideEntry::Raw`) — untyped, untouched, passed through for the
  consumer (nano-ros) to validate against its own per-target vocabulary.
- Entry point: `parse_platform_file(path) -> Result<PlatformFile, PlatformError>`
  dispatches on extension — `.yaml`/`.yml` → this schema
  (`parse_platform_file_yaml`), `.toml` → the legacy bridge (below).

### `SchedMapper` trait + registry

```rust
pub trait SchedMapper {
    fn name(&self) -> &str;
    fn map(&self, input: &MapperInput, facts: &PlatformFacts) -> Result<SchedPlan, MapError>;
}
```

- **`MapperInput`** — dependency-free per-node facts extracted by the caller
  (play_launch, wave 2) from launch + contract:
  `nodes: Vec<MapperNode>` where `MapperNode { name, scope, rate_hz: Option<f64>,
  deadline_us: Option<u64>, criticality: Option<Criticality>, path_budget_ms: Option<f64> }`;
  plus `legacy: Option<SystemSched>`, populated only by the `.toml` bridge for
  the `manual` mapper. No graph edges yet (YAGNI — a future field like
  `depends_on` can be added additively).
- **`PlatformFacts`** — type alias for `PlatformResources` (the platform
  file's parsed `resources`, per the selected target).
- **`SchedPlan`** — type alias for the crate's existing `ResolvedTierTable`
  (deliberately reused, not a new parallel type: it is already "ordered
  priority/core/sched_class placement grouped by tier, with member node
  names", which is exactly what every mapper produces). Built-in mappers that
  give each node its own priority represent that as a one-member-per-tier
  `ResolvedTier` (tier name = node name); nodes with no facts collapse into
  the existing `DEFAULT_TIER` (priority 0, no `sched_class`, i.e. non-RT) —
  the same shape an unmatched node gets in the v1 resolver.
- **`MapperRegistry`** — `register(Box<dyn SchedMapper>)`, `get(name) -> Option<&dyn SchedMapper>`,
  `with_builtins()` (pre-registers `manual`, `rate_monotonic`,
  `deadline_monotonic`). Consumers (play_launch, nano-ros) register
  additional mappers at link time; no dynamic loading.

### Built-in mappers

- **`manual`** — the legacy semantics. Requires `input.legacy` (only
  populated by the `.toml` bridge); delegates to `resolve()` against the
  legacy tiers + `[[assign]]` for `target = "posix"`, reproducing v1 output
  exactly. Ignores `facts`.
- **`rate_monotonic`** — higher `rate_hz` → higher priority, linearly spread
  across `resources.rt_priority_band` (rank 0 → `band.max`, last rank →
  `band.min`). Deterministic: ranked by rate descending, ties broken by node
  name ascending. Nodes with no `rate_hz` fall into the non-RT default tier.
  Errors (`MapError::MissingPriorityBand`) if the target isn't `posix` or the
  band is absent; errors (`MapError::InvalidPriorityBand`) if `min > max` or
  the band leaves the POSIX `1..=99` RT range (same `validate_posix()` rule
  as the parser — guards facts constructed programmatically, not via a file).
- **`deadline_monotonic`** — same shape, ranked by `deadline_us` ascending
  (shorter deadline → higher priority); ties broken by name ascending; no
  `deadline_us` → non-RT default.

Applying `overrides` on top of a derived plan ("override beats derived,
always") is **not** part of the trait — it's mapper-independent,
platform-file-scoped logic that the caller (play_launch, wave 2's pipeline)
applies after `map()` returns.

### Legacy `.toml` bridge

`parse_legacy_toml(input: &str) -> Result<PlatformFile, SchedError>`
(`bridge.rs`) parses a v1 `system.toml` document and produces a
`PlatformFile` with `target: "posix"`, `mapper: "manual"`, empty
`resources`/`overrides`, and the parsed `SystemSched` carried in
`PlatformFile::legacy`. Reachable through `parse_platform_file`'s `.toml`
extension dispatch. **Equivalence is tested**: fixture TOML → bridge →
`manual` mapper output is asserted equal to calling `resolve()` directly on
the same fixture (`bridge::tests::bridge_then_manual_mapper_matches_direct_resolve`).

### Validation helpers (design §6 conflict semantics)

Pure functions over an already-derived `SchedPlan`; the caller decides
warn-vs-strict and how to present the result (`--sched-apply`, `--explain` —
wave 2/4):

- **`band_violations(plan, band) -> Vec<BandViolation>`** — every non-default
  tier whose priority falls outside `[band.min, band.max]`.
- **`rate_priority_contradictions(input, plan) -> Vec<Contradiction>`** /
  **`deadline_priority_contradictions(input, plan) -> Vec<Contradiction>`** —
  pairwise scan: a node with a strictly higher rate (or strictly shorter
  deadline) than another must not end up at a strictly lower final priority;
  a violation is reported as a `Contradiction { node_a, node_b, kind }` (the
  built-in rate/deadline mappers' own output never triggers this by
  construction — only hand-authored overrides or the `manual` mapper's
  independent tiers can).

### Deprecation note

The v1 TOML schema (`SystemSched`/`TierDef`/`AssignRule`/`resolve()`,
documented above) is not deprecated by this wave — it keeps parsing and
resolving exactly as before, and is the sole implementation of the `manual`
mapper via the bridge. Per the design doc, it is scheduled for retirement
(Phase 41.6) only after `nano-ros` migrates to the v2 schema; there is no
flag day.

## Design of Record

See `docs/superpowers/specs/2026-07-01-shared-scheduling-crate-design.md` (the original v1 design document) and `docs/superpowers/specs/2026-07-16-rt-config-v2-design.md` (the v2/derived-scheduling design of record, Phase 41) for detailed rationale.
