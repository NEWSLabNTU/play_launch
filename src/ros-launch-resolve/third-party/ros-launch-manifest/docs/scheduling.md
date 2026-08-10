# Scheduling Specification Crate

**Crate:** `ros-launch-manifest-sched` (in the `src/ros-launch-manifest/`
workspace, alongside `types`, `check`, and `model`).

**Purpose:** portable scheduling specification and derivation shared
between `play_launch` (Linux RT, via `ros-launch-resolve`) and `nano-ros`
(RTOS targets). The integrator ships one small **platform file** per
target; per-node scheduling is **derived** from launch + contract timing
facts by a named, pluggable **mapper**, with explicit per-node
**overrides** that always beat derived values.

**Key invariant:** everything platform-agnostic (timing facts, chain
structure, priority *ordering*) is kept separate from platform
realization (OS priority numbers, scheduler classes, cores). The shared
part is the **algorithm**, not the output — each consumer runs its own
realizer over the same ranking core.

Two schemas coexist:

- **v2 platform file** (`<stem>.system.<target>.yaml`) — the current
  default: mapper name + platform facts + overrides. This is what ships
  through the contract discovery channels.
- **v1 `system.toml`** (hand-written tiers + `[[assign]]`) — legacy,
  still fully supported via a bridge to the `manual` mapper. Explicit
  `--sched <path>.toml` only; never discovered through channels.

## Where scheduling facts come from

The mapper input is derived from the contract files
(`<stem>.contract.yaml`, see [launch-manifest.md](launch-manifest.md))
joined with the launch tree:

| Fact | Contract source |
|------|-----------------|
| `rate_hz` | max over topic-level `rate_hz` on published topics and the node's own `pub.<ep>.min_rate_hz` |
| `deadline_us` | min `max_latency_ms` over the node's declared `paths` (×1000) |
| `criticality` | `nodes.<name>.criticality` (`high`/`medium`/`low`, advisory string on the node) |
| per-path facts | each path's `effective_trigger` (timer/input/once/spontaneous/unclassified), `max_latency_ms`, inputs/outputs |
| chains | `chains:` declarations, resolved cross-scope into segment/boundary structure |

This derivation is **per-consumer** (it needs a launch tree or a
SystemModel, which this crate deliberately does not depend on):
`ros-launch-resolve` derives from the parsed launch dump + manifests
(`sched_derive.rs` — the fact table above describes *its* rules);
nano-ros derives from the resolved SystemModel
(`nros-orchestration-ir::mapper_input`). Both produce the same
`MapperInput` *type*, but fill it differently — nano-ros currently
populates only `name`/`scope`/`criticality`/`paths`, with its own
timer-trigger derivation, leaving `rate_hz`/`deadline_us` unset.

## v2 Platform File

One file names one target (`target:` header). `posix` (Linux RT, with
`native` accepted as an alias) is typed concretely; any other target
parses as raw passthrough — the consumer (nano-ros) validates its own
per-target vocabulary.

```yaml
target: posix                # required, non-empty
mapper: chain_aware          # SchedMapper name, looked up in MapperRegistry
reservations: off            # off (default) | required — POLICY, so not under resources
resources:                   # platform facts, typed per target
  rt_priority_band: { min: 10, max: 40 }
  isolated_cpus: [0]
  rr_timeslice_us: 100000    # the host's GLOBAL SCHED_RR slice; absent = unknown
overrides:                   # explicit per-node pins; beat derived values, always
  control_node: { priority: 20, core: 0 }
  obstacle_detector: { budget_us: 8000, uclamp_max: 800 }
  telemetry_logger: { sched_class: SCHED_BATCH, nice: 10 }
  planner: { cpus: [4, 5] }
```

- **`posix` `resources`** (`PosixResources`, `deny_unknown_fields`):
  `rt_priority_band: Option<PriorityBand>` (`{ min: i64, max: i64 }`,
  inclusive), `isolated_cpus: Vec<u32>` (advisory; not enforced by this
  crate). For `target: posix` the band is validated at parse time:
  `min <= max` and entirely inside Linux's legal `SCHED_FIFO`/`SCHED_RR`
  range `1..=99` (`POSIX_RT_PRIORITY_MIN`/`MAX`), else
  `PlatformError::InvalidPriorityBand`.
- **`posix` `overrides.<node>`** (`PosixOverride`): `priority:
  Option<i64>`, `core: Option<u32>`, `cpus: Vec<u32>`, `sched_class:
  Option<String>`, `nice: Option<i32>`, `uclamp_min`/`uclamp_max:
  Option<u32>`, `budget_us: Option<u64>`.
  Keys use the same selector vocabulary as v1 `[[assign]].nodes`: full
  FQN or bare last segment. Parsing lives here; *applying* overrides is
  the caller's job (see [Consumers](#consumers)).

  Every combination rule is checked at parse time
  (`validate_posix_override`), because an override that reaches the
  syscall layer comes back as an `EINVAL` with no node name attached:
  `sched_class` must be one of the six real policies (**unknown is an
  error** — it used to become `SCHED_OTHER` silently, so a typo dropped a
  node out of real-time); `priority` only with `SCHED_FIFO`/`SCHED_RR`
  and `nice` only with `SCHED_OTHER`/`SCHED_BATCH`; `SCHED_DEADLINE`
  takes no CPU pin at all; `core` and `cpus` are mutually exclusive;
  `nice ∈ -20..=19`; `uclamp ∈ 0..=1024` with `min <= max`.

  `budget_us` is the **declared execution cost** — the only legitimate
  source for a `SCHED_DEADLINE` reservation's runtime. It is not a proven
  WCET; it is a declared high-percentile observed cost used *as* an upper
  bound. `budget_us: 0` is rejected: absent and zero are different
  answers, and a declared zero would silently mean "free".

- **`reservations`** (`ReservationMode`, top level beside `mapper:`):
  `off` (default) or `required`. Deliberately not inside `resources:` —
  that holds facts about the machine, and whether to reserve is a policy
  choice. Opt-in because reservations are all-or-nothing within a band: a
  reserved node preempts every fixed-priority thread regardless of
  priority, so a band holding both loses the ordering the mapper
  computed. Without the switch, adding one `budget_us` would turn that
  rule into a hard error nobody asked for.

- **`rr_timeslice_us`**: the host's `SCHED_RR` slice. A platform *fact*,
  which is why it sits in `resources:` — on Linux the slice is a global
  sysctl (`/proc/sys/kernel/sched_rr_timeslice_ms`, default **100 ms**),
  not a per-task value, so `TierPlatformSpec::time_slice_us` cannot
  express it. The `chain_aware` mapper needs it to decide whether
  `SCHED_RR` is worth deriving for a priority tie at all. Absent means
  unknown, never "assume the default".
- **Unknown target** (`zephyr`, `freertos`, …): `resources`/`overrides`
  parse as raw `serde_yaml_ng::Value` (`PlatformResources::Raw` /
  `PlatformOverrideEntry::Raw`) — untyped, never range-validated (e.g.
  Zephyr's negative cooperative priorities), passed through for the
  consumer to validate.
- Entry point: `parse_platform_file(path) -> Result<PlatformFile,
  PlatformError>` dispatches on extension — `.yaml`/`.yml` → this schema
  (`parse_platform_file_yaml`), `.toml` → the
  [legacy bridge](#legacy-toml-bridge).

```rust
pub struct PlatformFile {
    pub target: String,
    pub mapper: String,
    pub reservations: ReservationMode,
    pub resources: PlatformResources,
    pub overrides: BTreeMap<String, PlatformOverrideEntry>,
    pub legacy: Option<SystemSched>,   // Some only via the .toml bridge
}
```

## `SchedMapper` Trait and Registry

```rust
pub trait SchedMapper {
    fn name(&self) -> &str;
    fn map(&self, input: &MapperInput, facts: &PlatformFacts)
        -> Result<SchedPlan, MapError>;
    fn map_with_diagnostics(&self, input: &MapperInput, facts: &PlatformFacts)
        -> Result<(SchedPlan, MapDiagnostics), MapError> { /* default: map() + empty */ }
}
```

- **`MapperInput`** — dependency-free facts extracted by the caller:

  ```rust
  pub struct MapperNode {
      pub name: String,                    // FQN
      pub scope: String,                   // namespace / scope path
      pub rate_hz: Option<f64>,
      pub deadline_us: Option<u64>,
      pub criticality: Option<Criticality>, // Low < Medium < High
      pub path_budget_ms: Option<f64>,     // populated by callers; unused by built-in mappers
      pub paths: Vec<MapperPath>,          // per-path facts (chain_aware)
  }
  pub struct MapperInput {
      pub nodes: Vec<MapperNode>,
      pub legacy: Option<SystemSched>,     // manual mapper only (.toml bridge)
      pub chains: Vec<ResolvedChain>,      // chain_aware only
  }
  ```

- **`PlatformFacts`** — alias for `PlatformResources` (the platform
  file's parsed `resources`).
- **`SchedPlan`** — alias for `ResolvedTierTable` (deliberately reused:
  "ordered priority/core/sched_class placement grouped by tier, with
  member node names" is what every mapper produces). Mappers that give
  each node its own priority emit one-member-per-tier entries (tier name
  = node name); nodes with no usable facts collapse into `DEFAULT_TIER`
  (priority 0, no `sched_class` — non-RT).
- **`MapperRegistry`** — `register(Box<dyn SchedMapper>)`, `get(name)`;
  `with_builtins()` pre-registers **four** mappers: `manual`,
  `rate_monotonic`, `deadline_monotonic`, `chain_aware`. Consumers
  register additional mappers at link time; no dynamic loading.
- **`MapError`** — `MissingPriorityBand`, `InvalidPriorityBand`,
  `MissingLegacySpec`, `Resolve(SchedError)`.

## Built-in Mappers

- **`manual`** — legacy semantics. Requires `input.legacy` (populated
  only by the `.toml` bridge); delegates to `resolve()` against the v1
  tiers + `[[assign]]` for `target = "posix"`, reproducing v1 output
  exactly. Ignores `facts`.
- **`rate_monotonic`** — higher `rate_hz` → higher priority, spread
  linearly across `resources.rt_priority_band` (rank 0 → `band.max`,
  last → `band.min`; a narrow band produces ties, never inversions).
  Deterministic: rate descending, ties by node name. No `rate_hz` →
  `DEFAULT_TIER`. Requires a valid posix band.
- **`deadline_monotonic`** — same shape, ranked by `deadline_us`
  ascending (shorter deadline → higher priority).
- **`chain_aware`** — chain-first shaping; the primary mapper for
  systems with `chains:` declarations. Detailed below.

Both simple mappers emit `sched_class: SCHED_FIFO`, `class: real_time`
per ranked node.

**Applying `overrides` is not part of the trait** — "override beats
derived, always" is mapper-independent logic the caller applies after
`map()` returns.

## The `chain_aware` Mapper

Derives a global priority order from cross-scope chain declarations
(PiCAS-style: drain chains toward their sinks), falling back to
criticality-bucketed rate/deadline ordering for everything else. When
`input.chains` is empty it degrades gracefully to that fallback — this
is exactly how nano-ros currently runs it.

Algorithm (steps 1–4 platform-agnostic, 5–6 POSIX realization):

1. **Feasibility.** Per chain: `sampling_cost_ms = Σ over boundary
   elements (period_ms + exec_ms)`; `controllable = max_latency_ms −
   sampling_cost`. Not positive → `MapWarning::ChainInfeasible`, chain
   excluded from shaping (members fall through to the non-chain path).
   (Same rule as the static `chain-sampling-feasibility` check — see
   [contract-theory.md](contract-theory.md#cross-scope-chains-and-sampling-cost).)
2. **Chain order.** Criticality descending (chain criticality = max over
   member nodes, derived by the caller), controllable-slack ascending,
   name ascending.
3. **Within a chain.** Walk elements sink→source: each causal *segment*
   ranks drain-toward-sink (topological order reversed); each maximal
   run of timer *boundaries* keeps its walk position but is internally
   re-ordered rate-monotonically (shorter period first).
4. **Non-chain remainder.** Bucket by criticality (High, Medium, Low,
   none), then order by one unified ascending time budget per path:
   timer period (`1000/rate_hz`) for timer paths, `max_latency_ms` for
   input paths. Paths with no derivable budget — once, spontaneous,
   unclassified, or an input path with no declared `max_latency_ms` —
   never rank → `DEFAULT_TIER`. Items with exactly equal
   (criticality, budget) collapse into one rank (`tie_group`).
5. **Band compression** (POSIX realizer). Dense ranks are fitted into
   `rt_priority_band`: adjacent runs merge first within the same
   `fine_group` (segment / boundary run / bucket), then within the same
   chain (`coarse_group`); if still too wide, the overflow clamps into
   `band.min` (ties, never inversions) with
   `MapWarning::BandTooNarrow`. Priorities are dense from `band.max`
   downward — not the linear spread of the simple mappers.
6. **Node projection.** A node's final priority = **max** over all its
   ranked paths (this also resolves nodes shared across chains).

### Agnostic core / realizer split

The mapper is split so RTOS consumers can reuse the ranking without the
Linux priority model:

- **`chain_aware_rank(&MapperInput) -> RankedPlan`** (also
  `ChainAwareMapper::rank`) — steps 1–4 only. No `PlatformFacts`, no
  band, no OS priorities, infallible.

  ```rust
  pub struct RankItem {
      pub node: String,
      pub path: String,
      pub fine_group: usize,            // segment / boundary-run / bucket;
                                        // doubles as RTOS executor grouping
      pub coarse_group: Option<String>, // chain name; None = non-chain
      pub tie_group: Option<usize>,     // Some ⇒ unconditional collapse
      pub provenance: String,
  }
  pub struct RankedPlan { pub items: Vec<RankItem>, pub warnings: Vec<MapWarning> }
  ```

  `items` order *is* the priority order, highest first.
- **`realize_posix(ranked, input, facts)`** — private; steps 5–6.
  Reached through `ChainAwareMapper::map` / `map_with_diagnostics`.
  Split-parity is test-asserted: realize(rank(input)) is byte-identical
  to the pre-split combined output.
- nano-ros implements its **own realizer** (`realize_rtos`) over the
  same `RankedPlan` — see [Consumers](#consumers).

### Diagnostics

`map_with_diagnostics` returns `MapDiagnostics { details, warnings }`
(deliberately non-serializable — diagnostic output, never embedded in a
model):

- `ChainAwareDetail { node, path, priority, provenance }` — per-node
  `--explain` rows. Provenance strings:
  `derived(chain_aware: <chain> segment drain <k>/<n>)`,
  `derived(chain_aware: <chain> boundary RM period=<p>ms)`,
  `derived(chain_aware: non-chain criticality=Some(High) budget_ms=<b>)`
  (criticality rendered as Rust `Debug` of the `Option`),
  each suffixed `-> prio <p>` by the realizer.
- `MapWarning::ChainInfeasible { chain, sampling_cost_ms, budget_ms }`,
  `MapWarning::BandTooNarrow { distinct_classes, band_width, clamped }`.

## Chain Vocabulary (`chain.rs`)

A deliberate minimal mirror of the `types` crate's Vocabulary v2 — kept
dependency-free so `sched` stays a pure algorithm crate (no parser, no
`types`/`check` deps). All data types serde round-trip; the two
data-carrying enums (`EffectiveTrigger`, `ChainElement`) are adjacently
tagged so their YAML stays plain mappings, no `!tags`:

- `EffectiveTrigger` — `Timer { rate_hz } | Input(endpoints) | Once |
  Spontaneous | Unclassified`; `period_ms() = 1000/rate_hz`.
- `MapperPath { name, effective_trigger, max_latency_ms, exec_ms,
  inputs, outputs }` — the per-(node, path) requirement unit. `exec_ms`
  is the WCET slot; there is no WCET vocabulary in contracts yet, so
  callers pass `None` ("no WCET is ever invented").
- `ChainSemantics { Reaction, Age }`.
- `ChainElement` — `Segment { nodes_in_topo_order: Vec<SegmentNode> }`
  | `Boundary { node, path, period_ms, exec_ms }`.
- `ResolvedChain { name, criticality, max_latency_ms, semantics,
  elements }`. Chain-level criticality does not exist in the authored
  vocabulary — the caller derives it as the max over member nodes.

**Resolution the caller must do** before building `MapperInput`:
resolve `via:` topics across scopes and provide each segment's
`nodes_in_topo_order` — that needs the launch DAG, which this crate
deliberately doesn't have. In practice `ros-launch-resolve` takes the
chain declaration's segment order verbatim (a `segments:` list is
already author-linearized source-to-sink); the fan-in tie-break rule in
the crate's doc comments (longest-path-to-sink, then deadline, then
name) is the contract for callers that must linearize a branching DAG —
no caller implements it yet. Chains with any `chain-link`/shape errors
are excluded from the mapper input.

## Validation Helpers

Pure functions over an already-derived `SchedPlan`; the caller decides
warn-vs-strict and presentation (`--sched-apply`, `--explain`):

- **`band_violations(plan, band)`** — every non-default tier whose
  priority falls outside `[band.min, band.max]`.
- **`rate_priority_contradictions(input, plan)`** /
  **`deadline_priority_contradictions(input, plan)`** — pairwise scan: a
  node with strictly higher rate (or strictly shorter deadline) than
  another must not land at strictly lower priority.
  `rate_monotonic`/`deadline_monotonic` never trigger this by
  construction. `chain_aware` triggers it **by design** — chain rank
  deliberately overrides raw timing facts — so the consumer suppresses
  those as chain-intended; hand-authored overrides and the `manual`
  mapper's independent tiers can trigger it as genuine mistakes.

## Legacy v1 Schema (`system.toml`)

Hand-written tiers + sparse node binding. Still fully supported; sole
implementation of the `manual` mapper. Scheduled for retirement (Phase
41.6) only after nano-ros migrates off it — no flag day.

```toml
# ===== GENERIC (portable — byte-identical across platforms) =====
[tiers.control]
class = "real_time"        # best_effort | real_time | time_triggered | interrupt
deadline_us = 50000
period_us   = 20000
budget_us   = 5000
deadline_policy = "warn"   # ignore | warn | skip | fault
spin_period_us  = 1000

[[assign]]
tier  = "control"
nodes = ["ndt_localizer", "ekf_localizer"]   # FQN or bare-name selectors
[[assign]]
tier  = "perception"
scope = "/perception/lidar"                  # launch-scope subtree selector
# unmatched nodes → synthesized "default" tier (priority 0, non-RT)

# ===== PLATFORM (same shape per target, values differ) =====
[tiers.control.posix]      # `native` accepted as alias
priority    = 80
sched_class = "SCHED_FIFO"
core        = 1
[tiers.control.freertos]
priority    = 12
stack_bytes = 8192
deadline_us = 40000        # optional per-platform tighten
```

- **`SystemSched { tiers: BTreeMap<String, TierDef>, assign:
  Vec<AssignRule> }`**, `deny_unknown_fields` on the generic `TierDef`
  head — a stray `priority` on the head is a parse error; that is what
  enforces "no priority leakage" portability.
- **`TierPlatformSpec`** per target sub-table (`posix`, `freertos`,
  `zephyr`, `threadx`, `nuttx`): `priority: i64` (i64 admits Zephyr
  negative coop priorities), `stack_bytes`, `core`, `sched_class`,
  `preempt_threshold`, and per-platform overrides of the generic head:
  `deadline_us`, `budget_us`/`period_us` (sporadic budget +
  replenishment period, both required for a sporadic policy — lets one
  platform's kernel sporadic server engage, e.g. NuttX
  `SCHED_SPORADIC`, without affecting other targets), and
  `time_slice_us` (round-robin slice among same-priority tiers;
  ThreadX-only today, ignored elsewhere).
- **`resolve(tiers, assigns, nodes, target) -> ResolvedTierTable`**
  (`resolve.rs`): explicit `nodes` selectors win over `scope` selectors
  (silently); a same-level double-claim for two *different* tiers →
  `SchedError::NodeMatchedByMultipleTiers` (duplicate claims for the
  same tier are accepted); missing
  `[tiers.<t>.<target>]` → `MissingPlatformSpec`; unmatched selectors →
  `UnknownNodeSelector`/`UnknownScopeSelector`. Output sorted priority
  descending. `ResolvedTier` is the flat 13-field record (placement from
  the platform sub-table, policy from the generic head, `deadline_us =
  spec ?? head`, sorted `members`).
- These types do double duty: the SystemModel's `execution.tiers`
  reuses `TierDef`/`TierPlatformSpec` (re-exported through the `model`
  crate), so one schema serves v1 authoring, the model's applied-tier
  layer, and the mapper pipeline.

### Legacy `.toml` bridge

`parse_legacy_toml` (`bridge.rs`) wraps a v1 document into a v2
`PlatformFile { target: "posix", mapper: "manual", legacy: Some(sched),
resources/overrides: empty }`. Reached via `parse_platform_file`'s
`.toml` dispatch. Equivalence is test-asserted: TOML → bridge → `manual`
mapper output equals calling `resolve()` directly. RTOS targets don't go
through the bridge — nano-ros calls `resolve(..., target)` directly when
it consumes v1 tiers.

## Consumers

### play_launch / ros-launch-resolve (Linux RT) — shipped

The full pipeline lives in `ros-launch-resolve`
(`resolve/src/ros/sched_loader.rs`, `sched_derive.rs`); `play_launch`
consumes it and owns the apply layer. User guide:
play_launch `docs/guide/rt-scheduling.md`.

- **Discovery** (v2 files only): explicit `--sched <path>` > overlay
  (`--contracts` / `$PLAY_LAUNCH_CONTRACTS` / XDG / `/etc`, layout
  `<root>/<pkg>/launch/<stem>.system.<target>.yaml`) > provider sidecar
  next to the launch file. Same channels as contracts. `--target`
  (default `posix`) must match the file's `target:` header.
- **Derive pipeline** (`derive_sched_plan`): parse platform file → look
  up mapper in `with_builtins()` → `map_with_diagnostics` → flatten to
  one tier per node → **apply overrides** (selector = FQN or bare name;
  a priority-only override implies `SCHED_FIFO` + `real_time` and
  promotes the node out of the default tier; overriding a chain member
  below its chain rank warns) → band violations (clamp + warn, or error
  under `--sched-apply strict`) → rate/deadline contradiction warnings
  (with chain-intended suppression).
- **Where it runs**: fresh derive on `check --sched [--explain]`,
  `resolve`, `launch`, and `run`. `replay <model.yaml>` does **not**
  re-run the mapper — it reads the model's `execution.tiers` +
  `execution.bindings` (see below).
- **What lands in the SystemModel**: only the applied schedule —
  synthesized `TierDef`s + `bindings` (FQN → tier, default tier
  excluded). **No resolved plan is embedded** (`execution.sched` landed
  and was reverted, 2026-07-20 maintainer decision, rlm `f090400`): the
  model is *input*; causality + execution modeling is each consumer's
  job. Mapper identity, chain decomposition, per-path ranks, and
  diagnostics exist only on a fresh derive — hence `replay --explain`
  shows degraded `derived((applied): tier ...)` provenance.
- **Apply layer** (play_launch): `--sched-apply off|warn|strict`
  (default `warn`; on `launch`, `replay`, and `run`); per-TID
  `sched_setscheduler` (`SCHED_FIFO`/`SCHED_RR`, priority validated
  `1..=99`) + `sched_setaffinity` across `/proc/<pid>/task/*`; non-root
  via the `CAP_SYS_NICE` `play_launch_rt_helper`
  (`play_launch setcap`). Applied to regular nodes, container processes
  (re-applied on respawn), and composable nodes on their LOADED event.

  ```bash
  # Sidecar <stem>.system.posix.yaml shipped next to the launch file
  # (or in the overlay) is discovered automatically:
  play_launch check --explain <pkg> <launch_file>     # print derived plan + provenance
  play_launch launch <pkg> <launch_file>              # derive + apply (warn on failure)
  play_launch launch <pkg> <launch_file> --sched-apply strict   # abort if apply fails

  # Explicit platform file (also the only way to use a legacy .toml):
  play_launch check --sched bringup.system.posix.yaml --explain <pkg> <launch_file>
  ```

### nano-ros (RTOS) — shipped, derived path opt-in

nano-ros vendors this crate (nested submodule via `ros-launch-resolve`,
pinned in the CLI package) and consumes the **agnostic core**, never the
posix realizer. Design of record: nano-ros RFC-0050 §"Input model" and
RFC-0052 §"system-model RTOS mapper".

- **Derivation**: `nros-orchestration-ir::mapper_input` builds
  `MapperInput` from the resolved SystemModel's input layers
  (`structure.nodes`, `contracts.node_paths`, `contracts.pub_endpoints`)
  — never from any embedded plan. Chains are not yet declared in its
  models, so `chain_aware_rank` currently degrades to the
  criticality-bucketed rate/deadline fallback by construction.
- **Realizer**: `realize_rtos` maps the `RankedPlan` onto per-RTOS
  capabilities (`SchedCaps`: priority count, numbering direction, EDF,
  sporadic reservation, preemption threshold, affinity) for
  posix/Zephyr/FreeRTOS/ThreadX/NuttX, recording per-dimension
  native/backfill/degrade provenance. Its v1 realizes activation,
  urgency, deadline, and budget (e.g. Zephyr native EDF via
  `k_thread_deadline_set`, NuttX `SCHED_SPORADIC` budgets); placement
  and preemption-threshold are modeled in `SchedCaps` with runtime
  support landed, but the realizer does not emit them yet (later
  waves). `RankItem.fine_group` doubles as its executor grouping.
- **Authoring**: nano-ros authors its own `system.toml` (its bringup
  config: `[tiers.*]`, `[[node_overrides]]`, lifecycle, bridges — a
  superset role, ingested via the model's system-config layer, reusing
  `sched::TierDef`). It does **not** yet author v2
  `<stem>.system.<target>.yaml` platform files — its resolver plumbs
  `--sched`, but the workspace sync never passes it. The derived
  (mapper) path activates only when a model carries no declared
  `execution.tiers`.

## Distribution & Cross-Repo Sharing

- **Authored in** this repo; consumed by `ros-launch-resolve` (submodule
  `third-party/ros-launch-manifest`) which is in turn vendored by both
  play_launch and nano-ros. Consumers pin different revisions — check
  the submodule pins before assuming API parity.
- **Dependencies:** `serde`, `thiserror`, `toml`, `serde_yaml_ng`. Pure
  host code; no parser/`types`/`check` deps, no runtime deps.
- **Portability:** generic facts and the ranking core are byte-identical
  across platforms; platform numbers exist only in per-target sub-tables
  (v1), the posix realizer (Linux), or consumer-owned realizers (RTOS).

## Design of Record

- play_launch `docs/superpowers/specs/2026-07-01-shared-scheduling-crate-design.md`
  — v1 / shared-crate design.
- play_launch `docs/superpowers/specs/2026-07-16-rt-config-v2-design.md`
  — v2 derived-scheduling design (Phase 41).
- nano-ros `docs/design/0050-system-model.md`, `0052-system-model-rtos-mapper.md`
  — cross-repo agreement: input-only model, algorithm-shared-not-output,
  per-consumer realizers (2026-07-20, supersedes the earlier
  "scheduling SSoT" direction).

## Typed `posix` Placement (`posix.rs`)

`sched_class: Option<String>` + `priority: i64` + `core: Option<u32>` made
illegal states representable and left every consumer re-deriving which fields
were live from a string. A tier could carry `sched_class: "SCHED_OTHER"` beside
`priority: 10` — a state Linux cannot express, which shipped into
`system_model.yaml` and was fatal under strict mode.

```rust
pub enum PosixSched {
    Idle,
    Batch    { nice: i32 },
    Other    { nice: i32 },
    Fifo     { priority: i32 },
    Rr       { priority: i32 },
    Deadline { runtime_ns: u64, deadline_ns: u64, period_ns: u64, overrun: bool },
}

pub enum PosixAffinity { Inherit, Cpus { cpus: Vec<u32> }, Cpuset { path: String } }

pub struct PosixPlacement {
    pub sched: PosixSched,
    pub affinity: PosixAffinity,
    pub uclamp: Option<(u32, u32)>,
}
```

Each variant names only the parameters that policy actually has, so `Batch` has
no priority, `Fifo` has no nice, and `Deadline` has neither — it carries a
reservation. `PosixAffinity` is an enum rather than two optional fields because
a CPU mask and a cpuset are mutually exclusive *and* policy-dependent:
`SCHED_DEADLINE` may not use `sched_setaffinity(2)` at all, since a deadline
thread's affinity may not be narrower than the root domain it was created on
(`EPERM`). `PosixPlacement::validate` rejects the rest.

Two consequences worth stating, because both correspond to shipped defects:

- **`PosixSched::priority()` returns `None` for `Deadline`** — absent, not
  zero. A deadline thread preempts every fixed-priority thread regardless of RT
  priority, which is why the kernel gives it a reservation instead of a number.
  `band_violations` skips such tiers rather than comparing them against the RT
  band, the same way it skips the default tier.
- **`requires_reset_on_fork()` is a method, not a field**, and true only for
  `Deadline`. `SCHED_FLAG_RESET_ON_FORK` reads as hygiene but the kernel resets
  scheduling in `sched_fork()`, which runs for *thread* creation as well — so
  setting it on `SCHED_FIFO` stops threads created after an apply sweep from
  inheriting the policy, leaving an arbitrary subset of a node's threads at
  `SCHED_OTHER`. Measured downstream, not theorised.

`ResolvedTier::posix: Option<PosixPlacement>` is **additive**: `sched_class`,
`priority` and `core` remain for one release so consumers migrate without a
lockstep bump. When present, the typed placement is authoritative.
