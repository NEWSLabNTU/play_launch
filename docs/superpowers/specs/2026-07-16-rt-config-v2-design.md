# RT Config v2 — Derived Scheduling — Design

**Date:** 2026-07-16
**Status:** Approved (design), pending implementation
**Repo:** `play_launch` (+ `src/ros-launch-manifest` sched crate; nano-ros consumes the crate)
**Supersedes-in-part:** `2026-07-01-shared-scheduling-crate-design.md` (tier authoring model), Phase 38 config surface (apply layer itself unchanged).

## Motivation (feedback being answered)

The RT configuration currently spans three files with three formats:

1. **Launch** (XML) — modular execution plan + params; the graph and per-node/topic
   keys derive from it. Kept as-is for ROS 2 tool compatibility; the SSoT.
2. **Contract** (`<stem>.contract.yaml`, YAML) — platform-agnostic constraints and
   QoS (Phase 40 shipping: provider sidecar + user overlay).
3. **Platform config** (`system.toml`, TOML) — hand-written tiers with real
   priority numbers, `[[assign]]` binding.

Feedback:

- Related information is scattered across files. The scatter is principled
  (ROS compat + modularity; separation of platform concerns) but the UX suffers.
- The three file formats differ (XML / YAML / TOML).
- The sched context in `system.toml` is hand-written, yet the runtime can derive
  scheduling-relevant context from launch+contract (rates, deadlines, path
  budgets). Manual config can silently conflict with the derived truth.
- The mapping from the platform-agnostic contract down to a concrete sched
  context (Linux or RTOS) can be done many ways — it needs a pluggable
  algorithm, not a single hard-wired convention.

## Decisions (summary)

| Question | Decision |
|---|---|
| Derived vs hand-written sched context | **Derive + override**: a named mapper derives the plan from launch+contract; the platform file provides platform facts and explicit per-node overrides. Manual authoring survives as the `manual` mapper, not as the model. |
| Mapping pluggability | **`SchedMapper` trait + built-ins** in the sched crate; selected by name in config; consumers register extra impls at link time. No dynamic loading. |
| File layout / formats | **Three files kept** (the separation IS the principle); the two files we control unify on **YAML**. Launch XML untouched. Scatter answered by tooling (`--explain` merged view). |
| Platform file ownership | **Per-target files** (`target:` header, one file = one target), shipped through the same provider-sidecar + user-overlay channels as contracts. No multi-target sections, no deep-merge. |
| Migration | **Dual-format bridge**: legacy `system.toml` (current schema) keeps parsing and maps internally to the `manual` mapper. Extension selects the parser. TOML retired after nano-ros migrates. |

## 1. Principles (revised)

Three roles, one file each:

- **Launch XML** — graph SSoT. ROS compat; never extended (annex rejected in the
  Phase 40 investigation).
- **Contract YAML** — platform-agnostic constraints; provider-shipped, user-overlayable
  (Phase 40 channels unchanged).
- **Platform file YAML** — platform facts + explicit pins for ONE target;
  provider may ship a reference config, deployer overlays or writes their own.

New principle: **scheduling context is derived from launch+contract by a named
mapper; the platform file constrains and overrides it — it no longer authors
tiers.**

## 2. File model

### 2.1 Contract (`launch/<stem>.contract.yaml`)

Unchanged format and shipping. Existing timing fields (`rate_hz`,
`min_rate_hz`, `max_latency_ms`, path budgets) gain a second job: mapper
inputs. One new optional platform-agnostic field:

```yaml
nodes:
  control_node:
    criticality: high        # mapper hint: high | medium | low; no numbers
```

### 2.2 Platform file (`launch/<stem>.system.<target>.yaml`)

New schema, YAML, one target per file:

```yaml
target: posix                # required; must match filename target segment
mapper: rate_monotonic       # SchedMapper strategy name
resources:                   # platform facts the mapper works within
  rt_priority_band: { min: 10, max: 40 }
  isolated_cpus: [0]
overrides:                   # explicit pins; beat derived values, always
  control_node: { priority: 20, core: 0 }
```

- Target key examples: `posix` (Linux RT), `zephyr`, `freertos` (RTOS keys
  resolved by nano-ros). `resources`/`overrides` value vocabulary is
  per-target (posix: priority/core/sched_class; RTOS targets add stack sizes
  etc. — defined by the consumer, validated by the sched crate per target).
- **Per-target files, separate owners**: a provider shipping a POSIX reference
  and a board user writing a Zephyr config produce separate files. No shared
  file to fight over; channel resolution is whole-file-wins per target
  (the Phase 40 lesson — no section-level deep-merge).

### 2.3 Legacy `system.toml` (bridge)

Current schema (tiers with priorities + `[[assign]]`) keeps parsing. It maps
internally to `mapper: manual` plus an equivalent resolved plan; target
implied `posix` for play_launch. File extension selects the parser
(`.toml` → legacy schema, `.yaml` → new schema). Deprecation documented;
removal after nano-ros migrates to the new schema.

## 3. Shipping, resolution, and the /opt workflow

### 3.1 Channels (mirrors Phase 40 contracts)

For both contracts and platform files:

1. explicit CLI path (`--sched <path>` for the platform file) — always wins;
2. user overlay: `<overlay-root>/<pkg>/launch/<stem>.system.<target>.yaml`
   (and `<stem>.contract.yaml`);
3. provider sidecar: installed next to the launch file
   (`/opt/.../share/<pkg>/launch/...`, read-only — installed via the package's
   normal `install(DIRECTORY launch ...)`).

Target selection: `--target <t>` (play_launch default: `posix`; nano-ros
codegen passes its board target). Overlay overrides provider only for the
same target — a user Zephyr file never shadows a provider POSIX file.

**Auto-apply at launch (decision 2026-07-16):** when `--sched` is absent,
launch/replay resolve the platform file through the channels and APPLY it.
The provider sidecar is the vendor's shipped default and is trusted — RT
works out of the box on installed systems. The overlay exists to fill gaps
(vendor ships no config, or none for your target/board) and to tweak vendor
numbers that don't fit; per (stem, target) the overlay wins. Escape hatches:
`--sched-apply off` disables all applying; explicit `--sched <path>`
bypasses discovery entirely. `check` validates all channels and never
applies. The `--no-provider-contracts` flag disables the provider channel
for BOTH contracts and platform files.

### 3.2 Overlay-root discovery

The overlay root is the **first existing** of, in order (no cross-root
merging):

1. `--contracts <dir>` (explicit flag; when given, the rest are not consulted)
2. `$PLAY_LAUNCH_CONTRACTS`
3. `$XDG_CONFIG_HOME/play_launch/contracts` (default `~/.config/play_launch/contracts`)
4. `/etc/play_launch/contracts`

One overlay tree serves both file kinds:
`<root>/<pkg>/launch/<stem>.contract.yaml` and
`<root>/<pkg>/launch/<stem>.system.<target>.yaml`.

### 3.3 Adjustment ergonomics

New subcommand:

```
play_launch contract eject <pkg> <launch-file> [--target <t>] [--into <root>]
```

Copies the resolved provider contract (and the target's system file, if any)
into the overlay tree at the correct path, ready to edit. Editing never
touches `/opt`. Future tooling (out of scope now): `contract diff` comparing
overlay vs current provider to surface upgrade staleness; `--explain`
provenance already shows which channel won.

## 4. `SchedMapper` trait (sched crate)

```rust
pub trait SchedMapper {
    fn name(&self) -> &str;
    fn map(&self, input: &MapperInput, facts: &PlatformFacts) -> Result<SchedPlan, MapError>;
}
```

- `MapperInput`: dependency-free per-node records extracted from
  launch+contract — name, scope, rates, deadlines, criticality, path budgets,
  plus graph edges as needed. Same no-parser-dep constraint as today's
  `SchedNode` (the crate must stay pure-host; nano-ros links it).
- `PlatformFacts`: the platform file's `resources` (typed per target).
- `SchedPlan`: per-node `{ class, priority, core?, tier }` for the target.
- **Built-ins**: `manual` (legacy semantics; consumes the bridge or explicit
  assignments), `rate_monotonic` (priority order by rate within the band),
  `deadline_monotonic`.
- Registry keyed by name; play_launch and nano-ros register additional
  mappers at link time. External/WASM plugins out of scope.

## 5. Pipeline

```
launch + contract ──extract──▶ MapperInput ──mapper──▶ derived plan
platform file: resources ────────────────────┘              │
platform file: overrides ──────────apply───────────────────▶ final plan
                                                              │ validate
                                              posix: Phase 38 apply layer (unchanged)
                                              RTOS:  nano-ros codegen
```

The Phase 38 apply layer (per-TID SCHED_FIFO/RR + affinity via
`play_launch_rt_helper`) consumes a resolved plan exactly as today; nothing
below the plan changes.

## 6. Conflict semantics

- Override beats derived, always; every divergence is listed by `--explain`.
- Priority (derived or overridden) outside `resources.rt_priority_band`:
  warn + clamp under `--sched-apply warn`, error under `strict`.
- Node with no timing facts and no override → non-RT default (`SCHED_OTHER`),
  reported.
- Node with no timing facts but WITH an override → promoted to RT with the
  override's values (a pin is an explicit user statement; `SCHED_FIFO` unless
  the override says otherwise). Reported as `override` provenance.
- **Contract-vs-plan sanity** (the hand-written-conflict feedback made
  checkable): new check rule — a final priority order that contradicts the
  contract's rate/deadline order yields a warning citing both sources
  (contract line and platform-file line).

## 7. `--explain`

`play_launch check --sched <file> --explain` prints the merged logical view —
per-node table of final class/priority/core with provenance annotations:

```
/control/control_node   FIFO 20  cpu 0   override(system.posix.yaml:9)
/perception/sensor_node FIFO 38  —       derived(rate_monotonic: 100 Hz → prio 38)
/perception/filter      OTHER    —       default (no timing facts)
system file: overlay(~/.config/play_launch/contracts/rt_demo/launch/bringup.system.posix.yaml)
contract:    provider(share/rt_demo/launch/bringup.contract.yaml)
```

This is the answer to "the information is scattered": storage stays modular,
the tool presents one document.

## 8. Testing

- **Sched crate**: per-mapper unit tests; bridge equivalence (legacy TOML →
  identical plan to `manual` mapper); band clamp / conflict / missing-facts
  cases; per-target `resources`/`overrides` validation.
- **Integration** (`tests/`): overlay-root discovery order; `contract eject`
  round-trip; `--explain` golden assertions; rate-vs-priority contradiction
  warning; rt_workspace fixture gains `bringup.system.posix.yaml` (sidecar
  form) + a Zephyr stub proving per-target coexistence + a user-overlay
  example, while keeping `system.toml` as the living bridge test until
  retirement.

## 9. Migration and retirement

Phased (see the phase doc): land schema + mapper + channels alongside the old
path; migrate file formats, directory structure, fixtures, and tests; then
retire the legacy path (`system.toml` schema, bare-file conventions) once
nano-ros has migrated. No flag day.

## 10. Out of scope

- External/WASM mapper plugins.
- Callback-level (sub-node) scheduling; tiers remain node-granularity.
- Automatic criticality inference.
- The nano-ros migration itself (separate track; the trait and schema land in
  the shared crate so that track can consume them).
