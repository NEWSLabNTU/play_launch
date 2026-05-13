# Architecture Review — Component Boundaries

Snapshot of the codebase as of 2026-05-13, after Phases 17–36 + the
autoware-contract Phase 10 promotion campaign. Goal: identify where
component boundaries blur, and propose refactors that sharpen them.

## Current layout

```
                ┌──────────────────────────────────────────────┐
                │   launch file (XML / Python)                 │
                └────────────────┬─────────────────────────────┘
                                 │
                                 ▼
                ┌───────────────────────────────────────────┐
                │ play_launch_parser                        │
                │   - parses launch tree                    │
                │   - writes record.json                    │
                │   - optional --features ir (IR types)     │
                │   - separate submodule + Cargo workspace  │
                └────────────────┬──────────────────────────┘
                                 │ record.json
                                 ▼
        ┌────────────────────────────────────────────────────────┐
        │ play_launch (single binary, ~19k LOC)                  │
        │                                                        │
        │  ┌─────────────────────────────────────────────────┐   │
        │  │ cli/             arg parsing                    │   │
        │  │ commands/        top-level CLI handlers         │   │
        │  ├─────────────────────────────────────────────────┤   │
        │  │ execution/       cmdline build, context fan-out │   │
        │  │ member_actor/    per-node lifecycle actor       │   │
        │  │ monitoring/      resource monitor, csv writer   │   │
        │  │ process/         pgid, cleanup, tree            │   │
        │  ├─────────────────────────────────────────────────┤   │
        │  │ ros/             ROS interop                    │   │
        │  │   ament_index, manifest_loader, manifest_graph, │   │
        │  │   parameter_*, container_readiness, launch_dump │   │
        │  ├─────────────────────────────────────────────────┤   │
        │  │ runtime_enforcement/  Phase 36 RuleEngine       │   │
        │  │ interception/         SPSC consumer + mirror    │   │
        │  ├─────────────────────────────────────────────────┤   │
        │  │ diagnostics/    ROS /diagnostics topic capture  │   │
        │  │ web/            HTTP/SSE UI                     │   │
        │  │ python/         pyo3 bridges (parser, plot)     │   │
        │  │ ipc/            io_helper IPC over socketpair   │   │
        │  └─────────────────────────────────────────────────┘   │
        └────────────────────────────────────────────────────────┘
                                 │ LD_PRELOAD + env vars
                                 ▼
        ┌──────────────────────────────────────────────────┐
        │ play_launch_interception (cdylib)                │
        │   - hooks rcl_publisher/subscription_init,       │
        │     rcl_publish, rcl_take                        │
        │   - hooks rmw_create_publisher/subscription,     │
        │     rmw_publish, rmw_take_with_info,             │
        │     rmw_*_event_init, rmw_take_event             │
        │   - dlsyms rcl_expand_topic_name +               │
        │     rcl_remap_topic_name                         │
        │   - plugins: frontier, stats, qos_negotiation,   │
        │     dds_events, topic_names                      │
        │   - writes events through spsc_shm ring          │
        └──────────────────────────────────────────────────┘

External:
        ┌──────────────────────────────────────────────────┐
        │ ros-launch-manifest (submodule, two crates)      │
        │   - types: manifest schema + parser              │
        │   - check: rules (consistency, qos_match,        │
        │            qos_compat, satisfiability, etc.)     │
        └──────────────────────────────────────────────────┘
        ┌──────────────────────────────────────────────────┐
        │ rcl_interception_sys (submodule)                 │
        │   - bindgen for rcl/rosidl                       │
        │   - hand-written opaque types + fn pointer       │
        │     aliases for dlsym-only symbols               │
        └──────────────────────────────────────────────────┘
        ┌──────────────────────────────────────────────────┐
        │ spsc_shm                                         │
        │   - generic single-producer/consumer ring over   │
        │     memfd + eventfd. Zero dep on ROS or rcl.     │
        └──────────────────────────────────────────────────┘
        ┌──────────────────────────────────────────────────┐
        │ play_launch_container (C++ colcon pkg)           │
        │   - ObservableComponentManager                   │
        │   - CloneIsolatedComponentManager                │
        └──────────────────────────────────────────────────┘
        ┌──────────────────────────────────────────────────┐
        │ play_launch_msgs (C++ colcon pkg)                │
        │   - ComponentEvent.msg                           │
        └──────────────────────────────────────────────────┘
```

## Boundary issues

### 1. `EventKind` + `InterceptionEvent` is mirrored in two crates

**Problem.** Both `play_launch_interception::event` (producer, .so) and
`play_launch::interception::mod` (consumer) define the 13-variant
`EventKind` and the 40-byte `InterceptionEvent` separately. Each new
event variant requires two coordinated edits. Drift between the two
is a silent corruption risk because the byte layout is consumed via
`spsc_shm::Consumer<InterceptionEvent>` with no schema check.

**Fix.** Extract into a tiny `play_launch_interception_types` crate
(or fold into `spsc_shm` as a separate module). Both .so and consumer
depend on it. Single source of truth.

### 2. `record.json` types appear in two crates

**Problem.** `play_launch_parser::record::types` (producer) and
`play_launch::ros::launch_dump` (consumer) both define `ScopeEntry`,
`ScopeTable`, and per-record structs. Currently they're kept in
sync by hand. Deserialization on the executor side re-types fields
from the parser's serialization.

**Fix.** Extract a `play_launch_record_format` crate containing only
the on-disk schema (no executor-side semantics). Both parser and
executor depend on it. Frees the parser submodule from carrying a
serde-tied API surface.

### 3. `play_launch` crate is monolithic (~19k LOC)

**Problem.** Single library + binary handles parsing-consumer,
execution, monitoring, web UI, runtime enforcement, ROS interop,
Python bridge. Cross-feature deps are hard to enforce; CI compiles
all of it for any change. Refactoring one area risks unrelated areas.

**Fix (long-term).** Split into:

| Crate | Owns | Depends on |
|-------|------|-----------|
| `play_launch_record_format` | `record.json` schema | (none) |
| `play_launch_interception_types` | `InterceptionEvent`, `EventKind`, chunk codec | `spsc_shm` |
| `play_launch_manifest_index` | `ManifestIndex`, `manifest_loader`, `manifest_graph` | `ros-launch-manifest-types`, `record_format` |
| `play_launch_enforcement` | `RuleEngine`, lifecycle, violation IO | `manifest_index`, `interception_types`, `rclrs` (feature-gated) |
| `play_launch_executor` | `execution/`, `member_actor/`, `monitoring/`, `process/` | `record_format` |
| `play_launch_web` | `web/`, `diagnostics/` | `executor`, `enforcement` |
| `play_launch` | `cli/`, `commands/`, `main.rs` glue | all above |

Each step is mechanically straightforward but touches many files.
Worth doing incrementally — split out `interception_types` and
`record_format` first (lowest churn), the rest later.

### 4. `play_launch::ros` is a grab-bag

**Problem.** `src/play_launch/src/ros/` collects:

- `ament_index.rs` — generic ROS resource lookup
- `manifest_loader.rs`, `manifest_graph.rs` — manifest tree → index
- `parameter_proxy.rs`, `parameter_types.rs`, `parameter_conversion.rs` — ROS param RPC
- `container_readiness.rs` — composable-container readiness probe
- `launch_dump.rs` — duplicate of parser's record types
- `graph_builder.rs` — runtime graph snapshot

These don't share a single purpose. `parameter_proxy` is unrelated
to `manifest_loader` except both speak ROS.

**Fix.** After the split above, the manifest pieces move to
`manifest_index`, parameter pieces stay close to web/UI consumers,
`ament_index` becomes a small utility crate (reusable elsewhere),
`launch_dump` collapses into `record_format`.

### 5. `runtime_enforcement` couples to many layers

**Problem.** `RuleEngine` reaches into:

- `ros-launch-manifest-types` (ManifestIndex)
- `play_launch::interception::EventKind` (mirrored types)
- `lifecycle_msgs` (via rclrs)
- `tokio` (lifecycle_rx watch channel)
- `serde_json` (violation jsonl)

Five direct deps means the engine can't be reused outside `play_launch`
(e.g. as a library in a separate runtime monitor).

**Fix.** Pull manifest types behind a trait, accept `&dyn ManifestIndexLike`.
Move tokio integration into a thin wrapper layer. The core rule logic
should be sync, deterministic, and testable without spinning up a
runtime. (Mostly already is — the lifecycle channel is the one async
piece.)

### 6. `commands/` mixes business logic with CLI plumbing

**Problem.** `commands/launch.rs`, `commands/replay.rs` build full
async runtimes, spawn actors, set up web servers, etc. They aren't
just argument-routing shims; they're the executor's main loop. ~1500
LOC across `commands/`.

**Fix.** After the executor split, `commands/launch.rs` should be a
~50-line glue function: parse args, call
`play_launch_executor::run(record, config)`, return.

### 7. `play_launch_parser` and `play_launch` have parallel `record.json` schemas

Already noted in #2. Calling it out separately because the duplication
*now* is invisible — the executor side just deserializes what the
parser wrote — but future schema evolution is risky. The Phase 30
scope-table extension already required edits in both crates.

### 8. CLI args are flat

**Problem.** `cli::options::LaunchArgs` carries 30+ fields covering
manifest-dir, enforcement mode, interception toggles, web-ui binding,
container mode, monitoring switches. One struct for all subcommands.

**Fix.** Per-subcommand argument groups, composed via clap's
`#[clap(flatten)]`. Decouples feature flags from subcommand surface.

### 9. Submodule and feature-flag entanglement

- `play_launch_parser` is a git submodule but uses a `[workspace]` of
  its own + a `--features ir` flag for IR types. The parent
  workspace excludes it but still consumes it as a path dep. CI for
  parser changes requires `cd src/play_launch_parser && cargo test`.
- `ros-launch-manifest` is a submodule. Two crates (types, check).
  Cleanly factored.
- `rcl_interception_sys` is a submodule with two distros (humble,
  jazzy) gated by features.
- `spsc_shm` is local-path with its own `[workspace]`.

**Observation, not bug:** submodule boundaries are intentional —
each is independently publishable. Worth documenting in a `README.md`
which crates live where, since the layout isn't obvious.

## What is well-factored

- **`spsc_shm`** — zero ROS deps. Generic over `T`. Reusable.
- **`ros-launch-manifest`** — types/check split is exemplary. Static
  check (in `check`) has no executor or runtime deps; pure data + rules.
- **`play_launch_interception`** — clean cdylib. No deps on play_launch.
  Owns the producer side of the event format.
- **`rcl_interception_sys`** — small FFI crate. Hand-written opaque
  + fn-pointer aliases for dlsym symbols; bindgen handles the rest.
- **`play_launch_container`** — C++ pkg in colcon, separate from Rust.
  Clear ownership.

## Recommended next steps (lowest-risk first)

1. **`play_launch_interception_types` crate** — extract `EventKind`,
   `InterceptionEvent`, name/type-chunk codecs. Both producer and
   consumer depend on it. Eliminates mirror drift. ~200 LOC move.

2. **`play_launch_record_format` crate** — extract `record.json` types
   from `play_launch_parser` and `play_launch::ros::launch_dump`.
   Both producer (parser) and consumer (executor) depend on it.
   ~500 LOC move. Touches the parser submodule, so coordinated push
   needed.

3. **Trait-ify `ManifestIndex` access in `runtime_enforcement`.**
   Cheap (~50 LOC). Enables unit tests without a real manifest tree.

4. **Per-subcommand arg structs in `cli::options`.** Mechanical,
   touches ~5 files.

5. **Split `play_launch::ros::manifest_*` + `runtime_enforcement` into
   `play_launch_enforcement` crate.** Lets enforcement be reused as a
   library; isolates rclrs/tokio deps.

6. **Split executor into `play_launch_executor` crate.** Largest move
   (~8k LOC); biggest payoff for compile times and modularity.

Each step keeps the binary working and the test suite green. None
require a flag day.
