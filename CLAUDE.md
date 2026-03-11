# CLAUDE.md

Guide for Claude Code when working with this repository.

## Project Overview

ROS2 Launch Inspection Tool - Records and replays ROS 2 launch executions for performance analysis:
- **play_launch_parser** (Rust): Parses launch files to `record.json` (default, 3-12x faster)
- **dump_launch** (Python): Alternative parser for maximum compatibility
- **play_launch** (Rust): Replays with resource monitoring
- **play_launch_analyzer** (Python): Analyzes and visualizes logs
- **play_launch_container** (C++): Custom component container with event publishing
- **play_launch_msgs** (C++): ROS 2 message definitions (ComponentEvent.msg)
- **play_launch_wasm_common** (Rust): WASM ABI definitions (host function signatures, memory layout)
- **play_launch_wasm_codegen** (Rust): Compiles Launch IR → WASM bytecode
- **play_launch_wasm_runtime** (Rust): Executes WASM launch plans via wasmtime

## Launch File Parsing

**Default**: Rust parser (`play_launch_parser`, 3-12x faster, 413 tests, 100% Autoware)
**Alternative**: `play_launch launch <pkg> <file> --parser python`

**CRITICAL RULE**: When Rust and Python parser behaviors differ, **Python's behavior is always correct**. Fix Rust, not Python. Validate with `just compare-dumps` in test workspaces.

### Parser Architecture

The parser evaluates conditions during parsing and processes only the selected path. Key rules:
- **Conditional substitutions** (IfElse, Equals, etc.): call `perform()` to evaluate → "true"/"false"
- **LaunchConfiguration substitutions**: call `__str__()` to preserve as `$(var name)` for replay-time
- **Float parameters**: always include decimal point (`0.0` not `0`) for ROS type preservation
- **`<let>` statements**: sequential parse-time resolution — values resolved immediately, stored in record.json
- **Runtime fallback**: unresolved `$(var ...)` in executable names resolved at replay time (`src/execution/node_cmdline.rs`)
- **YAML params**: substitutions in YAML files resolved and typed before passing to nodes (`src/params.rs::load_and_resolve_param_file()`)

Implementation: `src/python/api/utils.rs` (substitution handling), `src/params.rs` (YAML resolution)

### Launch IR & WASM Pipeline (Phase 22)

The parser has a two-stage architecture: parse → IR → evaluate.

- **`analyze_launch_file_with_args()`** → `LaunchProgram` IR (preserves full structure: conditions, substitution expressions, groups, includes)
- **`compile_to_wasm()`** → WASM bytecode (IR compiled to wasmtime-executable module)
- **`execute_wasm()`** → `RecordJson` (WASM host functions build records via begin/set/end pattern)

IR types: `src/play_launch_parser/.../ir.rs` — `ActionKind` enum (DeclareArgument, SetVariable, SetEnv, UnsetEnv, PushNamespace, PopNamespace, SetParameter, SetRemap, Group, Include, Node, Container, LoadComposableNode, Executable, OpaqueFunction).

WASM ABI: `src/play_launch_wasm_common/src/lib.rs` — host function signatures, memory layout (return area + bump allocator).

CLI (feature-gated): `play_launch compile <pkg> <file> -o plan.wasm`, `play_launch exec plan.wasm -o record.json`, `play_launch dump <pkg> <file> --wasm`. Build with `just build-wasm` or `cargo build --features wasm`.

## Installation & Usage

```sh
just build                          # Full build: colcon + bundle + wheel
just build-cpp                      # C++ only (msgs + container)
just build-rust                     # Rust only (assumes C++ install/ exists)
just build-wheel                    # Bundle + wheel only (no colcon rebuild)
just build-interception             # Build interception .so (standalone, not in colcon)
just run launch <pkg> <launch_file> # Run with colcon build
play_launch launch <pkg> <launch>   # Run if installed via pip
play_launch plot                    # Analysis
```

## Architecture

**Execution Flow:** Load `record.json` → classify nodes → spawn async tasks → actor-based lifecycle → logs to `play_log/<timestamp>/`

**Key Concepts:**
- **Async/Tokio**: All background services run as async tasks
- **Actor Pattern**: Self-contained lifecycle management with respawn support
- **Virtual Members**: Composable nodes managed by container actors (LoadNode/Unload)

### Container Architecture

Custom C++ container (`src/play_launch_container/`) replaces stock `rclcpp_components`:

```
rclcpp_components::ComponentManager           (upstream)
  └── ObservableComponentManager              (ours: event publishing)
        └── CloneIsolatedComponentManager     (ours: fork+exec per-node isolation)
```

- **ObservableComponentManager**: publishes `ComponentEvent` on `~/_container/component_events` (reliable, transient_local, depth 100)
- **CloneIsolatedComponentManager**: fork()+exec() of `component_node` binary per composable node
  - Parameters: `request->parameters` serialized to temp YAML (`/**:` wildcard namespace), passed via `--params-file`
  - `extra_arguments`: only `use_intra_process_comms` is extracted and forwarded as `--use-intra-process-comms`
  - Ready pipe protocol: child writes `"OK name\n"` or `"ERR msg\n"`, 30s timeout
  - YAML type preservation: double values always include decimal point (`0.0` not `0`) to prevent integer_array/double_array mismatch
- **Two binaries**: `component_container [--use_multi_threaded_executor] [--isolated]` and `component_node` (standalone single-node loader)
- **Design docs**: `docs/design/container-isolation.md`, `docs/archive/clone-vm-container-design.md`
- **Roadmap**: `docs/roadmap/phase-19-isolated_container.md`

## Configuration

All features **enabled by default**: monitoring, diagnostics, web UI (http://127.0.0.1:8080).

Key flags: `--config <PATH>`, `--disable-monitoring`, `--disable-diagnostics`, `--disable-web-ui`, `--disable-all`, `--web-addr <IP:PORT>`, `--disable-respawn`, `--enable <FEATURE>`, `--container-mode <MODE>`.

**Container mode** (`--container-mode`, default `isolated`):
- `observable` — override all containers to use `play_launch_container` with `ComponentEvent` publishing
- `isolated` — use `play_launch_container` with `--isolated` (fork+exec per-node process isolation)
- `stock` — use the original container binary from the launch file (no override, no `ComponentEvent` subscription)

When mode is not `stock`, `prepare_container_contexts()` in `src/execution/context.rs` overrides the container's `package` and `executable`. If the original was `component_container_mt`, `--use_multi_threaded_executor` is prepended to args. Original `args` from the launch file (e.g. `--isolated`) are preserved.

See `tests/fixtures/autoware/autoware_config.yaml` for full config YAML reference.

**Interception** (`interception` config section, default: disabled):
- `enabled: true` — inject `LD_PRELOAD` interception .so into all child processes
- `frontier: true` — enable per-topic timestamp frontier tracking
- `stats: true` — enable per-topic message count/rate statistics
- `ring_capacity: 65536` — SPSC ring buffer capacity per child process

When enabled, play_launch creates a shared memory ring buffer per child, injects `LD_PRELOAD` + fd env vars, and spawns a consumer tokio task that writes `frontier_summary.json` and `stats_summary.json` to `play_log/<ts>/interception/` on shutdown.

### RCL Interception Architecture

`libplay_launch_interception.so` (LD_PRELOAD) hooks `rcl_publish`/`rcl_take` to extract `header.stamp` from messages. Compiled-in plugins (`FrontierPlugin`, `StatsPlugin`) write `InterceptionEvent` to a zero-copy SPSC ring buffer shared with play_launch.

Standalone crates (not in colcon workspace):
- `src/vendor/rcl_interception_sys/` — bindgen FFI types for rcl/rosidl
- `src/play_launch_interception/` — cdylib: hooks, registry, introspection, plugins
- `src/spsc_shm/` — generic SPSC ring buffer over shared memory (memfd + eventfd)

Env vars (set by play_launch, consumed by the .so):
- `PLAY_LAUNCH_INTERCEPTION_SHM_FD` — shared memory fd for ring buffer
- `PLAY_LAUNCH_INTERCEPTION_EVENT_FD` — eventfd for wakeup signaling
- `PLAY_LAUNCH_INTERCEPTION_SO` — override path to the .so (for tests)

Build: `just build-interception` (requires ROS environment). Design: `docs/roadmap/phase-29-rcl_interception.md`.

## Log Directory Structure

```
play_log/<timestamp>/
├── params_files/
├── interception/                   # when interception enabled
│   ├── frontier_summary.json       # per-topic frontier state
│   └── stats_summary.json          # per-topic pub/take counts, rates
├── system_stats.csv, diagnostics.csv
└── node/<node_name>/{metadata.json, metrics.csv, out, err, pid, status, cmdline}
```

Composable nodes don't have separate directories — metadata in parent container's `metadata.json`.

## Development Practices

### Building & Testing
- **ALWAYS** use `just build` (never `colcon build` directly)
- **ALWAYS** use Bash tool's `timeout` parameter (never `timeout` command prefix)
- Temp files in `tmp/` (gitignored), never `/tmp`
- Create temp scripts with Write tool, never inline in Bash
- **Standalone crates** (outside the workspace, with their own `[workspace]` in Cargo.toml) must have a `/target/` `.gitignore` — e.g. `play_launch_interception`, `rcl_interception_sys`, `spsc_shm`

### Process Management
- **NEVER** `kill -9` individual processes — kill the process group (PGID):
  ```bash
  PGID=$(ps -o pgid= -p $PID | tr -d ' '); kill -TERM -$PGID; sleep 2; kill -9 -$PGID
  ```
- Use `just kill-orphans` to clean up stray ROS processes

### Logging
- `error!`: unrecoverable | `warn!`: recoverable | `info!`: user-facing | `debug!`: technical
- Default `RUST_LOG=play_launch=info`. Enable debug: `RUST_LOG=play_launch=debug`
- **`info!` is only for end-users** — never promote `debug!` to `info!` just to make tests pass. Use `RUST_LOG` to control log visibility in tests instead (e.g. `cmd.env("RUST_LOG", "play_launch=debug")`)

## PyO3 and ROS2 Gotchas

- **Module functions**: use `module.getattr("fn")?.call0()?` (not `call_method0`)
- **launch_ros properties**: `target_container.expanded_node_namespace` (not a method call)
- **Node names**: check for leading `/` before prepending namespace to avoid `//`
- **SomeSubstitutionsType params**: accept `PyObject`, convert via `utils::pyobject_to_string()` — handles strings, substitutions, and lists
- **YAML substitutions**: always use `load_and_resolve_param_file()` (not raw `read_to_string`) to resolve `$(var ...)` and convert types

## Testing

```bash
just test              # Parser unit (413) + fast integration (6), ~3s
just test-all          # Parser unit (413) + all integration (42), ~70s
just test-unit         # Parser unit tests only
just test-integration  # All integration tests (simple + Autoware)
cargo test -p play_launch_wasm_runtime --test fixture_round_trip  # WASM round-trip (18 tests)
```

Two crates: parser unit tests (`src/play_launch_parser/`) and integration tests (`tests/`, excluded from workspace). Integration tests use `ManagedProcess` RAII guard (`tests/src/process.rs`) for guaranteed cleanup via `setsid()` + `PR_SET_PDEATHSIG` + PGID kill on Drop.

**DDS isolation**: `play_launch_cmd()` in `tests/src/fixtures.rs` assigns a unique `ROS_DOMAIN_ID` per invocation (PID + counter) so concurrent nextest processes don't cross-talk over DDS.

Test workspaces: `tests/fixtures/{autoware,simple_test,sequential_loading,concurrent_loading,container_events,parallel_loading}/` — each has `just dump-rust`, `just dump-python`, `just compare-dumps`.

## Key Recent Changes

- **2026-03-11**: Phase 29 (RCL Interception) integration complete — play_launch consumer wired into replay.rs, 5 integration tests passing (stats, frontier, disabled, defaults, toggles). Bug fix: `rcl_publish`/`rcl_take` hooks now fall back to `lookup_publisher_full()`/`lookup_subscription_full()` for messages without `header.stamp`, dispatching with `stamp: None` so StatsPlugin counts all messages.
- **2026-03-11**: Fixed `$(eval)` Python conditional expressions returning "false" — `needs_python_eval()` didn't recognize `" if "` / `" else "` keywords, so expressions like `'centerpoint_tiny' if ''=='' else ''` were handled by the Rust string comparison evaluator (which found `==` first), naively split on it, and returned "false". Fix: added `" if "` and `" else "` to the `needs_python_eval()` keywords list in `eval.rs` so conditional expressions delegate to Python eval.
- **2026-03-11**: Fixed composable node container matching — 5 nodes were failing to find their target containers:
  1. **`container.rs` (Python mock)**: `ComposableNodeContainer` was missing `#[getter]` for `name` and `namespace`, causing `LoadComposableNodes.extract_target_container()` to fall through to `__repr__()`, producing targets like `"ComposableNodeContainer(name='...', namespace='')"` instead of proper names. Fix: added `#[getter]` annotations.
  2. **`load_composable.rs` (Python mock)**: `extract_target_container()` wasn't combining ROS namespace (from `get_current_ros_namespace()`) with container's own namespace when building the target name. Fix: integrate ROS namespace using same logic as `capture_container()`.
  3. **`load_composable_node.rs` (XML path)**: `to_captures()` and `to_load_node_records()` were prepending `context.current_namespace()` to the resolved target name, but target is a reference by name, not a namespace-relative path. Fix: store raw resolved value. The builder handles matching via suffix fallback.
  4. **`builder.rs`**: Container matching used strict exact match only. When a relative target like `"pointcloud_container"` (no namespace prefix) normalized to `"/pointcloud_container"`, it wouldn't match a container at `"/pointcloud_container"` if the composable node was in a deep namespace. Fix: added suffix matching fallback — when exact match fails, find a container whose full name ends with `"/{target_name}"`.
- **2026-03-10**: Fixed two bugs affecting `<executable>` tags in launch XML:
  1. **`context.rs:to_exec_context()`**: Was bailing on all records where `exec_name` was `None`, but raw executables (from `<executable>` tags) legitimately have `package: None` and may lack `exec_name`. Fix: only require `exec_name` when `package` is `Some` (i.e., ROS nodes).
  2. **`record_conv.rs:into_record_json()`**: Global params backfill was injecting `-p key:=value` into `cmd` for ALL records in `self.records`, including raw executables. Raw executables don't understand ROS params, so these leaked params (e.g., `-p use_sim_time:=True`) were passed as arguments to non-ROS processes like CARLA. Fix: skip `-p` cmd injection when `node.package.is_none()`.
- **2026-03-03**: Rust 2024 edition migration — all Cargo.toml files updated to `edition = "2024"`. `std::env::set_var`/`remove_var` wrapped in `unsafe {}` (20 call sites). `clippy::collapsible_if` expanded to if-let chains; all ~120 instances fixed with `cargo clippy --fix`. Dead code cleanup: removed unused wrapper functions (`run_container`, `run_regular_node`), unused methods/fields, `MemberDetails` struct; kept semantic/test-used items with targeted `#[allow(dead_code)]`. colcon-cargo-ros2 0.4.0 migration — config now generated at `.cargo/config.toml` (auto-discovered by cargo) instead of `build/ros2_cargo_config.toml`. Removed `--config` flags from justfile (`check`, `generate-bindings`). Deleted `scripts/patch_cargo_config.sh` (colcon 0.4.0 generates `rosidl_runtime_rs = "0.6"` and patches `.cargo/config.toml` directly). Pinned `colcon-cargo-ros2>=0.4.0` in justfile `install-deps`, `docker/builder.Dockerfile`, and `.github/workflows/release-wheel.yml`.
- **2026-03-02**: Split `GraphView.js` (~2300 lines) into 6 modules in `web/assets/js/components/`: `graph-utils.js` (pure helpers), `graph-builders.js` (snapshot→Cytoscape elements), `graph-edges.js` (edge routing + port bundling), `graph-layout.js` (ELK integration + scrollbars), `graph-styles.js` (Cytoscape stylesheet), `GraphView.js` (component + event handlers). Public API unchanged (`export function GraphView`).
- **2026-03-01**: Phase 25 (Topic Introspection) — graph view animated ELK layout transitions, overlap fix, and SSE rebuild stability. `applyElkPositions` supports `animate` param (300ms slide via `ele.animation()`); initial load instant, subsequent layouts animated. `buildElkGraph` computes leaf/port dimensions from style formula (not stale `layoutDimensions()`). Branch edges use `haystack` curve-style to avoid invalid-endpoint warnings when port nodes overlap leaf nodes. SSE rebuild defers collapse via `setTimeout(0)` after `cy.json()` so expand-collapse extension can register new elements. `[GraphView]` console logs at all lifecycle points (init, update, rebuild, expand/collapse, layout, edges). New file: `GraphPanel.js`.
- **2026-02-27**: Phase 24 (Web UI Parameter Control) complete. ParameterProxy service client wrapper, ParamValue types with bidirectional ROS conversion, MemberHandle integration with FQN map, GET/POST `/api/nodes/:name/parameters` endpoints, SSE `/parameter_events` subscription, ParametersTab frontend with type-aware inputs (bool toggle, number/string inputs, range constraints), search/filter. New files: `ros/parameter_types.rs`, `ros/parameter_proxy.rs`, `web/assets/js/components/ParametersTab.js`.
- **2026-02-24**: Phase 22 (Launch Tree IR) phases 22.1–22.8 complete. IR preserves full launch structure (conditions, expressions, groups, includes). WASM pipeline: analyze → compile → execute produces identical output to direct parsing (18 round-trip tests). CLI integration behind `--features wasm`. Bugs fixed: `push-ros-namespace` `ns=` attribute in IR builder, `PopNamespace` added to IR, container namespace resolution in WASM host, load_node namespace extraction from target path.
- **2026-02-18**: Phase 21 (build optimization) mostly complete: `scripts/bundle_wheel.sh` with artifact manifest, incremental build recipes (`build-cpp`, `build-rust`, `build-wheel`), proper wheel platform tag via `wheel tags`. Phase 20 (web UI modernization) planned: Preact + htm + SSE-driven state, vendored locally (no CDN), zero polling.
- **2026-02-17**: Phase 19 complete — fork()+exec() isolation, child death monitoring, parallel loading, event-driven container status. PR_SET_CHILD_SUBREAPER added to replay. Subprocess PID cache uses time-based 1s interval.
- **2026-02-16**: Fix container parameter passing in fork+exec isolation: `write_params_file()` now serializes `request->parameters` (not `extra_arguments`), uses `/**:` wildcard YAML namespace, enforces decimal points on double arrays. Smoke test (`tests/src/health.rs`) detects `ComponentEvent LOAD_FAILED` events. `component_node` supports `--use-intra-process-comms`. Autoware: 64/64 composable nodes load successfully.
- **2026-02-15**: Default `--container-mode isolated` — replay overrides stock containers to `play_launch_container` with fork+exec isolation. `ComponentEvent` subscription conditional on mode (skipped for `stock`). Nextest failure output deferred to end (`--failure-output final`); isolated-container tests retry once for DDS flakes.
- **2026-02-08**: Phase 18 complete — ObservableComponentManager + play_launch_msgs packages.
- **2026-02-06**: Context unification investigation, namespace fix, YAML parameter loading for composable nodes. See `docs/roadmap/phase-17-context_unification.md`.
- **2026-02-01**: Phase 14 complete — Python launch file execution. Phase 15 — PyO3 type safety.
- **2026-01-27**: Phase 13 complete — Rust parser as default (3-12x speedup).

## Documentation

- **Roadmap**: `docs/roadmap/README.md` — all phases and progress
- **Container Design**: `docs/design/container-isolation.md` — why containers, Linux isolation, RMW consequences
- **Clone VM Design**: `docs/archive/clone-vm-container-design.md` — clone(CLONE_VM) implementation spec (archived)
- **Web UI Modernization**: `docs/roadmap/phase-20-web_ui_modernization.md` — Preact + SSE migration plan
- **Build Optimization**: `docs/roadmap/phase-21-build_optimization.md` — bundle script, incremental builds
- **Launch Tree IR**: `docs/roadmap/phase-22-launch_tree_ir.md` — IR design, WASM pipeline, validation
- **Topic Introspection**: `docs/roadmap/phase-25-topic_introspection.md` — graph view, namespace grouping, ELK layout, edge bundling
- **RCL Interception**: `docs/roadmap/phase-29-rcl_interception.md` — LD_PRELOAD interceptor, SPSC shared memory, frontier/stats plugins
- **Migration Guide**: `docs/guide/parser-migration.md` — Rust parser migration (v0.6.0+)

## Parser Parity Status

Entity counts match between parsers (100% functional equivalence). Remaining cosmetic differences: CMD deduplication, params_files expansion, CMD ordering, XML whitespace, array quoting, boolean case, extra_args defaults. See `docs/roadmap/phase-17-context_unification.md`.

## Distribution

PyPI: `pip install play_launch` (x86_64 + aarch64, Ubuntu 22.04+). Build: `just build`. Publish: `just publish-pypi`.
