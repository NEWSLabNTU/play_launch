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

## Launch File Parsing

**Default**: Rust parser (`play_launch_parser`, 3-12x faster, 311 tests, 100% Autoware)
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

## Installation & Usage

```sh
just build                          # Build (ALWAYS use this, NEVER colcon directly)
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
- **Design docs**: `docs/container-isolation-design.md`, `docs/clone-vm-container-design.md`
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

## Log Directory Structure

```
play_log/<timestamp>/
├── params_files/
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
just test              # Parser unit (311) + fast integration (6), ~3s
just test-all          # Parser unit (311) + all integration (42), ~70s
just test-unit         # Parser unit tests only
just test-integration  # All integration tests (simple + Autoware)
```

Two crates: parser unit tests (`src/play_launch_parser/`) and integration tests (`tests/`, excluded from workspace). Integration tests use `ManagedProcess` RAII guard (`tests/src/process.rs`) for guaranteed cleanup via `setsid()` + `PR_SET_PDEATHSIG` + PGID kill on Drop.

**DDS isolation**: `play_launch_cmd()` in `tests/src/fixtures.rs` assigns a unique `ROS_DOMAIN_ID` per invocation (PID + counter) so concurrent nextest processes don't cross-talk over DDS.

Test workspaces: `tests/fixtures/{autoware,simple_test,sequential_loading,concurrent_loading,container_events,parallel_loading}/` — each has `just dump-rust`, `just dump-python`, `just compare-dumps`.

## Key Recent Changes

- **2026-02-16**: Fix container parameter passing in fork+exec isolation: `write_params_file()` now serializes `request->parameters` (not `extra_arguments`), uses `/**:` wildcard YAML namespace, enforces decimal points on double arrays. Smoke test (`tests/src/health.rs`) detects `ComponentEvent LOAD_FAILED` events. `component_node` supports `--use-intra-process-comms`. Autoware: 64/64 composable nodes load successfully.
- **2026-02-15**: Default `--container-mode isolated` — replay overrides stock containers to `play_launch_container` with fork+exec isolation. `ComponentEvent` subscription conditional on mode (skipped for `stock`). Nextest failure output deferred to end (`--failure-output final`); isolated-container tests retry once for DDS flakes.
- **2026-02-11**: Consolidated 5 container docs → 2 (`container-isolation-design.md` + `clone-vm-container-design.md`). Revised Phase 19 roadmap with Phase 19.0 (consolidate binary).
- **2026-02-08**: Phase 18 complete — ObservableComponentManager + play_launch_msgs packages.
- **2026-02-07**: Nextest integration test infrastructure with ManagedProcess RAII guard.
- **2026-02-06**: Context unification investigation, namespace fix, YAML parameter loading for composable nodes. See `docs/roadmap/phase-17-context_unification.md`.
- **2026-02-04**: Namespace normalization, runtime substitution resolution, `<let>` ordering test, debug logging cleanup.
- **2026-02-01**: Phase 14 complete — Python launch file execution. Phase 15 — PyO3 type safety.
- **2026-01-27**: Phase 13 complete — Rust parser as default (3-12x speedup).

## Documentation

- **Roadmap**: `docs/roadmap/README.md` — all phases and progress
- **Container Design**: `docs/container-isolation-design.md` — why containers, Linux isolation, RMW consequences
- **Clone VM Design**: `docs/clone-vm-container-design.md` — clone(CLONE_VM) implementation spec
- **Migration Guide**: `docs/parser-migration-guide.md` — Rust parser migration (v0.6.0+)

## Parser Parity Status

Entity counts match between parsers (100% functional equivalence). Remaining cosmetic differences: CMD deduplication, params_files expansion, CMD ordering, XML whitespace, array quoting, boolean case, extra_args defaults. See `docs/roadmap/phase-17-context_unification.md`.

## Distribution

PyPI: `pip install play_launch` (x86_64 + aarch64, Ubuntu 22.04+). Build: `just build`. Publish: `just publish-pypi`.
