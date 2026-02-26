# Phase 23: Code Quality

**Status**: Complete (23.1–23.7)
**Priority**: Medium (technical debt reduction, maintainability)
**Scope**: Parser, WASM pipeline, and main CLI/runtime crates

Full audit of the scan→dump→runtime workflow. Findings organized into actionable work items.

Execution order: dead code first (smallest blast radius), file splits last (easiest after other cleanup).

---

## 23.1: Dead Code Removal

### Work items

#### Main crate

- [x] Delete `ComposableNodeMetadata` struct (`execution/context.rs`)
- [x] Delete `ComposableNodeContext::to_load_node_command()` and `to_standalone_node_command()`
- [x] Delete `ComposableActorHandle` struct, `composable_actors` field, `add_composable_actor()`, `wait_for_composable_actors()`
- [x] Delete entire `execution/spawn.rs` module (892 lines — entire module was dead, replaced by actor pattern)
- [x] Delete `ComposableNodeRecord` command-line methods in `launch_dump.rs` (revealed dead by spawn.rs deletion)
- [x] Remove dead `NodeContainerContext.node_container_name` field (only set, never read)
- [x] Clean up `member_actor/mod.rs` re-exports (remove 14 unused re-exports, keep 7 used ones)
- [x] Fix revealed unused imports in `context.rs`, `coordinator.rs`, `regular_node_actor.rs`, `launch_dump.rs`
- [x] Add accurate `#![allow(dead_code)]` comment on `member_actor/mod.rs` (trait-based dispatch, not stale code)
- [x] Add `#![allow(dead_code)]` on `container_readiness.rs` (infrastructure spawned but query methods orphaned)

#### WASM crates

- [x] Remove entire `WasmError` enum and `Result<T>` type alias (`wasm_common/lib.rs` — runtime uses `anyhow::Result`)
- [x] Remove `CommandPolicy` enum (`wasm_common/lib.rs` — never consulted)
- [x] Remove `memory::RETURN_AREA` and `RETURN_AREA_SIZE` (`wasm_common/lib.rs` — vestigial, multi-value returns used)
- [x] Remove `thiserror` dependency from `wasm_common/Cargo.toml`
- [x] Delete 3 associated tests (`test_memory_layout_no_overlap`, `test_command_policy_default`, `test_error_display`)

### Verification

- [x] `cargo clippy -p play_launch --config build/ros2_cargo_config.toml -- -D warnings` — zero warnings
- [x] `cargo clippy -p play_launch_wasm_common -p play_launch_wasm_codegen -p play_launch_wasm_runtime --config build/ros2_cargo_config.toml -- -D warnings` — zero warnings
- [x] `just test` — 353 parser + 30 integration tests pass
- [x] `cargo test -p play_launch_wasm_common` — 5 tests pass

---

## 23.2: Code Deduplication

### Work items

#### High priority

- [x] Extract `perform_obj` + `pyobject_to_string` wrappers from `python/api/substitutions.rs` (×8 identical copies) → `perform_or_to_string()` in `utils.rs`
- [x] Extract `to_bool()` helper from `python/api/substitutions.rs` (×3 copies in And/Or/IfElse) → `pyobject_to_bool()` in `utils.rs`
- [x] Extract `CleanupGuard` from `commands/replay.rs` and `commands/run.rs` (identical) → `commands/common.rs`
- [x] Extract `forward_state_events_and_wait` from `commands/replay.rs` and `commands/run.rs` (identical) → `commands/common.rs`
- [x] Extract Tokio runtime builder pattern from `replay.rs`, `run.rs`, `dump.rs` (×4) → `build_tokio_runtime()` in `commands/common.rs`
- [x] Unify package share resolution (`substitution/types.rs` and `python/api/substitutions.rs`) → `FindPackageShare` delegates to `types::find_package_share()` (cached)
- [x] Unify `find_package_share` distro fallback list (`types.rs`, `substitutions.rs`, `bridge.rs` — ×3 copies) → `KNOWN_ROS_DISTROS` constant in `substitution/types.rs`

#### Medium priority

- [x] Extract env-merge + global-params collection in WASM host (`host.rs`) → `collect_global_params()` + `collect_merged_env()` on `LaunchHost`
- [x] Extract service client cleanup pattern in `container_actor.rs` (×4) → `clear_ros_clients(&mut self)` method
- [x] Extract process registry register/unregister pattern in `container_actor.rs` (×5: 1 register + 4 unregister) → `register_process(&self, pid)` + `unregister_process(&self, pid)` methods
- [x] Replace `create_*_parameter` boilerplate in `ros/component_loader.rs` (×8) → `create_parameter_value!` macro + `default_parameter_value()` helper

### Verification

- [x] `cargo clippy` — zero warnings (parser + main + WASM crates)
- [x] `just test` — 353 parser + 30 integration tests pass
- [x] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip` — 18 WASM tests pass

---

## 23.3: Magic Numbers → Named Constants

### Work items

#### Timeouts (highest priority — should become configurable via config YAML)

- [x] `30s` in `container_actor.rs` (3 places) → `const SERVICE_CALL_TIMEOUT: Duration`
- [x] `5s` in `run.rs` → `const GRACEFUL_SHUTDOWN_TIMEOUT: Duration`
- [x] `10s` in `container_actor.rs` → `const LOADING_TIMEOUT: Duration`; in `replay.rs` → `const PROGRESS_INTERVAL: Duration`
- [x] `2s` in `replay.rs` → `const CLEANUP_TIMEOUT: Duration`; `container_actor.rs` → `const SERVICE_NOT_READY_LOG_INTERVAL`; `io_helper_client.rs` → `const HELPER_SHUTDOWN_TIMEOUT`
- [x] `100ms` in `container_actor.rs` → `const SERVICE_POLL_INTERVAL`; `run.rs` → `const SHUTDOWN_POLL_INTERVAL`; `replay.rs` → `const COMPLETION_CHECK_INTERVAL`; `io_helper_client.rs` → `const HELPER_INIT_DELAY`
- [x] `200ms` in `container_actor.rs` → `const POST_SERVICE_READY_WARMUP: Duration`; `replay.rs` → `const SIGNAL_DEBOUNCE`; `cleanup.rs` → `const TERMINATION_GRACE_PERIOD`
- [x] `3s` in `web/sse.rs` → `const SSE_KEEPALIVE_INTERVAL: Duration`
- [x] `50ms` in `replay.rs` → `const EXECUTOR_SPIN_TIMEOUT: Duration`
- [x] `5s` in `container_actor.rs` → `const LOADING_CHECK_INTERVAL: Duration`

#### Buffer sizes and limits

- [x] `8` in `commands/common.rs` → `const MAX_WORKER_THREADS: usize` (already deduplicated via `build_tokio_runtime()` in 23.2)
- [x] `100` in `coordinator.rs` → `const STATE_EVENT_CHANNEL_SIZE: usize`; `broadcaster.rs` → `const SSE_SUBSCRIBER_CHANNEL_SIZE`
- [x] `10` in `coordinator.rs` → `const CONTROL_CHANNEL_SIZE: usize`
- [x] `10 * 1024` in `coordinator.rs` → `const NOISY_STDERR_THRESHOLD: u64`
- [x] `1024 * 1024` in `io_helper_client.rs` → `const MAX_IPC_RESPONSE_SIZE: usize`
- [x] `1024` in `substitution/parser.rs` → `const SUBSTITUTION_CACHE_SIZE: usize`

#### ROS constants (duplicated across crates — use single definition)

- [x] `"rclcpp_components"` in `bridge.rs`, `container.rs` → `const DEFAULT_CONTAINER_PACKAGE: &str` in `actions/container.rs`
- [x] `"component_container"` in `bridge.rs`, `container.rs` → `const DEFAULT_CONTAINER_EXECUTABLE: &str` in `actions/container.rs`
- [x] `["jazzy","iron","humble","galactic","foxy"]` in `types.rs`, `substitutions.rs`, `bridge.rs` → `const KNOWN_ROS_DISTROS: &[&str]` (already unified in 23.2)
- [x] `/opt/ros/humble/...` in `io_helper_client.rs` → use `ROS_DISTRO` env var with humble fallback

#### WASM constants

- [x] `0x10000` in `compiler.rs` → `memory::WASM_PAGE_SIZE` in `wasm_common::memory`
- [x] `-1` sentinel in `compiler.rs`, `expr.rs` → `pub const NO_VALUE_SENTINEL: i32` in `wasm_common`

### Verification

- [x] `cargo clippy -p play_launch --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [x] `cargo clippy -p play_launch_wasm_common -p play_launch_wasm_codegen -p play_launch_wasm_runtime --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [x] `cargo clippy -p play_launch_parser --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [x] `just test` — 353 parser + 30 integration tests pass
- [x] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip` — 18 WASM tests pass

---

## 23.4: Unsafe Code Consolidation

### Work items

#### Parser crate — 3 unsafe blocks → 1

All three dereference a thread-local `*mut LaunchContext` for PyO3 interop:
- `python/bridge.rs:343` — `with_launch_context()` (central access point) — **keep**
- `python/api/actions.rs:953` — inline dereference — **route through `with_launch_context()`**
- `python/api/utils.rs:162` — inline dereference — **route through `with_launch_context()`**

- [x] Refactor `actions.rs:953` to use `try_with_launch_context(|ctx| ctx.set_configuration(...))`
- [x] Refactor `utils.rs:162` to use `try_with_launch_context(|ctx| ctx.get_configuration(...))`
- [x] Verify no other direct `get_current_launch_context()` + raw pointer dereference patterns exist

#### Main crate — no action needed

10 unsafe blocks for system-level operations (`prctl`, `setpgid`, `sched_setaffinity`, `setpriority`, pipe management). All necessary, all with correct safety comments.

#### WASM crates — no action needed

Zero unsafe blocks.

### Verification

- [x] `grep -rn 'unsafe' src/play_launch_parser/` — exactly 2 matches (both in `bridge.rs`: `with_launch_context()` + `try_with_launch_context()`)
- [x] `cargo clippy -p play_launch_parser -- -D warnings` — clean
- [x] `just test` — 353 parser + 30 integration tests pass (Python execution paths exercise the refactored code)

---

## 23.5: Naming Improvements

### Work items

#### Parser crate

- [x] `Expr(pub Vec<Substitution>)` in `ir.rs` — tuple struct → named struct `Expr { pub parts: Vec<Substitution> }` with `Expr::new()` constructor; all `.0` → `.parts`
- [x] `get_attr("name", false/true)` in `xml/entity.rs` — boolean param → `required_attr(name)` / `optional_attr(name)` + `required_attr_str` / `optional_attr_str`; 72 call sites updated
- [x] `params_remaps_to_action` in `ir_evaluator.rs` → `remaps_to_action`

#### WASM crates

- [x] `CompNodeBuilder` in `host.rs` → `ComposableNodeBuilder`
- [x] `SET_COMP_*` / `ADD_COMP_*` constants → `SET_COMP_NODE_*` / `ADD_COMP_NODE_*` (string ABI values unchanged)
- [x] `records: Vec<NodeRecord>` in `host.rs` → `nodes`
- [x] `ExecBuilder` in `host.rs` → `ExecutableBuilder`
- [x] `eval_simple_expr` in `linker.rs` → `eval_python_expr_simple`

#### Main crate

- [x] `component_loader.rs` module in `ros/` → renamed to `parameter_conversion.rs`
- [x] `handle_toggle_composable_auto_load` doc in `container_actor.rs` — fixed copy-paste error ("UnloadAllComposables" → "ToggleComposableAutoLoad")
- [x] `ContainerActor::new()` — 11 parameters → `ContainerActorParams` struct (6 params remaining, under clippy threshold); `#[allow(too_many_arguments)]` removed from both `new()` and `run_container()`

### Verification

- [x] `cargo clippy --workspace --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [x] `just test` — 353 parser + 30 integration tests pass
- [x] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip --config build/ros2_cargo_config.toml` — 18 WASM tests pass

---

## 23.6: Structural Issues

### Work items

- [x] `import_signature()` in `compiler.rs` — panic on unknown imports → returns `anyhow::Result`; all compile methods (`compile_action`, `compile_expr`, etc.) now return `Result<()>` with `?` propagation
- [x] Builder setters in `linker.rs` (27 `SET_*`/`ADD_*` handlers) — silent `if let Some(...)` no-op → `.context("... called without active builder")?` via `anyhow::Context`
- [x] `save_scope`/`restore_scope` docs in `wasm_common/lib.rs` — fixed: both are stack-based `() -> void`, not scope_id-based
- [x] `EntityExt::children()` blanket impl in `xml/entity.rs` — removed dead trait method entirely (never called through trait; `XmlEntity::children()` inherent method is the sole call target)
- [x] `mem::forget(executor_thread)` in `replay.rs` → `drop(executor_thread)` (dropping a `JoinHandle` detaches without joining)
- [x] `CLK_TCK` in `resource_monitor.rs` — 2 local consts → single module-level `const CLK_TCK: f64 = 100.0`

### Verification

- [x] `cargo clippy --workspace --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [x] `just test` — 353 parser + 30 integration tests pass
- [x] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip --config build/ros2_cargo_config.toml` — 18 WASM tests pass
- [x] `grep -rn 'mem::forget' src/play_launch/src/` — zero matches
- [x] `grep -rn 'panic!' src/play_launch_wasm_codegen/` — zero matches in non-test code

---

## 23.7: Large File Splits

### Work items

#### Parser crate (`play_launch_parser`)

- [x] `python/api/launch_ros.rs` (2607 lines) → `launch_ros/` directory: `mod.rs` (292), `node.rs` (537), `container.rs` (304), `composable_node.rs` (370), `lifecycle_node.rs` (431), `load_composable.rs` (508), `push_namespace.rs` (85), `helpers.rs` (150)
- [x] `python/api/substitutions.rs` (1221 lines) → `substitutions/` directory: `mod.rs` (177), `conditional.rs` (296), `lookup.rs` (254), `package.rs` (543)
- [x] `python/api/actions.rs` (1348 lines) → `actions/` directory: `mod.rs` (24), `declare_argument.rs` (86), `opaque_function.rs` (168), `group.rs` (101), `include.rs` (204), `simple_actions.rs` (353), `configuration.rs` (453)
- [x] `substitution/types.rs` (1361→991 lines) → extracted expression evaluator (378 lines) to `substitution/eval.rs`

#### Main crate (`play_launch`)

- [x] `member_actor/container_actor.rs` (2136→619 lines) → `container_actor/` directory: `mod.rs` (619), `service_calls.rs` (303), `component_events.rs` (294), `composable_nodes.rs` (593), `process_lifecycle.rs` (461)
- [x] `monitoring/resource_monitor.rs` (1345→600 lines) → extracted `proc_parser.rs` (172), `csv_writer.rs` (387), `system_stats.rs` (273)
- [x] `commands/replay.rs` (1137→631 lines) → extracted `signal_handler.rs` (519)
- [x] `member_actor/coordinator.rs` (1174 lines) → `coordinator/` directory: `mod.rs` (59), `builder.rs` (486), `handle.rs` (568), `runner.rs` (121)

#### WASM crates

- [x] `wasm_codegen/compiler.rs` (877→801 lines) → extracted `signatures.rs` (81 lines) with `import_signature()` lookup table
- [x] `wasm_runtime/linker.rs` (836→865 lines) → split `register_imports()` into 7 domain-specific sub-functions (context ops, resolution, node/exec/container/composable-node/load-node builders)

### Verification

- [x] `cargo clippy --workspace --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [x] `just test` — 353 parser + 30 integration tests pass
- [x] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip --config build/ros2_cargo_config.toml` — 18 WASM tests pass
- [x] All main crate files ≤800 lines (largest: `replay.rs` 631, `container_actor/mod.rs` 619, `resource_monitor.rs` 600)
- [x] All parser crate files ≤1000 lines (largest: `types.rs` 991)

---

## Implementation Order

```
23.1  Dead code removal                                    complete
23.2  Code deduplication                                   complete
23.3  Magic numbers → named constants                      complete
23.4  Unsafe code consolidation                            complete
23.5  Naming improvements                                  complete
23.6  Structural issues                                    complete
23.7  Large file splits                                    complete
```
