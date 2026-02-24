# Phase 23: Code Quality

**Status**: Planned
**Priority**: Medium (technical debt reduction, maintainability)
**Scope**: Parser, WASM pipeline, and main CLI/runtime crates

Full audit of the scan→dump→runtime workflow. Findings organized into actionable work items.

Execution order: dead code first (smallest blast radius), file splits last (easiest after other cleanup).

---

## 23.1: Dead Code Removal

### Work items

#### Main crate

- [ ] Delete `ComposableNodeMetadata` struct (`execution/context.rs:40-54`, comment: "Phase 12: No longer used")
- [ ] Delete `ComposableNodeContext::to_load_node_command()` (`execution/context.rs:92-106`, comment: "Kept for potential future standalone loading")
- [ ] Delete `run_load_composable_node_via_service()` (`execution/spawn.rs:800`, deprecated — returns error only)
- [ ] Delete `composable_actors` field (`container_actor.rs:137`, comment: "will be removed in Phase 12")
- [ ] Delete `add_composable_actor()` method (`container_actor.rs:260`, deprecated in Phase 12)
- [ ] Remove `#![allow(dead_code)]` from `execution/spawn.rs:3` — audit individually, annotate or delete each truly dead item
- [ ] Audit dead config fields in `cli/config.rs` — `ComposableNodeLoadingSettings` and `ProcessConfig` methods marked "not yet connected"; annotate with `#[allow(dead_code)]` individually with justification comments, or delete

#### WASM crates

- [ ] Remove unused `WasmError` variants (`wasm_common/lib.rs`, 9 variants never constructed — runtime uses `anyhow::Result`). Keep only variants that are actually matched/constructed, or remove the enum entirely if unused.
- [ ] Remove `CommandPolicy` enum (`wasm_common/lib.rs`, defined but never consulted)
- [ ] Remove `memory::RETURN_AREA` and `RETURN_AREA_SIZE` (`wasm_common/lib.rs`, vestigial — codegen uses multi-value returns)

### Verification

- [ ] `cargo build --workspace --config build/ros2_cargo_config.toml` — no new warnings
- [ ] `cargo clippy --workspace --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [ ] `just test` — 353 parser tests pass
- [ ] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip --config build/ros2_cargo_config.toml` — 18 WASM tests pass
- [ ] `grep -r 'allow(dead_code)' src/play_launch/src/` — no blanket module-level suppressions remain

---

## 23.2: Code Deduplication

### Work items

#### High priority

- [ ] Extract `perform_obj` + `pyobject_to_string` wrappers from `python/api/substitutions.rs` (×8 identical copies) → shared function in `utils.rs` or a trait with default implementations
- [ ] Extract `to_bool()` helper from `python/api/substitutions.rs` (×3 copies in And/Or/IfElse) → shared function
- [ ] Extract `CleanupGuard` from `commands/replay.rs` and `commands/run.rs` (identical) → `commands/common.rs`
- [ ] Extract `forward_state_events_and_wait` from `commands/replay.rs` and `commands/run.rs` (identical) → `commands/common.rs`
- [ ] Extract Tokio runtime builder pattern from `replay.rs`, `run.rs`, `dump.rs` (×3, all `min(num_cpus, 8)` + `max_blocking = workers * 2`) → `build_runtime()` helper in `commands/common.rs`
- [ ] Unify package share resolution (`substitution/types.rs` and `python/api/substitutions.rs` — identical AMENT_PREFIX_PATH → ROS_DISTRO → distro-fallback algorithm) → single shared function
- [ ] Unify `find_package_share` distro fallback list (`types.rs`, `substitutions.rs`, `bridge.rs` — ×3 copies of `["jazzy","iron","humble","galactic","foxy"]`) → single `KNOWN_ROS_DISTROS` constant

#### Medium priority

- [ ] Extract env-merge + global-params collection in WASM host (`host.rs` `end_node`/`end_executable`/`end_container` — ×3 near-identical blocks) → helper methods on `LaunchHost`
- [ ] Extract service client cleanup pattern in `container_actor.rs` (×4: `load_client = None; unload_client = None; ...`) → `clear_ros_clients(&mut self)` method
- [ ] Extract process registry unregister pattern in `container_actor.rs` (×4: `if let Some(ref registry) = ... { reg.remove(&pid) }`) → `unregister_process(&self, pid)` method
- [ ] Replace `create_*_parameter` boilerplate in `ros/component_loader.rs` (×8 nearly identical functions differing only in which field is set) → macro or generic helper

### Verification

- [ ] `cargo build --workspace --config build/ros2_cargo_config.toml` — compiles cleanly
- [ ] `cargo clippy --workspace --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [ ] `just test` — 353 parser tests pass
- [ ] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip --config build/ros2_cargo_config.toml` — 18 WASM tests pass
- [ ] `just test-all` — all 353 tests pass (integration tests exercise runtime builder, CleanupGuard, package resolution)
- [ ] No duplicate function bodies remain: `grep -c 'fn perform_obj' src/play_launch_parser/` returns 1

---

## 23.3: Magic Numbers → Named Constants

### Work items

#### Timeouts (highest priority — should become configurable via config YAML)

- [ ] `30s` in `container_actor.rs` (3 places) → `const SERVICE_CALL_TIMEOUT: Duration`
- [ ] `5s` in `container_actor.rs`, `run.rs` → `const GRACEFUL_SHUTDOWN_TIMEOUT: Duration`
- [ ] `10s` in `replay.rs` → `const PERIODIC_STATS_INTERVAL: Duration`
- [ ] `2s` in `replay.rs`, `io_helper_client.rs` → `const CLEANUP_TIMEOUT: Duration`
- [ ] `100ms` in `container_actor.rs`, `run.rs`, `spawn.rs` → `const POLL_INTERVAL: Duration`
- [ ] `200ms` in `container_actor.rs` → `const POST_SERVICE_READY_WARMUP: Duration`
- [ ] `3s` in `web/sse.rs` → `const SSE_KEEPALIVE_INTERVAL: Duration`
- [ ] `50ms` in `replay.rs` → `const ROS_EXECUTOR_SPIN_TIMEOUT: Duration`

#### Buffer sizes and limits

- [ ] `8` in `replay.rs`, `run.rs`, `dump.rs` (×3) → `const MAX_WORKER_THREADS: usize` (already deduplicated via `build_runtime()` in 23.2)
- [ ] `100` in `coordinator.rs` → `const STATE_EVENT_CHANNEL_SIZE: usize`
- [ ] `10` in `coordinator.rs` → `const CONTROL_EVENT_CHANNEL_SIZE: usize`
- [ ] `10 * 1024` in `coordinator.rs` → `const NOISY_STDERR_THRESHOLD: usize`
- [ ] `1024 * 1024` in `io_helper_client.rs` → `const MAX_IPC_RESPONSE_SIZE: usize`
- [ ] `1024` in `substitution/parser.rs` → `const SUBSTITUTION_CACHE_SIZE: usize`

#### ROS constants (duplicated across crates — use single definition)

- [ ] `"rclcpp_components"` in `bridge.rs`, `container.rs` → `const DEFAULT_CONTAINER_PACKAGE: &str`
- [ ] `"component_container"` in `bridge.rs`, `container.rs` → `const DEFAULT_CONTAINER_EXECUTABLE: &str`
- [ ] `["jazzy","iron","humble","galactic","foxy"]` in `types.rs`, `substitutions.rs`, `bridge.rs` → `const KNOWN_ROS_DISTROS: &[&str]` (already unified in 23.2)
- [ ] `/opt/ros/humble/...` in `generator.rs`, `launch_dump.rs`, `io_helper_client.rs`, `linker.rs` → remove hardcoded distro; use ament index or `ROS_DISTRO` env var

#### WASM constants

- [ ] `0x10000` in `compiler.rs`, `memory.rs` → `pub const WASM_PAGE_SIZE: usize` in `wasm_common::memory`
- [ ] `-1` sentinel in `compiler.rs`, `expr.rs` → `pub const NO_VALUE_SENTINEL: i32` in `wasm_common::memory`

### Verification

- [ ] `cargo build --workspace --config build/ros2_cargo_config.toml` — compiles cleanly
- [ ] `cargo clippy --workspace --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [ ] `just test` — 353 parser tests pass
- [ ] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip --config build/ros2_cargo_config.toml` — 18 WASM tests pass
- [ ] `grep -rn '0x10000' src/play_launch_wasm_codegen/ src/play_launch_wasm_runtime/` — no bare hex literals for page size
- [ ] `grep -rn '/opt/ros/humble' src/` — zero matches (all hardcoded paths removed)
- [ ] `grep -rn 'Duration::from_secs(30)' src/play_launch/src/` — zero matches (all replaced by named constants)

---

## 23.4: Unsafe Code Consolidation

### Work items

#### Parser crate — 3 unsafe blocks → 1

All three dereference a thread-local `*mut LaunchContext` for PyO3 interop:
- `python/bridge.rs:343` — `with_launch_context()` (central access point) — **keep**
- `python/api/actions.rs:953` — inline dereference — **route through `with_launch_context()`**
- `python/api/utils.rs:162` — inline dereference — **route through `with_launch_context()`**

- [ ] Refactor `actions.rs:953` to use `with_launch_context(|ctx| ctx.set_configuration(name_str, value_str))`
- [ ] Refactor `utils.rs:162` to use `with_launch_context(|ctx| ...)` for immutable context access
- [ ] Verify no other direct `get_current_launch_context()` + raw pointer dereference patterns exist

#### Main crate — no action needed

10 unsafe blocks for system-level operations (`prctl`, `setpgid`, `sched_setaffinity`, `setpriority`, pipe management). All necessary, all with correct safety comments.

#### WASM crates — no action needed

Zero unsafe blocks.

### Verification

- [ ] `grep -rn 'unsafe' src/play_launch_parser/` — exactly 1 match (in `with_launch_context()`)
- [ ] `cargo build --workspace --config build/ros2_cargo_config.toml` — compiles cleanly
- [ ] `just test` — 353 parser tests pass (Python execution paths exercise the refactored code)

---

## 23.5: Naming Improvements

### Work items

#### Parser crate

- [ ] `Expr(pub Vec<Substitution>)` in `ir.rs:13` — tuple struct accessed as `.0` everywhere → add named field `pub subs: Vec<Substitution>` or accessor method `fn subs(&self) -> &[Substitution]`; update all `.0` call sites
- [ ] `get_attr("name", false)` in `xml/entity.rs` — boolean param unclear at call sites → split into `get_required_attr(name)` / `get_optional_attr(name)`; update all call sites
- [ ] `params_remaps_to_action` in `ir_evaluator.rs:432` — name says "params_remaps" but only converts remaps → rename to `remaps_to_action`

#### WASM crates

- [ ] `CompNodeBuilder` in `host.rs:55` — abbreviated unlike siblings (`NodeBuilder`, `ContainerBuilder`) → rename to `ComposableNodeBuilder`
- [ ] `SET_COMP_*` constants in `wasm_common/lib.rs` — `COMP` inconsistent with full-word siblings → rename to `COMP_NODE_*` or `COMPOSABLE_NODE_*`
- [ ] `records: Vec<NodeRecord>` in `host.rs:78` — generic name; siblings are `containers`, `load_nodes` → rename to `nodes`
- [ ] `ExecBuilder` in `host.rs:36` — ambiguous ("exec" = executable? execution?) → rename to `ExecutableBuilder`
- [ ] `eval_simple_expr` in `linker.rs:817` — undersells what it does (Python-style expression eval) → rename to `eval_python_expr_simple`

#### Main crate

- [ ] `component_loader.rs` module in `ros/` — only contains parameter conversion, not loading → rename to `parameter_conversion.rs`
- [ ] `handle_toggle_composable_auto_load` doc in `container_actor.rs:1098` — says "Handle UnloadAllComposables" (copy-paste error) → fix doc comment
- [ ] `ContainerActor::new()` in `container_actor.rs:211` — 11 parameters, `#[allow(too_many_arguments)]` → introduce `ContainerActorConfig` struct

### Verification

- [ ] `cargo build --workspace --config build/ros2_cargo_config.toml` — compiles cleanly
- [ ] `cargo clippy --workspace --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [ ] `just test` — 353 parser tests pass
- [ ] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip --config build/ros2_cargo_config.toml` — 18 WASM tests pass
- [ ] `grep -rn '\.0\b' src/play_launch_parser/.../ir_evaluator.rs` — no bare `.0` tuple access on `Expr`
- [ ] `grep -rn 'CompNodeBuilder' src/play_launch_wasm_runtime/` — zero matches
- [ ] `grep -rn 'too_many_arguments' src/play_launch/src/member_actor/container_actor.rs` — zero matches

---

## 23.6: Structural Issues

### Work items

- [ ] `import_signature()` in `compiler.rs:869` panics on unknown imports → return `Result` or use an enum for compile-time exhaustiveness
- [ ] Builder setters in `linker.rs` (all `SET_*` handlers) silently no-op when no active builder → `anyhow::bail!` instead of silent `if let Some(...)`
- [ ] `save_scope` doc in `wasm_common/lib.rs:108` says returns `scope_id: i32` but ABI returns nothing → fix doc comment to match actual ABI
- [ ] `EntityExt::children()` blanket impl in parser `xml/entity.rs` returns empty Vec, conflicts with `XmlEntity::children()` inherent method → document the shadowing risk or refactor trait hierarchy
- [ ] `mem::forget(executor_thread)` in `replay.rs:306` to avoid blocking on join → drop the `JoinHandle` instead (dropping a `std::thread::JoinHandle` does not join)
- [ ] `CLK_TCK` hardcoded as `100.0` in `resource_monitor.rs` (×2 duplicated in same file) → single module-level `const CLK_TCK: f64` or query `sysconf(_SC_CLK_TCK)` at startup

### Verification

- [ ] `cargo build --workspace --config build/ros2_cargo_config.toml` — compiles cleanly
- [ ] `cargo clippy --workspace --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [ ] `just test` — 353 parser tests pass
- [ ] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip --config build/ros2_cargo_config.toml` — 18 WASM tests pass
- [ ] `grep -rn 'mem::forget' src/play_launch/src/` — zero matches
- [ ] `grep -rn 'panic!' src/play_launch_wasm_codegen/` — zero matches in non-test code

---

## 23.7: Large File Splits

### Work items

#### Parser crate (`play_launch_parser`)

- [ ] `python/api/launch_ros.rs` (2607 lines) → split into `python/api/launch_ros/{node,lifecycle_node,container,load_composable,push_namespace}.rs` with `mod.rs` re-exporting
- [ ] `python/api/substitutions.rs` (1549 lines) → split into `python/api/substitutions/{conditional,lookup,package}.rs` with `mod.rs` re-exporting
- [ ] `python/api/actions.rs` (1350 lines) → split into `python/api/actions/{declare_argument,group,include,opaque_function,...}.rs` with `mod.rs` re-exporting
- [ ] `substitution/types.rs` (1358 lines) → extract expression evaluator (~250 lines) to `substitution/eval.rs`

#### Main crate (`play_launch`)

- [ ] `member_actor/container_actor.rs` (2187 lines) → extract `service_calls.rs` (LoadNode/UnloadNode, ~260 lines), `component_events.rs` (event handling, ~200 lines); refactor `handle_running` (345-line `tokio::select!` with 7 branches) into per-branch methods
- [ ] `monitoring/resource_monitor.rs` (1344 lines) → extract `/proc` parsing to `proc_parser.rs`, CSV writing to `csv_writer.rs`, system stats to `system_stats.rs`
- [ ] `commands/replay.rs` (1197 lines) → extract signal handling to `signal_handler.rs` (shared code already in `common.rs` from 23.2)
- [ ] `member_actor/coordinator.rs` (1161 lines) → consider splitting builder/handle/runner into submodules

#### WASM crates

- [ ] `wasm_codegen/compiler.rs` (871 lines) → extract `import_signature()` lookup table; consider splitting string collection from action compilation
- [ ] `wasm_runtime/linker.rs` (836 lines) → split `register_imports()` by domain (context ops, node builder, container builder, composable node builder)

### Verification

- [ ] `cargo build --workspace --config build/ros2_cargo_config.toml` — compiles cleanly
- [ ] `cargo clippy --workspace --all-targets --config build/ros2_cargo_config.toml -- -D warnings` — clean
- [ ] `just test` — 353 parser tests pass
- [ ] `cargo test -p play_launch_wasm_runtime --test fixture_round_trip --config build/ros2_cargo_config.toml` — 18 WASM tests pass
- [ ] No source file in `src/play_launch/src/` exceeds 800 lines: `wc -l src/play_launch/src/**/*.rs | sort -rn | head -5`
- [ ] No source file in parser crate exceeds 1000 lines (non-test): `wc -l src/play_launch_parser/.../**/*.rs | sort -rn | head -5`

---

## Implementation Order

```
23.1  Dead code removal                                    planned
23.2  Code deduplication                                   planned
23.3  Magic numbers → named constants                      planned
23.4  Unsafe code consolidation                            planned
23.5  Naming improvements                                  planned
23.6  Structural issues                                    planned
23.7  Large file splits                                    planned
```
