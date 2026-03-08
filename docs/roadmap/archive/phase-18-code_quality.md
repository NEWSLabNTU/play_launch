# Phase 18: Code Quality Improvements

**Status**: ✅ Complete
**Priority**: Medium (maintainability and correctness)
**Dependencies**: Phase 17 (Context Unification) — complete first to avoid merge conflicts

## Overview

Code quality audit identified four categories of technical debt in the parser crate. These are non-functional improvements that reduce bug risk, improve maintainability, and clean up logging output. No behavioral changes to parser output.

## Scope

All work targets the parser crate: `src/play_launch_parser/src/play_launch_parser/src/`

---

## Phase 18.1: Remove Debug Logging Noise ✅

**Goal**: Clean user-facing log output by removing leftover debug statements and fixing log levels.

### 18.1.1: Remove `log::warn!("DEBUG: ...")` statements

11 leftover debug statements disguised as warnings:

| File                       | Line | Message                                               |
|----------------------------|------|-------------------------------------------------------|
| `actions/container.rs`     | 322  | `"DEBUG container.rs: Loading YAML file"`             |
| `python/api/launch_ros.rs` | 2106 | `"DEBUG extract_parameters: first 5 keys: ..."`       |
| `python/api/launch_ros.rs` | 2111 | `"DEBUG: single-key dict with key='...'"`             |
| `python/api/launch_ros.rs` | 2115 | `"DEBUG: value is string: ..."`                       |
| `python/api/launch_ros.rs` | 2117 | `"DEBUG: Loading YAML file: ..."`                     |
| `python/api/launch_ros.rs` | 2159 | `"DEBUG extract_parameters: item N is ParameterFile"` |
| `python/api/launch_ros.rs` | 2169 | `"DEBUG: is_yaml_file returned true"`                 |
| `python/api/launch_ros.rs` | 2192 | `"DEBUG: is_yaml_file returned false"`                |
| `python/api/launch_ros.rs` | 2201 | `"DEBUG: extract_parameters got string: ..."`         |
| `python/api/launch_ros.rs` | 2204 | `"DEBUG: is_yaml_file returned true"`                 |
| `python/api/launch_ros.rs` | 2222 | `"DEBUG: is_yaml_file returned false"`                |

**Action**: Remove all 11. If any provide useful trace information, downgrade to `log::trace!`.

### 18.1.2: Downgrade `log::info!` to `log::debug!`

Parser internals should not appear at `info` level. These are implementation details:

| File                       | Line | Message                                    | Reason              |
|----------------------------|------|--------------------------------------------|---------------------|
| `lib.rs`                   | 58   | `"Executing Python launch file: ..."`      | Internal dispatch   |
| `lib.rs`                   | 64   | `"Processing YAML launch file: ..."`       | Internal dispatch   |
| `python/executor.rs`       | 21   | `"Executing Python launch file: ..."`      | Duplicate of lib.rs |
| `traverser/include.rs`     | 48   | `"Including launch file: ..."`             | Internal traversal  |
| `traverser/include.rs`     | 61   | `"Including Python launch file: ..."`      | Internal traversal  |
| `traverser/include.rs`     | 79   | `"Including YAML launch file: ..."`        | Internal traversal  |
| `traverser/xml_include.rs` | 13   | `"Including XML launch file: ..."`         | Internal traversal  |
| `traverser/entity.rs`      | 283  | `"Skipping standalone composable_node..."` | Internal decision   |
| `traverser/entity.rs`      | 290  | `"Loaded N composable node(s)..."`         | Internal count      |
| `actions/node.rs`          | 80   | `"Found composable_node in container..."`  | Internal decision   |

**Keep at info** (user-facing CLI output):
- `main.rs:106` — `"Parsing launch file: ... from package ..."`
- `main.rs:119` — `"Parsing launch file: ..."`
- `main.rs:143` — `"Generated record.json: ..."`
- `main.rs:144` — `"  N nodes, N containers, N composable nodes"`

**Keep at info** (intentional user messages):
- `python/api/actions.rs:281` — `"Python Launch LogInfo: ..."` (forwards user's LogInfo action)
- `python/api/actions.rs:883` — `"Python Launch IncludeLaunchDescription: ..."` (user-visible include)

**Estimated effort**: Small (search-and-replace level changes)

---

## Phase 18.2: RAII Guards for Paired Operations ✅

**Goal**: Replace manual save/restore and set/clear pairs with RAII guards to prevent state corruption on early returns or panics.

### 18.2.1: Group Scope Guard (CRITICAL)

**File**: `traverser/entity.rs` lines 167-189

**Problem**: `traverse_entity(&child)?` can return early, skipping namespace and remapping restoration:
```rust
"group" => {
    let initial_depth = self.context.namespace_depth();       // SAVE
    let initial_remap_count = self.context.remapping_count(); // SAVE
    // ... push namespace ...
    for child in entity.children() {
        self.traverse_entity(&child)?;  // early return skips restore!
    }
    self.context.restore_namespace_depth(initial_depth);       // RESTORE
    self.context.restore_remapping_count(initial_remap_count); // RESTORE
}
```

**Solution**: `GroupScopeGuard` that restores both on drop:
```rust
struct GroupScopeGuard<'a> {
    context: &'a mut LaunchContext,
    namespace_depth: usize,
    remap_count: usize,
}
impl Drop for GroupScopeGuard<'_> {
    fn drop(&mut self) {
        self.context.restore_namespace_depth(self.namespace_depth);
        self.context.restore_remapping_count(self.remap_count);
    }
}
```

**Note**: Requires careful handling since `self.context` is borrowed mutably by the guard AND needed for `traverse_entity`. May need to use a closure-based approach instead:
```rust
fn with_group_scope<F>(&mut self, f: F) -> Result<()>
where F: FnOnce(&mut Self) -> Result<()>
{
    let depth = self.context.namespace_depth();
    let remap = self.context.remapping_count();
    let result = f(self);
    self.context.restore_namespace_depth(depth);
    self.context.restore_remapping_count(remap);
    result
}
```

### 18.2.2: Thread-Local Context Guard (HIGH)

**Files**: `python/bridge.rs` lines 345-362, `traverser/python_exec.rs` lines 51-61

**Problem**: Manual set/clear of `*mut LaunchContext` thread-local. If code between set and clear panics, the thread-local is left pointing to freed memory.

**Current**:
```rust
set_current_launch_context(&mut self.context);
let exec_result = executor.execute(path_str);
clear_current_launch_context();
```

**Solution**: `LaunchContextGuard` that clears on drop:
```rust
pub struct LaunchContextGuard;
impl LaunchContextGuard {
    pub fn new(ctx: &mut LaunchContext) -> Self {
        set_current_launch_context(ctx);
        Self
    }
}
impl Drop for LaunchContextGuard {
    fn drop(&mut self) { clear_current_launch_context(); }
}
```

### 18.2.3: Namespace Push Guard (MEDIUM)

**File**: `traverser/python_exec.rs` lines 152-159

**Problem**: Conditional push/pop around Python include execution:
```rust
if let Some(ref ns) = ros_ns {
    self.context.push_namespace(ns.clone());
}
let result = self.execute_python_file(...);
if ros_ns.is_some() {
    self.context.pop_namespace();
}
```

### 18.2.4: Resolution Depth Guard (LOW)

**File**: `substitution/context.rs` lines 200-227

**Problem**: Thread-local `RESOLUTION_DEPTH` counter incremented/decremented around recursive resolution. Panic during resolution leaves counter elevated.

**Estimated effort**: Medium (requires careful borrow-checker work for 18.2.1)

---

## Phase 18.3: Consolidate Duplicate Logic ✅

**Goal**: Extract shared functions for command building and parameter merging to eliminate inconsistencies.

### 18.3.1: Unify Command Array Building ✅

Extracted three shared functions in `record/generator.rs`:

- **`build_ros_command()`** — Unified command builder with canonical parameter ordering (matches Python parser): exec_path → args → --ros-args → __node → __ns → global params (normalized) → params_files → node params (normalized) → remappings
- **`resolve_exec_path()`** — Shared executable resolution (tries `find_package_executable()`, falls back to `/opt/ros/humble/lib/{pkg}/{exec}`)
- **`merge_params_with_global()`** — Global-first parameter merging with node-specific overrides by key

Refactored 6 callers:
- `CommandGenerator::build_node_command()` in `generator.rs` — delegates to `build_ros_command()`
- `NodeCapture::generate_command()` in `bridge.rs` — uses `build_ros_command()` (with ros2 run fallback for unresolved executables)
- `ContainerCapture::to_record()` in `bridge.rs` — uses `build_ros_command()` and `resolve_exec_path()`
- `ContainerAction::to_container_record()` in `container.rs` — uses `build_ros_command()` and `resolve_exec_path()` (**fixes missing normalize_param_value()**)
- `ContainerAction::to_node_record()` in `container.rs` — uses `build_ros_command()` and `resolve_exec_path()` (**fixes missing normalize_param_value()**)
- `ComposableNodeAction::to_load_node_record()` in `container.rs` — uses `merge_params_with_global()`
- `LoadNodeCapture::to_record()` in `bridge.rs` — uses `merge_params_with_global()`

### 18.3.2: Fix Backfill Normalization ✅

Added `normalize_param_value()` to the backfill loop in `record_conv.rs` (previously skipped normalization when inserting global params into cmd).

### 18.3.3: Param File Ordering Fix ✅

`bridge.rs` previously placed params_files **after** node params. Now uses canonical order (params_files **before** node params) matching Python parser behavior.

### 18.3.4: Capture vs Record Type Overlap

**Not recommended for this phase.** NodeCapture/NodeRecord, ContainerCapture/ContainerRecord, LoadNodeCapture/LoadNodeRecord have overlapping fields but serve different lifecycle stages (construction vs serialization). Unifying them would be a large refactor with limited benefit.

---

## Phase 18.4: Print Macro Cleanup ✅ (No changes needed)

**Goal**: Replace `println!`/`eprintln!` with structured logging where appropriate.

### Production Code Inventory

| File                            | Lines               | Macro       | Context                                  | Action                                  |
|---------------------------------|---------------------|-------------|------------------------------------------|-----------------------------------------|
| `commands/io_helper.rs`         | 31-32, 54-55, 79-91 | `println!`  | User-facing CLI output for setcap/verify | **Keep** (appropriate for CLI feedback) |
| `play_launch_io_helper/main.rs` | 61                  | `eprintln!` | Warning in binary entry point            | **Keep** (binary, no log setup)         |
| `play_launch_parser/main.rs`    | 110, 126            | `eprintln!` | Error reporting in CLI                   | **Keep** (appropriate for CLI errors)   |

**Result**: All print macros in production code are in CLI entry points where structured logging is not initialized or not appropriate. **No changes needed.**

**Estimated effort**: None

---

## Phase 18.5: Test Infrastructure ✅

**Goal**: Add process count verification and improve the comprehensive test script.

### 18.5.1: Process Count Helper Script ✅

New `scripts/count_processes.py`:
- Counts expected processes from `record.json` (`len(node) + len(container)`)
- Counts actual spawned processes from `play_log/latest/node/*/cmdline` files
- Uses `cmdline` (not `pid`) because containers don't write pid files
- Reports expected vs actual with PASS/FAIL exit code

### 18.5.2: Autoware Process Count Recipe ✅

New recipes in `test/autoware_planning_simulation/justfile`:
- `count-processes PARSER="rust"` — Dumps record, launches play_launch in background, polls cmdline files every 2s until count stabilizes (3 consecutive same readings) or 60s timeout, reports pass/fail
- `count-processes-rust` / `count-processes-python` — Convenience aliases
- Cleanup via process group kill (PGID) on exit

### 18.5.3: Revised Comprehensive Test Script ✅

Revised `tmp/run_all_checks.sh`:
- Helper functions (`run_stage`, `run_stage_launch`, `skip_stage`) for consistent formatting
- New Stage 5: Autoware process count (90s timeout)
- LCTK demo is now optional (skips if `~/repos/LCTK` doesn't exist)
- Summary table at end showing PASS/FAIL/SKIP for each stage

### Verification Results

Both parsers produce identical results on Autoware:

| Metric              | Rust Parser | Python Parser |
|---------------------|-------------|---------------|
| Expected processes  | 61          | 61            |
| Actual processes    | 61          | 61            |
| Result              | PASS        | PASS          |
| Dump time           | 0.26s       | 17.81s        |
| Time to 61 procs    | ~4s         | ~20s          |

---

## Implementation Order

1. **Phase 18.1** (logging cleanup) — Quick wins, zero risk, immediate noise reduction
2. **Phase 18.2** (RAII guards) — Safety improvements, moderate effort
3. **Phase 18.3** (dedup) — Consistency fixes, requires careful testing
4. **Phase 18.4** (print macros) — No action needed (already appropriate)
5. **Phase 18.5** (test infra) — Process counting and test script improvements

## Verification

- All existing tests must pass (311 unit + integration)
- Autoware comparison: 45/45 functionally equivalent (no behavioral changes)
- Autoware process count: 61/61 for both Rust and Python parsers
- `RUST_LOG=info` output should be minimal (only user-facing messages)
- `RUST_LOG=debug` output should show parser internals
- No `warn!("DEBUG: ...")` in output at any log level

## Files Modified

### Phase 18.1
- `actions/container.rs` — Remove 1 debug warn
- `python/api/launch_ros.rs` — Remove 10 debug warns
- `lib.rs` — Downgrade 2 info to debug
- `python/executor.rs` — Downgrade 1 info to debug
- `traverser/include.rs` — Downgrade 3 info to debug
- `traverser/xml_include.rs` — Downgrade 1 info to debug
- `traverser/entity.rs` — Downgrade 2 info to debug
- `actions/node.rs` — Downgrade 1 info to debug

### Phase 18.2
- `substitution/context.rs` — Add guard types (or new `guards.rs` module)
- `traverser/entity.rs` — Use GroupScopeGuard
- `python/bridge.rs` — Add LaunchContextGuard
- `traverser/python_exec.rs` — Use LaunchContextGuard and NamespacePushGuard

### Phase 18.3
- `record/generator.rs` — Extract `build_ros_command()`
- `python/bridge.rs` — Use shared command builder
- `actions/container.rs` — Use shared command builder, fix missing normalization
- `traverser/record_conv.rs` — Use shared command builder for backfill

### Phase 18.5
- `scripts/count_processes.py` — New process count helper
- `test/autoware_planning_simulation/justfile` — Add count-processes recipes
- `tmp/run_all_checks.sh` — Revised with new stage, optional LCTK, summary
