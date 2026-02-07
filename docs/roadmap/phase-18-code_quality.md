# Phase 18: Code Quality Improvements

**Status**: ⏳ Planned
**Priority**: Medium (maintainability and correctness)
**Dependencies**: Phase 17 (Context Unification) — complete first to avoid merge conflicts

## Overview

Code quality audit identified four categories of technical debt in the parser crate. These are non-functional improvements that reduce bug risk, improve maintainability, and clean up logging output. No behavioral changes to parser output.

## Scope

All work targets the parser crate: `src/play_launch_parser/src/play_launch_parser/src/`

---

## Phase 18.1: Remove Debug Logging Noise

**Goal**: Clean user-facing log output by removing leftover debug statements and fixing log levels.

### 18.1.1: Remove `log::warn!("DEBUG: ...")` statements

11 leftover debug statements disguised as warnings:

| File | Line | Message |
|------|------|---------|
| `actions/container.rs` | 322 | `"DEBUG container.rs: Loading YAML file"` |
| `python/api/launch_ros.rs` | 2106 | `"DEBUG extract_parameters: first 5 keys: ..."` |
| `python/api/launch_ros.rs` | 2111 | `"DEBUG: single-key dict with key='...'"` |
| `python/api/launch_ros.rs` | 2115 | `"DEBUG: value is string: ..."` |
| `python/api/launch_ros.rs` | 2117 | `"DEBUG: Loading YAML file: ..."` |
| `python/api/launch_ros.rs` | 2159 | `"DEBUG extract_parameters: item N is ParameterFile"` |
| `python/api/launch_ros.rs` | 2169 | `"DEBUG: is_yaml_file returned true"` |
| `python/api/launch_ros.rs` | 2192 | `"DEBUG: is_yaml_file returned false"` |
| `python/api/launch_ros.rs` | 2201 | `"DEBUG: extract_parameters got string: ..."` |
| `python/api/launch_ros.rs` | 2204 | `"DEBUG: is_yaml_file returned true"` |
| `python/api/launch_ros.rs` | 2222 | `"DEBUG: is_yaml_file returned false"` |

**Action**: Remove all 11. If any provide useful trace information, downgrade to `log::trace!`.

### 18.1.2: Downgrade `log::info!` to `log::debug!`

Parser internals should not appear at `info` level. These are implementation details:

| File | Line | Message | Reason |
|------|------|---------|--------|
| `lib.rs` | 58 | `"Executing Python launch file: ..."` | Internal dispatch |
| `lib.rs` | 64 | `"Processing YAML launch file: ..."` | Internal dispatch |
| `python/executor.rs` | 21 | `"Executing Python launch file: ..."` | Duplicate of lib.rs |
| `traverser/include.rs` | 48 | `"Including launch file: ..."` | Internal traversal |
| `traverser/include.rs` | 61 | `"Including Python launch file: ..."` | Internal traversal |
| `traverser/include.rs` | 79 | `"Including YAML launch file: ..."` | Internal traversal |
| `traverser/xml_include.rs` | 13 | `"Including XML launch file: ..."` | Internal traversal |
| `traverser/entity.rs` | 283 | `"Skipping standalone composable_node..."` | Internal decision |
| `traverser/entity.rs` | 290 | `"Loaded N composable node(s)..."` | Internal count |
| `actions/node.rs` | 80 | `"Found composable_node in container..."` | Internal decision |

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

## Phase 18.2: RAII Guards for Paired Operations

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

## Phase 18.3: Consolidate Duplicate Logic

**Goal**: Extract shared functions for command building and parameter merging to eliminate inconsistencies.

### 18.3.1: Unify Command Array Building

Three locations build nearly identical `cmd` arrays with subtle inconsistencies:

| Location | File | Lines | Normalization | Param file order |
|----------|------|-------|---------------|-----------------|
| XML nodes | `record/generator.rs` | 258-325 | Yes | Before params |
| Python nodes | `python/bridge.rs` | 75-152 | Yes | After params |
| XML containers | `actions/container.rs` | 118-180 | **No** | N/A |

**Inconsistencies found**:
- `container.rs` line 144: global params NOT boolean-normalized (missing `normalize_param_value()`)
- `bridge.rs` line 140: param files placed after params (generator.rs places them before)
- `record_conv.rs` line 66: backfill cmd modification skips normalization

**Solution**: Extract a shared `build_ros_command()` function:
```rust
pub fn build_ros_command(
    exec_path: String,
    name: Option<&str>,
    namespace: Option<&str>,
    global_params: &[(String, String)],
    params: &[(String, String)],
    params_files: &[String],
    remappings: &[(String, String)],
    arguments: &[String],
) -> Vec<String>
```

### 18.3.2: Unify Global Parameter Merging for LoadNodes

Two identical merge implementations:

| Location | File | Lines |
|----------|------|-------|
| Python captures | `python/bridge.rs` (LoadNodeCapture::to_record) | 242-269 |
| XML containers | `actions/container.rs` (to_load_node_record) | 429-440 |

Both do the same thing: start with global params, then merge node-specific params with override semantics. Extract to a shared helper.

### 18.3.3: Capture vs Record Type Overlap

**Not recommended for this phase.** NodeCapture/NodeRecord, ContainerCapture/ContainerRecord, LoadNodeCapture/LoadNodeRecord have overlapping fields but serve different lifecycle stages (construction vs serialization). Unifying them would be a large refactor with limited benefit. Document the relationship instead.

**Estimated effort**: Medium-Large (18.3.1 touches 3 files with careful testing)

---

## Phase 18.4: Print Macro Cleanup

**Goal**: Replace `println!`/`eprintln!` with structured logging where appropriate.

### Production Code Inventory

| File | Lines | Macro | Context | Action |
|------|-------|-------|---------|--------|
| `commands/io_helper.rs` | 31-32, 54-55, 79-91 | `println!` | User-facing CLI output for setcap/verify | **Keep** (appropriate for CLI feedback) |
| `play_launch_io_helper/main.rs` | 61 | `eprintln!` | Warning in binary entry point | **Keep** (binary, no log setup) |
| `play_launch_parser/main.rs` | 110, 126 | `eprintln!` | Error reporting in CLI | **Keep** (appropriate for CLI errors) |

**Result**: All print macros in production code are in CLI entry points where structured logging is not initialized or not appropriate. **No changes needed.**

**Estimated effort**: None

---

## Implementation Order

1. **Phase 18.1** (logging cleanup) — Quick wins, zero risk, immediate noise reduction
2. **Phase 18.2** (RAII guards) — Safety improvements, moderate effort
3. **Phase 18.3** (dedup) — Consistency fixes, requires careful testing
4. **Phase 18.4** (print macros) — No action needed (already appropriate)

## Verification

- All existing tests must pass (311 unit + integration)
- Autoware comparison: 45/45 functionally equivalent (no behavioral changes)
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
