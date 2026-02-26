# Context Unification Design Document

**Phase 17.1 Deliverable** | **Date**: 2026-02-06 | **Status**: Design Complete

## 1. Current Architecture

The parser uses **three** separate state mechanisms during parsing:

### 1.1 LaunchContext (`src/substitution/context.rs`)

Used by XML parsing and substitution resolution. Hybrid Arc + Local scope-chain pattern.

| Field                      | Type                                 | Purpose                                                          |
|----------------------------|--------------------------------------|------------------------------------------------------------------|
| `parent`                   | `Option<Arc<ParentScope>>`           | Immutable parent scope chain                                     |
| `local_configurations`     | `HashMap<String, Vec<Substitution>>` | Launch args stored as parsed substitutions (lazy resolution)     |
| `local_environment`        | `HashMap<String, String>`            | Context-local environment variables                              |
| `local_declared_arguments` | `HashMap<String, ArgumentMetadata>`  | Declared argument metadata (name, default, description, choices) |
| `local_global_parameters`  | `IndexMap<String, String>`           | ROS global parameters (SetParameter actions)                     |
| `local_remappings`         | `Vec<(String, String)>`              | Topic remappings (from, to)                                      |
| `current_file`             | `Option<PathBuf>`                    | Current file for `$(dirname)`/`$(filename)`                      |
| `namespace_stack`          | `Vec<String>`                        | **Fully-qualified** namespace stack                              |

**Namespace semantics**: Each stack element is a fully-qualified absolute path.
- Stack: `["/", "/perception", "/perception/traffic_light"]`
- `current_namespace()` returns `last()` → `/perception/traffic_light`
- `push_namespace("camera")` appends `"/perception/traffic_light/camera"`

**Key methods** (24 total): `child()` (O(1) scope fork), `set_configuration()` (stores as `Vec<Substitution>`), `get_configuration()` (resolves lazily, walks parent chain), `get_configuration_lenient()` (fallback string on failure), scope save/restore via `namespace_depth()`/`restore_namespace_depth()` and `remapping_count()`/`restore_remapping_count()`.

**Used in**: `lib.rs`, `arg.rs`, `condition.rs`, `node.rs`, `container.rs`, `executable.rs`, `load_composable_node.rs`, `params.rs`, `generator.rs`, `types.rs` (substitution resolution), `executor.rs`, `api/utils.rs`.

### 1.2 ParseContext (`src/context.rs`)

Used by Python API via thread-local storage. Flat structure (no parent chain).

| Field                   | Type                              | Purpose                             |
|-------------------------|-----------------------------------|-------------------------------------|
| `launch_configurations` | `HashMap<String, String>`         | Launch args stored as plain strings |
| `ros_namespace_stack`   | `Vec<String>`                     | **Segment-based** namespace stack   |
| `captured_nodes`        | `Vec<NodeCapture>`                | Captured regular nodes              |
| `captured_containers`   | `Vec<ContainerCapture>`           | Captured containers                 |
| `captured_load_nodes`   | `Vec<LoadNodeCapture>`            | Captured composable node loads      |
| `captured_includes`     | `Vec<IncludeCapture>`             | Captured include operations         |
| `environment`           | `Option<HashMap<String, String>>` | Optional env var overrides          |

**Namespace semantics**: Each stack element is a relative segment.
- Stack: `["", "/perception", "/traffic_light"]`
- `current_namespace()` returns `join("")` → `/perception/traffic_light`
- `push_namespace("/camera")` appends `"/camera"` segment

**Key methods** (22 total): `capture_node/container/load_node/include()`, `captured_*()` accessors, `to_record_json()`, `launch_configurations_as_pydict()`.

**Used in**: `lib.rs` (stored in `LaunchTraverser`), `bridge.rs` (thread-local access for Python API).

### 1.3 Global Statics (`src/python/bridge.rs`)

Thread-safe global storage for Python ↔ Rust communication.

| Static                  | Type                                              | Purpose                                             |
|-------------------------|---------------------------------------------------|-----------------------------------------------------|
| `LAUNCH_CONFIGURATIONS` | `Lazy<Arc<Mutex<HashMap<String, String>>>>`       | Python-visible launch args                          |
| `GLOBAL_PARAMETERS`     | `Lazy<Arc<Mutex<IndexMap<String, String>>>>`      | SetParameter values from Python                     |
| `CURRENT_PARSE_CONTEXT` | `thread_local RefCell<Option<*mut ParseContext>>` | Raw pointer to ParseContext during Python execution |

## 2. Field Mapping Table

| Concept                   | LaunchContext                                                                 | ParseContext                                         | Global Statics                                                     |
|---------------------------|-------------------------------------------------------------------------------|------------------------------------------------------|--------------------------------------------------------------------|
| **Launch configurations** | `local_configurations` (as `Vec<Substitution>`, lazy resolution, scope chain) | `launch_configurations` (as `String`, no resolution) | `LAUNCH_CONFIGURATIONS` (sync'd before Python)                     |
| **Namespace**             | `namespace_stack` (fully-qualified elements)                                  | `ros_namespace_stack` (segment elements)             | —                                                                  |
| **Environment variables** | `local_environment` (scope chain)                                             | `environment` (flat, optional)                       | —                                                                  |
| **Global parameters**     | `local_global_parameters` (scope chain)                                       | —                                                    | `GLOBAL_PARAMETERS` (Python writes, copied to LaunchContext after) |
| **Declared arguments**    | `local_declared_arguments`                                                    | —                                                    | —                                                                  |
| **Remappings**            | `local_remappings`                                                            | —                                                    | —                                                                  |
| **Current file**          | `current_file`                                                                | —                                                    | —                                                                  |
| **Node captures**         | —                                                                             | `captured_nodes`                                     | —                                                                  |
| **Container captures**    | —                                                                             | `captured_containers`                                | —                                                                  |
| **LoadNode captures**     | —                                                                             | `captured_load_nodes`                                | —                                                                  |
| **Include captures**      | —                                                                             | `captured_includes`                                  | —                                                                  |
| **Scope chain**           | `parent: Option<Arc<ParentScope>>`                                            | — (flat)                                             | —                                                                  |

### 2.1 Overlapping Fields

1. **Launch configurations** — stored in all three. LaunchContext has substitution-aware resolution; ParseContext stores plain strings; `LAUNCH_CONFIGURATIONS` is a sync copy.
2. **Namespace** — stored in both LaunchContext and ParseContext with incompatible semantics. Manual sync in `execute_python_file()`.
3. **Environment variables** — stored in both LaunchContext and ParseContext, but ParseContext's env is barely used.

### 2.2 Unique Fields

- **LaunchContext only**: declared_arguments, remappings, current_file, parent scope chain, substitution-aware config storage.
- **ParseContext only**: entity captures (nodes, containers, load_nodes, includes).
- **Global statics only**: `GLOBAL_PARAMETERS` (Python-side global params).

## 3. Current Synchronization Points

There are **6 synchronization points** in `lib.rs` that manually bridge the contexts:

1. **`execute_python_file()` lines 221-243**: Namespace sync LaunchContext → ParseContext (fully-qualified → segment conversion).
2. **`execute_python_file()` lines 255-268**: Namespace sync ParseContext → LaunchContext (segment → fully-qualified conversion).
3. **`execute_python_file()` lines 276-292**: Copy `GLOBAL_PARAMETERS` → LaunchContext.
4. **`process_xml_include_with_namespace()` lines 516-534**: Copy XML configurations → `LAUNCH_CONFIGURATIONS`.
5. **`push-ros-namespace` handler lines 1135-1136**: Push to both contexts.
6. **`pop-ros-namespace` handler lines 1140-1141**: Pop from both contexts.

Plus the thread-local set/clear bracket around Python execution (lines 247, 271).

## 4. Dual Record Storage in LaunchTraverser

`LaunchTraverser` stores parsed entities in **two** places:

| Source                   | Storage                                  | When Used                                                          |
|--------------------------|------------------------------------------|--------------------------------------------------------------------|
| XML nodes                | `self.records: Vec<NodeRecord>`          | `traverse_entity("node")`, `traverse_entity("executable")`         |
| XML containers           | `self.containers: Vec<ContainerRecord>`  | `traverse_entity("node_container")`                                |
| XML load_nodes           | `self.load_nodes: Vec<LoadNodeRecord>`   | `traverse_entity("node_container")` (from composable nodes within) |
| XML load_composable_node | `self.parse_context.captured_load_nodes` | `traverse_entity("load_composable_node")`                          |
| Python nodes             | `self.parse_context.captured_nodes`      | Python API via thread-local                                        |
| Python containers        | `self.parse_context.captured_containers` | Python API via thread-local                                        |
| Python load_nodes        | `self.parse_context.captured_load_nodes` | Python API via thread-local                                        |

In `into_record_json()`, both sources are merged: `parse_context.captured_*()` → convert → extend with `self.records/containers/load_nodes`.

## 5. Unified Context Design

### 5.1 Design Decision: Namespace Semantics

**Chosen: Fully-qualified** (LaunchContext's current approach).

Rationale:
- Simpler: `current_namespace()` is just `last()`, no join needed
- Matches ROS 2 behavior (namespaces are always absolute paths)
- Enables direct comparison without conversion
- Avoids segment-join edge cases (empty root, double slashes)

### 5.2 Design Decision: Configuration Storage

**Chosen: Substitution-aware** (LaunchContext's `Vec<Substitution>` approach).

Rationale:
- Required for lazy nested substitution resolution (`$(var a)` where `a = $(var b)`)
- Plain string storage (ParseContext) loses resolution capability
- Python API can convert to strings at the boundary

### 5.3 Unified Context Structure

```rust
/// Unified context for both XML and Python launch file parsing
///
/// Combines LaunchContext (substitution resolution, scope chain) with
/// ParseContext (entity captures) and eliminates global statics.
pub struct UnifiedContext {
    // === Scope chain (from LaunchContext) ===
    parent: Option<Arc<ParentScope>>,

    // === Local configuration state (from LaunchContext) ===
    local_configurations: HashMap<String, Vec<Substitution>>,
    local_environment: HashMap<String, String>,
    local_declared_arguments: HashMap<String, ArgumentMetadata>,
    local_global_parameters: IndexMap<String, String>,
    local_remappings: Vec<(String, String)>,

    // === File tracking (from LaunchContext) ===
    current_file: Option<PathBuf>,

    // === Namespace (fully-qualified, from LaunchContext) ===
    namespace_stack: Vec<String>,

    // === Entity captures (from ParseContext) ===
    captured_nodes: Vec<NodeCapture>,
    captured_containers: Vec<ContainerCapture>,
    captured_load_nodes: Vec<LoadNodeCapture>,
    captured_includes: Vec<IncludeCapture>,
}
```

The `ParentScope` struct gains capture fields:

```rust
struct ParentScope {
    // Existing fields
    configurations: HashMap<String, Vec<Substitution>>,
    environment: HashMap<String, String>,
    declared_arguments: HashMap<String, ArgumentMetadata>,
    global_parameters: IndexMap<String, String>,
    remappings: Vec<(String, String)>,
    parent: Option<Arc<ParentScope>>,

    // New: frozen captures from parent scope
    // (captures accumulate upward, not inherited by children)
}
```

**Note on captures and scoping**: Captures don't follow scope-chain inheritance. They accumulate at the top-level traverser. Child contexts (includes) capture locally, then merge back to parent after processing. This is the current behavior and should be preserved.

### 5.4 LaunchTraverser Simplification

```rust
pub struct LaunchTraverser {
    context: UnifiedContext,          // Single context replaces both
    include_chain: Vec<PathBuf>,
    records: Vec<NodeRecord>,         // XML-generated records (keep for now)
    containers: Vec<ComposableNodeContainerRecord>,
    load_nodes: Vec<LoadNodeRecord>,
}
```

### 5.5 Thread-Local Access for Python

The thread-local pattern stays but points to `UnifiedContext`:

```rust
thread_local! {
    static CURRENT_CONTEXT: RefCell<Option<*mut UnifiedContext>> = const { RefCell::new(None) };
}
```

Python API accesses namespace, captures, and configurations through a single context pointer.

### 5.6 Eliminating Global Statics

| Current Global          | Replacement                                                                        |
|-------------------------|------------------------------------------------------------------------------------|
| `LAUNCH_CONFIGURATIONS` | `UnifiedContext.configurations()` — Python reads directly via thread-local         |
| `GLOBAL_PARAMETERS`     | `UnifiedContext.local_global_parameters` — Python writes directly via thread-local |
| `CURRENT_PARSE_CONTEXT` | `CURRENT_CONTEXT` (same pattern, different type)                                   |

This eliminates 2 `Lazy<Arc<Mutex<...>>>` globals and their sync code.

## 6. Migration Strategy

### Phase 1: Extend LaunchContext with captures (Phase 17.2) ✅ COMPLETE

Rather than creating a separate `UnifiedContext` type, we extend `LaunchContext` directly
with capture fields. This avoids a rename/migration of every import and keeps the changes minimal.

**What was done**:
1. Created `src/captures.rs` — extracted `NodeCapture`, `ContainerCapture`, `LoadNodeCapture`, `IncludeCapture` struct definitions from `python/bridge.rs` into a neutral top-level module (eliminates `substitution → python` circular dependency)
2. Updated all imports (6 files: `context.rs`, `bridge.rs`, `node.rs`, `executable.rs`, `load_composable_node.rs`, `launch_ros.rs`, `actions.rs`) to use `crate::captures::*`
3. Added 4 capture fields to `LaunchContext`: `captured_nodes`, `captured_containers`, `captured_load_nodes`, `captured_includes`
4. Added 12 capture methods to `LaunchContext` (matching ParseContext's API): `capture_node()`, `captured_nodes()`, `captured_nodes_mut()`, etc.
5. Captures are **local only** — `child()` starts with empty captures (not inherited via `ParentScope`)
6. Wrote 7 unit tests: capture node/container/load_node/include, multiple entities, child isolation, mutable clear
7. All 317 tests pass (309 existing + 8 new), zero clippy warnings

**Key design decisions**:
- No new type: `LaunchContext` gains captures directly, avoiding import churn
- `ParentScope` unchanged: captures don't participate in scope chain inheritance
- `to_record()` impls stay in `bridge.rs`: they still depend on `GLOBAL_PARAMETERS` (cleaned up in Phase 4)
- `captures.rs` is a neutral module importable by both `substitution/` and `python/`

### Phase 2+3: Migrate XML + Python to LaunchContext (Phase 17.3 + 17.4) ✅ COMPLETE

Phases 17.3 and 17.4 were implemented together since they are tightly coupled.

**What was done**:
1. `bridge.rs`: Changed thread-local from `*mut ParseContext` to `*mut LaunchContext`
2. `bridge.rs`: Renamed `set_current_parse_context` → `set_current_launch_context`, etc.
3. `bridge.rs`: Renamed `with_parse_context` → `with_launch_context` (same API, different type)
4. `bridge.rs`: **Critical namespace fix** — removed "/" prefix normalization in `push_ros_namespace()`. LaunchContext handles relative/absolute semantics correctly; the old bridge normalization would have made relative namespaces absolute.
5. `bridge.rs`: Updated all capture and namespace functions to use `with_launch_context`
6. `lib.rs`: Removed `parse_context: ParseContext` field from `LaunchTraverser`
7. `lib.rs`: Updated `execute_python_file()` — removed all namespace sync code (both directions), set thread-local to `&mut self.context`
8. `lib.rs`: Updated `traverse_entity("push/pop-ros-namespace")` — only `self.context` (no dual context sync)
9. `lib.rs`: Updated `traverse_entity("load_composable_node")` — capture to `self.context`
10. `lib.rs`: Updated `process_include()` and `process_xml_include_with_namespace()` — merge captures from child `context` not `parse_context`
11. `lib.rs`: Updated `process_includes_parallel()` — no `parse_context` in child traverser
12. `lib.rs`: Updated `into_record_json()` — read captures from `self.context`
13. Kept `pub use context::ParseContext` for backward compatibility
14. All 316 tests pass, zero clippy warnings, entity counts match (46/15/54)

**Key elimination**: The namespace synchronization code is completely removed. Previously, `execute_python_file()` had 20+ lines converting between LaunchContext's fully-qualified stack and ParseContext's segment-based stack. With unified context, Python operates directly on LaunchContext — no conversion needed.

### Phase 4: Remove synchronization and ParseContext (Phase 17.5)

1. Remove namespace sync in `execute_python_file()` (6 sync points → 0)
2. Remove `LAUNCH_CONFIGURATIONS` global — Python reads from `LaunchContext` via thread-local
3. Remove `GLOBAL_PARAMETERS` global — Python writes to `LaunchContext` via thread-local
4. Remove `ParseContext` entirely (`src/context.rs`)
5. Update `to_record()` impls to take global params as parameter instead of reading from global
6. Run all tests + Autoware comparison

### Rollback Strategy

Each phase is independently testable. If a phase breaks tests:
1. **Phase 2 rollback**: Revert XML migration, LaunchContext captures are unused but harmless
2. **Phase 3 rollback**: Revert Python migration, keep XML on LaunchContext captures
3. **Phase 4 rollback**: Keep sync code temporarily while fixing edge cases

The key safety net is that 310+ parser tests + Autoware comparison will catch any regression.

## 7. Risk Assessment

### High Risk Areas

1. **Thread-local pointer safety**: Switching from `*mut ParseContext` to `*mut UnifiedContext` — same safety guarantees, just different type. Low risk.

2. **Capture accumulation across includes**: Currently, included file's `parse_context` captures are merged back to parent. With unified context, child `UnifiedContext` captures need the same merge-back behavior. **Must preserve this pattern**.

3. **Parallel include processing** (`process_includes_parallel`): Each thread creates its own `LaunchTraverser` with empty `ParseContext`. With unified context, each thread gets a `child()` of the unified context. Captures from parallel threads must merge back correctly.

### Medium Risk Areas

4. **YAML include scoping**: YAML includes modify parent context directly. Must work with unified context (already uses `self.context` so no change needed).

5. **Python `LAUNCH_CONFIGURATIONS` access pattern**: Multiple Python API classes read `LAUNCH_CONFIGURATIONS` directly. Must update all to read from unified context via thread-local.

### Low Risk Areas

6. **Namespace semantics change for Python**: Python API currently uses segment-based. Switching to fully-qualified means `push_namespace` and `current_namespace` in bridge.rs just delegate to `UnifiedContext` — which already uses fully-qualified.

## 8. Success Criteria

- [ ] All context fields documented with usage examples (this document)
- [ ] Unified context design handles both XML and Python needs (section 5)
- [ ] Migration plan with rollback strategy defined (section 6)
- [ ] All 310+ parser tests pass after each migration phase
- [ ] Autoware comparison: 0 regressions
- [ ] Zero manual synchronization code remains after Phase 4
- [ ] `LAUNCH_CONFIGURATIONS` and `GLOBAL_PARAMETERS` globals eliminated
- [ ] Code reduction: ~200-300 lines of sync/conversion code removed
