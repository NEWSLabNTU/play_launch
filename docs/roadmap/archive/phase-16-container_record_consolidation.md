# Container Record Consolidation Plan

**Status**: 📋 Planned
**Priority**: High (Fixes node loading warnings)
**Estimated Effort**: 1-2 days

## Problem Statement

Currently, containers are represented in TWO places in `record.json`:
1. `container[]` array - Minimal metadata (name + namespace only)
2. `node[]` array - Full process spawn information

This dual representation causes:
- ❌ Data duplication and inconsistency
- ❌ Complex classification logic (scan node[] to find containers)
- ❌ "Container not found" warnings when containers missing from node[]
- ❌ Confusion about source of truth

## Goal

**Containers should ONLY exist in `container[]` array with ALL information needed to spawn them.**

## Current State Analysis

### Record Structures

#### Python Parser (`python/play_launch/dump/launch_dump.py`)

✅ **Already correct structure**:
```python
@dataclass
class ComposableNodeContainerRecord:
    # All fields from NodeRecord
    executable: str
    package: str
    name: str
    namespace: str
    exec_name: str | None
    params: list[tuple[str, str]]
    params_files: list[str]
    remaps: list[tuple[str, str]]
    ros_args: list[str] | None
    args: list[str] | None
    cmd: list[str]
    env: list[tuple[str, str]] | None = None
    respawn: bool | None = None
    respawn_delay: float | None = None
    global_params: list[tuple[str, str]] | None = None
```

#### Rust Parser (`src/play_launch_parser/.../record/types.rs`)

✅ **Already correct structure** (lines 68-84):
```rust
pub struct ComposableNodeContainerRecord {
    pub args: Option<Vec<String>>,
    pub cmd: Vec<String>,
    pub env: Option<Vec<(String, String)>>,
    pub exec_name: Option<String>,
    pub executable: String,
    pub global_params: Option<Vec<(String, String)>>,
    pub name: String,
    pub namespace: String,
    pub package: String,
    pub params: Vec<(String, String)>,
    pub params_files: Vec<String>,
    pub remaps: Vec<(String, String)>,
    pub respawn: Option<bool>,
    pub respawn_delay: Option<f64>,
    pub ros_args: Option<Vec<String>>,
}
```

#### Rust Play_launch (`src/play_launch/src/ros/launch_dump.rs`)

❌ **WRONG - Minimal structure** (lines 36-39):
```rust
pub struct NodeContainerRecord {
    pub namespace: String,
    pub name: String,
}
```

### Current Processing Flow (Broken)

1. **replay.rs:409-421** - Build `container_names` HashSet from minimal `launch_dump.container`
2. **replay.rs:432** - Call `prepare_node_contexts(&launch_dump, &node_log_dir, &container_names)`
3. **context.rs:370-404** - Classify nodes from `launch_dump.node[]`:
   - Check if `(namespace, name)` is in `container_names`
   - If yes → container context
   - If no → regular node context
4. **replay.rs:484-511** - Add containers to builder using `context.node_context` (from node[] classification)

**Problem**: If containers aren't in node[] array, classification fails → "Container not found" warnings

## Target State

### New Processing Flow

1. **replay.rs** - Call new `prepare_container_contexts(&launch_dump, &container_log_dir)`
2. **context.rs** - New function reads directly from `launch_dump.container[]`
3. **prepare_node_contexts()** - No longer needs `container_names` parameter, no classification
4. **replay.rs** - Add containers directly from container contexts

## Implementation Plan

### Phase 1: Update Rust Play_launch Record Structure

**File**: `src/play_launch/src/ros/launch_dump.rs`

**Change**: Replace minimal `NodeContainerRecord` with full structure matching parsers.

```rust
// BEFORE (lines 36-39)
pub struct NodeContainerRecord {
    pub namespace: String,
    pub name: String,
}

// AFTER
pub struct NodeContainerRecord {
    pub executable: String,
    pub package: String,
    pub name: String,
    pub namespace: String,
    pub exec_name: Option<String>,
    #[serde(default)]
    pub params: Vec<(String, ParameterValue)>,
    pub params_files: Vec<String>,
    #[serde(default)]
    pub remaps: Vec<(String, String)>,
    pub ros_args: Option<Vec<String>>,
    pub args: Option<Vec<String>>,
    pub cmd: Vec<String>,
    pub env: Option<Vec<(String, String)>>,
    #[serde(default)]
    pub respawn: Option<bool>,
    #[serde(default)]
    pub respawn_delay: Option<f64>,
    #[serde(default)]
    pub global_params: Option<Vec<(String, ParameterValue)>>,
}
```

**Impact**: This matches the structure already used by both parsers.

### Phase 2: Add Container Context Preparation

**File**: `src/play_launch/src/execution/context.rs`

**Action**: Add new function to prepare container contexts directly from `launch_dump.container[]`.

```rust
/// Prepare container execution contexts from container records
pub fn prepare_container_contexts(
    launch_dump: &LaunchDump,
    container_log_dir: &Path,
) -> eyre::Result<Vec<NodeContainerContext>> {
    // Track base names for deduplication (e.g., "container" -> 1, 2, 3...)
    let mut name_counts = HashMap::<String, usize>::new();

    let container_contexts: Result<Vec<_>, _> = launch_dump
        .container
        .par_iter()
        .enumerate()
        .map(|(idx, container_record)| {
            // Build NodeRecord from NodeContainerRecord
            let node_record = NodeRecord {
                executable: container_record.executable.clone(),
                package: Some(container_record.package.clone()),
                name: Some(container_record.name.clone()),
                namespace: Some(container_record.namespace.clone()),
                exec_name: container_record.exec_name.clone(),
                params: container_record.params.clone(),
                params_files: container_record.params_files.clone(),
                remaps: container_record.remaps.clone(),
                ros_args: container_record.ros_args.clone(),
                args: container_record.args.clone(),
                cmd: container_record.cmd.clone(),
                env: container_record.env.clone(),
                respawn: container_record.respawn,
                respawn_delay: container_record.respawn_delay,
                global_params: container_record.global_params.clone(),
            };

            // Build full container name for matching with composable nodes
            let full_container_name = if container_record.namespace.ends_with('/') {
                format!("{}{}", container_record.namespace, container_record.name)
            } else {
                format!("{}/{}", container_record.namespace, container_record.name)
            };

            // Create NodeContext
            let node_context = NodeContext {
                record: node_record,
                cmdline: /* ... build from record ... */,
                output_dir: container_log_dir.join(format!("container_{}", idx)),
            };

            Ok(NodeContainerContext {
                node_container_name: full_container_name,
                node_context,
            })
        })
        .collect();

    container_contexts
}
```

### Phase 3: Update Node Context Preparation

**File**: `src/play_launch/src/execution/context.rs`

**Action**: Remove container classification from `prepare_node_contexts()`.

```rust
// BEFORE
pub fn prepare_node_contexts(
    launch_dump: &LaunchDump,
    node_log_dir: &Path,
    container_names: &HashSet<String>,  // ← Remove this parameter
) -> eyre::Result<NodeContextClasses>

// AFTER
pub fn prepare_node_contexts(
    launch_dump: &LaunchDump,
    node_log_dir: &Path,
) -> eyre::Result<Vec<NodeContext>>  // ← Return simple Vec, no classification
```

**Change**:
- Remove partition_map classification logic (lines 370-404)
- Return all node contexts as regular nodes
- No longer needs to check `container_names`

### Phase 4: Update Replay Logic

**File**: `src/play_launch/src/commands/replay.rs`

**Changes**:

1. **Remove** container names HashSet building (lines 409-425):
```rust
// DELETE THIS
let container_names: HashSet<String> = launch_dump
    .container
    .par_iter()
    .map(|record| { /* ... */ })
    .collect();
```

2. **Update** node context preparation (line 432):
```rust
// BEFORE
let NodeContextClasses {
    container_contexts,
    non_container_node_contexts: pure_node_contexts,
} = prepare_node_contexts(&launch_dump, &node_log_dir, &container_names)?;

// AFTER
let pure_node_contexts = prepare_node_contexts(&launch_dump, &node_log_dir)?;
let container_contexts = prepare_container_contexts(&launch_dump, &container_log_dir)?;
```

3. **Update** counts (line 443-452):
```rust
// Same logic, but contexts come from different sources
let num_pure_nodes = pure_node_contexts.len();
let num_containers = container_contexts.len();  // From container[] not node[]
let num_composable_nodes = load_node_contexts.len();
```

### Phase 5: Update Python Parser (Revert Recent Change)

**File**: `python/play_launch/dump/visitor/composable_node_container.py`

**Action**: Revert to NOT calling `visit_node()`. Keep containers ONLY in container[] array.

```python
# BEFORE (current broken state)
from .node import visit_node
container_actions = visit_node(container, context, dump)  # ← Adds to node[]

# AFTER (correct)
from .execute_process import visit_execute_process
container_actions = visit_execute_process(container, context, dump)  # ← No node[] addition
```

The container is still added to `dump.container` (line 179), which is correct.

## Testing Plan

### Parser Comparison Test (CRITICAL)

**Must pass**: `just compare-parsers`

This test validates that Rust and Python parsers produce **semantically equivalent** output:
- Runs both parsers on test launch files
- Normalizes output (handles ordering, type differences)
- Compares all record types (nodes, containers, composable nodes)
- **MUST PASS** before consolidation is complete

**Script**: `scripts/compare_parsers.sh` using `scripts/compare_records.py`

**Current test cases**:
- `test/simple_test/launch/pure_nodes.launch.xml` - Simple nodes
- `test/simple_test/launch/composition.launch.xml` - Composable nodes

### Unit Tests

1. **Test container context preparation**:
   - Create mock `LaunchDump` with containers
   - Call `prepare_container_contexts()`
   - Verify contexts have correct data

2. **Test node context preparation**:
   - Create mock `LaunchDump` with regular nodes
   - Call `prepare_node_contexts()`
   - Verify no containers are returned

### Integration Tests

1. **Simple container test** (`test/simple_test/`):
   - Generate record.json with both parsers
   - **Run comparison**: `just compare-dumps` in test directory
   - Verify containers only in container[] array
   - Run `play_launch replay`
   - Verify no warnings

2. **Autoware test** (`test/autoware_planning_simulation/`):
   - Generate record.json with both parsers
   - **Run comparison**: `just compare-dumps` in test directory
   - Verify 15 containers in container[] array
   - Verify 0 containers in node[] array
   - Run `just run-sim`
   - Verify all 54 composable nodes load successfully
   - Verify no "Container not found" warnings

### Validation Checklist

- [ ] **Parser comparison test passes** (`just compare-parsers`) ⭐ CRITICAL
- [ ] Both parsers produce identical record.json structure
- [ ] Containers only in container[] array (verified in record.json)
- [ ] Container[] array has all fields (executable, package, params, etc.)
- [ ] Node[] array has no containers (verified in record.json)
- [ ] Simple test runs without warnings
- [ ] Autoware test runs without warnings
- [ ] All composable nodes load successfully
- [ ] Manual Autoware comparison passes (`test/autoware_planning_simulation/just compare-dumps`)

## Benefits

✅ **Single Source of Truth**: Containers only in container[] array
✅ **No Duplication**: Container data stored once
✅ **Simpler Logic**: No classification needed, direct read from container[]
✅ **Consistent**: Python and Rust parsers produce identical structure
✅ **Fixes Warnings**: No more "Container not found" errors
✅ **Maintainable**: Clear separation of node types

## Migration Notes

### Backward Compatibility

⚠️ **Breaking Change**: Old record.json files with containers in node[] array will not work.

**Migration Path**:
1. Regenerate record.json with updated parser
2. Old files can be detected by checking container[] structure
3. Could add migration tool if needed

### Record Version

Consider adding `record_version` field to detect format:
```json
{
  "record_version": "2.0",
  "node": [],
  "container": [],
  "load_node": []
}
```

## Implementation Order

**CRITICAL**: Both parsers must be updated to keep outputs identical. Run comparison test after each phase!

1. ✅ **Phase 1**: Update NodeContainerRecord structure (30 min)
   - ⚠️ This changes deserialization format
   - Python parser already generates correct structure
   - Run `just compare-parsers` to verify (should still pass)

2. ✅ **Phase 2**: Add prepare_container_contexts() (1 hour)
   - ⚠️ Don't remove classification logic yet
   - Test new function in isolation
   - Existing tests should still pass

3. ✅ **Phase 3**: Update prepare_node_contexts() (30 min)
   - Remove classification logic
   - Return simple Vec<NodeContext>
   - Some tests may break (expected)

4. ✅ **Phase 4**: Update replay.rs (1 hour)
   - Call both prepare functions
   - Remove container_names HashSet
   - Integration tests should pass now

5. ✅ **Phase 5**: Revert Python parser change (15 min)
   - Remove visit_node() call
   - Keep visit_execute_process() (or remove entirely)
   - **MUST RUN**: `just compare-parsers` (should pass) ⭐

6. 🧪 **Testing**: Run all tests (2 hours)
   - `just compare-parsers` (MUST PASS) ⭐
   - `just quality` (all unit tests)
   - `just run-sim` in test/autoware_planning_simulation
   - Manual verification of output

7. 📝 **Documentation**: Update CLAUDE.md (30 min)
   - Document change in key recent changes
   - Update record.json format notes

**Total Estimated Time**: 6 hours (1 day)

### Critical Success Factors

1. ⭐ **Comparison test must pass**: `just compare-parsers` validates both parsers produce identical output
2. ⭐ **No regressions**: All existing tests must pass
3. ⭐ **Autoware validation**: 54 composable nodes load without warnings

## Files to Modify

### Rust Play_launch
- `src/play_launch/src/ros/launch_dump.rs` - Update NodeContainerRecord
- `src/play_launch/src/execution/context.rs` - Add prepare_container_contexts(), update prepare_node_contexts()
- `src/play_launch/src/commands/replay.rs` - Update container processing logic

### Python Parser
- `python/play_launch/dump/visitor/composable_node_container.py` - Revert to NOT call visit_node()

### Documentation
- `CLAUDE.md` - Update key recent changes
- `docs/roadmap/README.md` - Add this phase

### Tests
- Add unit tests for new container context preparation
- Update integration tests to verify new structure

## Related Issues

- Original issue: "Container not found" warnings in Autoware test
- Root cause: Containers missing from node[] array
- Better solution: Don't require containers in node[] at all

## References

- Python dump_launch: `python/play_launch/dump/visitor/composable_node_container.py`
- Rust parser: `src/play_launch_parser/src/play_launch_parser/src/python/bridge.rs:183-276`
- Current processing: `src/play_launch/src/commands/replay.rs:409-511`
