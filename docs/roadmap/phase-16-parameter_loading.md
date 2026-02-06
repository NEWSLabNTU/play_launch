# Phase 16: Parameter File Loading & Global Parameters

**Status**: 🔄 In Progress
**Started**: 2026-02-04
**Priority**: High (Blocking Autoware composable node loading)
**Dependencies**: Phase 15 (Python API Type Safety)

---

## Overview

Fix composable node loading failures in Autoware by ensuring all required parameters are provided. This involves loading YAML parameter files and capturing global parameters from the Python launch context.

---

## Problem Statement

### Issue 1: Missing Parameter Initialization ⚠️
Composable nodes fail to load with errors like:
```
Component constructor threw an exception: Statically typed parameter 'enable_whole_load' must be initialized.
```

### Root Causes Identified

1. **YAML files not loaded**: Rust parser stores `__param_file` paths instead of loading and expanding YAML contents
2. **Global parameters not captured**: Python launch framework injects global parameters (vehicle dimensions, sim_time, etc.) into every node, but Rust parser doesn't capture these

### Current Behavior

**Rust Parser** (Current):
```json
{
  "node_name": "pointcloud_map_loader",
  "params": [
    ["__param_file", "/path/to/params.yaml"],
    ["pcd_paths_or_directory", "[/path/to/map.pcd]"]
  ]
}
```
- 3 total parameters
- YAML file not loaded
- No global parameters

**Python Parser** (Expected):
```json
{
  "node_name": "pointcloud_map_loader",
  "params": [
    ["use_sim_time", "false"],
    ["wheel_radius", "0.383"],
    ["wheel_width", "0.235"],
    ...
    ["enable_whole_load", "true"],
    ["enable_downsampled_whole_load", "false"],
    ...
  ]
}
```
- 20+ total parameters
- YAML file loaded and expanded
- Global parameters included

---

## Work Completed ✅

### 1. Namespace Normalization (Complete)
**File**: `src/play_launch_parser/src/python/api/launch_ros.rs`

Fixed composable node namespaces to always have leading slashes:
```rust
let normalized_namespace = if node_namespace.is_empty() {
    "/".to_string()
} else if node_namespace.starts_with('/') {
    node_namespace
} else {
    format!("/{}", node_namespace)
};
```

**Impact**:
- ✅ Fixed all "Couldn't parse remap rule" errors
- ✅ 16 composable nodes now have correct namespaces
- ✅ Zero namespace-related LoadNode failures

### 2. YAML Parameter File Loading Infrastructure (Complete)
**File**: `src/play_launch_parser/src/python/api/launch_ros.rs` (lines 2165-2280)

Implemented helper functions:
- `load_yaml_params()` - Load and parse YAML files
- `flatten_yaml()` - Flatten nested YAML structure with dot notation
- `yaml_value_to_string()` - Convert YAML values to strings
- `is_yaml_file()` - Check if path is a YAML file

**ROS 2 Format Handling**:
Correctly strips ROS 2 parameter file wrappers:
```yaml
/**:                    # Wildcard node matcher (stripped)
  ros__parameters:      # ROS parameter namespace (stripped)
    enable_whole_load: true    # ← Extracted as parameter
    leaf_size: 3.0
```

**Status**: Infrastructure complete, but not yet connected to parameter capture

### 3. Modified Parameter Parsing Methods (Partial)
Updated four methods to use YAML loading:
- `ComposableNode::parse_parameters()` (line 269)
- `Node::parse_parameters()` (line 960)
- `LifecycleNode::parse_parameters()` (line 1399)
- `LoadComposableNodes::extract_parameters()` (line 1967)

**Status**: Code in place but not working because parameters don't come as file paths

---

## Work Remaining ⚠️

### Issue: Parameters Come as Dictionaries, Not File Paths

**Discovery** (2026-02-04):
Debug logging revealed that ALL parameters come through as `dict` objects, not as file path strings:
```rust
DEBUG: param_item type: dict
```

This means the Python launch framework loads YAML files BEFORE passing parameters to our Rust mock classes. By the time we see them, they're already expanded dictionaries.

### Root Cause: Global Parameters Not Captured

Comparison with Python parser reveals:

**For regular nodes**:
- Python adds `-p` flags to command line: `-p 'use_sim_time:=False' -p 'wheel_radius:=0.383'`
- Python extracts parameters from global context
- Rust doesn't capture these global parameters

**For composable nodes** (LoadNode service):
- Python includes global parameters in the params array
- Rust only has parameters explicitly passed to the node
- Result: Missing critical parameters like `enable_whole_load`

### Required Fix: Capture Global Parameters from Python Context

The Python launch framework stores global parameters in a context that gets injected into every node. We need to:

1. **Identify where Python stores global parameters**
   - Likely in `LaunchContext` or a global parameter store
   - These are parameters set by `SetLaunchConfiguration` or loaded from global parameter files

2. **Capture global parameters during Python execution**
   - Extract from Python launch context
   - Store in Rust parser's global state

3. **Merge global parameters with node parameters**
   - Before creating node captures
   - Apply to all nodes (regular, lifecycle, composable)

---

## Implementation Plan

### Phase 16.1: Investigate Global Parameter Storage ⏳

**Goal**: Understand where Python stores global parameters

**Tasks**:
1. Examine Python `LaunchContext` class
2. Look for global parameter dictionaries
3. Check `SetLaunchConfiguration` and parameter file loading
4. Trace how Python dump_launch visitor extracts these parameters

**Files to investigate**:
- `launch/launch_context.py` (Python ROS 2 launch)
- Our executor: `src/python/executor.rs`
- Our API: `src/python/api/actions.py`

### Phase 16.2: Capture Global Parameters 🔄 (Current)

**Goal**: Extract global parameters from Python launch context

**Approach**:
```rust
// In Python execution context
GLOBAL_PARAMETERS.with(|params| {
    let py_context = get_launch_context(py)?;
    let global_params = extract_global_parameters(py_context)?;
    *params.borrow_mut() = global_params;
});
```

**Integration points**:
- `src/python/executor.rs` - During launch file execution
- `src/python/bridge.rs` - Merge with node captures

### Phase 16.3: Merge Parameters with Node Captures ⏳

**Goal**: Inject global parameters into every node

**Approach**:
```rust
fn capture_node(mut node: NodeCapture) {
    // Merge global parameters
    let global_params = GLOBAL_PARAMETERS.with(|p| p.borrow().clone());
    node.parameters = merge_parameters(global_params, node.parameters);

    // Store capture
    CAPTURED_NODES.with(|nodes| {
        nodes.borrow_mut().push(node);
    });
}
```

**Apply to**:
- Regular nodes (`capture_node`)
- Composable nodes (`capture_load_node`)
- Lifecycle nodes

### Phase 16.4: Testing & Validation ⏳

**Test cases**:
1. Unit test: Global parameters injected into nodes
2. Comparison test: Rust vs Python parser parameter count matches
3. Autoware test: No "must be initialized" errors

**Success criteria**:
- ✅ Rust parser parameter count matches Python (20+ params per node)
- ✅ All global parameters captured (use_sim_time, vehicle dimensions, etc.)
- ✅ All composable nodes load successfully
- ✅ Zero "Statically typed parameter must be initialized" errors

---

## Technical Details

### Python Launch Parameter Flow

1. **Global parameters loaded**:
   - Vehicle info: `wheel_radius`, `wheel_base`, etc.
   - Simulation: `use_sim_time`
   - From files or `SetLaunchConfiguration`

2. **Parameters merged with nodes**:
   - Python launch framework injects global params
   - YAML parameter files loaded and expanded
   - Explicit node parameters override

3. **Parameters passed to node**:
   - As command line `-p` flags (regular nodes)
   - In LoadNode service request (composable nodes)

### Rust Parser Current Behavior

1. **No global parameter capture**:
   - Python execution happens in `src/python/executor.rs`
   - Parameters set via `SetLaunchConfiguration` not captured
   - Global parameter store not accessed

2. **Only explicit parameters captured**:
   - Parameters passed directly to node constructor
   - Parameters from `LoadComposableNodes` descriptions

3. **Result: Parameter mismatch**:
   - Rust: 2-3 parameters per node
   - Python: 20+ parameters per node

---

## Files Modified

### Completed Changes
1. `src/play_launch_parser/src/python/api/launch_ros.rs`
   - Lines 884-891: Namespace normalization
   - Lines 269-314: ComposableNode::parse_parameters (with YAML loading)
   - Lines 960-1000: Node::parse_parameters (with YAML loading)
   - Lines 1399-1430: LifecycleNode::parse_parameters (with YAML loading)
   - Lines 1967-2005: LoadComposableNodes::extract_parameters (with YAML loading)
   - Lines 2165-2280: YAML loading helper functions

### Pending Changes
1. `src/python/executor.rs` - Capture global parameters during execution
2. `src/python/bridge.rs` - Merge global parameters with captures
3. `src/python/api/actions.rs` - Extract parameters from LaunchContext

---

## Known Issues

### Issue 1: YAML Wrapper Handling
The `/**` key in YAML files is a ROS 2 wildcard pattern matching all nodes. Sometimes YAML files have specific node names instead:
```yaml
/map_container:
  ros__parameters:
    param: value
```

Current implementation handles `/**` but may need adjustment for specific node names.

**Status**: To be tested with various YAML formats

### Issue 2: Parameter Substitutions in YAML
YAML files may contain unresolved substitutions:
```yaml
pcd_paths_or_directory: [$(var pointcloud_map_path)]
```

Current implementation preserves these as strings. Runtime resolution in play_launch handles them.

**Status**: Acceptable for now, may need enhancement later

---

## Related Documentation

- [Phase 15: Python API Type Safety](./phase-15-python_api_type_safety.md)
- [Phase 14: Python Launch File Execution](./phase-14-python_execution.md)
- [Python Parser as Ground Truth](../../CLAUDE.md#python-parser-as-ground-truth)
- [Parameter File Substitution Resolution](../../CLAUDE.md#yaml-parameter-file-substitution-resolution)

---

## Progress Tracking

- [x] Problem analysis
- [x] Namespace normalization fix
- [x] YAML loading infrastructure
- [x] Modified parameter parsing methods
- [x] Identified root cause (global parameters)
- [ ] Investigate Python global parameter storage
- [ ] Capture global parameters from LaunchContext
- [ ] Merge global parameters with node captures
- [ ] Testing and validation
- [ ] Documentation update

**Current Step**: Phase 16.2 - Capture global parameters from Python context

---

## Timeline

- **2026-02-04**: Started, completed namespace fix and YAML infrastructure
- **2026-02-04**: Identified global parameter issue
- **Target**: Complete within 1-2 days
