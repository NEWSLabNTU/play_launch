# Phase 17: Context Unification and Parser Parity Completion

**Status**: 🔄 In Progress (Investigation)
**Started**: 2026-02-06
**Priority**: High (Affects parser correctness)
**Complexity**: Medium-High (Architectural refactoring)

## Overview

Unify the two separate context structures (LaunchContext and ParseContext) that have incompatible semantics, causing synchronization issues between XML and Python launch file parsing. Complete remaining parser discrepancies to achieve 100% Rust-Python parity.

## Problem Statement

Currently, the Rust parser uses two separate context structures:

1. **LaunchContext** (`src/substitution/context.rs`)
   - Used by: XML parsing, substitution resolution
   - Namespace stack: Contains fully-qualified namespaces
   - Namespace retrieval: Returns last element
   - Example stack: `["/", "/perception", "/perception/traffic_light_recognition"]`
   - Current namespace: `/perception/traffic_light_recognition`

2. **ParseContext** (`src/context.rs`)
   - Used by: Python API via thread-local storage
   - Namespace stack: Contains relative segments
   - Namespace retrieval: Joins all elements
   - Example stack: `["", "/perception", "/traffic_light_recognition"]`
   - Current namespace: `/perception/traffic_light_recognition` (after join)

**Issues**:
- **Incompatible semantics** require manual synchronization
- **Error-prone** context switching between XML and Python
- **Duplication** of configuration/parameter/environment storage
- **Maintenance burden** keeping two systems in sync

## Discovery Timeline

**2026-02-06**:
1. Initial investigation revealed namespace propagation failure from XML to Python
2. Root cause identified: Two separate context structures with incompatible stack semantics
3. Temporary fix implemented: Manual synchronization in `lib.rs::execute_python_file()`
   - Convert fully-qualified namespace to segment-based before Python execution
   - Convert back to fully-qualified after Python execution
4. Namespace issue resolved, but underlying architectural problem remains

## Goals

### Primary Goal
✅ **Unified Context Structure**: Single context type used by both XML and Python parsers with consistent semantics

### Secondary Goals
1. ✅ **100% Parser Parity**: Rust parser output matches Python parser exactly
2. ✅ **Simplified Architecture**: Eliminate synchronization code
3. ✅ **Type Safety**: Compile-time guarantees for context operations
4. ✅ **Maintainability**: Single source of truth for context state

## Work Items

### Phase 17.1: Context Analysis and Design (1-2 days)

**Tasks**:
- [ ] Audit all LaunchContext usage in XML parsing
- [ ] Audit all ParseContext usage in Python API
- [ ] Document all fields in both contexts and their purposes
- [ ] Identify overlapping fields vs unique fields
- [ ] Design unified context structure
- [ ] Choose namespace stack semantic (fully-qualified vs segments)
- [ ] Plan migration strategy

**Deliverables**:
- Architecture document: `docs/context_unification_design.md`
- Field mapping table (LaunchContext ↔ ParseContext)
- Migration checklist

**Success Criteria**:
- [ ] All context fields documented with usage examples
- [ ] Unified context design approved (handles both XML and Python needs)
- [ ] Migration plan with rollback strategy defined

### Phase 17.2: Unified Context Implementation (2-3 days)

**Tasks**:
- [ ] Create new `UnifiedContext` struct (or rename one of the existing)
- [ ] Standardize namespace stack semantic
  - **Recommended**: Fully-qualified (simpler, matches ROS 2 behavior)
  - Each push concatenates with current namespace
  - Single `current_namespace()` returns last element
- [ ] Merge configuration storage (launch_configurations)
- [ ] Merge environment variables
- [ ] Merge global parameters
- [ ] Add methods for both XML and Python use cases
- [ ] Implement comprehensive tests for all operations

**Deliverables**:
- New context implementation in `src/context/unified.rs` or updated `src/context.rs`
- Unit tests covering all context operations
- Documentation for public API

**Success Criteria**:
- [ ] All LaunchContext operations work with UnifiedContext
- [ ] All ParseContext operations work with UnifiedContext
- [ ] Namespace push/pop behavior consistent
- [ ] 100% test coverage for context operations
- [ ] Zero unsafe code

### Phase 17.3: XML Parser Migration (1-2 days)

**Tasks**:
- [ ] Update XML parser to use UnifiedContext
- [ ] Remove LaunchContext references
- [ ] Update substitution resolution to use UnifiedContext
- [ ] Update include processing
- [ ] Fix any namespace-related issues
- [ ] Run XML parsing tests

**Deliverables**:
- Updated XML parser using UnifiedContext
- Updated substitution resolver

**Success Criteria**:
- [ ] All XML parsing tests pass (260+ tests)
- [ ] Simple test comparison passes (Rust vs Python)
- [ ] No LaunchContext references remain in XML code

### Phase 17.4: Python API Migration (1-2 days)

**Tasks**:
- [ ] Update Python API to use UnifiedContext
- [ ] Remove ParseContext references
- [ ] Update thread-local context storage
- [ ] Update bridge.rs to use UnifiedContext
- [ ] Fix Node/Container/ComposableNode capture
- [ ] Run Python parsing tests

**Deliverables**:
- Updated Python API using UnifiedContext
- Updated bridge.rs

**Success Criteria**:
- [ ] All Python parsing tests pass (15+ tests)
- [ ] Python launch files execute correctly
- [ ] Nodes captured with correct namespace
- [ ] No ParseContext references remain in Python code

### Phase 17.5: Context Synchronization Cleanup (0.5 days)

**Tasks**:
- [ ] Remove manual namespace synchronization from `execute_python_file()`
- [ ] Remove namespace conversion logic
- [ ] Simplify XML-to-Python transition
- [ ] Add debug logging for context state

**Deliverables**:
- Simplified `lib.rs::execute_python_file()`
- Removed synchronization code

**Success Criteria**:
- [ ] No manual context copying between XML and Python
- [ ] Single context flows through entire parsing pipeline
- [ ] Debug logging shows correct namespace throughout

### Phase 17.6: Remaining Parser Discrepancies (1-2 days)

**Tasks**:
- [ ] Fix exec_name field (should use executable, not node name)
  - Currently: `exec_name: self.name.clone()`
  - Should be: `exec_name: Some(self.executable.clone())`
  - Location: `src/python/bridge.rs:76`
- [ ] Fix namespace in command line generation
  - Issue: `__ns:=/` instead of `__ns:=/perception/traffic_light_recognition/camera6/classification`
  - Root cause: `generate_command()` uses `self.namespace` but it's not set correctly
  - Location: `src/python/bridge.rs::generate_command()`
- [ ] Add global parameters to Rust node output
  - Python nodes include vehicle parameters (wheel_radius, etc.)
  - Rust nodes missing these parameters
  - Location: `src/python/bridge.rs::generate_command()` line 150-156
- [ ] Investigate params_files discrepancy
  - Rust: References map_projector_info.yaml
  - Python: References actual node parameter file
  - May be related to YAML parameter loading logic
- [ ] Investigate executable path resolution
  - Rust: Uses `/opt/ros/humble/lib/...`
  - Python: Uses workspace-specific `/home/.../install/.../lib/...`
  - Issue: AMENT_PREFIX_PATH search order or caching
  - Location: `src/python/bridge.rs::find_package_executable()`

**Deliverables**:
- Fixed exec_name generation
- Fixed namespace in command line
- Global parameters included in all nodes
- Params_files using correct files
- Executable paths matching Python parser

**Success Criteria**:
- [ ] Autoware comparison passes with zero discrepancies
- [ ] All node fields match between Rust and Python parsers
- [ ] Command lines identical except for path normalization

### Phase 17.7: Validation and Testing (1 day)

**Tasks**:
- [ ] Run full test suite (310+ tests)
- [ ] Run Autoware comparison test
- [ ] Run simple_test comparison
- [ ] Run LCTK demo comparison
- [ ] Performance benchmarking (ensure no regression)
- [ ] Update documentation

**Deliverables**:
- Test results summary
- Performance comparison (before/after)
- Updated CLAUDE.md

**Success Criteria**:
- [ ] All 310+ parser tests pass
- [ ] Autoware comparison: 0 discrepancies
- [ ] Simple test comparison: 0 discrepancies
- [ ] No performance regression (< 5% slowdown acceptable)
- [ ] Documentation updated

## Success Criteria (Overall)

### Must Have
- [ ] Single context structure used by both XML and Python
- [ ] No manual synchronization code
- [ ] All 310+ tests pass
- [ ] Autoware comparison: 0 discrepancies

### Should Have
- [ ] Performance maintained or improved
- [ ] Code complexity reduced (fewer lines of context management code)
- [ ] Clear documentation of unified context API

### Nice to Have
- [ ] Migration guide for external users (if any)
- [ ] Debug tooling for context inspection

## Remaining Parser Discrepancies (Detailed)

Based on Autoware comparison test (2026-02-06):

### 1. exec_name Field (All Nodes)
**Issue**: Uses node name instead of executable name
**Example**:
```
Rust:   exec_name: "traffic_light_occlusion_predictor"
Python: exec_name: "traffic_light_occlusion_predictor_node"
```
**Impact**: High - Affects all 46 nodes
**Fix Complexity**: Low - Single line change in bridge.rs:76

### 2. Namespace in Command Line (All Nodes)
**Issue**: `__ns:=/` instead of actual namespace
**Example**:
```
Rust:   '-r', '__ns:=/'
Python: '-r', '__ns:=/perception/traffic_light_recognition/camera6/classification'
```
**Impact**: High - Affects runtime behavior
**Fix Complexity**: Medium - Need to pass namespace to generate_command()

### 3. Global Parameters Missing (All Nodes)
**Issue**: Vehicle dimension parameters not included in Rust output
**Example**:
```
Python has: '-p', 'wheel_radius:=0.383', '-p', 'wheel_base:=2.79', ...
Rust missing these parameters
```
**Impact**: Medium - May affect node behavior if nodes rely on these
**Fix Complexity**: Low - Already implemented, may not be passed correctly

### 4. params_files Content Mismatch (Specific Nodes)
**Issue**: Different YAML files referenced
**Example** (traffic_light_occlusion_predictor):
```
Rust:   map_projector_info.yaml + lanelet2_map.osm paths
Python: Actual node parameters (azimuth_occlusion_resolution_deg, etc.)
```
**Impact**: Medium - Wrong configuration loaded
**Fix Complexity**: Medium - YAML parameter loading logic issue

### 5. Executable Path Resolution (All Nodes)
**Issue**: Different installation paths
**Example**:
```
Rust:   /opt/ros/humble/lib/package/executable
Python: /home/.../workspace/install/package/lib/package/executable
```
**Impact**: Low - Both should work, but inconsistent
**Fix Complexity**: Low - Adjust search order in find_package_executable()

## Risks and Mitigation

### Risk 1: Breaking Existing Functionality
**Probability**: Medium
**Impact**: High
**Mitigation**:
- Comprehensive test coverage before changes
- Incremental migration (XML first, then Python)
- Keep old contexts temporarily for comparison
- Extensive validation on real launch files

### Risk 2: Performance Regression
**Probability**: Low
**Impact**: Medium
**Mitigation**:
- Benchmark before/after
- Profile critical paths
- Optimize unified context operations
- Use Arc/Cow for efficient cloning if needed

### Risk 3: Semantic Mismatches
**Probability**: Medium
**Impact**: High
**Mitigation**:
- Thorough analysis of both context semantics
- Document all edge cases
- Extensive testing with complex launch files
- Validate with Autoware (46 nodes, 15 containers, 54 composables)

## Dependencies

- None (self-contained refactoring)

## Timeline Estimate

**Total**: 8-13 days

- Phase 17.1 (Analysis): 1-2 days
- Phase 17.2 (Implementation): 2-3 days
- Phase 17.3 (XML Migration): 1-2 days
- Phase 17.4 (Python Migration): 1-2 days
- Phase 17.5 (Cleanup): 0.5 days
- Phase 17.6 (Discrepancies): 1-2 days
- Phase 17.7 (Validation): 1 day

**Buffer**: 2-3 days for unexpected issues

## Related Issues

- Parser comparison failing with namespace mismatches (Fixed 2026-02-06)
- Manual synchronization in execute_python_file (To be removed)
- Duplicate context maintenance burden (To be eliminated)

## References

- `src/context.rs` - ParseContext implementation
- `src/substitution/context.rs` - LaunchContext implementation
- `src/lib.rs` - Parser entry point with manual synchronization
- `src/python/bridge.rs` - Thread-local context management
- `tmp/run_all_checks.sh` - Comprehensive test script with parser comparison
- `docs/roadmap/README.md` - Overall roadmap tracking

## Notes

- **Current Workaround**: Manual namespace synchronization in `execute_python_file()` works but is fragile
- **Python Parser as Ground Truth**: When in doubt, match Python parser behavior exactly
- **Backward Compatibility**: This is an internal refactoring; no API changes expected
- **Code Quality**: Aim to reduce total lines of context management code by 30-40%
