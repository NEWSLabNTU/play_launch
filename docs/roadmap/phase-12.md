# Phase 12: Container-Managed Composable Nodes

**Status**: ✅ Complete (2026-01-13)
**Priority**: High (Architecture Simplification)
**Actual Effort**: 2 weeks
**Dependencies**: Phase 10 (Actor Pattern - Complete)

---

## Completion Summary

Phase 12 has been successfully completed on 2026-01-13. All composable nodes are now managed as virtual members within their parent container actors, eliminating the need for separate composable node actors and significantly simplifying the architecture.

**Key Achievements**:
- ✅ Merged composable node actors into container actors (virtual members)
- ✅ Eliminated separate log directories for composable nodes
- ✅ Implemented parameter passing with proper type preservation (double vs integer)
- ✅ Added enhanced debugging logs for container restart issues
- ✅ Successfully tested with Autoware (49 composable nodes, 15 containers)
- ✅ Web UI fully functional with virtual members
- ✅ Documentation and migration guide completed

---

## Overview

Refactor the architecture to merge composable node actors into container actors. Composable nodes will become "virtual members" managed entirely by their parent containers, eliminating actor coordination complexity and aligning with the actual ROS architecture where composable nodes cannot exist independently of containers.

**Design Document**: [container-managed-composable-nodes-design.md](../container-managed-composable-nodes-design.md)

---

## Motivation

### Current Issues
- Separate actors for composable nodes and containers require complex coordination
- Watch channels needed for state synchronization between actors
- Complex spawn logic to match composable nodes with containers
- Unnatural separation (composable nodes can't exist without containers in ROS)
- Separate log directories for composable nodes despite sharing container process

### Benefits
- **Fewer actors**: Reduced overhead, simpler spawn logic
- **Natural ownership**: Container directly manages what's loaded
- **Simpler state management**: Single source of truth
- **Better logging**: One process = one log directory
- **Accurate metrics**: Container metrics represent actual process

---

## Architecture Changes

### Before (Current)
```
Coordinator
├── RegularNodeActor (separate process)
├── ContainerActor (separate process)
│   └── LoadNode service client
└── ComposableNodeActor (virtual, watches container)
    └── Sends LoadNode requests via channel
```

### After (Target)
```
Coordinator
├── RegularNodeActor (separate process)
└── ContainerActor (separate process)
    ├── LoadNode service client
    └── Manages composable nodes internally
        ├── ComposableNodeEntry (state, metadata)
        ├── Loading timeout detection
        └── ListNodes verification
```

---

## Work Items

### Task 12.1: Extend Container Actor State ✅

**Objective**: Add composable node management to container actor

**Subtasks**:
- [x] 12.1.1: Add `composable_nodes: HashMap<String, ComposableNodeEntry>` to ContainerActor
- [x] 12.1.2: Define `ComposableNodeEntry` struct with state, metadata, loading tracking
- [x] 12.1.3: Define `ComposableNodeMetadata` struct (no output_dir field)
- [x] 12.1.4: Add `list_nodes_loading_timeout_secs` to ActorConfig
- [x] 12.1.5: Remove `list_nodes_loading_timeout_secs` from ComposableActorConfig
- [x] 12.1.6: Update container constructor to accept composable node definitions

**Files to Modify**:
- `src/play_launch/src/member_actor/container_actor.rs`
- `src/play_launch/src/member_actor/state.rs`
- `src/play_launch/src/cli/config.rs`

**Tests**:
- Unit test: Container with no composable nodes
- Unit test: Container initialization with composable nodes
- Unit test: ComposableNodeEntry state transitions

**Estimated Effort**: 3-4 days

---

### Task 12.2: Add Control Event Handlers ✅

**Objective**: Implement composable node control in container actor

**Subtasks**:
- [x] 12.2.1: Add `LoadComposable { name }` to ControlEvent enum
- [x] 12.2.2: Add `UnloadComposable { name }` to ControlEvent enum
- [x] 12.2.3: Add `LoadAllComposables` to ControlEvent enum
- [x] 12.2.4: Add `UnloadAllComposables` to ControlEvent enum
- [x] 12.2.5: Add `ComposableDiscoveredLoaded { name, unique_id }` to ControlEvent enum
- [x] 12.2.6: Implement `load_composable(&mut self, name: &str)` method
- [x] 12.2.7: Implement `unload_composable(&mut self, name: &str)` method
- [x] 12.2.8: Implement `load_all_composables(&mut self)` method
- [x] 12.2.9: Implement `unload_all_composables(&mut self)` method
- [x] 12.2.10: Add control event handlers to `handle_running()` select loop

**Files to Modify**:
- `src/play_launch/src/member_actor/events.rs`
- `src/play_launch/src/member_actor/container_actor.rs`

**Tests**:
- Integration test: Manual load/unload via control events
- Integration test: Load all / unload all
- Integration test: Load request when container not running

**Estimated Effort**: 2-3 days

---

### Task 12.3: Implement Loading Timeout Detection ✅

**Objective**: Add ListNodes verification support to container

**Subtasks**:
- [x] 12.3.1: Implement `check_loading_timeouts(&mut self)` method
- [x] 12.3.2: Add timeout check to container's select loop (1s interval)
- [x] 12.3.3: Emit `StateEvent::ListNodesRequested` when timeout occurs
- [x] 12.3.4: Track `list_nodes_requested` flag per composable node
- [x] 12.3.5: Implement `handle_composable_discovered(&mut self, name, unique_id)` method
- [x] 12.3.6: Add `ComposableDiscoveredLoaded` handler to select loop

**Files to Modify**:
- `src/play_launch/src/member_actor/container_actor.rs`

**Tests**:
- Integration test: Loading timeout triggers verification
- Integration test: Discovery notification transitions to Loaded
- Unit test: Timeout check with multiple nodes at different stages

**Estimated Effort**: 2 days

---

### Task 12.4: Implement Auto-Load on Startup ✅

**Objective**: Auto-load composable nodes when container starts

**Subtasks**:
- [x] 12.4.1: Implement `auto_load_composables(&mut self)` method
- [x] 12.4.2: Call auto-load after container process starts
- [x] 12.4.3: Filter nodes by `auto_load: true` flag
- [x] 12.4.4: Transition nodes from Blocked → Loading
- [x] 12.4.5: Queue LoadNode requests for auto-load nodes

**Files to Modify**:
- `src/play_launch/src/member_actor/container_actor.rs`

**Tests**:
- Integration test: Container starts, auto-loads composable nodes
- Integration test: Container restart re-loads auto_load nodes
- Unit test: Auto-load skips nodes with auto_load=false

**Estimated Effort**: 1-2 days

---

### Task 12.5: Implement Shutdown Transitions ✅

**Objective**: Transition composable nodes to Blocked when container stops

**Subtasks**:
- [x] 12.5.1: Implement `transition_all_composables_to_blocked(&mut self, reason)` method
- [x] 12.5.2: Call on container process exit
- [x] 12.5.3: Emit `StateEvent::Blocked` for each composable node
- [x] 12.5.4: Handle container stop control event
- [x] 12.5.5: Handle container crash/failure

**Files to Modify**:
- `src/play_launch/src/member_actor/container_actor.rs`

**Tests**:
- Integration test: Container stop → all nodes Blocked
- Integration test: Container crash → all nodes Blocked
- Unit test: State transitions with different BlockReasons

**Estimated Effort**: 1 day

---

### Task 12.6: Update Coordinator - Virtual Member Routing ✅

**Objective**: Add virtual member support to coordinator

**Subtasks**:
- [x] 12.6.1: Add `virtual_member_routing: HashMap<String, String>` to MemberHandle
- [x] 12.6.2: Update `send_control()` to detect and route virtual members
- [x] 12.6.3: Translate generic controls (Start→LoadComposable, Stop→UnloadComposable)
- [x] 12.6.4: Update `handle_node_discovered()` to send `ComposableDiscoveredLoaded` to container
- [x] 12.6.5: Populate routing map during spawn

**Files to Modify**:
- `src/play_launch/src/member_actor/coordinator.rs`
- `src/play_launch/src/commands/replay.rs` (event handling)

**Tests**:
- Unit test: Virtual member routing
- Integration test: Web UI control of composable node routes to container
- Unit test: Discovery matching and routing

**Estimated Effort**: 2-3 days

---

### Task 12.7: Update Spawn Logic ✅

**Objective**: Modify spawn to pass composable nodes to containers

**Subtasks**:
- [x] 12.7.1: Update `ContainerDefinition` to include composable node definitions
- [x] 12.7.2: Update `add_composable_node()` to store in container definition
- [x] 12.7.3: Pass composable nodes to `ContainerActor::new()` during spawn
- [x] 12.7.4: Remove `ComposableNodeActor` spawning logic
- [x] 12.7.5: Keep metadata and state cache for virtual members
- [x] 12.7.6: Populate `virtual_member_routing` map

**Files to Modify**:
- `src/play_launch/src/member_actor/coordinator.rs`
- `src/play_launch/src/commands/replay.rs`

**Tests**:
- Integration test: Spawn with composable nodes
- Unit test: Container receives correct composable definitions
- Integration test: Virtual member metadata query

**Estimated Effort**: 2-3 days

---

### Task 12.8: Remove Composable Node Actor ✅

**Objective**: Delete obsolete composable node actor code

**Subtasks**:
- [x] 12.8.1: Remove `src/play_launch/src/member_actor/composable_node_actor.rs`
- [x] 12.8.2: Remove `ComposableNodeActor` from mod.rs exports
- [x] 12.8.3: Remove `run_composable_node()` function
- [x] 12.8.4: Remove `ComposableActorConfig` struct
- [x] 12.8.5: Clean up unused imports across codebase
- [x] 12.8.6: Remove composable-specific logic from event handlers (where applicable)

**Files to Modify**:
- `src/play_launch/src/member_actor/mod.rs`
- `src/play_launch/src/member_actor/composable_node_actor.rs` (delete)
- Various files (cleanup imports)

**Tests**:
- Compile test: Ensure no references remain
- Integration test: Full system test without composable actor

**Estimated Effort**: 1 day

---

### Task 12.9: Update Log Directory Structure ✅

**Objective**: Remove separate log directories for composable nodes

**Subtasks**:
- [x] 12.9.1: Remove `output_dir` creation for composable nodes
- [x] 12.9.2: Update context preparation to skip composable node directories
- [x] 12.9.3: Update container metadata.json to include composable node info
- [x] 12.9.4: Remove `load_node/` directory creation logic
- [x] 12.9.5: Update log directory documentation

**Files to Modify**:
- `src/play_launch/src/execution/context.rs`
- `src/play_launch/src/util/log_dir.rs`
- `CLAUDE.md` (log structure documentation)

**Tests**:
- Integration test: Verify no load_node/ directories created
- Integration test: Container metadata includes composable nodes
- Manual test: Check log directory structure

**Estimated Effort**: 1 day

---

### Task 12.10: Update Configuration ✅

**Objective**: Move list_nodes timeout to container config

**Subtasks**:
- [x] 12.10.1: Remove `loading_timeout_secs` from `ListNodesSettings`
- [x] 12.10.2: Remove `unloading_timeout_secs` from `ListNodesSettings` (unused)
- [x] 12.10.3: Add `list_nodes_loading_timeout_secs` to `ActorConfig`
- [x] 12.10.4: Update config loading in replay.rs to set container config
- [x] 12.10.5: Update config YAML documentation
- [x] 12.10.6: Add default value (30 seconds) to ActorConfig

**Files to Modify**:
- `src/play_launch/src/cli/config.rs`
- `src/play_launch/src/member_actor/state.rs`
- `src/play_launch/src/commands/replay.rs`
- `CLAUDE.md`

**Tests**:
- Unit test: Config parsing with new structure
- Unit test: Default values applied correctly
- Integration test: Per-container timeout configuration

**Estimated Effort**: 1 day

---

### Task 12.11: Update Web UI Integration ✅

**Objective**: Ensure Web UI continues to work with virtual members

**Subtasks**:
- [x] 12.11.1: Verify Web UI can query composable node metadata
- [x] 12.11.2: Verify Web UI can query composable node state
- [x] 12.11.3: Verify control buttons work (Load/Unload)
- [x] 12.11.4: Verify Load All / Unload All buttons work
- [x] 12.11.5: Verify state events update UI correctly
- [x] 12.11.6: Test Web UI with mixed node types

**Files to Modify**:
- Potentially `src/play_launch/src/web/handlers.rs` (if routing needs adjustment)

**Tests**:
- Manual test: Web UI control of composable nodes
- Manual test: Web UI display of composable node status
- Integration test: SSE event streaming for composable nodes

**Estimated Effort**: 1-2 days

---

### Task 12.12: Update Tests ✅

**Objective**: Update existing tests and add new ones

**Subtasks**:
- [x] 12.12.1: Update unit tests for container actor
- [x] 12.12.2: Update integration tests for composable node loading
- [x] 12.12.3: Add tests for virtual member routing
- [x] 12.12.4: Add tests for auto-load behavior
- [x] 12.12.5: Add tests for timeout detection
- [x] 12.12.6: Remove composable node actor tests
- [x] 12.12.7: Update test fixtures and helpers

**Files to Modify**:
- `src/play_launch/tests/` (all test files)
- Test workspaces: `test/simple_test/`, `test/sequential_loading/`, etc.

**Tests**:
- Run all existing integration tests
- Add new test cases for edge cases
- Verify Autoware test still works

**Estimated Effort**: 2-3 days

---

### Task 12.13: Update Documentation ✅

**Objective**: Document the new architecture

**Subtasks**:
- [x] 12.13.1: Update CLAUDE.md with new architecture
- [x] 12.13.2: Update log directory structure documentation
- [x] 12.13.3: Update configuration documentation
- [x] 12.13.4: Add migration guide for users
- [x] 12.13.5: Update Key Recent Changes section
- [x] 12.13.6: Document breaking changes
- [x] 12.13.7: Update module structure documentation

**Files to Modify**:
- `CLAUDE.md`
- `README.md` (if needed)
- Design documents

**Deliverables**:
- Updated architecture diagrams (ASCII art)
- Migration guide for log parsing scripts
- Configuration migration guide

**Estimated Effort**: 1-2 days

---

## Testing Strategy

### Unit Tests
- Container with no composable nodes
- Container with multiple composable nodes
- Auto-load behavior
- Manual load/unload
- Loading timeout detection
- State transitions
- Virtual member routing

### Integration Tests
- Full replay with composable nodes
- Web UI control of composable nodes
- ListNodes verification flow
- Container restart with auto-load
- Multiple containers with many composable nodes
- Container crash recovery

### Test Workspaces
- Update `test/simple_test/` (2 composable nodes)
- Update `test/sequential_loading/` (5 nodes in 1 container)
- Update `test/concurrent_loading/` (4 nodes in 4 containers)
- Update `test/mixed_loading/` (5 nodes in 2 containers)
- Update `test/autoware_planning_simulation/` (52 composable nodes)

### Manual Testing
- Web UI functionality
- Log directory structure
- Metrics interpretation
- Configuration loading

---

## Breaking Changes

### Log Directory Structure
**Before**:
```
play_log/2025-01-10/
├── node/my_container/
└── load_node/my_composable/
    ├── metadata.json
    ├── metrics.csv
    └── ...
```

**After**:
```
play_log/2025-01-10/
└── node/my_container/
    ├── metadata.json (includes composable info)
    ├── metrics.csv (container process only)
    ├── out (all stdout)
    └── err (all stderr)
```

**Impact**: Scripts that parse `load_node/` directories will break

**Migration**:
- Grep container logs for composable node output
- Use container metadata to identify composable nodes
- Update parsing scripts to read from container logs

### Configuration Structure
**Before**:
```yaml
list_nodes:
  loading_timeout_secs: 30  # Global
```

**After**:
```yaml
# Now per-container in ActorConfig
# Set via code, not directly in YAML currently
```

**Impact**: Global timeout setting removed

**Migration**:
- Default 30s timeout applied to all containers
- Future: Add per-container config in YAML if needed

---

## Rollout Plan

### Phase A: Foundation (Tasks 12.1-12.5)
**Duration**: 1 week
- Extend container actor with composable management
- Implement control handlers
- Add loading timeout detection
- Implement auto-load and shutdown transitions

### Phase B: Coordinator Integration (Tasks 12.6-12.7)
**Duration**: 3-4 days
- Add virtual member routing to coordinator
- Update spawn logic to pass composable nodes to containers
- Remove composable node actor spawning

### Phase C: Cleanup and Migration (Tasks 12.8-12.10)
**Duration**: 3 days
- Remove composable node actor code
- Update log directory structure
- Update configuration

### Phase D: Testing and Documentation (Tasks 12.11-12.13)
**Duration**: 4-5 days
- Update Web UI integration
- Update all tests
- Update documentation
- Write migration guide

### Total Estimated Duration: 2.5-3 weeks

---

## Success Criteria

- [x] All existing tests pass
- [x] Autoware test (52 composable nodes, 15 containers) works correctly
- [x] Web UI controls composable nodes successfully
- [x] ListNodes verification still works
- [x] No `load_node/` directories created
- [x] Container logs include composable node output
- [x] Fewer actors spawned (no composable node actors)
- [x] Documentation updated and accurate
- [x] Migration guide written

---

## Risks and Mitigations

### Risk 1: Container Actor Complexity
**Impact**: Medium
**Probability**: High
**Mitigation**:
- Extract composable management into separate module
- Thorough testing of state transitions
- Clear code documentation

### Risk 2: Regression in Composable Node Loading
**Impact**: High
**Probability**: Medium
**Mitigation**:
- Comprehensive integration tests
- Test with Autoware (52 composable nodes)
- Gradual rollout with feature flag (if needed)

### Risk 3: Web UI Compatibility
**Impact**: Medium
**Probability**: Low
**Mitigation**:
- Minimal changes to Web UI (routing only)
- State cache structure unchanged
- Thorough manual testing

### Risk 4: Log Parsing Scripts Break
**Impact**: Medium
**Probability**: High
**Mitigation**:
- Clear migration guide
- Document breaking changes in release notes
- Example scripts for parsing new structure

---

## Dependencies

- Phase 10 (Actor Pattern) must be complete ✅
- ListNodes verification system must be working ✅
- No external library changes required

---

## Future Enhancements

After Phase 12 completion, consider:

1. **Per-Container YAML Config**: Allow list_nodes timeout in YAML per container pattern
2. **Composable Node Metrics**: Application-level metrics via ROS topic statistics
3. **Load Ordering**: Explicit ordering of composable node loading within container
4. **Parallel Loading**: Load multiple composable nodes concurrently (requires ROS support)

---

## References

- [Design Document](../container-managed-composable-nodes-design.md)
- [Phase 10: Actor Pattern](./phase-10.md) (prerequisite)
- [ListNodes Verification Design](../list-nodes-verification-design.md)
- [CLAUDE.md](../../CLAUDE.md)
