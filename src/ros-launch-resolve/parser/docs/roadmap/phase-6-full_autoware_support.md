# Phase 6: Full Autoware Compatibility

**Status**: 📝 Planned (Session 11)
**Priority**: CRITICAL (for production Autoware use)
**Dependencies**: Phase 5 Complete ✅

---

## Overview

Achieve **95%+ Autoware coverage** by implementing missing features that currently block ~30% of entities.

**Current Coverage**: ~67%
- Nodes: 32/61 (52%)
- Containers: 12/15 (80%)
- Composable Nodes: 38/54 (70%)

**Target Coverage**: 95%+
- Nodes: 58+/61 (95%+)
- Containers: 14+/15 (93%+)
- Composable Nodes: 50+/54 (92%+)

**Estimated Time**: 1-2 weeks

---

## Gap Analysis

### Missing Entities by Category

**Composable Nodes (16 missing)**:
- 9-12: In control/planning containers (via `<load_composable_node>`)
- 2: In pointcloud_container (Python LoadComposableNodes)
- 2: In MRM containers (OpaqueFunction with file I/O)

**Containers (3 missing)**:
- 2: MRM operators (mrm_comfortable_stop_operator, mrm_emergency_stop_operator)
- 1: Additional system containers

**Regular Nodes (29 missing)**:
- 10+: topic_state_monitor_* nodes
- 1: simple_planning_simulator
- ~18: Other dynamic/conditional nodes

### Root Causes

1. **`<load_composable_node>` XML action not implemented** → ~12 composable nodes
2. **Python LoadComposableNodes target resolution** → 2 composable nodes
3. **Topic state monitor pattern** → 10+ regular nodes
4. **OpaqueFunction file I/O limitation** → 2-3 containers
5. **Dynamic node generation patterns** → Remaining nodes

---

## Work Items

### Phase 6.1: XML load_composable_node 🔴 CRITICAL

**Priority**: CRITICAL
**Estimated Time**: 2-3 days
**Expected Impact**: +9-12 composable nodes (67% → 80% coverage)

**Tasks**:
- [ ] Create `LoadComposableNodeAction` struct
  - [ ] Parse `target` attribute (container name/reference)
  - [ ] Parse nested `<composable_node>` elements
  - [ ] Support substitutions in all attributes
- [ ] Implement action handler in `lib.rs`
  - [ ] Resolve target container name
  - [ ] Create `LoadNodeRecord` for each composable node
  - [ ] Handle namespace inheritance
- [ ] Add tests
  - [ ] Unit tests for action parsing
  - [ ] Integration test with control container
  - [ ] Test namespace resolution

**Files**:
- `src/actions/load_composable_node.rs` (new)
- `src/actions/mod.rs` (export)
- `src/lib.rs` (handler)

**Example Usage**:
```xml
<load_composable_node target="/control/control_container">
  <composable_node pkg="..." plugin="..." name="...">
    <remap from="..." to="..."/>
    <param name="..." value="..."/>
  </composable_node>
</load_composable_node>
```

**Validation**:
- Run Autoware test: should capture lane_departure_checker, control_validator, collision_detector, AEB nodes

---

### Phase 6.2: Python LoadComposableNodes Enhancement 🟡 HIGH

**Priority**: HIGH
**Estimated Time**: 1 day
**Expected Impact**: +2 composable nodes (80% → 82% coverage)

**Tasks**:
- [ ] Improve `LoadComposableNodes::extract_target_container()`
  - [ ] Handle string container references properly
  - [ ] Resolve LaunchConfiguration in target
  - [ ] Add better error messages
- [ ] Add tests
  - [ ] Test with string container name
  - [ ] Test with LaunchConfiguration container
  - [ ] Verify namespace normalization

**Files**:
- `src/python/api/launch_ros.rs` (update)

**Validation**:
- Run Autoware test: should capture occupancy_grid_map and pointcloud_to_laserscan nodes in pointcloud_container

---

### Phase 6.3: Topic State Monitor Pattern 🟡 HIGH

**Priority**: HIGH
**Estimated Time**: 2-3 days
**Expected Impact**: +10-15 regular nodes (82% → 90% coverage)

**Tasks**:
- [ ] Investigate component_state_monitor.launch.py
  - [ ] Identify how topic_state_monitor_* nodes are created
  - [ ] Determine if it's loop-based or conditional generation
- [ ] Implement pattern support
  - [ ] Option A: Enhanced OpaqueFunction to handle loops
  - [ ] Option B: Special case for state monitor pattern
  - [ ] Option C: Better Python list comprehension support
- [ ] Add tests
  - [ ] Unit test for pattern
  - [ ] Integration test with component_state_monitor

**Files**:
- TBD based on investigation
- Likely: `src/python/executor.rs` or `src/python/api/actions.rs`

**Validation**:
- Run Autoware test: should capture all 10+ topic_state_monitor_* nodes

---

### Phase 6.4: OpaqueFunction File I/O (Optional) 🟢 MEDIUM

**Priority**: MEDIUM
**Estimated Time**: 1-2 days
**Expected Impact**: +2-3 containers (90% → 95% coverage)

**Tasks**:
- [ ] Add limited file I/O support to OpaqueFunction context
  - [ ] Mock `open()` function for YAML files
  - [ ] Implement `yaml.safe_load()` mock
  - [ ] Return empty/default config when file not found
- [ ] Update MockLaunchContext
  - [ ] Add file reading capability
  - [ ] Cache loaded YAML files
- [ ] Add tests
  - [ ] Test with mrm_comfortable_stop_operator.launch.py
  - [ ] Test with missing files (graceful degradation)

**Files**:
- `src/python/api/actions.rs` (OpaqueFunction.execute)
- `src/python/executor.rs` (context creation)

**Validation**:
- Run Autoware test: should capture mrm_comfortable_stop_operator and mrm_emergency_stop_operator containers

**Note**: This is optional because it's complex and only affects 2-3% of coverage. May defer to later phase.

---

### Phase 6.5: Additional Python Substitutions 🔵 LOW

**Priority**: LOW
**Estimated Time**: 1 day
**Expected Impact**: Minor (edge cases)

**Tasks**:
- [ ] Implement `Command` substitution
  - [ ] Execute shell command in mock environment
  - [ ] Return command output as string
- [ ] Implement boolean substitutions
  - [ ] `NotSubstitution` - Boolean NOT
  - [ ] `AndSubstitution` - Boolean AND
  - [ ] `OrSubstitution` - Boolean OR
- [ ] Add tests

**Files**:
- `src/python/api/substitutions.rs`

---

### Phase 6.6: Testing & Validation ✅ CRITICAL

**Priority**: CRITICAL
**Estimated Time**: 1 day (ongoing)
**Expected Impact**: Ensure quality

**Tasks**:
- [ ] Run Autoware test after each phase
  - [ ] Track coverage improvement
  - [ ] Document remaining gaps
- [ ] Add regression tests
  - [ ] XML load_composable_node
  - [ ] Python LoadComposableNodes variations
  - [ ] State monitor patterns
- [ ] Update documentation
  - [ ] feature_list.md with progress
  - [ ] This roadmap with completion status

**Success Criteria**:
- [ ] Autoware coverage ≥ 95%
- [ ] All 249+ tests passing
- [ ] 0 clippy warnings
- [ ] Performance maintained (<1s for planning_simulator.launch.xml)

---

## Timeline

### Week 1
- **Days 1-3**: Phase 6.1 - XML load_composable_node
  - Day 1: Implementation
  - Day 2: Testing & debugging
  - Day 3: Autoware validation
- **Day 4**: Phase 6.2 - Python LoadComposableNodes
- **Day 5**: Phase 6.3 Investigation - Topic state monitors

### Week 2 (if needed)
- **Days 1-2**: Phase 6.3 Implementation - State monitor pattern
- **Days 3-4**: Phase 6.4 (Optional) - OpaqueFunction file I/O
- **Day 5**: Phase 6.6 - Final testing & documentation

---

## Success Metrics

### Coverage Targets
| Metric | Current | Target | Status |
|--------|---------|--------|--------|
| Regular Nodes | 32/61 (52%) | 58+/61 (95%+) | ❌ |
| Containers | 12/15 (80%) | 14+/15 (93%+) | ❌ |
| Composable Nodes | 38/54 (70%) | 50+/54 (92%+) | ❌ |
| **Overall** | **~67%** | **≥95%** | ❌ |

### Milestone Checklist
- [ ] Phase 6.1: load_composable_node complete
- [ ] Phase 6.2: LoadComposableNodes enhanced
- [ ] Phase 6.3: State monitor pattern implemented
- [ ] Autoware coverage ≥ 80% (critical milestone)
- [ ] Autoware coverage ≥ 90% (stretch goal)
- [ ] Autoware coverage ≥ 95% (target)
- [ ] All tests passing
- [ ] Documentation updated

---

## Risk Assessment

### High Risk
- **Topic state monitor pattern complexity**
  - Mitigation: Investigate thoroughly before implementing
  - Fallback: Accept 90% coverage without this feature

### Medium Risk
- **OpaqueFunction file I/O scope creep**
  - Mitigation: Limit to YAML only, no arbitrary file operations
  - Fallback: Mark as future work if too complex

### Low Risk
- **load_composable_node XML parsing**
  - Mitigation: Similar to existing composable_node logic
  - Well-defined scope and test cases

---

## Post-Phase 6 Work

After reaching 95% Autoware coverage, remaining work:

**Phase 7: Performance & Optimization**
- Parallel include processing
- Python interpreter caching
- Lazy substitution resolution

**Phase 8: Documentation & Release**
- User guide
- API documentation
- Migration guide from dump_launch
- 1.0 release preparation

---

## Notes

- Phase 6.1 (load_composable_node) is **critical** - blocks 20%+ of coverage
- Phase 6.2-6.3 are **high priority** - each adds 5-10% coverage
- Phase 6.4 (file I/O) is **optional** - complex with diminishing returns
- Focus on 80-90% coverage first, then evaluate remaining gaps
