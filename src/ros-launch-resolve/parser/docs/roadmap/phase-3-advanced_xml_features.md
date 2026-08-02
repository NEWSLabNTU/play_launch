# Phase 3: Advanced XML Features

**Status**: 🔄 In Progress (30% Complete)
**Priority**: High
**Estimated Effort**: 2-3 weeks
**Dependencies**: Phase 2 (Complete ✅)

---

## Overview

Extend the XML parser with advanced features to achieve broader compatibility with ROS 2 launch files, including additional element types, substitutions, parameter files, and composable node support.

**Goal:** Support 80%+ of real-world ROS 2 XML launch files including Autoware

---

## Completed Features ✅

- ✅ `$(find-pkg-share)` substitution
- ✅ `<include>` tag with argument passing
- ✅ Conditional attributes (`if`/`unless`)
- ✅ `<group>` tag (basic implementation)
- ✅ `<let>` tag for scoped variables

---

## Remaining Work Items

### Task 3.1: Additional XML Elements 📦 Priority: High

**Objective**: Support more launch element types

**Subtasks**:
- [ ] 3.1.1: Implement `<executable>` tag
  - Non-ROS executable launching
  - Command and arguments
  - Environment variables
  - Output handling
- [ ] 3.1.2: Implement `<set_env>` / `<unset_env>` tags
  - Environment variable manipulation
  - Scoping rules
- [ ] 3.1.3: Implement `<declare_argument>` tag
  - Explicit argument declaration
  - Validation and choices
- [ ] 3.1.4: Implement `<set_parameter>` tag
  - Global parameter setting
  - Parameter types and validation

**Files to Create/Modify**:
- `src/actions/executable.rs`
- `src/actions/env_action.rs`
- `src/actions/mod.rs`
- `src/lib.rs` (traverse_entity)

**Acceptance Criteria**:
- [ ] Can parse and execute `<executable>` elements
- [ ] Environment variable setting/unsetting works
- [ ] Argument declaration with validation
- [ ] Global parameters are set correctly
- [ ] Unit tests for all new elements

**Estimated Time**: 4 days

---

### Task 3.2: Advanced Substitutions 🔄 Priority: High

**Objective**: Implement additional substitution types

**Subtasks**:
- [ ] 3.2.1: Implement `$(dirname)` - Current file directory
- [ ] 3.2.2: Implement `$(filename)` - Current file name
- [ ] 3.2.3: Implement `$(anon name)` - Anonymous name generation
- [ ] 3.2.4: Implement `$(command cmd)` - Command execution
- [ ] 3.2.5: Implement `$(optenv VAR default)` - Optional env var
- [ ] 3.2.6: Add context for file path tracking
- [ ] 3.2.7: Support nested substitutions

**Substitution Design Extensions**:
```rust
pub enum Substitution {
    // ... existing variants
    Dirname,                          // $(dirname)
    Filename,                         // $(filename)
    AnonymousName(String),            // $(anon name)
    Command(String),                  // $(command cmd)
    OptionalEnv { name: String, default: String },  // $(optenv VAR default)
}
```

**Files to Modify**:
- `src/substitution/types.rs`
- `src/substitution/parser.rs`
- `src/substitution/context.rs` (add current_file field)

**Acceptance Criteria**:
- [ ] All new substitution types parse correctly
- [ ] `$(dirname)` and `$(filename)` work with includes
- [ ] Anonymous names are unique
- [ ] Command execution is safe and restricted
- [ ] Nested substitutions work (e.g., `$(var $(var x))`)
- [ ] Unit tests for each substitution type

**Estimated Time**: 5 days

---

### Task 3.3: Parameter File Support 📄 Priority: High

**Objective**: Load and parse YAML parameter files

**Subtasks**:
- [ ] 3.3.1: Add serde_yaml dependency
- [ ] 3.3.2: Implement parameter file loading
- [ ] 3.3.3: Parse YAML parameter structure
- [ ] 3.3.4: Apply parameters to nodes
- [ ] 3.3.5: Handle parameter namespacing
- [ ] 3.3.6: Support `params_file` attribute on nodes
- [ ] 3.3.7: Merge inline params with file params

**Parameter File Format**:
```yaml
# params.yaml
/**:
  ros__parameters:
    param1: value1
    param2: 42
    param3: true

/node_name:
  ros__parameters:
    specific_param: "specific_value"
```

**Files to Create/Modify**:
- `src/params/mod.rs`
- `src/params/loader.rs`
- `src/params/types.rs`
- `src/actions/node.rs` (handle params_file attribute)

**Dependencies**:
```toml
serde_yaml = "0.9"
```

**Acceptance Criteria**:
- [ ] Can load YAML parameter files
- [ ] Parameters apply to correct nodes
- [ ] Namespacing rules followed
- [ ] Inline params override file params
- [ ] Type preservation (string, int, float, bool)
- [ ] Integration tests with parameter files

**Estimated Time**: 4 days

---

### Task 3.4: Full Namespace Scoping 🎯 Priority: Medium

**Objective**: Implement complete namespace scoping for groups

**Subtasks**:
- [ ] 3.4.1: Track namespace stack in context
- [ ] 3.4.2: Apply group namespaces to child nodes
- [ ] 3.4.3: Implement `<push-ros-namespace>` / `<pop-ros-namespace>`
- [ ] 3.4.4: Handle absolute vs relative namespaces
- [ ] 3.4.5: Apply scoped parameters
- [ ] 3.4.6: Apply scoped remappings

**Namespace Context Design**:
```rust
pub struct LaunchContext {
    configurations: HashMap<String, String>,
    namespace_stack: Vec<String>,
    current_file: Option<PathBuf>,
}

impl LaunchContext {
    pub fn push_namespace(&mut self, ns: String) {
        self.namespace_stack.push(ns);
    }

    pub fn pop_namespace(&mut self) {
        self.namespace_stack.pop();
    }

    pub fn current_namespace(&self) -> String {
        self.namespace_stack.join("/")
    }
}
```

**Files to Modify**:
- `src/substitution/context.rs`
- `src/actions/group.rs`
- `src/lib.rs` (namespace handling in traverse)
- `src/record/generator.rs` (apply scoped namespace)

**Acceptance Criteria**:
- [ ] Group namespaces apply to children
- [ ] Namespace stack works correctly
- [ ] Absolute namespaces override stack
- [ ] Nested groups work correctly
- [ ] Parameters and remaps respect namespaces
- [ ] Unit and integration tests pass

**Estimated Time**: 3 days

---

### Task 3.5: Composable Node Support 🧩 Priority: Medium

**Objective**: Support composable nodes and containers

**Subtasks**:
- [ ] 3.5.1: Define ComposableNode action
- [ ] 3.5.2: Define NodeContainer action
- [ ] 3.5.3: Define LoadComposableNode action
- [ ] 3.5.4: Parse composable node elements
- [ ] 3.5.5: Generate ComposableNodeContainerRecord
- [ ] 3.5.6: Generate LoadNodeRecord
- [ ] 3.5.7: Handle component plugin loading metadata
- [ ] 3.5.8: Test with real composable node examples

**Data Structures**:
```rust
#[derive(Debug, Clone)]
pub struct ComposableNodeAction {
    pub package: Vec<Substitution>,
    pub plugin: Vec<Substitution>,
    pub name: Option<Vec<Substitution>>,
    pub namespace: Option<Vec<Substitution>>,
    pub parameters: Vec<Parameter>,
    pub remappings: Vec<Remapping>,
    pub extra_arguments: Vec<ExtraArgument>,
}

#[derive(Debug, Clone)]
pub struct NodeContainerAction {
    pub name: Vec<Substitution>,
    pub namespace: Option<Vec<Substitution>>,
    pub package: Vec<Substitution>,
    pub executable: Vec<Substitution>,
    pub composable_nodes: Vec<ComposableNodeAction>,
}
```

**Files to Create**:
- `src/actions/composable_node.rs`
- `src/actions/container.rs`
- `src/actions/load_node.rs`

**Acceptance Criteria**:
- [ ] Parse composable node declarations
- [ ] Parse container declarations
- [ ] Generate container records
- [ ] Generate load node records
- [ ] Component metadata correct
- [ ] Integration test with demo composable nodes

**Estimated Time**: 5 days

---

### Task 3.6: Event Handler Framework 🔔 Priority: Low

**Objective**: Support launch event handlers

**Subtasks**:
- [ ] 3.6.1: Define Event and EventHandler traits
- [ ] 3.6.2: Implement OnInclude events
- [ ] 3.6.3: Implement OnExit events
- [ ] 3.6.4: Implement OnShutdown events
- [ ] 3.6.5: Event registration system
- [ ] 3.6.6: Event execution (deferred)
- [ ] 3.6.7: Event action types (log, emit, timer)

**Event System Design**:
```rust
pub trait Event {
    fn event_type(&self) -> &str;
}

pub trait EventHandler {
    fn on_event(&mut self, event: &dyn Event) -> Result<()>;
}

pub struct OnIncludeEvent {
    pub file_path: PathBuf,
    pub context: LaunchContext,
}

pub struct OnExitEvent {
    pub node_name: String,
    pub exit_code: i32,
}
```

**Files to Create**:
- `src/events/mod.rs`
- `src/events/types.rs`
- `src/events/handlers.rs`

**Acceptance Criteria**:
- [ ] Event trait defined
- [ ] Basic event types implemented
- [ ] Event registration works
- [ ] Events captured in record.json
- [ ] Unit tests for event system

**Estimated Time**: 4 days

**Note**: Event *execution* is deferred to runtime (play_launch), not parsing phase

---

### Task 3.7: Additional Improvements ✨ Priority: Low

**Objective**: Miscellaneous improvements and edge cases

**Subtasks**:
- [ ] 3.7.1: Better error messages with line numbers
- [ ] 3.7.2: Validation warnings for deprecated features
- [ ] 3.7.3: Launch file version checking
- [ ] 3.7.4: Improved package path resolution (ament_index)
- [ ] 3.7.5: Configuration file support (.yaml config)
- [ ] 3.7.6: Dry-run mode (parse without execution)
- [ ] 3.7.7: JSON output validation

**Acceptance Criteria**:
- [ ] Error messages include file and line information
- [ ] Deprecated feature warnings shown
- [ ] Version compatibility checking works
- [ ] Package resolution uses ament_index
- [ ] Configuration file loaded correctly

**Estimated Time**: 3 days

---

## Testing Strategy

### Unit Tests

**Coverage Target**: 90%+

**New Test Files**:
- `src/actions/executable_test.rs`
- `src/actions/env_action_test.rs`
- `src/params/loader_test.rs`
- `src/substitution/advanced_test.rs`
- `src/events/handlers_test.rs`

### Integration Tests

**Test Cases**:
1. Launch with executable elements
2. Launch with parameter files
3. Launch with composable nodes
4. Launch with nested namespaces
5. Launch with event handlers
6. Complex launch combining multiple features

### Real-World Tests

**Target Launch Files**:
- Autoware launch files (universe/launch)
- Navigation2 launch files
- Gazebo + RViz launch files
- Multi-robot launch files

---

## Success Criteria

- [ ] 80%+ of real-world launch files parse successfully
- [ ] All advanced substitution types work
- [ ] Parameter files load and apply correctly
- [ ] Composable nodes supported
- [ ] Full namespace scoping works
- [ ] 90%+ unit test coverage
- [ ] Integration tests with real launch files pass
- [ ] Performance: <500ms for Autoware launch tree

---

## Dependencies

**New Rust Crates**:
```toml
serde_yaml = "0.9"        # Parameter file loading
sha2 = "0.10"             # For anon name generation
```

**System Dependencies**:
- ament_index (for package resolution) - optional for now

---

## Deliverables

**Code**:
- [ ] Additional action types (executable, env, etc.)
- [ ] Advanced substitution types
- [ ] Parameter file loading system
- [ ] Full namespace scoping
- [ ] Composable node support
- [ ] Event handler framework
- [ ] Comprehensive tests

**Documentation**:
- [ ] Updated feature list
- [ ] Examples for new features
- [ ] API documentation updates
- [ ] Migration guide from dump_launch

---

## Timeline Estimate

**Total Estimated Time**: 2-3 weeks (28 working days)

**Breakdown**:
- Task 3.1: Additional Elements (4 days)
- Task 3.2: Advanced Substitutions (5 days)
- Task 3.3: Parameter Files (4 days)
- Task 3.4: Namespace Scoping (3 days)
- Task 3.5: Composable Nodes (5 days)
- Task 3.6: Event Handlers (4 days)
- Task 3.7: Improvements (3 days)

**Buffer**: +5 days for unexpected issues

---

## Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Nested substitution complexity | High | Medium | Implement iterative resolver, limit depth |
| Parameter file format variations | Medium | Medium | Study real examples, support common patterns |
| Composable node API complexity | Medium | High | Refer to launch_ros source code extensively |
| Event handler scope creep | Medium | Medium | Implement minimal framework, defer execution |
| ament_index FFI complexity | Low | Medium | Use subprocess fallback for MVP |

---

## Next Phase

**Phase 4: YAML Support**
- YAML launch file parsing
- Python-style launch files
- Integration with existing XML parser

**Phase 5: Performance & Validation**
- Benchmarking suite
- Comparison testing with dump_launch
- Optimization for large launch trees
- Production readiness

---

## References

- [launch_ros Source Code](https://github.com/ros2/launch_ros)
