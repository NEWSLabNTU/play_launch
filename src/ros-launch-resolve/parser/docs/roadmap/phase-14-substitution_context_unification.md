# Phase 14: Substitution Context Unification

**Status**: 📋 Planned
**Priority**: High
**Complexity**: Medium
**Estimated Effort**: 3-5 days

## Overview

Unify Rust and Python substitution contexts to properly share launch configurations, environment variables, and namespace state between Rust and Python execution contexts. This fixes nested substitution resolution and aligns with the original ROS 2 launch package architecture.

## Problem Statement

### Current Architecture Issues

**Issue 1: Disconnected Contexts**
- **Python side**: `LAUNCH_CONFIGURATIONS` global Mutex<HashMap>
- **Rust side**: `LaunchContext` struct with its own configuration storage
- **Problem**: When LaunchConfiguration.perform() resolves nested substitutions, it creates an empty Rust context that doesn't have access to Python's LAUNCH_CONFIGURATIONS

**Issue 2: Nested Substitution Resolution**
```python
# In Python launch file:
DeclareLaunchArgument('base_path', default_value='$(find-pkg-share my_pkg)')
DeclareLaunchArgument('full_path', default_value=[LaunchConfiguration('base_path'), '/config'])
```

When resolving `full_path`:
1. Python stores: `LAUNCH_CONFIGURATIONS['full_path'] = "$(var base_path)/config"`
2. Later, OpaqueFunction calls `LaunchConfiguration('full_path').perform(context)`
3. Our Rust `LaunchConfiguration.perform()` parses `"$(var base_path)/config"`
4. **BUG**: Creates empty `LaunchContext::new()` which doesn't have `base_path`
5. **Result**: `$(var base_path)` fails to resolve

**Issue 3: No Context Passing from Python**
```rust
// Current implementation in substitutions.rs:51-55
fn perform(&self, _context: &PyAny) -> PyResult<String> {
    use crate::python::bridge::LAUNCH_CONFIGURATIONS;
    // ...
    let ctx = LaunchContext::new();  // ← Empty context!
    resolve_substitutions(&subs, &ctx)  // ← Missing launch_configurations
}
```

### How Original ROS 2 Launch Works

**Python LaunchContext** (`/opt/ros/humble/lib/python3.10/site-packages/launch/launch_context.py`):
```python
class LaunchContext:
    def __init__(self):
        self.__launch_configurations = {}  # Dict[str, str]
        self.__environment_stack = []
        # ... other state

    @property
    def launch_configurations(self) -> Dict[str, str]:
        return self.__launch_configurations

    def perform_substitution(self, substitution: Substitution) -> str:
        return substitution.perform(self)  # ← Pass context to substitution
```

**Python LaunchConfiguration.perform()** (`launch/substitutions/launch_configuration.py:87-102`):
```python
def perform(self, context: LaunchContext) -> str:
    from ..utilities import perform_substitutions
    expanded_variable_name = perform_substitutions(context, self.__variable_name)
    if expanded_variable_name not in context.launch_configurations:
        if self.__default is None:
            raise SubstitutionFailure(...)
        else:
            return perform_substitutions(context, self.__default)
    return context.launch_configurations[expanded_variable_name]  # ← Direct lookup
```

**Key Insight**: The original implementation stores **already-resolved values** in `context.launch_configurations`, NOT substitution strings. When a LaunchConfiguration is declared with substitutions in its default value, those substitutions are resolved immediately and the final value is stored.

**Our Implementation Difference**: We store substitution strings like `"$(find-pkg-share pkg)/path"` in LAUNCH_CONFIGURATIONS and resolve them lazily during `.perform()`.

## Current State Analysis

### Global State in Rust

**File**: `src/python/bridge.rs`

```rust
// Line 13-14: Launch configurations storage
pub static LAUNCH_CONFIGURATIONS: Lazy<Arc<Mutex<HashMap<String, String>>>> = ...;

// Line 17-19: ROS namespace stack
pub static ROS_NAMESPACE_STACK: Lazy<Arc<Mutex<Vec<String>>>> = ...;
```

**What's stored**:
- `LAUNCH_CONFIGURATIONS`: Launch argument values (may contain unresolved substitution strings)
- `ROS_NAMESPACE_STACK`: Current namespace hierarchy
- Environment variables: Accessed via `std::env` (OS-level)

### Rust LaunchContext

**File**: `src/substitution/context.rs`

```rust
pub struct LaunchContext {
    configurations: HashMap<String, String>,  // ← Separate from LAUNCH_CONFIGURATIONS!
    global_parameters: HashMap<String, String>,
    environment: HashMap<String, String>,
    namespace_stack: Vec<String>,
    // ...
}

impl LaunchContext {
    pub fn new() -> Self {
        Self {
            configurations: HashMap::new(),  // ← Empty!
            // ...
        }
    }

    pub fn get_configuration_lenient(&self, name: &str) -> Result<String, SubstitutionError> {
        self.configurations.get(name)  // ← Only checks local HashMap
            .ok_or_else(|| SubstitutionError::UndefinedVariable(name.to_string()))
    }
}
```

### Python LaunchConfiguration.perform()

**File**: `src/python/api/substitutions.rs:51-90`

```rust
fn perform(&self, _context: &PyAny) -> PyResult<String> {
    use crate::python::bridge::LAUNCH_CONFIGURATIONS;
    use crate::substitution::context::LaunchContext;
    use crate::substitution::parser::parse_substitutions;
    use crate::substitution::types::resolve_substitutions;

    let configs = LAUNCH_CONFIGURATIONS.lock();
    let value = if let Some(v) = configs.get(&self.variable_name) {
        v.clone()
    } else if let Some(ref default) = self.default {
        default.clone()
    } else {
        String::new()
    };

    // If value contains substitution syntax, resolve it
    let result = if value.contains("$(") {
        let subs = parse_substitutions(&value).map_err(...)?;

        let ctx = LaunchContext::new();  // ← BUG: Empty context!
        resolve_substitutions(&subs, &ctx).map_err(...)?
    } else {
        value
    };

    Ok(result)
}
```

**Problems**:
1. Ignores `_context: &PyAny` parameter (the Python MockLaunchContext)
2. Creates empty Rust `LaunchContext::new()`
3. Nested `$(var other_var)` references fail because `other_var` isn't in the empty context

### MockLaunchContext (Python)

**File**: `src/python/api/actions.rs:162-180`

```python
class MockLaunchContext:
    def __init__(self, launch_configurations, ros_namespace, resolve_sub_fn):
        self.launch_configurations = launch_configurations  # ← Has the data!
        self.launch_configurations['ros_namespace'] = ros_namespace
        self.resolve_sub_fn = resolve_sub_fn

    def perform_substitution(self, sub):
        # Support LaunchConfiguration.perform(context)
        if hasattr(sub, 'variable_name'):
            value = self.launch_configurations.get(sub.variable_name, '')
            # Check if value contains substitution syntax
            if isinstance(value, str) and '$(' in value:
                # Resolve nested substitutions via Rust
                return self.resolve_sub_fn(value)
            return value
        # ... handle other substitution types
```

**Gap**: The Rust `LaunchConfiguration.perform()` doesn't use this context!

## Proposed Solution

### Architecture

**Unified Context Flow**:
```
Python MockLaunchContext (has launch_configurations dict)
         ↓
   PyAny context parameter
         ↓
Rust LaunchConfiguration.perform(context: &PyAny)
         ↓
Extract launch_configurations from PyAny
         ↓
Populate Rust LaunchContext
         ↓
Resolve nested substitutions
```

### Implementation Approach

**Option A: Extract from Python Context (Recommended)**
- Extract `launch_configurations` dict from Python `context` parameter
- Populate Rust `LaunchContext` with this data
- Resolve substitutions using populated context
- **Pros**: Aligns with original ROS 2 architecture, clean separation
- **Cons**: PyO3 dict extraction overhead (minimal)

**Option B: Direct Global Access**
- Keep using `LAUNCH_CONFIGURATIONS` global
- Populate Rust `LaunchContext` from global state
- **Pros**: Simpler, no PyO3 extraction
- **Cons**: Doesn't align with original architecture, less flexible

**Recommendation**: **Option A** - Extract from Python context for proper architectural alignment.

### Helper Function Design

**File**: `src/python/api/substitutions.rs` (new function)

```rust
/// Create a LaunchContext populated with data from Python context
fn create_context_from_python(py_context: &PyAny) -> PyResult<LaunchContext> {
    use crate::substitution::context::LaunchContext;
    use pyo3::types::PyDict;

    // Extract launch_configurations from Python context
    let launch_configs = py_context
        .getattr("launch_configurations")?
        .downcast::<PyDict>()?;

    // Create Rust context
    let mut ctx = LaunchContext::new();

    // Populate with launch configurations
    for (key, value) in launch_configs.iter() {
        let key_str: String = key.extract()?;
        let value_str: String = value.extract()?;
        ctx.set_configuration(key_str, value_str);
    }

    // TODO: Also extract and populate:
    // - Environment variables (if needed beyond std::env)
    // - ROS namespace stack
    // - Global parameters

    Ok(ctx)
}

/// Parse and resolve substitution string (with micro-optimization)
fn resolve_substitution_string(value: &str, context: &LaunchContext) -> Result<String, SubstitutionError> {
    use crate::substitution::parser::parse_substitutions;
    use crate::substitution::types::resolve_substitutions;

    // Micro-optimization: skip parsing if no substitution syntax
    if !value.contains("$(") {
        return Ok(value.to_string());
    }

    // Parse and resolve
    let subs = parse_substitutions(value)?;
    resolve_substitutions(&subs, context)
}
```

**Updated LaunchConfiguration.perform()**:
```rust
fn perform(&self, context: &PyAny) -> PyResult<String> {
    use crate::python::bridge::LAUNCH_CONFIGURATIONS;

    // Get value from LAUNCH_CONFIGURATIONS
    let configs = LAUNCH_CONFIGURATIONS.lock();
    let value = if let Some(v) = configs.get(&self.variable_name) {
        v.clone()
    } else if let Some(ref default) = self.default {
        default.clone()
    } else {
        String::new()
    };
    drop(configs); // Release lock early

    // Create Rust context from Python context
    let rust_context = create_context_from_python(context)?;

    // Resolve any nested substitutions
    let result = resolve_substitution_string(&value, &rust_context)
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(format!(
            "Failed to resolve substitutions in '{}': {}",
            value, e
        )))?;

    Ok(result)
}
```

## Implementation Plan

### Phase 14.1: Helper Functions (1 day)

**Tasks**:
- [ ] Create `create_context_from_python(py_context: &PyAny) -> PyResult<LaunchContext>`
  - [ ] Extract `launch_configurations` dict from Python context
  - [ ] Populate Rust `LaunchContext` with configurations
  - [ ] Add unit tests for context extraction
- [ ] Create `resolve_substitution_string(value: &str, context: &LaunchContext) -> Result<String, SubstitutionError>`
  - [ ] Include `contains("$(")` micro-optimization
  - [ ] Add unit tests for nested resolution

**Files**:
- `src/python/api/substitutions.rs` - Add helper functions
- `src/python/api/substitutions.rs` - Add tests

**Tests**:
```rust
#[test]
fn test_create_context_from_python() {
    Python::with_gil(|py| {
        // Create Python dict
        let py_dict = PyDict::new(py);
        py_dict.set_item("my_var", "value123").unwrap();
        py_dict.set_item("other_var", "hello").unwrap();

        // Create mock context
        let mock_context = py.eval(
            "type('MockContext', (), {'launch_configurations': d})()",
            Some([("d", py_dict)].into_py_dict(py)),
            None
        ).unwrap();

        // Extract to Rust context
        let ctx = create_context_from_python(mock_context).unwrap();

        // Verify
        assert_eq!(ctx.get_configuration("my_var").unwrap(), "value123");
        assert_eq!(ctx.get_configuration("other_var").unwrap(), "hello");
    });
}

#[test]
fn test_resolve_substitution_string_literal() {
    let ctx = LaunchContext::new();
    let result = resolve_substitution_string("hello", &ctx).unwrap();
    assert_eq!(result, "hello");
}

#[test]
fn test_resolve_substitution_string_with_var() {
    let mut ctx = LaunchContext::new();
    ctx.set_configuration("my_var", "world");

    let result = resolve_substitution_string("$(var my_var)", &ctx).unwrap();
    assert_eq!(result, "world");
}

#[test]
fn test_resolve_substitution_string_nested() {
    let mut ctx = LaunchContext::new();
    ctx.set_configuration("base_path", "/opt/ros/humble/share/pkg");

    let result = resolve_substitution_string("$(var base_path)/config", &ctx).unwrap();
    assert_eq!(result, "/opt/ros/humble/share/pkg/config");
}
```

### Phase 14.2: Update LaunchConfiguration.perform() (1 day)

**Tasks**:
- [ ] Update `LaunchConfiguration.perform()` to use `create_context_from_python()`
- [ ] Use `resolve_substitution_string()` for resolution
- [ ] Update error messages to be clear and actionable
- [ ] Ensure no fallback to original value (strict parsing)
- [ ] Test with existing Python launch files

**Files**:
- `src/python/api/substitutions.rs` - Update `perform()` method

**Changes**:
```rust
fn perform(&self, context: &PyAny) -> PyResult<String> {
    // ... (get value from LAUNCH_CONFIGURATIONS)

    // Create populated Rust context from Python context
    let rust_context = create_context_from_python(context)?;

    // Resolve nested substitutions
    resolve_substitution_string(&value, &rust_context)
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(format!(
            "Failed to resolve substitutions in '{}': {}",
            value, e
        )))
}
```

### Phase 14.3: Update Other Substitution Types (1 day)

**Tasks**:
- [ ] Review all PyO3 substitution classes that have `perform()` methods
  - [ ] `EnvironmentVariable` (substitutions.rs)
  - [ ] `FindPackageShare` (substitutions.rs)
  - [ ] `Command` (substitutions.rs)
  - [ ] Any others with `perform(context)` methods
- [ ] Update to use `create_context_from_python()` where needed
- [ ] Ensure consistent error handling
- [ ] Test each substitution type individually

**Example**:
```rust
// EnvironmentVariable.perform()
fn perform(&self, context: &PyAny) -> PyResult<String> {
    let rust_context = create_context_from_python(context)?;

    // Resolve name (which might contain substitutions)
    let name = resolve_substitution_string(&self.name, &rust_context)
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(format!(...)))?;

    // Get environment variable
    std::env::var(&name)
        .or_else(|_| {
            if let Some(ref default) = self.default {
                resolve_substitution_string(default, &rust_context)
            } else {
                Err(...)
            }
        })
        .map_err(...)
}
```

### Phase 14.4: Integration Testing (1 day)

**Tasks**:
- [ ] Test with existing fixtures (should all pass)
- [ ] Add new test fixtures for complex nested substitutions
  - [ ] Create `test_nested_launch_config.launch.py`
  - [ ] Test nested `$(var $(var name))` patterns
  - [ ] Test `$(var base_path)/suffix` patterns
- [ ] Test Autoware planning_simulator (full integration)
- [ ] Run all 297 unit tests
- [ ] Run comparison tests (Rust vs Python parser)
- [ ] Benchmark performance (ensure no regression)

**New test fixtures**:

**Fixture**: `test_nested_launch_config.launch.py`
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Test nested LaunchConfiguration resolution
    full_path = LaunchConfiguration('full_path').perform(context)
    print(f"Resolved path: {full_path}")
    # Should be: /opt/ros/humble/share/demo_nodes_cpp/config/params.yaml
    return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pkg_share',
            default_value=FindPackageShare('demo_nodes_cpp')
        ),
        DeclareLaunchArgument(
            'full_path',
            default_value=[LaunchConfiguration('pkg_share'), '/config/params.yaml']
        ),
        OpaqueFunction(function=launch_setup),
    ])
```

**Test**:
```rust
#[test]
fn test_nested_launch_configuration_resolution() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_nested_launch_config.launch.py");

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok(), "Should resolve nested LaunchConfiguration");
}
```

### Phase 14.5: Documentation and Cleanup (1 day)

**Tasks**:
- [ ] Update CLAUDE.md with new substitution architecture
  - [ ] Document `create_context_from_python()` usage
  - [ ] Document `resolve_substitution_string()` usage
  - [ ] Update "Key Recent Changes" section
- [ ] Document helper functions with rustdoc
  - [ ] Add examples to function documentation
  - [ ] Document error conditions
- [ ] Update roadmap status
  - [ ] Mark Phase 14 as complete
  - [ ] Update implementation_status.md
- [ ] Code review and cleanup
  - [ ] Remove any unused code
  - [ ] Ensure consistent naming
- [ ] Performance verification (no regressions)
  - [ ] Benchmark context creation overhead
  - [ ] Verify < 1% overhead

**Documentation additions**:

**CLAUDE.md**:
```markdown
### Substitution Resolution Architecture (Phase 14)

**Context Unification**: Rust and Python contexts are now properly unified:

1. Python `MockLaunchContext` contains `launch_configurations` dict
2. `LaunchConfiguration.perform(context)` receives Python context as `&PyAny`
3. `create_context_from_python()` extracts dict and populates Rust `LaunchContext`
4. `resolve_substitution_string()` handles nested substitution resolution
5. All substitution types use the same context population pattern

**Helper Functions**:
- `create_context_from_python(py_context: &PyAny) -> PyResult<LaunchContext>`
- `resolve_substitution_string(value: &str, context: &LaunchContext) -> Result<String, SubstitutionError>`

**Key Behavior**:
- Nested `$(var ...)` references are now properly resolved
- Strict parsing: no fallback to original value on resolution failure
- Context data flows: Python → PyO3 extraction → Rust LaunchContext → Resolution
```

## Success Criteria

- [ ] All 297 existing unit tests pass
- [ ] Nested LaunchConfiguration resolution works (e.g., `$(var base_path)/config` where `base_path` contains `$(find-pkg-share pkg)`)
- [ ] Autoware planning_simulator test passes
- [ ] No performance regression (context extraction overhead < 1%)
- [ ] No clippy warnings
- [ ] Code formatted with rustfmt
- [ ] All quality checks pass (`just quality`)

## Risks and Mitigations

**Risk 1: PyO3 Dict Extraction Performance**
- **Impact**: Medium
- **Likelihood**: Low
- **Mitigation**: Benchmark context creation overhead; optimize if > 1%

**Risk 2: Missing Context Data**
- **Impact**: High
- **Likelihood**: Medium
- **Mitigation**: Comprehensive testing with all substitution types; add environment and namespace if needed

**Risk 3: Test Failures**
- **Impact**: Medium
- **Likelihood**: Low
- **Mitigation**: Incremental changes with testing after each phase

## Future Enhancements

**Phase 14.5: Eliminate Global State** (High Priority Follow-up)

After Phase 14 is complete and working, refactor to eliminate global state entirely:
- Replace `LAUNCH_CONFIGURATIONS` global with local `ParseContext` struct
- Replace `ROS_NAMESPACE_STACK` global with context field
- Replace `CAPTURED_*` globals with context fields
- Thread `ParseContext` through parsing pipeline
- Pass context to Python via PyO3

**Benefits**:
- Cleaner architecture (no hidden globals)
- Better testability (each test gets own context)
- No Mutex overhead (single-threaded parsing)
- Matches original ROS 2 design
- Parallelization-ready

**See**: [Phase 14.5 Roadmap](./phase-14_5-eliminate_global_state.md) for detailed plan.

**Phase 14.1: Full Context Parity** (Optional)
- Extract and populate environment variables from Python context
- Extract and populate ROS namespace stack
- Extract global parameters
- Full bidirectional sync

**Phase 14.2: Performance Optimization** (Optional)
- Cache populated contexts per OpaqueFunction invocation
- Lazy context population (only when needed)
- Benchmark and optimize dict extraction

## Related Documents

- **Original Issue**: Nested substitution resolution failure
- **Test Failures**: `test_parameter_file_usage` (fixed by using `demo_nodes_cpp`)
- **ROS 2 Source**: `/opt/ros/humble/lib/python3.10/site-packages/launch/launch_context.py`
- **PyO3 Docs**: https://pyo3.rs/v0.23.3/

## References

**Original ROS 2 Launch Package**:
- `launch/launch_context.py` - Context class with `launch_configurations` dict
- `launch/substitutions/launch_configuration.py` - LaunchConfiguration.perform()
- Context passing: All substitutions receive `context` parameter

**Our Implementation**:
- `src/python/bridge.rs` - Global state (LAUNCH_CONFIGURATIONS, ROS_NAMESPACE_STACK)
- `src/substitution/context.rs` - Rust LaunchContext
- `src/python/api/substitutions.rs` - PyO3 LaunchConfiguration
- `src/python/api/actions.rs` - MockLaunchContext

**Follow-up Work**:
- [Phase 14.5: Eliminate Global State](./phase-14_5-eliminate_global_state.md) - Replace globals with local ParseContext
