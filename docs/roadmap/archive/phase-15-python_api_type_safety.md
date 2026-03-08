# Phase 15: Python API Type Safety Improvements

**Status**: ✅ Complete (2026-01-31)
**Priority**: High
**Actual Duration**: 1 day (implemented Phases 15.1-15.4)
**Dependencies**: Phase 14 (Python Execution) completed ✅

## Overview

This phase fixes type handling issues in our Python API implementation to ensure full compatibility with ROS 2's `SomeSubstitutionsType` pattern. Recent bugs (LogInfo, PythonExpression) revealed that several action classes still use `String` where they should accept `PyObject` to handle substitutions like `LaunchConfiguration`, lists, and other dynamic values.

**Root Cause**: ROS 2 uses `SomeSubstitutionsType` extensively, which is a union type accepting strings, substitutions, or iterables. Our PyO3 implementation must accept `PyObject` for these parameters to handle all three cases.

## Motivation

### Recent Bug Example

```python
# This failed before our fix:
LogInfo(msg=['Demo: ', LaunchConfiguration('mode')])
# Error: 'LaunchConfiguration' object cannot be converted to 'PyString'

# After fix (LogInfo now accepts PyObject):
LogInfo(msg=['Demo: ', LaunchConfiguration('mode')])  # ✅ Works!
```

### Current Issues

**4 classes with similar type handling problems:**

1. **SetEnvironmentVariable** - Cannot use substitutions for name/value
2. **AppendEnvironmentVariable** - Cannot use substitutions for name/separator
3. **ExecuteProcess** - Cannot use substitutions in cmd array
4. **Node.arguments** - Cannot use substitutions in arguments

### Impact

Real-world launch files that will fail:

```python
# Environment variable with dynamic value (FAILS)
SetEnvironmentVariable(
    name='ROS_DOMAIN_ID',
    value=LaunchConfiguration('domain_id')  # ❌ TypeError
)

# Append path with package prefix (FAILS)
AppendEnvironmentVariable(
    name='PYTHONPATH',
    value=[FindPackageShare('my_pkg'), '/scripts'],  # ❌ TypeError
    separator=':'
)

# Node with dynamic arguments (FAILS)
Node(
    package='my_pkg',
    executable='my_node',
    arguments=['--config', LaunchConfiguration('config_file')]  # ❌ TypeError
)
```

## Goals

✅ Fix all `String` parameters that should accept `SomeSubstitutionsType`
✅ Implement consistent `pyobject_to_string` conversion pattern
✅ Add comprehensive tests for each fix
✅ Document the pattern for future API additions
✅ Maintain backward compatibility (strings still work)

## Non-Goals

❌ Change parameters that correctly require literal strings (e.g., DeclareLaunchArgument.name)
❌ Modify classes that already correctly accept PyObject
❌ Add new ROS 2 features beyond type safety

## Technical Background

### SomeSubstitutionsType Definition

From ROS 2 source (`external/launch/launch/launch/some_substitutions_type.py`):

```python
SomeSubstitutionsType = Union[
    Text,                              # Plain string
    Substitution,                      # LaunchConfiguration, FindPackageShare, etc.
    Iterable[Union[Text, Substitution]], # Lists mixing strings and substitutions
]
```

### PyO3 Conversion Pattern

Our standard pattern (from LogInfo/PythonExpression fixes):

```rust
fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
    use pyo3::types::PyList;

    // Try direct string extraction
    if let Ok(s) = obj.extract::<String>(py) {
        return Ok(s);
    }

    // Try list (concatenate all elements)
    if let Ok(list) = obj.downcast::<PyList>(py) {
        let mut result = String::new();
        for item in list.iter() {
            if let Ok(s) = item.extract::<String>() {
                result.push_str(&s);
            } else if let Ok(str_result) = item.call_method0("__str__") {
                if let Ok(s) = str_result.extract::<String>() {
                    result.push_str(&s);
                }
            }
        }
        return Ok(result);
    }

    // Try calling __str__ method (for LaunchConfiguration, etc.)
    if let Ok(str_result) = obj.call_method0(py, "__str__") {
        if let Ok(s) = str_result.extract::<String>(py) {
            return Ok(s);
        }
    }

    // Fallback
    Ok(obj.to_string())
}
```

## Implementation Plan

### Phase 15.1: SetEnvironmentVariable (HIGH PRIORITY) ✅

**File**: `src/play_launch_parser/src/play_launch_parser/src/python/api/actions.rs:360-376`

**Current Code**:
```rust
pub struct SetEnvironmentVariable {
    name: String,   // ❌
    value: String,  // ❌
}

#[pymethods]
impl SetEnvironmentVariable {
    #[new]
    fn new(name: String, value: String) -> Self {
        log::debug!("Python Launch SetEnvironmentVariable: {}={}", name, value);
        Self { name, value }
    }
}
```

**Tasks**:
- [ ] Change struct fields to accept dynamic values:
  ```rust
  pub struct SetEnvironmentVariable {
      name: String,   // Keep as String (resolved value)
      value: String,  // Keep as String (resolved value)
  }
  ```

- [ ] Update `new()` to accept `PyObject` and convert:
  ```rust
  #[new]
  fn new(
      py: Python,
      name: PyObject,
      value: PyObject,
  ) -> PyResult<Self> {
      let name_str = Self::pyobject_to_string(py, &name)?;
      let value_str = Self::pyobject_to_string(py, &value)?;

      log::debug!("Python Launch SetEnvironmentVariable: {}={}", name_str, value_str);
      Ok(Self {
          name: name_str,
          value: value_str
      })
  }
  ```

- [ ] Add `pyobject_to_string` helper method (reuse pattern from LogInfo)
- [ ] Update tests to cover:
  - Plain strings: `SetEnvironmentVariable('VAR', 'value')`
  - Single substitution: `SetEnvironmentVariable('VAR', LaunchConfiguration('val'))`
  - List: `SetEnvironmentVariable('VAR', ['prefix_', LaunchConfiguration('suffix')])`

**Success Criteria**:
- ✅ All three test cases pass
- ✅ Existing string-only usage still works
- ✅ No clippy warnings
- ✅ Autoware validation passes

---

### Phase 15.2: AppendEnvironmentVariable (HIGH PRIORITY) ✅

**File**: `src/play_launch_parser/src/play_launch_parser/src/python/api/actions.rs:995-1069`

**Current Code**:
```rust
pub struct AppendEnvironmentVariable {
    name: String,      // ❌ Should accept PyObject
    value: PyObject,   // ✅ Already correct
    prepend: bool,     // ⚠️ Should accept PyObject (bool or substitution)
    separator: String, // ❌ Should accept PyObject
}

#[pymethods]
impl AppendEnvironmentVariable {
    #[new]
    #[pyo3(signature = (name, value, *, prepend=false, separator=":", **_kwargs))]
    fn new(
        py: Python,
        name: String,              // ❌
        value: PyObject,           // ✅
        prepend: Option<bool>,     // ⚠️
        separator: Option<&str>,   // ❌
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> Self
```

**Tasks**:
- [ ] Update struct to store resolved values:
  ```rust
  pub struct AppendEnvironmentVariable {
      name: String,      // Resolved value
      value: PyObject,   // Keep as PyObject (resolved later)
      prepend: bool,     // Resolved boolean
      separator: String, // Resolved separator
  }
  ```

- [ ] Change `new()` signature to accept PyObject:
  ```rust
  #[new]
  #[pyo3(signature = (name, value, *, prepend=None, separator=None, **_kwargs))]
  fn new(
      py: Python,
      name: PyObject,
      value: PyObject,
      prepend: Option<PyObject>,
      separator: Option<PyObject>,
      _kwargs: Option<&pyo3::types::PyDict>,
  ) -> PyResult<Self> {
      let name_str = Self::pyobject_to_string(py, &name)?;

      // Handle prepend (bool or substitution resolving to bool)
      let prepend_val = if let Some(p) = prepend {
          if let Ok(b) = p.extract::<bool>(py) {
              b
          } else if let Ok(s) = Self::pyobject_to_string(py, &p) {
              // Parse string as bool (YAML rules: true, True, yes, etc.)
              matches!(s.to_lowercase().as_str(), "true" | "yes" | "1")
          } else {
              false
          }
      } else {
          false
      };

      // Handle separator (default to os.pathsep equivalent, typically ":")
      let sep_str = if let Some(s) = separator {
          Self::pyobject_to_string(py, &s)?
      } else {
          ":".to_string()
      };

      log::debug!(
          "Python Launch AppendEnvironmentVariable: {}{}{}{}",
          if prepend_val { "prepending " } else { "appending " },
          "<value>",
          sep_str,
          name_str
      );

      Ok(Self {
          name: name_str,
          value,
          prepend: prepend_val,
          separator: sep_str,
      })
  }
  ```

- [ ] Add `pyobject_to_string` helper (if not already shared)
- [ ] Update tests to cover:
  - Substitution for name: `AppendEnvironmentVariable(LaunchConfiguration('var_name'), 'value')`
  - Substitution for separator: `AppendEnvironmentVariable('PATH', 'value', separator=LaunchConfiguration('sep'))`
  - Substitution for prepend: `AppendEnvironmentVariable('PATH', 'value', prepend=LaunchConfiguration('should_prepend'))`

**ROS 2 Reference**: `external/launch/launch/launch/actions/append_environment_variable.py:45-68`

**Success Criteria**:
- ✅ All substitution cases work
- ✅ Default values work when parameters omitted
- ✅ YAML-style bool parsing works for prepend
- ✅ No regressions in existing usage

---

### Phase 15.3: ExecuteProcess (MEDIUM PRIORITY) ✅

**File**: `src/play_launch_parser/src/play_launch_parser/src/python/api/actions.rs:469-502`

**Current Code**:
```rust
pub struct ExecuteProcess {
    cmd: Vec<String>,        // ⚠️ Should accept Vec<PyObject>
    cwd: Option<String>,     // ⚠️ Should accept PyObject
    name: Option<String>,    // ⚠️ Should accept PyObject
    output: String,
}

#[pymethods]
impl ExecuteProcess {
    #[new]
    #[pyo3(signature = (*, cmd, cwd=None, name=None, output=None, **_kwargs))]
    fn new(
        cmd: Vec<String>,        // ⚠️
        cwd: Option<String>,     // ⚠️
        name: Option<String>,    // ⚠️
        output: Option<String>,
        _kwargs: Option<&pyo3::types::PyDict>,
    ) -> Self
```

**Note**: ExecuteProcess is currently a **placeholder** (not actually executed - we only capture launch descriptions). This is MEDIUM priority unless we plan to support process execution in the future.

**Tasks**:
- [ ] Change `cmd` to accept flexible types:
  ```rust
  pub struct ExecuteProcess {
      cmd: Vec<String>,        // Resolved command parts
      cwd: Option<String>,     // Resolved working directory
      name: Option<String>,    // Resolved process name
      output: String,
  }
  ```

- [ ] Update `new()` to accept and convert PyObject:
  ```rust
  #[new]
  #[pyo3(signature = (*, cmd, cwd=None, name=None, output=None, **_kwargs))]
  fn new(
      py: Python,
      cmd: Vec<PyObject>,           // Each element can be string or substitution
      cwd: Option<PyObject>,
      name: Option<PyObject>,
      output: Option<String>,
      _kwargs: Option<&pyo3::types::PyDict>,
  ) -> PyResult<Self> {
      // Convert cmd elements to strings
      let cmd_strs: Result<Vec<String>, _> = cmd
          .iter()
          .map(|obj| Self::pyobject_to_string(py, obj))
          .collect();
      let cmd_vec = cmd_strs?;

      let cwd_str = cwd
          .map(|obj| Self::pyobject_to_string(py, &obj))
          .transpose()?;

      let name_str = name
          .map(|obj| Self::pyobject_to_string(py, &obj))
          .transpose()?;

      log::debug!("Python Launch ExecuteProcess: {:?}", cmd_vec);

      Ok(Self {
          cmd: cmd_vec,
          cwd: cwd_str,
          name: name_str,
          output: output.unwrap_or_else(|| "screen".to_string()),
      })
  }
  ```

- [ ] Add tests (if we decide to prioritize this):
  - Mixed cmd: `ExecuteProcess(cmd=['ros2', 'run', LaunchConfiguration('pkg'), 'node'])`
  - Dynamic cwd: `ExecuteProcess(cmd=['ls'], cwd=[FindPackageShare('pkg'), '/dir'])`

**ROS 2 Reference**: `external/launch/launch/launch/actions/execute_process.py:128-138`

**Success Criteria**:
- ✅ cmd accepts list of PyObject
- ✅ Each cmd element can be string, substitution, or list
- ✅ cwd and name accept PyObject

**Decision Point**: Consider deferring this if ExecuteProcess isn't actively used. Mark as ⏸️ Deferred if so.

---

### Phase 15.4: Node.arguments (MEDIUM PRIORITY) ✅

**File**: `src/play_launch_parser/src/play_launch_parser/src/python/api/launch_ros.rs:71`

**Current Code**:
```rust
pub struct Node {
    package: String,
    executable: String,
    name: Option<String>,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<PyObject>,
    arguments: Vec<String>,  // ⚠️ Should accept Vec<PyObject>
    env_vars: Vec<(String, String)>,
    condition: Option<PyObject>,
}

#[pymethods]
impl Node {
    #[new]
    #[pyo3(signature = (
        *,
        package,
        executable,
        name=None,
        namespace=None,
        parameters=None,
        remappings=None,
        arguments=None,  // ⚠️ Currently Option<Vec<String>>
        env=None,
        condition=None,
        **_kwargs
    ))]
    fn new(
        py: Python,
        package: PyObject,
        executable: PyObject,
        name: Option<PyObject>,
        namespace: Option<PyObject>,
        parameters: Option<Vec<PyObject>>,
        remappings: Option<Vec<PyObject>>,
        arguments: Option<Vec<String>>,  // ⚠️
        env: Option<Vec<(String, String)>>,
        condition: Option<PyObject>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self>
```

**Tasks**:
- [ ] Change `arguments` parameter to accept PyObject:
  ```rust
  // In function signature:
  arguments: Option<Vec<PyObject>>,  // Accept dynamic values
  ```

- [ ] Convert arguments during construction:
  ```rust
  // In new() body:
  let arguments_vec = if let Some(args) = arguments {
      args.iter()
          .map(|obj| Self::pyobject_to_string(py, obj))
          .collect::<Result<Vec<String>, _>>()?
  } else {
      Vec::new()
  };

  // Later when creating Self:
  arguments: arguments_vec,
  ```

- [ ] Update tests to cover:
  - String arguments: `Node(arguments=['--param', 'value'])`
  - Substitution arguments: `Node(arguments=['--config', LaunchConfiguration('config_file')])`
  - Mixed: `Node(arguments=['--prefix', LaunchConfiguration('prefix'), '--suffix', 'value'])`

**ROS 2 Reference**: `external/launch_ros/launch_ros/launch_ros/actions/node.py:129`

**Success Criteria**:
- ✅ arguments accepts Vec<PyObject>
- ✅ Each argument can be string or substitution
- ✅ Arguments are resolved and stored as Vec<String>
- ✅ Captured nodes have correct argument arrays

---

### Phase 15.5: Shared Helper and Documentation ✅

**Tasks**:
- [x] Create shared `pyobject_to_string` helper if not already extracted
  - Location: **Created** `src/python/api/utils.rs` ✅
  - Made public: `pub fn pyobject_to_string(...)` ✅
  - Added to module tree: `mod.rs` ✅

- [x] Add documentation comment explaining the pattern:
  ```rust
  /// Convert a PyObject to String, handling ROS 2's SomeSubstitutionsType pattern.
  ///
  /// Accepts three forms:
  /// 1. Plain string: `"literal_value"`
  /// 2. Substitution: `LaunchConfiguration('var')` -> calls `__str__()`
  /// 3. List: `[LaunchConfiguration('prefix'), '/suffix']` -> concatenates
  ///
  /// This mirrors ROS 2's `SomeSubstitutionsType = Union[Text, Substitution, Iterable[...]]`
  pub(crate) fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
      // ... implementation ...
  }
  ```
  **Status**: ✅ Added comprehensive doc comments with examples, conversion strategy, and usage notes

- [x] Update CLAUDE.md with this pattern for future reference:
  ```markdown
  ### Python API Type Handling Pattern

  When implementing ROS 2 launch API classes, parameters that accept `SomeSubstitutionsType`
  in the Python API should accept `PyObject` in our PyO3 bindings:

  - **ROS 2**: `name: SomeSubstitutionsType`
  - **Our PyO3**: `name: PyObject` (in signature), converted to `String` (in struct)

  Always use `pyobject_to_string()` helper for conversion.
  ```
  **Status**: ✅ Added complete section with pattern explanation, code examples, and usage guidelines

- [x] Add to `tmp/python_api_type_audit.md` a "Fixed" section tracking completion
  **Status**: ✅ All 4 issues marked as FIXED with implementation dates and test status

**Success Criteria**:
- ✅ Helper function is DRY (Don't Repeat Yourself) - Created in `utils.rs`
- ✅ Documentation explains the pattern clearly - Comprehensive doc comments added
- ✅ CLAUDE.md updated for future development - Pattern section added
- ✅ Audit report updated - All issues marked fixed with summary
- ✅ Unit tests added - 3 tests for pyobject_to_string (all passing)

---

## Testing Strategy

### Unit Tests

For each fixed class, add tests in the appropriate test file:

**Location**: `src/play_launch_parser/tests/python_tests.rs`

```rust
#[test]
#[cfg(feature = "python")]
fn test_set_environment_variable_with_substitutions() {
    let _guard = python_test_guard();

    let launch_file = r#"
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='42'),
        SetEnvironmentVariable(
            name='ROS_DOMAIN_ID',
            value=LaunchConfiguration('domain_id')
        ),
    ])
"#;

    create_temp_launch_file("set_env_test.launch.py", launch_file);

    let mut parser = LaunchParser::new();
    let result = parser.parse_launch_file(&test_path, &args);

    assert!(result.is_ok(), "Parse should succeed with LaunchConfiguration in SetEnvironmentVariable");

    // Verify the environment variable would be set (if we supported execution)
    // For now, just verify no errors
}

#[test]
#[cfg(feature = "python")]
fn test_append_environment_variable_with_list() {
    let _guard = python_test_guard();

    let launch_file = r#"
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        AppendEnvironmentVariable(
            name='PYTHONPATH',
            value=[FindPackageShare('my_pkg'), '/scripts'],
            separator=':'
        ),
    ])
"#;

    create_temp_launch_file("append_env_test.launch.py", launch_file);

    let mut parser = LaunchParser::new();
    let result = parser.parse_launch_file(&test_path, &args);

    assert!(result.is_ok(), "Parse should succeed with list value in AppendEnvironmentVariable");
}

#[test]
#[cfg(feature = "python")]
fn test_node_arguments_with_substitutions() {
    let _guard = python_test_guard();

    let launch_file = r#"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='/tmp/config.yaml'),
        Node(
            package='my_pkg',
            executable='my_node',
            name='test_node',
            arguments=['--config', LaunchConfiguration('config_file'), '--verbose']
        ),
    ])
"#;

    create_temp_launch_file("node_args_test.launch.py", launch_file);

    let mut parser = LaunchParser::new();
    let result = parser.parse_launch_file(&test_path, &args);

    assert!(result.is_ok(), "Parse should succeed with LaunchConfiguration in Node arguments");

    // Verify captured node has arguments (if we capture them)
    let captures = get_captured_nodes();
    assert_eq!(captures.len(), 1);
    assert_eq!(captures[0].arguments.len(), 3);
    // Note: arguments won't be resolved without execution context
}
```

### Integration Tests

Use existing test workspaces:

1. **LCTK Demo** (already using complex Python):
   - Verify demo.launch.py still works
   - Check for any environment variable usage

2. **Autoware** (if available):
   - Run full Autoware validation
   - Check for any regressions

### Validation

- [ ] Run `just test` (all Rust unit tests)
- [ ] Run `just test-compare` (Rust vs Python parser comparison)
- [ ] Run LCTK demo: `cd ~/repos/LCTK && env DISPLAY=:2 just demo`
- [ ] Run Autoware test (if available): `just test-autoware`
- [ ] Check clippy: `just check`
- [ ] Verify all 260+ tests still pass

---

## Success Criteria

### Phase Complete When:

- ✅ All 4 classes fixed (SetEnvironmentVariable, AppendEnvironmentVariable, ExecuteProcess, Node.arguments)
- ✅ Shared `pyobject_to_string` helper extracted and documented
- ✅ All unit tests pass (260+ total)
- ✅ All integration tests pass (LCTK demo, Autoware if available)
- ✅ No clippy warnings
- ✅ CLAUDE.md updated with pattern documentation
- ✅ Audit report marked complete

### Quality Metrics:

- **Code Coverage**: All new code paths tested
- **Backward Compatibility**: Existing string-only usage still works
- **Performance**: No measurable regression (these are conversions during parsing, not hot path)
- **Documentation**: Pattern clearly documented for future use

---

## Risk Assessment

### Low Risk ✅

- **Pattern Proven**: Same approach as LogInfo/PythonExpression (already working)
- **Backward Compatible**: Strings still work (PyObject can extract String)
- **Well-Tested**: ROS 2 source code provides reference implementation
- **Isolated Changes**: Each class can be fixed independently

### Potential Issues

1. **Edge Cases in Conversion**:
   - Mitigation: Comprehensive tests covering all SomeSubstitutionsType forms

2. **Performance**:
   - Mitigation: Conversions happen during parsing (one-time cost)
   - Measured impact: Negligible (similar to current string operations)

3. **Unexpected Substitution Types**:
   - Mitigation: Fallback to `__str__()` and `to_string()` handles unknown types
   - Logging: Debug output shows what was converted

---

## Timeline

### Day 1: High Priority Fixes
- Morning: Phase 15.1 (SetEnvironmentVariable)
- Afternoon: Phase 15.2 (AppendEnvironmentVariable)
- Evening: Tests and validation

### Day 2: Medium Priority Fixes
- Morning: Phase 15.3 (ExecuteProcess) OR defer decision
- Afternoon: Phase 15.4 (Node.arguments)
- Evening: Integration tests

### Day 3: Finalization
- Morning: Phase 15.5 (Shared helper, documentation)
- Afternoon: Full validation suite
- Evening: Update roadmap, close phase

**Total: 2-3 days** (can be accelerated to 1-2 days if ExecuteProcess is deferred)

---

## References

### ROS 2 Source Code
- `external/launch/launch/launch/some_substitutions_type.py` - Type definition
- `external/launch/launch/launch/actions/set_environment_variable.py:34-43`
- `external/launch/launch/launch/actions/append_environment_variable.py:45-52`
- `external/launch/launch/launch/actions/execute_process.py:128-138`
- `external/launch_ros/launch_ros/launch_ros/actions/node.py:119-131`

### Our Implementation
- `src/play_launch_parser/src/play_launch_parser/src/python/api/actions.rs`
- `src/play_launch_parser/src/play_launch_parser/src/python/api/launch_ros.rs`
- `src/play_launch_parser/src/play_launch_parser/src/python/api/substitutions.rs`

### Audit Report
- `tmp/python_api_type_audit.md` - Comprehensive analysis

### Related Fixes
- Phase 14: Python Execution (enabled dynamic value resolution)
- LogInfo fix (2026-01-31): First instance of this pattern
- PythonExpression fix (2026-01-31): Second instance, established pattern

---

## Future Considerations

### After Phase 15

1. **Audit Other Actions**: Check remaining action classes for similar issues
   - `TimerAction.period` (accepts `Union[float, SomeSubstitutionsType]`)
   - `GroupAction.launch_configurations` (dict with substitution keys/values)

2. **Type Annotations**: Consider adding PyO3 type hints for better IDE support

3. **Performance Optimization**: Profile conversion overhead (likely negligible)

4. **Error Messages**: Improve error messages when conversion fails
   - Show which parameter failed
   - Show the Python type that was provided

---

## Appendix: Complete Fix Template

Use this template for each class:

```rust
// 1. Update struct (no changes to fields - keep as resolved types)
pub struct ActionName {
    field: String,  // Resolved value
}

// 2. Update constructor signature
#[pymethods]
impl ActionName {
    #[new]
    fn new(
        py: Python,
        field: PyObject,  // Accept PyObject instead of String
        // ... other parameters
    ) -> PyResult<Self> {
        // 3. Convert PyObject to String
        let field_str = Self::pyobject_to_string(py, &field)?;

        // 4. Rest of logic
        log::debug!("ActionName: field={}", field_str);

        Ok(Self {
            field: field_str,
        })
    }

    // 5. Add helper (or use shared one)
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        // ... standard implementation ...
    }
}
```

---

**Status Legend**:
- ✅ Complete
- ⏳ In Progress
- 📋 Planned
- ⏸️ Deferred
- ❌ Blocked
