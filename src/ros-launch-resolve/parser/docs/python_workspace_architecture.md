# Python Support Workspace Architecture

**Version**: 1.0
**Date**: 2026-01-18
**Status**: Design Phase

---

## Overview

Architecture for integrating Python launch file support using **pyo3** to embed a Python interpreter and mock the ROS 2 `launch` API.

### Core Principle

**Capture on Construction**: When Python code creates a `Node`, our mock API intercepts and captures it immediately, without needing to understand Python syntax.

---

## Workspace Structure

```
play_launch_parser/
├── Cargo.toml                    # Workspace root
├── src/
│   └── play_launch_parser/       # Main parser crate
│       ├── Cargo.toml
│       ├── src/
│       │   ├── lib.rs            # XML parser (existing)
│       │   ├── main.rs           # CLI (existing)
│       │   ├── python/           # Python support module
│       │   │   ├── mod.rs        # Module root
│       │   │   ├── executor.rs   # Python execution engine
│       │   │   ├── api/          # Mock Python API
│       │   │   │   ├── mod.rs
│       │   │   │   ├── launch.rs           # launch module
│       │   │   │   ├── launch_ros.rs       # launch_ros module
│       │   │   │   ├── substitutions.rs    # Substitution classes
│       │   │   │   └── actions.rs          # Action classes
│       │   │   └── bridge.rs     # Rust ↔ Python data conversion
│       │   └── ...
│       └── tests/
│           └── python_tests.rs   # Python integration tests
└── docs/
    └── python_workspace_architecture.md
```

---

## Cargo Workspace Configuration

### Root `Cargo.toml` (Workspace)

```toml
[workspace]
members = ["src/play_launch_parser"]
resolver = "2"

[workspace.dependencies]
pyo3 = { version = "0.20", features = ["auto-initialize"] }
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
```

### Parser `Cargo.toml`

```toml
[package]
name = "play_launch_parser"
version = "0.1.0"
edition = "2021"

[dependencies]
# Existing dependencies
roxmltree = "0.19"
serde = { workspace = true }
serde_json = { workspace = true }
# ... other deps

# Python support (optional feature)
pyo3 = { workspace = true, optional = true }

[features]
default = []
python = ["pyo3"]

[build-dependencies]
# Optional: pyo3-build-config for build-time Python detection
```

---

## Module Architecture

### 1. Python Executor (`python/executor.rs`)

**Responsibility**: Manage Python interpreter lifecycle and execute launch files.

```rust
pub struct PythonLaunchExecutor {
    // Manages Python interpreter
}

impl PythonLaunchExecutor {
    pub fn new() -> PyResult<Self>;

    pub fn execute_launch_file(
        &self,
        path: &Path,
        args: &HashMap<String, String>
    ) -> Result<Vec<NodeRecord>>;
}
```

**Key Operations**:
1. Initialize Python interpreter (once, cached)
2. Register mock modules
3. Execute Python file
4. Extract captured nodes
5. Convert to `NodeRecord`

---

### 2. Mock Python API (`python/api/`)

**Responsibility**: Provide Python-compatible API that captures node definitions.

#### `launch.rs` - Core Launch Module

```rust
#[pyclass]
pub struct LaunchDescription {
    actions: Vec<PyObject>,
}

#[pymethods]
impl LaunchDescription {
    #[new]
    fn new(actions: Vec<PyObject>) -> Self;
}
```

#### `launch_ros.rs` - ROS Actions

```rust
#[pyclass]
pub struct Node {
    package: String,
    executable: String,
    name: Option<String>,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<(String, String)>,
    arguments: Vec<String>,
    env_vars: Vec<(String, String)>,
}

#[pymethods]
impl Node {
    #[new]
    fn new(/* ... */) -> Self {
        let node = Self { /* ... */ };
        // Capture immediately
        CAPTURED_NODES.lock().unwrap().push(
            NodeCapture::from_node(&node)
        );
        node
    }
}

#[pyclass]
pub struct ComposableNodeContainer { /* ... */ }

#[pyclass]
pub struct ComposableNode { /* ... */ }
```

#### `substitutions.rs` - Launch Substitutions

```rust
#[pyclass]
pub struct LaunchConfiguration {
    variable_name: String,
}

#[pyclass]
pub struct PathJoinSubstitution {
    paths: Vec<PyObject>,
}

#[pyclass]
pub struct FindPackageShare {
    package_name: String,
}
```

#### `actions.rs` - Launch Actions

```rust
#[pyclass]
pub struct DeclareLaunchArgument {
    name: String,
    default_value: Option<String>,
    description: Option<String>,
}

#[pyclass]
pub struct OpaqueFunction {
    function: PyObject,
}
```

---

### 3. Data Bridge (`python/bridge.rs`)

**Responsibility**: Convert between Python and Rust types.

```rust
pub struct NodeCapture {
    pub package: String,
    pub executable: String,
    pub name: Option<String>,
    // ...
}

impl NodeCapture {
    pub fn from_node(node: &Node) -> Self;

    pub fn to_record(
        &self,
        context: &LaunchContext
    ) -> Result<NodeRecord>;
}
```

**Handles**:
- Python objects → Rust structs
- Substitution resolution
- Type conversions
- Error mapping

---

### 4. Global Capture Storage

```rust
use std::sync::{Arc, Mutex};
use once_cell::sync::Lazy;

pub static CAPTURED_NODES: Lazy<Arc<Mutex<Vec<NodeCapture>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));

pub static CAPTURED_ARGUMENTS: Lazy<Arc<Mutex<Vec<ArgumentCapture>>>> =
    Lazy::new(|| Arc::new(Mutex::new(Vec::new())));
```

**Thread Safety**: Use `Mutex` for interior mutability in static context.

---

## Execution Flow

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. XML Parser encounters <include file="foo.launch.py">        │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. Check file extension → .py → PythonLaunchExecutor           │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. PythonLaunchExecutor::execute_launch_file()                 │
│    - Clear CAPTURED_NODES                                      │
│    - Read Python file                                          │
│    - Create execution context with launch args                 │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. Python::with_gil(|py| {                                     │
│       py.run(&code, None, Some(locals))?;                      │
│    })                                                           │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. Python imports: from launch_ros.actions import Node         │
│    → Loads our mock Node class                                 │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ 6. Python calls: Node(package='foo', executable='bar')         │
│    → Node::new() called                                        │
│    → Captures to CAPTURED_NODES                                │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ 7. Python execution completes                                  │
│    → Extract CAPTURED_NODES.lock().unwrap().clone()            │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ 8. Convert NodeCapture → NodeRecord                            │
│    → Resolve substitutions with LaunchContext                  │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ 9. Return Vec<NodeRecord> to main parser                       │
│    → Add to unified node collection                            │
└─────────────────────────────────────────────────────────────────┘
```

---

## Integration Points

### 1. Main Parser (`lib.rs`)

```rust
impl LaunchTraverser {
    fn process_include(&mut self, include: &IncludeAction) -> Result<()> {
        let file_path = resolve_path(&include.file, &self.context)?;

        match file_path.extension().and_then(|s| s.to_str()) {
            #[cfg(feature = "python")]
            Some("py") => self.process_python_file(&file_path),

            Some("yaml") | Some("yml") => Ok(()), // Skip config files

            _ => self.process_xml_file(&file_path),
        }
    }

    #[cfg(feature = "python")]
    fn process_python_file(&mut self, path: &Path) -> Result<()> {
        let executor = python::PythonLaunchExecutor::new()?;
        let nodes = executor.execute_launch_file(
            path,
            &self.context.configurations()
        )?;
        self.records.extend(nodes);
        Ok(())
    }
}
```

### 2. Feature Flags

```rust
// In lib.rs
#[cfg(feature = "python")]
pub mod python;

#[cfg(not(feature = "python"))]
impl LaunchTraverser {
    fn process_python_file(&mut self, path: &Path) -> Result<()> {
        log::warn!("Python support not enabled, skipping: {}", path.display());
        Ok(())
    }
}
```

---

## Module Registration

```rust
// python/api/mod.rs

pub fn register_modules(py: Python) -> PyResult<()> {
    // Create modules
    let launch = PyModule::new(py, "launch")?;
    let launch_actions = PyModule::new(py, "launch.actions")?;
    let launch_subs = PyModule::new(py, "launch.substitutions")?;
    let launch_ros = PyModule::new(py, "launch_ros")?;
    let launch_ros_actions = PyModule::new(py, "launch_ros.actions")?;
    let launch_ros_desc = PyModule::new(py, "launch_ros.descriptions")?;

    // Register classes
    launch.add_class::<LaunchDescription>()?;
    launch_actions.add_class::<DeclareLaunchArgument>()?;
    launch_subs.add_class::<LaunchConfiguration>()?;
    launch_ros_actions.add_class::<Node>()?;
    launch_ros_actions.add_class::<ComposableNodeContainer>()?;
    launch_ros_desc.add_class::<ComposableNode>()?;

    // Install in sys.modules
    let sys = py.import("sys")?;
    let modules = sys.getattr("modules")?;
    modules.set_item("launch", launch)?;
    modules.set_item("launch.actions", launch_actions)?;
    modules.set_item("launch.substitutions", launch_subs)?;
    modules.set_item("launch_ros", launch_ros)?;
    modules.set_item("launch_ros.actions", launch_ros_actions)?;
    modules.set_item("launch_ros.descriptions", launch_ros_desc)?;

    Ok(())
}
```

---

## Error Handling

### Error Types

```rust
#[derive(Error, Debug)]
pub enum PythonError {
    #[error("Python initialization failed: {0}")]
    InitializationError(String),

    #[error("Python execution failed: {0}")]
    ExecutionError(String),

    #[error("Missing generate_launch_description function")]
    MissingEntryPoint,

    #[error("Node capture failed: {0}")]
    CaptureError(String),

    #[error("Type conversion failed: {0}")]
    ConversionError(String),
}
```

### Error Propagation

```rust
impl From<PyErr> for PythonError {
    fn from(err: PyErr) -> Self {
        PythonError::ExecutionError(err.to_string())
    }
}

impl From<PythonError> for ParseError {
    fn from(err: PythonError) -> Self {
        ParseError::PythonError(err.to_string())
    }
}
```

---

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    #[test]
    fn test_simple_node_capture() {
        let code = r#"
from launch_ros.actions import Node

node = Node(package='demo', executable='talker')
"#;
        // Execute and verify capture
    }
}
```

### Integration Tests

```rust
// tests/python_tests.rs
#[test]
#[cfg(feature = "python")]
fn test_python_launch_file() {
    let path = Path::new("tests/fixtures/simple.launch.py");
    // ...
}
```

### Fixture Files

```
tests/
└── fixtures/
    ├── python/
    │   ├── simple.launch.py           # Basic node
    │   ├── with_params.launch.py      # Parameters
    │   ├── with_substitutions.launch.py
    │   └── container.launch.py        # Composable nodes
    └── ...
```

---

## Build Configuration

### Conditional Compilation

All Python code wrapped in `#[cfg(feature = "python")]`:

```rust
#[cfg(feature = "python")]
pub mod python;

#[cfg(feature = "python")]
impl LaunchTraverser {
    // Python-specific methods
}
```

### Build Commands

```bash
# Without Python support (default)
cargo build

# With Python support
cargo build --features python

# Tests with Python
cargo test --features python
```

---

## Dependencies

### Required Crates

- **pyo3** (0.20): Python bindings
- **once_cell**: Lazy static initialization
- **parking_lot** (optional): Better mutex performance

### Python Requirements

- **Python 3.8+**: Minimum supported version
- **No external packages**: Pure stdlib mocking

---

## Performance Considerations

### Interpreter Caching

```rust
use once_cell::sync::OnceCell;

static INTERPRETER: OnceCell<Python<'static>> = OnceCell::new();

pub fn get_interpreter() -> Python<'static> {
    INTERPRETER.get_or_init(|| {
        pyo3::prepare_freethreaded_python();
        // Return interpreter
    })
}
```

### Capture Storage

- **Thread-local**: For parallel execution
- **Mutex-guarded**: For thread safety
- **Clear before each file**: Avoid memory leaks

---

## Limitations & Trade-offs

### Supported

✅ Node creation
✅ Launch arguments
✅ Basic substitutions
✅ Composable nodes
✅ Conditional logic (Python-native)
✅ Loops (Python-native)

### Not Supported (Initially)

❌ `OpaqueFunction` (complex callbacks)
❌ Event handlers (`OnProcessStart`, etc.)
❌ Custom Python dependencies
❌ Dynamic imports from installed packages

### Future Extensions

- Full `OpaqueFunction` support
- Event system mocking
- Launch service integration
- Context manager support

---

## Migration Path

### Phase 1: Core Infrastructure
- ✅ Workspace structure
- ✅ Feature flags
- ✅ Basic executor

### Phase 2: Essential API
- Node, LaunchDescription
- LaunchConfiguration
- DeclareLaunchArgument

### Phase 3: Advanced Features
- ComposableNodeContainer
- Substitutions (PathJoin, FindPackage)
- OpaqueFunction (basic)

### Phase 4: Polish
- Error handling
- Performance optimization
- Documentation

---

## Security Considerations

### Sandboxing

⚠️ Python code runs **unsandboxed** in embedded interpreter.

**Mitigation**:
- Only parse trusted launch files
- Same trust model as XML (can execute arbitrary commands)
- Consider `restrictedpython` for untrusted sources

### Resource Limits

- No timeout by default
- Consider adding execution timeout
- Memory limits via Python's `resource` module

---

## Summary

**Architecture Type**: Embedded interpreter with mock API
**Integration**: Feature-flagged module in main crate
**Data Flow**: Python → Capture → Bridge → NodeRecord
**Thread Safety**: Mutex-guarded global storage
**Performance**: Single interpreter, cached between files

This architecture provides a clean separation between XML and Python parsing while sharing the same output format.
