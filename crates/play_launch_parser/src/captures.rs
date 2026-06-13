//! Capture types for parsed launch entities
//!
//! These types are used by both the XML parser (via LaunchContext) and the Python
//! API (via thread-local context) to store parsed nodes, containers, composable
//! node loads, and includes during launch file traversal.
//!
//! The `to_record()` implementations that convert captures to final record types
//! are defined in `python/bridge.rs` (they depend on global parameter state).

/// Captured node data from Python or XML parsing
#[derive(Debug, Clone)]
pub struct NodeCapture {
    pub package: String,
    pub executable: String,
    pub name: Option<String>,
    pub namespace: Option<String>,
    /// `<node machine="…">` target host (ROS 2 multi-host launch). `None` when
    /// the launch doesn't route the node to a specific machine.
    pub machine: Option<String>,
    pub parameters: Vec<(String, String)>,
    pub params_files: Vec<String>,
    pub remappings: Vec<(String, String)>,
    pub arguments: Vec<String>,
    pub env_vars: Vec<(String, String)>,
    /// Scope ID from the launch tree (set by traverser after capture)
    pub scope_id: Option<usize>,
}

/// Captured container data from Python or XML parsing
#[derive(Debug, Clone)]
pub struct ContainerCapture {
    pub name: String,
    pub namespace: String,
    pub package: Option<String>,
    pub executable: Option<String>,
    pub cmd: Vec<String>,
    /// Scope ID from the launch tree (set by traverser after capture)
    pub scope_id: Option<usize>,
}

/// Captured composable node data from Python or XML parsing
#[derive(Debug, Clone)]
pub struct LoadNodeCapture {
    pub package: String,
    pub plugin: String,
    pub target_container_name: String,
    pub node_name: String,
    pub namespace: String,
    pub parameters: Vec<(String, String)>,
    pub remappings: Vec<(String, String)>,
    /// Scope ID from the launch tree (set by traverser after capture)
    pub scope_id: Option<usize>,
}

/// Captured include data from Python or XML parsing
#[derive(Debug, Clone)]
pub struct IncludeCapture {
    pub file_path: String,
    pub args: Vec<(String, String)>,
    pub ros_namespace: String,
}
