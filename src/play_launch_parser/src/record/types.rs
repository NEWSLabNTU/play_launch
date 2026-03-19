//! record.json data structures

use serde::{Deserialize, Serialize};
use std::{collections::HashMap, path::Path};

/// A scope in the launch include tree.
/// Each scope corresponds to one launch file invocation.
/// Shared context (namespace, env, global params) is stored once here,
/// not duplicated per node.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScopeEntry {
    pub id: usize,
    /// ROS package name (extracted from ament install path, may be None)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pkg: Option<String>,
    /// Launch file name (basename, e.g. "sensing.launch.xml")
    pub file: String,
    /// Accumulated ROS namespace at this scope level
    pub ns: String,
    /// Launch arguments passed to this scope
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub args: HashMap<String, String>,
    /// Parent scope ID (None for root)
    pub parent: Option<usize>,
}

/// Table of all scopes encountered during parsing.
/// The root scope has id=0 and parent=None.
/// Parent references form the launch include tree.
#[derive(Debug, Clone, Default)]
pub struct ScopeTable {
    entries: Vec<ScopeEntry>,
}

impl ScopeTable {
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    /// Push a new scope, returning its ID.
    pub fn push(
        &mut self,
        pkg: Option<String>,
        file: String,
        ns: String,
        args: HashMap<String, String>,
        parent: Option<usize>,
    ) -> usize {
        let id = self.entries.len();
        self.entries.push(ScopeEntry {
            id,
            pkg,
            file,
            ns,
            args,
            parent,
        });
        id
    }

    pub fn get(&self, id: usize) -> Option<&ScopeEntry> {
        self.entries.get(id)
    }

    pub fn entries(&self) -> &[ScopeEntry] {
        &self.entries
    }

    pub fn into_entries(self) -> Vec<ScopeEntry> {
        self.entries
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// Extract ROS package name from an ament install path.
/// Looks for `/share/<pkg>/` in the path.
pub fn extract_package_from_path(path: &Path) -> Option<String> {
    let path_str = path.to_str()?;
    const SHARE_PREFIX: &str = "/share/";
    if let Some(share_pos) = path_str.find(SHARE_PREFIX) {
        let after_share = &path_str[share_pos + SHARE_PREFIX.len()..];
        // Package name is the next path component
        if let Some(slash_pos) = after_share.find('/') {
            let pkg = &after_share[..slash_pos];
            if !pkg.is_empty() {
                return Some(pkg.to_string());
            }
        }
    }
    None
}

/// Root structure for record.json
/// Fields ordered to match Python output
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecordJson {
    pub container: Vec<ComposableNodeContainerRecord>,
    pub file_data: HashMap<String, String>,
    pub lifecycle_node: Vec<String>,
    pub load_node: Vec<LoadNodeRecord>,
    pub node: Vec<NodeRecord>,
    /// Launch configuration variables (e.g., from DeclareLaunchArgument or CLI args)
    /// Maps variable names to their resolved values
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub variables: HashMap<String, String>,
    /// Launch tree scope table — each entry is one launch file invocation.
    /// Parent references form the include tree.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub scopes: Vec<ScopeEntry>,
}

impl RecordJson {
    pub fn new() -> Self {
        Self {
            container: Vec::new(),
            file_data: HashMap::new(),
            lifecycle_node: Vec::new(),
            load_node: Vec::new(),
            node: Vec::new(),
            variables: HashMap::new(),
            scopes: Vec::new(),
        }
    }

    pub fn to_json(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(self)
    }
}

impl Default for RecordJson {
    fn default() -> Self {
        Self::new()
    }
}

/// Node record structure
/// Fields ordered alphabetically to match Python output
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeRecord {
    pub args: Option<Vec<String>>,
    pub cmd: Vec<String>,
    pub env: Option<Vec<(String, String)>>,
    pub exec_name: Option<String>,
    pub executable: String,
    pub global_params: Option<Vec<(String, String)>>,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub package: Option<String>,
    pub params: Vec<(String, String)>,
    pub params_files: Vec<String>,
    pub remaps: Vec<(String, String)>,
    pub respawn: Option<bool>,
    pub respawn_delay: Option<f64>,
    pub ros_args: Option<Vec<String>>,
    /// Scope ID referencing the scopes table (launch file origin)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scope: Option<usize>,
}

/// Composable node container record
/// Contains all information needed to spawn the container process
#[derive(Debug, Clone, Serialize, Deserialize)]
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
    /// Scope ID referencing the scopes table (launch file origin)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scope: Option<usize>,
}

/// Load node record (for composable nodes)
/// Fields ordered alphabetically to match Python output
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadNodeRecord {
    pub env: Option<Vec<(String, String)>>,
    pub extra_args: HashMap<String, String>,
    pub log_level: Option<String>,
    pub namespace: String,
    pub node_name: String,
    pub package: String,
    pub params: Vec<(String, String)>,
    pub plugin: String,
    pub remaps: Vec<(String, String)>,
    pub target_container_name: String,
    /// Scope ID referencing the scopes table (launch file origin)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scope: Option<usize>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_record() {
        let record = RecordJson::new();
        assert_eq!(record.node.len(), 0);
        assert_eq!(record.container.len(), 0);
        assert_eq!(record.load_node.len(), 0);
        assert_eq!(record.lifecycle_node.len(), 0);
        assert_eq!(record.file_data.len(), 0);
        assert_eq!(record.variables.len(), 0);
    }

    #[test]
    fn test_serialize_empty() {
        let record = RecordJson::new();
        let json = record.to_json().unwrap();
        assert!(json.contains("\"node\""));
        assert!(json.contains("\"container\""));
        assert!(json.contains("\"load_node\""));
        assert!(json.contains("\"lifecycle_node\""));
        assert!(json.contains("\"file_data\""));
        // Note: "variables" is skipped when empty (matches Python behavior)
    }

    #[test]
    fn test_serialize_node_record() {
        let node = NodeRecord {
            args: None,
            cmd: vec![
                "/path/to/talker".to_string(),
                "--ros-args".to_string(),
                "-r".to_string(),
                "__node:=talker".to_string(),
            ],
            env: None,
            exec_name: Some("talker-1".to_string()),
            executable: "talker".to_string(),
            global_params: None,
            name: Some("/talker".to_string()),
            namespace: Some("/".to_string()),
            package: Some("demo_nodes_cpp".to_string()),
            params: vec![("rate".to_string(), "10.0".to_string())],
            params_files: vec![],
            remaps: vec![("chatter".to_string(), "/chat".to_string())],
            respawn: Some(false),
            respawn_delay: None,
            ros_args: None,
            scope: None,
        };

        let json = serde_json::to_string(&node).unwrap();
        assert!(json.contains("\"executable\":\"talker\""));
        assert!(json.contains("\"package\":\"demo_nodes_cpp\""));
    }

    #[test]
    fn test_tuple_serialization() {
        let node = NodeRecord {
            args: None,
            cmd: vec![],
            env: None,
            exec_name: None,
            executable: "node".to_string(),
            global_params: None,
            name: None,
            namespace: None,
            package: None,
            params: vec![
                ("param1".to_string(), "value1".to_string()),
                ("param2".to_string(), "value2".to_string()),
            ],
            params_files: vec![],
            remaps: vec![],
            respawn: None,
            respawn_delay: None,
            ros_args: None,
            scope: None,
        };

        let json = serde_json::to_string(&node).unwrap();
        // Tuples should serialize as arrays
        assert!(json.contains("[\"param1\",\"value1\"]"));
        assert!(json.contains("[\"param2\",\"value2\"]"));
    }
}
