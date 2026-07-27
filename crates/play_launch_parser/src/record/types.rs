//! record.json data structures

use serde::{Deserialize, Serialize};
use std::{collections::HashMap, path::Path};

/// Origin of a scope — identifies the launch file.
/// None for group scopes (anonymous).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScopeOrigin {
    /// ROS package name (extracted from ament install path, may be None)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pkg: Option<String>,
    /// Launch file name (basename, e.g. "sensing.launch.xml")
    pub file: String,
    /// Resolved absolute path of the launch file (new in Phase 40; needed for
    /// provider-sidecar contract lookup). None in records produced by older
    /// parsers.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
}

/// Resolve `path` to a canonicalized absolute path string.
///
/// Uses `fs::canonicalize` when possible (resolves symlinks, `.`/`..`); falls
/// back to an absolutized (but not canonicalized) path if canonicalization
/// fails (e.g. the file doesn't exist on disk, as in some test fixtures).
/// Never panics.
pub fn canonicalize_path(path: &Path) -> String {
    match std::fs::canonicalize(path) {
        Ok(canonical) => canonical.to_string_lossy().into_owned(),
        Err(_) => {
            let absolute = if path.is_absolute() {
                path.to_path_buf()
            } else {
                std::env::current_dir()
                    .map(|cwd| cwd.join(path))
                    .unwrap_or_else(|_| path.to_path_buf())
            };
            // Lexically collapse `.`/`..` so the fallback matches Python's
            // os.path.realpath, which normalizes dot-segments even for paths
            // it cannot stat. Without this, a nonexistent include containing
            // `..` would break cross-parser path parity.
            lexical_normalize(&absolute).to_string_lossy().into_owned()
        }
    }
}

/// Lexical `.`/`..` collapse (no filesystem access). Mirrors the dot-segment
/// handling of `os.path.normpath`/`realpath` for nonexistent paths.
fn lexical_normalize(path: &Path) -> std::path::PathBuf {
    use std::path::Component;
    let mut out = std::path::PathBuf::new();
    for comp in path.components() {
        match comp {
            Component::CurDir => {}
            Component::ParentDir => {
                // Pop a normal component if present; at the root, `..` is a
                // no-op (same as normpath on absolute paths).
                if !out.pop() {
                    out.push(Component::RootDir);
                }
            }
            other => out.push(other.as_os_str()),
        }
    }
    out
}

/// A scope in the launch tree.
/// File scopes have `origin: Some(...)` — one per launch file invocation.
/// Group scopes have `origin: None` — one per `<group>` that modifies
/// namespace, env, or global params.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScopeEntry {
    pub id: usize,
    /// None for group scopes, Some for file scopes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<ScopeOrigin>,
    /// Accumulated ROS namespace at this scope level
    pub ns: String,
    /// Launch arguments passed to this scope (file scopes only)
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub args: HashMap<String, String>,
    /// Parent scope ID (None for root)
    pub parent: Option<usize>,
}

impl ScopeEntry {
    /// Whether this is a file scope (has origin).
    pub fn is_file_scope(&self) -> bool {
        self.origin.is_some()
    }

    /// Package name (if file scope with known package).
    pub fn pkg(&self) -> Option<&str> {
        self.origin.as_ref().and_then(|o| o.pkg.as_deref())
    }

    /// File name (if file scope).
    pub fn file(&self) -> Option<&str> {
        self.origin.as_ref().map(|o| o.file.as_str())
    }

    /// Canonicalized absolute path of the launch file (if file scope with a
    /// known path — always populated by current parsers, may be `None` when
    /// reading older records).
    pub fn path(&self) -> Option<&str> {
        self.origin.as_ref().and_then(|o| o.path.as_deref())
    }
}

/// Table of all scopes encountered during parsing.
/// The root scope has id=0 and parent=None.
/// Parent references form the launch tree (files + groups).
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

    /// Push a file scope, returning its ID.
    ///
    /// `path` should be the canonicalized absolute path of the launch file
    /// (see [`canonicalize_path`]).
    #[allow(clippy::too_many_arguments)]
    pub fn push(
        &mut self,
        pkg: Option<String>,
        file: String,
        path: String,
        ns: String,
        args: HashMap<String, String>,
        parent: Option<usize>,
    ) -> usize {
        let id = self.entries.len();
        self.entries.push(ScopeEntry {
            id,
            origin: Some(ScopeOrigin {
                pkg,
                file,
                path: Some(path),
            }),
            ns,
            args,
            parent,
        });
        id
    }

    /// Push a group scope (anonymous, no file), returning its ID.
    pub fn push_group(&mut self, ns: String, parent: Option<usize>) -> usize {
        let id = self.entries.len();
        self.entries.push(ScopeEntry {
            id,
            origin: None,
            ns,
            args: HashMap::new(),
            parent,
        });
        id
    }

    /// Update the args of an existing scope entry.
    /// Called after traversal to include args resolved from defaults.
    pub fn update_args(&mut self, id: usize, args: HashMap<String, String>) {
        if let Some(entry) = self.entries.get_mut(id) {
            entry.args = args;
        }
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
/// phase-54 (issue 0007) — one parameter source in ROS's ordered model.
///
/// `launch_ros` keeps ONE list per node: `parse_nested_parameters` appends an
/// entry per `<param>` child in document order, and `execute` emits
/// `--params-file` / `-p` in that order (materializing an inline dict into a
/// temp file first). Kind carries NO precedence — position does. Serialized
/// as an externally-tagged enum so a record stays human-readable.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ParamSource {
    /// An inline `<param name= value=/>` (or a global param, which ROS applies
    /// before the node's own — those occupy the head of the list).
    Inline { name: String, value: String },
    /// A `<param from=/>` parameter file — the RESOLVED YAML CONTENT, matching
    /// `params_files` (the spawn path materializes it to a temp file and
    /// passes `--params-file`). A launch-created temp params file is expanded
    /// into `Inline` entries instead, mirroring what the Python dumper does.
    File { content: String },
}

/// Fields ordered alphabetically to match Python output
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeRecord {
    pub args: Option<Vec<String>>,
    pub cmd: Vec<String>,
    pub env: Option<Vec<(String, String)>>,
    pub exec_name: Option<String>,
    pub executable: String,
    pub global_params: Option<Vec<(String, String)>>,
    /// `<node machine="…">` — the target host the launch routes this node to
    /// (ROS 2 multi-host launch). Additive: omitted when absent so existing
    /// records round-trip byte-identical. Alphabetical slot (after
    /// `global_params`, before `name`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub machine: Option<String>,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub package: Option<String>,
    pub params: Vec<(String, String)>,
    pub params_files: Vec<String>,
    /// phase-54 (issue 0007) — the ORDERED parameter-source list; `params` /
    /// `params_files` above are the legacy split views kept for back-compat.
    /// Additive: omitted when empty so existing records round-trip
    /// byte-identical. Global params occupy the head (ROS applies them before
    /// the node's own).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_sources: Vec<ParamSource>,
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
    /// phase-54 (issue 0007) — the ORDERED parameter-source list; `params` /
    /// `params_files` above are the legacy split views kept for back-compat.
    /// Additive: omitted when empty so existing records round-trip
    /// byte-identical. Global params occupy the head (ROS applies them before
    /// the node's own).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_sources: Vec<ParamSource>,
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

    /// The fallback for nonexistent paths must collapse `.`/`..` exactly like
    /// Python's os.path.realpath does for paths it cannot stat — otherwise the
    /// two parsers emit different `origin.path` strings for the same include.
    #[test]
    fn test_canonicalize_fallback_normalizes_dot_segments() {
        let p = Path::new("/nonexistent_dir_for_test/a/b/../c/./x.launch.xml");
        assert_eq!(
            canonicalize_path(p),
            "/nonexistent_dir_for_test/a/c/x.launch.xml"
        );
        // `..` at the root is a no-op, as in normpath.
        let q = Path::new("/nonexistent_dir_for_test/../../y.launch.xml");
        assert_eq!(canonicalize_path(q), "/y.launch.xml");
    }

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
            machine: None,
            name: Some("/talker".to_string()),
            namespace: Some("/".to_string()),
            package: Some("demo_nodes_cpp".to_string()),
            params: vec![("rate".to_string(), "10.0".to_string())],
            params_files: vec![],
            param_sources: Vec::new(),
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
            machine: None,
            name: None,
            namespace: None,
            package: None,
            params: vec![
                ("param1".to_string(), "value1".to_string()),
                ("param2".to_string(), "value2".to_string()),
            ],
            params_files: vec![],
            param_sources: Vec::new(),
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
