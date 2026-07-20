use serde::{Deserialize, Serialize};
use std::{
    collections::HashMap,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

pub type ParameterValue = String;

/// Origin of a scope — identifies the launch file.
/// None for group scopes (anonymous).
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ScopeOrigin {
    /// ROS package name (extracted from ament install path, may be None)
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pkg: Option<String>,
    /// Launch file name (basename)
    pub file: String,
    /// Resolved absolute path of the launch file (new in Phase 40; needed for
    /// provider-sidecar contract lookup). `None` in records produced by older
    /// parsers.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
}

/// A scope in the launch tree (from parser Phase 30).
/// File scopes have `origin: Some(...)`, group scopes have `origin: None`.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ScopeEntry {
    pub id: usize,
    /// None for group scopes, Some for file scopes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<ScopeOrigin>,
    pub ns: String,
    #[serde(default)]
    pub args: HashMap<String, String>,
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
    /// known path). Drives the Phase 40 provider-sidecar contract + platform-
    /// file lookup, and (Phase 46.5) the stale-Python-install guard
    /// [`ensure_python_scope_paths`] — a file scope missing this is the
    /// signal of a pre-Phase-40.1 parser.
    pub fn path(&self) -> Option<&str> {
        self.origin.as_ref().and_then(|o| o.path.as_deref())
    }
}

/// The serialization format for a recorded launch.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct LaunchDump {
    pub node: Vec<NodeRecord>,
    pub load_node: Vec<ComposableNodeRecord>,
    pub container: Vec<NodeContainerRecord>,
    /// Lifecycle node names tracked for future implementation.
    /// Currently lifecycle nodes require manual intervention and are not automatically handled.
    #[allow(dead_code)]
    pub lifecycle_node: Vec<String>,
    /// File data cache for parameter files (used by removed print_shell functionality)
    #[allow(dead_code)]
    pub file_data: HashMap<PathBuf, String>,
    /// Launch configuration variables (e.g., from DeclareLaunchArgument or CLI args)
    /// Maps variable names to their resolved values
    /// Used to substitute $(var name) patterns in command arguments during replay
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub variables: HashMap<String, String>,
    /// Launch tree scope table — each entry is one launch file invocation.
    /// Present when the parser is run with scope tracking enabled.
    #[serde(default)]
    pub scopes: Vec<ScopeEntry>,
}

impl LaunchDump {
    /// An empty dump — used by `replay --model` (Phase 47.B3: there is no
    /// record.json companion to read anymore), whose best-effort/
    /// informational consumers (chain-colocation warnings, the web UI's
    /// launch-tree scope map) degrade to empty rather than erroring. `launch`
    /// (Phase 47.B4) instead passes the REAL in-memory dump it just parsed,
    /// so those consumers stay populated on that path.
    pub fn empty() -> Self {
        Self {
            node: Vec::new(),
            load_node: Vec::new(),
            container: Vec::new(),
            lifecycle_node: Vec::new(),
            file_data: HashMap::new(),
            variables: HashMap::new(),
            scopes: Vec::new(),
        }
    }
}

/// The serialization format for a node container record.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct NodeContainerRecord {
    pub executable: String,
    pub package: String,
    pub name: String,
    pub namespace: String,
    pub exec_name: Option<String>,
    #[serde(default)]
    pub params: Vec<(String, ParameterValue)>,
    pub params_files: Vec<String>,
    #[serde(default)]
    pub remaps: Vec<(String, String)>,
    pub ros_args: Option<Vec<String>>,
    pub args: Option<Vec<String>>,
    pub cmd: Vec<String>,
    pub env: Option<Vec<(String, String)>>,
    #[serde(default)]
    pub respawn: Option<bool>,
    #[serde(default)]
    pub respawn_delay: Option<f64>,
    #[serde(default)]
    pub global_params: Option<Vec<(String, ParameterValue)>>,
    /// Scope ID referencing the scopes table (launch file origin)
    #[serde(default)]
    pub scope: Option<usize>,
}

/// The serialization format for a ROS node record.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct NodeRecord {
    pub executable: String,
    pub package: Option<String>,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub exec_name: Option<String>,
    #[serde(default)]
    pub params: Vec<(String, ParameterValue)>,
    pub params_files: Vec<String>,
    #[serde(default)]
    pub remaps: Vec<(String, String)>,
    pub ros_args: Option<Vec<String>>,
    pub args: Option<Vec<String>>,
    /// Raw command line (used by removed print_shell functionality)
    #[allow(dead_code)]
    pub cmd: Vec<String>,
    pub env: Option<Vec<(String, String)>>,
    #[serde(default)]
    pub respawn: Option<bool>,
    #[serde(default)]
    pub respawn_delay: Option<f64>,
    /// Global parameters from SetParameter action (scope-aware)
    #[serde(default)]
    pub global_params: Option<Vec<(String, ParameterValue)>>,
    /// `<node machine="…">` — the target host the launch routes this node to
    /// (ROS 2 multi-host launch). Mirrors `play_launch_parser`'s
    /// `record::types::NodeRecord::machine` (nano-ros #236); additive so
    /// existing record.json without the field still deserializes (`None`).
    /// Only regular `<node>` records carry `machine` — the parser does not
    /// emit it for `<node_container>`/`<composable_node>` (see
    /// `ComposableNodeContainerRecord`/`ComposableNodeRecord` below, which
    /// intentionally have no `machine` field).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub machine: Option<String>,
    /// Scope ID referencing the scopes table (launch file origin)
    #[serde(default)]
    pub scope: Option<usize>,
}

/// The serialization format for a composable node record.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ComposableNodeRecord {
    pub package: String,
    pub plugin: String,
    pub target_container_name: String,
    pub node_name: String,
    pub namespace: String,
    pub log_level: Option<String>,

    #[serde(default)]
    pub remaps: Vec<(String, String)>,

    #[serde(default)]
    pub params: Vec<(String, ParameterValue)>,

    #[serde(default)]
    pub extra_args: HashMap<String, String>,

    /// Launch-declared env for the composable node. The Rust parser never
    /// currently populates this (composable nodes have no independent
    /// process/env of their own — they load into their container's), so in
    /// practice this is always `None`; carried through for forward-compat
    /// and read by `model_builder` into `NodeInstance::env` (Phase 46.2).
    pub env: Option<Vec<(String, String)>>,

    /// Scope ID referencing the scopes table (launch file origin)
    #[serde(default)]
    pub scope: Option<usize>,
}

/// Read an deserialize the launch record dump.
pub fn load_launch_dump(dump_file: &Path) -> eyre::Result<LaunchDump> {
    use tracing::debug;

    debug!("Opening file: {}", dump_file.display());
    let file = File::open(dump_file)?;
    debug!("File opened successfully, creating buffered reader...");

    let reader = BufReader::new(file);
    debug!("Buffered reader created, starting JSON deserialization...");

    let launch_dump: LaunchDump = serde_json::from_reader(reader)?;
    debug!("JSON deserialization complete!");

    Ok(launch_dump)
}

/// Phase 46.5 — fail loud on a stale pre-Phase-40.1 `play_launch` Python
/// install. Such an install silently omits `ScopeOrigin.path`, which
/// disables the provider-sidecar contract + platform-file channels with NO
/// error — producing a silently degraded model that `scripts/compare_models.py`/
/// `just compare-dumps` would score as PASS while actually reflecting the
/// stale parser, not the current source.
///
/// Every file scope emitted by the CURRENT Python source carries `path`
/// (see `python/play_launch/dump/visitor/include_launch_description.py`,
/// which always passes `path=` to `push_scope()`); the Rust parser likewise
/// emits it. So a file scope with `origin.is_some()` but `path().is_none()`
/// is a reliable stale-install signal. This guard fires on every Python
/// parse this in-memory `LaunchDump` intermediate feeds (`resolve`/`dump`/
/// `launch`, Phase 47: all model-building, no record.json artifact) — so
/// stale usage can never silently pass anywhere, including the parity
/// tooling.
pub fn ensure_python_scope_paths(dump: &LaunchDump) -> eyre::Result<()> {
    let stale: Vec<&str> = dump
        .scopes
        .iter()
        .filter(|s| s.is_file_scope() && s.path().is_none())
        .filter_map(|s| s.file())
        .collect();
    if !stale.is_empty() {
        eyre::bail!(
            "Python parser produced {} file scope(s) without `ScopeOrigin.path` \
             ({stale:?}) — this means a STALE `play_launch` Python install \
             (pre-Phase-40.1) is shadowing the current source on PYTHONPATH. \
             Contract/sched sidecar resolution would silently find nothing (a \
             degraded model) that `scripts/compare_models.py`/`just \
             compare-dumps` would score as PASS while reflecting the stale \
             parser. Fix: prepend the current source's `python/` dir to PYTHONPATH \
             (`export PYTHONPATH=<repo>/python:$PYTHONPATH`), or reinstall the \
             current wheel (`pip install --force-reinstall --no-deps \
             dist/play_launch-*.whl`).",
            stale.len()
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Old records (pre-Phase-40) have no `path` key in `ScopeOrigin`.
    /// `#[serde(default)]` must keep these deserializable, with `path()`
    /// returning `None`.
    #[test]
    fn test_scope_origin_path_defaults_to_none_for_old_records() {
        let json = r#"{
            "node": [],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {
                    "id": 0,
                    "origin": { "pkg": "demo_nodes_cpp", "file": "talker.launch.xml" },
                    "ns": "/",
                    "args": {},
                    "parent": null
                }
            ]
        }"#;

        let dump: LaunchDump = serde_json::from_str(json).unwrap();
        assert_eq!(dump.scopes.len(), 1);
        assert_eq!(dump.scopes[0].file(), Some("talker.launch.xml"));
        assert_eq!(dump.scopes[0].path(), None);
    }

    /// New records carry an absolute `path`; it round-trips through
    /// deserialization intact.
    #[test]
    fn test_scope_origin_path_present_in_new_records() {
        let json = r#"{
            "node": [],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {
                    "id": 0,
                    "origin": {
                        "pkg": "demo_nodes_cpp",
                        "file": "talker.launch.xml",
                        "path": "/opt/ros/humble/share/demo_nodes_cpp/launch/talker.launch.xml"
                    },
                    "ns": "/",
                    "args": {},
                    "parent": null
                }
            ]
        }"#;

        let dump: LaunchDump = serde_json::from_str(json).unwrap();
        assert_eq!(
            dump.scopes[0].path(),
            Some("/opt/ros/humble/share/demo_nodes_cpp/launch/talker.launch.xml")
        );
    }
}
