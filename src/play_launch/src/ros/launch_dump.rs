use serde::{Deserialize, Serialize};
use std::{
    collections::HashMap,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

pub type ParameterValue = String;

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

    #[allow(dead_code)]
    pub env: Option<Vec<(String, String)>>,
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
