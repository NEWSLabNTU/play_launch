use itertools::{chain, Itertools};
use serde::{Deserialize, Serialize};
use std::{
    borrow::Cow,
    collections::HashMap,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
    process::Command,
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

impl ComposableNodeRecord {
    fn to_cmdline(&self, standalone: bool) -> Vec<String> {
        if standalone {
            self.to_cmdline_standalone()
        } else {
            self.to_cmdline_component()
        }
    }

    fn to_cmdline_standalone(&self) -> Vec<String> {
        let Self {
            package: _,
            plugin: _,
            namespace,
            log_level,
            remaps,
            params: _,
            extra_args: _,
            node_name,
            target_container_name: _,
            env: _,
        } = self;

        // Option A: Spawn standalone container directly using ament index
        // Find component_container executable directly
        let container_exe = match crate::ros::ament_index::find_executable(
            "rclcpp_components",
            "component_container",
        ) {
            Ok(path) => path.to_string_lossy().into_owned(),
            Err(_) => {
                // Fallback to full path if ament index fails
                "/opt/ros/humble/lib/rclcpp_components/component_container".to_string()
            }
        };

        // Generate unique container name for this standalone node
        let container_name = format!("standalone_container_{}", node_name);
        let node_remap = format!("__node:={}", container_name);
        let ns_remap = format!("__ns:={}", namespace);

        let command = vec![
            Cow::from(container_exe),
            Cow::from("--ros-args"),
            Cow::from("-r"),
            Cow::from(node_remap),
            Cow::from("-r"),
            Cow::from(ns_remap),
        ];

        let remap_args = remaps
            .iter()
            .flat_map(|(src, tgt)| [Cow::from("-r"), format!("{src}:={tgt}").into()]);

        let log_level_args = log_level
            .as_deref()
            .map(|level| vec![Cow::from("--log-level"), Cow::from(level)])
            .unwrap_or_default();

        // Note: Standalone mode now only spawns the container.
        // Component loading would need to be done separately via component_loader,
        // which is a more complex refactoring. For now, this provides the container
        // infrastructure for manual component loading.
        chain!(command, log_level_args, remap_args)
            .map(|arg| arg.to_string())
            .collect()
    }

    fn to_cmdline_component(&self) -> Vec<String> {
        let Self {
            package,
            plugin,
            namespace,
            log_level,
            remaps,
            params,
            extra_args,
            target_container_name,
            node_name,
            env: _,
        } = self;

        let command = [
            "ros2",
            "component",
            "load",
            target_container_name,
            package,
            plugin,
            "-n",
            node_name,
            "--node-namespace",
            namespace,
        ]
        .into_iter()
        .map(Cow::from);

        let remap_args = remaps
            .iter()
            .flat_map(|(src, tgt)| [Cow::from("-r"), format!("{src}:={tgt}").into()]);

        let param_args = params
            .iter()
            .flat_map(|(name, value)| [Cow::from("-p"), format!("{name}:={}", value).into()]);

        let extra_arg_args = extra_args
            .iter()
            .flat_map(|(name, value)| [Cow::from("-e"), format!("{name}:={}", value).into()]);

        let log_level_args = log_level
            .as_deref()
            .map(|level| ["--log-level", level])
            .into_iter()
            .flatten()
            .map(Cow::from);

        chain!(
            command,
            log_level_args,
            remap_args,
            param_args,
            extra_arg_args
        )
        .map(|arg| arg.to_string())
        .collect()
    }

    pub fn to_command(&self, standalone: bool) -> Command {
        let cmdline = self.to_cmdline(standalone);
        let (program, args) = cmdline
            .split_first()
            .expect("command line must not be empty");
        let mut command = Command::new(program);
        command.args(args);
        command
    }

    pub fn to_shell(&self, standalone: bool) -> Vec<u8> {
        let cmdline = self.to_cmdline(standalone);
        Itertools::intersperse(
            cmdline.into_iter().map(|w| shell_quote::Sh::quote_vec(&w)),
            vec![b' '],
        )
        .flatten()
        .collect()
    }
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
