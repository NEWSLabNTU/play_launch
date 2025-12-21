//! Node registry for tracking and controlling ROS nodes.
//!
//! Provides a central registry for managing node lifecycle during replay,
//! supporting the web UI feature for node monitoring and control.

// Allow dead code during development - this module will be integrated in Phase 8.5
#![allow(dead_code)]

use crate::{
    context::{ComposableNodeContext, NodeContext},
    launch_dump::{ComposableNodeRecord, NodeRecord},
    node_cmdline::NodeCommandLine,
};
use serde::Serialize;
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};
use tokio::{process::Child, sync::Mutex};

/// Node status for regular nodes and containers (process-based)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum NodeStatus {
    /// Node is currently running
    Running,
    /// Node has stopped (exited normally)
    Stopped,
    /// Node has failed (exited with error)
    Failed,
    /// Node is pending start
    Pending,
}

/// Status for composable nodes (loaded into containers)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum ComposableNodeStatus {
    /// Successfully loaded into container
    Loaded,
    /// Load failed
    Failed,
    /// Not yet loaded
    Pending,
}

/// Unified status for all node types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(tag = "type", content = "value")]
pub enum UnifiedStatus {
    /// Process-based status (regular nodes and containers)
    Process(NodeStatus),
    /// Composable node status
    Composable(ComposableNodeStatus),
}

/// Type of node in the registry
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum NodeType {
    /// Regular ROS node
    Node,
    /// Node container (for composable nodes)
    Container,
    /// Composable node loaded into a container
    ComposableNode,
}

/// Paths to log files for a node
#[derive(Debug, Clone, Serialize)]
pub struct NodeLogPaths {
    /// Path to stdout log file
    pub stdout: PathBuf,
    /// Path to stderr log file
    pub stderr: PathBuf,
    /// Path to PID file
    pub pid_file: PathBuf,
    /// Path to command line file
    pub cmdline_file: PathBuf,
    /// Path to status file
    pub status_file: PathBuf,
    /// Path to metrics CSV (if monitoring enabled)
    pub metrics_file: PathBuf,
}

impl NodeLogPaths {
    /// Create log paths from an output directory
    pub fn from_output_dir(output_dir: &Path) -> Self {
        Self {
            stdout: output_dir.join("out"),
            stderr: output_dir.join("err"),
            pid_file: output_dir.join("pid"),
            cmdline_file: output_dir.join("cmdline"),
            status_file: output_dir.join("status"),
            metrics_file: output_dir.join("metrics.csv"),
        }
    }
}

/// Information about a regular node
#[derive(Debug)]
pub struct RegularNodeInfo {
    pub record: NodeRecord,
    pub cmdline: NodeCommandLine,
}

/// Information about a composable node
#[derive(Debug)]
pub struct ComposableNodeInfo {
    pub record: ComposableNodeRecord,
}

/// Node-specific information based on type
#[derive(Debug)]
pub enum NodeInfo {
    Regular(Box<RegularNodeInfo>),
    Composable(Box<ComposableNodeInfo>),
}

/// Handle for a registered node
#[derive(Debug)]
pub struct NodeHandle {
    /// Unique name for the node (directory name in logs)
    pub name: String,
    /// Type of node
    pub node_type: NodeType,
    /// Whether this node is a container
    pub is_container: bool,
    /// Node-specific information
    pub info: NodeInfo,
    /// Output directory for logs
    pub output_dir: PathBuf,
    /// Paths to log files
    pub log_paths: NodeLogPaths,
    /// Child process handle (if running)
    pub child: Option<Child>,
    /// Last known PID (may be stale if process exited)
    pub last_pid: Option<u32>,
}

impl NodeHandle {
    /// Create a new NodeHandle from a NodeContext
    pub fn from_node_context(context: &NodeContext, is_container: bool) -> Self {
        let name = context
            .output_dir
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown")
            .to_string();

        let node_type = if is_container {
            NodeType::Container
        } else {
            NodeType::Node
        };

        Self {
            name,
            node_type,
            is_container,
            info: NodeInfo::Regular(Box::new(RegularNodeInfo {
                record: context.record.clone(),
                cmdline: context.cmdline.clone(),
            })),
            output_dir: context.output_dir.clone(),
            log_paths: NodeLogPaths::from_output_dir(&context.output_dir),
            child: None,
            last_pid: None,
        }
    }

    /// Create a new NodeHandle from a ComposableNodeContext
    pub fn from_composable_node_context(context: &ComposableNodeContext) -> Self {
        let name = context
            .output_dir
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown")
            .to_string();

        Self {
            name,
            node_type: NodeType::ComposableNode,
            is_container: false,
            info: NodeInfo::Composable(Box::new(ComposableNodeInfo {
                record: context.record.clone(),
            })),
            output_dir: context.output_dir.clone(),
            log_paths: NodeLogPaths::from_output_dir(&context.output_dir),
            child: None,
            last_pid: None,
        }
    }

    /// Get the current PID, reading from filesystem if needed
    pub fn get_pid(&self) -> Option<u32> {
        // First try in-memory child handle
        if let Some(ref child) = self.child {
            if let Some(pid) = child.id() {
                return Some(pid);
            }
        }

        // Fall back to reading PID file
        if self.log_paths.pid_file.exists() {
            if let Ok(content) = std::fs::read_to_string(&self.log_paths.pid_file) {
                if let Ok(pid) = content.trim().parse::<u32>() {
                    return Some(pid);
                }
            }
        }

        self.last_pid
    }

    /// Get node status by checking process state
    pub fn get_status(&self) -> UnifiedStatus {
        match self.node_type {
            NodeType::Node | NodeType::Container => {
                // Process-based status checking
                let status = if let Some(ref child) = self.child {
                    if child.id().is_some() {
                        NodeStatus::Running
                    } else {
                        self.get_process_status()
                    }
                } else if let Some(pid) = self.get_pid() {
                    if is_process_running(pid) {
                        NodeStatus::Running
                    } else {
                        self.get_process_status()
                    }
                } else {
                    self.get_process_status()
                };

                UnifiedStatus::Process(status)
            }
            NodeType::ComposableNode => {
                // Composable node status checking (based on service_response.* files)
                let status = self.check_composable_node_status();
                UnifiedStatus::Composable(status)
            }
        }
    }

    /// Get process-based status from status file
    fn get_process_status(&self) -> NodeStatus {
        if self.log_paths.status_file.exists() {
            if let Ok(content) = std::fs::read_to_string(&self.log_paths.status_file) {
                let code = content.trim();
                if code == "0" {
                    return NodeStatus::Stopped;
                } else if code != "[none]" {
                    return NodeStatus::Failed;
                }
            }
            // Status file exists but no running process means stopped
            return NodeStatus::Stopped;
        }

        // No status file and no running process means pending
        NodeStatus::Pending
    }

    /// Check composable node status by reading service_response file and termination marker
    fn check_composable_node_status(&self) -> ComposableNodeStatus {
        let response_path = self.output_dir.join("service_response");
        let terminated_path = self.output_dir.join("terminated");

        // If terminated marker exists, the composable node is no longer loaded
        if terminated_path.exists() {
            return ComposableNodeStatus::Failed;
        }

        if let Ok(content) = std::fs::read_to_string(&response_path) {
            // Check the first line for "success: true" or "success: false"
            if let Some(first_line) = content.lines().next() {
                if first_line.contains("success: true") {
                    return ComposableNodeStatus::Loaded;
                } else if first_line.contains("success: false") {
                    return ComposableNodeStatus::Failed;
                }
            }
        }

        // No response file found or couldn't parse it
        ComposableNodeStatus::Pending
    }

    /// Get the package name
    pub fn get_package(&self) -> Option<&str> {
        match &self.info {
            NodeInfo::Regular(info) => info.record.package.as_deref(),
            NodeInfo::Composable(info) => Some(&info.record.package),
        }
    }

    /// Get the executable/plugin name
    pub fn get_executable(&self) -> &str {
        match &self.info {
            NodeInfo::Regular(info) => &info.record.executable,
            NodeInfo::Composable(info) => &info.record.plugin,
        }
    }

    /// Get the namespace
    pub fn get_namespace(&self) -> Option<&str> {
        match &self.info {
            NodeInfo::Regular(info) => info.record.namespace.as_deref(),
            NodeInfo::Composable(info) => Some(&info.record.namespace),
        }
    }

    /// Get the target container name (for composable nodes)
    pub fn get_target_container(&self) -> Option<&str> {
        match &self.info {
            NodeInfo::Regular(_) => None,
            NodeInfo::Composable(info) => Some(&info.record.target_container_name),
        }
    }

    /// Get the exec_name (unique execution name)
    pub fn get_exec_name(&self) -> Option<&str> {
        match &self.info {
            NodeInfo::Regular(info) => info.record.exec_name.as_deref(),
            NodeInfo::Composable(_) => None,
        }
    }

    /// Get the ROS node name
    pub fn get_node_name(&self) -> Option<&str> {
        match &self.info {
            NodeInfo::Regular(info) => info.record.name.as_deref(),
            NodeInfo::Composable(info) => Some(&info.record.node_name),
        }
    }
}

/// Check if a process with given PID is still running
fn is_process_running(pid: u32) -> bool {
    #[cfg(unix)]
    {
        use nix::{sys::signal::kill, unistd::Pid};
        // Signal 0 (None) checks if process exists without sending an actual signal
        kill(Pid::from_raw(pid as i32), None).is_ok()
    }

    #[cfg(not(unix))]
    {
        // On non-Unix, assume it's running
        let _ = pid;
        true
    }
}

/// Summary information about a node (for API responses)
#[derive(Debug, Clone, Serialize)]
pub struct NodeSummary {
    pub name: String,
    pub node_type: NodeType,
    pub status: UnifiedStatus,
    pub pid: Option<u32>,
    pub package: Option<String>,
    pub executable: String,
    pub namespace: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_container: Option<String>,
    pub is_container: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exec_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub node_name: Option<String>,
}

/// Detailed information about a node (for API responses)
#[derive(Debug, Clone, Serialize)]
pub struct NodeDetails {
    #[serde(flatten)]
    pub summary: NodeSummary,
    pub output_dir: PathBuf,
    pub log_paths: NodeLogPaths,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cmdline: Option<String>,
}

/// Central registry for tracking all nodes
pub struct NodeRegistry {
    /// Map of node name to handle
    nodes: HashMap<String, NodeHandle>,
    /// Base log directory
    log_dir: PathBuf,
    /// Process group ID for spawning nodes
    pgid: Option<i32>,
}

impl NodeRegistry {
    /// Create a new empty registry
    pub fn new(log_dir: PathBuf) -> Self {
        Self {
            nodes: HashMap::new(),
            log_dir,
            pgid: None,
        }
    }

    /// Set the process group ID for spawning nodes
    pub fn set_pgid(&mut self, pgid: i32) {
        self.pgid = Some(pgid);
    }

    /// Get the process group ID
    pub fn get_pgid(&self) -> Option<i32> {
        self.pgid
    }

    /// Get the base log directory
    pub fn log_dir(&self) -> &PathBuf {
        &self.log_dir
    }

    /// Register a regular node
    pub fn register_node(&mut self, context: &NodeContext, is_container: bool) {
        let handle = NodeHandle::from_node_context(context, is_container);
        let name = handle.name.clone();
        self.nodes.insert(name, handle);
    }

    /// Register a composable node
    pub fn register_composable_node(&mut self, context: &ComposableNodeContext) {
        let handle = NodeHandle::from_composable_node_context(context);
        let name = handle.name.clone();
        self.nodes.insert(name, handle);
    }

    /// Update the child process handle for a node
    pub fn set_child(&mut self, name: &str, child: Child) {
        if let Some(handle) = self.nodes.get_mut(name) {
            handle.last_pid = child.id();
            handle.child = Some(child);
        }
    }

    /// Clear the child process handle (when process exits)
    pub fn clear_child(&mut self, name: &str) {
        if let Some(handle) = self.nodes.get_mut(name) {
            handle.child = None;
        }
    }

    /// Get a reference to a node handle
    pub fn get(&self, name: &str) -> Option<&NodeHandle> {
        self.nodes.get(name)
    }

    /// Get a mutable reference to a node handle
    pub fn get_mut(&mut self, name: &str) -> Option<&mut NodeHandle> {
        self.nodes.get_mut(name)
    }

    /// Get the PID for a node
    pub fn get_pid(&self, name: &str) -> Option<u32> {
        self.nodes.get(name).and_then(|h| h.get_pid())
    }

    /// Get the status for a node
    pub fn get_status(&self, name: &str) -> Option<UnifiedStatus> {
        self.nodes.get(name).map(|h| h.get_status())
    }

    /// List all nodes with summary information
    pub fn list_nodes(&self) -> Vec<NodeSummary> {
        self.nodes
            .values()
            .map(|handle| {
                NodeSummary {
                    name: handle.name.clone(),
                    node_type: handle.node_type,
                    status: handle.get_status(),
                    pid: handle.get_pid(),
                    package: handle.get_package().map(String::from),
                    executable: handle.get_executable().to_string(),
                    namespace: handle.get_namespace().map(String::from),
                    target_container: handle.get_target_container().map(String::from),
                    is_container: handle.is_container,
                    exec_name: handle.get_exec_name().map(String::from),
                    node_name: handle.get_node_name().map(String::from),
                }
            })
            .collect()
    }

    /// Get detailed information about a node
    pub fn get_node_details(&self, name: &str) -> Option<NodeDetails> {
        self.nodes.get(name).map(|handle| {
            // Read command line from file if available
            let cmdline = if handle.log_paths.cmdline_file.exists() {
                std::fs::read_to_string(&handle.log_paths.cmdline_file).ok()
            } else {
                None
            };

            NodeDetails {
                summary: NodeSummary {
                    name: handle.name.clone(),
                    node_type: handle.node_type,
                    status: handle.get_status(),
                    pid: handle.get_pid(),
                    package: handle.get_package().map(String::from),
                    executable: handle.get_executable().to_string(),
                    namespace: handle.get_namespace().map(String::from),
                    target_container: handle.get_target_container().map(String::from),
                    is_container: handle.is_container,
                    exec_name: handle.get_exec_name().map(String::from),
                    node_name: handle.get_node_name().map(String::from),
                },
                output_dir: handle.output_dir.clone(),
                log_paths: handle.log_paths.clone(),
                cmdline,
            }
        })
    }

    /// Get health summary counts
    pub fn get_health_summary(&self) -> HealthSummary {
        let mut summary = HealthSummary {
            processes_running: 0,
            processes_stopped: 0,
            nodes_running: 0,
            nodes_stopped: 0,
            nodes_failed: 0,
            nodes_total: 0,
            containers_running: 0,
            containers_stopped: 0,
            containers_failed: 0,
            containers_total: 0,
            composable_loaded: 0,
            composable_failed: 0,
            composable_pending: 0,
            composable_total: 0,
            noisy: 0,
        };

        // Threshold for noisy nodes: 10KB of stderr output
        const NOISY_THRESHOLD_BYTES: u64 = 10 * 1024;

        for handle in self.nodes.values() {
            let status = handle.get_status();

            // Count by node type and status
            match handle.node_type {
                NodeType::Node => {
                    summary.nodes_total += 1;
                    match status {
                        UnifiedStatus::Process(NodeStatus::Running) => {
                            summary.nodes_running += 1;
                            summary.processes_running += 1;
                        }
                        UnifiedStatus::Process(NodeStatus::Stopped) => {
                            summary.nodes_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        UnifiedStatus::Process(NodeStatus::Failed) => {
                            summary.nodes_failed += 1;
                        }
                        _ => {}
                    }
                }
                NodeType::Container => {
                    summary.containers_total += 1;
                    match status {
                        UnifiedStatus::Process(NodeStatus::Running) => {
                            summary.containers_running += 1;
                            summary.processes_running += 1;
                        }
                        UnifiedStatus::Process(NodeStatus::Stopped) => {
                            summary.containers_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        UnifiedStatus::Process(NodeStatus::Failed) => {
                            summary.containers_failed += 1;
                        }
                        _ => {}
                    }
                }
                NodeType::ComposableNode => {
                    summary.composable_total += 1;
                    match status {
                        UnifiedStatus::Composable(ComposableNodeStatus::Loaded) => {
                            summary.composable_loaded += 1;
                        }
                        UnifiedStatus::Composable(ComposableNodeStatus::Failed) => {
                            summary.composable_failed += 1;
                        }
                        UnifiedStatus::Composable(ComposableNodeStatus::Pending) => {
                            summary.composable_pending += 1;
                        }
                        _ => {}
                    }
                }
            }

            // Check if node is noisy (has significant stderr output)
            if handle.log_paths.stderr.exists() {
                if let Ok(metadata) = std::fs::metadata(&handle.log_paths.stderr) {
                    if metadata.len() > NOISY_THRESHOLD_BYTES {
                        summary.noisy += 1;
                    }
                }
            }
        }

        summary
    }

    /// Get all node names
    pub fn node_names(&self) -> Vec<&str> {
        self.nodes.keys().map(|s| s.as_str()).collect()
    }

    /// Get the total number of nodes
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    /// Check if the registry is empty
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Stop a node by sending SIGTERM
    ///
    /// Returns Ok(true) if the node was stopped, Ok(false) if it wasn't running,
    /// or an error if the operation failed.
    pub fn stop_node(&mut self, name: &str) -> eyre::Result<bool> {
        use eyre::eyre;

        let handle = self
            .nodes
            .get_mut(name)
            .ok_or_else(|| eyre!("Node '{}' not found", name))?;

        // Check if we have a child process to stop
        if let Some(mut child) = handle.child.take() {
            // Try to get the PID before killing
            let pid = child.id();

            // Send SIGTERM (graceful shutdown)
            #[cfg(unix)]
            if let Some(pid) = pid {
                use nix::{
                    sys::signal::{kill, Signal},
                    unistd::Pid,
                };
                let _ = kill(Pid::from_raw(pid as i32), Signal::SIGTERM);
            }

            // Also call kill on the Child handle as a fallback
            // This will send SIGKILL if the process hasn't exited
            // We use start_kill which is non-blocking
            let _ = child.start_kill();

            // Update last_pid for reference
            if let Some(pid) = pid {
                handle.last_pid = Some(pid);
            }

            return Ok(true);
        }

        // No child handle, try to kill by PID from file
        if let Some(pid) = handle.get_pid() {
            if is_process_running(pid) {
                #[cfg(unix)]
                {
                    use nix::{
                        sys::signal::{kill, Signal},
                        unistd::Pid,
                    };
                    kill(Pid::from_raw(pid as i32), Signal::SIGTERM)
                        .map_err(|e| eyre!("Failed to send SIGTERM to PID {}: {}", pid, e))?;
                }
                return Ok(true);
            }
        }

        // Node wasn't running
        Ok(false)
    }

    /// Start a node by spawning a new process
    ///
    /// Only works for regular nodes (not composable nodes which are loaded via service).
    /// Uses the pgid stored in the registry (set via `set_pgid`).
    /// Returns Ok(pid) on success.
    pub async fn start_node(&mut self, name: &str) -> eyre::Result<u32> {
        use eyre::{eyre, WrapErr};
        use std::{fs::File, process::Stdio};
        use tokio::process::Command;

        let pgid = self.pgid;

        let handle = self
            .nodes
            .get_mut(name)
            .ok_or_else(|| eyre!("Node '{}' not found", name))?;

        // Check if already running
        if handle.get_status() == UnifiedStatus::Process(NodeStatus::Running) {
            return Err(eyre!("Node '{}' is already running", name));
        }

        // Get the command line - only available for regular nodes
        let cmdline = match &handle.info {
            NodeInfo::Regular(info) => &info.cmdline,
            NodeInfo::Composable(_) => {
                return Err(eyre!(
                    "Cannot start composable node '{}' - use container service instead",
                    name
                ));
            }
        };

        // Create output directory if it doesn't exist
        std::fs::create_dir_all(&handle.output_dir)
            .wrap_err_with(|| format!("Failed to create output directory for {}", name))?;

        // Set up output files
        let stdout_file = File::create(&handle.log_paths.stdout)
            .wrap_err_with(|| format!("Failed to create stdout file for {}", name))?;
        let stderr_file = File::create(&handle.log_paths.stderr)
            .wrap_err_with(|| format!("Failed to create stderr file for {}", name))?;

        // Build the command
        let args = cmdline.to_cmdline(false);
        let (program, args) = args
            .split_first()
            .ok_or_else(|| eyre!("Empty command line for node '{}'", name))?;

        let mut command = Command::new(program);
        command.args(args);

        // Apply environment variables from launch file
        command.envs(&cmdline.env);

        // Preserve AMENT_PREFIX_PATH
        if let Ok(ament_prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
            command.env("AMENT_PREFIX_PATH", ament_prefix_path);
        }

        // Set process group
        #[cfg(unix)]
        {
            #[allow(unused_imports)]
            use std::os::unix::process::CommandExt;
            if let Some(pgid) = pgid {
                command.process_group(pgid);
            } else {
                command.process_group(0);
            }
        }

        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        // Spawn the process
        let child = command
            .spawn()
            .wrap_err_with(|| format!("Failed to spawn process for node '{}'", name))?;

        let pid = child
            .id()
            .ok_or_else(|| eyre!("Failed to get PID for node '{}'", name))?;

        // Write PID file
        std::fs::write(&handle.log_paths.pid_file, format!("{}\n", pid))
            .wrap_err_with(|| format!("Failed to write PID file for {}", name))?;

        // Write command line file
        let cmdline_str = cmdline.to_shell(false);
        std::fs::write(&handle.log_paths.cmdline_file, &cmdline_str)
            .wrap_err_with(|| format!("Failed to write cmdline file for {}", name))?;

        // Store the child handle
        handle.last_pid = Some(pid);
        handle.child = Some(child);

        Ok(pid)
    }

    /// Restart a node by stopping it and starting it again
    ///
    /// Uses the pgid stored in the registry (set via `set_pgid`).
    /// Returns Ok(pid) of the new process on success.
    pub async fn restart_node(&mut self, name: &str) -> eyre::Result<u32> {
        use eyre::eyre;
        use std::time::Duration;

        // First, stop the node if it's running
        let was_running = self.stop_node(name)?;

        if was_running {
            // Wait a bit for the process to actually terminate
            tokio::time::sleep(Duration::from_millis(100)).await;

            // Check if process is actually stopped
            let handle = self
                .nodes
                .get(name)
                .ok_or_else(|| eyre!("Node '{}' not found", name))?;

            if let Some(pid) = handle.last_pid {
                // Wait up to 2 seconds for process to terminate
                for _ in 0..20 {
                    if !is_process_running(pid) {
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }

                // If still running, send SIGKILL
                if is_process_running(pid) {
                    #[cfg(unix)]
                    {
                        use nix::{
                            sys::signal::{kill, Signal},
                            unistd::Pid,
                        };
                        let _ = kill(Pid::from_raw(pid as i32), Signal::SIGKILL);
                    }
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            }
        }

        // Now start the node
        self.start_node(name).await
    }

    /// Get all composable nodes belonging to a container
    pub fn get_container_composable_nodes(&self, container_name: &str) -> Vec<String> {
        let container_full_name = self.get_container_full_ros_name(container_name);

        self.nodes
            .values()
            .filter(|handle| {
                if handle.node_type != NodeType::ComposableNode {
                    return false;
                }

                // Get target_container from composable node record
                if let NodeInfo::Composable(info) = &handle.info {
                    return &info.record.target_container_name == &container_full_name;
                }

                false
            })
            .map(|handle| handle.name.clone())
            .collect()
    }

    /// Get container's full ROS name (namespace + name)
    fn get_container_full_ros_name(&self, container_name: &str) -> String {
        if let Some(handle) = self.nodes.get(container_name) {
            if let NodeInfo::Regular(info) = &handle.info {
                let ns = info.record.namespace.as_deref();
                let name = &handle.name;

                return match ns {
                    Some("/") => format!("/{}", name),
                    Some(ns) if ns.ends_with('/') => format!("{}{}", ns, name),
                    Some(ns) => format!("{}/{}", ns, name),
                    None => format!("/{}", name),
                };
            }
        }

        // Fallback
        format!("/{}", container_name)
    }
}

/// Health summary for all nodes
#[derive(Debug, Clone, Serialize)]
pub struct HealthSummary {
    // Process-level counts
    pub processes_running: usize,
    pub processes_stopped: usize,

    // Regular node counts
    pub nodes_running: usize,
    pub nodes_stopped: usize,
    pub nodes_failed: usize,
    pub nodes_total: usize,

    // Container counts
    pub containers_running: usize,
    pub containers_stopped: usize,
    pub containers_failed: usize,
    pub containers_total: usize,

    // Composable node counts
    pub composable_loaded: usize,
    pub composable_failed: usize,
    pub composable_pending: usize,
    pub composable_total: usize,

    /// Number of nodes with significant stderr output (>10KB)
    pub noisy: usize,
}

/// Thread-safe handle to the node registry
pub type SharedNodeRegistry = Arc<Mutex<NodeRegistry>>;

/// Create a new shared node registry
pub fn create_shared_registry(log_dir: PathBuf) -> SharedNodeRegistry {
    Arc::new(Mutex::new(NodeRegistry::new(log_dir)))
}

/// Populate the registry from node contexts
///
/// This should be called before spawning nodes to register all nodes
/// in the registry. The nodes will be in Pending state until spawned.
pub fn populate_registry_from_contexts(
    registry: &SharedNodeRegistry,
    container_contexts: &[crate::context::NodeContainerContext],
    non_container_contexts: &[NodeContext],
    composable_node_contexts: &[ComposableNodeContext],
) {
    // Use blocking lock since this is called during initialization
    let mut reg = match registry.try_lock() {
        Ok(guard) => guard,
        Err(_) => {
            tracing::warn!("Failed to acquire registry lock during population");
            return;
        }
    };

    // Register container nodes
    for ctx in container_contexts {
        reg.register_node(&ctx.node_context, true);
    }

    // Register non-container nodes
    for ctx in non_container_contexts {
        reg.register_node(ctx, false);
    }

    // Register composable nodes
    for ctx in composable_node_contexts {
        reg.register_composable_node(ctx);
    }

    tracing::debug!("Populated registry with {} nodes", reg.len());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_log_paths() {
        let output_dir = PathBuf::from("/tmp/test_node");
        let paths = NodeLogPaths::from_output_dir(&output_dir);
        assert_eq!(paths.stdout, PathBuf::from("/tmp/test_node/out"));
        assert_eq!(paths.stderr, PathBuf::from("/tmp/test_node/err"));
        assert_eq!(paths.pid_file, PathBuf::from("/tmp/test_node/pid"));
    }

    #[test]
    fn test_health_summary() {
        let registry = NodeRegistry::new(PathBuf::from("/tmp/test"));
        let summary = registry.get_health_summary();
        assert_eq!(summary.total, 0);
        assert_eq!(summary.running, 0);
        assert_eq!(summary.failed, 0);
        assert_eq!(summary.noisy, 0);
    }
}
