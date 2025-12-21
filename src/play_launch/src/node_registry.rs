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

/// Node status based on process state
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
    pub fn get_status(&self) -> NodeStatus {
        // Check if we have a running child process
        if let Some(ref child) = self.child {
            if child.id().is_some() {
                return NodeStatus::Running;
            }
        }

        // Check PID file and process state
        if let Some(pid) = self.get_pid() {
            if is_process_running(pid) {
                return NodeStatus::Running;
            }
        }

        // Check status file for exit code
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
    pub status: NodeStatus,
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
    pub fn get_status(&self, name: &str) -> Option<NodeStatus> {
        self.nodes.get(name).map(|h| h.get_status())
    }

    /// List all nodes with summary information
    pub fn list_nodes(&self) -> Vec<NodeSummary> {
        self.nodes
            .values()
            .map(|handle| NodeSummary {
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
        let mut running = 0;
        let mut stopped = 0;
        let mut failed = 0;
        let mut pending = 0;
        let mut noisy = 0;
        let mut nodes = 0;
        let mut containers = 0;
        let mut composable_nodes = 0;

        // Threshold for noisy nodes: 10KB of stderr output
        const NOISY_THRESHOLD_BYTES: u64 = 10 * 1024;

        for handle in self.nodes.values() {
            match handle.get_status() {
                NodeStatus::Running => running += 1,
                NodeStatus::Stopped => stopped += 1,
                NodeStatus::Failed => failed += 1,
                NodeStatus::Pending => pending += 1,
            }

            // Count node types
            match handle.node_type {
                NodeType::Node => nodes += 1,
                NodeType::Container => containers += 1,
                NodeType::ComposableNode => composable_nodes += 1,
            }

            // Check if node is noisy (has significant stderr output)
            if handle.log_paths.stderr.exists() {
                if let Ok(metadata) = std::fs::metadata(&handle.log_paths.stderr) {
                    if metadata.len() > NOISY_THRESHOLD_BYTES {
                        noisy += 1;
                    }
                }
            }
        }

        HealthSummary {
            total: self.nodes.len(),
            running,
            stopped,
            failed,
            pending,
            noisy,
            nodes,
            containers,
            composable_nodes,
        }
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
        if handle.get_status() == NodeStatus::Running {
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
}

/// Health summary for all nodes
#[derive(Debug, Clone, Serialize)]
pub struct HealthSummary {
    pub total: usize,
    pub running: usize,
    pub stopped: usize,
    pub failed: usize,
    pub pending: usize,
    /// Number of nodes with significant stderr output (>10KB)
    pub noisy: usize,
    /// Number of regular nodes
    pub nodes: usize,
    /// Number of containers
    pub containers: usize,
    /// Number of composable nodes
    pub composable_nodes: usize,
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
