//! Member registry - pure data storage for all members
//!
//! This module implements the MemberRegistry which stores all members (nodes, containers,
//! composable nodes) with efficient indexing for container-composable node relationships.
//!
//! # Design
//! - Pure data storage (no business logic)
//! - Indexed relationships between containers and composable nodes
//! - State stored in memory (not computed from filesystem)
//! - State transitions with validation

use crate::{
    member::{
        BlockReason, ComposableNode, ComposableState, Container, Member, ProcessState, RegularNode,
    },
    web_types::{
        ComposableBlockReason, ComposableNodeStatus, HealthSummary, NodeStatus, NodeSummary,
        NodeType, UnifiedStatus,
    },
};
use eyre::{bail, Result};
use std::{collections::HashMap, path::PathBuf};

/// Central registry for all members
///
/// # Design
/// - Stores all members in a HashMap
/// - Maintains bidirectional indexes for container-composable relationships
/// - Provides O(1) lookup for common operations
/// - Validates state transitions
pub struct MemberRegistry {
    /// Map of member name to Member
    members: HashMap<String, Member>,

    /// Index: container name → list of composable node names
    container_to_composable: HashMap<String, Vec<String>>,

    /// Index: composable node name → container name
    composable_to_container: HashMap<String, String>,

    /// Base log directory
    #[allow(dead_code)]
    log_dir: PathBuf,
}

impl MemberRegistry {
    /// Create a new empty registry
    pub fn new(log_dir: PathBuf) -> Self {
        Self {
            members: HashMap::new(),
            container_to_composable: HashMap::new(),
            composable_to_container: HashMap::new(),
            log_dir,
        }
    }

    /// Get the base log directory
    #[allow(dead_code)]
    pub fn log_dir(&self) -> &PathBuf {
        &self.log_dir
    }

    /// Register a regular node
    pub fn register_node(&mut self, node: RegularNode) {
        let name = node.name.clone();
        self.members.insert(name, Member::Node(node));
    }

    /// Register a container
    pub fn register_container(&mut self, container: Container) {
        let name = container.name.clone();
        // Initialize empty composable nodes list
        self.container_to_composable
            .insert(name.clone(), Vec::new());
        self.members.insert(name, Member::Container(container));
    }

    /// Register a composable node
    pub fn register_composable_node(&mut self, composable: ComposableNode) {
        let name = composable.name.clone();
        let container_name = composable.container_name.clone();

        // Update indexes
        self.composable_to_container
            .insert(name.clone(), container_name.clone());

        // Add to container's list
        self.container_to_composable
            .entry(container_name)
            .or_default()
            .push(name.clone());

        self.members
            .insert(name, Member::ComposableNode(composable));
    }

    /// Get a member by name
    pub fn get(&self, name: &str) -> Option<&Member> {
        self.members.get(name)
    }

    /// Get a mutable member by name
    pub fn get_mut(&mut self, name: &str) -> Option<&mut Member> {
        self.members.get_mut(name)
    }

    /// Get all member names
    pub fn names(&self) -> Vec<String> {
        self.members.keys().cloned().collect()
    }

    /// Get the number of members
    pub fn len(&self) -> usize {
        self.members.len()
    }

    /// Check if the registry is empty
    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.members.is_empty()
    }

    /// Get the container name for a composable node
    #[allow(dead_code)]
    pub fn get_container_for_composable(&self, composable_name: &str) -> Option<&str> {
        self.composable_to_container
            .get(composable_name)
            .map(|s| s.as_str())
    }

    /// Get all composable nodes for a container
    pub fn get_composable_nodes_for_container(&self, container_name: &str) -> Vec<String> {
        self.container_to_composable
            .get(container_name)
            .cloned()
            .unwrap_or_default()
    }

    // ===== State Transition Methods =====

    /// Transition a process (node or container) to Running state
    pub fn transition_to_running(&mut self, name: &str, pid: u32) -> Result<()> {
        let member = self
            .members
            .get_mut(name)
            .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

        match member {
            Member::Node(node) => {
                node.state = ProcessState::Running { pid };
            }
            Member::Container(container) => {
                container.state = ProcessState::Running { pid };
            }
            Member::ComposableNode(_) => {
                bail!(
                    "Cannot transition composable node {} to Running state",
                    name
                );
            }
        }

        Ok(())
    }

    /// Transition a process to Stopped state
    pub fn transition_to_stopped(&mut self, name: &str) -> Result<()> {
        let member = self
            .members
            .get_mut(name)
            .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

        match member {
            Member::Node(node) => {
                node.state = ProcessState::Stopped;
            }
            Member::Container(container) => {
                container.state = ProcessState::Stopped;
            }
            Member::ComposableNode(_) => {
                bail!(
                    "Cannot transition composable node {} to Stopped state",
                    name
                );
            }
        }

        Ok(())
    }

    /// Transition a process to Failed state
    pub fn transition_to_failed(&mut self, name: &str, exit_code: Option<i32>) -> Result<()> {
        let member = self
            .members
            .get_mut(name)
            .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

        match member {
            Member::Node(node) => {
                node.state = ProcessState::Failed { exit_code };
            }
            Member::Container(container) => {
                container.state = ProcessState::Failed { exit_code };
            }
            Member::ComposableNode(_) => {
                bail!("Cannot transition composable node {} to Failed state", name);
            }
        }

        Ok(())
    }

    /// Transition a composable node to Loading state
    pub fn transition_to_loading(&mut self, name: &str) -> Result<()> {
        let member = self
            .members
            .get_mut(name)
            .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

        match member {
            Member::ComposableNode(composable) => {
                composable.state = ComposableState::Loading;
            }
            _ => {
                bail!(
                    "Cannot transition non-composable member {} to Loading state",
                    name
                );
            }
        }

        Ok(())
    }

    /// Transition a composable node to Loaded state
    pub fn transition_to_loaded(&mut self, name: &str) -> Result<()> {
        let member = self
            .members
            .get_mut(name)
            .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

        match member {
            Member::ComposableNode(composable) => {
                composable.state = ComposableState::Loaded;
            }
            _ => {
                bail!(
                    "Cannot transition non-composable member {} to Loaded state",
                    name
                );
            }
        }

        Ok(())
    }

    /// Transition a composable node to Unloaded state
    pub fn transition_to_unloaded(&mut self, name: &str) -> Result<()> {
        let member = self
            .members
            .get_mut(name)
            .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

        match member {
            Member::ComposableNode(composable) => {
                composable.state = ComposableState::Unloaded;
            }
            _ => {
                bail!(
                    "Cannot transition non-composable member {} to Unloaded state",
                    name
                );
            }
        }

        Ok(())
    }

    /// Block a composable node (container unavailable)
    pub fn block_composable_node(&mut self, name: &str, reason: BlockReason) -> Result<()> {
        let member = self
            .members
            .get_mut(name)
            .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

        match member {
            Member::ComposableNode(composable) => {
                composable.state = ComposableState::Blocked { reason };
            }
            _ => {
                bail!("Cannot block non-composable member {}", name);
            }
        }

        Ok(())
    }

    /// Unblock a composable node (container became available)
    pub fn unblock_composable_node(&mut self, name: &str) -> Result<()> {
        let member = self
            .members
            .get_mut(name)
            .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

        match member {
            Member::ComposableNode(composable) => {
                // Transition from Blocked to Unloaded (ready to load)
                if composable.state.is_blocked() {
                    composable.state = ComposableState::Unloaded;
                }
            }
            _ => {
                bail!("Cannot unblock non-composable member {}", name);
            }
        }

        Ok(())
    }

    /// Get the ProcessState for a node or container
    #[allow(dead_code)]
    pub fn get_process_state(&self, name: &str) -> Option<ProcessState> {
        match self.members.get(name)? {
            Member::Node(node) => Some(node.state),
            Member::Container(container) => Some(container.state),
            Member::ComposableNode(_) => None,
        }
    }

    /// Get the ComposableState for a composable node
    #[allow(dead_code)]
    pub fn get_composable_state(&self, name: &str) -> Option<ComposableState> {
        match self.members.get(name)? {
            Member::ComposableNode(composable) => Some(composable.state.clone()),
            _ => None,
        }
    }

    /// Get the PID for a running process
    pub fn get_pid(&self, name: &str) -> Option<u32> {
        match self.members.get(name)? {
            Member::Node(node) => node.state.pid(),
            Member::Container(container) => container.state.pid(),
            Member::ComposableNode(_) => None,
        }
    }

    // === Web UI Support Methods ===

    /// List all members as NodeSummary for web UI
    pub fn list_nodes_summary(&self) -> Vec<NodeSummary> {
        self.members
            .values()
            .map(|member| self.member_to_summary(member))
            .collect()
    }

    /// Get composable nodes for a container
    #[allow(dead_code)]
    pub fn get_container_composable_nodes(&self, container_name: &str) -> Vec<String> {
        self.container_to_composable
            .get(container_name)
            .cloned()
            .unwrap_or_default()
    }

    /// Get health summary for web UI
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

        for member in self.members.values() {
            match member {
                Member::Node(node) => {
                    summary.nodes_total += 1;
                    match node.state {
                        ProcessState::Running { .. } => {
                            summary.nodes_running += 1;
                            summary.processes_running += 1;
                        }
                        ProcessState::Stopped => {
                            summary.nodes_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        ProcessState::Failed { .. } => {
                            summary.nodes_failed += 1;
                        }
                        ProcessState::Pending => {}
                    }
                    // Check stderr size
                    if let Ok(metadata) = std::fs::metadata(&node.log_paths.stderr) {
                        if metadata.len() > 10240 {
                            summary.noisy += 1;
                        }
                    }
                }
                Member::Container(container) => {
                    summary.containers_total += 1;
                    match container.state {
                        ProcessState::Running { .. } => {
                            summary.containers_running += 1;
                            summary.processes_running += 1;
                        }
                        ProcessState::Stopped => {
                            summary.containers_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        ProcessState::Failed { .. } => {
                            summary.containers_failed += 1;
                        }
                        ProcessState::Pending => {}
                    }
                    // Check stderr size
                    if let Ok(metadata) = std::fs::metadata(&container.log_paths.stderr) {
                        if metadata.len() > 10240 {
                            summary.noisy += 1;
                        }
                    }
                }
                Member::ComposableNode(composable) => {
                    summary.composable_total += 1;
                    match &composable.state {
                        ComposableState::Loaded => summary.composable_loaded += 1,
                        ComposableState::Loading => summary.composable_pending += 1,
                        ComposableState::Unloaded => summary.composable_pending += 1,
                        ComposableState::Blocked { .. } => summary.composable_failed += 1,
                    }
                }
            }
        }

        summary
    }

    /// Convert a Member to NodeSummary for web UI
    fn member_to_summary(&self, member: &Member) -> NodeSummary {
        match member {
            Member::Node(node) => {
                let status = match node.state {
                    ProcessState::Running { .. } => UnifiedStatus::Process(NodeStatus::Running),
                    ProcessState::Stopped => UnifiedStatus::Process(NodeStatus::Stopped),
                    ProcessState::Failed { .. } => UnifiedStatus::Process(NodeStatus::Failed),
                    ProcessState::Pending => UnifiedStatus::Process(NodeStatus::Pending),
                };

                let (stderr_mtime, stderr_size, stderr_preview) =
                    self.read_stderr_info(&node.log_paths.stderr);

                NodeSummary {
                    name: node.name.clone(),
                    node_type: NodeType::Node,
                    status,
                    pid: node.state.pid(),
                    package: node.record.package.clone(),
                    executable: node.record.executable.clone(),
                    namespace: node.record.namespace.clone(),
                    target_container: None,
                    is_container: false,
                    exec_name: node.record.exec_name.clone(),
                    node_name: node.record.name.clone(),
                    stderr_last_modified: stderr_mtime,
                    stderr_size,
                    stderr_preview,
                    respawn_enabled: Some(node.respawn_enabled),
                    respawn_delay: if node.respawn_enabled {
                        Some(node.respawn_delay)
                    } else {
                        None
                    },
                }
            }
            Member::Container(container) => {
                let status = match container.state {
                    ProcessState::Running { .. } => UnifiedStatus::Process(NodeStatus::Running),
                    ProcessState::Stopped => UnifiedStatus::Process(NodeStatus::Stopped),
                    ProcessState::Failed { .. } => UnifiedStatus::Process(NodeStatus::Failed),
                    ProcessState::Pending => UnifiedStatus::Process(NodeStatus::Pending),
                };

                let (stderr_mtime, stderr_size, stderr_preview) =
                    self.read_stderr_info(&container.log_paths.stderr);

                NodeSummary {
                    name: container.name.clone(),
                    node_type: NodeType::Container,
                    status,
                    pid: container.state.pid(),
                    package: container.record.package.clone(),
                    executable: container.record.executable.clone(),
                    namespace: container.record.namespace.clone(),
                    target_container: None,
                    is_container: true,
                    exec_name: container.record.exec_name.clone(),
                    node_name: container.record.name.clone(),
                    stderr_last_modified: stderr_mtime,
                    stderr_size,
                    stderr_preview,
                    respawn_enabled: Some(container.respawn_enabled),
                    respawn_delay: if container.respawn_enabled {
                        Some(container.respawn_delay)
                    } else {
                        None
                    },
                }
            }
            Member::ComposableNode(composable) => {
                let status = match &composable.state {
                    ComposableState::Loaded => {
                        UnifiedStatus::Composable(ComposableNodeStatus::Loaded)
                    }
                    ComposableState::Loading => {
                        UnifiedStatus::Composable(ComposableNodeStatus::Loading)
                    }
                    ComposableState::Unloaded => {
                        UnifiedStatus::Composable(ComposableNodeStatus::Pending)
                    }
                    ComposableState::Blocked { reason } => {
                        let block_reason = match reason {
                            BlockReason::Stopped => ComposableBlockReason::ContainerStopped,
                            BlockReason::Failed => ComposableBlockReason::ContainerFailed,
                            BlockReason::NotStarted => ComposableBlockReason::ContainerNotStarted,
                        };
                        UnifiedStatus::Composable(ComposableNodeStatus::Blocked(block_reason))
                    }
                };

                let (stderr_mtime, stderr_size, stderr_preview) =
                    self.read_stderr_info(&composable.log_paths.stderr);

                NodeSummary {
                    name: composable.name.clone(),
                    node_type: NodeType::ComposableNode,
                    status,
                    pid: None,
                    package: Some(composable.record.package.clone()),
                    executable: composable.record.plugin.clone(),
                    namespace: Some(composable.record.namespace.clone()),
                    target_container: Some(composable.container_name.clone()),
                    is_container: false,
                    exec_name: Some(composable.record.node_name.clone()),
                    node_name: Some(composable.record.node_name.clone()),
                    stderr_last_modified: stderr_mtime,
                    stderr_size,
                    stderr_preview,
                    respawn_enabled: None,
                    respawn_delay: None,
                }
            }
        }
    }

    /// Read stderr file metadata and preview lines
    fn read_stderr_info(
        &self,
        stderr_path: &std::path::Path,
    ) -> (Option<u64>, u64, Option<Vec<String>>) {
        use std::io::{BufRead, BufReader};

        let metadata = match std::fs::metadata(stderr_path) {
            Ok(m) => m,
            Err(_) => return (None, 0, None),
        };

        let size = metadata.len();
        let mtime = metadata
            .modified()
            .ok()
            .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
            .map(|d| d.as_secs());

        // Read last 5 lines for preview
        let preview = if size > 0 {
            std::fs::File::open(stderr_path).ok().and_then(|file| {
                let reader = BufReader::new(file);
                let lines: Vec<String> = reader
                    .lines()
                    .map_while(Result::ok)
                    .filter(|l| !l.trim().is_empty())
                    .collect::<Vec<_>>()
                    .into_iter()
                    .rev()
                    .take(5)
                    .collect::<Vec<_>>()
                    .into_iter()
                    .rev()
                    .collect();
                if lines.is_empty() {
                    None
                } else {
                    Some(lines)
                }
            })
        } else {
            None
        };

        (mtime, size, preview)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        launch_dump::{ComposableNodeRecord, NodeRecord},
        node_cmdline::NodeCommandLine,
        web_types::NodeLogPaths,
    };

    fn make_test_node(name: &str) -> RegularNode {
        RegularNode {
            name: name.to_string(),
            state: ProcessState::Pending,
            record: NodeRecord {
                executable: "test".to_string(),
                package: Some("test_pkg".to_string()),
                name: Some(name.to_string()),
                namespace: Some("/".to_string()),
                exec_name: None,
                params: vec![],
                params_files: vec![],
                remaps: vec![],
                ros_args: None,
                args: None,
                cmd: vec![],
                env: None,
                respawn: None,
                respawn_delay: None,
                global_params: None,
            },
            cmdline: NodeCommandLine {
                command: vec!["test".to_string()],
                user_args: vec![],
                remaps: HashMap::new(),
                params: HashMap::new(),
                params_files: std::collections::HashSet::new(),
                log_level: None,
                log_config_file: None,
                rosout_logs: None,
                stdout_logs: None,
                enclave: None,
                env: HashMap::new(),
            },
            output_dir: PathBuf::from("/tmp"),
            log_paths: NodeLogPaths {
                stdout: PathBuf::from("/tmp/out"),
                stderr: PathBuf::from("/tmp/err"),
                pid_file: PathBuf::from("/tmp/pid"),
                cmdline_file: PathBuf::from("/tmp/cmdline"),
                status_file: PathBuf::from("/tmp/status"),
                metrics_file: PathBuf::from("/tmp/metrics.csv"),
            },
            respawn_enabled: false,
            respawn_delay: 0.0,
        }
    }

    #[test]
    fn test_register_and_get_node() {
        let mut registry = MemberRegistry::new(PathBuf::from("/tmp"));
        let node = make_test_node("test_node");

        registry.register_node(node);

        assert_eq!(registry.len(), 1);
        assert!(registry.get("test_node").is_some());
        assert!(registry.get("test_node").unwrap().is_regular_node());
    }

    #[test]
    fn test_transition_to_running() {
        let mut registry = MemberRegistry::new(PathBuf::from("/tmp"));
        let node = make_test_node("test_node");
        registry.register_node(node);

        registry.transition_to_running("test_node", 1234).unwrap();

        let state = registry.get_process_state("test_node").unwrap();
        assert_eq!(state, ProcessState::Running { pid: 1234 });
        assert_eq!(registry.get_pid("test_node"), Some(1234));
    }

    #[test]
    fn test_container_composable_indexing() {
        let mut registry = MemberRegistry::new(PathBuf::from("/tmp"));

        // Register container
        let container = Container {
            name: "test_container".to_string(),
            state: ProcessState::Pending,
            record: NodeRecord {
                executable: "test_container".to_string(),
                package: Some("test_pkg".to_string()),
                name: Some("test_container".to_string()),
                namespace: Some("/".to_string()),
                exec_name: None,
                params: vec![],
                params_files: vec![],
                remaps: vec![],
                ros_args: None,
                args: None,
                cmd: vec![],
                env: None,
                respawn: None,
                respawn_delay: None,
                global_params: None,
            },
            cmdline: NodeCommandLine {
                command: vec!["test".to_string()],
                user_args: vec![],
                remaps: HashMap::new(),
                params: HashMap::new(),
                params_files: std::collections::HashSet::new(),
                log_level: None,
                log_config_file: None,
                rosout_logs: None,
                stdout_logs: None,
                enclave: None,
                env: HashMap::new(),
            },
            output_dir: PathBuf::from("/tmp"),
            log_paths: NodeLogPaths {
                stdout: PathBuf::from("/tmp/out"),
                stderr: PathBuf::from("/tmp/err"),
                pid_file: PathBuf::from("/tmp/pid"),
                cmdline_file: PathBuf::from("/tmp/cmdline"),
                status_file: PathBuf::from("/tmp/status"),
                metrics_file: PathBuf::from("/tmp/metrics.csv"),
            },
            composable_nodes: vec![],
            respawn_enabled: false,
            respawn_delay: 0.0,
        };
        registry.register_container(container);

        // Register composable node
        let composable = ComposableNode {
            name: "test_composable".to_string(),
            state: ComposableState::Unloaded,
            container_name: "test_container".to_string(),
            record: ComposableNodeRecord {
                package: "test_pkg".to_string(),
                plugin: "test::Plugin".to_string(),
                target_container_name: "test_container".to_string(),
                node_name: "test_composable".to_string(),
                namespace: "/".to_string(),
                log_level: None,
                remaps: vec![],
                params: vec![],
                extra_args: HashMap::new(),
                env: None,
            },
            output_dir: PathBuf::from("/tmp"),
            log_paths: NodeLogPaths {
                stdout: PathBuf::from("/tmp/out"),
                stderr: PathBuf::from("/tmp/err"),
                pid_file: PathBuf::from("/tmp/pid"),
                cmdline_file: PathBuf::from("/tmp/cmdline"),
                status_file: PathBuf::from("/tmp/status"),
                metrics_file: PathBuf::from("/tmp/metrics.csv"),
            },
        };
        registry.register_composable_node(composable);

        // Test indexing
        assert_eq!(
            registry.get_container_for_composable("test_composable"),
            Some("test_container")
        );

        let composables = registry.get_composable_nodes_for_container("test_container");
        assert_eq!(composables.len(), 1);
        assert_eq!(composables[0], "test_composable");
    }

    #[test]
    fn test_block_unblock_composable() {
        let mut registry = MemberRegistry::new(PathBuf::from("/tmp"));

        let composable = ComposableNode {
            name: "test_composable".to_string(),
            state: ComposableState::Unloaded,
            container_name: "test_container".to_string(),
            record: ComposableNodeRecord {
                package: "test_pkg".to_string(),
                plugin: "test::Plugin".to_string(),
                target_container_name: "test_container".to_string(),
                node_name: "test_composable".to_string(),
                namespace: "/".to_string(),
                log_level: None,
                remaps: vec![],
                params: vec![],
                extra_args: HashMap::new(),
                env: None,
            },
            output_dir: PathBuf::from("/tmp"),
            log_paths: NodeLogPaths {
                stdout: PathBuf::from("/tmp/out"),
                stderr: PathBuf::from("/tmp/err"),
                pid_file: PathBuf::from("/tmp/pid"),
                cmdline_file: PathBuf::from("/tmp/cmdline"),
                status_file: PathBuf::from("/tmp/status"),
                metrics_file: PathBuf::from("/tmp/metrics.csv"),
            },
        };
        registry.register_composable_node(composable);

        // Block the node
        registry
            .block_composable_node("test_composable", BlockReason::Failed)
            .unwrap();

        let state = registry.get_composable_state("test_composable").unwrap();
        assert!(state.is_blocked());

        // Unblock the node
        registry.unblock_composable_node("test_composable").unwrap();

        let state = registry.get_composable_state("test_composable").unwrap();
        assert_eq!(state, ComposableState::Unloaded);
    }
}
