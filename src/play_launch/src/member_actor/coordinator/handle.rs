//! External handle for controlling and querying members

use super::{read_last_n_lines, MemberMetadata, NOISY_STDERR_THRESHOLD};
use crate::{
    member_actor::{
        events::ControlEvent,
        web_query::{HealthSummary, MemberState, MemberSummary, MemberType},
    },
    ros::parameter_types::{ParamEntry, ParamValue, SetParamResult},
};
use eyre::{Context, Result};
use std::{collections::HashMap, sync::Arc};
use tokio::sync::{mpsc, watch, RwLock};
use tracing::{debug, warn};

/// External handle for controlling and querying members
/// Wrap in Arc<MemberHandle> for sharing (not Clone-able itself)
pub struct MemberHandle {
    /// Control channels for sending commands to actors
    control_channels: HashMap<String, mpsc::Sender<ControlEvent>>,
    /// Member metadata (mutable for dynamic config updates like respawn_enabled)
    metadata: Arc<RwLock<HashMap<String, MemberMetadata>>>,
    /// Shared state map (actors write directly, Web UI reads)
    shared_state: Arc<dashmap::DashMap<String, MemberState>>,
    /// Shutdown signal broadcaster
    shutdown_tx: watch::Sender<bool>,
    /// Virtual member routing: maps composable node names to parent container names (Phase 12)
    virtual_member_routing: HashMap<String, String>,
    /// Shared ROS node for parameter service calls (Phase 24)
    shared_ros_node: Option<Arc<rclrs::Node>>,
    /// Member name → fully-qualified ROS node name (Phase 24)
    node_fqn_map: Arc<RwLock<HashMap<String, String>>>,
}

impl MemberHandle {
    /// Create a new MemberHandle (called from builder)
    pub(super) fn new(
        control_channels: HashMap<String, mpsc::Sender<ControlEvent>>,
        metadata: Arc<RwLock<HashMap<String, MemberMetadata>>>,
        shared_state: Arc<dashmap::DashMap<String, MemberState>>,
        shutdown_tx: watch::Sender<bool>,
        virtual_member_routing: HashMap<String, String>,
        shared_ros_node: Option<Arc<rclrs::Node>>,
        node_fqn_map: Arc<RwLock<HashMap<String, String>>>,
    ) -> Self {
        Self {
            control_channels,
            metadata,
            shared_state,
            shutdown_tx,
            virtual_member_routing,
            shared_ros_node,
            node_fqn_map,
        }
    }

    /// Get list of all members with current state
    pub async fn list_members(&self) -> Vec<MemberSummary> {
        let mut summaries = Vec::new();

        let metadata_guard = self.metadata.read().await;
        tracing::debug!(
            "list_members: metadata_guard has {} entries",
            metadata_guard.len()
        );
        for (name, meta) in metadata_guard.iter() {
            // Read state from DashMap
            let state = self
                .shared_state
                .get(name)
                .map(|entry| entry.value().clone())
                .unwrap_or(MemberState::Pending);

            let pid = match &state {
                MemberState::Running { pid } => Some(*pid),
                _ => None,
            };

            // Read stderr info from log files
            let stderr_path = meta.output_dir.join("err");
            let (stderr_last_modified, stderr_size, stderr_preview) =
                if let Ok(metadata) = std::fs::metadata(&stderr_path) {
                    let size = metadata.len();
                    let modified = metadata.modified().ok().and_then(|t| {
                        t.duration_since(std::time::UNIX_EPOCH)
                            .ok()
                            .map(|d| d.as_secs())
                    });

                    // Read last 5 lines for preview
                    let preview = if size > 0 {
                        read_last_n_lines(&stderr_path, 5).ok()
                    } else {
                        None
                    };

                    (modified, size, preview)
                } else {
                    (None, 0, None)
                };

            summaries.push(MemberSummary {
                name: meta.name.clone(),
                member_type: meta.member_type,
                state,
                pid,
                package: meta.package.clone(),
                executable: meta.executable.clone(),
                namespace: meta.namespace.clone(),
                target_container: meta.target_container.clone(),
                is_container: meta.member_type == MemberType::Container,
                exec_name: meta.exec_name.clone(),
                node_name: meta.node_name.clone(),
                stderr_last_modified,
                stderr_size,
                stderr_preview,
                respawn_enabled: meta.respawn_enabled,
                respawn_delay: meta.respawn_delay,
                auto_load: meta.auto_load,
                output_dir: meta.output_dir.clone(),
            });
        }

        summaries
    }

    /// Get detailed state for a specific member
    pub async fn get_member_state(&self, name: &str) -> Option<MemberSummary> {
        let metadata_guard = self.metadata.read().await;
        let meta = metadata_guard.get(name)?;

        // Read state from DashMap
        let state = self
            .shared_state
            .get(name)
            .map(|entry| entry.value().clone())
            .unwrap_or(MemberState::Pending);

        let pid = match &state {
            MemberState::Running { pid } => Some(*pid),
            _ => None,
        };

        // Read stderr info from log files
        let stderr_path = meta.output_dir.join("err");
        let (stderr_last_modified, stderr_size, stderr_preview) =
            if let Ok(metadata) = std::fs::metadata(&stderr_path) {
                let size = metadata.len();
                let modified = metadata.modified().ok().and_then(|t| {
                    t.duration_since(std::time::UNIX_EPOCH)
                        .ok()
                        .map(|d| d.as_secs())
                });

                // Read last 5 lines for preview
                let preview = if size > 0 {
                    read_last_n_lines(&stderr_path, 5).ok()
                } else {
                    None
                };

                (modified, size, preview)
            } else {
                (None, 0, None)
            };

        Some(MemberSummary {
            name: meta.name.clone(),
            member_type: meta.member_type,
            state,
            pid,
            package: meta.package.clone(),
            executable: meta.executable.clone(),
            namespace: meta.namespace.clone(),
            target_container: meta.target_container.clone(),
            is_container: meta.member_type == MemberType::Container,
            exec_name: meta.exec_name.clone(),
            node_name: meta.node_name.clone(),
            stderr_last_modified,
            stderr_size,
            stderr_preview,
            respawn_enabled: meta.respawn_enabled,
            respawn_delay: meta.respawn_delay,
            auto_load: meta.auto_load,
            output_dir: meta.output_dir.clone(),
        })
    }

    /// Get health summary statistics
    pub async fn get_health_summary(&self) -> HealthSummary {
        let mut summary = HealthSummary::default();

        let metadata_guard = self.metadata.read().await;
        for (name, meta) in metadata_guard.iter() {
            // Read state from DashMap
            let state = self
                .shared_state
                .get(name)
                .map(|entry| entry.value().clone())
                .unwrap_or(MemberState::Pending);

            // Count by member type
            match meta.member_type {
                MemberType::Node => {
                    summary.nodes_total += 1;
                    match &state {
                        MemberState::Running { .. } | MemberState::Respawning { .. } => {
                            summary.nodes_running += 1;
                            summary.processes_running += 1;
                        }
                        MemberState::Stopped => {
                            summary.nodes_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        MemberState::Failed { .. } => {
                            summary.nodes_failed += 1;
                            summary.processes_stopped += 1;
                        }
                        MemberState::Pending => {
                            // Pending means not started yet, don't count as stopped
                        }
                        _ => {}
                    }
                }
                MemberType::Container => {
                    summary.containers_total += 1;
                    match &state {
                        MemberState::Running { .. } | MemberState::Respawning { .. } => {
                            summary.containers_running += 1;
                            summary.processes_running += 1;
                        }
                        MemberState::Stopped => {
                            summary.containers_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        MemberState::Failed { .. } => {
                            summary.containers_failed += 1;
                            summary.processes_stopped += 1;
                        }
                        MemberState::Pending => {
                            // Pending means not started yet, don't count as stopped
                        }
                        _ => {}
                    }
                }
                MemberType::ComposableNode => {
                    summary.composable_total += 1;
                    match &state {
                        MemberState::Loaded { .. } => {
                            summary.composable_loaded += 1;
                        }
                        MemberState::Failed { .. } => {
                            summary.composable_failed += 1;
                        }
                        MemberState::Pending | MemberState::Loading => {
                            summary.composable_pending += 1;
                        }
                        _ => {}
                    }
                }
            }

            // Count noisy nodes (stderr > 10KB)
            let stderr_path = meta.output_dir.join("err");
            if let Ok(metadata) = std::fs::metadata(&stderr_path) {
                if metadata.len() > NOISY_STDERR_THRESHOLD {
                    summary.noisy += 1;
                }
            }
        }

        summary
    }

    /// Send a control event to a specific actor
    ///
    /// Phase 12: For virtual members (composable nodes managed by containers),
    /// this translates and routes control events to the parent container.
    pub async fn send_control(&self, name: &str, event: ControlEvent) -> Result<()> {
        // Check if this is a virtual member (composable node)
        if let Some(parent_container) = self.virtual_member_routing.get(name) {
            // Translate control event for virtual member
            let translated_event = match event {
                ControlEvent::Start => ControlEvent::LoadComposable {
                    name: name.to_string(),
                },
                ControlEvent::Stop => ControlEvent::UnloadComposable {
                    name: name.to_string(),
                },
                ControlEvent::Restart => {
                    // For restart, send Unload followed by Load
                    // First unload
                    let control_tx =
                        self.control_channels.get(parent_container).ok_or_else(|| {
                            eyre::eyre!("Parent container not found: {}", parent_container)
                        })?;

                    control_tx
                        .send(ControlEvent::UnloadComposable {
                            name: name.to_string(),
                        })
                        .await
                        .context("Failed to send Unload to parent container")?;

                    // Then load
                    ControlEvent::LoadComposable {
                        name: name.to_string(),
                    }
                }
                ControlEvent::ToggleRespawn(_) => {
                    // Not supported for virtual members
                    warn!("ToggleRespawn not supported for virtual member: {}", name);
                    return Ok(());
                }
                ControlEvent::Load => ControlEvent::LoadComposable {
                    name: name.to_string(),
                },
                ControlEvent::Unload => ControlEvent::UnloadComposable {
                    name: name.to_string(),
                },
                ControlEvent::ToggleAutoLoad(enabled) => {
                    // Update metadata and forward to container so it can update internal state
                    let mut metadata_guard = self.metadata.write().await;
                    if let Some(meta) = metadata_guard.get_mut(name) {
                        meta.auto_load = Some(enabled);
                    }
                    drop(metadata_guard);

                    // Forward to container with composable node name included
                    ControlEvent::ToggleComposableAutoLoad {
                        name: name.to_string(),
                        enabled,
                    }
                }
                other => {
                    // Forward other events as-is
                    other
                }
            };

            // Send to parent container
            let control_tx = self
                .control_channels
                .get(parent_container)
                .ok_or_else(|| eyre::eyre!("Parent container not found: {}", parent_container))?;

            control_tx
                .send(translated_event)
                .await
                .context("Failed to send translated control event to parent container")
        } else {
            // Regular member - send directly
            let control_tx = self
                .control_channels
                .get(name)
                .ok_or_else(|| eyre::eyre!("Actor not found: {}", name))?;

            control_tx
                .send(event)
                .await
                .context("Failed to send control event to actor")
        }
    }

    /// Start a member (send Start control event)
    pub async fn start_member(&self, name: &str) -> Result<()> {
        self.send_control(name, ControlEvent::Start).await
    }

    /// Stop a member (send Stop control event)
    pub async fn stop_member(&self, name: &str) -> Result<()> {
        self.send_control(name, ControlEvent::Stop).await
    }

    /// Restart a member (send Restart control event)
    pub async fn restart_member(&self, name: &str) -> Result<()> {
        self.send_control(name, ControlEvent::Restart).await
    }

    /// Toggle respawn for a member
    pub async fn toggle_respawn(&self, name: &str, enabled: bool) -> Result<()> {
        // Send control event to actor
        self.send_control(name, ControlEvent::ToggleRespawn(enabled))
            .await?;

        // Update metadata so web UI reflects the new state
        let mut metadata_guard = self.metadata.write().await;
        if let Some(meta) = metadata_guard.get_mut(name) {
            meta.respawn_enabled = Some(enabled);
        }

        Ok(())
    }

    /// Toggle auto-load for a composable node
    pub async fn toggle_auto_load(&self, name: &str, enabled: bool) -> Result<()> {
        // Send control event to actor
        self.send_control(name, ControlEvent::ToggleAutoLoad(enabled))
            .await?;

        // Update metadata so web UI reflects the new state
        let mut metadata_guard = self.metadata.write().await;
        if let Some(meta) = metadata_guard.get_mut(name) {
            if meta.member_type != MemberType::ComposableNode {
                return Err(eyre::eyre!("Auto-load only applies to composable nodes"));
            }
            meta.auto_load = Some(enabled);
        } else {
            return Err(eyre::eyre!("Member '{}' not found", name));
        }

        Ok(())
    }

    /// Request a composable node to load (retry loading)
    pub async fn load_member(&self, name: &str) -> Result<()> {
        // Immediately update shared_state to Loading for instant Web UI feedback
        self.shared_state
            .insert(name.to_string(), MemberState::Loading);

        self.send_control(name, ControlEvent::Load).await
    }

    /// Request a composable node to unload
    pub async fn unload_member(&self, name: &str) -> Result<()> {
        // Immediately update shared_state to Unloading for instant Web UI feedback
        self.shared_state
            .insert(name.to_string(), MemberState::Unloading);

        self.send_control(name, ControlEvent::Unload).await
    }

    /// Load all unloaded or failed composable nodes in a container
    pub async fn load_all_container_children(&self, container_name: &str) -> Result<usize> {
        // First verify the container exists and is a container
        let metadata_guard = self.metadata.read().await;
        if let Some(container_meta) = metadata_guard.get(container_name) {
            if container_meta.member_type != MemberType::Container {
                return Err(eyre::eyre!("'{}' is not a container", container_name));
            }
        } else {
            return Err(eyre::eyre!("Container '{}' not found", container_name));
        }

        // Use virtual_member_routing to find composable nodes that belong to this container
        // Count composable nodes that need loading (for return value)
        let mut nodes_to_load_count = 0;
        for (composable_name, parent_container) in self.virtual_member_routing.iter() {
            if parent_container == container_name {
                // Check if this node is in Unloaded or Failed state
                if let Some(entry) = self.shared_state.get(composable_name.as_str()) {
                    match entry.value() {
                        MemberState::Unloaded | MemberState::Failed { .. } => {
                            nodes_to_load_count += 1;
                        }
                        _ => {} // Skip nodes in other states
                    }
                }
            }
        }
        drop(metadata_guard); // Release lock before sending control

        // Send LoadAllComposables to container (Phase 12)
        self.send_control(container_name, ControlEvent::LoadAllComposables)
            .await?;

        Ok(nodes_to_load_count)
    }

    /// Unload all loaded composable nodes in a container
    pub async fn unload_all_container_children(&self, container_name: &str) -> Result<usize> {
        // First verify the container exists and is a container
        let metadata_guard = self.metadata.read().await;
        if let Some(container_meta) = metadata_guard.get(container_name) {
            if container_meta.member_type != MemberType::Container {
                return Err(eyre::eyre!("'{}' is not a container", container_name));
            }
        } else {
            return Err(eyre::eyre!("Container '{}' not found", container_name));
        }

        // Use virtual_member_routing to find composable nodes that belong to this container
        // Count composable nodes that are loaded (for return value)
        let mut nodes_to_unload_count = 0;
        for (composable_name, parent_container) in self.virtual_member_routing.iter() {
            if parent_container == container_name {
                // Check if this node is in Loaded state
                if let Some(entry) = self.shared_state.get(composable_name.as_str()) {
                    if let MemberState::Loaded { .. } = entry.value() {
                        nodes_to_unload_count += 1;
                    }
                }
            }
        }
        drop(metadata_guard); // Release lock before sending control

        // Send UnloadAllComposables to container (Phase 12)
        self.send_control(container_name, ControlEvent::UnloadAllComposables)
            .await?;

        Ok(nodes_to_unload_count)
    }

    /// Start all nodes and containers (not composable nodes - they're managed by containers)
    pub async fn start_all(&self) -> Result<usize> {
        let metadata_guard = self.metadata.read().await;
        let mut count = 0;

        for (name, meta) in metadata_guard.iter() {
            // Only send to regular nodes and containers
            if meta.member_type == MemberType::Node || meta.member_type == MemberType::Container {
                if let Some(tx) = self.control_channels.get(name) {
                    let _ = tx.send(ControlEvent::Start).await;
                    count += 1;
                }
            }
        }

        Ok(count)
    }

    /// Stop all nodes and containers (not composable nodes - they're managed by containers)
    pub async fn stop_all(&self) -> Result<usize> {
        let metadata_guard = self.metadata.read().await;
        let mut count = 0;

        for (name, meta) in metadata_guard.iter() {
            // Only send to regular nodes and containers
            if meta.member_type == MemberType::Node || meta.member_type == MemberType::Container {
                if let Some(tx) = self.control_channels.get(name) {
                    let _ = tx.send(ControlEvent::Stop).await;
                    count += 1;
                }
            }
        }

        Ok(count)
    }

    /// Restart all nodes and containers (not composable nodes - they're managed by containers)
    pub async fn restart_all(&self) -> Result<usize> {
        let metadata_guard = self.metadata.read().await;
        let mut count = 0;

        for (name, meta) in metadata_guard.iter() {
            // Only send to regular nodes and containers
            if meta.member_type == MemberType::Node || meta.member_type == MemberType::Container {
                if let Some(tx) = self.control_channels.get(name) {
                    let _ = tx.send(ControlEvent::Restart).await;
                    count += 1;
                }
            }
        }

        Ok(count)
    }

    /// Broadcast shutdown signal to all actors
    pub fn shutdown(&self) -> Result<()> {
        self.shutdown_tx
            .send(true)
            .context("Failed to send shutdown signal")
    }

    /// Get the number of registered members
    pub async fn member_count(&self) -> usize {
        self.metadata.read().await.len()
    }

    /// Check if a specific member is registered
    pub async fn has_member(&self, name: &str) -> bool {
        self.metadata.read().await.contains_key(name)
    }

    /// Get reference to the shared state map
    pub fn shared_state(&self) -> &Arc<dashmap::DashMap<String, MemberState>> {
        &self.shared_state
    }

    /// Get reference to the node FQN map (Phase 24)
    pub fn node_fqn_map(&self) -> &Arc<RwLock<HashMap<String, String>>> {
        &self.node_fqn_map
    }

    // --- Parameter operations (Phase 24) ---

    /// Get all parameters for a node. Creates a temporary ParameterProxy.
    pub async fn get_parameters(&self, member_name: &str) -> Result<Vec<ParamEntry>> {
        let ros_node = self
            .shared_ros_node
            .as_ref()
            .ok_or_else(|| eyre::eyre!("ROS node not available"))?;

        let fqn_map = self.node_fqn_map.read().await;
        let fqn = fqn_map
            .get(member_name)
            .ok_or_else(|| eyre::eyre!("Node '{}' not running or FQN unknown", member_name))?
            .clone();
        drop(fqn_map);

        debug!("Getting parameters for {} (FQN: {})", member_name, fqn);

        let proxy = crate::ros::parameter_proxy::ParameterProxy::new(ros_node, &fqn)
            .context("Failed to create parameter proxy")?;

        proxy.list_all().await
    }

    /// Set a single parameter on a node.
    pub async fn set_parameter(
        &self,
        member_name: &str,
        name: &str,
        value: ParamValue,
    ) -> Result<SetParamResult> {
        let ros_node = self
            .shared_ros_node
            .as_ref()
            .ok_or_else(|| eyre::eyre!("ROS node not available"))?;

        let fqn_map = self.node_fqn_map.read().await;
        let fqn = fqn_map
            .get(member_name)
            .ok_or_else(|| eyre::eyre!("Node '{}' not running or FQN unknown", member_name))?
            .clone();
        drop(fqn_map);

        debug!(
            "Setting parameter '{}' on {} (FQN: {})",
            name, member_name, fqn
        );

        let proxy = crate::ros::parameter_proxy::ParameterProxy::new(ros_node, &fqn)
            .context("Failed to create parameter proxy")?;

        proxy.set(name, value).await
    }

    // --- Graph introspection (Phase 25) ---

    /// Build a full graph snapshot of all topics and services.
    pub async fn build_graph_snapshot(
        &self,
    ) -> Result<crate::ros::graph_builder::GraphSnapshot> {
        let ros_node = self
            .shared_ros_node
            .as_ref()
            .ok_or_else(|| eyre::eyre!("ROS node not available"))?;

        // Build reverse map: FQN → member_name
        let fqn_map = self.node_fqn_map.read().await;
        let fqn_to_member: std::collections::HashMap<String, String> = fqn_map
            .iter()
            .map(|(member, fqn)| (fqn.clone(), member.clone()))
            .collect();
        drop(fqn_map);

        crate::ros::graph_builder::build_graph_snapshot(ros_node, &fqn_to_member)
    }

    /// Get topics and services for a single node.
    pub async fn get_node_topics(
        &self,
        member_name: &str,
    ) -> Result<crate::ros::graph_builder::NodeTopics> {
        let ros_node = self
            .shared_ros_node
            .as_ref()
            .ok_or_else(|| eyre::eyre!("ROS node not available"))?;

        let fqn_map = self.node_fqn_map.read().await;
        let fqn = fqn_map
            .get(member_name)
            .ok_or_else(|| eyre::eyre!("Node '{}' not running or FQN unknown", member_name))?
            .clone();
        drop(fqn_map);

        debug!("Getting topics for {} (FQN: {})", member_name, fqn);

        crate::ros::graph_builder::build_node_topics(ros_node, &fqn)
    }
}
