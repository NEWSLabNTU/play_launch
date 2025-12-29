//! Event processor - handles all events and coordinates state changes
//!
//! This module implements the EventProcessor which is the heart of the event-driven architecture.
//! It receives events from the EventBus and coordinates state changes across Registry
//! and ProcessMonitor.
//!
//! # Status
//! Phase 2 implementation - event handlers now perform actual state changes

use super::{
    events::{EventBus, MemberEvent},
    member::{BlockReason, Member},
    process_monitor::ProcessMonitor,
    registry::Registry,
};
use eyre::{bail, Result};
use std::sync::Arc;
use tokio::sync::{mpsc, watch, Mutex};
use tracing::{debug, error, info, warn};

/// Event processor handles all events and coordinates state changes
///
/// # Design
/// - Receives events from EventBus
/// - Coordinates between Registry and ProcessMonitor
/// - Applies business logic for state transitions
/// - Publishes new events as side effects
pub struct EventProcessor {
    /// Member registry for state storage
    registry: Arc<Mutex<Registry>>,
    /// Event receiver (consumes events from EventBus)
    event_rx: mpsc::UnboundedReceiver<MemberEvent>,
    /// Event bus for publishing new events
    event_bus: EventBus,
    /// Process monitor for spawning and killing processes
    process_monitor: Arc<ProcessMonitor>,
    /// Shutdown signal transmitter
    shutdown_tx: watch::Sender<bool>,
    /// Shutdown signal receiver (for checking shutdown state)
    shutdown_rx: watch::Receiver<bool>,
    /// Process registry for resource monitoring
    process_registry:
        Option<Arc<std::sync::Mutex<std::collections::HashMap<u32, std::path::PathBuf>>>>,
    /// Process group ID for spawned processes
    pgid: Option<i32>,
    /// Component loader for loading composable nodes
    component_loader: Option<crate::ros::component_loader::ComponentLoaderHandle>,
    /// Service discovery handle for checking container readiness
    service_discovery: Option<crate::ros::container_readiness::ServiceDiscoveryHandle>,
}

impl EventProcessor {
    /// Create a new EventProcessor
    ///
    /// # Arguments
    /// - `registry`: Member registry for state storage
    /// - `event_rx`: Receiver for events from EventBus
    /// - `event_bus`: EventBus for publishing new events
    /// - `process_monitor`: ProcessMonitor for process management
    /// - `shutdown_tx`: Shutdown signal transmitter
    /// - `shutdown_rx`: Shutdown signal receiver
    /// - `process_registry`: Optional process registry for resource monitoring
    /// - `pgid`: Optional process group ID for spawned processes
    /// - `component_loader`: Optional component loader for loading composable nodes
    /// - `service_discovery`: Optional service discovery handle for container readiness checking
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        registry: Arc<Mutex<Registry>>,
        event_rx: mpsc::UnboundedReceiver<MemberEvent>,
        event_bus: EventBus,
        process_monitor: Arc<ProcessMonitor>,
        shutdown_tx: watch::Sender<bool>,
        shutdown_rx: watch::Receiver<bool>,
        process_registry: Option<
            Arc<std::sync::Mutex<std::collections::HashMap<u32, std::path::PathBuf>>>,
        >,
        pgid: Option<i32>,
        component_loader: Option<crate::ros::component_loader::ComponentLoaderHandle>,
        service_discovery: Option<crate::ros::container_readiness::ServiceDiscoveryHandle>,
    ) -> Self {
        Self {
            registry,
            event_rx,
            event_bus,
            process_monitor,
            shutdown_tx,
            shutdown_rx,
            process_registry,
            pgid,
            component_loader,
            service_discovery,
        }
    }

    /// Run the event processor loop
    ///
    /// This is the main event loop that processes events until shutdown or the channel closes.
    pub async fn run(mut self) {
        info!("Event processor starting");

        loop {
            tokio::select! {
                Some(event) = self.event_rx.recv() => {
                    if let Err(e) = self.process_event(event).await {
                        error!("Error processing event: {}", e);
                    }
                }
                _ = self.shutdown_tx.closed() => {
                    info!("Shutdown requested, stopping event processor");
                    break;
                }
                else => {
                    info!("Event channel closed, stopping event processor");
                    break;
                }
            }
        }

        info!("Event processor stopped");
    }

    /// Process a single event
    ///
    /// # Current Status
    /// Stub implementation - just logs the event
    async fn process_event(&mut self, event: MemberEvent) -> Result<()> {
        info!("Processing event: {:?}", event);

        match event {
            MemberEvent::ProcessStartRequested { name } => {
                self.handle_process_start_requested(&name).await?;
            }
            MemberEvent::ProcessStarted { name, pid } => {
                self.handle_process_started(&name, pid).await?;
            }
            MemberEvent::ProcessExited { name, exit_code } => {
                self.handle_process_exited(&name, exit_code).await?;
            }
            MemberEvent::ProcessStopped { name } => {
                self.handle_process_stopped(&name).await?;
            }
            MemberEvent::ProcessFailed { name, exit_code } => {
                self.handle_process_failed(&name, exit_code).await?;
            }
            MemberEvent::LoadRequested { name } => {
                self.handle_load_requested(&name).await?;
            }
            MemberEvent::LoadStarted { name } => {
                self.handle_load_started(&name).await?;
            }
            MemberEvent::LoadSucceeded {
                name,
                full_node_name,
                unique_id,
            } => {
                self.handle_load_succeeded(&name, &full_node_name, unique_id)
                    .await?;
            }
            MemberEvent::LoadFailed { name, error } => {
                self.handle_load_failed(&name, &error).await?;
            }
            MemberEvent::Blocked { name, reason } => {
                self.handle_blocked(&name, reason).await?;
            }
            MemberEvent::Unblocked { name } => {
                self.handle_unblocked(&name).await?;
            }
            MemberEvent::StartRequested { name } => {
                self.handle_start_requested(&name).await?;
            }
            MemberEvent::StopRequested { name } => {
                self.handle_stop_requested(&name).await?;
            }
            MemberEvent::RestartRequested { name } => {
                self.handle_restart_requested(&name).await?;
            }
            MemberEvent::RespawnToggled { name, enabled } => {
                self.handle_respawn_toggled(&name, enabled).await?;
            }
            MemberEvent::ShutdownRequested => {
                self.handle_shutdown_requested().await?;
            }
            MemberEvent::StateChanged { name } => {
                self.handle_state_changed(&name).await?;
            }
        }

        Ok(())
    }

    // ===== Event Handlers (Phase 2) =====

    async fn handle_process_start_requested(&mut self, name: &str) -> Result<()> {
        info!("Process start requested for {}", name);
        // This will be used in Phase 3 when integrating with execution.rs
        Ok(())
    }

    async fn handle_process_started(&mut self, name: &str, pid: u32) -> Result<()> {
        info!("Process {} started with PID {}", name, pid);

        // Update member state to Running
        {
            let mut registry = self.registry.lock().await;
            registry.transition_to_running(name, pid)?;
        }

        // Check if this is a container - if so, unblock composable nodes
        let member_type = {
            let registry = self.registry.lock().await;
            registry.get(name).map(|m| m.is_container())
        };

        if let Some(true) = member_type {
            info!("Container {} started, waiting for service readiness", name);

            // Get the container's ROS name for service discovery
            let container_ros_name = {
                let registry = self.registry.lock().await;
                registry.get(name).and_then(|m| match m {
                    Member::Container(c) => Some(c.record.name.clone().unwrap_or_default()),
                    _ => None,
                })
            };

            // Get composable nodes for this container
            let composable_nodes = {
                let registry = self.registry.lock().await;
                registry.get_composable_nodes_for_container(name)
            };

            // Clone necessary items for the async task
            let event_bus = self.event_bus.clone();
            let registry = self.registry.clone();
            let service_discovery = self.service_discovery.clone();
            let container_name = name.to_string();

            // Spawn a task to wait for container readiness and then load composable nodes
            tokio::spawn(async move {
                // Wait for container service to be ready (if service discovery is available)
                if let (Some(discovery), Some(ros_name)) = (service_discovery, container_ros_name) {
                    info!(
                        "Waiting for container {} (ROS name: {}) LoadNode service to be ready",
                        container_name, ros_name
                    );

                    // Wait for up to 120 seconds for the service to be ready
                    let config =
                        crate::ros::container_readiness::ContainerWaitConfig::new(120, 500);

                    match crate::ros::container_readiness::wait_for_containers_ready(
                        std::slice::from_ref(&ros_name),
                        &config,
                        &discovery,
                    )
                    .await
                    {
                        Ok(_) => {
                            info!("Container {} service is ready", container_name);
                        }
                        Err(e) => {
                            warn!(
                                "Error waiting for container {} service: {}",
                                container_name, e
                            );
                            // Continue anyway - the component loader will handle timeouts
                        }
                    }
                } else {
                    // No service discovery available, add a small delay to give container time to start
                    warn!(
                        "Service discovery not available for container {}, using fixed delay",
                        container_name
                    );
                    tokio::time::sleep(std::time::Duration::from_secs(2)).await;
                }

                // Now unblock and load all composable nodes for this container
                for node_name in &composable_nodes {
                    // Unblock the composable node
                    {
                        let mut reg = registry.lock().await;
                        if let Err(e) = reg.unblock_composable_node(node_name) {
                            error!("Failed to unblock composable node {}: {}", node_name, e);
                            continue;
                        }
                    }

                    // Publish Unblocked event
                    if let Err(e) = event_bus.publish(MemberEvent::Unblocked {
                        name: node_name.clone(),
                    }) {
                        error!("Failed to publish Unblocked event for {}: {}", node_name, e);
                        continue;
                    }

                    // Trigger loading for each composable node
                    if let Err(e) = event_bus.publish(MemberEvent::LoadRequested {
                        name: node_name.clone(),
                    }) {
                        error!(
                            "Failed to publish LoadRequested event for {}: {}",
                            node_name, e
                        );
                    }
                }

                info!(
                    "Triggered loading for {} composable nodes in container {}",
                    composable_nodes.len(),
                    container_name
                );
            });
        }

        // Publish state changed event
        self.event_bus.publish(MemberEvent::StateChanged {
            name: name.to_string(),
        })?;

        Ok(())
    }

    async fn handle_process_exited(&mut self, name: &str, exit_code: Option<i32>) -> Result<()> {
        info!("Process {} exited with exit code {:?}", name, exit_code);

        let is_failure = exit_code.unwrap_or(0) != 0;

        // Get PID before transitioning state (to unregister from monitoring)
        let (is_container, pid) = {
            let registry = self.registry.lock().await;
            let is_container = registry
                .get(name)
                .map(|m| m.is_container())
                .unwrap_or(false);
            let pid = registry.get_pid(name);
            (is_container, pid)
        };

        // Unregister PID from process monitoring registry
        if let (Some(ref process_registry), Some(pid)) = (&self.process_registry, pid) {
            if let Ok(mut reg) = process_registry.lock() {
                reg.remove(&pid);
                debug!("Unregistered PID {} for {}", pid, name);
            }
        }

        // Update member state
        if is_failure {
            let mut registry = self.registry.lock().await;
            registry.transition_to_failed(name, exit_code)?;
            drop(registry);
            self.event_bus.publish(MemberEvent::ProcessFailed {
                name: name.to_string(),
                exit_code,
            })?;
        } else {
            let mut registry = self.registry.lock().await;
            registry.transition_to_stopped(name)?;
            drop(registry);
            self.event_bus.publish(MemberEvent::ProcessStopped {
                name: name.to_string(),
            })?;
        }

        // If this is a container, block all composable nodes
        if is_container {
            let reason = if is_failure {
                BlockReason::Failed
            } else {
                BlockReason::Stopped
            };

            info!(
                "Container {} exited, blocking composable nodes with reason {:?}",
                name, reason
            );

            let composable_nodes = {
                let registry = self.registry.lock().await;
                registry.get_composable_nodes_for_container(name)
            };

            for node_name in &composable_nodes {
                let mut registry = self.registry.lock().await;
                registry.block_composable_node(node_name, reason)?;
                drop(registry);

                // Publish blocked event
                self.event_bus.publish(MemberEvent::Blocked {
                    name: node_name.clone(),
                    reason,
                })?;

                // Publish state changed event
                self.event_bus.publish(MemberEvent::StateChanged {
                    name: node_name.clone(),
                })?;
            }
        }

        // Publish state changed event for the process
        self.event_bus.publish(MemberEvent::StateChanged {
            name: name.to_string(),
        })?;

        // Handle respawn if enabled
        self.handle_respawn_if_enabled(name).await?;

        Ok(())
    }

    async fn handle_process_stopped(&mut self, name: &str) -> Result<()> {
        info!("Process {} stopped cleanly", name);
        // State already updated in handle_process_exited
        Ok(())
    }

    async fn handle_process_failed(&mut self, name: &str, exit_code: Option<i32>) -> Result<()> {
        warn!("Process {} failed with exit code {:?}", name, exit_code);
        // State already updated in handle_process_exited
        // Could add additional failure handling here (e.g., respawn logic)
        Ok(())
    }

    async fn handle_load_requested(&mut self, name: &str) -> Result<()> {
        info!("Load requested for composable node {}", name);

        // Get component loader
        let component_loader = match &self.component_loader {
            Some(loader) => loader.clone(),
            None => {
                warn!(
                    "Component loader not available, cannot load composable node {}",
                    name
                );
                self.event_bus.publish(MemberEvent::LoadFailed {
                    name: name.to_string(),
                    error: "Component loader not available".to_string(),
                })?;
                return Ok(());
            }
        };

        // Get composable node record from registry
        let (record, _output_dir) = {
            let registry = self.registry.lock().await;
            let member = registry
                .get(name)
                .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

            match member {
                Member::ComposableNode(node) => (node.record.clone(), node.output_dir.clone()),
                _ => {
                    bail!("Member {} is not a composable node", name);
                }
            }
        };

        // Update state to Loading
        {
            let mut registry = self.registry.lock().await;
            registry.transition_to_loading(name)?;
        }

        // Publish LoadStarted event
        self.event_bus.publish(MemberEvent::LoadStarted {
            name: name.to_string(),
        })?;

        // Publish state changed event
        self.event_bus.publish(MemberEvent::StateChanged {
            name: name.to_string(),
        })?;

        // Parameters are already strings (ParameterValue is a type alias for String)
        let parameters: Vec<(String, String)> = record
            .params
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();

        // Convert remaps to remap_rules
        let remap_rules: Vec<String> = record
            .remaps
            .iter()
            .map(|(from, to)| format!("{}:={}", from, to))
            .collect();

        // Convert extra_args
        let extra_args: Vec<(String, String)> = record
            .extra_args
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();

        // Spawn background task to load the node
        let event_bus = self.event_bus.clone();
        let name_clone = name.to_string();

        tokio::spawn(async move {
            info!("Calling LoadNode service for {}", name_clone);

            let result = component_loader
                .load_node(
                    &record.target_container_name,
                    &record.package,
                    &record.plugin,
                    &record.node_name,
                    &record.namespace,
                    remap_rules,
                    parameters,
                    extra_args,
                    std::time::Duration::from_secs(30),
                )
                .await;

            match result {
                Ok(response) => {
                    if response.success {
                        info!(
                            "LoadNode succeeded for {}: {}",
                            name_clone, response.full_node_name
                        );

                        // Publish success with response data
                        event_bus
                            .publish(MemberEvent::LoadSucceeded {
                                name: name_clone,
                                full_node_name: response.full_node_name,
                                unique_id: response.unique_id,
                            })
                            .ok();
                    } else {
                        warn!(
                            "LoadNode failed for {}: {}",
                            name_clone, response.error_message
                        );

                        // Publish failure with error message
                        event_bus
                            .publish(MemberEvent::LoadFailed {
                                name: name_clone,
                                error: response.error_message,
                            })
                            .ok();
                    }
                }
                Err(e) => {
                    error!("LoadNode service call failed for {}: {}", name_clone, e);
                    event_bus
                        .publish(MemberEvent::LoadFailed {
                            name: name_clone,
                            error: e.to_string(),
                        })
                        .ok();
                }
            }
        });

        Ok(())
    }

    async fn handle_load_started(&mut self, name: &str) -> Result<()> {
        info!("LoadNode service call started for {}", name);
        // State already updated to Loading in handle_load_requested
        Ok(())
    }

    async fn handle_load_succeeded(
        &mut self,
        name: &str,
        full_node_name: &str,
        unique_id: u64,
    ) -> Result<()> {
        info!(
            "Composable node {} loaded successfully: {}",
            name, full_node_name
        );

        // Get output directory for writing service response
        let output_dir = {
            let registry = self.registry.lock().await;
            let member = registry
                .get(name)
                .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

            match member {
                super::member::Member::ComposableNode(node) => node.output_dir.clone(),
                _ => bail!("Member {} is not a composable node", name),
            }
        };

        // Write service response file (synchronously in event loop)
        // Format matches traditional execution mode for compatibility with verification scripts
        let response_content = format!(
            "success: true\nerror_message: \nfull_node_name: {}\nunique_id: {}\n",
            full_node_name, unique_id
        );
        if let Err(e) = std::fs::write(output_dir.join("service_response"), response_content) {
            error!("Failed to write service response file: {}", e);
        }

        // Update state to Loaded
        {
            let mut registry = self.registry.lock().await;
            registry.transition_to_loaded(name)?;
        }

        // Publish state changed event
        self.event_bus.publish(MemberEvent::StateChanged {
            name: name.to_string(),
        })?;

        Ok(())
    }

    async fn handle_load_failed(&mut self, name: &str, error: &str) -> Result<()> {
        warn!("Failed to load composable node {}: {}", name, error);

        // Get output directory for writing service response
        let output_dir = {
            let registry = self.registry.lock().await;
            let member = registry
                .get(name)
                .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

            match member {
                super::member::Member::ComposableNode(node) => node.output_dir.clone(),
                _ => bail!("Member {} is not a composable node", name),
            }
        };

        // Write service response file (synchronously in event loop)
        // Format matches traditional execution mode for compatibility with verification scripts
        let response_content = format!(
            "success: false\nerror_message: {}\nfull_node_name: \nunique_id: 0\n",
            error
        );
        if let Err(e) = std::fs::write(output_dir.join("service_response"), response_content) {
            error!("Failed to write service response file: {}", e);
        }

        // Update state to Unloaded
        {
            let mut registry = self.registry.lock().await;
            registry.transition_to_unloaded(name)?;
        }

        // Publish state changed event
        self.event_bus.publish(MemberEvent::StateChanged {
            name: name.to_string(),
        })?;

        Ok(())
    }

    async fn handle_blocked(
        &mut self,
        name: &str,
        reason: super::member::BlockReason,
    ) -> Result<()> {
        info!("Composable node {} blocked: {:?}", name, reason);
        // State already updated in handle_process_exited
        Ok(())
    }

    async fn handle_unblocked(&mut self, name: &str) -> Result<()> {
        info!("Composable node {} unblocked", name);
        // State already updated in handle_process_started
        Ok(())
    }

    async fn handle_start_requested(&mut self, name: &str) -> Result<()> {
        info!("Start requested for {}", name);

        // Get member type to dispatch appropriately
        let member = {
            let registry = self.registry.lock().await;
            registry.get(name).cloned()
        };

        match member {
            Some(Member::Node(_)) | Some(Member::Container(_)) => {
                // For processes, spawn using event-driven architecture
                info!("Spawning process {}", name);

                // Spawn the process (Phase 3 integration)
                if let Err(e) = self.spawn_node_from_registry(name).await {
                    error!("Failed to spawn {}: {}", name, e);

                    // Publish failure event
                    self.event_bus.publish(MemberEvent::ProcessFailed {
                        name: name.to_string(),
                        exit_code: None,
                    })?;

                    return Err(e);
                }
            }
            Some(Member::ComposableNode(_)) => {
                // For composable nodes, publish LoadRequested
                self.event_bus.publish(MemberEvent::LoadRequested {
                    name: name.to_string(),
                })?;
            }
            None => {
                bail!("Member {} not found", name);
            }
        }

        Ok(())
    }

    async fn handle_stop_requested(&mut self, name: &str) -> Result<()> {
        info!("Stop requested for {}", name);

        // Get PID from member state
        let pid = {
            let registry = self.registry.lock().await;
            registry.get_pid(name)
        };

        match pid {
            Some(pid) => {
                // Use ProcessMonitor to gracefully shutdown
                self.process_monitor.shutdown_process(name, pid).await?;
                info!("Sent shutdown signal to process {} (PID {})", name, pid);
                // ProcessMonitor will publish ProcessExited event when process exits
            }
            None => {
                warn!("Cannot stop {}: not running or no PID", name);
            }
        }

        Ok(())
    }

    async fn handle_restart_requested(&mut self, name: &str) -> Result<()> {
        info!("Restart requested for {}", name);

        // Check if running
        let pid = {
            let registry = self.registry.lock().await;
            registry.get_pid(name)
        };

        // If running, stop first
        if let Some(pid) = pid {
            info!("Stopping {} before restart", name);
            self.process_monitor.shutdown_process(name, pid).await?;
            // Wait a bit for process to exit
            tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
        }

        // Then start
        self.event_bus.publish(MemberEvent::StartRequested {
            name: name.to_string(),
        })?;

        Ok(())
    }

    async fn handle_respawn_toggled(&mut self, name: &str, enabled: bool) -> Result<()> {
        info!("Respawn toggled for {} to {}", name, enabled);

        // Update respawn configuration in member
        let mut registry = self.registry.lock().await;
        let member = registry
            .get_mut(name)
            .ok_or_else(|| eyre::eyre!("Member {} not found", name))?;

        match member {
            Member::Node(node) => {
                node.respawn_enabled = enabled;
            }
            Member::Container(container) => {
                container.respawn_enabled = enabled;
            }
            Member::ComposableNode(_) => {
                warn!("Respawn not supported for composable nodes");
            }
        }

        Ok(())
    }

    async fn handle_shutdown_requested(&mut self) -> Result<()> {
        info!("Shutdown requested");

        // Get all running processes
        let running_members: Vec<(String, u32)> = {
            let registry = self.registry.lock().await;
            registry
                .names()
                .into_iter()
                .filter_map(|name| registry.get_pid(&name).map(|pid| (name, pid)))
                .collect()
        };

        // Shutdown all running processes
        for (name, pid) in running_members {
            info!("Shutting down {} (PID {})", name, pid);
            if let Err(e) = self.process_monitor.shutdown_process(&name, pid).await {
                error!("Failed to shutdown {}: {}", name, e);
            }
        }

        // Signal shutdown to event processor
        let _ = self.shutdown_tx.send(true);

        Ok(())
    }

    async fn handle_state_changed(&mut self, name: &str) -> Result<()> {
        // This event is primarily for web UI notifications
        // No additional action needed here
        info!("State changed for {}", name);
        Ok(())
    }

    // ========================================================================
    // Helper Methods (Phase 3)
    // ========================================================================

    /// Handle respawn if enabled for the given member
    async fn handle_respawn_if_enabled(&mut self, name: &str) -> Result<()> {
        // Check if shutdown is already signaled
        if *self.shutdown_rx.borrow() {
            debug!("Shutdown signaled, skipping respawn for {}", name);
            return Ok(());
        }

        // Get respawn configuration from registry
        let respawn_config = {
            let registry = self.registry.lock().await;
            match registry.get(name) {
                Some(super::member::Member::Node(node)) => {
                    if node.respawn_enabled {
                        Some((node.respawn_delay, node.name.clone()))
                    } else {
                        None
                    }
                }
                Some(super::member::Member::Container(container)) => {
                    if container.respawn_enabled {
                        Some((container.respawn_delay, container.name.clone()))
                    } else {
                        None
                    }
                }
                _ => None, // Composable nodes don't support respawn
            }
        };

        if let Some((delay_secs, member_name)) = respawn_config {
            info!("{} will respawn in {:.1}s", member_name, delay_secs);

            // Spawn a background task to handle delayed respawn
            let event_bus = self.event_bus.clone();
            let mut shutdown_rx = self.shutdown_rx.clone();

            tokio::spawn(async move {
                tokio::select! {
                    _ = tokio::time::sleep(std::time::Duration::from_secs_f64(delay_secs)) => {
                        // Delay complete, request start
                        if let Err(e) = event_bus.publish(super::events::MemberEvent::StartRequested {
                            name: member_name.clone(),
                        }) {
                            error!("Failed to publish StartRequested for {}: {}", member_name, e);
                        } else {
                            info!("Respawn triggered for {}", member_name);
                        }
                    }
                    _ = shutdown_rx.changed() => {
                        debug!("Shutdown signal received during respawn delay for {}", member_name);
                    }
                }
            });
        }

        Ok(())
    }

    /// Spawn a node from registry data (for respawn)
    ///
    /// This constructs a NodeContext from the stored NodeRecord and spawns the process.
    async fn spawn_node_from_registry(&self, name: &str) -> Result<()> {
        // Get node data from registry
        let context = {
            let registry = self.registry.lock().await;
            match registry.get(name) {
                Some(super::member::Member::Node(node)) => {
                    // Construct NodeContext from stored data
                    crate::execution::context::NodeContext {
                        record: node.record.clone(),
                        cmdline: node.cmdline.clone(),
                        output_dir: node.output_dir.clone(),
                    }
                }
                Some(super::member::Member::Container(container)) => {
                    // Construct NodeContext for container
                    crate::execution::context::NodeContext {
                        record: container.record.clone(),
                        cmdline: container.cmdline.clone(),
                        output_dir: container.output_dir.clone(),
                    }
                }
                _ => {
                    return Err(eyre::eyre!(
                        "Cannot spawn {}: not a regular node or container",
                        name
                    ));
                }
            }
        };

        // Spawn using the event-driven spawn function
        crate::execution::spawn::spawn_node_event_driven(
            &context,
            &self.process_monitor,
            &self.event_bus,
            self.process_registry.clone(),
            self.pgid,
        )
        .await?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        event_driven::member::{
            ComposableNode, ComposableState, Container, ProcessState, RegularNode,
        },
        execution::node_cmdline::NodeCommandLine,
        ros::launch_dump::NodeRecord,
        web::web_types::NodeLogPaths,
    };
    use std::{collections::HashMap, path::PathBuf};
    use tokio::sync::watch;

    fn make_test_node(name: &str) -> RegularNode {
        use std::collections::HashSet;

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
                params_files: HashSet::new(),
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

    fn make_test_container(name: &str) -> Container {
        use std::collections::HashSet;

        Container {
            name: name.to_string(),
            state: ProcessState::Pending,
            record: NodeRecord {
                executable: "test_container".to_string(),
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
                params_files: HashSet::new(),
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
        }
    }

    #[tokio::test]
    async fn test_event_processor_basic() {
        let (event_bus, event_rx) = EventBus::new();
        let (shutdown_tx, shutdown_rx) = watch::channel(false);
        let registry = Arc::new(Mutex::new(Registry::new(PathBuf::from("/tmp"))));
        let process_monitor = Arc::new(ProcessMonitor::new(event_bus.clone(), shutdown_rx.clone()));

        let processor = EventProcessor::new(
            registry.clone(),
            event_rx,
            event_bus.clone(),
            process_monitor,
            shutdown_tx.clone(),
            shutdown_rx.clone(),
            None, // process_registry
            None, // pgid
            None, // component_loader
            None, // service_discovery
        );

        // Register a test node
        {
            let mut reg = registry.lock().await;
            reg.register_node(make_test_node("test_node"));
        }

        // Spawn processor in background
        let processor_handle = tokio::spawn(async move {
            processor.run().await;
        });

        // Send a test event
        event_bus
            .publish(MemberEvent::StartRequested {
                name: "test_node".to_string(),
            })
            .unwrap();

        // Give it time to process
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Shutdown
        drop(shutdown_tx);

        // Wait for processor to finish
        processor_handle.await.unwrap();
    }

    #[tokio::test]
    async fn test_process_started_updates_state() {
        let (event_bus, event_rx) = EventBus::new();
        let (shutdown_tx, shutdown_rx) = watch::channel(false);
        let registry = Arc::new(Mutex::new(Registry::new(PathBuf::from("/tmp"))));
        let process_monitor = Arc::new(ProcessMonitor::new(event_bus.clone(), shutdown_rx.clone()));

        let processor = EventProcessor::new(
            registry.clone(),
            event_rx,
            event_bus.clone(),
            process_monitor,
            shutdown_tx.clone(),
            shutdown_rx.clone(),
            None, // process_registry
            None, // pgid
            None, // component_loader
            None, // service_discovery
        );

        // Register a test node
        {
            let mut reg = registry.lock().await;
            reg.register_node(make_test_node("test_node"));
        }

        // Spawn processor in background
        let processor_handle = tokio::spawn(async move {
            processor.run().await;
        });

        // Publish ProcessStarted event
        event_bus
            .publish(MemberEvent::ProcessStarted {
                name: "test_node".to_string(),
                pid: 1234,
            })
            .unwrap();

        // Give it time to process
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Check state was updated
        {
            let reg = registry.lock().await;
            let state = reg.get_process_state("test_node").unwrap();
            assert_eq!(state, ProcessState::Running { pid: 1234 });
        }

        // Shutdown
        drop(shutdown_tx);
        processor_handle.await.unwrap();
    }

    #[tokio::test]
    async fn test_container_exit_blocks_composable_nodes() {
        use crate::ros::launch_dump::ComposableNodeRecord;

        let (event_bus, event_rx) = EventBus::new();
        let (shutdown_tx, shutdown_rx) = watch::channel(false);
        let registry = Arc::new(Mutex::new(Registry::new(PathBuf::from("/tmp"))));
        let process_monitor = Arc::new(ProcessMonitor::new(event_bus.clone(), shutdown_rx.clone()));

        let processor = EventProcessor::new(
            registry.clone(),
            event_rx,
            event_bus.clone(),
            process_monitor,
            shutdown_tx.clone(),
            shutdown_rx.clone(),
            None, // process_registry
            None, // pgid
            None, // component_loader
            None, // service_discovery
        );

        // Register a container and composable node
        {
            let mut reg = registry.lock().await;
            reg.register_container(make_test_container("test_container"));
            reg.register_composable_node(ComposableNode {
                name: "test_composable".to_string(),
                state: ComposableState::Loaded,
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
            });

            // Start the container
            reg.transition_to_running("test_container", 1234).unwrap();
        }

        // Spawn processor in background
        let processor_handle = tokio::spawn(async move {
            processor.run().await;
        });

        // Publish ProcessExited event for container (failure)
        event_bus
            .publish(MemberEvent::ProcessExited {
                name: "test_container".to_string(),
                exit_code: Some(1),
            })
            .unwrap();

        // Give it time to process
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;

        // Check composable node was blocked
        {
            let reg = registry.lock().await;
            let state = reg.get_composable_state("test_composable").unwrap();
            assert!(state.is_blocked());
            match state {
                ComposableState::Blocked { reason } => {
                    assert_eq!(reason, BlockReason::Failed);
                }
                _ => panic!("Expected Blocked state"),
            }
        }

        // Shutdown
        drop(shutdown_tx);
        processor_handle.await.unwrap();
    }
}
