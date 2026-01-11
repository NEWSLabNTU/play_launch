//! ListNodes query manager for verifying composable node states
//!
//! This module provides a centralized task that performs on-demand queries
//! of container ListNodes services to verify the loaded state of composable nodes.
//!
//! # Architecture
//!
//! - Centralized query manager (single task for all containers)
//! - Rate-limited queries (max 1 per container per 5 seconds by default)
//! - On-demand triggering (only queries when requested by actors)
//! - Broadcasts discovered nodes via StateEvent
//!
//! # Usage
//!
//! ```no_run
//! use play_launch::member_actor::list_nodes_manager::ListNodesManager;
//!
//! # async fn example() -> eyre::Result<()> {
//! // Create and spawn the manager
//! let manager = ListNodesManager::new(config, ros_node, state_tx, shutdown_rx);
//! tokio::spawn(async move {
//!     manager.run().await;
//! });
//!
//! // Actors send requests via StateEvent::ListNodesRequested
//! # Ok(())
//! # }
//! ```

use crate::{cli::config::ListNodesSettings, member_actor::events::StateEvent};
use eyre::{Context, Result};
use std::{
    collections::HashMap,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::sync::{mpsc, watch};
use tracing::{debug, error, warn};

/// Manager for ListNodes queries
pub struct ListNodesManager {
    /// Configuration settings
    config: ListNodesSettings,
    /// Shared ROS node
    ros_node: Arc<rclrs::Node>,
    /// ListNodes service clients per container
    clients: HashMap<String, Arc<rclrs::Client<composition_interfaces::srv::ListNodes>>>,
    /// Last query time per container (for rate limiting)
    last_query_time: HashMap<String, Instant>,
    /// Channel to receive state events (including ListNodesRequested)
    state_rx: mpsc::Receiver<StateEvent>,
    /// Channel to send state events (NodeDiscovered)
    state_tx: mpsc::Sender<StateEvent>,
    /// Shutdown signal
    shutdown_rx: watch::Receiver<bool>,
}

impl ListNodesManager {
    /// Create a new ListNodes manager
    pub fn new(
        config: ListNodesSettings,
        ros_node: Arc<rclrs::Node>,
        state_rx: mpsc::Receiver<StateEvent>,
        state_tx: mpsc::Sender<StateEvent>,
        shutdown_rx: watch::Receiver<bool>,
    ) -> Self {
        Self {
            config,
            ros_node,
            clients: HashMap::new(),
            last_query_time: HashMap::new(),
            state_rx,
            state_tx,
            shutdown_rx,
        }
    }

    /// Run the manager task
    pub async fn run(mut self) {
        debug!("ListNodesManager started");

        loop {
            tokio::select! {
                // Check for shutdown signal
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        debug!("ListNodesManager shutting down");
                        break;
                    }
                }

                // Process state events
                Some(event) = self.state_rx.recv() => {
                    if let StateEvent::ListNodesRequested { container_name, requester } = event {
                        self.handle_query_request(&container_name, &requester).await;
                    }
                }

                else => {
                    // Channel closed
                    debug!("ListNodesManager channel closed");
                    break;
                }
            }
        }

        debug!("ListNodesManager exited");
    }

    /// Handle a query request from an actor
    async fn handle_query_request(&mut self, container_name: &str, requester: &str) {
        // Check rate limit
        if let Some(&last_time) = self.last_query_time.get(container_name) {
            let elapsed = last_time.elapsed();
            let rate_limit_duration = Duration::from_secs(self.config.rate_limit_secs);

            if elapsed < rate_limit_duration {
                debug!(
                    "ListNodes query for '{}' rate-limited (last query {}ms ago, limit {}s). Request from '{}'.",
                    container_name,
                    elapsed.as_millis(),
                    self.config.rate_limit_secs,
                    requester
                );
                return;
            }
        }

        // Perform the query
        if let Err(e) = self.query_list_nodes(container_name).await {
            warn!(
                "ListNodes query failed for '{}' (requested by '{}'): {}",
                container_name, requester, e
            );
        } else {
            // Update rate limit tracker
            self.last_query_time
                .insert(container_name.to_string(), Instant::now());
        }
    }

    /// Query the ListNodes service for a container
    async fn query_list_nodes(&mut self, container_name: &str) -> Result<()> {
        debug!("Querying ListNodes for container '{}'", container_name);

        // Get or create service client
        let client = self.get_or_create_client(container_name).await?;

        // Create request
        let request = composition_interfaces::srv::ListNodes_Request::default();

        // Call service with timeout
        let call_future = client
            .call(&request)
            .context("Failed to initiate ListNodes service call")?;

        let timeout_duration = Duration::from_millis(self.config.call_timeout_ms);
        let response: composition_interfaces::srv::ListNodes_Response =
            match tokio::time::timeout(timeout_duration, call_future).await {
                Ok(Ok(response)) => response,
                Ok(Err(e)) => {
                    return Err(eyre::eyre!(
                        "ListNodes service call failed for '{}': {:?}",
                        container_name,
                        e
                    ));
                }
                Err(_) => {
                    return Err(eyre::eyre!(
                        "ListNodes service call timed out for '{}' ({}ms)",
                        container_name,
                        self.config.call_timeout_ms
                    ));
                }
            };

        // Process response
        let node_count = response.full_node_names.len();
        debug!(
            "ListNodes for '{}' returned {} nodes",
            container_name, node_count
        );

        // Broadcast discovered nodes
        for (full_node_name, unique_id) in response
            .full_node_names
            .iter()
            .zip(response.unique_ids.iter())
        {
            let event = StateEvent::NodeDiscovered {
                container_name: container_name.to_string(),
                full_node_name: full_node_name.clone(),
                unique_id: *unique_id,
            };

            if let Err(e) = self.state_tx.send(event).await {
                error!(
                    "Failed to send NodeDiscovered event for '{}': {}",
                    full_node_name, e
                );
            }
        }

        Ok(())
    }

    /// Get or create a service client for a container
    async fn get_or_create_client(
        &mut self,
        container_name: &str,
    ) -> Result<Arc<rclrs::Client<composition_interfaces::srv::ListNodes>>> {
        if let Some(client) = self.clients.get(container_name) {
            return Ok(Arc::clone(client));
        }

        // Create service name (container_name + "/_container/list_nodes")
        let service_name = format!("{}/_container/list_nodes", container_name);

        debug!(
            "Creating ListNodes service client for '{}' (service: '{}')",
            container_name, service_name
        );

        let client = self
            .ros_node
            .create_client::<composition_interfaces::srv::ListNodes>(&service_name)
            .context(format!(
                "Failed to create ListNodes client for '{}'",
                container_name
            ))?;

        let client = Arc::new(client);
        self.clients
            .insert(container_name.to_string(), Arc::clone(&client));

        Ok(client)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = ListNodesSettings::default();
        assert_eq!(config.rate_limit_secs, 5);
        assert_eq!(config.loading_timeout_secs, 30);
        assert_eq!(config.unloading_timeout_secs, 10);
        assert_eq!(config.call_timeout_ms, 5000);
    }
}
