//! Container readiness checking module
//!
//! This module provides ROS service discovery for checking when containers are ready.
//! It uses rclrs native service discovery to check for `/_container/load_node` services.
//!
//! ## Usage
//!
//! Enable with `--wait-for-service-ready` flag. By default, only process-based checking is used
//! (see `execution::wait_for_containers_running()`), which is faster but less reliable for
//! complex systems like Autoware where DDS initialization takes significant time.
//!
//! ## Timeout Options
//!
//! - `--service-ready-timeout-secs 0`: Wait indefinitely until services are available
//! - `--service-ready-timeout-secs N`: Wait up to N seconds per container (default: 120)
//!
//! ## Performance Notes
//!
//! - DDS service discovery can be slow in large systems (several minutes for Autoware)
//! - Consider using longer timeouts or unlimited wait for production systems
//! - Process-based checking is faster but doesn't verify DDS readiness

use std::time::Duration;
use tokio::sync::{mpsc, oneshot};
use tracing::{debug, error, info, warn};

/// Configuration for waiting for container readiness
#[derive(Clone, Debug)]
pub struct ContainerWaitConfig {
    /// Maximum wait duration. None means unlimited wait.
    pub max_wait: Option<Duration>,
    pub poll_interval: Duration,
}

impl ContainerWaitConfig {
    /// Create a new config. If timeout_secs is 0, wait time is unlimited.
    pub fn new(timeout_secs: u64, poll_interval_ms: u64) -> Self {
        Self {
            max_wait: if timeout_secs == 0 {
                None
            } else {
                Some(Duration::from_secs(timeout_secs))
            },
            poll_interval: Duration::from_millis(poll_interval_ms),
        }
    }
}

/// Request to check if a container node is ready
#[derive(Debug)]
struct ContainerCheckRequest {
    container_name: String,
    response_tx: oneshot::Sender<bool>,
}

/// Handle for communicating with the ROS service discovery thread
#[derive(Clone, Debug)]
pub struct ServiceDiscoveryHandle {
    request_tx: mpsc::UnboundedSender<ContainerCheckRequest>,
}

impl ServiceDiscoveryHandle {
    /// Check if a container is ready by querying its services
    pub async fn check_container_ready(&self, container_name: String) -> eyre::Result<bool> {
        let (response_tx, response_rx) = oneshot::channel();

        self.request_tx
            .send(ContainerCheckRequest {
                container_name,
                response_tx,
            })
            .map_err(|_| eyre::eyre!("ROS discovery thread has stopped"))?;

        response_rx
            .await
            .map_err(|_| eyre::eyre!("Failed to receive response from ROS discovery thread"))
    }
}

/// Start the ROS service discovery thread
/// Returns a handle for querying service existence
pub fn start_service_discovery_thread() -> eyre::Result<ServiceDiscoveryHandle> {
    let (request_tx, request_rx) = mpsc::unbounded_channel();

    std::thread::Builder::new()
        .name("ros_discovery".to_string())
        .spawn(move || {
            if let Err(e) = run_ros_discovery_loop(request_rx) {
                error!("ROS discovery thread error: {}", e);
            }
        })?;

    Ok(ServiceDiscoveryHandle { request_tx })
}

/// Parse container full name into (node_name, namespace)
/// Examples:
/// - "/test_container" -> ("test_container", "/")
/// - "/system/container" -> ("container", "/system")
/// - "container" -> ("container", "/")
fn parse_container_name(full_name: &str) -> (&str, &str) {
    if full_name.is_empty() {
        return ("", "/");
    }

    // Ensure the name starts with /
    let normalized = if full_name.starts_with('/') {
        full_name
    } else {
        // If it doesn't start with /, it's in root namespace
        return (full_name, "/");
    };

    // Find the last slash to separate namespace from name
    if let Some(last_slash_idx) = normalized[1..].rfind('/') {
        // Found a slash after the leading slash
        let actual_idx = last_slash_idx + 1; // Adjust for starting search at index 1
        let namespace = &normalized[..actual_idx];
        let name = &normalized[actual_idx + 1..];
        (name, namespace)
    } else {
        // Only one slash at the beginning, so it's in root namespace
        let name = &normalized[1..];
        (name, "/")
    }
}

/// Run the ROS discovery loop in a dedicated thread
fn run_ros_discovery_loop(
    mut request_rx: mpsc::UnboundedReceiver<ContainerCheckRequest>,
) -> eyre::Result<()> {
    use rclrs::CreateBasicExecutor;

    debug!("Starting ROS container discovery thread");

    // Initialize ROS context and node (use empty args to avoid deprecation warnings)
    let context = rclrs::Context::new(
        vec!["play_launch".to_string()],
        rclrs::InitOptions::default(),
    )?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("play_launch_container_discovery")?;

    debug!("ROS container discovery node initialized");

    // Process requests
    while let Some(request) = request_rx.blocking_recv() {
        let ContainerCheckRequest {
            container_name,
            response_tx,
        } = request;

        // Parse container name into node name and namespace
        let (node_name, namespace) = parse_container_name(&container_name);

        debug!(
            "Checking container '{}' (node: '{}', ns: '{}')",
            container_name, node_name, namespace
        );

        // Query services for this specific node using per-node API
        let is_ready = match node.get_service_names_and_types_by_node(node_name, namespace) {
            Ok(services) => {
                debug!(
                    "Found {} services for container '{}' (node: '{}', ns: '{}')",
                    services.len(),
                    container_name,
                    node_name,
                    namespace
                );

                // Log all service names for debugging
                for service_name in services.keys() {
                    debug!("  Service: {}", service_name);
                }

                // Check if this node has the container services
                let has_list_nodes = services
                    .keys()
                    .any(|name| name.ends_with("/_container/list_nodes"));

                let has_load_node = services
                    .keys()
                    .any(|name| name.ends_with("/_container/load_node"));

                if has_list_nodes && has_load_node {
                    if crate::is_verbose() {
                        info!(
                            "Container '{}' is ready (has list_nodes and load_node services)",
                            container_name
                        );
                    }
                    true
                } else {
                    debug!(
                        "Container '{}' not ready yet (list_nodes: {}, load_node: {})",
                        container_name, has_list_nodes, has_load_node
                    );
                    false
                }
            }
            Err(e) => {
                warn!(
                    "Failed to query services for container '{}' (node: '{}', ns: '{}'): {}",
                    container_name, node_name, namespace, e
                );
                false
            }
        };

        // Send response (ignore if receiver dropped)
        let _ = response_tx.send(is_ready);
    }

    debug!("ROS container discovery thread shutting down");
    Ok(())
}

/// Wait for all containers to be ready by checking for their _container/load_node services
pub async fn wait_for_containers_ready(
    container_names: &[String],
    config: &ContainerWaitConfig,
    discovery_handle: &ServiceDiscoveryHandle,
) -> eyre::Result<()> {
    if container_names.is_empty() {
        return Ok(());
    }

    info!(
        "Waiting for {} container(s) to be ready...",
        container_names.len()
    );

    let mut tasks = vec![];

    for container_name in container_names {
        let name = container_name.clone();
        let cfg = config.clone();
        let handle = discovery_handle.clone();

        let task =
            tokio::spawn(async move { wait_for_single_container(&name, &cfg, &handle).await });

        tasks.push(task);
    }

    // Wait for all containers in parallel
    for (idx, task) in tasks.into_iter().enumerate() {
        match task.await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                warn!(
                    "Error waiting for container {}: {}",
                    container_names[idx], e
                );
            }
            Err(e) => {
                warn!(
                    "Task panicked for container {}: {}",
                    container_names[idx], e
                );
            }
        }
    }

    info!("All containers ready (or timed out)");
    Ok(())
}

/// Wait for a single container to be ready by checking its services via per-node query
async fn wait_for_single_container(
    container_name: &str,
    config: &ContainerWaitConfig,
    discovery_handle: &ServiceDiscoveryHandle,
) -> eyre::Result<()> {
    let deadline = config.max_wait.map(|d| tokio::time::Instant::now() + d);

    if let Some(max_wait) = config.max_wait {
        debug!(
            "Waiting up to {:?} for container '{}' to be ready",
            max_wait, container_name
        );
    } else {
        debug!(
            "Waiting indefinitely for container '{}' to be ready",
            container_name
        );
    }

    loop {
        // Check if the container is ready (has required services)
        match discovery_handle
            .check_container_ready(container_name.to_string())
            .await
        {
            Ok(true) => {
                if crate::is_verbose() {
                    info!("Container '{}' is ready", container_name);
                }
                return Ok(());
            }
            Ok(false) => {
                // Container not yet ready, continue polling
            }
            Err(e) => {
                warn!("Error checking container '{}': {}", container_name, e);
                // Continue polling despite error
            }
        }

        // Check timeout (if set)
        if let Some(deadline) = deadline {
            if tokio::time::Instant::now() >= deadline {
                warn!(
                    "Timeout waiting for container '{}' after {:?}, proceeding anyway",
                    container_name,
                    config.max_wait.unwrap()
                );
                return Ok(());
            }
        }

        tokio::time::sleep(config.poll_interval).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_container_wait_config() {
        let config = ContainerWaitConfig::new(10, 200);
        assert_eq!(config.max_wait, Some(Duration::from_secs(10)));
        assert_eq!(config.poll_interval, Duration::from_millis(200));
    }

    #[tokio::test]
    async fn test_container_wait_config_unlimited() {
        let config = ContainerWaitConfig::new(0, 200);
        assert_eq!(config.max_wait, None);
        assert_eq!(config.poll_interval, Duration::from_millis(200));
    }
}
