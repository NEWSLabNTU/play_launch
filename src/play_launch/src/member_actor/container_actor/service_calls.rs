//! ROS service call methods for LoadNode and UnloadNode.
//!
//! These methods handle creating and calling ROS 2 service clients
//! for loading and unloading composable nodes in a container.

use super::{
    ContainerActor, POST_SERVICE_READY_WARMUP, SERVICE_CALL_TIMEOUT,
    SERVICE_NOT_READY_LOG_INTERVAL, SERVICE_POLL_INTERVAL,
};
use crate::member_actor::container_control::{LoadNodeResponse, LoadParams};
use eyre::{Context as _, Result};
use tracing::{debug, warn};

impl ContainerActor {
    /// Call the LoadNode ROS service for a composable node.
    ///
    /// This is a standalone function so it can be spawned as an async task.
    pub(super) async fn call_load_node_service(
        container_name: String,
        load_client: Option<rclrs::Client<composition_interfaces::srv::LoadNode>>,
        params: LoadParams,
        start_time: std::time::Instant,
    ) -> Result<LoadNodeResponse> {
        // Check if client exists
        let client = load_client.ok_or_else(|| {
            eyre::eyre!(
                "No LoadNode service client available for container {}",
                container_name
            )
        })?;

        // Create per-node output directory for isolated container logging
        let output_dir = params.output_dir.clone();
        if !output_dir.as_os_str().is_empty() {
            if let Err(e) = std::fs::create_dir_all(&output_dir) {
                warn!(
                    "{}: Failed to create output dir for {}: {}",
                    container_name, params.composable_name, e
                );
            }
        }

        // Build LoadNode request
        let mut extra_arguments =
            crate::ros::parameter_conversion::convert_parameters_to_ros(&params.extra_args)?;

        // Inject log_dir so the isolated container can redirect stdout/stderr
        if !output_dir.as_os_str().is_empty() {
            extra_arguments.push(rcl_interfaces::msg::Parameter {
                name: "log_dir".to_string(),
                value: rcl_interfaces::msg::ParameterValue {
                    type_: rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
                    string_value: output_dir.to_string_lossy().to_string(),
                    bool_value: false,
                    integer_value: 0,
                    double_value: 0.0,
                    byte_array_value: Vec::new(),
                    bool_array_value: Vec::new(),
                    integer_array_value: Vec::new(),
                    double_array_value: Vec::new(),
                    string_array_value: Vec::new(),
                },
            });
        }

        let ros_request = composition_interfaces::srv::LoadNode_Request {
            package_name: params.package.clone(),
            plugin_name: params.plugin.clone(),
            node_name: params.node_name.clone(),
            node_namespace: params.node_namespace.clone(),
            log_level: 0, // Default log level
            remap_rules: params.remap_rules,
            parameters: crate::ros::parameter_conversion::convert_parameters_to_ros(
                &params.parameters,
            )?,
            extra_arguments,
        };

        // Wait for LoadNode service to be available
        // Container may have just started and service not yet registered
        debug!(
            "{}: Waiting for LoadNode service to be available for {}",
            container_name, params.composable_name
        );

        let service_wait_start = std::time::Instant::now();
        let service_timeout = SERVICE_CALL_TIMEOUT;
        let mut last_log_time = service_wait_start;

        loop {
            match client.service_is_ready() {
                Ok(true) => break,
                Ok(false) => {
                    // Service not ready yet, log periodically
                    if last_log_time.elapsed() > SERVICE_NOT_READY_LOG_INTERVAL {
                        debug!(
                            "{}: Still waiting for LoadNode service for {} ({}s elapsed)",
                            container_name,
                            params.composable_name,
                            service_wait_start.elapsed().as_secs()
                        );
                        last_log_time = std::time::Instant::now();
                    }
                }
                Err(e) => {
                    warn!(
                        "{}: Error checking service readiness for {}: {:?}",
                        container_name, params.composable_name, e
                    );
                    // Continue waiting despite error
                }
            }

            if service_wait_start.elapsed() > service_timeout {
                return Err(eyre::eyre!(
                    "LoadNode service not available after 30s (container may not have started properly)"
                ));
            }

            // Sleep briefly before checking again
            tokio::time::sleep(SERVICE_POLL_INTERVAL).await;
        }

        debug!(
            "{}: LoadNode service available after {}ms, calling service for {}/{} (node_name: {}, namespace: {})",
            container_name,
            service_wait_start.elapsed().as_millis(),
            params.package,
            params.plugin,
            params.node_name,
            params.node_namespace
        );

        // CRITICAL FIX: Add warmup delay after service becomes ready
        // Race condition: service_is_ready() returns true when service is registered in ROS graph,
        // but container executor may not be spinning/processing requests yet.
        // Without this delay, the service call can hang forever.
        // This is especially common after container restart when service registration happens
        // faster than executor startup.
        debug!(
            "{}: Waiting 200ms for container executor to start processing requests",
            container_name
        );
        tokio::time::sleep(POST_SERVICE_READY_WARMUP).await;

        // Phase 19.5b: Call service with 30s timeout (safety net -- ComponentEvent should arrive first)
        debug!(
            "{}: Initiating LoadNode service call for {}",
            container_name, params.composable_name
        );

        let response_future: rclrs::Promise<composition_interfaces::srv::LoadNode_Response> =
            client
                .call(&ros_request)
                .context("Failed to initiate LoadNode service call")?;

        debug!(
            "{}: Waiting for LoadNode response for {}",
            container_name, params.composable_name
        );

        let service_timeout = SERVICE_CALL_TIMEOUT;
        match tokio::time::timeout(service_timeout, response_future).await {
            Ok(Ok(response)) => {
                // Calculate timing metrics
                let service_call_ms = start_time.elapsed().as_millis() as u64;
                let queue_wait_ms =
                    start_time.duration_since(params.request_time).as_millis() as u64;
                let total_duration_ms = params.request_time.elapsed().as_millis() as u64;

                if response.success {
                    debug!(
                        "{}: LoadNode SUCCESS for {}: unique_id={}, queue={}ms, service={}ms, total={}ms",
                        container_name,
                        params.composable_name,
                        response.unique_id,
                        queue_wait_ms,
                        service_call_ms,
                        total_duration_ms
                    );
                } else {
                    warn!(
                        "{}: LoadNode FAILED for {}: error='{}', queue={}ms, service={}ms, total={}ms",
                        container_name,
                        params.composable_name,
                        response.error_message,
                        queue_wait_ms,
                        service_call_ms,
                        total_duration_ms
                    );
                }

                Ok(LoadNodeResponse {
                    success: response.success,
                    error_message: response.error_message.clone(),
                    full_node_name: response.full_node_name.clone(),
                    unique_id: response.unique_id,
                    timing: crate::member_actor::container_control::LoadTimingMetrics {
                        queue_wait_ms,
                        service_call_ms,
                        total_duration_ms,
                    },
                })
            }
            Ok(Err(e)) => {
                warn!(
                    "{}: LoadNode service call ERROR for {}: {:?}",
                    container_name, params.composable_name, e
                );
                Err(eyre::eyre!("Service call failed: {:?}", e))
            }
            Err(_) => {
                warn!(
                    "{}: LoadNode service call timed out after 30s for {}",
                    container_name, params.composable_name
                );
                Err(eyre::eyre!("LoadNode service call timed out after 30s"))
            }
        }
    }

    /// Call UnloadNode ROS service.
    pub(super) async fn call_unload_node_service(
        container_name: String,
        unload_client: Option<rclrs::Client<composition_interfaces::srv::UnloadNode>>,
        unique_id: u64,
    ) -> Result<crate::member_actor::container_control::UnloadNodeResponse> {
        // Check if client exists
        let client = unload_client.ok_or_else(|| {
            eyre::eyre!(
                "No UnloadNode service client available for container {}",
                container_name
            )
        })?;

        // Build UnloadNode request
        let ros_request = composition_interfaces::srv::UnloadNode_Request { unique_id };

        debug!(
            "{}: Calling UnloadNode service (unique_id: {})",
            container_name, unique_id
        );

        // Call the service with timeout (DDS SHM port exhaustion can cause hangs)
        let response_future: rclrs::Promise<composition_interfaces::srv::UnloadNode_Response> =
            client
                .call(&ros_request)
                .context("Failed to initiate UnloadNode service call")?;

        let service_timeout = SERVICE_CALL_TIMEOUT;
        match tokio::time::timeout(service_timeout, response_future).await {
            Ok(Ok(response)) => {
                debug!(
                    "{}: UnloadNode response: success={}, error_message={}",
                    container_name, response.success, response.error_message
                );

                Ok(crate::member_actor::container_control::UnloadNodeResponse {
                    success: response.success,
                    error_message: response.error_message.clone(),
                })
            }
            Ok(Err(e)) => {
                warn!("{}: UnloadNode service call error: {:?}", container_name, e);
                Err(eyre::eyre!("Service call failed: {:?}", e))
            }
            Err(_) => {
                warn!(
                    "{}: UnloadNode service call timed out after {}s (unique_id: {})",
                    container_name,
                    service_timeout.as_secs(),
                    unique_id
                );
                Err(eyre::eyre!("UnloadNode service call timed out"))
            }
        }
    }

    /// Handle an UnloadNode request by spawning a service call task.
    pub(super) async fn handle_unload_request(
        &self,
        composable_name: String,
        unique_id: u64,
        response_tx: tokio::sync::oneshot::Sender<
            Result<crate::member_actor::container_control::UnloadNodeResponse>,
        >,
    ) {
        let container_name = self.name.clone();
        let unload_client = self.unload_client.clone();

        // Spawn service call as a task (so it doesn't block the actor)
        tokio::spawn(async move {
            let result =
                Self::call_unload_node_service(container_name, unload_client, unique_id).await;

            // Send response back
            if let Err(result) = response_tx.send(result) {
                warn!(
                    "Failed to send UnloadNode response for {}: {:?}",
                    composable_name, result
                );
            }
        });
    }
}
