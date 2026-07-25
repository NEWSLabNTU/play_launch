//! ROS-side interface of a container (phase-51.4): the Load/List/Unload
//! service clients, the ComponentEvent subscription, ALL container service
//! name construction, and the service-call implementations.
//!
//! Grep gate: `format!("…/_container/…")` exists only here.

use super::timing::{
    LIST_VERIFY_TIMEOUT, LOAD_MAX_ATTEMPTS, LOAD_RETRY_TIMEOUT, LOAD_TOTAL_BUDGET,
    LOAD_VERIFY_POLL_INTERVAL, POST_SERVICE_READY_WARMUP, SERVICE_CALL_TIMEOUT,
    SERVICE_NOT_READY_LOG_INTERVAL, SERVICE_POLL_INTERVAL,
};
use crate::member_actor::container_control::{LoadNodeResponse, LoadParams};
use eyre::{Context as _, Result};
use rclrs::IntoPrimitiveOptions;
use std::sync::Arc;
use tracing::{debug, warn};

/// The per-kind suffix of a container's `_container` interfaces.
#[derive(Debug, Clone, Copy)]
pub(super) enum ServiceKind {
    LoadNode,
    ListNodes,
    UnloadNode,
    ComponentEvents,
}

impl ServiceKind {
    pub(super) fn suffix(self) -> &'static str {
        match self {
            ServiceKind::LoadNode => "/_container/load_node",
            ServiceKind::ListNodes => "/_container/list_nodes",
            ServiceKind::UnloadNode => "/_container/unload_node",
            ServiceKind::ComponentEvents => "/_container/component_events",
        }
    }
}

/// The ONE place a container service/topic name is built from a container's
/// fully-qualified node name.
pub(super) fn container_service(container_fqn: &str, kind: ServiceKind) -> String {
    format!("{}{}", container_fqn, kind.suffix())
}

/// Outcome of a post-timeout ListNodes verification.
enum VerifyOutcome {
    /// Component is present; carries its unique_id.
    Present(u64),
    /// Container answered and the component is definitively absent.
    Absent,
    /// Container did not answer (busy executor / blocked ctor) — unknown.
    Unavailable,
}

/// ROS clients + event subscription for one container. Created when the
/// container process starts, cleared on exit/restart so a respawn gets
/// fresh clients.
#[derive(Default)]
pub(super) struct ContainerClients {
    /// ROS service client for LoadNode
    pub(super) load_client: Option<rclrs::Client<composition_interfaces::srv::LoadNode>>,
    /// ListNodes client — verifies whether a timed-out LoadNode actually
    /// landed before retrying (LoadNode is not idempotent).
    pub(super) list_client: Option<rclrs::Client<composition_interfaces::srv::ListNodes>>,
    /// ROS service client for UnloadNode
    pub(super) unload_client: Option<rclrs::Client<composition_interfaces::srv::UnloadNode>>,
    /// ROS subscription for ComponentEvent messages (Phase 19.4a)
    pub(super) component_event_sub:
        Option<rclrs::Subscription<play_launch_msgs::msg::ComponentEvent>>,
    /// Receiver for ComponentEvent messages bridged from ROS callback
    pub(super) component_event_rx:
        Option<tokio::sync::mpsc::UnboundedReceiver<play_launch_msgs::msg::ComponentEvent>>,
}

impl ContainerClients {
    /// Whether the ComponentEvent subscription is live (play_launch
    /// containers) — the authoritative load-result path.
    pub(super) fn has_event_sub(&self) -> bool {
        self.component_event_sub.is_some()
    }

    /// Drop all clients and the event subscription so they're recreated on
    /// restart.
    pub(super) fn clear(&mut self) {
        self.load_client = None;
        self.list_client = None;
        self.unload_client = None;
        self.component_event_sub = None;
        self.component_event_rx = None;
    }

    /// Create the service clients (+ ComponentEvent subscription for
    /// play_launch containers) against a freshly started container.
    /// Failures are logged, not fatal — loads/unloads will surface errors.
    pub(super) fn create(
        &mut self,
        actor_name: &str,
        ros_node: &Arc<rclrs::Node>,
        container_fqn: &str,
        use_component_events: bool,
    ) {
        let load_name = container_service(container_fqn, ServiceKind::LoadNode);
        debug!(
            "{}: Creating LoadNode service client for {}",
            actor_name, load_name
        );
        match ros_node.create_client::<composition_interfaces::srv::LoadNode>(&load_name) {
            Ok(client) => {
                self.load_client = Some(client);
                debug!(
                    "{}: LoadNode service client created successfully",
                    actor_name
                );
            }
            Err(e) => {
                warn!(
                    "{}: Failed to create LoadNode service client: {:#}",
                    actor_name, e
                );
                // Don't fail the container - composable nodes will get errors when trying to load
            }
        }

        // ListNodes client (for post-timeout load verification)
        let list_name = container_service(container_fqn, ServiceKind::ListNodes);
        match ros_node.create_client::<composition_interfaces::srv::ListNodes>(&list_name) {
            Ok(client) => {
                self.list_client = Some(client);
            }
            Err(e) => {
                warn!(
                    "{}: Failed to create ListNodes service client: {:#}",
                    actor_name, e
                );
            }
        }

        let unload_name = container_service(container_fqn, ServiceKind::UnloadNode);
        debug!(
            "{}: Creating UnloadNode service client for {}",
            actor_name, unload_name
        );
        match ros_node.create_client::<composition_interfaces::srv::UnloadNode>(&unload_name) {
            Ok(client) => {
                self.unload_client = Some(client);
                debug!(
                    "{}: UnloadNode service client created successfully",
                    actor_name
                );
            }
            Err(e) => {
                warn!(
                    "{}: Failed to create UnloadNode service client: {:#}",
                    actor_name, e
                );
                // Don't fail the container - composable nodes will get errors when trying to unload
            }
        }

        // Phase 19.4a: ComponentEvent subscription — only when using
        // play_launch_container (not stock containers).
        if use_component_events {
            let event_topic = container_service(container_fqn, ServiceKind::ComponentEvents);
            debug!(
                "{}: Creating ComponentEvent subscription on {}",
                actor_name, event_topic
            );

            let (event_tx, event_rx) = tokio::sync::mpsc::unbounded_channel();
            match ros_node.create_subscription(
                event_topic
                    .as_str()
                    .reliable()
                    .transient_local()
                    .keep_last(100),
                move |msg: play_launch_msgs::msg::ComponentEvent| {
                    let _ = event_tx.send(msg);
                },
            ) {
                Ok(sub) => {
                    self.component_event_sub = Some(sub);
                    self.component_event_rx = Some(event_rx);
                    debug!(
                        "{}: ComponentEvent subscription created successfully",
                        actor_name
                    );
                }
                Err(e) => {
                    warn!(
                        "{}: Failed to create ComponentEvent subscription: {:#}",
                        actor_name, e
                    );
                }
            }
        } else {
            debug!(
                "{}: Skipping ComponentEvent subscription (stock container)",
                actor_name
            );
        }
    }
}

/// Call the LoadNode ROS service for a composable node.
///
/// Standalone so it can be spawned as an independent async task.
pub(super) async fn call_load_node_service(
    container_name: String,
    load_client: Option<rclrs::Client<composition_interfaces::srv::LoadNode>>,
    list_client: Option<rclrs::Client<composition_interfaces::srv::ListNodes>>,
    has_event_sub: bool,
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
    if !output_dir.as_os_str().is_empty()
        && let Err(e) = std::fs::create_dir_all(&output_dir)
    {
        warn!(
            "{}: Failed to create output dir for {}: {}",
            container_name, params.composable_name, e
        );
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

    // Attempt loop with post-timeout verification. A single-threaded
    // container executes composable ctors SERIALLY, so a later load's
    // response can legitimately exceed the flat timeout while the load
    // still succeeds server-side. LoadNode is NOT idempotent, so before
    // any retry we verify via ListNodes whether the component landed.
    let expected_full_name = if params.node_namespace == "/" {
        format!("/{}", params.node_name)
    } else {
        format!(
            "{}/{}",
            params.node_namespace.trim_end_matches('/'),
            params.node_name
        )
    };

    let mut attempt = 0usize;
    loop {
        attempt += 1;
        debug!(
            "{}: Initiating LoadNode service call for {} (attempt {}/{})",
            container_name, params.composable_name, attempt, LOAD_MAX_ATTEMPTS
        );

        let response_future: rclrs::Promise<composition_interfaces::srv::LoadNode_Response> =
            client
                .call(&ros_request)
                .context("Failed to initiate LoadNode service call")?;

        let this_timeout = if attempt == 1 {
            SERVICE_CALL_TIMEOUT
        } else {
            LOAD_RETRY_TIMEOUT
        };
        match tokio::time::timeout(this_timeout, response_future).await {
            Ok(Ok(response)) => {
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

                return Ok(LoadNodeResponse {
                    success: response.success,
                    error_message: response.error_message.clone(),
                    full_node_name: response.full_node_name.clone(),
                    unique_id: response.unique_id,
                    timing: crate::member_actor::container_control::LoadTimingMetrics {
                        queue_wait_ms,
                        service_call_ms,
                        total_duration_ms,
                    },
                });
            }
            Ok(Err(e)) => {
                warn!(
                    "{}: LoadNode service call ERROR for {}: {:?}",
                    container_name, params.composable_name, e
                );
                return Err(eyre::eyre!("Service call failed: {:?}", e));
            }
            Err(_) => {
                warn!(
                    "{}: LoadNode service call timed out after {}s for {} (attempt {}/{})",
                    container_name,
                    this_timeout.as_secs(),
                    params.composable_name,
                    attempt,
                    LOAD_MAX_ATTEMPTS
                );

                // play_launch containers (event sub present) are AUTHORITATIVE
                // via ComponentEvent: the isolated manager pre-assigns the id
                // and responds fast, and its ListNodes deliberately HIDES
                // mid-construction nodes — an Absent verdict here would lie
                // and a resend would double-load. Hand the outcome to the
                // event path instead (name-fallback matching + the stuck-
                // Loading promoter cover event loss).
                if has_event_sub {
                    warn!(
                        "{}: LoadNode response lost for {} — deferring to ComponentEvent",
                        container_name, params.composable_name
                    );
                    let service_call_ms = start_time.elapsed().as_millis() as u64;
                    let queue_wait_ms =
                        start_time.duration_since(params.request_time).as_millis() as u64;
                    return Ok(LoadNodeResponse {
                        success: true,
                        error_message: "LoadNode response lost; awaiting ComponentEvent"
                            .to_string(),
                        full_node_name: expected_full_name,
                        unique_id: 0,
                        timing: crate::member_actor::container_control::LoadTimingMetrics {
                            queue_wait_ms,
                            service_call_ms,
                            total_duration_ms: params.request_time.elapsed().as_millis() as u64,
                        },
                    });
                }

                // Poll verification until DEFINITIVE. A busy blocking-mode
                // container (multi-minute composable ctor, e.g. a TensorRT
                // engine load) can't answer ListNodes either — resending
                // LoadNode there would DOUBLE-LOAD once the ctor returns,
                // so on Unavailable we wait and poll, never resend.
                let verdict = loop {
                    match verify_component_loaded(
                        &container_name,
                        &list_client,
                        &expected_full_name,
                    )
                    .await
                    {
                        VerifyOutcome::Present(unique_id) => break Some(unique_id),
                        VerifyOutcome::Absent => break None,
                        VerifyOutcome::Unavailable => {
                            if start_time.elapsed() > LOAD_TOTAL_BUDGET {
                                return Err(eyre::eyre!(
                                    "container unresponsive for {}s while loading {} — \
                                     giving up without resend (avoids double-load)",
                                    LOAD_TOTAL_BUDGET.as_secs(),
                                    params.composable_name
                                ));
                            }
                            debug!(
                                "{}: container busy; re-polling ListNodes for {} ({}s elapsed)",
                                container_name,
                                params.composable_name,
                                start_time.elapsed().as_secs()
                            );
                            tokio::time::sleep(LOAD_VERIFY_POLL_INTERVAL).await;
                        }
                    }
                };

                if let Some(unique_id) = verdict {
                    warn!(
                        "{}: {} confirmed loaded via ListNodes after timeout (unique_id={})",
                        container_name, params.composable_name, unique_id
                    );
                    let service_call_ms = start_time.elapsed().as_millis() as u64;
                    let queue_wait_ms =
                        start_time.duration_since(params.request_time).as_millis() as u64;
                    return Ok(LoadNodeResponse {
                        success: true,
                        error_message: String::new(),
                        full_node_name: expected_full_name,
                        unique_id,
                        timing: crate::member_actor::container_control::LoadTimingMetrics {
                            queue_wait_ms,
                            service_call_ms,
                            total_duration_ms: params.request_time.elapsed().as_millis() as u64,
                        },
                    });
                }

                if attempt >= LOAD_MAX_ATTEMPTS {
                    return Err(eyre::eyre!(
                        "LoadNode timed out after {} attempts (container answered \
                         ListNodes but the component never appeared)",
                        LOAD_MAX_ATTEMPTS
                    ));
                }
                // Definitive absence — safe to resend.
            }
        }
    }
}

/// Post-timeout verification: tri-state — Present(unique_id) when the
/// container reports `expected_full_name`, Absent when it answered
/// without it, Unavailable when it could not answer (busy/blocked).
async fn verify_component_loaded(
    container_name: &str,
    list_client: &Option<rclrs::Client<composition_interfaces::srv::ListNodes>>,
    expected_full_name: &str,
) -> VerifyOutcome {
    let Some(client) = list_client.as_ref() else {
        return VerifyOutcome::Unavailable;
    };
    let request = composition_interfaces::srv::ListNodes_Request::default();
    let Ok(future) = client.call(&request) else {
        return VerifyOutcome::Unavailable;
    };
    let future: rclrs::Promise<composition_interfaces::srv::ListNodes_Response> = future;
    match tokio::time::timeout(LIST_VERIFY_TIMEOUT, future).await {
        Ok(Ok(response)) => match response
            .full_node_names
            .iter()
            .position(|n| n == expected_full_name)
            .and_then(|i| response.unique_ids.get(i).copied())
        {
            Some(id) => VerifyOutcome::Present(id),
            None => VerifyOutcome::Absent,
        },
        _ => {
            debug!(
                "{}: ListNodes verification unavailable for {}",
                container_name, expected_full_name
            );
            VerifyOutcome::Unavailable
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
    let response_future: rclrs::Promise<composition_interfaces::srv::UnloadNode_Response> = client
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
