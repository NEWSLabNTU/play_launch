//! HTTP handlers for the web API.

use super::WebState;
use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::{Html, IntoResponse, Response},
    Json,
};
use std::sync::Arc;

/// RAII guard for tracking operations in progress
struct OperationGuard {
    node_name: String,
    state: Arc<WebState>,
}

impl OperationGuard {
    /// Try to acquire operation lock for a node
    async fn try_acquire(node_name: String, state: Arc<WebState>) -> Result<Self, String> {
        {
            let mut ops = state.operations_in_progress.lock().await;
            if ops.contains(&node_name) {
                return Err(format!(
                    "Operation already in progress for node '{}'",
                    node_name
                ));
            }
            ops.insert(node_name.clone());
        } // Drop the lock here
        Ok(Self { node_name, state })
    }
}

impl Drop for OperationGuard {
    fn drop(&mut self) {
        // Remove from operations_in_progress when guard is dropped
        let node_name = self.node_name.clone();
        let state = self.state.clone();
        tokio::spawn(async move {
            let mut ops = state.operations_in_progress.lock().await;
            ops.remove(&node_name);
        });
    }
}

/// Helper to generate node card HTML
fn render_node_card(node: &crate::node_registry::NodeSummary, indent_class: &str) -> String {
    let status_class = match node.status {
        crate::node_registry::UnifiedStatus::Process(status) => match status {
            crate::node_registry::NodeStatus::Running => "status-running",
            crate::node_registry::NodeStatus::Stopped => "status-stopped",
            crate::node_registry::NodeStatus::Failed => "status-failed",
            crate::node_registry::NodeStatus::Pending => "status-pending",
        },
        crate::node_registry::UnifiedStatus::Composable(status) => match status {
            crate::node_registry::ComposableNodeStatus::Loaded => "status-loaded",
            crate::node_registry::ComposableNodeStatus::Failed => "status-failed",
            crate::node_registry::ComposableNodeStatus::Pending => "status-pending",
        },
    };

    // Use exec_name as primary name if available, otherwise fall back to directory name
    let display_name = node.exec_name.as_deref().unwrap_or(&node.name);

    // Construct full ROS node name (namespace + node_name)
    let ros_full_name = if let (Some(ns), Some(node_name)) = (&node.namespace, &node.node_name) {
        if ns == "/" {
            format!("/{}", node_name)
        } else if ns.ends_with('/') {
            format!("{}{}", ns, node_name)
        } else {
            format!("{}/{}", ns, node_name)
        }
    } else {
        String::new()
    };

    // Node type with CSS class for coloring
    let (node_type_label, node_type_class) = match node.node_type {
        crate::node_registry::NodeType::Node => ("node", "type-node"),
        crate::node_registry::NodeType::Container => ("container", "type-container"),
        crate::node_registry::NodeType::ComposableNode => ("composable", "type-composable"),
    };

    // Only show PID for regular nodes and containers, not composable nodes
    let pid_section = match node.node_type {
        crate::node_registry::NodeType::ComposableNode => String::new(),
        _ => {
            let pid_text = node
                .pid
                .map(|p| format!("PID: {}", p))
                .unwrap_or_else(|| "PID: -".to_string());
            format!(r#"<span class="node-pid">{}</span>"#, pid_text)
        }
    };

    // Control buttons based on node type
    let controls = match node.node_type {
        crate::node_registry::NodeType::ComposableNode => {
            // Only Details button for composable nodes
            format!(
                r#"<button onclick="showNodeDetails('{}')" class="btn-details">Details</button>"#,
                node.name
            )
        }
        crate::node_registry::NodeType::Container | crate::node_registry::NodeType::Node => {
            // Full controls: Start/Stop, Restart, Details, Logs
            let start_stop_button = match node.status {
                crate::node_registry::UnifiedStatus::Process(
                    crate::node_registry::NodeStatus::Running,
                ) => {
                    format!(
                        r#"<button hx-post="/api/nodes/{}/stop" hx-swap="none" hx-disabled-elt="closest .node-controls" class="btn-stop">
    <span class="btn-text">Stop</span>
    <span class="btn-loading">Stopping...</span>
</button>"#,
                        node.name
                    )
                }
                _ => {
                    format!(
                        r#"<button hx-post="/api/nodes/{}/start" hx-swap="none" hx-disabled-elt="closest .node-controls" class="btn-start">
    <span class="btn-text">Start</span>
    <span class="btn-loading">Starting...</span>
</button>"#,
                        node.name
                    )
                }
            };

            format!(
                r#"{}
    <button hx-post="/api/nodes/{}/restart" hx-swap="none" hx-disabled-elt="closest .node-controls" class="btn-restart">
        <span class="btn-text">Restart</span>
        <span class="btn-loading">Restarting...</span>
    </button>
    <button onclick="showNodeDetails('{}')" class="btn-details">Details</button>
    <button onclick="showNodeLogs('{}')" class="btn-logs">Logs</button>"#,
                start_stop_button, node.name, node.name, node.name
            )
        }
    };

    format!(
        r##"<div class="node-card {} {}" data-node="{}">
  <div class="node-info">
    <div class="node-header">
      <span class="node-type {}">{}</span>
      <span class="node-name">{}</span>
      {}
    </div>
    <div class="node-meta">
      <span class="node-ros-name">{}</span>
    </div>
  </div>
  <div class="node-controls">
    {}
  </div>
</div>
"##,
        status_class,
        indent_class,
        node.name,
        node_type_class,
        node_type_label,
        display_name,
        pid_section,
        ros_full_name,
        controls,
    )
}

/// List all nodes - returns HTML fragment for htmx
pub async fn list_nodes(State(state): State<Arc<WebState>>) -> Response {
    let registry = state.registry.lock().await;
    let nodes = registry.list_nodes();

    // Generate HTML fragment for htmx swap
    let mut html = String::new();

    if nodes.is_empty() {
        html.push_str(r#"<div class="no-nodes">No nodes registered</div>"#);
    } else {
        // Separate containers, composable nodes, and regular nodes
        let (containers, composables, regulars): (Vec<_>, Vec<_>, Vec<_>) =
            nodes
                .iter()
                .fold((Vec::new(), Vec::new(), Vec::new()), |mut acc, node| {
                    match node.node_type {
                        crate::node_registry::NodeType::Container => acc.0.push(node),
                        crate::node_registry::NodeType::ComposableNode => acc.1.push(node),
                        crate::node_registry::NodeType::Node => acc.2.push(node),
                    }
                    acc
                });

        // Render regular nodes first
        for node in &regulars {
            html.push_str(&render_node_card(node, ""));
        }

        // Render containers with their composable nodes
        for container in &containers {
            html.push_str(&render_node_card(container, "container-node"));

            // Construct the full ROS name for this container (namespace + name)
            let container_full_name = if let Some(ns) = &container.namespace {
                if ns == "/" {
                    format!("/{}", container.name)
                } else if ns.ends_with('/') {
                    format!("{}{}", ns, container.name)
                } else {
                    format!("{}/{}", ns, container.name)
                }
            } else {
                format!("/{}", container.name)
            };

            // Find composable nodes that belong to this container
            let children: Vec<_> = composables
                .iter()
                .filter(|n| {
                    n.target_container.as_ref().map(|s| s.as_str())
                        == Some(container_full_name.as_str())
                })
                .collect();

            // Render children with indentation
            for child in children {
                html.push_str(&render_node_card(child, "child-node"));
            }
        }
    }

    Html(html).into_response()
}

/// Get node details - returns JSON
pub async fn get_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    let registry = state.registry.lock().await;

    match registry.get_node_details(&name) {
        Some(details) => Json(details).into_response(),
        None => (StatusCode::NOT_FOUND, format!("Node '{}' not found", name)).into_response(),
    }
}

/// Start a node
pub async fn start_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    use crate::node_registry::NodeType;
    use tracing::info;

    // Acquire operation lock to prevent racing conditions
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => {
            return (StatusCode::CONFLICT, e).into_response();
        }
    };

    // Check if this is a container and remove termination markers for composable nodes
    let is_container = {
        let registry = state.registry.lock().await;
        registry
            .get(&name)
            .map(|h| h.node_type == NodeType::Container)
            .unwrap_or(false)
    };

    if is_container {
        // Get composable nodes before starting container
        let composable_nodes = {
            let registry = state.registry.lock().await;
            registry.get_container_composable_nodes(&name)
        };

        // Remove termination markers and write loading markers
        for comp_name in &composable_nodes {
            let registry = state.registry.lock().await;
            if let Some(handle) = registry.get(comp_name) {
                let terminated_path = handle.output_dir.join("terminated");
                if terminated_path.exists() {
                    let _ = std::fs::remove_file(&terminated_path);
                    info!(
                        "[Web UI] Removed termination marker for composable node '{}'",
                        comp_name
                    );
                }

                // Write loading marker to indicate loading is in progress
                let loading_path = handle.output_dir.join("loading");
                if let Err(e) = std::fs::write(&loading_path, "loading") {
                    info!(
                        "[Web UI] Failed to write loading marker for '{}': {}",
                        comp_name, e
                    );
                } else {
                    info!("[Web UI] Marked composable node '{}' as loading", comp_name);
                }
            }
        }
    }

    let mut registry = state.registry.lock().await;

    match registry.start_node(&name).await {
        Ok(pid) => {
            info!("[Web UI] Started node '{}' with PID {}", name, pid);
            (
                StatusCode::OK,
                format!("Node '{}' started with PID {}", name, pid),
            )
                .into_response()
        }
        Err(e) => {
            info!("[Web UI] Failed to start node '{}': {}", name, e);
            (
                StatusCode::BAD_REQUEST,
                format!("Failed to start node '{}': {}", name, e),
            )
                .into_response()
        }
    }
}

/// Stop a node
pub async fn stop_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    use crate::node_registry::NodeType;
    use tracing::info;

    // Acquire operation lock to prevent racing conditions
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => {
            return (StatusCode::CONFLICT, e).into_response();
        }
    };

    // Check if this is a container
    let is_container = {
        let registry = state.registry.lock().await;
        registry
            .get(&name)
            .map(|h| h.node_type == NodeType::Container)
            .unwrap_or(false)
    };

    let mut registry = state.registry.lock().await;

    match registry.stop_node(&name) {
        Ok(true) => {
            info!("[Web UI] Stopped node '{}'", name);

            // If it's a container, mark all composable nodes as terminated
            if is_container {
                let composable_nodes = registry.get_container_composable_nodes(&name);
                for comp_name in &composable_nodes {
                    if let Some(handle) = registry.get(comp_name) {
                        let terminated_path = handle.output_dir.join("terminated");
                        if let Err(e) = std::fs::write(&terminated_path, "container_stopped") {
                            info!(
                                "[Web UI] Failed to write termination marker for '{}': {}",
                                comp_name, e
                            );
                        } else {
                            info!(
                                "[Web UI] Marked composable node '{}' as terminated",
                                comp_name
                            );
                        }
                    }
                }
            }

            (StatusCode::OK, format!("Node '{}' stopped", name)).into_response()
        }
        Ok(false) => {
            info!("[Web UI] Node '{}' was not running", name);
            (StatusCode::OK, format!("Node '{}' was not running", name)).into_response()
        }
        Err(e) => {
            info!("[Web UI] Failed to stop node '{}': {}", name, e);
            (
                StatusCode::BAD_REQUEST,
                format!("Failed to stop node '{}': {}", name, e),
            )
                .into_response()
        }
    }
}

/// Restart a node
pub async fn restart_node(
    State(state): State<Arc<WebState>>,
    Path(name): Path<String>,
) -> Response {
    use crate::node_registry::NodeType;
    use tracing::info;

    // Acquire operation lock to prevent racing conditions
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => {
            return (StatusCode::CONFLICT, e).into_response();
        }
    };

    // Check if this is a container
    let is_container = {
        let registry = state.registry.lock().await;
        registry
            .get(&name)
            .map(|h| h.node_type == NodeType::Container)
            .unwrap_or(false)
    };

    if is_container && state.component_loader.is_some() {
        info!(
            "[Web UI] Restarting container '{}' (will reload composable nodes)",
            name
        );
        // Get composable nodes before restarting container
        let composable_nodes = {
            let registry = state.registry.lock().await;
            registry.get_container_composable_nodes(&name)
        };

        // Write loading markers for all composable nodes
        {
            let registry = state.registry.lock().await;
            for comp_name in &composable_nodes {
                if let Some(handle) = registry.get(comp_name) {
                    let loading_path = handle.output_dir.join("loading");
                    if let Err(e) = std::fs::write(&loading_path, "loading") {
                        info!(
                            "[Web UI] Failed to write loading marker for '{}': {}",
                            comp_name, e
                        );
                    } else {
                        info!("[Web UI] Marked composable node '{}' as loading", comp_name);
                    }
                }
            }
        }

        // Restart the container process
        let restart_result = {
            let mut registry = state.registry.lock().await;
            registry.restart_node(&name).await
        };

        match restart_result {
            Ok(pid) => {
                info!(
                    "[Web UI] Container '{}' restarted with PID {}, reloading {} composable nodes",
                    name,
                    pid,
                    composable_nodes.len()
                );

                // Reload composable nodes
                let mut reload_results = Vec::new();

                for comp_name in &composable_nodes {
                    match reload_composable_node(comp_name, &state).await {
                        Ok(_) => {
                            info!("[Web UI] Reloaded composable node '{}'", comp_name);
                            reload_results.push(format!("✓ {}", comp_name));
                        }
                        Err(e) => {
                            // Mark as failed but continue
                            info!(
                                "[Web UI] Failed to reload composable node '{}': {}",
                                comp_name, e
                            );
                            reload_results.push(format!("✗ {}: {}", comp_name, e));
                        }
                    }
                }

                let message = if reload_results.is_empty() {
                    format!("Container '{}' restarted with PID {}", name, pid)
                } else {
                    format!(
                        "Container '{}' restarted with PID {}\nReloaded composable nodes:\n{}",
                        name,
                        pid,
                        reload_results.join("\n")
                    )
                };

                (StatusCode::OK, message).into_response()
            }
            Err(e) => {
                info!("[Web UI] Failed to restart container '{}': {}", name, e);
                (
                    StatusCode::BAD_REQUEST,
                    format!("Failed to restart container '{}': {}", name, e),
                )
                    .into_response()
            }
        }
    } else {
        // Regular node restart
        info!("[Web UI] Restarting node '{}'", name);
        let mut registry = state.registry.lock().await;
        match registry.restart_node(&name).await {
            Ok(pid) => {
                info!("[Web UI] Restarted node '{}' with PID {}", name, pid);
                (
                    StatusCode::OK,
                    format!("Node '{}' restarted with PID {}", name, pid),
                )
                    .into_response()
            }
            Err(e) => {
                info!("[Web UI] Failed to restart node '{}': {}", name, e);
                (
                    StatusCode::BAD_REQUEST,
                    format!("Failed to restart node '{}': {}", name, e),
                )
                    .into_response()
            }
        }
    }
}

/// Reload a single composable node into its container
async fn reload_composable_node(node_name: &str, state: &Arc<WebState>) -> eyre::Result<()> {
    use crate::node_registry::{NodeInfo, NodeType};
    use eyre::{eyre, WrapErr};
    use std::time::Duration;

    // Get node info
    let record = {
        let registry = state.registry.lock().await;
        let handle = registry
            .get(node_name)
            .ok_or_else(|| eyre!("Node '{}' not found", node_name))?;

        if handle.node_type != NodeType::ComposableNode {
            return Err(eyre!("Node '{}' is not a composable node", node_name));
        }

        match &handle.info {
            NodeInfo::Composable(info) => info.record.clone(),
            _ => return Err(eyre!("Node '{}' has invalid info type", node_name)),
        }
    };

    // Get component loader
    let loader = state
        .component_loader
        .as_ref()
        .ok_or_else(|| eyre!("Component loader not available"))?;

    // Prepare remap rules
    let remap_rules: Vec<String> = record
        .remaps
        .iter()
        .map(|(from, to)| format!("{}:={}", from, to))
        .collect();

    // Prepare extra args
    let extra_args: Vec<(String, String)> = record
        .extra_args
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();

    // Load timeout (30 seconds)
    let timeout = Duration::from_secs(30);

    // Call loader service
    loader
        .load_node(
            &record.target_container_name,
            &record.package,
            &record.plugin,
            &record.node_name,
            &record.namespace,
            remap_rules,
            record.params.clone(),
            extra_args,
            timeout,
        )
        .await
        .wrap_err_with(|| format!("Failed to reload composable node '{}'", node_name))?;

    // Remove loading marker after successful load
    let registry = state.registry.lock().await;
    if let Some(handle) = registry.get(node_name) {
        let loading_path = handle.output_dir.join("loading");
        if loading_path.exists() {
            let _ = std::fs::remove_file(&loading_path);
        }
    }

    Ok(())
}

/// Health summary - returns HTML fragment with badges
pub async fn health_summary(State(state): State<Arc<WebState>>) -> Response {
    let registry = state.registry.lock().await;
    let summary = registry.get_health_summary();

    // Only show noisy badge if there are noisy nodes
    let noisy_badge = if summary.noisy > 0 {
        format!(
            r#"<span class="badge badge-noisy">Noisy: {}</span>"#,
            summary.noisy
        )
    } else {
        String::new()
    };

    let html = format!(
        r#"<div class="health-summary">
  <span class="badge badge-nodes">Nodes: {} running / {} total</span>
  <span class="badge badge-containers">Containers: {} running / {} total</span>
  <span class="badge badge-composable">Composable: {} loaded / {} total</span>
  <span class="badge badge-processes">Processes: {} running</span>
  {}
</div>"#,
        summary.nodes_running,
        summary.nodes_total,
        summary.containers_running,
        summary.containers_total,
        summary.composable_loaded,
        summary.composable_total,
        summary.processes_running,
        noisy_badge
    );

    Html(html).into_response()
}

/// Start all stopped nodes and containers
pub async fn start_all(State(state): State<Arc<WebState>>) -> Response {
    use crate::node_registry::{NodeStatus, NodeType, UnifiedStatus};
    use tracing::info;

    let mut registry = state.registry.lock().await;
    let nodes = registry.list_nodes();

    let mut results = Vec::new();
    let mut success_count = 0;
    let mut fail_count = 0;

    info!("[Web UI] Starting all stopped nodes/containers");

    for node in nodes {
        // Only start process-based nodes (not composable)
        if matches!(node.node_type, NodeType::Node | NodeType::Container) {
            // Check if stopped
            if matches!(
                node.status,
                UnifiedStatus::Process(NodeStatus::Stopped)
                    | UnifiedStatus::Process(NodeStatus::Failed)
            ) {
                // If it's a container, remove termination markers and write loading markers
                if node.node_type == NodeType::Container {
                    let composable_nodes = registry.get_container_composable_nodes(&node.name);
                    for comp_name in &composable_nodes {
                        if let Some(handle) = registry.get(comp_name) {
                            let terminated_path = handle.output_dir.join("terminated");
                            if terminated_path.exists() {
                                let _ = std::fs::remove_file(&terminated_path);
                                info!(
                                    "[Web UI] Removed termination marker for composable node '{}'",
                                    comp_name
                                );
                            }

                            // Write loading marker
                            let loading_path = handle.output_dir.join("loading");
                            if let Err(e) = std::fs::write(&loading_path, "loading") {
                                info!(
                                    "[Web UI] Failed to write loading marker for '{}': {}",
                                    comp_name, e
                                );
                            } else {
                                info!("[Web UI] Marked composable node '{}' as loading", comp_name);
                            }
                        }
                    }
                }

                match registry.start_node(&node.name).await {
                    Ok(pid) => {
                        info!("[Web UI] Started '{}' with PID {}", node.name, pid);
                        results.push(format!("✓ {}", node.name));
                        success_count += 1;
                    }
                    Err(e) => {
                        info!("[Web UI] Failed to start '{}': {}", node.name, e);
                        results.push(format!("✗ {}: {}", node.name, e));
                        fail_count += 1;
                    }
                }
            }
        }
    }

    let message = format!(
        "Started {} nodes ({} succeeded, {} failed)\n{}",
        success_count + fail_count,
        success_count,
        fail_count,
        results.join("\n")
    );

    (StatusCode::OK, message).into_response()
}

/// Stop all running nodes and containers
pub async fn stop_all(State(state): State<Arc<WebState>>) -> Response {
    use crate::node_registry::{NodeStatus, NodeType, UnifiedStatus};
    use tracing::info;

    let mut registry = state.registry.lock().await;
    let nodes = registry.list_nodes();

    let mut results = Vec::new();
    let mut success_count = 0;
    let mut fail_count = 0;

    info!("[Web UI] Stopping all running nodes/containers");

    for node in nodes {
        // Only stop process-based nodes (not composable)
        if matches!(node.node_type, NodeType::Node | NodeType::Container) {
            // Check if running
            if matches!(node.status, UnifiedStatus::Process(NodeStatus::Running)) {
                match registry.stop_node(&node.name) {
                    Ok(true) => {
                        info!("[Web UI] Stopped '{}'", node.name);
                        results.push(format!("✓ {}", node.name));
                        success_count += 1;
                    }
                    Ok(false) => {
                        // Already stopped, skip
                    }
                    Err(e) => {
                        info!("[Web UI] Failed to stop '{}': {}", node.name, e);
                        results.push(format!("✗ {}: {}", node.name, e));
                        fail_count += 1;
                    }
                }
            }
        }
    }

    let message = format!(
        "Stopped {} nodes ({} succeeded, {} failed)\n{}",
        success_count + fail_count,
        success_count,
        fail_count,
        results.join("\n")
    );

    (StatusCode::OK, message).into_response()
}

/// Restart all nodes and containers
pub async fn restart_all(State(state): State<Arc<WebState>>) -> Response {
    use crate::node_registry::NodeType;
    use tracing::info;

    let nodes = {
        let registry = state.registry.lock().await;
        registry.list_nodes()
    };

    let mut results = Vec::new();
    let mut success_count = 0;
    let mut fail_count = 0;

    info!("[Web UI] Restarting all nodes/containers");

    for node in nodes {
        // Only restart process-based nodes (not composable)
        if matches!(node.node_type, NodeType::Node | NodeType::Container) {
            let mut registry = state.registry.lock().await;
            match registry.restart_node(&node.name).await {
                Ok(pid) => {
                    info!("[Web UI] Restarted '{}' with PID {}", node.name, pid);
                    results.push(format!("✓ {}", node.name));
                    success_count += 1;
                }
                Err(e) => {
                    info!("[Web UI] Failed to restart '{}': {}", node.name, e);
                    results.push(format!("✗ {}: {}", node.name, e));
                    fail_count += 1;
                }
            }
        }
    }

    let message = format!(
        "Restarted {} nodes ({} succeeded, {} failed)\n{}",
        success_count + fail_count,
        success_count,
        fail_count,
        results.join("\n")
    );

    (StatusCode::OK, message).into_response()
}
