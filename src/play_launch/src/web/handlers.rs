//! HTTP handlers for the web API.

use super::WebState;
use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::{Html, IntoResponse, Response},
    Json,
};
use std::sync::Arc;

/// Helper to generate node card HTML
fn render_node_card(
    node: &crate::node_registry::NodeSummary,
    indent_class: &str,
) -> String {
    let status_class = match node.status {
        crate::node_registry::NodeStatus::Running => "status-running",
        crate::node_registry::NodeStatus::Stopped => "status-stopped",
        crate::node_registry::NodeStatus::Failed => "status-failed",
        crate::node_registry::NodeStatus::Pending => "status-pending",
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

    // Show Start or Stop button based on status
    let start_stop_button = if node.status == crate::node_registry::NodeStatus::Running {
        format!(
            r#"<button hx-post="/api/nodes/{}/stop" hx-swap="none" class="btn-stop">Stop</button>"#,
            node.name
        )
    } else {
        format!(
            r#"<button hx-post="/api/nodes/{}/start" hx-swap="none" class="btn-start">Start</button>"#,
            node.name
        )
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
    <button hx-post="/api/nodes/{}/restart" hx-swap="none" class="btn-restart">Restart</button>
    <button onclick="showNodeDetails('{}')" class="btn-details">Details</button>
    <button onclick="showNodeLogs('{}')" class="btn-logs">Logs</button>
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
        start_stop_button,
        node.name,
        node.name,
        node.name,
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
        let (containers, composables, regulars): (Vec<_>, Vec<_>, Vec<_>) = nodes
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
                    n.target_container.as_ref().map(|s| s.as_str()) == Some(container_full_name.as_str())
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
    let mut registry = state.registry.lock().await;

    match registry.start_node(&name).await {
        Ok(pid) => (
            StatusCode::OK,
            format!("Node '{}' started with PID {}", name, pid),
        )
            .into_response(),
        Err(e) => (
            StatusCode::BAD_REQUEST,
            format!("Failed to start node '{}': {}", name, e),
        )
            .into_response(),
    }
}

/// Stop a node
pub async fn stop_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    let mut registry = state.registry.lock().await;

    match registry.stop_node(&name) {
        Ok(true) => (StatusCode::OK, format!("Node '{}' stopped", name)).into_response(),
        Ok(false) => (StatusCode::OK, format!("Node '{}' was not running", name)).into_response(),
        Err(e) => (
            StatusCode::BAD_REQUEST,
            format!("Failed to stop node '{}': {}", name, e),
        )
            .into_response(),
    }
}

/// Restart a node
pub async fn restart_node(
    State(state): State<Arc<WebState>>,
    Path(name): Path<String>,
) -> Response {
    let mut registry = state.registry.lock().await;

    match registry.restart_node(&name).await {
        Ok(pid) => (
            StatusCode::OK,
            format!("Node '{}' restarted with PID {}", name, pid),
        )
            .into_response(),
        Err(e) => (
            StatusCode::BAD_REQUEST,
            format!("Failed to restart node '{}': {}", name, e),
        )
            .into_response(),
    }
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
  <span class="badge badge-total">Total: {}</span>
  <span class="badge badge-running">Running: {}</span>
  <span class="badge badge-stopped">Stopped: {}</span>
  <span class="badge badge-failed">Failed: {}</span>
  <span class="badge badge-pending">Pending: {}</span>
  <span class="badge badge-nodes">Nodes: {}</span>
  <span class="badge badge-containers">Containers: {}</span>
  <span class="badge badge-composable">Composable: {}</span>
  {}
</div>"#,
        summary.total,
        summary.running,
        summary.stopped,
        summary.failed,
        summary.pending,
        summary.nodes,
        summary.containers,
        summary.composable_nodes,
        noisy_badge
    );

    Html(html).into_response()
}
