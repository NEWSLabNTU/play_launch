//! HTTP handlers for the web API.

use super::WebState;
use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::{Html, IntoResponse, Response},
    Json,
};
use std::sync::Arc;

/// List all nodes - returns HTML fragment for htmx
pub async fn list_nodes(State(state): State<Arc<WebState>>) -> Response {
    let registry = state.registry.lock().await;
    let nodes = registry.list_nodes();

    // Generate HTML fragment for htmx swap
    let mut html = String::new();

    if nodes.is_empty() {
        html.push_str(r#"<div class="no-nodes">No nodes registered</div>"#);
    } else {
        for node in &nodes {
            let status_class = match node.status {
                crate::node_registry::NodeStatus::Running => "status-running",
                crate::node_registry::NodeStatus::Stopped => "status-stopped",
                crate::node_registry::NodeStatus::Failed => "status-failed",
                crate::node_registry::NodeStatus::Pending => "status-pending",
            };

            let status_text = format!("{:?}", node.status).to_lowercase();
            let node_type = format!("{:?}", node.node_type).to_lowercase();
            let pid_text = node
                .pid
                .map(|p| p.to_string())
                .unwrap_or_else(|| "-".to_string());

            let start_disabled = if node.status == crate::node_registry::NodeStatus::Running {
                "disabled"
            } else {
                ""
            };
            let stop_disabled = if node.status != crate::node_registry::NodeStatus::Running {
                "disabled"
            } else {
                ""
            };

            // Build HTML using multiple push_str calls to avoid raw string delimiter issues
            html.push_str(&format!(
                r##"<div class="node-card {}" data-node="{}">
  <div class="node-header">
    <span class="node-name">{}</span>
    <span class="node-type">{}</span>
  </div>
  <div class="node-info">
    <span class="node-status">{}</span>
    <span class="node-pid">PID: {}</span>
  </div>
  <div class="node-controls">
    <button hx-post="/api/nodes/{}/start" hx-swap="none" class="btn-start" {}>Start</button>
    <button hx-post="/api/nodes/{}/stop" hx-swap="none" class="btn-stop" {}>Stop</button>
    <button hx-post="/api/nodes/{}/restart" hx-swap="none" class="btn-restart">Restart</button>
    <button hx-get="/api/nodes/{}" hx-target="#node-details" class="btn-details">Details</button>
    <button class="btn-logs">Logs</button>
  </div>
</div>
"##,
                status_class,
                node.name,
                node.name,
                node_type,
                status_text,
                pid_text,
                node.name,
                start_disabled,
                node.name,
                stop_disabled,
                node.name,
                node.name,
            ));
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
  {}
</div>"#,
        summary.total,
        summary.running,
        summary.stopped,
        summary.failed,
        summary.pending,
        noisy_badge
    );

    Html(html).into_response()
}
