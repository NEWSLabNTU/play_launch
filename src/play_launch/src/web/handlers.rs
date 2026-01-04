//! HTTP handlers for the web API (event-driven architecture).

use super::WebState;
use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::{Html, IntoResponse, Response},
    Json,
};
use serde_json::json;
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
        }
        Ok(Self { node_name, state })
    }
}

impl Drop for OperationGuard {
    fn drop(&mut self) {
        let node_name = self.node_name.clone();
        let state = self.state.clone();
        tokio::spawn(async move {
            let mut ops = state.operations_in_progress.lock().await;
            ops.remove(&node_name);
        });
    }
}

/// Helper to render clickable ROS namespace segments
fn render_clickable_ros_name(ros_name: &str) -> String {
    if ros_name.is_empty() {
        return String::new();
    }

    let segments: Vec<&str> = ros_name.split('/').filter(|s| !s.is_empty()).collect();
    if segments.is_empty() {
        return String::new();
    }

    let mut html = String::new();
    let mut current_path = String::new();

    if ros_name.starts_with('/') {
        html.push_str(
            r#"<span class="ns-segment" data-ns="/" onclick="filterByNamespace('/')">/</span>"#,
        );
        current_path.push('/');
    }

    for (i, segment) in segments.iter().enumerate() {
        if i > 0 || !ros_name.starts_with('/') {
            current_path.push('/');
        }
        current_path.push_str(segment);

        let escaped_segment = segment
            .replace('&', "&amp;")
            .replace('<', "&lt;")
            .replace('>', "&gt;")
            .replace('"', "&quot;");

        let escaped_path = current_path.replace('&', "&amp;").replace('"', "&quot;");

        if i > 0 {
            html.push_str(r#"<span class="ns-sep">/</span>"#);
        }

        let class = if i == segments.len() - 1 {
            "ns-segment ns-leaf"
        } else {
            "ns-segment"
        };

        html.push_str(&format!(
            r#"<span class="{}" data-ns="{}" onclick="filterByNamespace('{}')">{}</span>"#,
            class, escaped_path, escaped_path, escaped_segment
        ));
    }

    html
}

/// Helper to render a node card
fn render_node_card(node: &super::web_types::NodeSummary, indent_class: &str) -> String {
    use super::web_types::{
        ComposableBlockReason, ComposableNodeStatus, NodeStatus, UnifiedStatus,
    };

    let (status_class, block_reason_tooltip) = match &node.status {
        UnifiedStatus::Process(status) => {
            let class = match status {
                NodeStatus::Running => "status-running",
                NodeStatus::Stopped => "status-stopped",
                NodeStatus::Failed => "status-failed",
                NodeStatus::Pending => "status-pending",
            };
            (class, None)
        }
        UnifiedStatus::Composable(status) => {
            let (class, reason) = match status {
                ComposableNodeStatus::Loaded => ("status-loaded", None),
                ComposableNodeStatus::Loading => ("status-loading", None),
                ComposableNodeStatus::Failed => ("status-failed", None),
                ComposableNodeStatus::Pending => ("status-pending", None),
                ComposableNodeStatus::Blocked(reason) => {
                    let reason_text = match reason {
                        ComposableBlockReason::ContainerStopped => "Container stopped",
                        ComposableBlockReason::ContainerFailed => "Container failed",
                        ComposableBlockReason::ContainerNotStarted => "Container not started",
                        ComposableBlockReason::Shutdown => "Shutdown",
                    };
                    ("status-blocked", Some(reason_text))
                }
            };
            (class, reason)
        }
    };

    let title_attr = if let Some(reason) = block_reason_tooltip {
        format!(r#" title="Blocked: {}""#, reason)
    } else {
        String::new()
    };

    // Stderr activity indicator
    let stderr_indicator = if let Some(stderr_mtime) = node.stderr_last_modified {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let age_secs = now.saturating_sub(stderr_mtime);

        if age_secs < 10 && node.stderr_size > 0 {
            // Active (0-10s): yellow, jumping animation
            let preview = node
                .stderr_preview
                .as_ref()
                .map(|lines| lines.join("\n"))
                .unwrap_or_default();
            format!(
                r#"<span class="stderr-indicator active" title="Active stderr output (last 5 lines):&#10;{}">📋</span>"#,
                preview.replace('"', "&quot;")
            )
        } else if age_secs < 60 && node.stderr_size > 0 {
            // Recent (10-60s): orange, static
            let preview = node
                .stderr_preview
                .as_ref()
                .map(|lines| lines.join("\n"))
                .unwrap_or_default();
            format!(
                r#"<span class="stderr-indicator recent" title="Recent stderr output (last 5 lines):&#10;{}">📋</span>"#,
                preview.replace('"', "&quot;")
            )
        } else {
            String::new()
        }
    } else {
        String::new()
    };

    let pid_display = node
        .pid
        .map(|p| format!("PID: {}{}", p, stderr_indicator))
        .unwrap_or_else(|| "No PID".to_string());

    let namespace_html = node
        .namespace
        .as_ref()
        .map(|ns| render_clickable_ros_name(ns))
        .unwrap_or_default();

    let target_container_html = node
        .target_container
        .as_ref()
        .map(|tc| {
            format!(
                r#"<div class="detail"><strong>Container:</strong> {}</div>"#,
                tc
            )
        })
        .unwrap_or_default();

    let respawn_html = if let Some(enabled) = node.respawn_enabled {
        if enabled {
            let delay_str = node
                .respawn_delay
                .map(|d| format!(" ({}s)", d))
                .unwrap_or_default();
            format!(
                r#"<div class="detail"><strong>Respawn:</strong> Enabled{}</div>"#,
                delay_str
            )
        } else {
            r#"<div class="detail"><strong>Respawn:</strong> Disabled</div>"#.to_string()
        }
    } else {
        String::new()
    };

    format!(
        r##"<div class="node-card {} {}" data-node="{}"{}>
  <div class="node-info">
    <div class="node-name">{}</div>
    <div class="node-details">
      <div class="detail"><strong>Package:</strong> {}</div>
      <div class="detail"><strong>Executable:</strong> {}</div>
      <div class="detail"><strong>Namespace:</strong> {}</div>
      {}
      <div class="detail"><strong>Status:</strong> {}</div>
      {}
    </div>
  </div>
  <div class="node-controls">
    <button class="btn btn-sm btn-success" hx-post="/api/nodes/{}/start" hx-swap="none">Start</button>
    <button class="btn btn-sm btn-danger" hx-post="/api/nodes/{}/stop" hx-swap="none">Stop</button>
    <button class="btn btn-sm btn-warning" hx-post="/api/nodes/{}/restart" hx-swap="none">Restart</button>
    <button class="btn btn-sm btn-primary" onclick="showNodeDetails('{}')">Details</button>
    <button class="btn btn-sm btn-secondary" onclick="showLogs('{}')">Logs</button>
  </div>
</div>"##,
        indent_class,
        status_class,
        node.name,
        title_attr,
        node.name,
        node.package.as_ref().unwrap_or(&"N/A".to_string()),
        node.executable,
        namespace_html,
        target_container_html,
        pid_display,
        respawn_html,
        node.name,
        node.name,
        node.name,
        node.name,
        node.name,
    )
}

/// List all nodes - returns HTML fragment for htmx
pub async fn list_nodes(State(state): State<Arc<WebState>>) -> Response {
    let coordinator = &state.member_handle;
    let nodes = coordinator.list_members().await;

    // Convert to the format expected by render_node_card
    let node_summaries: Vec<_> = nodes
        .iter()
        .map(super::web_types::NodeSummary::from_member_summary)
        .collect();

    let mut html = String::new();

    // Build parent-child relationships for rendering
    let mut containers: Vec<_> = node_summaries.iter().filter(|n| n.is_container).collect();
    containers.sort_by(|a, b| a.name.cmp(&b.name));

    let mut regular_nodes: Vec<_> = node_summaries
        .iter()
        .filter(|n| !n.is_container && n.target_container.is_none())
        .collect();
    regular_nodes.sort_by(|a, b| a.name.cmp(&b.name));

    // Render containers with their composable nodes
    for container in containers {
        html.push_str(&render_node_card(container, ""));

        // Render composable nodes under this container
        let mut composable_nodes: Vec<_> = node_summaries
            .iter()
            .filter(|n| {
                n.target_container
                    .as_ref()
                    .map(|tc| tc == &container.name)
                    .unwrap_or(false)
            })
            .collect();
        composable_nodes.sort_by(|a, b| a.name.cmp(&b.name));

        for comp in composable_nodes {
            html.push_str(&render_node_card(comp, "composable-indent"));
        }
    }

    // Render regular nodes
    for node in regular_nodes {
        html.push_str(&render_node_card(node, ""));
    }

    Html(html).into_response()
}

/// Get node details - returns JSON
pub async fn get_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    let coordinator = &state.member_handle;

    if let Some(member) = coordinator.get_member_state(&name).await {
        let node = super::web_types::NodeSummary::from_member_summary(&member);
        Json(json!({
            "name": node.name,
            "package": node.package,
            "executable": node.executable,
            "namespace": node.namespace,
            "target_container": node.target_container,
            "status": node.status,
            "pid": node.pid,
            "is_container": node.is_container,
            "respawn_enabled": node.respawn_enabled,
            "respawn_delay": node.respawn_delay,
        }))
        .into_response()
    } else {
        (StatusCode::NOT_FOUND, "Node not found").into_response()
    }
}

/// Start a node
pub async fn start_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => return (StatusCode::CONFLICT, e).into_response(),
    };

    let coordinator = &state.member_handle;
    match coordinator.start_member(&name).await {
        Ok(()) => {
            tracing::info!("[Web UI] Sent Start control to '{}'", name);
            (
                StatusCode::ACCEPTED,
                format!("Start request for '{}' sent", name),
            )
                .into_response()
        }
        Err(e) => {
            tracing::warn!("[Web UI] Failed to start '{}': {}", name, e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to start: {}", e),
            )
                .into_response()
        }
    }
}

/// Stop a node
pub async fn stop_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => return (StatusCode::CONFLICT, e).into_response(),
    };

    let coordinator = &state.member_handle;
    match coordinator.stop_member(&name).await {
        Ok(()) => {
            tracing::info!("[Web UI] Sent Stop control to '{}'", name);
            (
                StatusCode::ACCEPTED,
                format!("Stop request for '{}' sent", name),
            )
                .into_response()
        }
        Err(e) => {
            tracing::warn!("[Web UI] Failed to stop '{}': {}", name, e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to stop: {}", e),
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
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => return (StatusCode::CONFLICT, e).into_response(),
    };

    let coordinator = &state.member_handle;
    match coordinator.restart_member(&name).await {
        Ok(()) => {
            tracing::info!("[Web UI] Sent Restart control to '{}'", name);
            (
                StatusCode::ACCEPTED,
                format!("Restart request for '{}' sent", name),
            )
                .into_response()
        }
        Err(e) => {
            tracing::warn!("[Web UI] Failed to restart '{}': {}", name, e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to restart: {}", e),
            )
                .into_response()
        }
    }
}

/// Toggle respawn for a node
pub async fn toggle_respawn(
    State(state): State<Arc<WebState>>,
    Path((name, enabled_str)): Path<(String, String)>,
) -> Response {
    let enabled = match enabled_str.as_str() {
        "true" | "1" => true,
        "false" | "0" => false,
        _ => {
            return (
                StatusCode::BAD_REQUEST,
                format!(
                    "Invalid enabled value: '{}'. Use 'true' or 'false'",
                    enabled_str
                ),
            )
                .into_response();
        }
    };

    let coordinator = &state.member_handle;
    match coordinator.toggle_respawn(&name, enabled).await {
        Ok(()) => {
            tracing::info!(
                "[Web UI] Sent ToggleRespawn({}) control to '{}'",
                enabled,
                name
            );
            (
                StatusCode::ACCEPTED,
                format!("Respawn toggle request for '{}' sent", name),
            )
                .into_response()
        }
        Err(e) => {
            tracing::warn!("[Web UI] Failed to toggle respawn for '{}': {}", name, e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to toggle respawn: {}", e),
            )
                .into_response()
        }
    }
}

/// Health summary - returns HTML for htmx
pub async fn health_summary(State(state): State<Arc<WebState>>) -> Response {
    let coordinator = &state.member_handle;
    let summary = coordinator.get_health_summary().await;

    let html = format!(
        r#"<span class="badge badge-success">{} running</span>
           <span class="badge badge-secondary">{} total</span>
           <span class="badge badge-info">{} containers</span>
           <span class="badge badge-primary">{} composable</span>"#,
        summary.processes_running,
        summary.nodes_total + summary.containers_total,
        summary.containers_running,
        summary.composable_loaded,
    );

    Html(html).into_response()
}

// ===== Bulk Operations =====
// TODO(Phase 11.5): Implement bulk operations via coordinator

/// Start all nodes
/// TODO(Phase 11.5): Implement via coordinator.start_all()
pub async fn start_all(State(_state): State<Arc<WebState>>) -> Response {
    (
        StatusCode::NOT_IMPLEMENTED,
        "Bulk operations not yet implemented",
    )
        .into_response()
}

/// Stop all nodes
/// TODO(Phase 11.5): Implement via coordinator.stop_all()
pub async fn stop_all(State(_state): State<Arc<WebState>>) -> Response {
    (
        StatusCode::NOT_IMPLEMENTED,
        "Bulk operations not yet implemented",
    )
        .into_response()
}

/// Restart all nodes
/// TODO(Phase 11.5): Implement via coordinator.restart_all()
pub async fn restart_all(State(_state): State<Arc<WebState>>) -> Response {
    (
        StatusCode::NOT_IMPLEMENTED,
        "Bulk operations not yet implemented",
    )
        .into_response()
}
