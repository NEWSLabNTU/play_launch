//! HTTP handlers for the web API (event-driven architecture).

use super::WebState;
use crate::diagnostics::{DiagnosticCounts, DiagnosticStatus};
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

/// Helper to clean node name by removing prefixes like "NODE 'name'" or "LOAD_NODE 'name'"
fn clean_node_name(name: &str) -> &str {
    // Check if name matches pattern: PREFIX 'actual_name'
    if let Some(quote_start) = name.find('\'') {
        if let Some(quote_end) = name.rfind('\'') {
            if quote_start < quote_end {
                return &name[quote_start + 1..quote_end];
            }
        }
    }
    name
}

/// Helper to render a node card
fn render_node_card(node: &super::web_types::NodeSummary, indent_class: &str) -> String {
    use super::web_types::{ComposableNodeStatus, NodeStatus, NodeType, UnifiedStatus};

    // Determine status class and block reason tooltip
    let status_class = match &node.status {
        UnifiedStatus::Process(status) => match status {
            NodeStatus::Running => "status-running",
            NodeStatus::Stopped => "status-stopped",
            NodeStatus::Failed => "status-failed",
            NodeStatus::Pending => "status-pending",
        },
        UnifiedStatus::Composable(status) => match status {
            ComposableNodeStatus::Loaded => "status-loaded",
            ComposableNodeStatus::Loading => "status-loading",
            ComposableNodeStatus::Unloading => "status-unloading",
            ComposableNodeStatus::Failed => "status-failed",
            ComposableNodeStatus::Pending => "status-pending",
            ComposableNodeStatus::Unloaded => "status-unloaded",
            ComposableNodeStatus::Blocked(_) => "status-blocked",
        },
    };

    // Use exec_name as primary name if available, otherwise clean the directory name
    let display_name = node
        .exec_name
        .as_deref()
        .unwrap_or_else(|| clean_node_name(&node.name));

    // Construct full ROS node name (namespace + node_name)
    let ros_full_name_plain =
        if let (Some(ns), Some(node_name)) = (&node.namespace, &node.node_name) {
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

    // Render ROS name with clickable segments
    let ros_full_name = render_clickable_ros_name(&ros_full_name_plain);

    // Node type with CSS class for coloring
    let (node_type_label, node_type_class) = match node.node_type {
        NodeType::Node => ("node", "type-node"),
        NodeType::Container => ("container", "type-container"),
        NodeType::ComposableNode => ("composable", "type-composable"),
    };

    // Only show PID for regular nodes and containers, not composable nodes
    let pid_section = match node.node_type {
        NodeType::ComposableNode => String::new(),
        _ => {
            let pid_text = node
                .pid
                .map(|p| format!("PID: {}", p))
                .unwrap_or_else(|| "PID: -".to_string());
            format!(r#"<span class="node-pid">{}</span>"#, pid_text)
        }
    };

    // Escape node name for JavaScript (escape single quotes and backslashes)
    let js_escaped_name = node.name.replace('\\', r"\\").replace('\'', r"\'");

    // Control buttons based on node type
    let controls = match node.node_type {
        NodeType::ComposableNode => {
            // Auto-load checkbox for composable nodes
            let auto_load_checkbox = if let Some(auto_load) = node.auto_load {
                let checked = if auto_load { "checked" } else { "" };
                format!(
                    r#"<label class="auto-load-checkbox" title="Auto-load when container starts">
        <input type="checkbox" {} onchange="toggleAutoLoad('{}', this.checked)">
        <span class="auto-load-label">Auto-load</span>
    </label>"#,
                    checked, js_escaped_name
                )
            } else {
                String::new()
            };

            // Load/Unload button for composable nodes
            let load_unload_button = match &node.status {
                UnifiedStatus::Composable(ComposableNodeStatus::Unloaded) => {
                    // Show Load button when unloaded
                    format!(
                        r#"<button hx-post="/api/nodes/{}/load" hx-swap="outerHTML" hx-target="closest .node-card" hx-disabled-elt="closest .node-controls" class="btn-load">
    <span class="btn-text">Load</span>
    <span class="btn-loading">Loading...</span>
</button>
    "#,
                        node.name
                    )
                }
                UnifiedStatus::Composable(ComposableNodeStatus::Failed) => {
                    // Show Load button when failed (can retry)
                    format!(
                        r#"<button hx-post="/api/nodes/{}/load" hx-swap="outerHTML" hx-target="closest .node-card" hx-disabled-elt="closest .node-controls" class="btn-load">
    <span class="btn-text">Load</span>
    <span class="btn-loading">Loading...</span>
</button>
    "#,
                        node.name
                    )
                }
                UnifiedStatus::Composable(ComposableNodeStatus::Loaded) => {
                    // Show Unload button when loaded
                    format!(
                        r#"<button hx-post="/api/nodes/{}/unload" hx-swap="outerHTML" hx-target="closest .node-card" hx-disabled-elt="closest .node-controls" class="btn-unload">
    <span class="btn-text">Unload</span>
    <span class="btn-loading">Unloading...</span>
</button>
    "#,
                        node.name
                    )
                }
                UnifiedStatus::Composable(ComposableNodeStatus::Loading) => {
                    // Show disabled Loading button
                    r#"<button disabled class="btn-load">
    <span class="btn-text">Loading...</span>
</button>
    "#
                    .to_string()
                }
                UnifiedStatus::Composable(ComposableNodeStatus::Unloading) => {
                    // Show disabled Unloading button
                    r#"<button disabled class="btn-unload">
    <span class="btn-text">Unloading...</span>
</button>
    "#
                    .to_string()
                }
                UnifiedStatus::Composable(ComposableNodeStatus::Blocked(_)) => {
                    // Show disabled Load button when blocked (container stopped)
                    r#"<button disabled class="btn-load">
    <span class="btn-text">Load</span>
</button>
    "#
                    .to_string()
                }
                UnifiedStatus::Composable(ComposableNodeStatus::Pending) => {
                    // Show disabled Load button when pending
                    r#"<button disabled class="btn-load">
    <span class="btn-text">Load</span>
</button>
    "#
                    .to_string()
                }
                _ => String::new(), // Should not reach here for composable nodes
            };

            format!(
                r#"{}
    {}
    <button onclick="showNodePanel('{}')" class="btn-view">View</button>"#,
                auto_load_checkbox, load_unload_button, js_escaped_name
            )
        }
        NodeType::Container | NodeType::Node => {
            // Full controls: Start/Stop, Restart, Details, Logs, Respawn checkbox
            let start_stop_button = match node.status {
                UnifiedStatus::Process(NodeStatus::Running) => {
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

            // Add "Load All" and "Unload All" buttons for containers when running
            let container_bulk_buttons = if node.node_type == NodeType::Container {
                match node.status {
                    UnifiedStatus::Process(NodeStatus::Running) => {
                        format!(
                            r#"
    <button hx-post="/api/nodes/{}/load-all" hx-swap="none" hx-disabled-elt="closest .node-controls" class="btn-load-all">
        <span class="btn-text">Load All</span>
        <span class="btn-loading">Loading...</span>
    </button>
    <button hx-post="/api/nodes/{}/unload-all" hx-swap="none" hx-disabled-elt="closest .node-controls" class="btn-unload-all">
        <span class="btn-text">Unload All</span>
        <span class="btn-loading">Unloading...</span>
    </button>"#,
                            node.name, node.name
                        )
                    }
                    _ => String::new(),
                }
            } else {
                String::new()
            };

            // Add respawn checkbox if respawn configuration is available
            let respawn_checkbox = if let Some(respawn_enabled) = node.respawn_enabled {
                let checked = if respawn_enabled { "checked" } else { "" };
                let delay_text = if let Some(delay) = node.respawn_delay {
                    if delay > 0.0 {
                        format!(" ({}s)", delay)
                    } else {
                        String::new()
                    }
                } else {
                    String::new()
                };
                format!(
                    r#"<label class="respawn-checkbox" title="Auto-restart when node exits">
        <input type="checkbox" {} onchange="toggleRespawn('{}', this.checked)">
        <span class="respawn-label">Auto-restart{}</span>
    </label>"#,
                    checked, js_escaped_name, delay_text
                )
            } else {
                String::new()
            };

            format!(
                r#"{}
    {}{}
    <button onclick="showNodePanel('{}')" class="btn-view">View</button>"#,
                respawn_checkbox, start_stop_button, container_bulk_buttons, js_escaped_name
            )
        }
    };

    // Format stderr data attributes (including preview as JSON)
    let stderr_attrs = if let Some(mtime) = node.stderr_last_modified {
        let preview_json = if let Some(ref lines) = node.stderr_preview {
            // Escape JSON string
            serde_json::to_string(lines).unwrap_or_else(|_| "[]".to_string())
        } else {
            "[]".to_string()
        };

        format!(
            r#" data-stderr-mtime="{}" data-stderr-size="{}" data-stderr-preview='{}'"#,
            mtime, node.stderr_size, preview_json
        )
    } else {
        String::new()
    };

    format!(
        r##"<div class="node-card {} {}" data-node="{}"{}>
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
        stderr_attrs,
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
    let coordinator = &state.member_handle;
    let nodes = coordinator.list_members().await;

    // Convert to the format expected by render_node_card
    let node_summaries: Vec<_> = nodes
        .iter()
        .map(super::web_types::NodeSummary::from_member_summary)
        .collect();

    let mut html = String::new();

    // Separate containers, composable nodes, and regular nodes by type
    let (containers, composables, regulars): (Vec<_>, Vec<_>, Vec<_>) =
        node_summaries
            .iter()
            .fold((Vec::new(), Vec::new(), Vec::new()), |mut acc, node| {
                match node.node_type {
                    super::web_types::NodeType::Container => acc.0.push(node),
                    super::web_types::NodeType::ComposableNode => acc.1.push(node),
                    super::web_types::NodeType::Node => acc.2.push(node),
                }
                acc
            });

    // Render regular nodes first
    for node in &regulars {
        html.push_str(&render_node_card(node, ""));
    }

    // Render containers with their composable nodes
    for container in &containers {
        html.push_str(&render_node_card(container, ""));

        // Construct the full ROS name for this container (namespace + node_name or name)
        // Containers typically have node_name=None, so we use the member name as the node name
        let container_full_name = if let Some(ns) = &container.namespace {
            let node_name = container.node_name.as_ref().unwrap_or(&container.name);
            if ns == "/" {
                format!("/{}", node_name)
            } else if ns.ends_with('/') {
                format!("{}{}", ns, node_name)
            } else {
                format!("{}/{}", ns, node_name)
            }
        } else {
            // No namespace: use just "/" + name
            format!("/{}", container.name)
        };

        tracing::debug!(
            "Container '{}': full_name='{}', namespace={:?}, node_name={:?}",
            container.name,
            container_full_name,
            container.namespace,
            container.node_name
        );

        // Find composable nodes that belong to this container
        let mut children: Vec<_> = composables
            .iter()
            .filter(|n| {
                let matches = n.target_container.as_deref() == Some(container_full_name.as_str());
                tracing::debug!(
                    "Composable '{}': target={:?}, matches '{}'? {}",
                    n.name,
                    n.target_container,
                    container_full_name,
                    matches
                );
                matches
            })
            .collect();
        children.sort_by(|a, b| a.name.cmp(&b.name));

        for child in children {
            html.push_str(&render_node_card(child, "child-node"));
        }
    }

    Html(html).into_response()
}

/// Get node details - returns JSON
pub async fn get_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    let coordinator = &state.member_handle;

    if let Some(member) = coordinator.get_member_state(&name).await {
        let node = super::web_types::NodeSummary::from_member_summary(&member);

        // Convert node_type to string for JSON
        let node_type_str = match node.node_type {
            super::web_types::NodeType::Node => "Node",
            super::web_types::NodeType::Container => "Container",
            super::web_types::NodeType::ComposableNode => "ComposableNode",
        };

        // For composable nodes, find the container's member name (not the full ROS name)
        let container_member_name = if node.node_type == super::web_types::NodeType::ComposableNode
        {
            if let Some(ref target_container_ros_name) = node.target_container {
                // Look up all members to find the container with matching ROS name
                let all_members = coordinator.list_members().await;
                all_members
                    .iter()
                    .find(|m| {
                        m.is_container
                            && m.namespace
                                .as_ref()
                                .map(|ns| {
                                    format!("{}/{}", ns, m.node_name.as_ref().unwrap_or(&m.name))
                                })
                                .or_else(|| m.node_name.as_ref().map(|n| format!("/{}", n)))
                                .as_ref()
                                == Some(target_container_ros_name)
                    })
                    .map(|container| container.name.clone())
            } else {
                None
            }
        } else {
            None
        };

        Json(json!({
            "name": node.name,
            "package": node.package,
            "executable": node.executable,
            "namespace": node.namespace,
            "target_container": node.target_container,
            "container_name": container_member_name,
            "status": node.status,
            "pid": node.pid,
            "is_container": node.is_container,
            "node_type": node_type_str,
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

/// Load a composable node (retry loading)
pub async fn load_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => return (StatusCode::CONFLICT, e).into_response(),
    };

    let coordinator = &state.member_handle;
    match coordinator.load_member(&name).await {
        Ok(()) => {
            tracing::info!("[Web UI] Sent Load control to '{}'", name);

            // Get the updated node state and render the card
            let nodes = coordinator.list_members().await;
            if let Some(node) = nodes.iter().find(|n| n.name == name) {
                let node_summary = super::web_types::NodeSummary::from_member_summary(node);
                let card_html = render_node_card(&node_summary, "child-node");
                Html(card_html).into_response()
            } else {
                (
                    StatusCode::ACCEPTED,
                    format!("Load request for '{}' sent", name),
                )
                    .into_response()
            }
        }
        Err(e) => {
            tracing::warn!("[Web UI] Failed to load '{}': {}", name, e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to load: {}", e),
            )
                .into_response()
        }
    }
}

/// Unload a composable node
pub async fn unload_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => return (StatusCode::CONFLICT, e).into_response(),
    };

    let coordinator = &state.member_handle;
    match coordinator.unload_member(&name).await {
        Ok(()) => {
            tracing::info!("[Web UI] Sent Unload control to '{}'", name);

            // Get the updated node state and render the card
            let nodes = coordinator.list_members().await;
            if let Some(node) = nodes.iter().find(|n| n.name == name) {
                let node_summary = super::web_types::NodeSummary::from_member_summary(node);
                let card_html = render_node_card(&node_summary, "child-node");
                Html(card_html).into_response()
            } else {
                (
                    StatusCode::ACCEPTED,
                    format!("Unload request for '{}' sent", name),
                )
                    .into_response()
            }
        }
        Err(e) => {
            tracing::warn!("[Web UI] Failed to unload '{}': {}", name, e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to unload: {}", e),
            )
                .into_response()
        }
    }
}

/// Load all unloaded/failed composable nodes in a container
pub async fn load_all_nodes(
    State(state): State<Arc<WebState>>,
    Path(name): Path<String>,
) -> Response {
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => return (StatusCode::CONFLICT, e).into_response(),
    };

    let coordinator = &state.member_handle;
    match coordinator.load_all_container_children(&name).await {
        Ok(count) => {
            tracing::info!(
                "[Web UI] Sent Load control to {} composable nodes in '{}'",
                count,
                name
            );
            (
                StatusCode::ACCEPTED,
                format!("Load request sent to {} nodes in '{}'", count, name),
            )
                .into_response()
        }
        Err(e) => {
            tracing::warn!("[Web UI] Failed to load nodes in '{}': {}", name, e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to load nodes: {}", e),
            )
                .into_response()
        }
    }
}

/// Unload all loaded composable nodes in a container
pub async fn unload_all_nodes(
    State(state): State<Arc<WebState>>,
    Path(name): Path<String>,
) -> Response {
    let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
        Ok(guard) => guard,
        Err(e) => return (StatusCode::CONFLICT, e).into_response(),
    };

    let coordinator = &state.member_handle;
    match coordinator.unload_all_container_children(&name).await {
        Ok(count) => {
            tracing::info!(
                "[Web UI] Sent Unload control to {} composable nodes in '{}'",
                count,
                name
            );
            (
                StatusCode::ACCEPTED,
                format!("Unload request sent to {} nodes in '{}'", count, name),
            )
                .into_response()
        }
        Err(e) => {
            tracing::warn!("[Web UI] Failed to unload nodes in '{}': {}", name, e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to unload nodes: {}", e),
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

/// Toggle auto-load for a composable node
pub async fn toggle_auto_load(
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
    match coordinator.toggle_auto_load(&name, enabled).await {
        Ok(()) => {
            tracing::info!("[Web UI] Toggled auto-load to {} for '{}'", enabled, name);
            (
                StatusCode::ACCEPTED,
                format!("Auto-load toggle request for '{}' sent", name),
            )
                .into_response()
        }
        Err(e) => {
            tracing::warn!("[Web UI] Failed to toggle auto-load for '{}': {}", name, e);
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("Failed to toggle auto-load: {}", e),
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
        r#"<span class="badge badge-nodes">{}/{} nodes</span>
           <span class="badge badge-containers">{}/{} containers</span>
           <span class="badge badge-composable">{}/{} composable</span>"#,
        summary.nodes_running,
        summary.nodes_total,
        summary.containers_running,
        summary.containers_total,
        summary.composable_loaded,
        summary.composable_total,
    );

    Html(html).into_response()
}

// ===== Bulk Operations =====

/// Start all nodes and containers
pub async fn start_all(State(state): State<Arc<WebState>>) -> Response {
    let coordinator = &state.member_handle;
    match coordinator.start_all().await {
        Ok(count) => {
            tracing::info!("[Web UI] Sent Start control to {} nodes/containers", count);
            (
                StatusCode::OK,
                format!("Started {} nodes/containers", count),
            )
                .into_response()
        }
        Err(e) => {
            tracing::error!("[Web UI] Failed to start all: {:#}", e);
            (StatusCode::INTERNAL_SERVER_ERROR, format!("Error: {:#}", e)).into_response()
        }
    }
}

/// Stop all nodes and containers
pub async fn stop_all(State(state): State<Arc<WebState>>) -> Response {
    let coordinator = &state.member_handle;
    match coordinator.stop_all().await {
        Ok(count) => {
            tracing::info!("[Web UI] Sent Stop control to {} nodes/containers", count);
            (
                StatusCode::OK,
                format!("Stopped {} nodes/containers", count),
            )
                .into_response()
        }
        Err(e) => {
            tracing::error!("[Web UI] Failed to stop all: {:#}", e);
            (StatusCode::INTERNAL_SERVER_ERROR, format!("Error: {:#}", e)).into_response()
        }
    }
}

/// Restart all nodes and containers
pub async fn restart_all(State(state): State<Arc<WebState>>) -> Response {
    let coordinator = &state.member_handle;
    match coordinator.restart_all().await {
        Ok(count) => {
            tracing::info!(
                "[Web UI] Sent Restart control to {} nodes/containers",
                count
            );
            (
                StatusCode::OK,
                format!("Restarted {} nodes/containers", count),
            )
                .into_response()
        }
        Err(e) => {
            tracing::error!("[Web UI] Failed to restart all: {:#}", e);
            (StatusCode::INTERNAL_SERVER_ERROR, format!("Error: {:#}", e)).into_response()
        }
    }
}

// ===== Diagnostics API =====

/// List all current diagnostics (latest status for each hardware_id/name)
pub async fn list_diagnostics(
    State(state): State<Arc<WebState>>,
) -> Result<Json<Vec<DiagnosticStatus>>, StatusCode> {
    let diagnostics = state.diagnostic_registry.list_all();
    Ok(Json(diagnostics))
}

/// Get diagnostic counts by level (OK/WARNING/ERROR/STALE)
pub async fn get_diagnostic_counts(
    State(state): State<Arc<WebState>>,
) -> Result<Json<DiagnosticCounts>, StatusCode> {
    let counts = state.diagnostic_registry.get_counts();
    Ok(Json(counts))
}
