//! HTTP handlers for the web API (event-driven architecture).

use super::WebState;
use crate::{
    diagnostics::{DiagnosticCounts, DiagnosticStatus},
    member_actor::web_query::HealthSummary,
};
use axum::{
    extract::{Path, State},
    http::StatusCode,
    response::{IntoResponse, Response},
    Json,
};
use serde_json::json;
use std::{collections::HashMap, sync::Arc};

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

/// List all nodes - returns JSON
pub async fn list_nodes(
    State(state): State<Arc<WebState>>,
) -> Json<Vec<super::web_types::NodeSummary>> {
    let coordinator = &state.member_handle;
    let nodes = coordinator.list_members().await;

    // Build container ROS-name → member-name lookup map
    let mut container_lookup: HashMap<String, String> = HashMap::new();
    for member in &nodes {
        if member.is_container {
            let ros_name = if let Some(ns) = &member.namespace {
                let node_name = member.node_name.as_ref().unwrap_or(&member.name);
                if ns == "/" {
                    format!("/{}", node_name)
                } else if ns.ends_with('/') {
                    format!("{}{}", ns, node_name)
                } else {
                    format!("{}/{}", ns, node_name)
                }
            } else {
                format!("/{}", member.name)
            };
            container_lookup.insert(ros_name, member.name.clone());
        }
    }

    let node_summaries: Vec<_> = nodes
        .iter()
        .map(|member| {
            let mut summary = super::web_types::NodeSummary::from_member_summary(member);
            // Populate container_name for composable nodes
            if summary.node_type == super::web_types::NodeType::ComposableNode {
                if let Some(ref target) = summary.target_container {
                    // Normalize: target_container may lack leading "/" while
                    // container_lookup keys always start with "/"
                    let normalized = if target.starts_with('/') {
                        target.clone()
                    } else {
                        format!("/{}", target)
                    };
                    summary.container_name = container_lookup
                        .get(&normalized)
                        .or_else(|| container_lookup.get(target))
                        .cloned();
                }
            }
            summary
        })
        .collect();

    Json(node_summaries)
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
            if let Some(ref target) = node.target_container {
                let normalized_target = if target.starts_with('/') {
                    target.clone()
                } else {
                    format!("/{}", target)
                };
                // Look up all members to find the container with matching ROS name
                let all_members = coordinator.list_members().await;
                all_members
                    .iter()
                    .find(|m| {
                        if !m.is_container {
                            return false;
                        }
                        let node_name = m.node_name.as_ref().unwrap_or(&m.name);
                        let ros_name = match m.namespace.as_deref() {
                            Some("/") | None => format!("/{}", node_name),
                            Some(ns) if ns.ends_with('/') => format!("{}{}", ns, node_name),
                            Some(ns) => format!("{}/{}", ns, node_name),
                        };
                        ros_name == normalized_target
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

            // Get the updated node state and return as JSON
            let nodes = coordinator.list_members().await;
            if let Some(node) = nodes.iter().find(|n| n.name == name) {
                let node_summary = super::web_types::NodeSummary::from_member_summary(node);
                Json(node_summary).into_response()
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

            // Get the updated node state and return as JSON
            let nodes = coordinator.list_members().await;
            if let Some(node) = nodes.iter().find(|n| n.name == name) {
                let node_summary = super::web_types::NodeSummary::from_member_summary(node);
                Json(node_summary).into_response()
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
    Path((name, enabled)): Path<(String, super::web_types::BoolParam)>,
) -> Response {
    let enabled = enabled.as_bool();
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
    Path((name, enabled)): Path<(String, super::web_types::BoolParam)>,
) -> Response {
    let enabled = enabled.as_bool();
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

/// Health summary - returns JSON
pub async fn health_summary(State(state): State<Arc<WebState>>) -> Json<HealthSummary> {
    let coordinator = &state.member_handle;
    let summary = coordinator.get_health_summary().await;
    Json(summary)
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
