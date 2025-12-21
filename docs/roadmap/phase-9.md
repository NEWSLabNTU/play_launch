# Web UI Status System Refactoring

## Overview

This document outlines the implementation plan for refactoring the Web UI status system to properly handle distinct status types for regular nodes, containers, and composable nodes, along with improving the health summary display and container lifecycle management.

## Goals

1. **Distinct Status Types**: Separate status enums for process-based nodes (NodeStatus) and loaded composable nodes (ComposableNodeStatus)
2. **Clear Health Metrics**: Show categorized counts for processes, nodes, containers, and composable nodes
3. **Container Lifecycle**: Support container restart with automatic composable node reloading
4. **Improved UX**: Clear visual distinction between node types with appropriate controls

## Current Issues

1. **Status Confusion**: Loaded composable nodes incorrectly shown as "stopped" instead of "loaded"
2. **Ambiguous Counts**: "Running/Stopped" labels don't clarify if counting processes or nodes
3. **Missing Composable Logs**: Individual composable node logs don't exist (they log to container), but Logs button is shown
4. **Container Restart**: Restarting a container orphans composable nodes without reloading them
5. **Control Button Confusion**: Composable nodes show Start/Stop buttons but can't be controlled that way

## Background

### Node Type Hierarchy

```
┌─────────────────────────────────────────────────────────────┐
│ Regular Node                                                │
│ - OS process with PID                                       │
│ - Status: Running/Stopped based on process state            │
│ - Controls: Start/Stop/Restart                              │
│ - Logs: stdout/stderr files                                 │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Container Node                                              │
│ - OS process (component_container)                          │
│ - Status: Running/Stopped based on process state            │
│ - Controls: Start/Stop/Restart (with composable reload)     │
│ - Logs: stdout/stderr files                                 │
│ - Children: Contains composable nodes                       │
└─────────────────────────────────────────────────────────────┘
                        │
                        ├── Composable Node 1
                        ├── Composable Node 2
                        └── Composable Node 3
                              │
                              └─ Loaded into container via service call
                                 - No separate process (runs in container)
                                 - Status: Loaded/Failed/Pending
                                 - Controls: None (managed by container)
                                 - Logs: No individual logs (logs to container)
```

---

## Implementation Plan

### Phase 1: Refactor Status System

**Estimated Effort**: 2-3 hours

#### 1.1 Create Distinct Status Enums

**File**: `src/play_launch/src/node_registry.rs`

**Add new status types**:
```rust
/// Status for regular nodes and containers (process-based)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum NodeStatus {
    Running,
    Stopped,
    Failed,
    Pending,
}

/// Status for composable nodes (loaded into containers)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ComposableNodeStatus {
    Loaded,    // Successfully loaded into container
    Failed,    // Load failed
    Pending,   // Not yet loaded
}

/// Unified status for all node types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(tag = "status_type", content = "status")]
pub enum UnifiedStatus {
    Process(NodeStatus),
    Composable(ComposableNodeStatus),
}
```

**Update NodeSummary**:
```rust
pub struct NodeSummary {
    pub name: String,
    pub node_type: NodeType,
    pub status: UnifiedStatus,  // Changed from NodeStatus
    pub exec_name: Option<String>,
    pub node_name: Option<String>,
    pub namespace: Option<String>,
    pub pid: Option<u32>,
    pub target_container: Option<String>,
}
```

**Work Items**:
- [x] Define NodeStatus enum for process-based nodes
- [x] Define ComposableNodeStatus enum for loaded nodes
- [x] Define UnifiedStatus wrapper enum
- [x] Update NodeSummary to use UnifiedStatus
- [x] Add serde attributes for JSON serialization

---

#### 1.2 Update Status Determination Logic

**File**: `src/play_launch/src/node_registry.rs`

**Update `NodeHandle::get_status()` method**:
```rust
impl NodeHandle {
    pub fn get_status(&self) -> UnifiedStatus {
        match &self.info {
            NodeInfo::Regular(_) => {
                // Check PID file
                let running = self.log_paths.pid_file.exists() &&
                    self.last_pid.map(is_process_running).unwrap_or(false);

                let status = if running {
                    NodeStatus::Running
                } else if self.log_paths.status_file.exists() {
                    // Check status file for failure
                    NodeStatus::Stopped
                } else {
                    NodeStatus::Pending
                };

                UnifiedStatus::Process(status)
            }
            NodeInfo::Composable(_) => {
                // Check service_response file existence (simple file check)
                let loaded = self.output_dir
                    .join("service_response.success")
                    .exists();

                let failed = self.output_dir
                    .join("service_response.error")
                    .exists();

                let status = if loaded {
                    ComposableNodeStatus::Loaded
                } else if failed {
                    ComposableNodeStatus::Failed
                } else {
                    ComposableNodeStatus::Pending
                };

                UnifiedStatus::Composable(status)
            }
        }
    }
}
```

**Work Items**:
- [x] Update get_status() to return UnifiedStatus
- [x] Implement process-based status checking (PID file)
- [x] Implement composable status checking (service_response file)
- [x] Handle edge cases (missing files, terminated processes)

---

### Phase 2: Refactor Health Summary

**Estimated Effort**: 1-2 hours

#### 2.1 Update HealthSummary Structure

**File**: `src/play_launch/src/node_registry.rs`

**Replace current structure**:
```rust
pub struct HealthSummary {
    // Process-level counts
    pub processes_running: usize,
    pub processes_stopped: usize,

    // Regular node counts
    pub nodes_running: usize,
    pub nodes_stopped: usize,
    pub nodes_failed: usize,
    pub nodes_total: usize,

    // Container counts
    pub containers_running: usize,
    pub containers_stopped: usize,
    pub containers_failed: usize,
    pub containers_total: usize,

    // Composable node counts
    pub composable_loaded: usize,
    pub composable_failed: usize,
    pub composable_pending: usize,
    pub composable_total: usize,

    // Legacy
    pub noisy: usize,
}
```

**Work Items**:
- [x] Define new HealthSummary structure
- [x] Remove old fields (total, running, stopped, failed, pending, nodes, containers, composable_nodes)
- [x] Add categorized counts by node type and status

---

#### 2.2 Implement get_health_summary()

**File**: `src/play_launch/src/node_registry.rs`

```rust
impl NodeRegistry {
    pub fn get_health_summary(&self) -> HealthSummary {
        let mut summary = HealthSummary::default();

        for handle in self.nodes.values() {
            let status = handle.get_status();

            match handle.node_type() {
                NodeType::Node => {
                    summary.nodes_total += 1;
                    match status {
                        UnifiedStatus::Process(NodeStatus::Running) => {
                            summary.nodes_running += 1;
                            summary.processes_running += 1;
                        }
                        UnifiedStatus::Process(NodeStatus::Stopped) => {
                            summary.nodes_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        UnifiedStatus::Process(NodeStatus::Failed) => {
                            summary.nodes_failed += 1;
                        }
                        _ => {}
                    }
                }
                NodeType::Container => {
                    summary.containers_total += 1;
                    match status {
                        UnifiedStatus::Process(NodeStatus::Running) => {
                            summary.containers_running += 1;
                            summary.processes_running += 1;
                        }
                        UnifiedStatus::Process(NodeStatus::Stopped) => {
                            summary.containers_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        UnifiedStatus::Process(NodeStatus::Failed) => {
                            summary.containers_failed += 1;
                        }
                        _ => {}
                    }
                }
                NodeType::ComposableNode => {
                    summary.composable_total += 1;
                    match status {
                        UnifiedStatus::Composable(ComposableNodeStatus::Loaded) => {
                            summary.composable_loaded += 1;
                        }
                        UnifiedStatus::Composable(ComposableNodeStatus::Failed) => {
                            summary.composable_failed += 1;
                        }
                        UnifiedStatus::Composable(ComposableNodeStatus::Pending) => {
                            summary.composable_pending += 1;
                        }
                        _ => {}
                    }
                }
            }

            // Check if noisy
            if handle.is_noisy() {
                summary.noisy += 1;
            }
        }

        summary
    }
}
```

**Work Items**:
- [x] Implement categorized counting logic
- [x] Separate process counts from node type counts
- [x] Handle all status variants

---

### Phase 3: Container Restart with Composable Reload

**Estimated Effort**: 3-4 hours

#### 3.1 Add ComponentLoader to WebState

**File**: `src/play_launch/src/web/mod.rs`

```rust
pub struct WebState {
    pub registry: Arc<Mutex<NodeRegistry>>,
    pub component_loader: Option<ComponentLoaderHandle>,
}
```

**Work Items**:
- [x] Add component_loader field to WebState
- [x] Pass ComponentLoaderHandle from main.rs when web UI is enabled
- [x] Make it Optional (None if not using composable nodes)

---

#### 3.2 Add Helper Methods to NodeRegistry

**File**: `src/play_launch/src/node_registry.rs`

```rust
impl NodeRegistry {
    /// Get all composable nodes belonging to a container
    pub fn get_container_composable_nodes(&self, container_name: &str) -> Vec<String> {
        let container_full_name = self.get_container_full_ros_name(container_name);

        self.nodes
            .values()
            .filter(|h| {
                matches!(h.node_type(), NodeType::ComposableNode) &&
                h.target_container.as_ref() == Some(&container_full_name)
            })
            .map(|h| h.name.clone())
            .collect()
    }

    /// Get container's full ROS name (namespace + name)
    fn get_container_full_ros_name(&self, container_name: &str) -> String {
        if let Some(handle) = self.nodes.get(container_name) {
            if let Some(ns) = &handle.namespace {
                if ns == "/" {
                    format!("/{}", container_name)
                } else if ns.ends_with('/') {
                    format!("{}{}", ns, container_name)
                } else {
                    format!("{}/{}", ns, container_name)
                }
            } else {
                format!("/{}", container_name)
            }
        } else {
            format!("/{}", container_name)
        }
    }
}
```

**Work Items**:
- [x] Implement get_container_composable_nodes()
- [x] Implement get_container_full_ros_name() helper
- [x] Add tests for name resolution

---

#### 3.3 Update restart_node for Containers

**File**: `src/play_launch/src/web/handlers.rs`

```rust
pub async fn restart_node(
    State(state): State<Arc<WebState>>,
    Path(name): Path<String>,
) -> Response {
    // Check if this is a container
    let is_container = {
        let registry = state.registry.lock().await;
        registry.get(&name)
            .map(|h| matches!(h.node_type(), NodeType::Container))
            .unwrap_or(false)
    };

    if is_container && state.component_loader.is_some() {
        // Get composable nodes before stopping container
        let composable_nodes = {
            let registry = state.registry.lock().await;
            registry.get_container_composable_nodes(&name)
        };

        // Restart the container process
        let restart_result = {
            let mut registry = state.registry.lock().await;
            registry.restart_node(&name).await
        };

        match restart_result {
            Ok(pid) => {
                // Reload composable nodes
                let mut reload_results = Vec::new();

                for comp_name in &composable_nodes {
                    if let Some(loader) = &state.component_loader {
                        match reload_composable_node(comp_name, &state.registry, loader).await {
                            Ok(_) => {
                                reload_results.push(format!("✓ {}", comp_name));
                            }
                            Err(e) => {
                                // Mark as failed but continue
                                reload_results.push(format!("✗ {}: {}", comp_name, e));
                            }
                        }
                    }
                }

                let message = format!(
                    "Container '{}' restarted with PID {}\nReloaded composable nodes:\n{}",
                    name, pid, reload_results.join("\n")
                );

                (StatusCode::OK, message).into_response()
            }
            Err(e) => {
                (
                    StatusCode::BAD_REQUEST,
                    format!("Failed to restart container '{}': {}", name, e),
                )
                    .into_response()
            }
        }
    } else {
        // Regular node restart
        let mut registry = state.registry.lock().await;
        match registry.restart_node(&name).await {
            Ok(pid) => {
                (StatusCode::OK, format!("Node '{}' restarted with PID {}", name, pid))
                    .into_response()
            }
            Err(e) => {
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
async fn reload_composable_node(
    node_name: &str,
    registry: &Arc<Mutex<NodeRegistry>>,
    loader: &ComponentLoaderHandle,
) -> eyre::Result<()> {
    // Get node info
    let (target_container, record) = {
        let reg = registry.lock().await;
        let handle = reg.get(node_name)
            .ok_or_else(|| eyre::eyre!("Node not found"))?;

        match &handle.info {
            NodeInfo::Composable(info) => {
                (info.record.target_container.clone(), info.record.clone())
            }
            _ => return Err(eyre::eyre!("Not a composable node")),
        }
    };

    // Call loader service
    loader.load_node(&target_container, &record).await?;

    Ok(())
}
```

**Work Items**:
- [x] Detect if node is a container
- [x] Get list of composable nodes before restart
- [x] Restart container process
- [x] Reload each composable node via ComponentLoader
- [x] Mark failed reloads as Failed, continue with others
- [x] Return detailed status message

---

### Phase 4: Update Web UI

**Estimated Effort**: 2 hours

#### 4.1 Update render_node_card Controls

**File**: `src/play_launch/src/web/handlers.rs`

```rust
// Control buttons based on node type
let controls = match node.node_type {
    NodeType::ComposableNode => {
        // Only Details button (no Start/Stop/Restart/Logs)
        format!(
            r#"<button onclick="showNodeDetails('{}')" class="btn-details">Details</button>"#,
            node.name
        )
    }
    NodeType::Container | NodeType::Node => {
        // Full controls: Start/Stop, Restart, Details, Logs
        format!(
            r#"{}
            <button hx-post="/api/nodes/{}/restart" hx-swap="none" class="btn-restart">Restart</button>
            <button onclick="showNodeDetails('{}')" class="btn-details">Details</button>
            <button onclick="showNodeLogs('{}')" class="btn-logs">Logs</button>"#,
            start_stop_button, node.name, node.name, node.name
        )
    }
};
```

**Work Items**:
- [x] Remove Start/Stop/Restart/Logs buttons from composable node cards
- [x] Keep only Details button for composable nodes
- [x] Keep full controls for regular nodes and containers

---

#### 4.2 Update health_summary Badge Format

**File**: `src/play_launch/src/web/handlers.rs`

```rust
pub async fn health_summary(State(state): State<Arc<WebState>>) -> Response {
    let registry = state.registry.lock().await;
    let summary = registry.get_health_summary();

    let html = format!(
        r#"<div class="health-summary">
  <span class="badge badge-nodes">Nodes: {running} running / {total} total</span>
  <span class="badge badge-containers">Containers: {c_running} running / {c_total} total</span>
  <span class="badge badge-composable">Composable: {loaded} loaded / {comp_total} total</span>
  <span class="badge badge-processes">Processes: {proc_running} running</span>
  {noisy_badge}
</div>"#,
        running = summary.nodes_running,
        total = summary.nodes_total,
        c_running = summary.containers_running,
        c_total = summary.containers_total,
        loaded = summary.composable_loaded,
        comp_total = summary.composable_total,
        proc_running = summary.processes_running,
        noisy_badge = if summary.noisy > 0 {
            format!(r#"<span class="badge badge-noisy">Noisy: {}</span>"#, summary.noisy)
        } else {
            String::new()
        }
    );

    Html(html).into_response()
}
```

**Work Items**:
- [x] Update badge format to "X running / Y total"
- [x] Show separate badges for Nodes, Containers, Composable, Processes
- [x] Keep noisy badge if noisy > 0

---

#### 4.3 Update CSS for Badge Colors

**File**: `src/play_launch/src/web/assets/index.html`

```css
/* Badge color scheme */
.badge-nodes {
    background: #10b981;    /* Green - running nodes */
    color: white;
}

.badge-containers {
    background: #8b5cf6;    /* Purple - containers */
    color: white;
}

.badge-composable {
    background: #06b6d4;    /* Cyan - composable nodes */
    color: white;
}

.badge-processes {
    background: #6b7280;    /* Gray - process count */
    color: white;
}

.badge-noisy {
    background: #f59e0b;    /* Orange - noisy nodes */
    color: white;
}
```

**Work Items**:
- [x] Add color scheme for each badge type
- [x] Use green for running nodes
- [x] Use purple for containers
- [x] Use cyan for composable nodes
- [x] Use gray for processes
- [x] Keep orange for noisy

---

#### 4.4 Update Status Badge Display

**File**: `src/play_launch/src/web/handlers.rs`

```rust
// Status class based on UnifiedStatus
let status_class = match &node.status {
    UnifiedStatus::Process(NodeStatus::Running) => "status-running",
    UnifiedStatus::Process(NodeStatus::Stopped) => "status-stopped",
    UnifiedStatus::Process(NodeStatus::Failed) => "status-failed",
    UnifiedStatus::Process(NodeStatus::Pending) => "status-pending",
    UnifiedStatus::Composable(ComposableNodeStatus::Loaded) => "status-loaded",
    UnifiedStatus::Composable(ComposableNodeStatus::Failed) => "status-failed",
    UnifiedStatus::Composable(ComposableNodeStatus::Pending) => "status-pending",
};
```

**Work Items**:
- [x] Add status-loaded CSS class (green like running)
- [x] Update status class logic to handle UnifiedStatus

---

### Phase 5: Testing & Validation

**Estimated Effort**: 2-3 hours

#### 5.1 Unit Tests

**File**: `src/play_launch/src/node_registry.rs`

- [x] Test NodeStatus enum serialization
- [x] Test ComposableNodeStatus enum serialization
- [x] Test UnifiedStatus enum serialization
- [x] Test get_status() for regular nodes
- [x] Test get_status() for containers
- [x] Test get_status() for composable nodes
- [x] Test get_health_summary() with mixed node types

#### 5.2 Integration Tests

**Manual Testing Checklist**:
- [ ] Start demo with `--web-ui`
- [ ] Verify health summary shows correct counts
- [ ] Verify composable nodes show "Loaded" status (green)
- [ ] Verify composable nodes have only Details button (no Logs/Start/Stop/Restart)
- [ ] Verify regular nodes have all control buttons
- [ ] Verify containers have all control buttons
- [ ] Stop a container, verify composable nodes show "Failed"
- [ ] Restart a container, verify composable nodes reload and show "Loaded"
- [ ] Verify restart failure marks composable nodes as Failed but continues

#### 5.3 Autoware Test

- [ ] Run Autoware test with `just start-sim --web-ui`
- [ ] Verify all 15 containers shown correctly
- [ ] Verify all ~50 composable nodes shown as "Loaded"
- [ ] Restart a container, verify composable nodes reload
- [ ] Check logs for errors

---

## File Change Summary

| File | Changes | Estimated Lines |
|------|---------|----------------|
| `node_registry.rs` | Add status enums, update get_status(), update HealthSummary, add helper methods | +150 |
| `web/mod.rs` | Add component_loader to WebState | +5 |
| `web/handlers.rs` | Update render_node_card controls, update health_summary format, update restart_node logic | +80 |
| `web/assets/index.html` | Update CSS badge colors, add status-loaded class | +20 |
| `main.rs` | Pass component_loader to WebState | +5 |

**Total Estimated Changes**: ~260 lines

---

## Success Criteria

### Phase 1: Status System
- [x] NodeStatus enum defined for process-based nodes
- [x] ComposableNodeStatus enum defined for loaded nodes
- [x] UnifiedStatus wrapper enum properly serializes to JSON
- [x] get_status() returns correct status for all node types
- [x] Composable nodes show "Loaded" instead of "Stopped"

### Phase 2: Health Summary
- [x] HealthSummary structure includes categorized counts
- [x] get_health_summary() correctly counts all categories
- [x] Badge format shows "X running / Y total"
- [x] Distinct badges for Nodes, Containers, Composable, Processes

### Phase 3: Container Restart
- [x] restart_node detects container type
- [x] Composable nodes are reloaded after container restart
- [x] Failed reloads are marked but don't stop the process
- [x] Detailed status message shows reload results

### Phase 4: Web UI
- [x] Composable node cards show only Details button
- [x] Regular nodes and containers show all control buttons
- [x] Health summary displays with correct colors
- [x] "Loaded" status shows green like "Running"

### Phase 5: Testing
- [ ] Unit tests pass
- [ ] Manual tests pass
- [ ] Autoware integration test passes

---

## Risks & Mitigation

| Risk | Impact | Mitigation |
|------|--------|------------|
| ComponentLoader not available in WebState | High | Make it Optional, check before reload |
| Composable reload fails | Medium | Mark as Failed, continue with others |
| Status enum serialization breaks API | High | Add comprehensive tests |
| Container restart timeout | Medium | Use existing timeout logic from restart_node |
| Breaking changes to JSON API | High | Update frontend to handle UnifiedStatus |

---

## Timeline Estimate

| Phase | Estimated Time | Cumulative |
|-------|----------------|------------|
| Phase 1: Status System | 2-3 hours | 2-3 hours |
| Phase 2: Health Summary | 1-2 hours | 3-5 hours |
| Phase 3: Container Restart | 3-4 hours | 6-9 hours |
| Phase 4: Web UI Updates | 2 hours | 8-11 hours |
| Phase 5: Testing | 2-3 hours | 10-14 hours |
| **Total** | **10-14 hours** | **~1.5-2 days** |

---

## Dependencies

### External
- ComponentLoaderHandle must be available from main.rs
- NodeRegistry must be populated before web UI starts

### Internal
- UnifiedStatus enum must be properly serialized for JSON API
- Frontend must handle new status format
- CSS must support new status-loaded class

---

## Future Enhancements

### Optional Improvements (Not in Scope)
- [ ] Add "Unload" button for composable nodes (requires implementing unload service call)
- [ ] Show container-composable hierarchy visually in UI (tree view)
- [ ] Add "Reload All" button for containers
- [ ] Show composable node status in container details
- [ ] Add filtering by status (show only failed, show only loaded, etc.)
- [ ] Add reload progress indicator during container restart
