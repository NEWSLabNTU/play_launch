# Autoware GUI Landscape & play_launch Opportunities

Research into Autoware community discussions about GUI needs, and how play_launch
can be improved to address them.

*Last updated: 2026-02-07*

## Autoware's Existing GUI Tools

### Autoware Launch GUI (Active Development)

- **Repository**: [autowarefoundation/autoware-launch-gui](https://github.com/autowarefoundation/autoware-launch-gui)
- **Discussion**: [Discussion #3889](https://github.com/orgs/autowarefoundation/discussions/3889)
- **Technology**: Tauri/NextJS desktop application

Features:
- Launch Autoware with custom parameters
- Real-time CPU/memory monitoring
- Log viewing with search and filtering (INFO, WARN, DEBUG, ERROR by component)
- Profile management for saving/loading configurations
- YAML parameter file editing
- Bag file management (play, record, view .db3/.mcap files)
- Topic management (list, types, echo output)
- Remote SSH operation

Community feedback indicates demand for multiple configuration profiles with
quick-load buttons for different maps, vehicle models, and sensor configurations.

### Legacy Runtime Manager (Autoware.AI — End of Life)

Had persistent issues:
- Multiple launch failures ([Issue #1508](https://github.com/autowarefoundation/autoware/issues/1508))
- Parameter synchronization problems ([Issue #598](https://github.com/autowarefoundation/autoware_ai/issues/598))
- Display and terminal compatibility problems

### Third-Party Tools

- **Foxglove Studio**: Live ROS 2 visualization, topic panels, 3D views. [Docs](https://docs.ros.org/en/foxy/How-To-Guides/Visualizing-ROS-2-Data-With-Foxglove-Studio.html)
- **ROS2 STUDIO**: Real-time monitoring, bag recorder/player, system dashboard. [Repo](https://github.com/Sourav0607/ROS2-STUDIO)
- **rqt_* tools**: `rqt_graph` (node graph), `rqt_plot` (topic plotting), `rqt_reconfigure` (parameter tuning)

## Critical Pain Points from Community Discussions

### 1. Launch Hierarchy is Opaque

**Source**: [Discussion #5313 — Improve the Autoware Launch System](https://github.com/orgs/autowarefoundation/discussions/5313)

- 8-level deep launch file nesting across 3 repositories and 4 packages
- Impossible to trace which file sets a parameter's final value
- Argument scope bleeds downstream without explicit declaration
- Configuration duplication across `.param.yaml` files
- Proposed solutions: centralized launch repository, simplified hierarchy,
  explicit argument passing, automated validation

### 2. Node Visibility & Control

**Source**: [Discussion #3194 — AutowareNode base class for Lifecycle and Monitoring](https://github.com/orgs/autowarefoundation/discussions/3194)

- "It's hard to know what Autoware nodes are running currently"
- No centralized dashboard showing node states (error/warning/dead/inactive)
- Difficulty shutting down specific nodes
- Missing heartbeat/watchdog for all nodes
- LifecycleNodes sit idle without centralized state management

Proposed an **Autoware Control Center (ACC)** with:
- Centralized monitoring and management node (similar to nav2_lifecycle_manager)
- Automatic node registration on startup
- Remote state transitions (active/inactive) and shutdown commands
- Heartbeat/watchdog monitoring without developer overhead
- Dashboard showing node states

### 3. Debugging Single Nodes is Painful

**Source**: [Discussion #6701 — How to launch a single node for debugging](https://github.com/orgs/autowarefoundation/discussions/6701)

- ComposableNodes in containers complicate debugging (can't isolate easily)
- Developers resort to `ps x | grep NODE_NAME` to extract launch commands
- Converting ComposableNode to regular Node just for debugging
- Large parameter sets become unconfigured when isolating nodes
- Need to manually extract process commands with full parameters/remappings

### 4. Remote & Fleet Management

**Sources**:
- [Discussion #5369 — Autoware API for Remote/Direct Control](https://github.com/orgs/autowarefoundation/discussions/5369)
- [Zenoh FMS](https://github.com/evshary/zenoh_autoware_fms), [Discussion #4780](https://github.com/orgs/autowarefoundation/discussions/4780)
- [Blog: Managing Multiple Autoware Vehicles with Zenoh](https://autoware.org/managing-multiple-autoware-vehicles-with-zenoh/)

Use cases:
- Remote operation for vehicles without steering wheels/pedals
- Fleet management via web interfaces
- Multi-vehicle monitoring and control
- Vehicle status dashboards (gear, velocity, steering)

### 5. Container Architecture Debate

**Source**: [Discussion #3818 — Put every node into a single component container?](https://github.com/orgs/autowarefoundation/discussions/3818)

- Separation preferred for safety: single container crash brings down entire system
- Different failure modes require different responses (emergency shutdown vs. pull over)
- Multiple containers enable better failure isolation
- Recommendation: make all nodes composable for flexibility, let users choose container configs

### 6. Runtime Parameter Tuning

**Source**: [ROS Discourse — Runtime-configurable Parameters](https://discourse.ros.org/t/runtime-configurable-parameters-in-autoware-ai/10653)

- Confusion about what should be runtime-configurable
- Parameters scattered across launch files and config files
- No validation during runtime parameter changes
- Difficult to track which parameters affect behavior

## What play_launch Already Covers

| Autoware Need | play_launch Status |
|---|---|
| See all running nodes & status | Done — real-time node list with status colors |
| Start/Stop/Restart individual nodes | Done — per-node controls |
| Container management (Load/Unload) | Done — composable node operations |
| Log streaming with filtering | Done — real-time stdout/stderr via SSE |
| Diagnostics monitoring | Done — `/diagnostics` with severity filtering |
| CPU/memory monitoring | Done — per-process and system-wide |
| Bulk operations | Done — Start/Stop/Restart All, Load/Unload All |
| Web-based (no desktop app needed) | Done — lightweight, embedded in binary |
| Auto-restart/respawn | Done — per-node toggle |

## Enhancement Opportunities

### Strategic Positioning

The Autoware Launch GUI is a **pre-launch** tool (configure then launch).
play_launch is a **runtime** tool (launch then monitor and debug).
They are complementary, not competing.

Our strongest differentiator is the **parser**: we already understand the full
launch graph, parameter origins, and node relationships. Building visualization
on top of that parsed data is something no other runtime tool can easily replicate.

The biggest gap in the Autoware ecosystem is **"what is actually running and why
does it have these settings?"** — exactly where our parser + Web UI combination
can shine.

### Tier 1 — Unique Differentiators

Features only play_launch is positioned to do well, leveraging our parser + replay
architecture.

#### 1. Launch File Hierarchy Visualization

Our parser already resolves the full 8-level launch tree. We could render an
interactive tree showing:
- Which file included which
- Parameter flow (where each param value comes from)
- Condition evaluation paths (which `if/unless` branches were taken)
- Click-to-navigate to source files

Directly addresses Discussion #5313's top complaint. No other tool does this.

#### 2. Node Isolation / Debug Helper

Since `record.json` captures the full command, parameters, remaps, and environment
for every node, we could add a **"Copy debug command"** button that extracts a
standalone launch command for any node — including composable nodes extracted from
their container.

Directly addresses Discussion #6701. Eliminates the `ps x | grep` workaround.

#### 3. Parameter Provenance Tracking

Extend the parser to record *where* each parameter came from (which YAML file,
which `<param>` tag, which `SetParameter` action). Display this in the Web UI so
users can answer "why does this node have this value?" — the #1 configuration
debugging question.

### Tier 2 — High Value, Moderate Effort

#### 4. LifecycleNode State Management

Add GUI controls for ROS 2 lifecycle transitions (configure, activate, deactivate,
shutdown). Many Autoware nodes are LifecycleNodes but lack centralized management.
Addresses Discussion #3194's ACC proposal.

#### 5. Runtime Parameter Editing

Allow modifying ROS parameters at runtime via the Web UI (using `ros2 param set`
under the hood). Show current vs. launch-time values, highlight drift.

#### 6. Topic / Connection Graph

Show which nodes publish/subscribe to which topics — a lightweight `rqt_graph` in
the browser. Could leverage `ros2 topic list` and `ros2 node info` data collected
periodically.

#### 7. Configuration Profiles

Save/load different launch configurations (parameter overrides, enabled/disabled
nodes). The Autoware Launch GUI already does this — we would need to match it for
the runtime side.

### Tier 3 — Nice to Have

#### 8. Bag File Integration

Play/record/inspect bag files from the UI.

#### 9. Multi-Instance / Fleet Mode

Monitor multiple Autoware stacks from one dashboard. Relevant for Zenoh-based
fleet management setups.

#### 10. Historical Metrics Trending

Graph CPU/memory over time in the Web UI (currently CSV-only).

#### 11. Node Health Scoring

Aggregate diagnostics + resource usage into a composite health score per node.

## References

- [Introducing Autoware Launch GUI — Discussion #3889](https://github.com/orgs/autowarefoundation/discussions/3889)
- [Autoware Launch GUI Repository](https://github.com/autowarefoundation/autoware-launch-gui)
- [Improve the Autoware Launch System — Discussion #5313](https://github.com/orgs/autowarefoundation/discussions/5313)
- [How to launch a single node for debugging — Discussion #6701](https://github.com/orgs/autowarefoundation/discussions/6701)
- [AutowareNode base class for Lifecycle and Monitoring — Discussion #3194](https://github.com/orgs/autowarefoundation/discussions/3194)
- [Autoware API for Remote/Direct Control — Discussion #5369](https://github.com/orgs/autowarefoundation/discussions/5369)
- [Managing Multiple Autoware Vehicles with Zenoh](https://autoware.org/managing-multiple-autoware-vehicles-with-zenoh/)
- [Zenoh Autoware FMS Repository](https://github.com/evshary/zenoh_autoware_fms)
- [Fleet Management System Demo — Discussion #4780](https://github.com/orgs/autowarefoundation/discussions/4780)
- [Put every node into a single component container? — Discussion #3818](https://github.com/orgs/autowarefoundation/discussions/3818)
- [Runtime-configurable Parameters in Autoware.ai — ROS Discourse](https://discourse.ros.org/t/runtime-configurable-parameters-in-autoware-ai/10653)
- [System Monitor for Autoware](https://autowarefoundation.github.io/autoware.universe/pr-2609/system/system_monitor/)
- [Debug Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/pr-366/how-to-guides/debug-autoware/)
- [Scenario Simulator v2](https://github.com/tier4/scenario_simulator_v2)
- [Foxglove Studio — ROS 2 Visualization](https://docs.ros.org/en/foxy/How-To-Guides/Visualizing-ROS-2-Data-With-Foxglove-Studio.html)
- [ROS2 STUDIO Repository](https://github.com/Sourav0607/ROS2-STUDIO)
