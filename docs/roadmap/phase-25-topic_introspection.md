# Phase 25: Runtime Graph & Topic Introspection

**Status**: 25.1–25.9 complete, 25.10 in progress
**Priority**: Medium (Observability)
**Dependencies**: Phase 24 (Parameter Control), rclrs vendor patch

---

## Overview

Build a runtime communication graph of all managed nodes: which topics and
services connect them, what QoS each endpoint uses, and where connections are
missing. This enables dangling topic/node detection — a key diagnostic for
understanding why data isn't flowing.

Three UI surfaces expose the graph:

1. **Topic count badges on node cards** — at-a-glance pub/sub/dangling counts
2. **TopicsTab in right panel** — per-node topic detail with click-to-expand
   connected nodes and jump-to-node navigation
3. **Graph view** — namespace-grouped interactive visualization with
   drill-down, powered by Cytoscape.js

play_launch already knows every node from `record.json` and has a shared
`rclrs::Node` with a spinning executor. The rclrs graph API queries the
DDS/Zenoh discovery cache in-memory — no subscriptions, no network I/O, very
cheap. QoS information requires a small patch to the vendored rclrs.

### Autoware production data (512 topics, 140 nodes)

```
Published topics:  486    Subscribed topics:  319
Published but 0 subscribers:  265    (54% of published topics!)
Subscribed but 0 publishers:   98    (31% of subscribed topics!)
```

Many of these are expected (API endpoints awaiting external callers, sensor
inputs not connected in simulation), but having this visibility in the UI lets
users immediately spot genuine wiring problems.

---

## Research Findings

### 1. rclrs Graph API (Ready)

`NodeState` in `src/vendor/ros2_rust/rclrs/src/node/graph.rs` exposes:

**Per-node queries** (take remote node name + namespace):

| Method                                               | Returns                              |
|------------------------------------------------------|--------------------------------------|
| `get_publisher_names_and_types_by_node(name, ns)`    | `HashMap<TopicName, Vec<MsgType>>`   |
| `get_subscription_names_and_types_by_node(name, ns)` | `HashMap<TopicName, Vec<MsgType>>`   |
| `get_service_names_and_types_by_node(name, ns)`      | `HashMap<ServiceName, Vec<SrvType>>` |
| `get_client_names_and_types_by_node(name, ns)`       | `HashMap<ServiceName, Vec<SrvType>>` |

**Per-topic queries**:

| Method                                   | Returns                                                          |
|------------------------------------------|------------------------------------------------------------------|
| `get_publishers_info_by_topic(topic)`    | `Vec<TopicEndpointInfo>` (node_name, node_namespace, topic_type) |
| `get_subscriptions_info_by_topic(topic)` | `Vec<TopicEndpointInfo>`                                         |
| `count_publishers(topic)`                | `usize`                                                          |
| `count_subscriptions(topic)`             | `usize`                                                          |

**Global queries**:

| Method                        | Returns                            |
|-------------------------------|------------------------------------|
| `get_topic_names_and_types()` | `HashMap<TopicName, Vec<MsgType>>` |
| `get_node_names()`            | `Vec<NodeNameInfo>`                |

All methods query the in-memory discovery cache. One-shot, synchronous, ~microseconds.

### 2. QoS in TopicEndpointInfo (Requires rclrs Patch)

The underlying `rmw_topic_endpoint_info_t` has `qos_profile` with full details
(reliability, durability, deadline, history depth, lifespan, liveliness), but
rclrs's `TopicEndpointInfo` struct drops it during conversion.

**Fix**: ~10 lines in `src/vendor/ros2_rust/rclrs/src/node/graph.rs`. The
`From<&rmw_qos_profile_t> for QoSProfile` conversion already exists in `qos.rs`.

### 3. Topic Statistics — Reference (Not in Scope)

No RMW-agnostic way to get message rate or bandwidth without subscribing.
RMW-specific approaches documented below for future reference.

**FastDDS**: Statistics Module publishes to internal `_fastdds_statistics_*`
topics. Requires `FASTDDS_STATISTICS` env var before node launch. FastDDS-only.

**CycloneDDS**: Debug Monitor (`<MonitorPort>`) provides JSON dump via HTTP with
per-writer `sent_bytes`, `seq` (message count), per-reader `received_bytes`.
Public `dds_statistics.h` API is limited (4 writer + 1 reader counters, no
message count). Debug monitor is diagnostic-only, not a production API.

**Zenoh**: Router admin space (`@/*/router?_stats=true`) provides aggregate
router-level counters only. Requires `stats` Cargo feature at build time. No
per-topic breakdown. rmw_zenoh has no message counting at all.

### 4. Visualization Library Selection

**Requirement**: Interactive namespace-grouped graph with compound (nested)
nodes, click/drag/hover/zoom, collapsible groups, vendorable (no CDN).

| Library          | Gzip    | Compound Nodes  | Collapsible   | License |
|------------------|---------|-----------------|---------------|---------|
| **Cytoscape.js** | ~110 KB | First-class     | Via extension | MIT     |
| D3-force + SVG   | ~35 KB  | Manual only     | Manual only   | ISC     |
| ELK.js           | ~435 KB | First-class     | Layout-only   | EPL-2.0 |
| Dagre            | ~30 KB  | No              | No            | MIT     |
| vis-network      | ~160 KB | No (clustering) | Clustering    | MIT     |
| Sigma.js v3      | ~60 KB  | No              | No            | MIT     |

**Selected: Cytoscape.js** — only mainstream library with first-class compound
node support. Canvas-based (fine at 50-200 nodes). Extensions:

- `cytoscape-fcose` (~15 KB) — fast compound spring embedder layout
- `cytoscape-expand-collapse` (~10 KB) — collapsible namespace groups

Total: ~135 KB gzipped. MIT throughout. Vendor as ESM in
`src/play_launch/src/web/assets/js/vendor/`.

---

## Design

### Runtime graph model

```
GraphSnapshot {
    nodes:    HashMap<FQN, NodeGraph>,
    topics:   HashMap<TopicName, TopicGraph>,
    services: HashMap<ServiceName, ServiceGraph>,
}

NodeGraph {
    fqn, member_name,
    publishers:  Vec<TopicEndpoint>,   // topics this node publishes
    subscribers: Vec<TopicEndpoint>,   // topics this node subscribes to
    servers:     Vec<ServiceEndpoint>, // services this node offers
    clients:     Vec<ServiceEndpoint>, // services this node calls
}

TopicGraph {
    name, msg_type,
    publishers:  Vec<FQN>,   // which nodes publish
    subscribers: Vec<FQN>,   // which nodes subscribe
    // Derived:
    dangling: bool,          // true if pub_count==0 or sub_count==0
}

ServiceGraph {
    name, srv_type,
    servers: Vec<FQN>,
    clients: Vec<FQN>,
}
```

### Caching & efficiency

The graph snapshot is cached server-side and rebuilt on StateEvents (node
started/stopped/loaded/unloaded), debounced 500ms. With 140 nodes, building
costs ~280 rclrs calls (2 per managed node for pub/sub topics) — all in-memory,
<10ms total.

Two-tier QoS: the snapshot carries topology only (which nodes connect to which
topics). QoS detail is fetched lazily when the user expands a specific topic in
the TopicsTab (via `get_publishers_info_by_topic` / `get_subscriptions_info_by_topic`).

### Data flow

```
Browser
  |                          ^
  | GET /api/graph           | GraphSnapshot JSON (cached)
  | GET /api/nodes/:n/topics | per-node view (from cache)
  v                          |
Axum handlers
  |                          ^
  | GraphBuilder             |
  |   .snapshot()  (cached)  |
  |   .node_topics()         |
  v                          |
rclrs graph API -----------> DDS/Zenoh discovery cache (in-memory)
```

### Dangling detection

A topic is "dangling" if it has publishers but zero subscribers, or subscribers
but zero publishers. The API annotates each topic with:
- `publisher_count` / `subscriber_count` (from `count_publishers`/`count_subscriptions`)
- `dangling: bool` flag

The frontend highlights dangling topics with a distinct color/icon.

Filtering infrastructure topics: `/parameter_events`, `/rosout`, `/tf`,
`/tf_static`, `/diagnostics` are published by every node with typically no
dedicated subscriber in play_launch's scope — they should be filterable or
de-emphasized in the UI.

---

## UI Design

### A. Topic count badges on node cards

Each node card shows a compact summary line below the ROS name:

```
┌───────────────────────────────────────────────┐
│ [Container] route_selector        PID 12345   │
│ /planning/scenario_planning                   │
│ 3 pub  5 sub  2 srv  ⚠ 1 dangling            │
│ [Stop]                            [View]      │
└───────────────────────────────────────────────┘
```

Badge counts are derived from the cached GraphSnapshot — no per-card API calls.
The dangling count is highlighted in amber. Badges are hidden when the node is
not running (no DDS endpoints exist).

### B. TopicsTab in right panel — click-to-expand, jump-to-node

**Tab order**: `Member Info | Params | Topics | stdout | stderr`

Four collapsible sections (Publishers, Subscribers, Service Servers, Service
Clients). Each topic row is clickable to expand:

```
PUBLISHERS (7)                                          [collapse]
─────────────────────────────────────────────────────────────────
▶ /perception/objects    PointCloud2   REL VOL KL(1)    3 sub
▶ /control/status        Bool          ⚠ 0 sub
```

**Click a topic** → inline expansion shows connected nodes:

```
▼ /perception/objects    PointCloud2   REL VOL KL(1)    3 sub
    Subscribers:
      planning_validator (running)                    [Jump]
      motion_planner (running)                        [Jump]
      /external/logger (unmanaged)
    Other publishers:
      lidar_preprocessor (running)                    [Jump]
```

**Click [Jump]** → scrolls the main node list to that node's card, selects it,
and opens its right panel. Unmanaged nodes (discovered in DDS but not in
`record.json`) are shown without a jump link.

**QoS badges** (compact inline after message type):
- `REL` / `BE` (reliability)
- `VOL` / `TL` (durability)
- `KL(N)` / `KA` (history keep_last/keep_all)

Hover shows full QoS detail in a tooltip (deadline, lifespan, liveliness).

**Infrastructure filtering**: Toggle to show/hide `/parameter_events`, `/rosout`,
`/tf`, `/tf_static`, `/clock`, `/diagnostics`. Hidden by default.

**State awareness**:
- Running/Loaded: fetch and display topics
- Containers: fetch and display (containers have DDS endpoints)
- Pending/Loading: "Waiting for node to start..."
- Stopped/Failed: "Node not running"

### C. Graph view — namespace-grouped interactive visualization

A new top-level view (alongside "Nodes" and "Diagnostics") called **"Graph"**.

Nodes are grouped by ROS namespace. At the top level, users see namespace
clusters with aggregated edge trunks between them. Clicking a namespace zooms
in to reveal sub-namespaces and individual nodes.

**Top level (zoomed out)**:

```
┌──────────────┐         ┌────────────────┐         ┌──────────────┐
│  /sensing    │  ====►  │  /perception   │  ====►  │  /planning   │
│  (12 nodes)  │         │  (28 nodes)    │         │  (35 nodes)  │
└──────────────┘         └────────────────┘         └──────────────┘
                                                          ║
                                                          ▼
                         ┌────────────────┐         ┌──────────────┐
                         │  /vehicle      │  ◄====  │  /control    │
                         │  (4 nodes)     │         │  (8 nodes)   │
                         └────────────────┘         └──────────────┘
```

Edge trunks show aggregated topic count (e.g., "12 topics" on hover). Trunk
thickness is proportional to topic count.

**Drill into a namespace** (click `/perception`):

```
┌─ /perception ─────────────────────────────────────────────┐
│                                                           │
│  ┌───────────────┐     ┌──────────────────┐              │
│  │ lidar_preproc │ ──► │ obstacle_detector │ ──► (exits) │
│  └───────────────┘     └──────────────────┘              │
│                              │                            │
│  ┌───────────────┐           ▼                            │
│  │ radar_detect  │ ──► ┌──────────────┐                  │
│  └───────────────┘     │ fusion_node  │ ──► (exits)      │
│                        └──────────────┘                   │
│                                                           │
│  ┌─ /perception/tracking ──────┐                         │
│  │  multi_object_tracker       │ ──► (exits)             │
│  │  (3 nodes)                  │                         │
│  └─────────────────────────────┘                         │
└───────────────────────────────────────────────────────────┘
```

Sub-namespaces appear as nested compound nodes (Cytoscape `parent` field).
Edges leaving the current scope show as `(exits)` stubs pointing to external
namespace badges.

**Interactions**:

| Action           | Result                                                     |
|------------------|------------------------------------------------------------|
| Click namespace  | Zoom into it, showing children (sub-namespaces + nodes)    |
| Click node       | Select it, open right panel with TopicsTab                 |
| Hover node       | Tooltip: pub/sub/srv counts, status, PID                   |
| Hover edge/trunk | Tooltip: topic names and message types                     |
| Drag node        | Reposition within the graph                                |
| Pan/zoom         | Navigate the graph canvas (mouse wheel + drag background)  |
| Breadcrumb       | `/` > `/perception` > `/perception/tracking` — navigate up |

**Cytoscape.js configuration**:
- Layout: `fcose` (fast compound spring embedder) — respects compound node
  boundaries, produces clean hierarchical layouts
- Edge style: `curve-style: 'taxi'` for orthogonal routing through groups
- Compound nodes: namespace groups use Cytoscape's `parent` field for automatic
  containment and bounding box calculation
- Expand/collapse: `cytoscape-expand-collapse` extension manages group
  state, creates meta-edges for collapsed groups
- Styling: node color by status (running=green, stopped=grey, failed=red),
  dangling edges highlighted amber

**Client-side graph construction** from `GET /api/graph`:

1. Parse GraphSnapshot JSON
2. Build namespace tree from node FQNs (split on `/`)
3. Create Cytoscape compound nodes for each namespace level
4. Create leaf nodes for individual ROS nodes
5. Aggregate edges: for each topic, draw edges from publishers to subscribers;
   when both endpoints are in collapsed namespaces, the expand-collapse
   extension automatically creates meta-edges between the namespace nodes
6. Edge labels: individual topic name when visible, aggregated count when
   namespaces are collapsed

---

## Implementation Phases

### Phase 25.1: Patch rclrs — Expose QoS in TopicEndpointInfo

**File**: `src/vendor/ros2_rust/rclrs/src/node/graph.rs`

Add `qos_profile` field to `TopicEndpointInfo`:

```rust
pub struct TopicEndpointInfo {
    pub node_name: String,
    pub node_namespace: String,
    pub topic_type: String,
    pub qos_profile: QoSProfile,  // NEW
}
```

Update the conversion from `rmw_topic_endpoint_info_t` (around line 391):

```rust
TopicEndpointInfo {
    node_name,
    node_namespace,
    topic_type,
    qos_profile: QoSProfile::from(&info.qos_profile),
}
```

**Workspace wiring**: Add to root `Cargo.toml`:

```toml
[patch.crates-io]
rclrs = { path = "src/vendor/ros2_rust/rclrs" }
```

Update `src/play_launch/Cargo.toml` to match the vendor version (`0.7`).

#### Work items

- [x] Add `qos_profile: QoSProfile` field to `TopicEndpointInfo` struct
- [x] Update `From<rmw_topic_endpoint_info_t>` conversion to populate `qos_profile`
- [x] Patch `ros2_cargo_config.toml` via `scripts/patch_cargo_config.sh` to point rclrs and rosidl_runtime_rs to vendored paths
- [x] Update rclrs version in `src/play_launch/Cargo.toml` from `0.6` to `0.7`
- [x] Vendor `rosidl_runtime_rs` at `src/vendor/rosidl_runtime_rs/` (0.6 source); patch colcon-generated message crates from `"0.5"` to `"0.6"` via `scripts/patch_cargo_config.sh`

#### Acceptance criteria

- [x] `cargo check -p play_launch` compiles with patched rclrs
- [x] `QoSProfile` fields (reliability, durability, history, deadline, lifespan, liveliness) are populated from `rmw_qos_profile_t`
- [x] Existing `just test` passes (no regressions)

---

### Phase 25.2: GraphBuilder (Rust)

**New file**: `src/play_launch/src/ros/graph_builder.rs`

#### Types

```rust
#[derive(Serialize)]
pub struct GraphSnapshot {
    pub topics: Vec<TopicGraph>,
    pub services: Vec<ServiceGraph>,
}

#[derive(Serialize)]
pub struct TopicGraph {
    pub name: String,
    pub msg_type: String,
    pub publishers: Vec<EndpointNode>,
    pub subscribers: Vec<EndpointNode>,
    pub dangling: bool,
}

#[derive(Serialize)]
pub struct EndpointNode {
    pub fqn: String,                      // fully-qualified node name
    pub member_name: Option<String>,      // play_launch member name (if managed)
    pub qos: Option<QosSummary>,          // from patched TopicEndpointInfo
}

#[derive(Serialize)]
pub struct QosSummary {
    pub reliability: String,   // "reliable" | "best_effort"
    pub durability: String,    // "volatile" | "transient_local"
    pub history: String,       // "keep_last(N)" | "keep_all"
    pub deadline_ms: Option<f64>,
    pub lifespan_ms: Option<f64>,
    pub liveliness: String,    // "automatic" | "manual_by_topic"
}

#[derive(Serialize)]
pub struct ServiceGraph {
    pub name: String,
    pub srv_type: String,
    pub servers: Vec<String>,  // FQNs
    pub clients: Vec<String>,  // FQNs
}

#[derive(Serialize)]
pub struct NodeTopics {
    pub publishers: Vec<NodeTopicEntry>,
    pub subscribers: Vec<NodeTopicEntry>,
    pub servers: Vec<NodeServiceEntry>,
    pub clients: Vec<NodeServiceEntry>,
}

#[derive(Serialize)]
pub struct NodeTopicEntry {
    pub name: String,
    pub msg_type: String,
    pub qos: Option<QosSummary>,
    pub publisher_count: usize,
    pub subscriber_count: usize,
    pub dangling: bool,
}

#[derive(Serialize)]
pub struct NodeServiceEntry {
    pub name: String,
    pub srv_type: String,
}
```

#### Methods

```rust
/// Build a full graph snapshot across all managed nodes.
pub fn build_snapshot(
    ros_node: &Arc<rclrs::Node>,
    fqn_map: &HashMap<String, String>,  // member_name -> FQN
) -> Result<GraphSnapshot>

/// Get topics/services for one specific node.
pub fn node_topics(
    ros_node: &Arc<rclrs::Node>,
    node_name: &str,
    namespace: &str,
) -> Result<NodeTopics>
```

**`build_snapshot` algorithm**:
1. `get_topic_names_and_types()` — all topics in the graph
2. For each topic: `count_publishers()`, `count_subscriptions()`,
   `get_publishers_info_by_topic()`, `get_subscriptions_info_by_topic()`
3. Build reverse map: FQN → member_name from `fqn_map`
4. Filter: exclude `rcl_interfaces/srv/*` parameter services from the
   service list (these are infrastructure, already handled by Phase 24)
5. Annotate `dangling` flag on each topic

**`node_topics` algorithm**:
1. Split FQN into (node_name, namespace)
2. `get_publisher_names_and_types_by_node()` + `get_subscription_names_and_types_by_node()`
3. `get_service_names_and_types_by_node()` + `get_client_names_and_types_by_node()`
4. For each topic: look up publisher/subscriber counts, compute `dangling`
5. For each topic this node publishes: get its QoS via
   `get_publishers_info_by_topic()` and find this node's endpoint

#### Work items

- [x] Define `GraphSnapshot`, `TopicGraph`, `EndpointNode`, `QosSummary` structs with `Serialize`
- [x] Define `ServiceGraph`, `NodeTopics`, `NodeTopicEntry`, `NodeServiceEntry` structs
- [x] Implement `build_graph_snapshot()` — iterate all topics, resolve endpoints, annotate dangling
- [x] Implement `build_node_topics()` — per-node pub/sub/srv/client query with dangling + QoS
- [x] Build reverse FQN → member_name map to label managed vs unmanaged nodes
- [x] Filter out `rcl_interfaces/srv/*` parameter services from service lists
- [x] Add `build_graph_snapshot()` and `get_node_topics()` methods to `MemberHandle`
- [x] Add `pub mod graph_builder;` to `src/play_launch/src/ros/mod.rs`

**Design note**: No server-side cache — rclrs graph queries are in-memory
microsecond lookups. Client-side debounced refetch (500ms) via SSE state events
in `store.js` serves the same purpose with less complexity.

#### Acceptance criteria

- [x] `build_graph_snapshot()` returns topics with correct publisher/subscriber lists
- [x] `dangling` flag is `true` when pub_count==0 or sub_count==0
- [x] Managed nodes have `member_name: Some(...)`, unmanaged nodes have `None`
- [x] QoS fields populated for each endpoint (reliability, durability, history, deadline, lifespan, liveliness)
- [x] Infrastructure services (`rcl_interfaces/srv/*`) excluded from service lists
- [x] `cargo check -p play_launch` compiles

---

### Phase 25.3: Web API endpoints

```rust
/// GET /api/graph
/// Returns GraphSnapshot — full topology for visualization
/// Cached, rebuilt on StateEvents (debounced 500ms)
pub async fn get_graph(...) -> Response

/// GET /api/nodes/:name/topics
/// Returns NodeTopics — per-node topic/service/client detail
pub async fn get_node_topics(...) -> Response
```

Register in `create_router()`.

#### Work items

- [x] Add `get_graph` handler — calls `MemberHandle::build_graph_snapshot()`
- [x] Add `get_node_topics` handler — calls `MemberHandle::get_node_topics()`
- [x] Register `GET /api/graph` route in `create_router()`
- [x] Register `GET /api/nodes/:name/topics` route in `create_router()`

#### Acceptance criteria

- [x] `GET /api/graph` returns 200 with valid `GraphSnapshot` JSON
- [x] `GET /api/graph` response includes topics with publishers, subscribers, dangling flags
- [x] `GET /api/graph` response includes services with servers and clients
- [x] `GET /api/nodes/:name/topics` returns 200 with `NodeTopics` JSON for a running node
- [x] `GET /api/nodes/:name/topics` returns 503 for node not running / FQN unknown

---

### Phase 25.4: TopicsTab frontend component

**New file**: `src/play_launch/src/web/assets/js/components/TopicsTab.js`

Click-to-expand topics, jump-to-node navigation. See [UI Design B](#b-topicstab-in-right-panel--click-to-expand-jump-to-node) above.

Register in RightPanel. Tab order:

```
Member Info | Params | Topics | stdout | stderr
```

#### Work items

- [x] Create `TopicsTab.js` component with Preact + htm
- [x] Fetch `GET /api/nodes/:name/topics` when tab is active and node is running
- [x] Render four collapsible sections: Publishers, Subscribers, Service Servers, Service Clients
- [x] Show section headers with counts (e.g., "PUBLISHERS (7)")
- [x] Each topic row shows: topic name, message type, endpoint count, dangling indicator
- [x] Clickable topic rows expand inline to show connected nodes list
  - [x] Expanded view shows subscribers (for published topics) or publishers (for subscribed topics)
  - [x] Connected node endpoints looked up from cached `graphSnapshot` signal
- [x] QoS badges rendered inline: `REL`/`BE`, `VOL`/`TL`, `KL(N)`/`KA`
- [x] `[Jump]` button on each managed connected node
  - [x] Click scrolls to node card in main list (via `scrollIntoView`)
  - [x] Sets `selectedNode` in store to open its right panel
- [x] Unmanaged nodes shown without `[Jump]` link
- [x] Dangling topics highlighted with `!` indicator
- [x] Infrastructure filter toggle (hide `/parameter_events`, `/rosout`, `/tf`, `/tf_static`, `/clock`, `/diagnostics`, `/_action`)
  - [x] Hidden by default, toggle to show
- [x] State awareness messages:
  - [x] Running/Loaded nodes: fetch and display topics
  - [x] Pending/Loading: show "Waiting for node to start..."
  - [x] Stopped/Failed: show "Node not running"
- [x] Register Topics tab in `RightPanel.js` between Params and stdout
- [x] Add TopicsTab CSS styles to `panels.css`

#### Acceptance criteria

- [x] TopicsTab appears as third tab in right panel (after Params, before stdout)
- [x] Selecting a running node shows its publishers and subscribers
- [x] Clicking a topic row expands to show connected nodes with status
- [x] Clicking `[Jump]` navigates to the target node card and selects it
- [x] Dangling topics visually distinct
- [x] QoS badges visible on each topic row
- [x] Infrastructure topics hidden by default; toggle makes them visible
- [x] Stopped/pending nodes show appropriate status message instead of topic list

---

### Phase 25.5: Node card topic badges

Add topic count badges to NodeCard. Counts derived from cached GraphSnapshot
(fetched once via `GET /api/graph`, refreshed on SSE state events).

```
3 pub  5 sub  2 srv  ⚠ 1 dangling
```

#### Work items

- [x] Add `graphSnapshot` signal to `store.js`
- [x] Add `fetchGraph()` and `debounceFetchGraph()` to store
- [x] Fetch `GET /api/graph` on initial load, SSE reconnect, and after SSE state events (500ms debounce)
- [x] Add `nodeTopicCounts` computed signal: derives `Map<memberName, {pub, sub, srv, dangling}>` from snapshot
- [x] Add badge row to `NodeCard.js` below the ROS name line
- [x] Show `N pub`, `N sub`, `N srv` as compact inline badges
- [x] Show `N dangling` in amber when dangling count > 0
- [x] Hide badge row when node is not running (no DDS endpoints)
- [x] Add CSS styles for `.topic-badges` in `nodes.css` (with dark theme support)

#### Acceptance criteria

- [x] Running nodes show topic count badges on their card
- [x] Badge counts derived from single cached `graphSnapshot` (no per-card API calls)
- [x] Dangling count highlighted in amber
- [x] Badges hidden for stopped/pending nodes
- [x] Badges update after state events via debounced graph refresh

---

### Phase 25.6: Vendor Cytoscape.js

Download and vendor into `src/play_launch/src/web/assets/js/vendor/`:

- `cytoscape.esm.min.js` (423 KB, native ESM from jsdelivr)
- `cytoscape-fcose.js` (124 KB, ESM bundle from esm.sh, cose-base bundled)
- `cytoscape-expand-collapse.js` (32 KB, ESM bundle from esm.sh)

Vendor script: `scripts/vendor_cytoscape.sh` — downloads pinned versions with
SHA-256 checksum verification, skips download if file already present with valid
checksum. Supports `--verify` (check only) and `--dry-run` (show new checksums).

**Versions**: cytoscape 3.33.1, cytoscape-fcose 2.2.0, cytoscape-expand-collapse 4.1.1.

**ESM strategy**: cytoscape ships native ESM. The two extensions (fcose,
expand-collapse) only ship UMD, so we use esm.sh's `?bundle-deps` ESM conversion
which also bundles internal dependencies (cose-base for fcose). All three files
export a default export and have no external import dependencies.

#### Work items

- [x] Download `cytoscape.esm.min.js` from jsdelivr (v3.33.1, native ESM)
- [x] Download `cytoscape-fcose` ESM bundle from esm.sh (v2.2.0, cose-base bundled)
- [x] Download `cytoscape-expand-collapse` ESM bundle from esm.sh (v4.1.1)
- [x] Place all three in `src/play_launch/src/web/assets/js/vendor/`
- [x] Create `scripts/vendor_cytoscape.sh` with checksum verification and skip-if-exists
- [x] Verify ES module imports work: `import cytoscape from './vendor/cytoscape.esm.min.js'`
- [x] Register fcose layout: `cytoscape.use(fcose)`
- [x] Register expand-collapse extension
- [x] Add Cytoscape.js MIT license text to `THIRD_PARTY_LICENSES`
- [x] Add cytoscape-fcose MIT license text to `THIRD_PARTY_LICENSES`
- [x] Add cytoscape-expand-collapse MIT license text to `THIRD_PARTY_LICENSES`

#### Acceptance criteria

- [x] All three vendor files present and importable as ES modules
- [x] `cytoscape()` creates a graph instance without errors
- [x] `fcose` layout runs on a simple test graph with compound nodes
- [x] `expand-collapse` extension loads and provides `api.collapseAll()` / `api.expandAll()`
- [x] `THIRD_PARTY_LICENSES` includes all three MIT license entries
- [x] `just build-wheel` bundles the vendor files into the wheel (rust-embed picks them up)

---

### Phase 25.7: GraphView frontend component

**New files**:
- `src/play_launch/src/web/assets/js/components/GraphView.js`
- `src/play_launch/src/web/assets/css/graph.css`

New top-level view (alongside "Nodes" and "Diagnostics"). See
[UI Design C](#c-graph-view--namespace-grouped-interactive-visualization) above.

#### Work items — Data pipeline

- [x] Fetch `GET /api/graph` (reuse `graphSnapshot` signal from store)
- [x] Build namespace tree from node FQNs (split on `/`, group recursively)
- [x] Create Cytoscape compound nodes for each namespace level (set `parent` field)
- [x] Create Cytoscape leaf nodes for individual ROS nodes
- [x] Create Cytoscape edges from topic publisher → subscriber relationships
- [x] Deduplicate edges: multiple topics between same node pair → single edge with count
- [x] Edge labels: topic name when single, "N topics" when aggregated

#### Work items — Layout and styling

- [x] Apply `fcose` layout with compound node constraints
- [x] Namespace compound nodes: subtle border, label showing namespace + node count
- [x] Leaf node styling: rounded rectangle, label = node name, color by status
  - [x] Running = green, stopped = grey, failed = red, loading = yellow
- [x] Edge styling: `curve-style: 'bezier'` (bezier instead of taxi — cleaner with fcose compound layout)
- [x] Dangling edges highlighted in amber
- [x] Trunk edge width proportional to aggregated topic count
- [x] Create `graph.css` with canvas container sizing, breadcrumb bar, controls

#### Work items — Interactions

- [x] Click namespace compound node → toggle expand/collapse via expand-collapse extension
- [x] Click leaf node → set `selectedNode` in store, open right panel with TopicsTab
- [x] Hover leaf node → tooltip showing pub/sub/srv counts, status, PID
- [x] Hover edge → tooltip showing topic names and message types
- [x] Drag nodes → reposition within the graph (Cytoscape default)
- [x] Pan (drag background) and zoom (mouse wheel) the canvas
- [x] Breadcrumb bar: `/` > `/perception` > `/perception/tracking`
  - [x] Click breadcrumb segment to navigate back to that namespace level
- [x] "Fit" button to reset zoom to fit all visible elements
- [x] "Collapse All" / "Expand All" buttons

#### Work items — View integration

- [x] Add "Graph" option to `currentView` signal in `store.js`
- [x] Add "Graph" button/tab to `Header.js` (alongside "Nodes" and "Diagnostics")
- [x] Conditionally render `GraphView` when `currentView === 'graph'`
- [x] Graph canvas resizes with window (responsive container)
- [x] Re-run layout when graph data changes (new snapshot from SSE events)

#### Acceptance criteria

- [x] Graph view accessible via "Graph" button in header
- [x] Top-level namespaces rendered as compound nodes with correct child counts
- [x] Edges between namespace groups represent aggregated topic connections
- [x] Click namespace → expands to show sub-namespaces and individual nodes
- [x] Click leaf node → right panel opens with that node's TopicsTab
- [x] Hover on node shows tooltip with pub/sub/srv counts and status
- [x] Hover on edge shows tooltip with topic names
- [x] Nodes are draggable; pan and zoom work smoothly
- [x] Breadcrumb bar shows current navigation path; clicking a segment navigates up
- [x] Dangling edges are visually distinct (amber color)
- [x] Graph updates when nodes start/stop (re-layout triggered by snapshot change)
- [x] "Fit" button resets view to show all visible elements
- [x] Canvas fills available space and resizes with window
- [x] Works with Autoware scale: 140 nodes, 512 topics render without freezing (fcose quality=default for >100 nodes)

---

### Phase 25.8: Hierarchical Edge Aggregation with Hub Nodes

**Status**: Complete

Replace individual per-topic edges with hub-based trunk routing. When namespaces
are collapsed, a single thick trunk edge connects namespace pairs instead of
hundreds of individual edges. Expanding a namespace reveals thin branch edges
fanning from leaf nodes to a central hub, with the trunk continuing to other
namespaces.

#### Edge types

| Type       | Connects                         | Appears when                        | Style                                      |
|------------|----------------------------------|-------------------------------------|--------------------------------------------|
| **Trunk**  | hub↔hub, hub↔collapsed-NS       | Cross-namespace traffic exists      | Thick (`2+log2(N)*1.5`, cap 8), arrow, 0.8 |
| **Branch** | leaf/collapsed-sub-NS → own hub  | Namespace expanded, external traffic| Thin (1px), dotted, no arrow, 0.35         |
| **Direct** | sibling↔sibling in same parent   | Both visible, same parent NS        | Normal (`1+log2(N)*0.8`, cap 5), arrow     |

Hub nodes are invisible 4×4px elements inside expanded namespaces. No hub is
created for root namespace — root-level elements are trunk endpoints directly.

The expand-collapse extension's meta-edge mechanism is bypassed: no edges are
loaded initially, so it finds nothing to reroute. After each expand/collapse,
`recomputeEdges()` clears all edges/hubs and rebuilds from raw data.

#### Work items

- [x] Split `buildCyElements` into `buildCyNodes` + `buildRawEdges`
- [x] Add `resolveVisible`, `parentNsOf`, `getCollapsedSet` helpers
- [x] Implement `recomputeEdges` core algorithm (direct/trunk/branch classification)
- [x] Add `rawEdgesRef` and wire into `useMemo` / main `useEffect`
- [x] Hook `recomputeEdges` into namespace tap, expandAll, collapseAll, breadcrumb
- [x] Update `getCyStyle`: add hub/trunk/branch/direct styles, remove meta-edge style
- [x] Suppress branch edge tooltips in mouseover handler

#### Acceptance criteria

- [ ] All namespaces collapsed: ~10-20 direct edges between top-level NS (not 500+)
- [ ] Expand one NS: branches fan from leaves to hub, trunk to other NS
- [ ] Expand both NS: branches on both sides, one trunk between hubs
- [ ] No `cy-expand-collapse-meta-edge` class appears in the graph
- [ ] Hub nodes invisible (not clickable, no tooltip)
- [ ] Trunk edges thick (indigo), branch edges thin/dotted, direct edges normal
- [ ] Tooltip on trunk/direct edges shows topic list; no tooltip on branch edges
- [ ] Existing interactions preserved: click node, breadcrumb, Fit, ExpandAll, CollapseAll

---

## Files to create/modify

| File                                                                    | Action                                            | Phase |
|-------------------------------------------------------------------------|---------------------------------------------------|-------|
| `src/vendor/ros2_rust/rclrs/src/node/graph.rs`                          | Patch: add `qos_profile` to `TopicEndpointInfo`   | 25.1  |
| `Cargo.toml` (root)                                                     | Add `[patch.crates-io]` for rclrs                 | 25.1  |
| `src/play_launch/Cargo.toml`                                            | Update rclrs version to match vendor              | 25.1  |
| `src/play_launch/src/ros/graph_builder.rs`                              | **New** — GraphBuilder, GraphSnapshot, NodeTopics | 25.2  |
| `src/play_launch/src/ros/mod.rs`                                        | Add `pub mod graph_builder;`                      | 25.2  |
| `src/play_launch/src/web/handlers.rs`                                   | Add `get_graph`, `get_node_topics`                | 25.3  |
| `src/play_launch/src/web/mod.rs`                                        | Register routes                                   | 25.3  |
| `src/play_launch/src/web/assets/js/components/TopicsTab.js`             | **New** — per-node topic detail                   | 25.4  |
| `src/play_launch/src/web/assets/js/components/RightPanel.js`            | Add Topics tab                                    | 25.4  |
| `src/play_launch/src/web/assets/css/panels.css`                         | TopicsTab styles                                  | 25.4  |
| `src/play_launch/src/web/assets/js/components/NodeCard.js`              | Add topic count badges                            | 25.5  |
| `src/play_launch/src/web/assets/js/store.js`                            | Add `graphSnapshot` signal, graph fetch logic     | 25.5  |
| `src/play_launch/src/web/assets/js/vendor/cytoscape.esm.min.js`         | **New** — vendored Cytoscape.js                   | 25.6  |
| `src/play_launch/src/web/assets/js/vendor/cytoscape-fcose.js`           | **New** — vendored layout extension               | 25.6  |
| `src/play_launch/src/web/assets/js/vendor/cytoscape-expand-collapse.js` | **New** — vendored group extension                | 25.6  |
| `src/play_launch/src/web/assets/js/vendor/THIRD_PARTY_LICENSES`         | Add Cytoscape MIT license                         | 25.6  |
| `scripts/vendor_cytoscape.sh`                                           | **New** — vendor download script with checksums   | 25.6  |
| `src/play_launch/src/web/assets/js/components/GraphView.js`             | **New** — namespace graph visualization + hub edges | 25.7, 25.8 |
| `src/play_launch/src/web/assets/js/components/Header.js`                | Add "Graph" view toggle                           | 25.7  |
| `src/play_launch/src/web/assets/js/app.js`                              | Conditionally render GraphView                    | 25.7  |
| `src/play_launch/src/web/assets/index.html`                             | Add `graph.css` link                              | 25.7  |
| `src/play_launch/src/web/assets/css/graph.css`                          | **New** — GraphView styles                        | 25.7  |

---

## Verification

### Build & test (automated)

- [x] `just build-rust` compiles with patched rclrs
- [x] `just test` passes (30 passed, 5 skipped — no regressions)
- [ ] `just build-wheel` produces a wheel containing all vendor files

### Manual testing (`just run-autoware`)

#### Node card badges (25.5)

- [ ] Running nodes show `N pub  N sub  N srv` badges below ROS name
- [ ] Dangling count shown as `⚠ N dangling` in amber when > 0
- [ ] Badges hidden for stopped/pending nodes
- [ ] Badges update within ~2s when a node starts or stops

#### TopicsTab (25.4)

- [ ] Topics tab appears between Params and stdout in right panel
- [ ] Selecting a running node lists its publishers and subscribers
- [ ] Selecting a container lists its DDS endpoints
- [ ] Click a topic row → expands to show connected nodes with status
- [ ] Click `[Jump]` → scrolls to target node card and selects it
- [ ] Dangling topics show amber `⚠ 0 sub` or `⚠ 0 pub`
- [ ] QoS badges (`REL`/`BE`, `VOL`/`TL`, `KL(N)`) visible; tooltip shows full detail
- [ ] Infrastructure topics hidden by default; toggle reveals them
- [ ] Stopped/pending nodes show status message instead of topic list

#### GraphView (25.7)

- [ ] "Graph" button visible in header alongside "Nodes" and "Diagnostics"
- [ ] Top-level view shows namespace compound nodes with node counts
- [ ] Edge trunks between namespaces show aggregated topic count on hover
- [ ] Click namespace → drills into sub-namespaces and individual nodes
- [ ] Click leaf node → right panel opens with TopicsTab for that node
- [ ] Hover node → tooltip with pub/sub/srv counts, status, PID
- [ ] Hover edge → tooltip with topic names and message types
- [ ] Nodes draggable; pan and zoom work
- [ ] Breadcrumb bar updates on drill-down; clicking navigates up
- [ ] "Fit" button resets zoom to fit all elements
- [ ] Dangling edges visually distinct (amber)
- [ ] Graph re-renders when nodes start/stop (snapshot refresh)

#### Hub Edge Aggregation (25.8)

- [ ] All namespaces collapsed: ~10-20 direct edges between top-level NS (not 500+)
- [ ] Expand one NS: branches fan from leaves to hub, trunk to other NS
- [ ] Expand both NS: branches on both sides, one trunk between hubs
- [ ] No `cy-expand-collapse-meta-edge` class appears in the graph
- [ ] Hub nodes invisible (not clickable, no tooltip)
- [ ] Trunk edges thick (indigo), branch edges thin/dotted, direct edges normal
- [ ] Tooltip on trunk/direct edges shows topic list; no tooltip on branch edges
- [ ] Existing interactions preserved: click node, breadcrumb, Fit, ExpandAll, CollapseAll

#### API endpoints (25.3)

- [ ] `curl http://127.0.0.1:8080/api/graph` returns valid JSON with topics and services
- [ ] `curl http://127.0.0.1:8080/api/nodes/<name>/topics` returns per-node topic detail
- [ ] Response latency < 50ms (cached snapshot)

---

### Phase 25.9: Graph View Overhaul — ELK Layout + Visual Optimizations

**Status**: Complete

Replaces fcose (force-directed) with ELK layered layout for cleaner hierarchical
flow. Simplifies edges (drops hubs), adds focus mode, edge visibility toggle, and
semantic zoom. Research into production tools (Kiali, Datadog, Grafana) informed
the design.

#### Changes

1. **ELK layered layout** — replaces fcose with elkjs v0.10.0 (EPL-2.0). Vendored
   as `elk.bundled.esm.js` (UMD → ESM wrapper). Custom ~80-line adapter in
   GraphView.js (`buildElkGraph` → `applyElkPositions` → `runElkLayout` with
   concurrency guard). Direction: LEFT→RIGHT, hierarchy handling: INCLUDE_CHILDREN.

2. **Simplified edges** — drops hub nodes and trunk/branch/direct classification.
   Single `recomputeEdges()` (~40 lines) aggregates raw edges by visible endpoint
   pair. One unified edge style with width proportional to `log2(topicCount)`.

3. **Focus mode** — double-click (`dbltap`) a leaf node to highlight its
   neighborhood. Non-neighbors receive `.faded` class (opacity 0.08). Double-click
   same node or click canvas to exit. Focus auto-exits on expand/collapse.

4. **Edge visibility toggle** — three-button group (All / Sel / None) between
   Infra toggle and Fit button. "Sel" mode shows only edges connected to selected
   nodes. Applied after each `recomputeEdges()` call and on node select/unselect.

5. **Semantic zoom** — zoom event listener classifies zoom level into bands:
   `full` (≥0.8), `reduced` (0.4–0.8), `minimal` (<0.4). Class-based styles
   adapt detail: minimal shrinks leaf nodes to 16px dots without labels, reduces
   edge width/opacity; reduced uses smaller font size.

#### Files modified

| File | Change |
|------|--------|
| `scripts/vendor_cytoscape.sh` | Remove fcose entry, add elk checksum verification |
| `src/.../vendor/elk.bundled.esm.js` | NEW — vendored elkjs v0.10.0 (~1.5 MB) |
| `src/.../vendor/cytoscape-fcose.js` | DELETED (126 KB saved) |
| `src/.../vendor/THIRD_PARTY_LICENSES` | Replace fcose MIT with elkjs EPL-2.0 |
| `src/.../js/components/GraphView.js` | Major rewrite: ELK layout, simplified edges, focus mode, edge toggle, semantic zoom |
| `src/.../css/graph.css` | Add `.graph-edge-toggle` button group styles |
| `docs/roadmap/phase-25-topic_introspection.md` | This section |

#### Work items

- [x] Download elkjs bundled ESM, add to vendor_cytoscape.sh with checksum
- [x] Add EPL-2.0 license to THIRD_PARTY_LICENSES
- [x] Delete cytoscape-fcose.js vendor file
- [x] Simplify `recomputeEdges()` — drop hubs, single aggregated edge type
- [x] Remove hub/trunk/branch/direct styles from `getCyStyle()`
- [x] Implement `buildElkGraph()` — cy→ELK hierarchy conversion
- [x] Implement `applyElkPositions()` — ELK result→cy position mapping
- [x] Implement `runElkLayout()` with concurrency guard
- [x] Make all layout call sites async
- [x] Add focus mode (dbltap handler, `.faded` class)
- [x] Add edge visibility toggle (state, toolbar UI, `applyEdgeVisibility`)
- [x] Add semantic zoom (zoom event, class-based style levels)
- [x] Add styles to graph.css (edge toggle buttons, active state)
- [x] Document Phase 25.9 in roadmap doc

---

### Phase 25.10: Graph View UX Fixes — Stability, Focus, Cue Buttons, Port Edges

**Status**: In progress

Four UX issues discovered during manual testing at Autoware scale.

#### Problem 1: Auto-expand on SSE updates

The main `useEffect` depends on `[nodeElements, rawEdges, view]`. When SSE snapshot
updates arrive, `nodeElements` changes → effect re-runs → `cy.json({ elements })`
replaces all elements → expand-collapse internal state destroyed → namespaces randomly
expand while the user does nothing.

**Fix**: Track initialization via `initializedRef`. First run does full element load +
collapse + layout. Subsequent SSE updates only update leaf node status/colors in-place.
If the node set changes (nodes appear/disappear), save collapsed set before `cy.json()`,
restore it after.

#### Problem 2: Clicking edge doesn't clear node highlight

The `tap` handler for highlight only listens on `'node'`. Clicking an edge doesn't
clear the previous node highlight.

**Fix**: Widen handler to `'node, edge'`. Clicking any element clears previous highlight
and highlights the clicked element + its connected neighborhood. Clicking canvas clears.

#### Problem 3: Accidental collapse on expanded namespace

Double-click anywhere on a large expanded namespace collapses it — easy to trigger
accidentally when trying to interact with nodes inside.

**Fix**: Remove `dbltap` handler on `node.namespace`. Enable expand-collapse extension's
built-in cue buttons (`cueEnabled: true`) — renders [+]/[−] icons in namespace corner.
Wire `expandcollapse.afterExpand/afterCollapse` events for edge recomputation and layout.
Root namespace protected from collapse.

#### Problem 4: Edge hairball when namespace expanded

Individual cross-namespace edges from 30+ nodes create a messy tangle that passes
through namespace backgrounds, making the graph unreadable.

**Fix**: Port-based edge bundling. When a namespace is expanded, cross-namespace edges
from internal nodes merge through a port node (small visible dot) inside the namespace.
One aggregated trunk edge per external target exits from the port. Internal edges
(both endpoints in the same NS) stay as direct lines. Rewrite `recomputeEdges()` to
classify internal vs cross-NS edges and create port/branch/trunk infrastructure.

#### Work items

- [ ] Add `initializedRef` to prevent SSE updates from replacing elements
- [ ] In-place status color updates for leaf nodes on subsequent SSE runs
- [ ] Save/restore collapsed set when node set changes
- [ ] Widen tap highlight handler to `'node, edge'` for unified focus
- [ ] Edge click: highlight edge + its two endpoints
- [ ] Remove `dbltap` handler on `node.namespace`
- [ ] Enable expand-collapse extension cue buttons (`cueEnabled: true`)
- [ ] Wire `afterExpand`/`afterCollapse` events for edge recompute + layout
- [ ] Protect root namespace from collapse in extension events
- [ ] Rewrite `recomputeEdges()` with port-based edge bundling
- [ ] Add port node creation for expanded namespaces with cross-NS traffic
- [ ] Add branch edges (internal node → port) and trunk edges (port → external)
- [ ] Add `node.port-node`, `edge.branch-edge`, `edge.trunk-edge` styles
- [ ] Double-click leaf node opens right panel (not single-click)
