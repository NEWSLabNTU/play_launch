# Launch Manifest Design

## Problem

ROS 2 launch files declare which nodes to run but not which topics they
create. Topic creation happens in source code — publishers and subscribers
are invisible until runtime. The Autoware planning simulator launches ~110
nodes that collectively create ~500 topics. If a code change adds or removes
a topic, the divergence goes unnoticed unless someone manually checks.

play_launch already has runtime graph introspection (Phase 25) and RCL
interception (Phase 29). The missing piece is a **reference specification**
to compare against.

## Model

A manifest file describes what one launch file contributes to the
communication graph: its nodes and the topics they create. Manifest files
are organized by package and launch file in a manifest directory.

The parser loads manifests alongside launch files. When it encounters
`<include pkg="X" file="Y.launch.xml">`, it looks up
`<manifest_dir>/X/Y.yaml`. The parser applies the namespace from the
include context, so manifest files use relative names and are reusable
across different namespace contexts.

**Four concepts:**

1. **Topic** — a first-class entity: the communication channel. Carries
   message type and QoS. Connection is by name matching, same as ROS 2.
   Services and actions are analogous.

2. **Node** — a leaf execution entity. References topics by name (pub/sub
   lists). Type and QoS live on the topic, not the node.

3. **Component** — an optional organizational grouping within a manifest
   file (for `<group>` blocks). The launch file include tree provides the
   primary component structure — the parser handles it automatically.

4. **Interface** — the subset of topics that cross a launch file's
   boundary. Topics are internal by default. Interface topics accumulate
   upward through the include tree.

```
  Manifest for perception.launch.xml
  (loaded with namespace: /perception)
  ┌────────────────────────────────────────────────────┐
  │                                                    │
  │  topics:                                           │
  │    cropped: PointCloud2                            │
  │    objects: DetectedObjects                        │
  │                                                    │
  │  ┌──────────────┐          ┌──────────────┐        │
  │  │ cropbox      │ cropped  │ centerpoint  │        │
  │  │  pub:[cropped]│────────→│  sub:[cropped]│        │
  │  │  sub:[input] │         │  pub:[objects]│        │
  │  └──────────────┘          └──────────────┘        │
  │       ↑                          │                 │
  │  interface.sub                interface.pub         │
  └───────┬──────────────────────────┬─────────────────┘
          │                          │
     external input            external output
```

No explicit edges — the topic name IS the connection, matching ROS 2
semantics. QoS lives on the topic, the single source of truth.

## Workflow

```
1. CAPTURE
   play_launch launch <pkg> <launch_file> \
       --save-manifest-dir manifests/

   Run → stabilize → snapshot graph → write per-launch-file manifests

2. AUDIT
   play_launch launch <pkg> <launch_file> \
       --manifest-dir manifests/

   Parse → load manifests alongside includes → build expected graph
   Run → periodically diff actual vs expected → warn on deviations
```

## Format

### Quick Example

```yaml
# manifests/demo_nodes_cpp/talker_listener.launch.yaml
topics:
  chatter: std_msgs/msg/String

nodes:
  talker:
    pub: [chatter]
  listener:
    sub: [chatter]

interface:
  pub: [chatter]
```

### Metadata

Optional. Only meaningful in the top-level launch file's manifest.

| Field              | Required | Description                                                        |
|--------------------|----------|--------------------------------------------------------------------|
| `exclude_patterns` | no       | Topic prefixes to ignore (default: `/rosout`, `/parameter_events`) |

### Topics

Topics are first-class entities declared with type and optional QoS.
Two forms:

```yaml
topics:
  # Shorthand: type only, ROS default QoS
  cropped: sensor_msgs/msg/PointCloud2
  filtered: sensor_msgs/msg/PointCloud2
  objects: autoware_perception_msgs/msg/DetectedObjects

  # Full form: type + explicit QoS
  pointcloud:
    type: sensor_msgs/msg/PointCloud2
    qos:
      reliability: best_effort
      depth: 1
```

**QoS fields** (all optional — omitted fields use ROS defaults, not audited):

| Field          | Values                               |
|----------------|--------------------------------------|
| `reliability`  | `reliable` \| `best_effort`          |
| `durability`   | `volatile` \| `transient_local`      |
| `depth`        | integer (history depth, keep\_last)  |
| `history`      | `keep_last` \| `keep_all`            |
| `deadline_ms`  | integer                              |
| `lifespan_ms`  | integer                              |
| `liveliness`   | `automatic` \| `manual_by_topic`     |

All names are **relative** — the parser applies the namespace from the
launch file's include context. Use absolute names (leading `/`) only for
global topics like `/tf` or `/clock`.

Every topic referenced by a node must be declared in the same manifest's
`topics:` section or as an absolute topic from the runtime environment.

### Services and Actions

Same pattern as topics — declared with type, referenced by nodes.

```yaml
services:
  configure: std_srvs/srv/SetBool

actions:
  navigate: nav2_msgs/action/NavigateToPose

nodes:
  driver:
    srv: [configure]
  controller:
    cli: [configure]
  navigator:
    action_server: [navigate]
  planner:
    action_client: [navigate]
```

### Nodes

Nodes reference topics, services, and actions by name. All keys are optional.

```yaml
nodes:
  cropbox_filter:
    pub: [cropped]
    sub: [pointcloud]

  fusion_node:
    pub: [fused_objects]
    sub: [lidar_objects, camera_objects]

  # Minimal — just registers the node's existence
  evaluator:
```

Node names are relative (prefixed by the launch file's namespace context).

### Composable Nodes

Composable nodes (`<load_composable_node>`) appear as regular nodes in the
manifest. The container relationship is a deployment detail — from the topic
graph perspective, composable nodes publish and subscribe like any other node.

```yaml
# This launch file declares a container and loads composable nodes into it.
# Only the composable nodes need topic declarations — the container is a
# process host with no topics of its own.
nodes:
  cropbox_filter:         # composable node, loaded into some_container
    pub: [cropped]
    sub: [pointcloud]
  centerpoint:            # another composable node in the same container
    sub: [cropped]
    pub: [objects]
```

When a composable node is loaded into a container declared in a **different**
launch file (common in Autoware), the node belongs to the manifest of the
launch file that contains the `<load_composable_node>`, not the one that
declares the container.

### Components

Components are optional groupings **within** a single manifest file. They
correspond to `<group>` blocks in the launch file. For `<include>` blocks,
the parser handles the nesting automatically by looking up the included
launch file's manifest from the manifest directory.

```yaml
# Components for <group> blocks within this launch file
components:
  lidar:
    namespace: lidar
    topics:
      cropped: sensor_msgs/msg/PointCloud2
      objects: autoware_perception_msgs/msg/DetectedObjects
    nodes:
      cropbox_filter:
        sub: [pointcloud]
        pub: [cropped]
      centerpoint:
        sub: [cropped]
        pub: [objects]
    interface:
      pub: [objects]

  # Component without namespace (organizational grouping only)
  fusion_group:
    nodes:
      fusion_node: ...
    sync: ...
```

**Namespace rules** (same as ROS 2):
- `namespace:` is optional. If omitted, inherits parent's namespace.
- Absolute (leading `/`): replaces parent namespace.
- Relative: appended to parent namespace.

### Interface

The interface declares which topics cross the launch file's boundary.
Topics are **internal by default**. The interface is what the parent launch
file (the one that `<include>`s this one) can see.

```yaml
interface:
  pub: [tracked_objects]
  sub: [pointcloud, image]
  srv: [configure]
```

A simple list of topic names from the manifest's `topics:` section.

**Absolute topics** (`/tf`, `/clock`) are globally visible and do not need
to be listed in the interface.

### Interface Accumulation

When a launch file includes another launch file that has a manifest, the
included manifest's interface topics that are not consumed within the parent
**accumulate** to the parent's interface.

```
Parent launch file includes:
  - sensing.launch.xml (ns: /sensing)     → interface.pub: [pointcloud]
  - perception.launch.xml (ns: /perception) → interface.sub: [/sensing/pointcloud]
                                              interface.pub: [tracked_objects]

/sensing/pointcloud is published by sensing, subscribed by perception → internal
/perception/tracked_objects is not consumed by any sibling → accumulates to parent
```

**Rules:**
1. Resolve each included manifest's interface topics with its namespace.
2. An interface topic matched by a sibling (by resolved name) is **internal**.
3. An unmatched interface topic **accumulates** to the parent's interface.
4. Absolute topics accumulate unchanged at every level.

### Sync Policies

Declared inside components that contain multi-input fusion nodes.

```yaml
sync:
  object_fusion:
    inputs: [lidar_objects, camera_objects]
    correlation: timestamp
    tolerance_ms: 50
    timeout_ms: 80
    on_drop: propagate     # propagate | skip | best_effort
```

- **`propagate`**: if any input drops, discard all correlated messages.
- **`skip`**: merge node skips that cycle, other branches unaffected.
- **`best_effort`**: use whatever is available (may use stale data).

### Requirements

Quality contracts for timing and reliability, specified at per-node and
per-chain granularity. Based on AUTOSAR TIMEX semantics and CARET
measurement definitions.

See `docs/research/data-quality-semantics.md` for the full research survey.

#### Per-node contracts

Each node can declare an **assume-guarantee** timing contract:

```yaml
nodes:
  centerpoint:
    sub: [no_ground]
    pub: [objects]
    requirements:
      # Node latency: time from subscription callback to publish
      # Measured as: t_publish - t_subscribe (CARET definition)
      node_latency_ms: 30        # p99 budget
      # Maximum consecutive callbacks that miss the deadline
      max_deadline_misses: 3
```

#### Per-chain contracts (latency chains)

A latency chain defines an end-to-end path from stimulus to response,
with two AUTOSAR TIMEX metrics:

```yaml
chains:
  sensor_to_actuation:
    description: "Lidar to vehicle control command"
    stimulus: /sensing/pointcloud       # first topic
    response: /control/command          # last topic
    # MRT: worst-case time from new stimulus to first response
    max_reaction_time_ms: 100
    # MDA: max age of data used at response point
    max_data_age_ms: 50
    # Per-node budgets (optional, sum ≤ MRT)
    node_budgets:
      - node: /sensing/lidar/driver
        budget_ms: 5
      - node: /perception/centerpoint
        budget_ms: 30
      - node: /perception/tracker
        budget_ms: 20
      - node: /planning/motion_planner
        budget_ms: 35
      - node: /control/vehicle_cmd_gate
        budget_ms: 10
```

**Fork-join composition**: at a merge point, the critical path is the
slowest branch:

```
MRT = sensing (5ms) + max(lidar (50ms), camera (30ms)) + fusion (20ms) + planning (100ms)
    = 175ms
```

#### Reliability contracts

```yaml
reliability:
  /sensing/pointcloud:
    # Loss budget: fraction of published messages not received
    # Measured as: 1 - (take_count / pub_count) via StatsPlugin
    max_loss_ratio: 0.05           # 5% over sliding window
    window_size: 100               # evaluate over last 100 messages
    max_consecutive_drops: 2       # alert after 3 in a row

  /planning/trajectory:
    # Deadline: maximum inter-message interval
    # Aligns with DDS deadline QoS but monitored per-chain
    deadline_ms: 100
    max_deadline_misses: 0         # zero tolerance for control topics
```

#### Measurement sources

| Metric | Definition | Source |
|--------|-----------|--------|
| Node latency | `t_publish - t_subscribe` | RCL interception (frontier plugin timestamps) |
| Communication latency | `t_sub_callback - t_publish` | RCL interception (monotonic timestamps) |
| Chain MRT | Stimulus to first response | Frontier: sensor frontier vs control frontier |
| Chain MDA | Data age at response | Frontier: response timestamp - data generation timestamp |
| Loss ratio | `1 - take_count / pub_count` | Stats plugin per-topic counters |
| Inter-message jitter | `var(t[n+1] - t[n])` | Stats plugin monotonic timestamps |
| Deadline miss | `t[n+1] - t[n] > deadline` | Stats plugin |

All measurements use the existing Phase 29 interception infrastructure.
No new hooks needed — `FrontierPlugin` and `StatsPlugin` already capture
the required data.

## Pipeline Example

Autoware-like perception pipeline with branches, merge, and sync.

```
sensing.launch.xml ──→ lidar_perception.launch.xml ──┐
                                                      ├──→ fusion (inline)
sensing.launch.xml ──→ camera_perception.launch.xml ──┘
                                                      ──→ planning.launch.xml
```

**Manifest directory:**
```
manifests/
├── tier4_sensing_launch/
│   └── sensing.launch.yaml
├── tier4_perception_launch/
│   ├── perception.launch.yaml
│   ├── lidar_perception.launch.yaml
│   └── camera_perception.launch.yaml
├── tier4_planning_launch/
│   └── planning.launch.yaml
└── autoware_launch/
    └── planning_simulator.launch.yaml
```

**`autoware_launch/planning_simulator.launch.yaml`**:
```yaml
exclude_patterns: [/rosout, /parameter_events]

# Cross-launch-file topics declared here for QoS specification.
# These are referenced by absolute name in child manifests.
topics:
  /tf:
    type: tf2_msgs/msg/TFMessage
    qos: { reliability: reliable, depth: 100 }

interface:
  pub: [/planning/trajectory]
```

**`tier4_sensing_launch/sensing.launch.yaml`**:
```yaml
topics:
  pointcloud: sensor_msgs/msg/PointCloud2
  image: sensor_msgs/msg/Image

nodes:
  lidar/driver:
    pub: [pointcloud]
  camera/driver:
    pub: [image]

interface:
  pub: [pointcloud, image]

requirements:
  latency_ms: 5
```

**`tier4_perception_launch/perception.launch.yaml`**:
```yaml
# This launch file includes lidar_perception.launch.xml and
# camera_perception.launch.xml via <include>. The parser looks up
# their manifests automatically. Only inline content is declared here.

topics:
  fused:
    type: autoware_perception_msgs/msg/DetectedObjects
    qos: { reliability: reliable, depth: 1 }
  tracked_objects:
    type: autoware_perception_msgs/msg/TrackedObjects
    qos: { reliability: reliable, depth: 1 }

nodes:
  fusion_node:
    sub: [lidar/objects, camera/objects]
    pub: [fused]
  tracker:
    sub: [fused]
    pub: [tracked_objects]

sync:
  object_fusion:
    inputs: [lidar/objects, camera/objects]
    correlation: timestamp
    tolerance_ms: 50
    timeout_ms: 80
    on_drop: propagate

interface:
  pub: [tracked_objects]

requirements:
  latency_ms: 80
```

**`tier4_perception_launch/lidar_perception.launch.yaml`**:
```yaml
topics:
  cropped: sensor_msgs/msg/PointCloud2
  no_ground: sensor_msgs/msg/PointCloud2
  objects:
    type: autoware_perception_msgs/msg/DetectedObjects
    qos: { reliability: best_effort, depth: 1 }

nodes:
  cropbox_filter:
    sub: [/sensing/pointcloud]
    pub: [cropped]
  ground_filter:
    sub: [cropped]
    pub: [no_ground]
  centerpoint:
    sub: [no_ground]
    pub: [objects]

interface:
  pub: [objects]

requirements:
  latency_ms: 50
```

**`tier4_perception_launch/camera_perception.launch.yaml`**:
```yaml
topics:
  rectified: sensor_msgs/msg/Image
  objects:
    type: autoware_perception_msgs/msg/DetectedObjects2D
    qos: { reliability: best_effort, depth: 1 }

nodes:
  rectifier:
    sub: [/sensing/image]
    pub: [rectified]
  yolo:
    sub: [rectified]
    pub: [objects]

interface:
  pub: [objects]

requirements:
  latency_ms: 30
```

**`tier4_planning_launch/planning.launch.yaml`**:
```yaml
topics:
  predicted_objects:
    type: autoware_perception_msgs/msg/PredictedObjects
    qos: { reliability: reliable, depth: 1 }
  trajectory: autoware_planning_msgs/msg/Trajectory

nodes:
  prediction:
    sub: [/perception/tracked_objects]
    pub: [predicted_objects]
  motion_planner:
    sub: [predicted_objects]
    pub: [trajectory]

interface:
  pub: [trajectory]

requirements:
  latency_ms: 100
```

### Latency Analysis

```
sensing (5ms) → max(lidar (50ms), camera (30ms)) → fusion (20ms) → planning (100ms)
Critical path: 5 + 50 + 20 + 100 = 175ms
```

## File Organization

### Manifest Directory

Manifests are organized by package and launch file in a user-specified
directory:

```
<manifest_dir>/
├── <package_a>/
│   ├── <launch_file_1>.yaml
│   └── <launch_file_2>.yaml
├── <package_b>/
│   └── <launch_file_3>.yaml
└── ...
```

The user specifies the manifest directory via CLI flag:

```bash
# Audit with manifests
play_launch launch autoware_launch planning_simulator.launch.xml \
    --manifest-dir ./manifests

# Capture manifests from a running system
play_launch launch autoware_launch planning_simulator.launch.xml \
    --save-manifest-dir ./manifests
```

### Manifest Lookup

During parsing, when the parser encounters
`<include pkg="X" file="Y.launch.xml">`, it looks for:

```
<manifest_dir>/X/Y.yaml
```

If the file exists, the manifest is loaded and its names are resolved using
the namespace from the include context. If the file does not exist, the
include is processed normally without manifest auditing (partial coverage).

### Manifest Association

Each manifest file describes **one launch file**. The file path within the
manifest directory mirrors the ROS package structure:

| Launch file | Manifest file |
|---|---|
| `camera_driver/camera_driver.launch.xml` | `camera_driver/camera_driver.launch.yaml` |
| `tier4_perception_launch/perception.launch.xml` | `tier4_perception_launch/perception.launch.yaml` |
| `autoware_launch/planning_simulator.launch.xml` | `autoware_launch/planning_simulator.launch.yaml` |

The same manifest works for multiple instantiations of the same launch file
(e.g., `camera_driver.launch.xml` included twice under different namespaces).
The parser applies the namespace each time.

## Expected Graph in record.json

The parser builds the expected topic graph from manifest files and embeds it
into `record.json` as new optional fields. This maintains backward
compatibility — old `record.json` files without these fields work as before.
The executor enables auditing only when the fields are present.

```
Parser (with --manifest-dir)
  └── processes launch files + loads manifests → record.json
        ├── nodes, params, remaps, ...     (existing fields)
        ├── topics: { ... }                (new — from manifests)
        ├── services: { ... }              (new)
        ├── actions: { ... }               (new)
        └── manifest_sync: { ... }         (new — sync policies)

Executor
  ├── record.json → spawn nodes            (existing)
  └── record.json topics + GraphSnapshot → audit  (new, if fields present)
```

**New fields in record.json:**

| Field | Type | Description |
|-------|------|-------------|
| `topics` | map | Resolved topic name → `{ type, qos, publishers: [node_fqn], subscribers: [node_fqn] }` |
| `services` | map | Resolved service name → `{ type, servers: [node_fqn], clients: [node_fqn] }` |
| `actions` | map | Resolved action name → `{ type, servers: [node_fqn], clients: [node_fqn] }` |
| `manifest_sync` | map | Sync policy name → `{ inputs, correlation, tolerance_ms, timeout_ms, on_drop }` |
| `manifest_requirements` | map | Node FQN or manifest path → `{ latency_ms, drop_rate }` |

These fields are populated by the parser when `--manifest-dir` is provided.
Each node's existing record gains optional `pub`, `sub`, `srv`, `cli`,
`action_server`, `action_client` lists (topic/service/action names).

**Construction algorithm:**

1. Parser encounters `<include pkg="X" file="Y.launch.xml" ns="/foo">`.
2. Parser processes the launch file (existing → node records).
3. Parser looks up `<manifest_dir>/X/Y.yaml`.
4. If found, resolves relative names using namespace `/foo`.
5. Adds topic declarations to `record.json` `topics` map.
6. Annotates each node record with pub/sub/srv/cli lists.
7. Repeats recursively for nested includes.
8. After all includes: resolves connections (topic name matching),
   validates interface accumulation, checks QoS compatibility.
9. Writes single `record.json` with both execution plan and expected graph.

The expected graph embedded in `record.json` can be inspected, diffed
against previous versions, or consumed by external tools without running
the system.

## Capture and Audit

### Capture

```bash
play_launch launch autoware_launch planning_simulator.launch.xml \
    --save-manifest-dir ./manifests
```

The parser tracks which nodes originate from which `<include>`. After the
system stabilizes, the runtime graph is partitioned back into per-launch-file
manifests.

**Capture algorithm:**
1. Build `GraphSnapshot` via `build_graph_snapshot()`
2. Map each node to its originating launch file (from parser include tracking)
3. For each launch file, collect its nodes and their pub/sub topics
4. Strip namespace prefix → relative topic and node names
5. Collect topic declarations: name, type, QoS from runtime
6. Infer interface from topics with no internal counterpart
7. Write `<manifest_dir>/<package>/<launch_stem>.yaml`

**Auto-captured:** topics (type + QoS), nodes (pub/sub lists), interface.
**User-authored:** sync policies, requirements.

### Audit

Two-phase auditing, using the expected graph embedded in `record.json`:

**Phase 1 — Parse-time (static):**
- Parser builds expected graph from manifests into `record.json`
- Checks QoS compatibility between publishers and subscribers
- Validates interface consistency (declared vs accumulated)
- Reports issues before any node starts

**Phase 2 — Runtime (dynamic):**
- Executor reads `topics` field from `record.json` (if present)
- Periodically builds `GraphSnapshot` from the running system
- Diffs actual graph vs expected graph
- Catches topics created in code but not declared in manifests

| Category             | Phase   | Severity |
|----------------------|---------|----------|
| QoS incompatible     | parse   | `error`  |
| New topic            | runtime | `warn`   |
| Missing topic        | runtime | `info` → `warn` after stabilization |
| New endpoint         | runtime | `warn`   |
| Missing endpoint     | runtime | `info`   |
| QoS mismatch         | runtime | `warn`   |
| Latency exceeded     | runtime | `warn`   |
| Sync violation        | runtime | `warn`   |

Output: terminal log, `GET /api/manifest/diff`, `play_log/<ts>/manifest_audit.json`.

## References

- **AUTOSAR Timing Extensions** (R22-11): Event chains with fork/join, age
  and reaction constraints.
  ([spec](https://www.autosar.org/fileadmin/standards/R22-11/CP/AUTOSAR_TPS_TimingExtensions.pdf))
- **CARET**: Chain-Aware ROS 2 Evaluation Tool for cause-effect chain latency.
  ([paper](https://www.researchgate.net/publication/369815699))
- **Casini et al.** (ECRTS 2019): Response-time analysis of ROS 2 processing
  chains under reservation-based scheduling.
- **ROS 2 `message_filters`**: ApproximateTimeSynchronizer pivot algorithm.
  ([docs](https://docs.ros.org/en/humble/p/message_filters/doc/index.html))
- **ros-plan**: System description format with sockets, links, and QoS.
  ([repo](https://github.com/tnagyzambo/ros-plan))

## Non-Goals (v3)

- **Automatic manifest resolution** (sidecar, central store, AMENT_PREFIX_PATH
  lookup) — for now, `--manifest-dir` is the only mechanism.
- **Blocking enforcement** via RCL interception (future).
- **Drop propagation enforcement** at the middleware level (future).
- **Semantic component extraction** (namespace-based splitting is mechanical;
  meaningful grouping is user-authored).
