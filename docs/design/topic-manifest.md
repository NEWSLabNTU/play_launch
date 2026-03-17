# Topic Manifest Design

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

The manifest describes the communication graph using four concepts:

1. **Topic** — a first-class entity: the communication channel. Carries the
   message type and QoS. Topics are the connection mechanism — publishers and
   subscribers on the same topic name are connected, just like in ROS 2.
   Services and actions are analogous first-class entities.

2. **Node** — a leaf execution entity. References topics by name
   (pub/sub lists). Does not carry type or QoS — those live on the topic.

3. **Component** — a composite entity containing nodes, topics, and
   sub-components. Has a namespace and an interface.

4. **Interface** — the subset of topics that cross a component's boundary.
   Topics are internal by default. Unconnected interface topics accumulate
   upward to the parent.

```
  Component "perception" (namespace: /perception)
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
1. CAPTURE   play_launch launch ... --save-manifest dir/
   Run → stabilize → snapshot graph → write manifest files

2. AUDIT     play_launch launch ... --manifest dir/manifest.yaml
   Run → periodically diff graph vs manifest → warn on deviations
```

## Format

### Quick Example

```yaml
version: 3

topics:
  chatter: std_msgs/msg/String                   # shorthand: type only, default QoS

nodes:
  talker:
    pub: [chatter]
  listener:
    sub: [chatter]

interface:
  pub: [chatter]
```

### Metadata

Root manifests only.

| Field              | Required | Description                                                        |
|--------------------|----------|--------------------------------------------------------------------|
| `version`          | yes      | Schema version (`3`)                                               |
| `captured_at`      | no       | ISO 8601 timestamp                                                 |
| `launch_file`      | no       | Source launch file (package/file or path)                          |
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
  /sensing/pointcloud:
    type: sensor_msgs/msg/PointCloud2
    qos:
      reliability: best_effort
      depth: 1

  /tf:
    type: tf2_msgs/msg/TFMessage
    qos: { reliability: reliable, durability: volatile, depth: 100 }
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

**Where to declare topics:**

| Scope                 | Declare in              | Example                      |
|-----------------------|-------------------------|------------------------------|
| Cross-component       | Root manifest `topics:` | `/sensing/pointcloud`, `/tf` |
| Internal to component | Component's `topics:`   | `cropped`, `filtered`        |

Every topic referenced by a node must be declared in the component's or an
ancestor's `topics:` section.

### Services and Actions

Same pattern as topics — declared with type, referenced by nodes.

```yaml
services:
  /driver/configure: std_srvs/srv/SetBool
  /driver/set_mode:
    type: custom_msgs/srv/SetMode
    # No QoS — services use ROS defaults (reliable, volatile)

actions:
  /navigator/navigate: nav2_msgs/action/NavigateToPose

nodes:
  driver:
    srv: [/driver/configure, /driver/set_mode]
  controller:
    cli: [/driver/configure]
  navigator:
    action_server: [/navigator/navigate]
  planner:
    action_client: [/navigator/navigate]
```

### Nodes

Nodes reference topics, services, and actions by name. All keys are optional.

```yaml
nodes:
  cropbox_filter:
    pub: [cropped]
    sub: [/sensing/pointcloud]

  fusion_node:
    pub: [fused_objects]
    sub: [lidar/objects, camera/objects]

  # Minimal — just registers the node's existence
  evaluator:
```

Node names are relative (prefixed by component namespace) or absolute
(leading `/`).

### Components

A component groups nodes, topics, and sub-components under a namespace.

```yaml
components:
  # Include from file
  perception:
    include: perception.topics.yaml
    namespace: /perception

  # Include from ROS package (reusable)
  front_camera:
    include: { pkg: camera_driver, file: camera_driver.topics.yaml }
    namespace: /sensing/camera/front

  # Same manifest, different namespace
  rear_camera:
    include: { pkg: camera_driver, file: camera_driver.topics.yaml }
    namespace: /sensing/camera/rear

  # Inline component
  safety:
    namespace: /system
    topics:
      heartbeat: std_msgs/msg/Bool
    nodes:
      watchdog:
        pub: [heartbeat]
    interface:
      pub: [heartbeat]

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

| ROS launch construct                            | Manifest equivalent                       |
|-------------------------------------------------|-------------------------------------------|
| `<group>` + `<push_ros_namespace ns="foo">`     | `components:` entry with `namespace: foo` |
| `<include namespace="foo">`                     | `include:` with `namespace: foo`          |
| `<group>` without namespace                     | Component without `namespace:`            |

### Interface

The interface declares which topics cross the component boundary. Topics
are **internal by default**.

```yaml
interface:
  pub: [tracked_objects, /tf]
  sub: [/sensing/pointcloud, /sensing/image]
  srv: [configure]
```

A simple list of topic names from the component's `topics:` section (or
absolute topics from an ancestor). From the parent, these are the component's
visible topics.

**Absolute topics** (`/tf`, `/clock`) bypass the interface — they are
globally visible and do not need to be listed.

### Interface Accumulation

A component's interface topic that is not published or subscribed by any
sibling entity in the parent **accumulates** to the parent's interface.

```yaml
# perception.topics.yaml
interface:
  sub: [/sensing/pointcloud]
  pub: [tracked_objects]           # relative → /perception/tracked_objects

# root manifest
components:
  sensing: ...
  perception:
    include: perception.topics.yaml
    namespace: /perception

# /sensing/pointcloud is published by sensing → consumed, does NOT accumulate
# /perception/tracked_objects is not consumed by any sibling → ACCUMULATES

# Effective root interface (accumulated):
interface:
  pub: [/perception/tracked_objects]
```

**Rules:**
1. Resolve each child's interface topics with its `namespace:`.
2. A child's interface topic that matches a sibling's topic (by resolved
   name) is **internal** — consumed within the parent.
3. A child's interface topic with **no matching sibling** accumulates to the
   parent's interface.
4. Absolute topics accumulate unchanged at every level.
5. Explicit `interface:` on the parent takes precedence over accumulation.

### Sync Policies

Declared inside components that contain multi-input fusion nodes.

```yaml
sync:
  object_fusion:
    inputs: [lidar/objects, camera/objects]    # topic names
    correlation: timestamp
    tolerance_ms: 50
    timeout_ms: 80
    on_drop: propagate     # propagate | skip | best_effort
```

- **`propagate`**: if any input drops, discard all correlated messages.
- **`skip`**: merge node skips that cycle, other branches unaffected.
- **`best_effort`**: use whatever is available (may use stale data).

### Requirements

Per-component latency and reliability contracts.

```yaml
requirements:
  latency_ms: 50
  drop_rate: 0.01
```

Requirements compose across pipelines: parallel branches contribute their
max latency (fork-join critical path).

## Pipeline Example

Autoware-like perception pipeline with branches, merge, and sync.

```
sensing ──→ lidar_perception ──┐
                               ├──→ fusion ──→ planning
sensing ──→ camera_perception ──┘
```

**Directory layout:**
```
manifests/autoware_launch/
├── planning_simulator.topics.yaml     ← root
├── sensing.topics.yaml
├── perception.topics.yaml
└── planning.topics.yaml
```

**`planning_simulator.topics.yaml`** (root):
```yaml
version: 3
launch_file: autoware_launch/planning_simulator.launch.xml
exclude_patterns: [/rosout, /parameter_events]

# Cross-component topics
topics:
  /sensing/pointcloud:
    type: sensor_msgs/msg/PointCloud2
    qos: { reliability: best_effort, depth: 1 }
  /sensing/image:
    type: sensor_msgs/msg/Image
    qos: { reliability: best_effort, depth: 1 }
  /perception/tracked_objects:
    type: autoware_perception_msgs/msg/TrackedObjects
    qos: { reliability: reliable, depth: 1 }
  /planning/trajectory:
    type: autoware_planning_msgs/msg/Trajectory
    qos: { reliability: reliable, durability: transient_local, depth: 1 }
  /tf:
    type: tf2_msgs/msg/TFMessage
    qos: { reliability: reliable, depth: 100 }

components:
  sensing:
    include: sensing.topics.yaml
    namespace: /sensing
  perception:
    include: perception.topics.yaml
    namespace: /perception
  planning:
    include: planning.topics.yaml
    namespace: /planning

interface:
  pub: [/planning/trajectory]
```

**`sensing.topics.yaml`**:
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

**`perception.topics.yaml`**:
```yaml
components:
  lidar:
    namespace: lidar
    topics:
      cropped: sensor_msgs/msg/PointCloud2
      no_ground: sensor_msgs/msg/PointCloud2
      objects: autoware_perception_msgs/msg/DetectedObjects
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

  camera:
    namespace: camera
    topics:
      rectified: sensor_msgs/msg/Image
      objects: autoware_perception_msgs/msg/DetectedObjects2D
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

  fusion:
    topics:
      fused: autoware_perception_msgs/msg/DetectedObjects
      tracked_objects: autoware_perception_msgs/msg/TrackedObjects
    nodes:
      fusion_node:
        sub: [/perception/lidar/objects, /perception/camera/objects]
        pub: [fused]
      tracker:
        sub: [fused]
        pub: [tracked_objects]
    sync:
      object_fusion:
        inputs: [/perception/lidar/objects, /perception/camera/objects]
        correlation: timestamp
        tolerance_ms: 50
        timeout_ms: 80
        on_drop: propagate
    interface:
      pub: [tracked_objects]
    requirements:
      latency_ms: 20

interface:
  pub: [fusion/tracked_objects]

requirements:
  latency_ms: 80
```

**`planning.topics.yaml`**:
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

### Manifest Association

A manifest is associated with a **top-level launch invocation**. Sub-files
are organizational splits, not independent manifests. Package-level manifests
(with relative names) can be reused via `include:` with `namespace:`.

### Resolution

Root manifest lookup:
```
1. --manifest /path/to/manifest.yaml      (explicit)
2. {launch_dir}/{stem}.topics.yaml        (sidecar)
3. ~/.config/play_launch/manifests/{pkg}/{file}.topics.yaml  (central store)
4. None                                   (auditing disabled)
```

Component `include:` paths resolve relative to the including file's directory.
Package includes (`pkg: ...`) resolve via `AMENT_PREFIX_PATH`.

### Splitting Guidelines

| Node count | Recommendation                         |
|-----------|----------------------------------------|
| 1-10      | Inline in parent                       |
| 10-25     | Own file, flat                         |
| 25+       | Own file + split into sub-components   |

## Capture and Audit

### Capture

`--save-manifest manifest.yaml` → single flat file.
`--save-manifest dir/` → directory tree split by namespace.

**Capture algorithm:**
1. Build `GraphSnapshot` via `build_graph_snapshot()`
2. Group nodes by namespace → components
3. For each component, strip namespace prefix → relative names
4. Collect topics: name, type, QoS from runtime graph
5. Classify topics: cross-component (root) vs internal (component)
6. Infer interface from topics with no internal counterpart
7. Write YAML

**Auto-captured:** topics (type + QoS), nodes (pub/sub lists), interface.
**User-authored:** sync policies, requirements, component restructuring.

### Audit

Auditing is **warn-only**. Deviations:

| Category             | Severity |
|----------------------|----------|
| New topic            | `warn`   |
| Missing topic        | `info` → `warn` after stabilization |
| New endpoint         | `warn`   |
| Missing endpoint     | `info`   |
| QoS mismatch         | `warn`   |
| QoS incompatible     | `error`  |
| Latency exceeded     | `warn`   |
| Sync violation        | `warn`   |

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

- **Blocking enforcement** via RCL interception (future).
- **Drop propagation enforcement** at the middleware level (future).
- **Semantic component extraction** (namespace-based splitting is mechanical;
  meaningful grouping is user-authored).
- **Reusable component manifests** with abstract port names — the manifest
  uses resolved topic names for direct ROS launch file mapping. Reusability
  via relative names + `namespace:` on include.
