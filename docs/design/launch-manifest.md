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

**Five concepts:**

1. **Node** — a leaf execution entity. Declares named endpoints
   (pub/sub/srv ports) with optional rate/jitter properties.
   Optionally declares causal `paths:` with timing constraints.

2. **Topic** — wires node endpoints together. Carries message type,
   QoS, and optional channel properties (rate, transport drops).

3. **Include** — a child scope (separate manifest or inline group).
   Name = ROS namespace. Has its own nodes, topics, and imports/exports.

4. **Imports / Exports** — the scope's boundary. Named groups of
   endpoints that parent scopes use to wire children together.

5. **Paths** — named causal relations (input→output) with timing
   constraints. Declared on nodes and scopes.

```
  Manifest for perception.launch.xml
  (loaded with namespace: /perception)
  ┌────────────────────────────────────────────────────┐
  │                                                    │
  │  ┌──────────────┐  cropped  ┌──────────────┐      │
  │  │ cropbox      │──────────→│ centerpoint  │      │
  │  │  sub:[input] │           │  pub:[objects]│      │
  │  └──────────────┘           └──────────────┘      │
  │       ↑                          │                │
  │   imports.input             exports.output        │
  └───────┬──────────────────────────┬────────────────┘
          │                          │
     parent wires via topic    parent wires via topic
```

Topics wire endpoints explicitly. Imports/exports define the scope
boundary — parent scopes connect them via topic declarations.

## Format

### Quick Example

```yaml
# Minimal manifest — no contracts
version: 1

nodes:
  talker:
    pub: [chatter]
  listener:
    sub: [chatter]

topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]

exports:
  output: [talker/chatter]
```

```yaml
# With contracts (rate, jitter, drop)
version: 1

nodes:
  talker:
    pub:
      chatter:
        min_rate_hz: 10
        jitter_ms: 5
  listener:
    sub:
      chatter:
        min_rate_hz: 10

topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
    rate_hz: 10

exports:
  output: [talker/chatter]
```

### Metadata

| Field              | Required | Description                                                        |
|--------------------|----------|--------------------------------------------------------------------|
| `version`          | yes      | Manifest format version (currently `1`)                            |
| `exclude_patterns` | no       | Topic prefixes to ignore (default: `/rosout`, `/parameter_events`) |

### Nodes

Nodes declare **endpoints** — named pub/sub/service/action ports.
Endpoint names are the node's pre-remap topic names (before launch
file `<remap>` is applied). Each endpoint must have a corresponding
`<remap from="...">` in the launch file; warn if missing.

Endpoint names must be **unique per node** across pub, sub, srv, cli.

Endpoints can be a plain list (no properties) or a map with optional
per-endpoint properties:

```yaml
nodes:
  # Map form — with endpoint properties
  lidar_driver:
    pub:
      pointcloud:
        min_rate_hz: 10
        jitter_ms: 5

  # Plain list — no properties
  cropbox_filter:
    pub: [output]
    sub: [input]

  # Sub endpoints with state/required markers
  ndt_scan_matcher:
    sub:
      sensor_points:
        min_rate_hz: 10
      initial_pose:
        required: true
      regularization_pose:
        state: true

  # With services
  map_loader:
    srv:
      get_map:
        max_latency_ms: 1000

  # Minimal — just registers existence
  evaluator:
```

**Subscriber endpoint properties** (all optional):

| Field            | Meaning                                            |
|------------------|----------------------------------------------------|
| `min_rate_hz`    | Floor — "I need at least this rate"                |
| `max_rate_hz`    | Ceiling — "I can't process faster" (burst prevention) |
| `state: true`    | Read-latest, not causal (breaks feedback cycles)   |
| `required: true` | Must receive at least once before operational      |

**Publisher endpoint properties** (all optional):

| Field          | Meaning                                          |
|----------------|--------------------------------------------------|
| `min_rate_hz`  | Floor — "I produce at least this fast"           |
| `max_rate_hz`  | Ceiling — "something is wrong if faster"         |
| `jitter_ms`    | Max deviation from ideal period (timer-driven)   |

**Service endpoint properties** (all optional):

| Field          | Meaning                                          |
|----------------|--------------------------------------------------|
| `max_latency_ms`   | Max request-to-response time                     |

### Composable Nodes

Composable nodes appear as regular nodes. The container is a deployment
detail — from the topic graph perspective, composable nodes publish and
subscribe like any other node. When a composable node is loaded into a
container declared in a different launch file, the node belongs to the
manifest of the launch file that contains `<load_composable_node>`.

### Topics

Topics wire node endpoints together. Each topic declares its type,
which endpoints publish to it, and which subscribe. Endpoints are
referenced as `node/endpoint` or `include_name/export_or_import_name`
for cross-scope wiring.

```yaml
topics:
  # Full form
  cropped:
    type: sensor_msgs/msg/PointCloud2
    pub: [cropbox_filter/output]
    sub: [ground_filter/input]
    qos:
      reliability: best_effort
      depth: 1
    rate_hz: 10
    drop: 1 / 100

  # Shorthand: type only (no wiring)
  debug_output: sensor_msgs/msg/PointCloud2
```

Topic names are **relative** — the parser applies the namespace from
the include context. The real topic name becomes `<ns>/topic_name`.

**Undeclared topics**: if a node endpoint is not wired by any topic in
the manifest, the auditor emits a warning (not an error). This allows
gradual adoption.

**QoS fields** (all optional — omitted = ROS defaults, not audited):

| Field         | Values                              |
|---------------|-------------------------------------|
| `reliability` | `reliable` \| `best_effort`         |
| `durability`  | `volatile` \| `transient_local`     |
| `depth`       | integer (history depth, keep\_last) |
| `history`     | `keep_last` \| `keep_all`           |
| `lifespan_ms` | integer                             |
| `liveliness`  | `automatic` \| `manual_by_topic`    |

**Channel properties** (all optional):

| Field      | Meaning                                              |
|------------|------------------------------------------------------|
| `rate_hz`  | Negotiated channel rate                              |
| `drop`     | Transport drop tolerance (`N / W` notation)          |

**Rate hierarchy**: `pub.min_rate_hz >= topic.rate_hz >= sub.min_rate_hz`.
The topic `rate_hz` is the negotiated agreement. Per-endpoint
`min_rate_hz` / `max_rate_hz` are optional overrides when sides differ.
Static check: `topic.rate_hz >= max(all sub.min_rate_hz)`.

**Drop notation**: `drop: 5 / 100` means "up to 5 drops per 100
messages." Full form:

```yaml
drop:
  max_count: 5 / 100
  max_consecutive: 3
```

### Services and Actions

Same pattern as topics — declared with type, wire endpoints.

```yaml
services:
  configure:
    type: std_srvs/srv/SetBool
    server: [driver/configure]
    client: [controller/configure]

actions:
  navigate:
    type: nav2_msgs/action/NavigateToPose
    server: [navigator/navigate]
    client: [planner/navigate]
```

### Imports and Exports

Imports and exports are the scope's boundary — named groups of
endpoints that parent scopes use to wire children together.

- **`exports:`** — publisher endpoints this scope provides to its parent
- **`imports:`** — subscriber endpoints this scope needs from its parent

```yaml
imports:
  raw_data: [cropbox_filter/input]

exports:
  detections: [centerpoint/objects]
```

**Cross-scope aggregation**: imports/exports can reference child
scope's imports/exports:

```yaml
imports:
  raw_data:
    - lidar/raw_data
    - camera/raw_data
exports:
  detections:
    - lidar/detections
    - camera/detections
```

**Constraint**: imports/exports in the same scope cannot reference each
other (prevents cycles). Deep paths (`lidar/cropbox_filter/output`)
are allowed but imports/exports are preferred for encapsulation.

**Unresolved imports**: if a parent scope does not wire a child's import
via a topic, the auditor warns "unresolved import."

### Includes

Includes represent `<include>` (separate manifest) or `<group>` blocks
(inline). The include **name is the ROS namespace** — it maps to the
`<push-ros-namespace>` in the launch file.

```yaml
includes:
  # External — loaded from separate manifest file
  lidar:
    manifest: tier4_perception_launch/lidar_perception.launch.yaml
  camera:
    manifest: tier4_perception_launch/camera_perception.launch.yaml

  # Inline — from <group> block
  safety:
    nodes:
      emergency_stop:
        pub: [stop_cmd]
        sub: [diagnostics]
    topics:
      stop_command:
        type: std_msgs/msg/Bool
        pub: [emergency_stop/stop_cmd]
    exports:
      commands: [emergency_stop/stop_cmd]
```

External includes use `manifest:` with `package/file.yaml` — resolved
as `<manifest_dir>/package/file.yaml`.

Inline includes have the same structure as a top-level manifest (minus
`version:`). They can contain `nodes:`, `topics:`, `includes:`,
`imports:`, `exports:`, and `paths:`.

Each include must correspond to exactly one `<push-ros-namespace>` in
the launch file.

### Global Topics

Absolute topic names (`/tf`, `/clock`) are references to the runtime
environment. Any scope can reference them in topic `pub:`/`sub:` lists
without local declaration. Optionally constrain their type and QoS:

```yaml
global_topics:
  /tf: { type: tf2_msgs/msg/TFMessage, qos: { reliability: reliable, depth: 100 } }
  /clock: { type: rosgraph_msgs/msg/Clock }
```

Any manifest can declare `global_topics:` — this allows inner launch
files to be launched directly without the top-level manifest.

### Causal Paths

The `paths:` section on a node or scope declares **named causal
paths** — explicit input→output relations with timing constraints.

Each path is a causal relation: "when input arrives, output is
produced." Paths are named (map keys) for diagnostic clarity.

```yaml
paths:
  localization:
    input: sensor_points
    output: [ndt_pose, ndt_pose_with_covariance]
    max_latency_ms: 50
    drop:
      max_count: 10 / 100
      max_consecutive: 5
  debug:
    input: sensor_points
    output: [exe_time_ms, transform_probability]
```

**Path fields:**

| Field              | Meaning                                            |
|--------------------|----------------------------------------------------|
| `input`            | Single endpoint or list of endpoints (from `sub:`)  |
| `output`           | List of endpoints (from `pub:`)                    |
| `max_latency_ms`   | Worst-case time from trigger to output publish     |
| `min_latency_ms`   | Best-case time (optional, anomaly detection)       |
| `max_age_ms`       | Max data age from original source (optional)       |
| `correlation`      | Multi-input matching: `timestamp` or `latest`      |
| `tolerance_ms`     | Max timestamp difference for correlation           |
| `drop`             | Drop tolerance (`N / W` or full form)              |

**Trigger is implicit** from path structure:
- 0 inputs → source/periodic (output-only path)
- 1 input → on_arrival
- N inputs → all_ready (barrier)

#### Node Examples

```yaml
nodes:
  # Simple pipe
  centerpoint:
    sub: [pointcloud]
    pub: [objects]
    paths:
      main: { input: pointcloud, output: [objects], max_latency_ms: 30 }

  # Fusion — all inputs, timestamp-correlated
  fusion_node:
    sub: [lidar_objects, camera_objects]
    pub: [fused]
    paths:
      fusion:
        input: [lidar_objects, camera_objects]
        output: [fused]
        correlation: timestamp
        tolerance_ms: 50
        max_latency_ms: 20
        drop:
          max_count: 10 / 100
          max_consecutive: 3

  # Timer-driven tracker — input is state
  tracker:
    sub:
      fused:
        state: true
    pub:
      tracked_objects:
        min_rate_hz: 10
        jitter_ms: 10
    paths:
      tracking:
        output: [tracked_objects]
        max_age_ms: 200

  # NDT with feedback cycle and map precondition
  ndt_scan_matcher:
    sub:
      sensor_points:
        min_rate_hz: 10
      initial_pose:
        required: true
      regularization_pose:
        state: true
    pub:
      ndt_pose:
        min_rate_hz: 10
      exe_time_ms:
    srv:
      trigger_node:
        max_latency_ms: 100
    paths:
      localization:
        input: sensor_points
        output: [ndt_pose]
        max_latency_ms: 50
        min_latency_ms: 10        # < 10ms is suspicious (stale cache?)
        drop:
          max_count: 10 / 100
          max_consecutive: 5
      debug:
        input: sensor_points
        output: [exe_time_ms]

  # Source — periodic output (no paths needed)
  lidar_driver:
    pub:
      pointcloud:
        min_rate_hz: 10
        jitter_ms: 5

```

### Scope Paths

A scope declares `paths:` at the top level using the same structure.
Paths reference import/export group names.

```yaml
imports:
  raw_data: [cropbox_filter/input]
exports:
  detections: [centerpoint/objects]

paths:
  main:
    input: raw_data
    output: [detections]
    max_latency_ms: 50
    max_age_ms: 120              # from original sensor to scope export
```

**Composition**: parent scope budgets compose from children's paths.
Fork-join follows the critical path:

```
perception
  max_latency_ms: 85
  = max(lidar:50, camera:30) + fusion:20 + tracker:15 - overlap
```

The scope tree from Phase 30 provides the composition hierarchy.

### Measurement Sources

| Metric     | Definition                                                | Source                      |
|------------|-----------------------------------------------------------|-----------------------------|
| Latency    | `t_pub - t_take(trigger_input)`                           | RCL interception timestamps |
| Sync       | `max(stamp_i) - min(stamp_i)`                             | RCL interception stamps     |
| Rate       | `t_pub[n] - t_pub[n-1]`                                   | Stats plugin                |
| Jitter     | `|interval - 1/rate_hz|`                                  | Stats plugin                |
| Age        | sum of chain latencies (static) or `t_pub - header.stamp` | Static / interception       |
| Node drop  | `1 - output_count / input_count`                          | Stats plugin                |
| Burst      | max consecutive missing outputs                           | Stats plugin                |
| Xport drop | `1 - sub_take_count / pub_count`                          | Stats plugin                |
| Burstiness | lag-1 autocorrelation, dispersion index, max run          | Stats plugin (always-on)    |

All measurements use the existing Phase 29 interception infrastructure.
Burstiness diagnostics are always-on (negligible cost) but reported
selectively — full detail for topics with `drop:` declared, discovery
alerts for undeclared topics with anomalies.

### Formal Foundations

The endpoint properties, paths, and topic rate/drop form an
assume-guarantee contract system. Node paths compose to scope paths
via series/parallel rules.

See `docs/design/contract-theory.md` for the formal foundations and
`docs/design/contract-verification.md` for implementation tooling.

## Pipeline Example

Autoware-like perception pipeline with branches, merge, and planning.

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

**`tier4_sensing_launch/sensing.launch.yaml`**:
```yaml
version: 1

nodes:
  lidar_driver:
    pub:
      pointcloud:
        min_rate_hz: 10
        jitter_ms: 5
  camera_driver:
    pub:
      image:
        min_rate_hz: 30
        jitter_ms: 3

topics:
  pointcloud:
    type: sensor_msgs/msg/PointCloud2
    pub: [lidar_driver/pointcloud]
    rate_hz: 10
  image:
    type: sensor_msgs/msg/Image
    pub: [camera_driver/image]
    rate_hz: 30

exports:
  pointcloud: [lidar_driver/pointcloud]
  image: [camera_driver/image]
```

**`tier4_perception_launch/lidar_perception.launch.yaml`**:
```yaml
version: 1

nodes:
  cropbox_filter:
    pub: [output]
    sub: [input]
    paths:
      main: { input: input, output: [output], max_latency_ms: 5 }
  ground_filter:
    pub: [output]
    sub: [input]
    paths:
      main: { input: input, output: [output], max_latency_ms: 15 }
  centerpoint:
    pub: [objects]
    sub: [pointcloud]
    paths:
      main: { input: pointcloud, output: [objects], max_latency_ms: 30 }

topics:
  cropped:
    type: sensor_msgs/msg/PointCloud2
    pub: [cropbox_filter/output]
    sub: [ground_filter/input]
  no_ground:
    type: sensor_msgs/msg/PointCloud2
    pub: [ground_filter/output]
    sub: [centerpoint/pointcloud]
  detected_objects:
    type: autoware_perception_msgs/msg/DetectedObjects
    pub: [centerpoint/objects]
    rate_hz: 10
    drop: 5 / 100

imports:
  raw_data: [cropbox_filter/input]
exports:
  detections: [centerpoint/objects]

paths:
  main: { input: raw_data, output: [detections], max_latency_ms: 50 }
```

**`tier4_perception_launch/camera_perception.launch.yaml`**:
```yaml
version: 1

nodes:
  rectifier:
    pub: [output]
    sub: [input]
    paths:
      main: { input: input, output: [output], max_latency_ms: 5 }
  yolo:
    pub: [objects]
    sub: [input]
    paths:
      main: { input: input, output: [objects], max_latency_ms: 25 }

topics:
  rectified:
    type: sensor_msgs/msg/Image
    pub: [rectifier/output]
    sub: [yolo/input]
  detected_objects:
    type: autoware_perception_msgs/msg/DetectedObjects2D
    pub: [yolo/objects]

imports:
  raw_data: [rectifier/input]
exports:
  detections: [yolo/objects]

paths:
  main: { input: raw_data, output: [detections], max_latency_ms: 30 }
```

**`tier4_perception_launch/perception.launch.yaml`**:
```yaml
version: 1

includes:
  lidar:
    manifest: tier4_perception_launch/lidar_perception.launch.yaml
  camera:
    manifest: tier4_perception_launch/camera_perception.launch.yaml

nodes:
  fusion_node:
    sub: [lidar_objects, camera_objects]
    pub: [fused_objects]
    paths:
      fusion:
        input: [lidar_objects, camera_objects]
        output: [fused_objects]
        correlation: timestamp
        tolerance_ms: 50
        max_latency_ms: 20
        drop:
          max_count: 10 / 100
          max_consecutive: 3
  tracker:
    sub:
      fused:
        state: true
    pub:
      tracked_objects:
        min_rate_hz: 10
        jitter_ms: 10
    paths:
      tracking:
        output: [tracked_objects]
        max_age_ms: 200

topics:
  lidar_objects:
    type: autoware_perception_msgs/msg/DetectedObjects
    pub: [lidar/detections]
    sub: [fusion_node/lidar_objects]
    rate_hz: 10
  camera_objects:
    type: autoware_perception_msgs/msg/DetectedObjects2D
    pub: [camera/detections]
    sub: [fusion_node/camera_objects]
    rate_hz: 10
  fused:
    type: autoware_perception_msgs/msg/DetectedObjects
    pub: [fusion_node/fused_objects]
    sub: [tracker/fused]
    qos: { reliability: reliable, depth: 1 }
  tracked:
    type: autoware_perception_msgs/msg/TrackedObjects
    pub: [tracker/tracked_objects]
    rate_hz: 10

imports:
  raw_data:
    - lidar/raw_data
    - camera/raw_data
exports:
  detections: [tracker/tracked_objects]

paths:
  main: { input: raw_data, output: [detections], max_latency_ms: 85, max_age_ms: 150 }
```

**`tier4_planning_launch/planning.launch.yaml`**:
```yaml
version: 1

nodes:
  prediction:
    sub: [tracked_objects]
    pub: [predicted_objects]
    paths:
      main: { input: tracked_objects, output: [predicted_objects], max_latency_ms: 35 }
  motion_planner:
    sub: [predicted_objects]
    pub: [trajectory]
    paths:
      main: { input: predicted_objects, output: [trajectory], max_latency_ms: 65 }

topics:
  predicted:
    type: autoware_perception_msgs/msg/PredictedObjects
    pub: [prediction/predicted_objects]
    sub: [motion_planner/predicted_objects]
    qos: { reliability: reliable, depth: 1 }
  trajectory:
    type: autoware_planning_msgs/msg/Trajectory
    pub: [motion_planner/trajectory]

imports:
  tracked_data: [prediction/tracked_objects]
exports:
  plan: [motion_planner/trajectory]

paths:
  main: { input: tracked_data, output: [plan], max_latency_ms: 100 }
```

**`autoware_launch/planning_simulator.launch.yaml`**:
```yaml
version: 1
exclude_patterns: [/rosout, /parameter_events]

global_topics:
  /tf: { type: tf2_msgs/msg/TFMessage, qos: { reliability: reliable, depth: 100 } }
  /clock: { type: rosgraph_msgs/msg/Clock }

includes:
  sensing:
    manifest: tier4_sensing_launch/sensing.launch.yaml
  perception:
    manifest: tier4_perception_launch/perception.launch.yaml
  planning:
    manifest: tier4_planning_launch/planning.launch.yaml

topics:
  pointcloud:
    type: sensor_msgs/msg/PointCloud2
    pub: [sensing/pointcloud]
    sub: [perception/raw_data]
  image:
    type: sensor_msgs/msg/Image
    pub: [sensing/image]
    sub: [perception/raw_data]
  tracked_objects:
    type: autoware_perception_msgs/msg/TrackedObjects
    pub: [perception/detections]
    sub: [planning/tracked_data]
  trajectory:
    type: autoware_planning_msgs/msg/Trajectory
    pub: [planning/plan]

exports:
  plan: [planning/plan]

paths:
  sensor_to_plan:
    input: [sensing/pointcloud, sensing/image]
    output: [plan]
    max_latency_ms: 200
    max_age_ms: 250              # includes cross-machine transport
```

### Latency Analysis

Each scope's paths are verified independently. The E2E critical
path is derived from the scope tree:

```
sensing (source, 10 Hz)
  → max(lidar: 50ms, camera: 30ms)
  → fusion: 20ms + tracker (periodic, 10 Hz)
  → planning: 100ms
Critical path: 50 + 20 + 100 = 170ms
```

No user-defined chains — the scope hierarchy IS the chain structure.

## References

- **AUTOSAR Timing Extensions** (R22-11): Event chains with fork/join, age
  and reaction constraints.
  ([spec](https://www.autosar.org/fileadmin/standards/R22-11/CP/AUTOSAR_TPS_TimingExtensions.pdf))
- **CARET**: Chain-Aware ROS 2 Evaluation Tool for cause-effect chain latency.
  ([paper](https://www.researchgate.net/publication/369815699))
- **ROS 2 `message_filters`**: ApproximateTimeSynchronizer pivot algorithm.
  ([docs](https://docs.ros.org/en/humble/p/message_filters/doc/index.html))
- **Prior art survey**: `docs/research/manifest-prior-art.md` — Jsonnet, CUE,
  K8s reconciliation, protobuf contracts, system_modes, CARET, ros2-performance.
- **Data quality semantics**: `docs/research/data-quality-semantics.md`

## Non-Goals (v4)

- **Automatic manifest resolution** (sidecar, central store, AMENT_PREFIX_PATH
  lookup) — for now, `--manifest-dir` is the only mechanism.
- **Blocking enforcement** via RCL interception (future).
- **User-defined chains** — replaced by scope-level I/O contracts that
  compose through the launch tree hierarchy.
- **Semantic component extraction** (namespace-based splitting is mechanical;
  meaningful grouping is user-authored).
