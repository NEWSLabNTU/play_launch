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

pub_groups:
  output: [talker/chatter]
```

### Metadata

| Field              | Required | Description                                                        |
|--------------------|----------|--------------------------------------------------------------------|
| `version`          | yes      | Manifest format version (currently `1`)                            |
| `exclude_patterns` | no       | Topic prefixes to ignore (default: `/rosout`, `/parameter_events`) |

### Nodes

Nodes declare their **endpoints** — named pub/sub/service/action ports.
Endpoint names are the node's pre-remap topic names (before launch file
`<remap>` is applied). Each endpoint must have a corresponding
`<remap from="...">` in the launch file; warn if missing.

```yaml
nodes:
  cropbox_filter:
    pub: [output]             # pre-remap pub endpoint names
    sub: [input]              # pre-remap sub endpoint names

  fusion_node:
    pub: [fused_objects]
    sub: [lidar_objects, camera_objects]

  driver:
    pub: [pointcloud]
    srv: [configure]          # service server endpoint
    cli: [get_map]            # service client endpoint

  # Minimal — just registers the node's existence
  evaluator:
```

Node names are relative (prefixed by the launch file's namespace).

Endpoints are **not** topic names — they are the node's internal port
names. The `topics:` section assigns real topic names and wires
endpoints together.

### Composable Nodes

Composable nodes appear as regular nodes. The container is a deployment
detail — from the topic graph perspective, composable nodes publish and
subscribe like any other node. When a composable node is loaded into a
container declared in a different launch file, the node belongs to the
manifest of the launch file that contains `<load_composable_node>`.

### Topics

Topics wire node endpoints together. Each topic declares its type,
which endpoints publish to it, and which subscribe.

```yaml
topics:
  # Full form: type + endpoints + optional QoS and contract
  cropped:
    type: sensor_msgs/msg/PointCloud2
    pub: [cropbox_filter/output]
    sub: [ground_filter/input]
    qos:
      reliability: best_effort
      depth: 1
    contract:
      rate_hz: 10
      deadline_ms: 150

  # Shorthand: type only (no wiring — for undeclared topic tolerance)
  debug_output: sensor_msgs/msg/PointCloud2
```

Topic names are **relative** — the parser applies the namespace from
the launch file's include context. Absolute names (leading `/`) are
references to the runtime environment (e.g., `/tf`, `/clock`) and
do not need local declaration.

**Undeclared topics**: if a node endpoint is not wired by any topic in
the manifest, the auditor emits a warning (not an error). This allows
gradual adoption — you don't have to declare every topic on day one.

**QoS fields** (all optional — omitted = ROS defaults, not audited):

| Field         | Values                              |
|---------------|-------------------------------------|
| `reliability` | `reliable` \| `best_effort`         |
| `durability`  | `volatile` \| `transient_local`     |
| `depth`       | integer (history depth, keep\_last) |
| `history`     | `keep_last` \| `keep_all`           |
| `deadline_ms` | integer                             |
| `lifespan_ms` | integer                             |
| `liveliness`  | `automatic` \| `manual_by_topic`    |

**Topic contract** (channel-level timing, optional):

| Field                   | Meaning                                               |
|-------------------------|-------------------------------------------------------|
| `rate_hz`               | Expected message frequency                            |
| `deadline_ms`           | Max gap between consecutive messages                  |
| `max_drop_ratio`        | Max fraction of messages lost over `window`           |
| `max_consecutive_drops` | Alert after N consecutive missed messages             |
| `window`                | Sliding window for drop ratio (message count)         |

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

### Endpoint Groups

Endpoint groups aggregate node endpoints (and child scope groups) into
named virtual endpoints on the scope. Parent scopes reference these
groups to wire child scopes together.

```yaml
sub_groups:
  input: [cropbox_filter/input]

pub_groups:
  output: [centerpoint/objects]
  all_debug: [cropbox_filter/debug, ground_filter/debug]
```

**Cross-scope references**: groups can reference subscope groups:

```yaml
pub_groups:
  all_detections:
    - lidar_perception/output        # child scope's pub_group
    - camera_perception/output       # another child's pub_group
```

**Constraint**: groups in the same scope cannot reference each other
(prevents cycles). A group is a virtual endpoint on the scope — parent
scopes access child content only through groups.

### Groups (Subscopes)

Groups represent `<group>` blocks (inline) or `<include>` files (separate
manifests). Each group has its own namespace prefix (one
`<push-ros-namespace>` per group). Groups have the same structure as
a top-level manifest — they can contain nodes, topics, nested groups,
and endpoint groups.

```yaml
groups:
  # Loaded from separate manifest file (via <include>)
  lidar_perception:

  # Inline group (from <group> block with <push-ros-namespace>)
  safety_group:
    nodes:
      emergency_stop:
        pub: [stop_cmd]
        sub: [diagnostics]
    topics:
      stop_command:
        type: std_msgs/msg/Bool
        pub: [emergency_stop/stop_cmd]
    pub_groups:
      commands: [emergency_stop/stop_cmd]
```

### Global Topics

Absolute topic names (`/tf`, `/clock`) are references to the runtime
environment. Any scope can reference them in topic `pub:`/`sub:` lists
without local declaration. The top-level manifest can optionally
constrain their type and QoS:

```yaml
# Top-level manifest only
global_topics:
  /tf: { type: tf2_msgs/msg/TFMessage, qos: { reliability: reliable, depth: 100 } }
  /clock: { type: rosgraph_msgs/msg/Clock }
```

If an inner launch file is launched directly (without the top-level),
it can declare its own `global_topics:`.

### Node I/O Contract

The `io:` block on a node describes its **computation contract** — how
it transforms inputs to outputs. It references the node's own endpoint
names (from `pub:` / `sub:`). No redundancy — `io:` only adds timing
and role annotations.

#### Primitives

All fields optional — specify only what you want to audit.

**1. Input roles** — annotate endpoints from the node's `sub:` list.

| Per-input property | Values              | Default                    | Meaning                                                   |
|--------------------|---------------------|----------------------------|-----------------------------------------------------------|
| `role`             | `trigger` / `state` | `trigger`                  | Does arrival fire computation? State = read-latest.       |
| `consume`          | integer             | 1                          | Messages consumed per firing (SDF-style rate).            |
| `min_count`        | integer             | 1 for trigger, 0 for state | Readiness precondition — min messages before operational. |

Shorthand: `state: [...]` marks those sub endpoints as state.
`required: [...]` marks state endpoints with `min_count: 1`.

**2. Trigger** — what causes output to be produced.

| Value              | Meaning                                               |
|--------------------|-------------------------------------------------------|
| `on_arrival`       | Any trigger-input arrival fires computation (default) |
| `all_ready`        | All trigger-inputs must arrive before firing          |
| `periodic: <hz>`   | Timer-driven at the given frequency                   |

**3. Correlation** — how multiple trigger-inputs are associated.

| Value       | Meaning                                        |
|-------------|------------------------------------------------|
| `timestamp` | Match by `header.stamp` within `tolerance_ms`  |
| `latest`    | Always use most recent of each (no sync)       |

**4. Timing** — bounds on trigger-to-output.

| Field          | Meaning                                           |
|----------------|---------------------------------------------------|
| `latency_ms`   | Max time from trigger to output publish           |
| `freshness_ms` | Max age of input data at output time (event-time) |

#### Examples

```yaml
nodes:
  # Simple pipe — endpoint names match sub:/pub:
  centerpoint:
    sub: [pointcloud]
    pub: [objects]
    io: { latency_ms: 30 }

  # Fusion — all inputs must arrive, timestamp-correlated
  fusion_node:
    sub: [lidar_objects, camera_objects]
    pub: [fused]
    io:
      trigger: all_ready
      correlation: timestamp
      tolerance_ms: 50
      latency_ms: 20

  # Timer-driven tracker — inputs are state
  tracker:
    sub: [fused]
    pub: [tracked_objects]
    io:
      state: [fused]
      trigger: { periodic: 10 }
      freshness_ms: 200

  # NDT with feedback cycle and map precondition
  ndt_scan_matcher:
    sub: [points_raw, ekf_pose]
    pub: [ndt_pose]
    io:
      state: [ekf_pose]
      required: [map]
      latency_ms: 50

  # Source — no inputs, periodic output
  lidar_driver:
    pub: [pointcloud]
    io:
      trigger: { periodic: 10 }

  # Gateway — only deadline on outputs
  vehicle_cmd_gate:
    sub: [auto_control_cmd, remote_control_cmd]
    pub: [control_cmd, gear_cmd]
    io:
      deadline_ms: 100

  # Accumulator — consume 10 scans before firing
  voxel_accumulator:
    sub: [pointcloud]
    pub: [dense_cloud]
    io:
      inputs:
        pointcloud: { consume: 10 }
      latency_ms: 500
```

### Scope I/O Contract

A scope contract describes the E2E timing of the entire launch file
or group. It references the scope's own endpoint groups.

```yaml
sub_groups:
  input: [cropbox_filter/input]
pub_groups:
  output: [centerpoint/objects]

io:
  latency_ms: 50               # E2E: input group → output group
```

**Composition**: parent scope budgets compose from children's contracts.
Fork-join follows the critical path:

```
perception.launch.xml
  latency_ms: 85
  = max(lidar:50, camera:30) + fusion:20 + tracker:15 - overlap
```

The scope tree from Phase 30 provides the composition hierarchy.

### Measurement Sources

| Metric         | Definition                                        | Source                          |
|----------------|---------------------------------------------------|---------------------------------|
| Latency        | `mono_publish(output) - mono_take(trigger_input)` | RCL interception timestamps     |
| Sync tolerance | `max(stamp_i) - min(stamp_i)`                     | RCL interception `header.stamp` |
| Rate           | `mono_pub[n] - mono_pub[n-1]`                     | Stats plugin publish timestamps |
| Freshness      | `mono_publish - stamp(oldest_input)`              | Frontier plugin                 |
| Drop ratio     | `1 - output_count / input_count`                  | Stats plugin per-topic counters |
| Deadline miss  | `t[n] - t[n-1] > deadline`                        | Stats plugin                    |

All measurements use the existing Phase 29 interception infrastructure.

### Contract Theory

The `io:` blocks (node-level) and `contract:` blocks (topic-level) form
an assume-guarantee contract system. Node contracts compose to scope
contracts via series/parallel rules. Topic contracts are the channel-
level quality bounds.

See `docs/design/contract-theory.md` for the full formalization.

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
    pub: [pointcloud]
    io: { trigger: { periodic: 10 } }
  camera_driver:
    pub: [image]
    io: { trigger: { periodic: 30 } }

topics:
  pointcloud:
    type: sensor_msgs/msg/PointCloud2
    pub: [lidar_driver/pointcloud]
    contract: { rate_hz: 10 }
  image:
    type: sensor_msgs/msg/Image
    pub: [camera_driver/image]
    contract: { rate_hz: 30 }

pub_groups:
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
    io: { latency_ms: 5 }
  ground_filter:
    pub: [output]
    sub: [input]
    io: { latency_ms: 15 }
  centerpoint:
    pub: [objects]
    sub: [pointcloud]
    io: { latency_ms: 30 }

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
    contract: { rate_hz: 10, max_drop_ratio: 0.05 }

sub_groups:
  input: [cropbox_filter/input]
pub_groups:
  output: [centerpoint/objects]

io: { latency_ms: 50 }
```

**`tier4_perception_launch/camera_perception.launch.yaml`**:
```yaml
version: 1

nodes:
  rectifier:
    pub: [output]
    sub: [input]
    io: { latency_ms: 5 }
  yolo:
    pub: [objects]
    sub: [input]
    io: { latency_ms: 25 }

topics:
  rectified:
    type: sensor_msgs/msg/Image
    pub: [rectifier/output]
    sub: [yolo/input]
  detected_objects:
    type: autoware_perception_msgs/msg/DetectedObjects2D
    pub: [yolo/objects]

sub_groups:
  input: [rectifier/input]
pub_groups:
  output: [yolo/objects]

io: { latency_ms: 30 }
```

**`tier4_perception_launch/perception.launch.yaml`**:
```yaml
version: 1

groups:
  lidar_perception:       # loaded from lidar_perception.launch.yaml
  camera_perception:      # loaded from camera_perception.launch.yaml

nodes:
  fusion_node:
    pub: [fused_objects]
    sub: [lidar_objects, camera_objects]
    io:
      trigger: all_ready
      correlation: timestamp
      tolerance_ms: 50
      latency_ms: 20
  tracker:
    pub: [tracked_objects]
    sub: [fused]
    io:
      state: [fused]
      trigger: { periodic: 10 }
      freshness_ms: 200

topics:
  lidar_objects:
    type: autoware_perception_msgs/msg/DetectedObjects
    pub: [lidar_perception/output]             # child pub_group
    sub: [fusion_node/lidar_objects]
  camera_objects:
    type: autoware_perception_msgs/msg/DetectedObjects2D
    pub: [camera_perception/output]            # child pub_group
    sub: [fusion_node/camera_objects]
  fused:
    type: autoware_perception_msgs/msg/DetectedObjects
    pub: [fusion_node/fused_objects]
    sub: [tracker/fused]
    qos: { reliability: reliable, depth: 1 }
  tracked:
    type: autoware_perception_msgs/msg/TrackedObjects
    pub: [tracker/tracked_objects]
    contract: { rate_hz: 10 }

sub_groups:
  input:
    - lidar_perception/input                   # child sub_groups
    - camera_perception/input
pub_groups:
  output: [tracker/tracked_objects]

io: { latency_ms: 85 }
```

**`tier4_planning_launch/planning.launch.yaml`**:
```yaml
version: 1

nodes:
  prediction:
    pub: [predicted_objects]
    sub: [tracked_objects]
    io: { latency_ms: 35 }
  motion_planner:
    pub: [trajectory]
    sub: [predicted_objects]
    io: { latency_ms: 65 }

topics:
  predicted:
    type: autoware_perception_msgs/msg/PredictedObjects
    pub: [prediction/predicted_objects]
    sub: [motion_planner/predicted_objects]
    qos: { reliability: reliable, depth: 1 }
  trajectory:
    type: autoware_planning_msgs/msg/Trajectory
    pub: [motion_planner/trajectory]

sub_groups:
  input: [prediction/tracked_objects]
pub_groups:
  output: [motion_planner/trajectory]

io: { latency_ms: 100 }
```

**`autoware_launch/planning_simulator.launch.yaml`**:
```yaml
version: 1
exclude_patterns: [/rosout, /parameter_events]

global_topics:
  /tf: { type: tf2_msgs/msg/TFMessage, qos: { reliability: reliable, depth: 100 } }
  /clock: { type: rosgraph_msgs/msg/Clock }

groups:
  sensing:
  perception:
  planning:

topics:
  pointcloud:
    type: sensor_msgs/msg/PointCloud2
    pub: [sensing/pointcloud]              # sensing's pub_group
    sub: [perception/input]                # perception's sub_group (→lidar path)
  image:
    type: sensor_msgs/msg/Image
    pub: [sensing/image]
    sub: [perception/input]                # perception's sub_group (→camera path)
  tracked_objects:
    type: autoware_perception_msgs/msg/TrackedObjects
    pub: [perception/output]
    sub: [planning/input]
  trajectory:
    type: autoware_planning_msgs/msg/Trajectory
    pub: [planning/output]

pub_groups:
  output: [planning/output]
```

### Latency Analysis

Each scope's `io.latency_ms` is verified independently. The E2E
critical path is derived from the scope tree:

```
sensing (source, 10 Hz)
  → max(lidar_perception: 50ms, camera_perception: 30ms)
  → fusion: 20ms + tracker (periodic, 10 Hz)
  → planning: 100ms
Critical path: 50 + 20 + 100 = 170ms (from pointcloud to trajectory)
```

No user-defined chains — the scope hierarchy IS the chain structure.

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

| Launch file                                     | Manifest file                                    |
|-------------------------------------------------|--------------------------------------------------|
| `camera_driver/camera_driver.launch.xml`        | `camera_driver/camera_driver.launch.yaml`        |
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
        ├── manifest_topics: { ... }       (new — expected topic graph)
        ├── manifest_services: { ... }     (new)
        ├── node_contracts: { ... }        (new — per-node io)
        └── topic_contracts: { ... }       (new — per-topic timing)

Executor
  ├── record.json → spawn nodes            (existing)
  └── manifest fields + GraphSnapshot → audit  (new, if fields present)
```

**New fields in record.json:**

| Field               | Type | Description                                                                 |
|---------------------|------|-----------------------------------------------------------------------------|
| `manifest_topics`   | map  | Resolved topic name → `{ type, qos, pub: [node_fqn], sub: [node_fqn] }`    |
| `manifest_services` | map  | Resolved service name → `{ type, server: [node_fqn], client: [node_fqn] }` |
| `node_contracts`    | map  | Node FQN → `{ trigger, correlation, latency_ms, ... }`                     |
| `topic_contracts`   | map  | Resolved topic name → `{ rate_hz, deadline_ms, max_drop_ratio, ... }`      |

These fields are populated by the parser when `--manifest-dir` is provided.

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
| I/O contract violated | runtime | `warn`  |

Output: terminal log, `GET /api/manifest/diff`, `play_log/<ts>/manifest_audit.json`.

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
