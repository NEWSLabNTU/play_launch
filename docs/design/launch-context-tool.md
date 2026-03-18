# Launch Context Tool Design

## Motivation

Autoware launches ~110 nodes. Debugging one node means launching the entire
system. This wastes time and hardware — you need a GPU cluster just to test
if a perception node handles a parameter change correctly.

The idea: **extract a single node's (or sub-launch-file's) full execution
context**, then run it in isolation with identical behavior to the full
system.

## Per-Node Execution Context

The context has two parts:

### Part 1: Launch context (from parser)

What the launch file provides to the node. The parser already captures this
in `record.json`.

```yaml
# Extracted from the full launch tree for /perception/lidar/centerpoint
launch:
  name: centerpoint
  namespace: /perception/lidar
  package: autoware_lidar_centerpoint
  executable: lidar_centerpoint_node
  params:
    score_threshold: "0.35"
    densification_num_past_frames: "1"
    class_names: "[CAR, TRUCK, BUS, BICYCLE, PEDESTRIAN]"
  params_files:
    - $(find-pkg-share autoware_lidar_centerpoint)/config/centerpoint.param.yaml
  remaps:
    - [input/pointcloud, /perception/lidar/no_ground]
    - [~/output/objects, /perception/lidar/objects]
  env:
    CUDA_VISIBLE_DEVICES: "0"
  cmd:
    - /opt/autoware/lib/autoware_lidar_centerpoint/lidar_centerpoint_node
    - --ros-args -r __node:=centerpoint -r __ns:=/perception/lidar
    - -p score_threshold:=0.35 ...
```

### Part 2: Manifest context (from launch manifest)

What the manifest adds — the communication contract that the launch file
doesn't capture.

```yaml
manifest:
  source: tier4_perception_launch/lidar_perception.launch.yaml
  publishes:
    - topic: objects
      resolved: /perception/lidar/objects
      type: autoware_perception_msgs/msg/DetectedObjects
      qos: { reliability: best_effort, depth: 1 }
  subscribes:
    - topic: /perception/lidar/no_ground       # via remap from input/pointcloud
      type: sensor_msgs/msg/PointCloud2
      qos: { reliability: best_effort }
  requirements:
    node_latency_ms: 30
    max_deadline_misses: 3
  interface_role: internal          # topics are internal to perception component
  component: perception/lidar       # which component this node belongs to
```

### Combined context

The two parts together give everything needed to:
- **Run the node** with correct params, remaps, namespace, env
- **Validate the node** against its communication contract
- **Stub inputs** with the correct topic names, types, and QoS
- **Check outputs** against expected topics and timing

## Use Cases

### 1. Inspect a node's context

See what a node receives from the launch tree + manifest, without running
anything.

```bash
play_launch context autoware_launch planning_simulator.launch.xml \
    --node /perception/lidar/centerpoint \
    --manifest-dir manifests/
```

Output: YAML with both parts of the context.

### 2. Run a single node in isolation

Extract context, generate a standalone launch, run with recorded input.

```bash
# Extract and run
play_launch run-isolated autoware_launch planning_simulator.launch.xml \
    --node /perception/lidar/centerpoint \
    --manifest-dir manifests/ \
    --input-bag recorded_data.db3
```

The tool:
1. Extracts the node's full context
2. Generates a minimal launch file with all params/remaps/namespace
3. Generates a rosbag play command for input topics (from manifest's `subscribes`)
4. Optionally generates a subscriber that validates output topics

### 3. Run an inner launch file

Run a sub-launch-file with the context its parent would provide.

```bash
# Run just the perception sub-launch
play_launch run-isolated autoware_launch planning_simulator.launch.xml \
    --launch tier4_perception_launch perception.launch.xml \
    --manifest-dir manifests/ \
    --input-bag recorded_data.db3
```

The tool:
1. Traces the include tree to find where `perception.launch.xml` is included
2. Extracts the parent-provided context: namespace, args, global params
3. Generates: `ros2 launch tier4_perception_launch perception.launch.xml`
   with all args from the parent context
4. Stubs input topics from the manifest's interface `subscribes`

### 4. Component-level testing

Run all nodes in a manifest component with their internal wiring intact,
only stubbing the component interface.

```bash
# Run the lidar perception component (cropbox + ground_filter + centerpoint)
play_launch run-isolated autoware_launch planning_simulator.launch.xml \
    --component perception/lidar \
    --manifest-dir manifests/ \
    --input-bag recorded_data.db3
```

The tool stubs only the interface topics (`subscribes: [/sensing/pointcloud]`),
lets internal topics (`cropped`, `no_ground`) flow naturally between nodes.

## Architecture

```
                                      ┌──────────────────────┐
  launch file ──→ Parser ─────────────┤ record.json          │
                    │                 │  (full node records)  │
                    │ --manifest-dir  │                       │
  manifest files ──→ ManifestLoader ──┤ expected_graph        │
                                      │  (topics, QoS, etc.) │
                                      └──────────┬───────────┘
                                                 │
                                                 v
                                      ┌──────────────────────┐
                                      │ ContextExtractor      │
                                      │                       │
                                      │  --node FQN           │
                                      │  --launch pkg/file    │
                                      │  --component name     │
                                      │                       │
                                      │  Produces:            │
                                      │  - NodeContext (yaml)  │
                                      │  - StubLaunch (xml)    │
                                      │  - RunScript (bash)    │
                                      └───────────────────────┘
```

The tool is a **new subcommand** on `play_launch` (not a separate binary),
since it reuses the parser and manifest loader directly.

## Launch Tree Scoping

### The problem

The current `record.json` stores flat lists of nodes, containers, and
composable nodes. No tree structure — you can't tell which launch file
a node came from, or which invocation of a multiply-included launch file
produced it.

Example: `camera_driver/camera.launch.xml` is included twice:

```
autoware_launch/planning_simulator.launch.xml
├── tier4_sensing_launch/sensing.launch.xml  (ns: /sensing)
│   ├── camera_driver/camera.launch.xml      (ns: /sensing/camera/front)
│   │   └── node: driver                     → /sensing/camera/front/driver
│   └── camera_driver/camera.launch.xml      (ns: /sensing/camera/rear)
│       └── node: driver                     → /sensing/camera/rear/driver
└── tier4_perception_launch/perception.launch.xml  (ns: /perception)
    └── ...
```

Both `driver` nodes come from the same launch file but are different
invocations at different positions in the tree. The node FQNs differ
(`/sensing/camera/front/driver` vs `/sensing/camera/rear/driver`), but
`record.json` has no way to trace them back to their tree position.

### Scope path

Each node record gains a **scope path** — the chain of includes from root
to the node's launch file. Each scope entry records the package, launch
file, namespace, and arguments at that level.

```json
{
  "name": "/sensing/camera/front/driver",
  "scope": [
    {
      "pkg": "autoware_launch",
      "file": "planning_simulator.launch.xml",
      "ns": ""
    },
    {
      "pkg": "tier4_sensing_launch",
      "file": "sensing.launch.xml",
      "ns": "/sensing"
    },
    {
      "pkg": "camera_driver",
      "file": "camera.launch.xml",
      "ns": "/sensing/camera/front",
      "args": { "camera_id": "0" }
    }
  ],
  "executable": "camera_driver_node",
  "package": "camera_driver",
  ...
}
```

The scope path uniquely identifies each invocation:
- Same launch file, different namespace → different scope paths
- Same launch file, different args → different scope paths

**Duplicate invocation detection**: two includes with identical
`(pkg, file, namespace, resolved_args)` produce duplicate node FQNs and
duplicate topics — this is always a bug. The parser flags it as a
parse-time error. Since valid launch trees always differ in namespace or
args, the tuple `(pkg, file, namespace, args)` is a unique key.

### Launch tree in record.json

In addition to per-node scope paths, `record.json` gains an optional
`launch_tree` field that captures the full include hierarchy:

```json
{
  "launch_tree": {
    "pkg": "autoware_launch",
    "file": "planning_simulator.launch.xml",
    "ns": "",
    "nodes": [],
    "includes": [
      {
        "pkg": "tier4_sensing_launch",
        "file": "sensing.launch.xml",
        "ns": "/sensing",
        "args": { "vehicle_model": "sample_vehicle" },
        "nodes": [],
        "includes": [
          {
            "pkg": "camera_driver",
            "file": "camera.launch.xml",
            "ns": "/sensing/camera/front",
            "args": { "camera_id": "0" },
            "nodes": ["/sensing/camera/front/driver"],
            "includes": []
          },
          {
            "pkg": "camera_driver",
            "file": "camera.launch.xml",
            "ns": "/sensing/camera/rear",
            "args": { "camera_id": "1" },
            "nodes": ["/sensing/camera/rear/driver"],
            "includes": []
          }
        ]
      }
    ]
  }
}
```

This is optional (`#[serde(default, skip_serializing_if = "Option::is_none")]`)
for backward compatibility.

### Parser changes

The parser's traverser already enters/exits scopes via `save_scope()` /
`restore_scope()`. It processes includes in `process_include()`. The
changes:

1. **Track scope stack**: maintain a `Vec<ScopeEntry>` alongside the
   existing `LaunchContext`. Push on `process_include()`, pop on return.

2. **Stamp each capture**: when a `NodeCapture`, `ContainerCapture`, or
   `LoadNodeCapture` is created, copy the current scope stack into a
   `scope` field on the capture.

3. **Build launch tree**: during traversal, build a tree of
   `LaunchTreeNode` mirroring the include structure. Each tree node
   records its package, file, namespace, args, and the FQNs of nodes
   declared directly in that launch file.

4. **Serialize**: add `scope` to `NodeRecord` and `launch_tree` to
   `RecordJson`.

### Addressing the user

The user selects a scope (a position in the launch tree) rather than
just a node FQN:

```bash
# Select by node FQN (always unique)
play_launch context ... --node /sensing/camera/front/driver

# Select by launch file (unique if included once)
play_launch context ... --launch camera_driver camera.launch.xml

# Disambiguate multiple invocations with namespace
play_launch context ... \
    --launch camera_driver camera.launch.xml \
    --namespace /sensing/camera/front

# Select all invocations of a launch file
play_launch context ... --launch camera_driver camera.launch.xml --all
```

Since `(pkg, file, namespace, args)` is unique in valid launch trees,
`--launch pkg file --namespace ns` always resolves to exactly one
invocation.

## Context Extraction Algorithm

### For a single node (`--node /perception/lidar/centerpoint`):

1. Parse the full launch tree → `record.json` with scope paths
2. Load manifests → expected graph
3. Find the node record matching the FQN
4. Read its scope path → determine originating launch file
5. Find the manifest entry for this node
6. Merge launch context + manifest context → `NodeContext`

### For an inner launch file (`--launch pkg file --namespace ns`):

1. Parse the full launch tree with scope tracking
2. Search `launch_tree` for the entry matching (pkg, file, ns)
3. Extract the parent-provided context from the scope entry:
   - Namespace
   - Arguments passed at the include
   - Global params and env vars in scope at that point
4. Collect all node FQNs under that subtree
5. Load the manifest for the launch file
6. Produce: launch command + parent context + manifest interface stubs

### For a component (`--component perception/lidar`):

1. Parse + load manifests
2. Find all nodes belonging to the component (from manifest)
3. Match to scope entries in the launch tree
4. Extract each node's context
5. The component's interface `subscribes` become input stubs
6. The component's interface `publishes` become output validators
7. Internal topics are left unwired (nodes connect naturally)

## Output Formats

### `context` subcommand — YAML inspection

```yaml
# play_launch context ... --node /perception/lidar/centerpoint
node: /perception/lidar/centerpoint
launch:
  package: autoware_lidar_centerpoint
  executable: lidar_centerpoint_node
  namespace: /perception/lidar
  params:
    score_threshold: "0.35"
  params_files:
    - /opt/autoware/share/.../centerpoint.param.yaml
  remaps:
    - [input/pointcloud, /perception/lidar/no_ground]
  env: {}
manifest:
  source: tier4_perception_launch/lidar_perception.launch.yaml
  publishes:
    - { topic: /perception/lidar/objects, type: "autoware_perception_msgs/msg/DetectedObjects" }
  subscribes:
    - { topic: /perception/lidar/no_ground, type: "sensor_msgs/msg/PointCloud2" }
  requirements:
    node_latency_ms: 30
```

### `run-isolated` subcommand — generated launch file

```xml
<!-- Generated by play_launch run-isolated -->
<launch>
  <!-- Input stubs: replay these topics from bag -->
  <!-- /perception/lidar/no_ground (sensor_msgs/msg/PointCloud2) -->

  <node pkg="autoware_lidar_centerpoint" exec="lidar_centerpoint_node"
        name="centerpoint" namespace="/perception/lidar">
    <param from="$(find-pkg-share autoware_lidar_centerpoint)/config/centerpoint.param.yaml"/>
    <param name="score_threshold" value="0.35"/>
    <remap from="input/pointcloud" to="/perception/lidar/no_ground"/>
    <remap from="~/output/objects" to="/perception/lidar/objects"/>
  </node>
</launch>
```

Plus a run script:

```bash
#!/bin/bash
# Generated by play_launch run-isolated
# Input topics needed: /perception/lidar/no_ground
# Expected output: /perception/lidar/objects

# Replay input data
ros2 bag play recorded_data.db3 \
    --topics /perception/lidar/no_ground &

# Launch the isolated node
ros2 launch /tmp/play_launch_isolated/centerpoint.launch.xml
```

## Parser Patches Needed

The current parser has flat capture lists (`Vec<NodeCapture>`, etc.) with
no tree structure. `IncludeCapture` exists but only records `file_path`,
`args`, `ros_namespace` — no parent-child relationship.

1. **Scope stack**: add `Vec<ScopeEntry>` to the traverser context. Push
   on `process_include()` (entity.rs:91, yaml.rs:249, python_exec.rs:151),
   pop when `restore_scope()` is called.

2. **Stamp captures**: when `NodeCapture`, `ContainerCapture`, or
   `LoadNodeCapture` is created, copy the current scope stack. Requires
   adding a `scope: Vec<ScopeEntry>` field to each capture type.

3. **Build launch tree**: during traversal, maintain a tree structure
   mirroring the include hierarchy. Each node records pkg, file, ns, args,
   and the FQNs of entities declared at that level.

4. **Serialize**: add `scope` to `NodeRecord`/`ComposableNodeContainerRecord`/
   `LoadNodeRecord` and `launch_tree` to `RecordJson`. All optional with
   `skip_serializing_if` for backward compat.

5. **Manifest integration**: Phase 30 manifest loading (already planned).

The scope stack and launch tree are lightweight additions — the traverser
already enters/exits scopes. The main change is recording what's already
known rather than discarding it.

## Relationship to Existing Commands

| Command | What it does | How this differs |
|---------|-------------|-----------------|
| `play_launch dump` | Parse → record.json (all nodes) | `context` extracts ONE node's full context |
| `play_launch launch` | Parse → record.json → replay ALL nodes | `run-isolated` replays a SUBSET |
| `play_launch replay` | Replay from existing record.json | `run-isolated` generates a standalone launch, no play_launch needed |
| `ros2 launch` | Run a launch file | `run-isolated` provides the parent context that `ros2 launch` can't |

The key difference: `run-isolated` produces a **self-contained** launch
that can be run with plain `ros2 launch` — no play_launch needed at
runtime. play_launch is only used for context extraction.

## Future Extensions

- **Input recording**: `play_launch record-context` — run the full system,
  record each node's input topics as rosbag. Produces per-node bags for
  later `run-isolated` use.

- **Output validation**: `run-isolated --validate` — subscribe to output
  topics and check against manifest expectations (correct type, QoS, rate,
  latency within budget).

- **Diff mode**: `play_launch context-diff` — compare two contexts (e.g.,
  before and after a parameter change, or between two launch configurations).

- **IDE integration**: export context as a VS Code launch.json or CLion
  run configuration for debugging with breakpoints.
