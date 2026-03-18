# Record Format (record.json)

## Overview

`record.json` is the primary artifact produced by the parser and consumed
by the executor. It captures everything needed to replay a ROS 2 launch
without re-parsing: node processes, container processes, composable node
loads, parameter files, and resolved variables.

Source: `src/play_launch_parser/.../record/types.rs` (parser)
Consumer: `src/play_launch/src/ros/launch_dump.rs` (executor)

## Current Format

```json
{
  "container": [ ... ],
  "file_data": { ... },
  "lifecycle_node": [ ... ],
  "load_node": [ ... ],
  "node": [ ... ],
  "variables": { ... }
}
```

### Top-level fields

| Field | Type | Description |
|-------|------|-------------|
| `node` | `Vec<NodeRecord>` | Regular ROS nodes and executables |
| `container` | `Vec<ComposableNodeContainerRecord>` | Component containers |
| `load_node` | `Vec<LoadNodeRecord>` | Composable nodes loaded into containers |
| `lifecycle_node` | `Vec<String>` | Lifecycle node names (tracked, not auto-managed) |
| `file_data` | `Map<String, String>` | Parameter file contents (path → YAML text) |
| `variables` | `Map<String, String>` | Resolved launch arguments (optional, skipped if empty) |

### NodeRecord

A regular node or raw executable.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `executable` | string | yes | Executable name |
| `package` | string? | no | ROS package (None for raw executables) |
| `name` | string? | no | Node name (may be None; exec_name used as fallback) |
| `namespace` | string? | no | Node namespace |
| `exec_name` | string? | no | Launch-level exec name (for FQN mapping) |
| `params` | `[(key, value)]` | yes | Parameter key-value pairs |
| `params_files` | `[string]` | yes | Paths to parameter YAML files |
| `remaps` | `[(from, to)]` | yes | Topic remappings |
| `args` | `[string]`? | no | Additional arguments |
| `cmd` | `[string]` | yes | Full command line |
| `env` | `[(key, value)]`? | no | Environment variable overrides |
| `ros_args` | `[string]`? | no | ROS-specific arguments |
| `global_params` | `[(key, value)]`? | no | Global parameters (from SetParameter) |
| `respawn` | bool? | no | Whether to respawn on exit |
| `respawn_delay` | float? | no | Delay before respawn (seconds) |

### ComposableNodeContainerRecord

Same fields as `NodeRecord` but `name`, `namespace`, and `package` are
required (not optional) since containers always have these.

### LoadNodeRecord

A composable node loaded into a container.

| Field | Type | Description |
|-------|------|-------------|
| `package` | string | Plugin package |
| `plugin` | string | Plugin class name |
| `target_container_name` | string | Target container FQN |
| `node_name` | string | Composable node name |
| `namespace` | string | Node namespace |
| `params` | `[(key, value)]` | Parameters |
| `remaps` | `[(from, to)]` | Remappings |
| `extra_args` | `Map<String, String>` | Extra arguments (e.g., `use_intra_process_comms`) |
| `env` | `[(key, value)]`? | Environment overrides |
| `log_level` | string? | Log level override |

## Planned Extensions (Phase 30)

New optional fields for launch manifest support. Old record.json files
without these fields continue to work — the executor checks presence
before enabling auditing.

| Field | Type | Description |
|-------|------|-------------|
| `expected_graph` | `ExpectedGraph?` | Expected communication graph from manifests |
| `launch_tree` | `LaunchTreeNode?` | Include hierarchy with scope information |

Per-node scope annotation (on `NodeRecord`, `ComposableNodeContainerRecord`,
`LoadNodeRecord`):

| Field | Type | Description |
|-------|------|-------------|
| `scope` | `[ScopeEntry]?` | Chain of includes from root to this node's launch file |

### ExpectedGraph

Built by the parser from manifest files when `--manifest-dir` is provided.

```json
{
  "expected_graph": {
    "topics": {
      "/sensing/pointcloud": {
        "type": "sensor_msgs/msg/PointCloud2",
        "qos": { "reliability": "best_effort", "depth": 1 },
        "publishers": ["/sensing/lidar/driver"],
        "subscribers": ["/perception/lidar/cropbox_filter"]
      }
    },
    "services": { ... },
    "actions": { ... },
    "sync": { ... },
    "requirements": { ... }
  }
}
```

### LaunchTreeNode

```json
{
  "launch_tree": {
    "pkg": "autoware_launch",
    "file": "planning_simulator.launch.xml",
    "ns": "",
    "args": {},
    "nodes": [],
    "includes": [
      {
        "pkg": "tier4_sensing_launch",
        "file": "sensing.launch.xml",
        "ns": "/sensing",
        "args": { "vehicle_model": "sample_vehicle" },
        "nodes": ["/sensing/lidar/driver", "/sensing/camera/driver"],
        "includes": [ ... ]
      }
    ]
  }
}
```

### ScopeEntry

```json
{
  "scope": [
    { "pkg": "autoware_launch", "file": "planning_simulator.launch.xml", "ns": "" },
    { "pkg": "tier4_sensing_launch", "file": "sensing.launch.xml", "ns": "/sensing" }
  ]
}
```

All new fields use `#[serde(default, skip_serializing_if = "Option::is_none")]`
for backward compatibility.

## Serialization Notes

- Fields are ordered alphabetically to match the Python parser's output
  (for diff-based validation during parser migration)
- Tuples `(key, value)` serialize as JSON arrays: `["key", "value"]`
- `variables` is skipped when empty (matches Python behavior)
- `file_data` keys are file paths; values are raw YAML content
