# record.json Format Specification

Complete specification of the record.json format used by dump_launch and consumed by play_launch runtime.

**Version:** Based on play_launch as of 2026-01-18

---

## 1. Overview

The `record.json` file is a **launch execution manifest** that describes:
- Regular ROS 2 nodes to execute
- Composable node containers
- Composable nodes to load into containers
- Lifecycle nodes
- Parameter file contents

**Purpose:** Enable deterministic replay of launch configurations without re-executing launch files.

---

## 2. Root Structure

```json
{
    "node": [/* NodeRecord[] */],
    "container": [/* ComposableNodeContainerRecord[] */],
    "load_node": [/* LoadNodeRecord[] */],
    "lifecycle_node": [/* string[] */],
    "file_data": {/* Map<path, content> */}
}
```

### Field Descriptions

| Field | Type | Description |
|-------|------|-------------|
| `node` | `NodeRecord[]` | Regular ROS 2 nodes and containers |
| `container` | `ComposableNodeContainerRecord[]` | Composable node container metadata |
| `load_node` | `LoadNodeRecord[]` | Composable nodes to load |
| `lifecycle_node` | `string[]` | Lifecycle node full names |
| `file_data` | `Map<string, string>` | Parameter file path → content mapping |

---

## 3. NodeRecord

Describes a regular ROS 2 node or container to execute.

### Schema

```typescript
interface NodeRecord {
    executable: string;                        // Executable name
    package: string | null;                    // ROS 2 package name
    name: string | null;                       // Node name (with namespace)
    namespace: string | null;                  // Node namespace
    exec_name: string | null;                  // Execution name (for tracking)
    params: Array<[string, string]>;           // Inline parameters
    params_files: string[];                    // Parameter file contents
    remaps: Array<[string, string]>;           // Topic/service remappings
    ros_args: string[] | null;                 // ROS-specific arguments
    args: string[] | null;                     // User-defined arguments
    cmd: string[];                             // Complete command line
    env: Array<[string, string]> | null;       // Environment variables
    respawn: boolean | null;                   // Auto-respawn on exit
    respawn_delay: number | null;              // Delay before respawn (seconds)
    global_params: Array<[string, string]> | null;  // Global parameters
}
```

### Field Details

#### `executable` (required)

Executable name or path.

**Examples:**
- `"talker"`
- `"component_container_mt"`
- `"/usr/bin/custom_node"`

#### `package` (nullable)

ROS 2 package name containing the executable.

**Examples:**
- `"demo_nodes_cpp"`
- `"rclcpp_components"`
- `null` (for system executables)

#### `name` (nullable)

Fully qualified node name including namespace.

**Format:** `/<namespace>/<node_name>` or `/<node_name>` if namespace is `/`

**Examples:**
- `"/talker"`
- `"/perception/lidar_detector"`
- `null` (uses executable name)

#### `namespace` (nullable)

Node namespace.

**Examples:**
- `"/"`
- `"/perception"`
- `null` (defaults to `/`)

#### `exec_name` (nullable)

Execution identifier for tracking (auto-generated with counter).

**Format:** `<executable>-<count>`

**Examples:**
- `"talker-1"`
- `"component_container-2"`
- `null`

#### `params` (array of tuples)

Inline parameters as `[name, value]` pairs.

**Format:** `[[name, value], ...]`

**Examples:**
```json
[
    ["use_sim_time", "true"],
    ["rate", "10.0"],
    ["topic_name", "chatter"]
]
```

**Notes:**
- Values are always strings
- Boolean parameters: `"true"` or `"false"` (lowercase)
- Numeric parameters: string representation (`"10"`, `"3.14"`)

#### `params_files` (array of strings)

Parameter file contents (YAML strings).

**Example:**
```json
[
    "/**:\n  ros__parameters:\n    param1: value1\n    param2: 42\n"
]
```

**Notes:**
- Content stored verbatim (not path)
- YAML format (ROS 2 parameter file syntax)
- Referenced files also stored in `file_data` map

#### `remaps` (array of tuples)

Topic/service/action remappings as `[from, to]` pairs.

**Format:** `[[from, to], ...]`

**Examples:**
```json
[
    ["chatter", "/chat"],
    ["__node", "my_custom_name"],
    ["__ns", "/custom_namespace"]
]
```

**Special Remappings:**
- `__node`: Node name remapping
- `__ns`: Namespace remapping

#### `ros_args` (nullable array)

ROS-specific command-line arguments.

**Examples:**
```json
[
    "--log-level", "debug",
    "--ros-args", "-r", "__node:=custom_name"
]
```

**Notes:**
- Typically `null` (handled via `cmd` field)
- Used for explicit ROS argument passing

#### `args` (nullable array)

User-defined command-line arguments.

**Examples:**
```json
["--config", "/path/to/config.yaml", "--verbose"]
```

#### `cmd` (required array)

Complete command line to execute.

**Format:** `[executable, arg1, arg2, ...]`

**Example:**
```json
[
    "/opt/ros/humble/lib/demo_nodes_cpp/talker",
    "--ros-args",
    "-r", "__node:=talker",
    "-r", "__ns:=/",
    "-p", "use_sim_time:=true"
]
```

**ROS 2 Argument Format:**
- `--ros-args`: ROS argument delimiter
- `-r <from>:=<to>`: Remapping
- `-p <name>:=<value>`: Parameter
- `--params-file <path>`: Parameter file
- `--log-level <level>`: Logging level

#### `env` (nullable array of tuples)

Environment variables as `[name, value]` pairs.

**Examples:**
```json
[
    ["RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"],
    ["RCUTILS_LOGGING_BUFFERED_STREAM", "1"]
]
```

#### `respawn` (nullable boolean)

Auto-respawn on process exit.

**Examples:**
- `true`: Respawn on exit
- `false`: Don't respawn
- `null`: Not specified (defaults to `false`)

#### `respawn_delay` (nullable number)

Delay in seconds before respawning.

**Examples:**
- `2.0`: Wait 2 seconds
- `0.0`: Respawn immediately
- `null`: Not specified

#### `global_params` (nullable array of tuples)

Global parameters set via `SetParameter` actions.

**Format:** `[[name, value], ...]`

**Examples:**
```json
[
    ["use_sim_time", "true"],
    ["publish_frequency", "50.0"]
]
```

---

## 4. ComposableNodeContainerRecord

Metadata about composable node containers.

### Schema

```typescript
interface ComposableNodeContainerRecord {
    name: string;       // Full container name (with namespace)
    namespace: string;  // Container namespace
}
```

### Field Details

#### `name` (required)

Full container name including namespace.

**Format:** `/<namespace>/<container_name>` or `/<container_name>` if namespace is `/`

**Examples:**
- `"/my_container"`
- `"/perception/lidar_container"`

#### `namespace` (required)

Container namespace.

**Examples:**
- `"/"`
- `"/perception"`

### Example

```json
{
    "container": [
        {
            "name": "/pointcloud_container",
            "namespace": "/"
        },
        {
            "name": "/planning/behavior_container",
            "namespace": "/planning"
        }
    ]
}
```

---

## 5. LoadNodeRecord

Describes a composable node to load into a container.

### Schema

```typescript
interface LoadNodeRecord {
    package: string;                          // Plugin package
    plugin: string;                           // Plugin class name
    target_container_name: string;            // Container full name
    node_name: string;                        // Node name (without namespace)
    namespace: string;                        // Node namespace
    log_level: string | null;                 // Log level (DEBUG/INFO/WARN/ERROR/FATAL)
    remaps: Array<[string, string]>;          // Remappings
    params: Array<[string, string]>;          // Parameters
    extra_args: Record<string, string>;       // Additional arguments
    env: Array<[string, string]> | null;      // Environment variables
}
```

### Field Details

#### `package` (required)

Package containing the composable node plugin.

**Examples:**
- `"composition"`
- `"autoware_glog_component"`

#### `plugin` (required)

Fully qualified plugin class name.

**Format:** `<namespace>::<ClassName>`

**Examples:**
- `"composition::Talker"`
- `"autoware::glog_component::GlogComponent"`

#### `target_container_name` (required)

Full name of the container to load into.

**Format:** Same as container `name` field

**Examples:**
- `"/my_container"`
- `"/perception/lidar_container"`

#### `node_name` (required)

Node name (without namespace prefix).

**Examples:**
- `"talker"`
- `"glog_component"`

#### `namespace` (required)

Node namespace.

**Examples:**
- `"/"`
- `"/perception"`

#### `log_level` (nullable)

ROS 2 logging level.

**Valid Values:**
- `"DEBUG"` (most verbose)
- `"INFO"`
- `"WARN"`
- `"ERROR"`
- `"FATAL"` (least verbose)
- `null` (use default)

#### `remaps` (array of tuples)

Topic/service remappings.

**Format:** Same as `NodeRecord.remaps`

#### `params` (array of tuples)

Node parameters.

**Format:** Same as `NodeRecord.params`

**Example:**
```json
[
    ["rate", "10.0"],
    ["topic", "chatter"]
]
```

#### `extra_args` (object)

Additional plugin-specific arguments.

**Examples:**
```json
{
    "use_intra_process_comms": "true",
    "start_parameter_services": "false"
}
```

#### `env` (nullable array of tuples)

Environment variables (rarely used for composable nodes).

**Format:** Same as `NodeRecord.env`

### Example

```json
{
    "load_node": [
        {
            "package": "composition",
            "plugin": "composition::Talker",
            "target_container_name": "/my_container",
            "node_name": "talker",
            "namespace": "/",
            "log_level": "INFO",
            "remaps": [["chatter", "/chat"]],
            "params": [["rate", "5.0"]],
            "extra_args": {},
            "env": null
        }
    ]
}
```

---

## 6. Lifecycle Nodes

Array of lifecycle node full names.

### Schema

```typescript
type LifecycleNodeList = string[];
```

### Format

Full node name including namespace.

### Examples

```json
{
    "lifecycle_node": [
        "/talker",
        "/perception/lidar_driver"
    ]
}
```

**Note:** Lifecycle nodes also appear in the `node` array. This field just tags them as lifecycle-aware.

---

## 7. File Data

Map of parameter file paths to their contents.

### Schema

```typescript
type FileData = Record<string, string>;
```

### Format

- **Key:** Absolute file path
- **Value:** File content (YAML string)

### Example

```json
{
    "file_data": {
        "/path/to/params.yaml": "/**:\n  ros__parameters:\n    param1: value1\n    nested:\n      param2: 42\n",
        "/path/to/other.yaml": "/**:\n  ros__parameters:\n    debug: true\n"
    }
}
```

### ROS 2 Parameter File Format

```yaml
/**:
  ros__parameters:
    param_name: value
    nested_namespace:
      nested_param: value
```

**Wildcards:**
- `/**`: Apply to all nodes
- `/namespace/**`: Apply to all nodes in namespace
- `/node_name`: Apply to specific node

---

## 8. Complete Example

```json
{
    "node": [
        {
            "executable": "component_container",
            "package": "rclcpp_components",
            "name": "/my_container",
            "namespace": "/",
            "exec_name": "component_container-1",
            "params": [],
            "params_files": [],
            "remaps": [],
            "ros_args": null,
            "args": null,
            "cmd": [
                "/opt/ros/humble/lib/rclcpp_components/component_container",
                "--ros-args",
                "-r", "__node:=my_container",
                "-r", "__ns:=/"
            ],
            "env": null,
            "respawn": false,
            "respawn_delay": null,
            "global_params": [["use_sim_time", "true"]]
        },
        {
            "executable": "talker",
            "package": "demo_nodes_cpp",
            "name": "/talker",
            "namespace": "/",
            "exec_name": "talker-1",
            "params": [["rate", "5.0"]],
            "params_files": ["/**:\n  ros__parameters:\n    use_sim_time: true\n"],
            "remaps": [["chatter", "/chat"]],
            "ros_args": null,
            "args": null,
            "cmd": [
                "/opt/ros/humble/lib/demo_nodes_cpp/talker",
                "--ros-args",
                "-r", "__node:=talker",
                "-r", "__ns:=/",
                "-r", "chatter:=/chat",
                "-p", "rate:=5.0",
                "--params-file", "/tmp/params_12345.yaml"
            ],
            "env": null,
            "respawn": true,
            "respawn_delay": 2.0,
            "global_params": [["use_sim_time", "true"]]
        }
    ],
    "container": [
        {
            "name": "/my_container",
            "namespace": "/"
        }
    ],
    "load_node": [
        {
            "package": "composition",
            "plugin": "composition::Listener",
            "target_container_name": "/my_container",
            "node_name": "listener",
            "namespace": "/",
            "log_level": "INFO",
            "remaps": [["chatter", "/chat"]],
            "params": [],
            "extra_args": {},
            "env": null
        }
    ],
    "lifecycle_node": [],
    "file_data": {
        "/tmp/params_12345.yaml": "/**:\n  ros__parameters:\n    use_sim_time: true\n"
    }
}
```

---

## 9. Validation Rules

### Required Fields

- `node[].executable`
- `node[].cmd`
- `container[].name`
- `container[].namespace`
- `load_node[].package`
- `load_node[].plugin`
- `load_node[].target_container_name`
- `load_node[].node_name`
- `load_node[].namespace`

### Type Constraints

- All `params` and `remaps` must be 2-tuples
- All `env` entries must be 2-tuples
- `respawn_delay` must be non-negative if present
- `cmd` must be non-empty array

### Semantic Constraints

- Containers in `container[]` should have corresponding entries in `node[]`
- `load_node[].target_container_name` should reference a container in `container[]`
- Node names should be unique (unless intentional duplicates)
- Lifecycle nodes should also be in `node[]`

---

## 10. Compatibility Notes

### play_launch Runtime Requirements

The record.json file is consumed by the play_launch Rust runtime, which expects:

1. **Exact field names** (case-sensitive)
2. **Correct types** (string vs. number vs. boolean vs. null)
3. **Array structures** for params/remaps/env (not objects)
4. **Full node names** with namespace prefix in `name` field
5. **Command arrays** (not space-separated strings)

### Version Compatibility

**Current Version:** Based on play_launch as of 2026-01-18

**Breaking Changes:** Any modification to field names or types may break compatibility with play_launch runtime.

**Extension:** New optional fields can be added without breaking backward compatibility.

---

## 11. Rust Parser Considerations

### Serialization

Use `serde_json` with appropriate structs:

```rust
#[derive(Serialize, Deserialize)]
struct RecordJson {
    node: Vec<NodeRecord>,
    container: Vec<ComposableNodeContainerRecord>,
    load_node: Vec<LoadNodeRecord>,
    lifecycle_node: Vec<String>,
    file_data: HashMap<String, String>,
}

#[derive(Serialize, Deserialize)]
struct NodeRecord {
    executable: String,
    package: Option<String>,
    name: Option<String>,
    namespace: Option<String>,
    exec_name: Option<String>,
    params: Vec<(String, String)>,
    params_files: Vec<String>,
    remaps: Vec<(String, String)>,
    ros_args: Option<Vec<String>>,
    args: Option<Vec<String>>,
    cmd: Vec<String>,
    env: Option<Vec<(String, String)>>,
    respawn: Option<bool>,
    respawn_delay: Option<f64>,
    global_params: Option<Vec<(String, String)>>,
}

// ... other record types
```

### Field Naming

Use `#[serde(rename_all = "snake_case")]` to match JSON field names.

### Null Handling

Use `Option<T>` for nullable fields.

---

## 12. Summary

The `record.json` format is a **comprehensive launch manifest** that:
- Describes all nodes, containers, and composable nodes
- Includes complete command lines and parameters
- Stores parameter file contents inline
- Enables deterministic replay without launch file re-execution

**Key Design Principles:**
- **Self-contained:** All necessary data included
- **Deterministic:** Same input produces same output
- **Flat structure:** Minimal nesting for easy parsing
- **Type-safe:** Explicit types and validation rules
