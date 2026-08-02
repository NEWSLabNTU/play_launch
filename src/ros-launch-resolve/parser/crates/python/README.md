# play-launch-parser

High-performance ROS 2 launch file parser, written in Rust with Python bindings.

Parses `.launch.xml`, `.launch.py`, and `.launch.yaml` files and returns a structured dict of all nodes, containers, composable nodes, and the launch include tree.

## Install

```bash
pip install play-launch-parser
```

## Python API

```python
from play_launch_parser import parse_file, parse_package

# Parse by file path
result = parse_file("/path/to/launch.xml")
result = parse_file("/path/to/launch.xml", args={"vehicle_model": "sample"})

# Parse by ROS package name (requires AMENT_PREFIX_PATH)
result = parse_package("autoware_launch", "planning_simulator.launch.xml")

# Result is a plain dict
for node in result["node"]:
    print(node["name"], node["package"])
```

## CLI

After installing, the `play-launch-parser` command is available:

```bash
# JSON to stdout (default)
play-launch-parser file /path/to/launch.xml
play-launch-parser launch autoware_launch planning_simulator.launch.xml

# Pass launch arguments
play-launch-parser file /path/to/launch.xml vehicle_model:=sample

# Human-readable summary
play-launch-parser file /path/to/launch.xml --format summary

# One node name per line (for piping to grep, wc, etc.)
play-launch-parser file /path/to/launch.xml --format names

# Write to file
play-launch-parser file /path/to/launch.xml -o record.json
```

### Options

| Flag | Description |
|------|-------------|
| `-f`, `--format` | `json` (default), `summary`, or `names` |
| `-o`, `--output` | Write to file instead of stdout |
| `-v`, `--verbose` | Debug logging to stderr |
| `-q`, `--quiet` | Suppress all log output |

### Exit codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Parse error |
| 2 | File or package not found |

## Output schema

`parse_file()` and `parse_package()` return a dict with these keys:

### `node` — standalone nodes

```python
{
    "name": "my_node",             # Node name (may be None)
    "namespace": "/sensing",       # ROS namespace
    "package": "my_package",       # ROS package name (None for raw executables)
    "executable": "my_exec",       # Executable name
    "exec_name": "my_exec-1",     # Unique exec name
    "cmd": ["/path/to/exec", "--ros-args", ...],  # Full command line
    "params": [["key", "value"]],  # Node parameters
    "params_files": ["/path/to/params.yaml"],
    "remaps": [["from", "to"]],    # Topic remappings
    "env": [["KEY", "VALUE"]],     # Environment variables (may be None)
    "args": ["--flag"],            # Extra arguments (may be None)
    "ros_args": ["--log-level", "debug"],  # ROS arguments (may be None)
    "global_params": [["key", "value"]],   # Global parameters (may be None)
    "respawn": true,               # Respawn on crash (may be None)
    "respawn_delay": 2.0,          # Respawn delay in seconds (may be None)
    "scope": 3                     # Index into scopes list (may be None)
}
```

### `container` — composable node containers

Same fields as `node`, plus `name` and `namespace` are always present (not optional).

### `load_node` — composable nodes loaded into containers

```python
{
    "node_name": "my_component",
    "namespace": "/sensing",
    "package": "my_package",
    "plugin": "my_package::MyComponent",
    "target_container_name": "/my_container",
    "params": [["key", "value"]],
    "remaps": [["from", "to"]],
    "extra_args": {"use_intra_process_comms": "true"},
    "env": [["KEY", "VALUE"]],
    "log_level": "info",
    "scope": 5
}
```

### `scopes` — launch include tree

Each entry represents one launch file invocation or group scope:

```python
{
    "id": 0,
    "origin": {"pkg": "autoware_launch", "file": "planning_simulator.launch.xml"},
    "ns": "/",
    "args": {"vehicle_model": "sample"},
    "parent": null    # None for root, index for children
}
```

Use `scope` fields on nodes to trace which launch file defined each node.

### `variables` — resolved launch arguments

Dict of all `DeclareLaunchArgument` and `<let>` values after resolution.

### `file_data`, `lifecycle_node`

Metadata and lifecycle node names (usually empty).

## Requirements

- Python >= 3.10
- No ROS runtime required to parse XML/YAML launch files
- `AMENT_PREFIX_PATH` must be set for `parse_package()` and `$(find-pkg-share ...)` substitutions
