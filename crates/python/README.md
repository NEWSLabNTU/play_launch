# play-launch-parser

High-performance ROS 2 launch file parser, written in Rust with Python bindings.

## Install

```bash
pip install play-launch-parser
```

## Usage

```python
from play_launch_parser import parse_file, parse_package

# Parse by file path
result = parse_file("/path/to/launch.xml")
result = parse_file("/path/to/launch.xml", args={"vehicle_model": "sample"})

# Parse by ROS package name (requires AMENT_PREFIX_PATH)
result = parse_package("demo_nodes_cpp", "talker_listener.launch.xml")

# Result is a plain dict
for node in result["node"]:
    print(node["name"], node["package"])
```

## Output

Returns a dict with keys:

- `node` — list of standalone node records
- `container` — list of composable node container records
- `load_node` — list of composable node (LoadNode) records
- `scopes` — launch tree (which file each node comes from)
- `variables` — resolved launch arguments
- `file_data` — file metadata
- `lifecycle_node` — lifecycle node names
