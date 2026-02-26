# play_launch CLI Interface Specification

## Overview

`play_launch` provides a unified command-line interface that mimics ROS 2 standard commands while enabling launch inspection, recording, and replay capabilities. The tool automatically invokes `dump_launch` to record launch executions and then replays them using the optimized Rust runtime.

## Command Syntax

### Launch Files

```bash
# Launch from a ROS package
play_launch launch <package_name> <launch_file> [key:=value...] [options...]

# Launch from a file path
play_launch launch <launch_file_path> [key:=value...] [options...]
```

**Examples:**
```bash
# Launch Autoware planning simulator
play_launch launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning

# Launch from absolute path
play_launch launch /path/to/my_launch.py use_sim_time:=true

# Launch with monitoring enabled
play_launch launch demo_nodes_cpp talker_listener.launch.py \
    --enable-monitoring --monitor-interval-ms 500
```

### Running Nodes

```bash
# Run a single node from a package
play_launch run <package_name> <executable> [args...] [options...]
```

**Examples:**
```bash
# Run a talker node
play_launch run demo_nodes_cpp talker

# Run with arguments
play_launch run demo_nodes_cpp talker --ros-args -p topic:=chatter
```

### Dump Only (No Replay)

```bash
# Dump launch execution without replaying
play_launch dump launch <package_name> <launch_file> [key:=value...] [dump_options...]
play_launch dump launch <launch_file_path> [key:=value...]

# Dump node execution
play_launch dump run <package_name> <executable> [args...]
```

**Dump-Specific Options:**
- `--output <file>` or `-o <file>`: Output file (default: `record.json`)
- `--debug`: Enable debug output during dump

**Examples:**
```bash
# Dump to custom file
play_launch dump launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning \
    --output autoware_dump.json

# Dump with debug output
play_launch dump launch demo_nodes_cpp talker_listener.launch.py --debug
```

### Replay Only

```bash
# Replay from existing record.json (current behavior)
play_launch replay [options...]

# Shorthand (backward compatible)
play_launch [options...]
```

**Examples:**
```bash
# Replay with monitoring
play_launch replay --enable-monitoring --config myconfig.yaml

# Replay with custom input
play_launch replay --input-file autoware_dump.json --log-dir logs/autoware_run1
```

## Common Options

These options apply to `launch`, `run`, and `replay` subcommands:

### Output Configuration
- `--log-dir <path>`: Log directory for execution outputs (default: `play_log`)
- `--input-file <path>`: Input record file for replay (default: `record.json`)

### Monitoring & Performance
- `--config <path>` or `-c <path>`: Runtime configuration YAML file
- `--enable-monitoring`: Enable resource monitoring for all nodes
- `--monitor-interval-ms <ms>`: Sampling interval in milliseconds

### Composable Node Loading
- `--delay-load-node-millis <ms>`: Delay before loading composable nodes (default: 2000)
- `--load-node-timeout-millis <ms>`: Timeout for loading each composable node (default: 30000)
- `--load-node-attempts <n>`: Max retry attempts for loading (default: 3)
- `--max-concurrent-load-node-spawn <n>`: Concurrent loading limit (default: 10)
- `--standalone-composable-nodes`: Run composable nodes in standalone mode
- `--load-orphan-composable-nodes`: Load composable nodes without matching containers

### Container Readiness
- `--wait-for-service-ready`: Wait for container services via ROS service discovery
- `--service-ready-timeout-secs <n>`: Max wait time for container services (default: 120, 0=unlimited)
- `--service-poll-interval-ms <ms>`: Polling interval for service discovery (default: 500)

### Other
- `--print-shell`: Generate shell script instead of executing

## Workflow Explanation

### Automatic Dump-and-Replay (`launch` and `run`)

When you use `play_launch launch` or `play_launch run`:

1. **Dump Phase**: `play_launch` invokes `dump_launch` (found via PATH)
   - Records launch execution to `record.json` (or custom output via `--output`)
   - Captures nodes, composable nodes, containers, parameters, and remappings

2. **Replay Phase**: `play_launch` executes the recorded launch
   - Spawns nodes and containers
   - Loads composable nodes using direct RCL service calls
   - Monitors resources if enabled
   - Saves all outputs to log directory

### Manual Workflow

For more control, you can separate the dump and replay steps:

```bash
# Step 1: Dump only
play_launch dump launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning \
    --output autoware_planning.json

# Step 2: Replay with custom options
play_launch replay \
    --input-file autoware_planning.json \
    --log-dir logs/run1 \
    --enable-monitoring \
    --wait-for-service-ready \
    --service-ready-timeout-secs 300
```

## Environment Requirements

Both `dump_launch` and `play_launch` must be available in PATH:

```bash
# Build the workspace
cd /path/to/play_launch
make build

# Source the workspace
source install/setup.bash

# Verify binaries are available
which dump_launch  # Should show path in install/
which play_launch  # Should show path in install/
```

## Exit Codes

- `0`: Success
- `1`: General error (dump failed, replay failed, etc.)
- `2`: Invalid arguments
- `130`: Interrupted by user (Ctrl-C)

## Comparison with ros2 Commands

| ros2 command | play_launch equivalent |
|--------------|------------------------|
| `ros2 launch pkg file.py` | `play_launch launch pkg file.py` |
| `ros2 launch file.py` | `play_launch launch file.py` |
| `ros2 run pkg exec` | `play_launch run pkg exec` |

**Key Differences:**
- `play_launch` records the launch execution before replaying
- Provides resource monitoring, process control, and detailed logging
- Uses optimized Rust runtime for replay
- Supports offline replay and analysis

## Advanced Usage

### Resource Monitoring with Process Control

```bash
# Create config.yaml
cat > config.yaml <<EOF
monitoring:
  enabled: true
  sample_interval_ms: 1000

processes:
  - node_pattern: "NODE 'rclcpp_components/component_container*"
    monitor: true
    cpu_affinity: [0, 1]
    nice: 5
EOF

# Launch with config
play_launch launch demo_nodes_cpp talker_listener.launch.py \
    --config config.yaml
```

### Container Service Readiness

For large launches like Autoware, ensure all container services are ready before loading composable nodes:

```bash
play_launch launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning \
    --wait-for-service-ready \
    --service-ready-timeout-secs 300 \
    --load-node-timeout-millis 60000
```

### Generating Shell Scripts

Export the launch as a shell script for debugging or manual execution:

```bash
play_launch launch demo_nodes_cpp talker_listener.launch.py \
    --print-shell > launch_script.sh

# Review and execute manually
bash launch_script.sh
```

## Troubleshooting

### dump_launch not found

Ensure the workspace is sourced:
```bash
source install/setup.bash
which dump_launch
```

### Composable nodes fail to load

1. Enable service readiness checking: `--wait-for-service-ready`
2. Increase timeout: `--load-node-timeout-millis 60000`
3. Check container logs in `play_log/node/` directories

### Orphan composable nodes warning

If you see warnings about orphan composable nodes:
- Use `--load-orphan-composable-nodes` to attempt loading them anyway
- Check that container names in launch files match actual container node names
- Review namespace resolution (relative vs absolute names)

