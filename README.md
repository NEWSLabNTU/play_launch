# play_launch

Record, replay, and analyze ROS 2 launch executions with resource monitoring and interactive management.

[![Watch the demo](assets/demo.png)](assets/demo.mp4)

## Installation

Install from PyPI:

```bash
pip install play_launch
```

Optional: Enable I/O monitoring (requires sudo):

```bash
play_launch setcap-io-helper
```

## Quick Start

Launch any ROS 2 package with monitoring and Web UI enabled by default:

```bash
play_launch launch demo_nodes_cpp talker_listener.launch.py
```

Access Web UI at `http://127.0.0.1:8080` for real-time node management and log streaming.

## Usage

### Launch Files

Replace `ros2 launch` with `play_launch launch`:

```bash
play_launch launch <package> <launch_file> [arguments...]
```

### Single Nodes

Replace `ros2 run` with `play_launch run`:

```bash
play_launch run <package> <executable> [arguments...]
```

### Two-Step Workflow

Record first, replay multiple times:

```bash
# Record
play_launch dump launch <package> <launch_file> [arguments...]

# Replay
play_launch replay
```

## Features

All features enabled by default:
- **Resource monitoring**: CPU, memory, I/O, GPU (2s interval)
- **Diagnostic monitoring**: `/diagnostics` and `/diagnostics_agg` topics
- **Web UI**: Interactive management at `http://127.0.0.1:8080`

### Disable Features

Disable specific features:

```bash
play_launch launch <package> <launch_file> --disable-monitoring
play_launch launch <package> <launch_file> --disable-diagnostics
play_launch launch <package> <launch_file> --disable-web-ui
play_launch launch <package> <launch_file> --disable-all
```

### Adjust Monitoring

Change sampling interval (default: 2000ms):

```bash
play_launch launch <package> <launch_file> --monitor-interval-ms 500
```

### Configure Web UI

Change address or port (default: `127.0.0.1:8080`):

```bash
play_launch launch <package> <launch_file> --web-addr 0.0.0.0:8080
```

### Configuration File

Use YAML for advanced control:

```yaml
# config.yaml
monitoring:
  enabled: true
  sample_interval_ms: 2000

processes:
  - node_pattern: "NODE 'rclcpp_components/component_container*"
    cpu_affinity: [0, 1]
    nice: 5
```

Apply configuration:

```bash
play_launch replay --config config.yaml
```

## Visualization

Generate interactive plots from monitoring data:

```bash
# Plot latest execution
play_launch plot

# Plot specific log directory
play_launch plot --log-dir play_log/2025-10-28_16-17-56

# Plot specific metrics
play_launch plot --metrics cpu memory

# List available metrics
play_launch plot --list-metrics
```

Output saved to `play_log/<timestamp>/plot/`:
- `cpu_timeline.html` - CPU usage over time
- `memory_timeline.html` - Memory usage over time
- `io_timeline.html` - I/O read/write rates
- `cpu_distribution.html` - CPU distribution box plot
- `memory_distribution.html` - Memory distribution box plot
- `statistics.txt` - Top 10 rankings for all metrics

## Web UI Features

- **Node management**: Start/Stop/Restart individual or all nodes
- **Container controls**: Load/Unload composable nodes
- **Real-time logs**: Stream stdout/stderr with auto-reconnect
- **Status monitoring**: Color-coded node states
- **Auto-restart**: Per-node automatic restart configuration
- **Search & filter**: Find nodes in large deployments

## Output Structure

```
play_log/<timestamp>/
├── node/<node_name>/
│   ├── metadata.json
│   ├── metrics.csv       # Resource metrics (when enabled)
│   ├── stdout/stderr     # Process logs
│   └── pid/status/cmdline
├── system_stats.csv      # System-wide metrics
└── plot/                 # Generated visualizations
```

## Command Reference

```bash
# Launch (all features enabled by default)
play_launch launch <package> <launch_file> [args...]
play_launch run <package> <executable> [args...]

# Dump and replay
play_launch dump launch <package> <launch_file> [args...]
play_launch replay [--input-file record.json]

# Disable features
play_launch launch <pkg> <file> --disable-monitoring
play_launch launch <pkg> <file> --disable-diagnostics
play_launch launch <pkg> <file> --disable-web-ui
play_launch launch <pkg> <file> --disable-all

# Enable only specific features
play_launch launch <pkg> <file> --enable monitoring
play_launch launch <pkg> <file> --enable web-ui --enable diagnostics

# Adjust settings
play_launch launch <pkg> <file> --monitor-interval-ms 500
play_launch launch <pkg> <file> --web-addr 0.0.0.0:8080
play_launch launch <pkg> <file> --config config.yaml

# Logging
play_launch launch <pkg> <file> --verbose              # INFO level
RUST_LOG=play_launch=debug play_launch launch <pkg> <file>  # DEBUG level

# Visualization
play_launch plot
play_launch plot --log-dir <dir>
play_launch plot --metrics cpu memory io gpu
play_launch plot --list-metrics
```

## Development

See [CLAUDE.md](CLAUDE.md) for development guidelines and architecture details.

```bash
# Lint code
just lint

# Format code
just format

# Run tests
just test
```

## License

MIT License. See [LICENSE.txt](LICENSE.txt).
