# CLAUDE.md

Guide for Claude Code when working with this repository.

## Project Overview

ROS2 Launch Inspection Tool - Records and replays ROS 2 launch executions for performance analysis:
- **dump_launch** (Python): Records launch execution to `record.json`
- **play_launch** (Rust): Replays with resource monitoring
- **play_launch_analyzer** (Python): Analyzes and visualizes logs

## Installation & Usage

```sh
# Install from PyPI (requires ROS2 Humble)
pip install play_launch

# Or build from source
just build                          # Build wheel
just run launch <pkg> <launch_file> # Run with colcon build
play_launch launch <pkg> <launch>   # Run if installed via pip

# Optional: I/O monitoring (requires sudo)
just setcap-io-helper

# Analysis
play_launch plot
```

## Architecture

**Execution Flow:**
1. Load `record.json`, copy parameter files
2. Classify nodes (containers vs regular)
3. Spawn async background tasks (monitoring, service discovery, web UI)
4. Actor-based lifecycle management:
   - Regular nodes and containers: separate actors
   - Composable nodes: virtual members managed by parent containers
5. Logs saved to `play_log/<timestamp>/`

**Key Concepts:**
- **Async/Tokio**: All background services run as async tasks
- **Actor Pattern**: Self-contained lifecycle management with respawn support
- **Virtual Members**: Composable nodes managed as internal state by container actors (LoadNode/Unload operations)
- **ListNodes Verification**: Automatic verification if LoadNode calls timeout (>30s)

## Performance Characteristics

play_launch is designed to be lightweight, with Phase 1 optimizations (2026-01-18) achieving:
- **CPU**: ~19% on large deployments (115 nodes) = 0.17% per managed node
- **Memory**: ~116 MB RSS
- **Threads**: ~60 total
  - 8-19 async worker threads (adaptive tokio pool)
  - ~40 ROS/DDS threads
  - ~2 other threads

**Tuning Options:**
- `--monitor-interval-ms <MS>`: Adjust sampling frequency (default: 2000ms)
- `--disable-monitoring`: Disable resource monitoring entirely (~5% CPU savings)
- `--disable-diagnostics`: Disable diagnostic subscriptions (~2% CPU savings)
- `--disable-all`: Disable all features for minimal overhead

**Technical Details:**
- Tokio runtime: 8 worker threads, 16 max blocking threads
- ROS executor: 50ms spin interval (acceptable latency for management operations)
- Monitoring interval: 2000ms default (configurable via CLI or config file)

## Configuration

**Key CLI Flags:**
- `--config <PATH>`: Runtime config YAML
- `--enable <FEATURE>`: Enable only specific features (can be repeated)
  - Values: `monitoring`, `diagnostics`, `web-ui`
  - Conflicts with `--disable-*` flags
- `--disable-monitoring`: Disable resource monitoring (enabled by default)
- `--disable-diagnostics`: Disable diagnostic monitoring (enabled by default)
- `--disable-web-ui`: Disable web UI (enabled by default)
- `--disable-all`: Disable all features at once
- `--web-addr <IP:PORT>`: Web UI address (default: 127.0.0.1:8080)
- `--disable-respawn`: Disable automatic respawn

**Default Behavior** (as of 2026-01-18):
All features are **enabled by default** for better out-of-box experience:
- Resource monitoring: ✅ enabled
- Diagnostic monitoring: ✅ enabled
- Web UI: ✅ enabled (http://127.0.0.1:8080)

**Config YAML (see `test/autoware_planning_simulation/autoware_config.yaml`):**
```yaml
composable_node_loading:
  load_node_timeout_millis: 30000
  max_concurrent_load_node_spawn: 10

list_nodes:
  loading_timeout_secs: 30    # Trigger verification after timeout
  rate_limit_secs: 5          # Min time between queries per container

monitoring:
  enabled: false
  sample_interval_ms: 2000  # Default: 2000ms (reduced from 1000ms for lower CPU overhead)

diagnostics:
  enabled: false               # Enable ROS2 diagnostic monitoring
  topics:                      # Topics to subscribe to
    - /diagnostics
    - /diagnostics_agg
  debounce_ms: 100             # Min time between diagnostic updates
  filter_hardware_ids: []      # Filter by hardware_id (empty = all)

processes:
  - node_pattern: "NODE 'rclcpp_components/component_container*"
    cpu_affinity: [0, 1]
    nice: 5
```

## Log Directory Structure

```
play_log/
├── latest -> 2025-12-21_09-44-52/  # Symlink to most recent
└── 2025-12-21_09-44-52/
    ├── params_files/
    ├── system_stats.csv           # System-wide metrics
    ├── diagnostics.csv            # ROS2 diagnostic messages (when enabled)
    └── node/<node_name>/          # Flat structure (no composable subdirs)
        ├── metadata.json          # For containers: includes composable_nodes array
        ├── metrics.csv            # Per-process metrics (when enabled)
        ├── out/err/pid/status
        └── cmdline
```

**Notes**:
- Composable nodes don't have separate directories. Metadata appears in parent container's `metadata.json`.
- `diagnostics.csv` contains all ROS2 diagnostic messages with columns: timestamp, hardware_id, diagnostic_name, level, level_name, message, key, value (each key-value pair is a separate row).

## Web UI

**Enabled by default** at http://127.0.0.1:8080 (disable with `--disable-web-ui`). Features:
- Two-panel layout with light/dark theme
- Node list with status colors, search/filter, clickable namespaces
- Per-node controls (Start/Stop/Restart) + bulk operations
- Container controls for composable nodes (Load/Unload)
- Real-time log streaming (stdout/stderr) with auto-reconnect
- Stderr activity indicator
- Auto-restart checkbox per node
- **Diagnostics page** (enabled by default, disable with `--disable-diagnostics`):
  - Real-time ROS2 diagnostic status monitoring
  - Sortable table by hardware_id, name, level, or timestamp
  - Color-coded by severity (OK/WARNING/ERROR/STALE)
  - Visual indicators for fresh diagnostics (<10s old)
  - Dashboard badge showing diagnostic counts
  - Auto-refresh every 5 seconds

Security: Binds to `127.0.0.1:8080` by default (localhost only). Use `--web-addr 0.0.0.0:8080` only on trusted networks.

## Development Practices

### Building
- **ALWAYS** use `just build` (NEVER `colcon build` directly)
- After code changes, run `just build` before testing

### Bash Tool Usage
- **ALWAYS** use Bash tool's `timeout` parameter: `Bash(command="...", timeout=15000)`
- **NEVER** prefix commands with `timeout`: ~~`timeout 15 play_launch replay`~~

### Avoiding Orphan Processes
When killing play_launch or ros2 launch:
- **NEVER** use `kill -9` on individual processes
- **ALWAYS** kill the entire process group (PGID):
  ```bash
  PGID=$(ps -o pgid= -p $PID | tr -d ' ')
  kill -TERM -$PGID
  sleep 2
  kill -9 -$PGID
  ```
- Use `just kill-orphans` to clean up stray ROS processes

### Temporary Files
- **ALWAYS** store temp files in `/home/aeon/repos/play_launch/tmp/` (gitignored)
- **NEVER** use system `/tmp` for project-related files

### Logging Practices

**Log Levels:**
- `error!`: Unrecoverable errors
- `warn!`: Recoverable errors/unexpected conditions
- `info!`: User-facing events (startup, shutdown, Start/Stop commands, major lifecycle events)
- `debug!`: Technical details (state transitions, service readiness, timing)

**Default `RUST_LOG=play_launch=info` should show only essential user-facing information.**

Examples:
```rust
// User-facing (info!)
info!("{}: Received Stop command, killing container (PID: {})", name, pid);

// Technical details (debug!)
debug!("{}: LoadNode service available after {}ms", name, elapsed);
debug!("{}: Transitioning {} blocked nodes to Unloaded", name, count);
```

Enable debug logs: `RUST_LOG=play_launch=debug play_launch replay`

## Key Recent Changes

- **2026-01-18**: Phase 1 resource optimizations - 46% CPU reduction (34.5% → 19.2%), 45% thread reduction (110 → 61), optimized tokio runtime (8 workers, 16 max blocking), ROS executor sleep (10ms → 50ms), monitoring interval (1000ms → 2000ms default)
- **2026-01-18**: CLI defaults changed - All features (monitoring, diagnostics, Web UI) now enabled by default; new `--enable <FEATURE>` flag for selective enabling; merged `--web-ui-addr`/`--web-ui-port` into `--web-addr <IP:PORT>`
- **2026-01-17**: Web UI modularization - Split 2,492-line index.html into 16 files (7 CSS modules, 8 JS modules) for better maintainability
- **2026-01-17**: Diagnostic monitoring system - Full ROS2 `/diagnostics` topic monitoring with Web UI, CSV logging, and real-time dashboard
- **2026-01-17**: Web UI diagnostics table - Sortable table with zebra striping, hover effects, color-coded severity, and fresh diagnostic indicators
- **2026-01-17**: Logging cleanup - Removed Web UI console.log statements, changed internal state transitions to debug! level
- **2026-01-17**: Fixed composable node log streaming - Added node_type/container_name fields to API, dynamic lookup
- **2026-01-17**: Web UI improvements - View button styling, auto-switch panel, hierarchical sorting, tab preservation
- **2026-01-17**: Unloading state implementation - Proper completion handling with immediate visual feedback
- **2026-01-16**: Composable node matching fix - Python/Rust coordination for full ROS names with namespaces
- **2026-01-16**: Web UI bulk operations - Start/Stop/Restart All, Load/Unload All per container
- **2026-01-14**: Container restart race condition fix - 200ms warmup delay after service ready
- **2026-01-14**: Logging level improvements - Changed many info! to debug! for cleaner output
- **2026-01-13**: Member name simplification - Removed "NODE ''" and "LOAD_NODE ''" prefixes
- **2026-01-13**: Phase 12 cleanup - Removed deprecated composable node actor code
- **2026-01-12**: Phase 12 - Container-managed composable nodes (virtual members)
- **2026-01-10**: ListNodes verification system - On-demand verification for stuck loading states
- **2026-01-09**: Orphan process prevention - PR_SET_PDEATHSIG implementation
- **2026-01-05**: Single shared ROS node architecture - Reduced resource usage
- **2026-01-03**: Async/Tokio refactoring - Unified shutdown handling
- **2026-01-01**: Actor pattern migration - Cleaner state management

## Testing

- `test/autoware_planning_simulation/`: Full Autoware test (52 composable nodes, 15 containers)
- `test/simple_test/`: Basic container with 2 nodes
- `test/sequential_loading/`: FIFO queue testing
- `test/concurrent_loading/`: Parallel loading testing
- Each test workspace has `justfile` with `just run`, `just run-debug`

## Distribution

- **Primary**: PyPI (`pip install play_launch`)
- Wheels for x86_64 and aarch64 (Ubuntu 22.04+)
- Binary optimization: 94% size reduction (strip + LTO)
- Build: `just build` → `dist/play_launch-*.whl`
- Publish: `just publish-pypi` or GitHub Actions on version tags
