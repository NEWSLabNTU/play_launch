# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Launch Inspection Tool - Records and replays ROS 2 launch file executions for performance analysis:
1. **dump_launch** (Python): Records launch execution to `record.json`
2. **play_launch** (Rust): Replays recorded execution with resource monitoring
3. **play_launch_wrapper** (CMake): Provides `play_launch` command in PATH
4. **play_launch_analyzer** (Python): Analyzes and visualizes logs

## Installation

### Option 1: pip install (Recommended for Users)

```sh
# Prerequisites: ROS2 Humble must be installed and sourced
source /opt/ros/humble/setup.bash

# Install from PyPI
pip install play_launch

# Or install from local wheel after building
pip install dist/play_launch-*.whl

# Optional: Enable privileged I/O monitoring
play_launch setcap-io-helper    # Requires sudo

# Verify I/O helper status
play_launch verify-io-helper
```

### Option 2: Build from Source (For Development)

```sh
# Install dependencies (first-time setup)
just install-deps        # Install colcon-cargo-ros2, run rosdep

# Build workspace and wheel (single-stage colcon build + wheel packaging)
just build               # Outputs wheel to dist/play_launch-*.whl

# Enable I/O monitoring for privileged processes
just setcap-io-helper    # Requires sudo, reapply after rebuild

# Verify I/O helper status
just verify-io-helper

# Source workspace (when running commands outside of just)
. install/setup.bash
```

## Usage

```sh
# Run play_launch with arguments (colcon build: use just run, pip: use directly)
just run launch <package> <launch_file>         # Example: just run launch demo_nodes_cpp talker_listener.launch.py
just run --help                                 # Show play_launch help

# Or if installed via pip or sourced:
play_launch launch <package> <launch_file>      # Record & replay
play_launch run <package> <executable>          # Run single node
play_launch dump launch <package> <launch_file> # Record only
play_launch replay --config config.yaml         # Replay with config

# Analysis
play_launch plot                                # Plot latest logs
play_launch plot --metrics cpu memory           # Plot specific metrics
```

## Architecture

### Execution Flow (play_launch)

1. **Load** (launch_dump.rs): Deserialize `record.json`, copy parameter files
2. **Context Preparation** (context.rs): Classify nodes (containers vs regular)
3. **Component Loader** (component_loader.rs): Background thread with rclrs service clients for LoadNode
4. **Execution** (execution.rs):
   - Regular nodes: Spawned directly
   - Composable nodes: Load via service calls to containers (or standalone with `--standalone-composable-nodes`)
5. **Container Readiness** (container_readiness.rs): Wait for LoadNode services (default: enabled)
6. **Logging**: All output saved to `play_log/<timestamp>/`

### Key Modules

- **main.rs**: Entry point, CleanupGuard for subprocess management
- **execution.rs**: Process spawning, composable node loading
- **component_loader.rs**: Direct rclrs service calls to LoadNode
- **resource_monitor.rs**: Per-node and system-wide metrics collection
- **config.rs**: YAML configuration with process control (CPU affinity, nice)
- **options.rs**: CLI parsing
- **node_registry.rs**: Node tracking and control for web UI
- **web/mod.rs**: axum web server with embedded assets
- **web/handlers.rs**: HTTP API handlers for node control
- **web/sse.rs**: Server-Sent Events for log streaming

## Configuration

### CLI Flags

- `--config <PATH>` (`-c`): Runtime config YAML
- `--verbose` (`-v`): Show per-node progress (default: summary only)
- `--enable-monitoring`: Enable resource monitoring
- `--monitor-interval-ms <MS>`: Sampling interval (default: 1000ms)
- `--standalone-composable-nodes`: Run composable nodes standalone
- `--load-orphan-composable-nodes`: Load nodes without matching containers
- `--disable-respawn`: Disable automatic respawn even if configured in launch file
- `--web-ui`: Enable web-based management interface (default: disabled)
- `--web-ui-port <PORT>`: Web UI port (default: 8080)

### Config YAML Example

```yaml
composable_node_loading:
  load_node_timeout_millis: 30000
  max_concurrent_load_node_spawn: 10

container_readiness:
  wait_for_service_ready: true        # Enabled by default
  service_ready_timeout_secs: 120

monitoring:
  enabled: false
  sample_interval_ms: 1000

processes:
  - node_pattern: "NODE 'rclcpp_components/component_container*"
    monitor: true
    cpu_affinity: [0, 1]
    nice: 5                           # Negative nice requires CAP_SYS_NICE
```

See `test/autoware_planning_simulation/autoware_config.yaml` and `docs/resource-monitoring-design.md` for details.

## Log Directory Structure

```
play_log/
├── latest -> 2025-12-21_09-44-52/  # Symlink to most recent run
├── 2025-12-21_09-44-40/
├── 2025-12-21_09-44-46/
└── 2025-12-21_09-44-52/
    ├── params_files/              # Cached parameter files
    ├── system_stats.csv           # System-wide metrics
    ├── node/<node_name>/          # Regular nodes (flat structure)
    │   ├── metadata.json          # Package, namespace, container info
    │   ├── metrics.csv            # Resource metrics (when enabled)
    │   ├── out/err/pid/status     # Process logs
    │   └── cmdline                # Executed command
    └── load_node/<node_name>/     # Composable nodes (flat structure)
        ├── metadata.json
        ├── metrics.csv
        ├── service_response.*     # LoadNode responses
        └── status
```

**Features**:
- **`latest` symlink**: Automatically updated to point to the most recent run
- Flat 1-level structure for easy scripting
- Short directory names (e.g., `control_evaluator`) with deduplication (`_2`, `_3`)
- Each node directory is self-contained with all data
- `play_launch plot` uses `latest` symlink by default (no need to specify log directory)

## Resource Monitoring

### Status
- ✅ **Phase 1 Complete**: Per-node and system-wide monitoring (CPU, memory, I/O, network, threads, FDs)
- ✅ **Phase 2 Complete**: GPU monitoring (NVIDIA), I/O rates, network connections, CAP_SYS_NICE
- ✅ **Phase 3.1 Complete**: Interactive Plotly dashboard with zoom/pan/hover tooltips

### Visualization
The `play_launch plot` command generates individual interactive HTML charts using Plotly:

**Timeline Charts** (~4-5 MB each, shows metrics over time):
- `cpu_timeline.html` - CPU usage with container-aware hover panel
- `memory_timeline.html` - Memory usage with container-aware hover panel
- `io_timeline.html` - I/O read/write rates (when available)
- `network_timeline.html` - TCP/UDP connections (when available)
- `gpu_timeline.html` - GPU memory usage (when available)
- `gpu_temp_power.html` - GPU temperature and power (when available)
- `gpu_clocks.html` - GPU clock frequencies (when available)

**Distribution Charts** (box plots sorted low to high by average):
- `cpu_distribution.html` - CPU distribution with statistics panel
- `memory_distribution.html` - Memory distribution with statistics panel

**Interactive Features**:
- Full-screen viewing (separate files for maximum chart size)
- Container-aware floating panels:
  - Timeline charts: Hover over container curve → shows list of contained composable nodes
  - Distribution charts: Hover over container → shows box plot statistics + contained nodes
- Abbreviated labels in distributions, full names on hover
- No legend clutter (names in hover tooltips only)
- Zoom and pan (drag to zoom, double-click to reset)
- Download as PNG via toolbar

**Statistics Report**: `statistics.txt` with top 10 rankings for all metrics

**Implementation**: JavaScript injection pattern (inject_statistics_panel, inject_container_panel) embeds container_map as JSON, adds plotly_hover/plotly_unhover event listeners for dynamic panels

### Per-Process Metrics (metrics.csv)
CPU, memory (RSS/VMS), disk I/O, total I/O with rates, process state, threads, file descriptors, GPU (NVIDIA), TCP/UDP connections

### System-Wide Metrics (system_stats.csv)
Overall CPU%, memory, network rates, disk I/O rates, GPU stats (Jetson via jtop - pending)

### GPU Support
- **NVIDIA GPUs**: Per-process metrics via NVML (nvml-wrapper 0.11)
- **Jetson/Tegra**: ⚠️ Per-process GPU metrics NOT available (hardware limitation). Use system-wide monitoring.

### Platform Notes
- **CPU metrics**: Accurate per-process measurement via `/proc/[pid]/stat` (utime + stime). Correctly shows individual process CPU usage independent of system loading.
- **I/O metrics**: See dedicated I/O Monitoring section below
- **Negative nice values**: Requires `sudo setcap cap_sys_nice+ep install/play_launch/lib/play_launch/play_launch` (reapply after rebuild)

### I/O Monitoring

**Standard processes**: Direct `/proc/[pid]/io` reads work without special permissions

**Privileged processes** (containers, capabilities-enabled binaries): Require helper daemon
- Helper binary: `play_launch_io_helper` (built with main package)
- Capability required: `CAP_SYS_PTRACE` (set via `just setcap-io-helper`)
- Architecture: Anonymous pipes for IPC, PR_SET_PDEATHSIG for cleanup
- Batch processing: Single IPC call per monitoring interval (efficient)

**Without helper/capabilities**:
- Warning logged once: "I/O helper unavailable. Privileged processes will have zero I/O stats."
- I/O fields show zeros for affected processes
- Monitoring continues normally for other metrics

**Extended I/O metrics** (7 fields from `/proc/[pid]/io`):
- `rchar` / `wchar` - Total bytes read/written (includes cache)
- `read_bytes` / `write_bytes` - Actual storage I/O (excludes cache)
- `syscr` / `syscw` - Read/write syscall counts
- `cancelled_write_bytes` - Writes later truncated

**Reapply after rebuild**: File capabilities are cleared when binaries change

**Jetson/Tegra note**: `/proc/[pid]/io` not available (hardware limitation). System-wide I/O monitoring still works.

### I/O Monitoring Troubleshooting

**"I/O helper unavailable" warning**:
1. Check binary exists: `ls install/play_launch/lib/play_launch/play_launch_io_helper`
2. If missing: `just build`
3. Set capability: `just setcap-io-helper`
4. Verify: `just verify-io-helper`

**Zero I/O stats for containers**:
- Likely missing CAP_SYS_PTRACE on helper
- Run: `just setcap-io-helper`

**"Permission denied" in helper logs**:
- Check helper capability: `getcap install/play_launch/lib/play_launch/play_launch_io_helper`
- Should show: `cap_sys_ptrace+ep`

## Important Implementation Details

### Environment
- **User responsibility**: Source ROS setup files before running play_launch
- play_launch does NOT source setup files internally
- AMENT_PREFIX_PATH explicitly preserved for containers (node_cmdline.rs)
- Environment variables from launch files (`<env>` tags) replayed to child processes

### Process Cleanup
- **CleanupGuard**: RAII pattern kills all children on exit
- **Signal Handlers**: SIGTERM/SIGINT call `kill_all_descendants()`
- **Process Group Isolation**: `.process_group(0)` prevents children from receiving terminal signals
- Only play_launch receives Ctrl-C, then explicitly kills all descendants
- **Graceful Shutdown**: SIGTERM triggers 5-second grace period before SIGKILL (allows large systems like Autoware to shut down cleanly)
- **Progressive SIGINT**: Multiple Ctrl-C presses escalate from SIGTERM → SIGQUIT → SIGKILL for manual control

### Composable Node Loading
- Service-based only (direct rclrs service calls to LoadNode)
- Service clients cached per container
- Concurrent loading (default: 10 parallel)
- Parameters auto-converted to ROS types
- Retry logic: 3 attempts, 30s timeout per node
- Namespace resolution: Relative names (e.g., `pointcloud_container`) resolved to absolute (e.g., `/pointcloud_container`)

### Respawn Support
- **Automatic Restart**: Nodes with `respawn=True` in launch files automatically restart when they exit
- **Respawn Delay**: `respawn_delay` parameter controls delay (in seconds) before restarting
- **Graceful Shutdown**: Ctrl-C immediately stops respawning nodes (fixed via tokio::sync::watch channel for persistent shutdown state)
- **Regular Nodes Only**: Currently supports respawn for regular nodes and containers (composable nodes loaded into containers do NOT auto-reload on container restart)
- **CLI Override**: Use `--disable-respawn` flag to disable all respawn behavior for testing
- **on_exit Handlers**: Not supported - warning logged if detected in launch files
- **Infinite Restarts**: Matches ROS2 launch behavior (no retry limit by default)

## Web UI

Optional web-based interface for monitoring and controlling nodes during replay.

### Enabling

```sh
# Start replay with web UI (localhost only, default port 8080)
play_launch replay --web-ui

# Custom port
play_launch replay --web-ui --web-ui-port 3000

# Expose to network (INSECURE - use with caution on trusted networks only)
play_launch replay --web-ui --web-ui-addr 0.0.0.0

# Bind to specific interface
play_launch replay --web-ui --web-ui-addr 192.168.1.100 --web-ui-port 3000
```

### Security

- **Default: Localhost Only**: Web server binds to `127.0.0.1` by default for security
- **Configurable**: Use `--web-ui-addr` to bind to specific interface or expose to network
- **Warning**: Binding to `0.0.0.0` exposes the web UI to the network without authentication
- **No Authentication**: Designed for trusted local or network access only
- **Recommendation**: Only use `0.0.0.0` on isolated/trusted networks

### Features

**Two-Panel Layout:**
- **Left Panel**: Compact node list with status-based background colors
- **Right Panel**: Collapsible details/logs panel (hidden by default)
- **Close Button**: Hide the right panel when done viewing

**Theme Support:**
- **Light Theme**: Clean, bright interface
- **Dark Theme**: Eye-friendly dark mode
- **System Default**: Automatically follows OS theme preference
- **Theme Toggle**: Switch themes with button click (☀/🌙 icon)

**Node List:**
- Compact cards with status reflected in background color (running=green, stopped=gray, failed=red, pending=yellow)
- Container hierarchy: composable nodes indented under their containers
- Live status updates every 5 seconds
- Per-node control buttons (Start, Stop, Restart, Details, Logs)
- Search/filter by node name or ROS namespace
- Health summary badges showing running/total counts
- Clickable ROS namespace segments for instant filtering

**Stderr Activity Indicator:**
- 📋 icon appears next to PID when node has stderr output
- Yellow (jumping) for active output (0-10 seconds old)
- Orange (static) for recent output (10-60 seconds old)
- Hover over icon → tooltip shows last 5 stderr lines
- Icon auto-disappears after 60 seconds of no activity

**Namespace Filtering:**
- ROS names rendered as clickable segments (e.g., `/planning/mission/planner`)
- Click any segment → filters to that namespace
- Clicked namespace populates search box (editable)
- Progressive filtering: click deeper segments for narrower results
- Hover highlights show which segments are clickable

**Node Details:**
- Firefox-style JSON viewer with expandable rows
- Formatted key-value pairs for easy reading
- Type-specific color coding (strings=red, numbers=purple, booleans=blue)
- Shows all node metadata (PID, package, paths, command line)

**Log Viewer:**
- Integrated into right panel (no modal overlay)
- Switch between stdout/stderr tabs
- Real-time streaming via Server-Sent Events (SSE)
- Auto-scroll with manual override
- Append mode: logs preserved across restarts with timestamped markers
- Start/stop markers show when nodes were started/stopped
- Clear and scroll-to-bottom controls

**Node Control:**
- Individual node controls: Start, Stop, Restart buttons per card
- Automatic refresh after control actions
- Prevents concurrent operations on same node

### Architecture

- **Framework**: axum web server with embedded static assets
- **Frontend**: htmx for reactive updates + vanilla JavaScript for interactions
- **UI Pattern**: Two-panel IDE-style layout with collapsible sidebar
- **Log Streaming**: Server-Sent Events (SSE) with file tailing
- **Registry**: NodeRegistry tracks all nodes, determines status from filesystem
- **Theme Management**: CSS variables + localStorage persistence + system theme detection
- **Binding**: Configurable via `--web-ui-addr` (default: `127.0.0.1` localhost only)
- **Port**: Configurable via `--web-ui-port` (default: 8080)

### API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Serve embedded index.html |
| `/api/nodes` | GET | HTML fragment with node cards |
| `/api/nodes/{name}` | GET | JSON node details |
| `/api/nodes/{name}/start` | POST | Start node |
| `/api/nodes/{name}/stop` | POST | Stop node |
| `/api/nodes/{name}/restart` | POST | Restart node |
| `/api/nodes/{name}/logs/stdout` | GET | SSE stream of stdout |
| `/api/nodes/{name}/logs/stderr` | GET | SSE stream of stderr |
| `/api/health` | GET | HTML health summary badges |

## Dependencies

**Rust**: tokio, rayon, eyre, clap, serde/serde_json/serde_yaml, bincode, rclrs, composition_interfaces, rcl_interfaces, sysinfo, nvml-wrapper, csv, axum, tower-http, rust-embed, mime_guess

**Python**: pyyaml, lark, packaging, plotly (visualization)

## Testing

Autoware planning simulator integration test in `test/autoware_planning_simulation/`:
- `just start-sim`: Start simulator with play_launch
- `just drive`: Run autonomous driving test
- `just plot`: Generate resource plots
- Tested with 52 composable nodes, 15 containers

## Distribution & Packaging

### PyPI Package

The primary distribution method is via PyPI:

```bash
pip install play_launch
```

Wheels are built for both x86_64 and aarch64 (Ubuntu 22.04+).

**Binary optimization**:
- Release profile with `strip = true` and `lto = "thin"`
- 94% size reduction: 137MB → 8.7MB
- See `Cargo.toml` [profile.release] section

### Build System

- **justfile**: Task runner (replaced Makefile 2025-11-04)
- Use `just --list` to see all available recipes
- Main recipes:
  - `install-deps`: Install colcon-cargo-ros2, run rosdep (interactive prompts for conflicts)
  - `build`: Single-stage colcon build + wheel packaging (outputs to dist/)
  - `run *ARGS`: Run play_launch with arguments (e.g., `just run launch demo_nodes_cpp talker_listener.launch.py`)
  - `setcap-io-helper`: Apply CAP_SYS_PTRACE to I/O helper (requires sudo)
  - `verify-io-helper`: Verify I/O helper capability status
  - `clean`: Remove build/install/log/dist directories and copied binaries
  - `test-wheel`: Test wheel installation in fresh venv
  - `publish-pypi`: Publish wheel to PyPI (requires PYPI_TOKEN env var)
  - `publish-testpypi`: Publish wheel to TestPyPI (requires TESTPYPI_TOKEN env var)
  - `test`: Run package tests
  - `lint`: Run clippy and ruff
  - `format`: Format Rust and Python code

### GitHub Workflows

**CI Workflow** (`.github/workflows/ci.yml`):
- Triggers on push to main and pull requests
- Runs on Ubuntu 22.04 with ROS2 Humble
- Executes: `just build`, `just test`, `just lint`
- Verifies code quality and functionality

**Wheel Release Workflow** (`.github/workflows/release-wheel.yml`):
- Triggers on version tags (`v*`) or manual dispatch
- Builds Python wheels for both amd64 and arm64 architectures
- Uses Docker containers with QEMU emulation for cross-compilation
- Build process:
  1. Set up Docker container with ROS2 Humble
  2. Build Rust binaries with colcon (colcon-cargo-ros2)
  3. Copy binaries to `python/play_launch/bin/`
  4. Build wheel with setuptools
  5. Rename wheel with platform-specific tag (linux_x86_64 or linux_aarch64)
- Uploads wheels to GitHub release and publishes to PyPI (trusted publishing)
- Note: Wheels are platform-specific due to embedded Rust binaries

**PyPI Publishing**:
- Automatic: Triggered by version tags via GitHub Actions (trusted publishing)
- Manual: `just publish-pypi` (requires PYPI_TOKEN env var)

## Key Recent Fixes

- **2025-12-22**: Web UI enhancements - Added clickable namespace segments for instant filtering (click any segment in ROS name like `/planning` to filter nodes by namespace). Implemented stderr activity indicator (📋 icon) next to PID with hover tooltip showing last 5 stderr lines (yellow/jumping for active output 0-10s, orange/static for recent 10-60s). Changed log files to append mode with timestamped start/stop markers. Removed buggy bulk control buttons (Start/Stop/Restart All). Enhanced search to match both node names and ROS namespaces.
- **2025-12-21**: Autoware justfile cleanup - Simplified recipe output to show only one line for start/stop/restart operations with clear status indicators (✓/⚠/✗). Ensured all {stop,restart,logs,status}-{sim,demo} recipes are available. Updated start scripts to output single-line success messages with Web UI URLs. Removed verbose multi-line output for cleaner CLI experience.
- **2025-12-21**: Complete Web UI redesign - Implemented two-panel IDE-style layout with collapsible right sidebar for details/logs. Added light/dark theme support with system default detection and toggle button. Created Firefox-style expandable JSON viewer for node details. Moved log viewer from modal overlay to right panel with stdout/stderr tabs. Made node list more compact with status reflected in background colors (running=green, stopped=gray, failed=red, pending=yellow). Right panel hidden by default with close button (×).
- **2025-12-21**: Autoware test scripts reorganization - Simplified script structure by moving systemd-run logic into wrapper scripts. Created start-{sim,demo}.sh (wrappers with systemd-run) and start-{sim,demo}-inner.sh (execution logic). Removed duplicated start-sim-and-drive.sh. Fixed "Access denied" error by resolving absolute paths and conditionally setting DISPLAY environment variable only when non-empty.
- **2025-12-21**: Improved shutdown speed for large systems - Increased SIGTERM grace period from 200ms to 5 seconds in play_launch to allow large systems (like Autoware) to shut down cleanly. Added timeout-based cleanup (10s) in start-demo-systemd.sh script with automatic SIGKILL fallback. Fixes slow `just stop-demo` issue where systemd would wait for 90s timeout.
- **2025-12-21**: Web UI route fix - Fixed Axum routing panic with catch-all parameter. Changed `/static/{*path}` to `/static/*path` for compatibility with newer Axum versions. Web UI now works correctly with `play_launch launch --web-ui` command.
- **2025-12-21**: Web UI security and configurability - Changed default bind address from `0.0.0.0` (network-exposed) to `127.0.0.1` (localhost-only) for security. Added `--web-ui-addr` flag to allow users to explicitly configure bind address when network access is needed. Logs warning when binding to `0.0.0.0`.
- **2025-12-21**: Log directory symlink - Added `play_log/latest` symlink that automatically points to the most recent timestamped log directory. Updated after each run. `play_launch plot` uses this symlink by default, eliminating the need to specify log directories manually.
- **2025-12-21**: ROS Jazzy compatibility - Fixed import compatibility for both ROS Humble and Jazzy. Humble uses `launch.some_actions_type.SomeActionsType` while Jazzy uses `launch.some_entities_type.SomeEntitiesType`. Added try/except fallback to support both.
- **2025-12-18**: SetParameter (global params) support - Global parameters from `<set_parameter>` XML tag or `SetParameter` Python action are now captured in a separate `NodeRecord.global_params` field (not merged into `params`). dump_launch extracts from `context.launch_configurations['global_params']` (the same source ROS2 launch uses internally). play_launch (Rust) reads this field and passes them as `-p` flags, with node-specific params able to override. Clean separation between global and node-specific params for better debugging.
- **2025-11-30**: Simplified packaging - Removed Debian packaging (PyPI-only distribution). Merged `build` and `build-wheel` recipes. Added `setcap-io-helper` and `verify-io-helper` CLI commands to Rust binary. Removed unused pandas/numpy dependencies from plot script (pure plotly). Auto-detect ROS2 distro (humble for Ubuntu 22.04, jazzy for 24.04) in justfile.
- **2025-11-26**: pip installation support - Added unified Python package in `python/play_launch/` that embeds dump_launch and play_launch_analyzer. Build workflow: `just build` then `just build-wheel`. Python CLI wrapper finds bundled Rust binary in package `bin/` directory. Wheel includes both Rust binaries (~3.3MB total). GitHub Actions workflow builds wheels for amd64 and arm64.
- **2025-11-23**: GitHub Actions CI/CD - Created automated workflows for continuous integration (build/test/lint on every push) and release automation (multi-architecture Debian package building on version tags). Release workflow uses QEMU cross-compilation for ARM64 packages (~20-30min), works on GitHub free tier without special runners.
- **2025-11-23**: Build system refactoring - Migrated to colcon-cargo-ros2 (single-stage build). Removed boilerplate packages from src/ros2_rust and src/interface. Simplified debian/rules to use single-stage colcon build (removed wheel-building complexity, fixed symlink issues). Enhanced justfile with install-deps recipe (interactive conflict resolution), run recipe, and verify-io-helper. All recipes now properly source /opt/ros/humble/setup.bash.
- **2025-11-04**: Build system migration - Replaced Makefile with justfile for cleaner syntax. Created Debian packaging with proper Ubuntu 22.04 paths.
- **2025-11-03**: Binary optimization - 94% size reduction (137MB → 8.7MB) via Cargo release profile with strip+LTO.
- **2025-11-03**: Fixed Python dependency - Replaced ruamel.yaml with standard PyYAML (system package).
- **2025-11-03**: Binary optimization - Added Cargo.toml release profile (strip=true, lto="thin") and Makefile --cargo-args --release flag. Reduced play_launch from 109MB to 6.5MB, play_launch_io_helper from 28MB to 2.2MB (94% total reduction).
- **2025-11-03**: Fixed ROS deprecation warnings - replaced ruamel.yaml with standard PyYAML in dump_launch (utils.py). Changed LaunchInspector argv to empty list since launch arguments are passed separately via launch_arguments parameter (__init__.py).
- **2025-11-03**: Fixed "Found remap rule" warnings during replay - changed ROS context initialization in component_loader.rs and container_readiness.rs to use minimal args vector instead of std::env::args(). Launch arguments (e.g., start_rviz:=true) are not ROS node arguments and should not be passed to rclrs::Context::new().
- **2025-10-30**: Fixed respawn race condition - replaced `tokio::sync::Notify` with `tokio::sync::watch` channel for persistent shutdown state. Respawning nodes (like RViz) now stop immediately on Ctrl-C without spurious restarts. Watch channel provides both immediate `.borrow()` checks and awaitable `.changed()` notifications.
- **2025-10-30**: Respawn support added - nodes with respawn=True automatically restart on exit, honoring respawn_delay. Ctrl-C during respawn delay stops restart gracefully. Only regular nodes supported (composable nodes limitation documented).
- **2025-10-29**: CPU metrics completely rewritten - now parses `/proc/[pid]/stat` directly for accurate utime/stime measurement. Previous implementation incorrectly used `run_time()` (wall-clock time) instead of actual CPU time, causing all processes to show similar CPU% affected by system loading.
- **2025-10-29**: I/O warning for Jetson (`/proc/[pid]/io` unavailable)
- **2025-10-29**: Logging verbosity control (`--verbose` for per-node details)
- **2025-10-28**: NVML library loading (nvml-wrapper 0.11)
- **2025-10-27**: Namespace resolution for composable nodes
- **2025-10-26**: Flat log directory structure with metadata.json
- **2025-10-25**: AMENT_PREFIX_PATH explicit preservation
- **2025-10-22**: Service-based component loading (rclrs, no subprocess overhead)
- **2025-10-22**: Process group isolation for proper cleanup

## Development Practices

### Temporary Files
- **ALWAYS** store all temporary files in `tmp/` directory at the project root (`/home/aeon/repos/play_launch/tmp/`)
- This directory is gitignored and used for testing, debugging, and experimentation
- **NEVER** use system `/tmp` for project-related temporary files
- Use `$project/tmp/` for all test runs, temporary log directories, and scratch work
- Example: `cd /home/aeon/repos/play_launch/tmp && play_launch ...`

### External Dependencies (Study/Reference)
- `external/` directory contains cloned ROS repositories for study and reference
- **Gitignored** - not part of the project build
- Currently contains:
  - `external/launch/` - ROS 2 launch (humble branch)
  - `external/launch_ros/` - ROS 2 launch_ros (humble branch)
- Used for understanding ROS launch architecture for potential Rust rewrite
- See `docs/launch-rust-rewrite-analysis.md` for analysis

## Performance Optimization Roadmap

### dump_launch Performance
- **Current**: Python-based launch introspection (slow for large launch files)
- **Analysis**: See `docs/launch-rust-rewrite-analysis.md`
- **Options**:
  1. Short-term: Optimize Python implementation (1-2 weeks)
  2. Medium-term: Rust XML/YAML parser + visitor (2-3 months)
  3. Long-term: Full Rust launch implementation (6+ months)
- **Challenge**: Python `.launch.py` files have arbitrary code execution
- **Target**: 10x speedup for Autoware-sized launch files

## Known Issues / TODO

(No known issues at this time)
