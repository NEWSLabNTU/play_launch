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

### Execution Flow

1. **Load**: Deserialize `record.json`, copy parameter files
2. **Context Preparation**: Classify nodes (containers vs regular)
3. **Background Tasks**: Async tokio tasks for component loading, service discovery, monitoring, and process management
4. **Execution**: Actor-based lifecycle management for regular nodes, containers, and composable nodes
5. **Logging**: All output saved to `play_log/<timestamp>/`

### Module Structure

- **member_actor/**: Actor-based lifecycle management for nodes, containers, and composable nodes
- **execution/**: Process spawning and orchestration
- **ros/**: ROS integration (component loading, service discovery, package resolution)
- **monitoring/**: Resource monitoring and I/O helper client
- **web/**: Web UI server and API handlers
- **cli/**: Command-line interface and configuration
- **python/**: Python integration (dump_launch, analyzer)
- **event_driven/**: Web UI compatibility layer

### Runtime Architecture

- **Async/Tokio**: All background services run as async tasks with unified shutdown handling
- **Actor Pattern**: Self-contained lifecycle management for nodes with respawn support
- **ListNodes Verification**: On-demand query manager verifies composable node loading states when LoadNode calls hang or timeout
- **Web UI Mode**: Event-driven architecture bridges actor state to web interface

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

list_nodes:
  rate_limit_secs: 5                  # Min time between queries per container
  loading_timeout_secs: 30            # Trigger verification after this timeout
  unloading_timeout_secs: 10          # Reserved for future unload verification
  call_timeout_ms: 5000               # ListNodes service call timeout

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

Enable with `--enable-monitoring` flag. Collects per-node and system-wide metrics (CPU, memory, I/O, network, GPU).

### Visualization

`play_launch plot` generates interactive Plotly charts:
- Timeline charts: CPU, memory, I/O, network, GPU over time
- Distribution charts: Box plots for CPU and memory
- Container-aware hover panels showing composable nodes
- Statistics report with top 10 rankings

### Metrics Collected

- **Per-Process**: CPU, memory (RSS/VMS), disk I/O, threads, file descriptors, GPU (NVIDIA), TCP/UDP connections
- **System-Wide**: Overall CPU%, memory, network rates, disk I/O rates

### GPU Support

- **NVIDIA GPUs**: Per-process metrics via NVML
- **Jetson/Tegra**: System-wide only (hardware limitation)

### I/O Monitoring

Privileged processes (containers) require I/O helper with CAP_SYS_PTRACE capability:
- Run `just setcap-io-helper` after building (requires sudo)
- Verify with `just verify-io-helper`
- Reapply after rebuilds

**Note**: Jetson/Tegra platforms don't support `/proc/[pid]/io` (hardware limitation)

## Important Implementation Details

### Environment
- User must source ROS setup files before running play_launch
- Environment variables from launch files (`<env>` tags) are replayed to child processes

### Process Management
- Graceful shutdown with 5-second SIGTERM grace period before SIGKILL
- Multiple Ctrl-C presses escalate: SIGTERM → SIGQUIT → SIGKILL
- **Orphan process prevention**: PR_SET_PDEATHSIG ensures kernel sends SIGKILL to all child processes if play_launch crashes (even with SIGKILL -9)
- Process groups (PGID) provide redundant cleanup for graceful shutdown

### Composable Node Loading
- Service-based loading via rclrs LoadNode calls
- Container-managed FIFO queues (sequential per container, parallel across containers)
- Single load attempt (no retries) with automatic verification fallback
- **ListNodes Verification**: If loading takes >30s (configurable), automatically queries container to verify if node is already loaded
- Each node logs timing metrics (queue wait, service call, total duration)
- Rate-limited verification queries prevent ROS service overload (max 1 query per container per 5 seconds)

### Respawn Support
- Nodes with `respawn=True` automatically restart on exit
- Respawn delay configurable via `respawn_delay` parameter
- Supported for regular nodes and containers only (not composable nodes)
- Disable with `--disable-respawn` flag
- **Note**: `on_exit` handlers not supported

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
- Clean node names without internal prefixes (e.g., "control_validator" instead of "LOAD_NODE 'control_validator'")
- Live status updates every 5 seconds
- Per-node control buttons: Auto-restart checkbox (leftmost), Start/Stop (conditional), Restart, Details, Logs
- Auto-restart checkbox with visual state persistence and 1-second refresh delay
- Search/filter by node name or ROS namespace
- Health summary badges showing running/total counts (e.g., "42/46 nodes", "15/15 containers", "52/52 composable")
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
- Defaults to stderr when opened (most relevant for debugging)
- Switch between stdout/stderr tabs
- Real-time streaming via Server-Sent Events (SSE)
- Auto-scroll with manual override
- Append mode: logs preserved across restarts with timestamped markers
- Start/stop markers show when nodes were started/stopped
- Clear and scroll-to-bottom controls

**Node Control:**
- Individual node controls: Start, Stop, Restart buttons per card
- Actors remain alive after Stop, allowing Start/Restart commands to work
- Automatic refresh after control actions
- Prevents concurrent operations on same node
- JavaScript escaping handles node names with special characters (quotes, etc.)

### Technical Details

- Built with axum web server and htmx
- Server-Sent Events (SSE) for real-time log streaming
- Binds to `127.0.0.1` by default (localhost only) for security

## Dependencies

**Rust**: tokio, rayon, eyre, clap, serde/serde_json/serde_yaml, bincode, rclrs, composition_interfaces, rcl_interfaces, sysinfo, nvml-wrapper, csv, axum, tower-http, rust-embed, mime_guess

**Python**: pyyaml, lark, packaging, plotly (visualization)

## Testing

**Autoware integration test** (`test/autoware_planning_simulation/`):
- `just start-sim`: Start simulator with play_launch
- `just drive`: Run autonomous driving test
- `just plot`: Generate resource plots
- Tested with 52 composable nodes, 15 containers

**Container loading test workspaces**:
- `test/simple_test/`: Basic container with 2 composable nodes (talker, listener)
- `test/sequential_loading/`: 5 nodes in 1 container - tests FIFO queue processing
- `test/concurrent_loading/`: 4 nodes in 4 containers - tests parallel loading
- `test/mixed_loading/`: 5 nodes in 2 containers - tests both patterns
- Each workspace includes launch files, justfiles (`just run`, `just run-debug`, `just check-timing`), and README

**I/O stress test** (`test/io_stress/`):
- High-frequency I/O operations for testing monitoring overhead

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

## Key Recent Changes

- **2026-01-10**: ListNodes verification system (ALL PHASES COMPLETE) - Fully implemented on-demand verification of composable node loading states to resolve issues where nodes get stuck in "Loading" state. Phase 1: Added configuration (ListNodesSettings), event types (NodeDiscovered, ListNodesRequested, DiscoveredLoaded), and ListNodesManager module with rate-limiting and timeout handling. Phase 2: Integrated manager into coordinator spawn flow with automatic event forwarding from MemberRunner. Manager spawns as background task, receives ListNodesRequested events via dedicated channel, and broadcasts NodeDiscovered events back to state stream. Phase 3: Refactored composable_node_actor to detect loading timeouts and handle discovery. Added list_nodes_loading_timeout_secs to ComposableActorConfig (default 30s). Refactored handle_loading() to send LoadNode request non-blocking and enter select loop that monitors: LoadNode response, timeout for verification (emits ListNodesRequested), DiscoveredLoaded control event (transitions to Loaded), container state changes, and shutdown. Phase 4: Implemented node discovery matching in coordinator (coordinator.rs:835-916). Added handle_node_discovered() method that matches discovered nodes to composable actors by container name and full node name, then sends DiscoveredLoaded control event. Integrated into event loop (replay.rs:1066-1077) to process NodeDiscovered events before state cache update. SYSTEM NOW FULLY OPERATIONAL: composable nodes that are already loaded or take too long will be detected and verified through ListNodes queries.
- **2026-01-09**: Web UI Start/Restart after exit fix - Fixed Start button not working after process exits. Issue: actor terminated when process exited with respawn disabled, making control channel unavailable. Solution: keep actor alive in Stopped state to handle Start/Restart commands (regular_node_actor.rs:236).
- **2026-01-09**: Web UI Auto-restart checkbox fix - Fixed checkbox reverting after toggling. Issue: metadata was immutable after spawning, so Web UI always read the initial state. Solution: made metadata mutable (wrapped in `Arc<RwLock>`) and update it when ToggleRespawn control is sent (coordinator.rs:384, 633-645).
- **2026-01-09**: Container matching fix - Fixed composable nodes not finding their target containers. Issue: containers were stored with leading slash (e.g., "/pointcloud_container") but composable nodes looked them up without it (e.g., "pointcloud_container"). Solution: normalize target container names by prepending "/" if missing before lookup (coordinator.rs:289-295).
- **2026-01-09**: Code quality improvements - Refactored completion functions to use `CompletionContext` struct, reducing function parameters from 8 to 5 (Unix) / 4 (Windows) and fixing clippy warnings (replay.rs:636-642).
- **2026-01-09**: Shutdown responsiveness fix - Fixed Ctrl-C not working during cleanup phase. Added 2-second timeout to background task cleanup - stubborn tasks are forcefully aborted after timeout. Background tasks now tracked by name (anchor, stats, monitor, service_discovery, web_ui) with debug output showing which tasks are stubborn. Maintains signal handling throughout shutdown with proper 3-stage escalation (SIGTERM → SIGTERM → SIGKILL). Added 1-second timeout to service discovery ROS FFI calls.
- **2026-01-09**: Orphan process prevention - Implemented PR_SET_PDEATHSIG to prevent orphan processes when play_launch crashes. Kernel automatically sends SIGKILL to all children if parent dies.
- **2026-01-09**: Web UI improvements - Composable nodes display under containers, logs default to stderr, health badges show running/total counts (e.g., "42/46 nodes"), clean node names without prefixes, Auto-restart checkbox moved to leftmost position.
- **2026-01-05**: Single shared ROS node architecture - Consolidated to one shared node (`/play_launch`) with dedicated executor thread. Reduced resource usage and eliminated race conditions.
- **2026-01-05**: Load timing metrics - Each composable node logs `load_timing.csv` with queue wait, service call, and total duration metrics.
- **2026-01-03**: Async/Tokio refactoring complete - Converted all background threads to async tasks with unified shutdown handling. Shutdown response improved from ~1s to <100ms.
- **2026-01-01**: Actor pattern migration - Replaced event-driven architecture with actor pattern for all node types. Cleaner state management and better testability.
- **2025-12-22**: Web UI enhancements - Added clickable namespace segments, stderr activity indicator, and append-mode logs with timestamped markers.
- **2025-12-21**: Web UI redesign - Two-panel IDE-style layout with light/dark theme support and Firefox-style JSON viewer.
- **2025-12-21**: Improved shutdown - 5-second SIGTERM grace period for large systems like Autoware.
- **2025-11-30**: PyPI-only distribution - Removed Debian packaging, simplified build system.

## Development Practices

### Bash Tool Usage
- **ALWAYS** use the Bash tool's `timeout` parameter instead of prefixing commands with `timeout`
- **GOOD**: `Bash(command="play_launch replay", timeout=15000)` (timeout in milliseconds)
- **BAD**: `Bash(command="timeout 15 play_launch replay")` - blocks the entire command
- Reason: Long-running commands like `play_launch` can block, and the Bash tool's timeout provides better control

### Temporary Files
- **ALWAYS** store all temporary files in `tmp/` directory at the project root (`/home/aeon/repos/play_launch/tmp/`)
- This directory is gitignored and used for testing, debugging, and experimentation
- **NEVER** use system `/tmp` for project-related temporary files
- Use `$project/tmp/` for all test runs, temporary log directories, and scratch work
- Example: `cd /home/aeon/repos/play_launch/tmp && play_launch ...`

### External Dependencies
- `external/` directory is for placing 3rd party projects for study and reference
- **Gitignored** - not part of the project build

### Language Server Protocol (LSP) Tools
- **pyright-lsp**: Available for Python type checking and code intelligence
- **rust-analyzer-lsp**: Available for Rust code intelligence and refactoring
- These LSP tools can be used for advanced code analysis, navigation, and refactoring tasks
- Use these tools when performing complex code transformations or when you need deep code understanding

### Building the Project
- **ALWAYS** use `just build` to rebuild the project (NEVER use `colcon build` directly)
- `just build` handles the correct build sequence and ensures proper integration
- After code changes, always run `just build` before testing

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
