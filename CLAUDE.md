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
4. **Execution**: Actor-based lifecycle management for regular nodes and containers; composable nodes are virtual members managed by their parent containers
5. **Logging**: All output saved to `play_log/<timestamp>/`

### Module Structure

- **member_actor/**: Actor-based lifecycle management for nodes and containers; composable nodes as virtual members
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
- **Virtual Members (Phase 12)**: Composable nodes are managed as internal state by container actors, not separate actors. Container actors handle LoadNode service calls, state transitions, and timeout detection. Control events (Start/Stop/Restart) for composable nodes are routed to parent containers and translated to Load/Unload operations.
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
    └── node/<node_name>/          # Regular nodes and containers (flat structure)
        ├── metadata.json          # Package, namespace, composable_nodes array for containers
        ├── metrics.csv            # Resource metrics (when enabled) - per-process only
        ├── out/err/pid/status     # Process logs
        └── cmdline                # Executed command
```

**Features**:
- **`latest` symlink**: Automatically updated to point to the most recent run
- Flat 1-level structure for easy scripting
- Short directory names (e.g., `control_evaluator`) with deduplication (`_2`, `_3`)
- Each node directory is self-contained with all data
- **Phase 12**: Composable nodes no longer have separate log directories. Their metadata is included in the parent container's `metadata.json` under the `composable_nodes` array. Metrics are per-process (container only).
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
- Bulk operations: Start All, Stop All, Restart All buttons in header
- Bulk operations target regular nodes and containers only (composable nodes managed by containers)
- Per-container bulk operations: Load All / Unload All for composable nodes within each container
- Operations return count of affected members for user feedback

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

- **2026-01-17**: Unloading state implementation with proper completion handling and immediate visual feedback - Fixed Unloading state never transitioning to Unloaded by implementing a complete response mechanism similar to Loading. **Root cause**: The spawned unload task sent StateEvent to coordinator but never updated the container actor's internal composable_nodes state from Unloading to Unloaded. **Solution**: (1) Added CurrentUnload struct to track in-flight unload operations (container_control.rs:124-132). (2) Modified handle_unload_composable to spawn tracked task stored in current_unload field (container_actor.rs:806-820). (3) Added select! arm to poll current_unload.task and transition to Unloaded on success (container_actor.rs:1616-1684). (4) Updated drain_queue to cancel current_unload when container crashes (container_actor.rs:619-625). (5) Removed "X running" processes badge from Web UI (handlers.rs:776-786). (6) Improved button rendering: Loaded→Unload button, Loading→disabled "Loading...", Unloaded→Load button, Unloading→disabled "Unloading...", Blocked→disabled Load, Failed→Load for retry (handlers.rs:200-263). (7) Added Unloading CSS colors: light (peach/orange #ffe5b4), dark (amber #3d2e1a) (index.html:41-43, 85-87, 419-422). (8) Fixed immediate visual feedback: Modified load_node/unload_node handlers to return updated HTML instead of text, changed hx-swap from "none" to "outerHTML" with hx-target="closest .node-card" to immediately swap the card when button clicked (handlers.rs:597-661). Result: Load/Unload buttons now show immediate state change (Loading/Unloading), followed by final state (Loaded/Unloaded) after service response. Tested with Autoware simulation - all state transitions work correctly (container_control.rs:124-132; container_actor.rs:152-153, 231, 806-820, 1616-1684; handlers.rs:200-263, 597-661; index.html:41-43, 85-87, 419-422).
- **2026-01-16**: Composable node to container matching fix (Python + Rust) - **Root cause**: Python dump_launch only stored container node_name (e.g., "container") in target_container_name, but Rust coordinator built full ROS names with namespace (e.g., "/adapi/container"). Additionally, multiple containers with same name but different namespaces caused HashMap collisions. **Solution**: (1) Python fix - Changed load_composable_nodes.py to use `expanded_node_namespace()` + `node_name` to build full target_container_name matching ROS node name format. (2) Rust fix - Added deduplication for containers with duplicate names (container, container_2, container_3, etc.) using name_counts HashMap. (3) Metadata fix - Updated metadata.name to match unique member name for correct Web UI control routing. (4) Loading/Unloading state fix - Added immediate shared_state updates: Loading state when Load button clicked, Unloading state when Unload button clicked, providing instant Web UI feedback. Added Unloading to MemberState enum (web_query.rs), ComposableNodeStatus enum (web_types.rs), status conversion, and CSS class mapping (handlers.rs). Result: All 54 composable nodes successfully matched and visible in Web UI with immediate state feedback (python/play_launch/dump/visitor/load_composable_nodes.py:31-41; coordinator.rs:240-324, 909-924; web_query.rs:44; web_types.rs:47,166; handlers.rs:129).
- **2026-01-16**: Web UI bulk operations and state synchronization fixes - Implemented all bulk control operations: Start All, Stop All, Restart All buttons in Web UI header operate on all regular nodes and containers; Load All / Unload All buttons within each container card operate on composable nodes. Fixed 8+ missing shared_state updates where state transitions (Stopped, Failed, Blocked→Unloaded, Loading→Loaded/Failed) updated actor internal state but not DashMap, causing Web UI to show stale states. Fixed bulk child operations to use virtual_member_routing HashMap for correct container-to-composable matching. All operations log affected member count and work correctly across multiple restart cycles (container_actor.rs:1267-1269, 1407-1414, 1532-1586, 1726-1806; coordinator.rs:907-1018; handlers.rs:790-835).
- **2026-01-14**: Container restart race condition fix - Fixed composable nodes getting stuck in "Loading" state after container restart. **Root cause**: Race condition between ROS service registration and container executor startup. `service_is_ready()` returns true when service is registered in ROS graph, but container executor may not be spinning/processing requests yet. When service becomes ready too quickly (0ms), service calls hang forever. **Solution**: Added 200ms warmup delay after `service_is_ready()` returns true to ensure container executor is actually processing requests before calling LoadNode service (container_actor.rs:442-452). Tested with 3 consecutive restart cycles - all composable nodes loaded successfully without timeouts.
- **2026-01-14**: Logging level improvements - Changed many `info!` logs to `debug!` to reduce noise for end-users. Technical details like service client creation, state transitions (Blocked→Unloaded), service readiness checks, and internal timing are now DEBUG-level. INFO level now shows only essential user-facing events: user actions (Start/Stop commands), major lifecycle events (container started/terminated, startup complete), and errors/warnings. Use `RUST_LOG=play_launch=info` (default) for clean output or `RUST_LOG=play_launch=debug` for detailed troubleshooting. Documented logging practices in CLAUDE.md (container_actor.rs:389-440, 1039-1109).
- **2026-01-13**: Member name simplification - Removed "NODE ''" and "LOAD_NODE ''" prefixes from all member names. Names now come directly from record.json: regular nodes use `record.name`, containers use `record.name`, composable nodes use `record.node_name`. This results in cleaner log directory names (e.g., "talker-1" instead of "NODE 'talker-1'", "listener" instead of "LOAD_NODE 'listener'") and cleaner Web UI display (replay.rs:413-420, 441-449, 476; run.rs:231-237).
- **2026-01-13**: Phase 12 cleanup - Removed all deprecated composable node actor code. Deleted `composable_node_actor.rs` (41KB), removed `ComposableActorConfig` struct, simplified coordinator API to pass `auto_load` directly instead of config object. All references to deprecated code removed from mod.rs, coordinator.rs, and replay.rs. Codebase is now cleaner with Phase 12 fully complete.
- **2026-01-13**: Logging improvements and restart debugging - Adjusted log levels for composable node loading to reduce noise: changed all loading-related messages to DEBUG level ("Successfully loaded composable node", "Transitioning N blocked composable nodes", "Queueing N composable nodes for loading", "No composable nodes to load"). Startup output is now much cleaner - INFO level only shows essential progress. Added detailed state tracking in handle_load_all_composables() to help diagnose container restart issues - logs state summary (Blocked/Unloaded/Loading/Loaded/Failed counts) and which specific nodes are being queued (visible with RUST_LOG=play_launch=debug). Tested with Autoware simulation - all 49 composable nodes loaded successfully (container_actor.rs:792, 797, 1074, 1334, 757-809).
- **2026-01-13**: Phase 12: Parameter passing and type preservation fix - Fixed LoadNode service calls not passing parameters to composable nodes. Added helper functions `ros_params_to_strings()` and `parameter_value_to_string()` to convert ROS rcl_interfaces::msg::Parameter objects to string tuples. Fixed double parameter type preservation by ensuring all double values include a decimal point (e.g., "1.0" not "1") to prevent misinterpretation as integers. Updated `handle_load_composable()` to convert metadata.parameters and metadata.extra_args before creating LoadRequest. Fixes "Statically typed parameter 'X' must be initialized" and "parameter 'X' has invalid type: Wrong parameter type, parameter {X} is of type {double}, setting it to {integer} is not allowed" errors (container_actor.rs:153-189, 665-679).
- **2026-01-12**: Phase 12: Container-managed composable nodes - Merged composable node actors into container actors. Composable nodes are now virtual members managed as internal state by their parent containers. Control events (Start/Stop/Restart) are routed to containers and translated to Load/Unload operations. Log directories consolidated - composable node metadata appears in container's metadata.json, no separate load_node directories. Deprecated ComposableNodeActor module (container_actor.rs, coordinator.rs, context.rs, replay.rs).
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

### Avoiding Orphan Processes
When killing play_launch or testing with ros2 launch:
- **NEVER** use `kill -9` or `pkill -9` on individual processes - this leaves orphans
- **ALWAYS** kill the entire process group (PGID) to ensure all children are terminated
- Proper cleanup sequence:
  1. Send SIGTERM to PGID first: `kill -TERM -$PGID`
  2. Wait 2 seconds for graceful shutdown
  3. Send SIGKILL to PGID for stubborn processes: `kill -9 -$PGID`
- Example cleanup script:
  ```bash
  LAUNCH_PID=12345
  PGID=$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')
  kill -TERM -$PGID  # Kill entire process group
  sleep 2
  kill -9 -$PGID     # Force kill remaining
  ```
- **Why**: play_launch and ros2 launch spawn many child processes (containers, nodes). Killing only the parent or individual processes leaves orphans that consume resources
- Use `just kill-orphans` in test directories to clean up stray ROS processes

### Temporary Files
- **ALWAYS** store all temporary files in `tmp/` directory at the project root (`/home/aeon/repos/play_launch/tmp/`)
- This directory is gitignored and used for testing, debugging, and experimentation
- **NEVER** use system `/tmp` for project-related temporary files
- Use `$project/tmp/` for all test runs, temporary log directories, debugging logs, and scratch work
- Example: `cd /home/aeon/repos/play_launch/tmp && play_launch ...`
- Example: Redirect logs to `/home/aeon/repos/play_launch/tmp/debug.log`, not `/tmp/debug.log`

### External Dependencies
- `external/` directory is for placing 3rd party projects for study and reference
- **Gitignored** - not part of the project build

### Using ros2 launch for Testing
When running `ros2 launch` manually for debugging or comparison:
- **CRITICAL**: Always kill both the launch process AND its children to prevent orphan processes
- Method 1: Store PID and kill process group:
  ```bash
  ros2 launch <package> <launch_file> &
  LAUNCH_PID=$!
  # ... do work ...
  pkill -TERM -P $LAUNCH_PID  # Kill children with SIGTERM
  sleep 2
  pkill -9 -P $LAUNCH_PID     # Force kill stubborn children
  kill -9 $LAUNCH_PID         # Kill parent
  ```
- Method 2: Use PID file:
  ```bash
  ros2 launch <package> <launch_file> > /tmp/ros2_launch.log 2>&1 &
  echo $! > /tmp/launch_pid.txt
  # ... do work ...
  LAUNCH_PID=$(cat /tmp/launch_pid.txt)
  pkill -TERM -P $LAUNCH_PID
  sleep 2
  pkill -9 -P $LAUNCH_PID
  kill -9 $LAUNCH_PID
  ```
- **Why**: `ros2 launch` spawns many child processes (nodes, containers). Killing only the parent leaves orphans running
- This is especially important for containers and composable nodes which may not auto-terminate

### Language Server Protocol (LSP) Tools
- **pyright-lsp**: Available for Python type checking and code intelligence
- **rust-analyzer-lsp**: Available for Rust code intelligence and refactoring
- These LSP tools can be used for advanced code analysis, navigation, and refactoring tasks
- Use these tools when performing complex code transformations or when you need deep code understanding

### Building the Project
- **ALWAYS** use `just build` to rebuild the project (NEVER use `colcon build` directly)
- `just build` handles the correct build sequence and ensures proper integration
- After code changes, always run `just build` before testing

### Logging Practices

**Log Levels:**
- `error!`: Unrecoverable errors that prevent normal operation
- `warn!`: Recoverable errors or unexpected conditions that users should know about
- `info!`: High-level user-facing events (startup complete, shutdown initiated, user actions like Start/Stop/Restart)
- `debug!`: Detailed internal state changes and technical details for troubleshooting
- `trace!`: Very verbose execution details (rarely used)

**Guidelines:**
- Default `RUST_LOG=play_launch=info` should show only essential user-facing information
- Use `debug!` for technical details like:
  - Service client creation
  - State transitions (Blocked→Unloaded, Loading→Loaded)
  - Service readiness checks
  - Internal timing measurements
  - Queue depths and processing details
- Use `info!` only for:
  - User-initiated actions (received Stop/Start commands)
  - Major lifecycle events (container started/terminated, startup complete)
  - Critical state changes users need to monitor
  - Errors and warnings
- Examples:
  ```rust
  // GOOD: User-facing action
  info!("{}: Received Stop command, killing container (PID: {})", name, pid);
  info!("{}: Container process terminated", name);

  // GOOD: Technical details at debug level
  debug!("{}: Creating LoadNode service client for {}", name, service);
  debug!("{}: Transitioning {} blocked nodes to Unloaded", name, count);
  debug!("{}: LoadNode service available after {}ms", name, elapsed);
  ```

**Enabling Debug Logs:**
```sh
# Show all debug logs
RUST_LOG=play_launch=debug play_launch replay

# Show debug logs for specific modules
RUST_LOG=play_launch::member_actor=debug play_launch replay

# Show debug logs with web server verbose output
RUST_LOG=play_launch=debug,play_launch::web=trace play_launch replay
```

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

### Current Status (2026-01-14)

**Recent Work Completed:**
- ✅ Container restart race condition fixed (200ms warmup delay)
- ✅ Logging levels improved (info→debug for technical details)
- ✅ Logging practices documented in CLAUDE.md
- ✅ All changes tested and verified working
- ✅ Build successful: `dist/play_launch-0.4.0-py3-none-any.whl`

**Key Files Modified:**
- `src/play_launch/src/member_actor/container_actor.rs`: Lines 389-452, 1039-1109
- `CLAUDE.md`: Added logging practices section and updated key recent changes

**Testing:**
- Container restart bug: 3 consecutive restart cycles, all successful
- INFO logging: Clean, user-facing output only
- DEBUG logging: Full technical details for troubleshooting

(No known issues at this time)
