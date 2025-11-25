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

# Install from wheel (when available)
pip install play_launch

# Or install from local wheel after building
pip install target/wheels/play_launch-*.whl

# Optional: Enable privileged I/O monitoring
sudo setcap cap_sys_ptrace+ep $(which play_launch_io_helper)
```

### Option 2: Build from Source (For Development)

```sh
# Install dependencies (first-time setup)
just install-deps        # Install colcon-cargo-ros2, maturin, run rosdep

# Build workspace (single-stage colcon build with colcon-cargo-ros2)
just build

# Enable I/O monitoring for privileged processes
just setcap-io-helper    # Requires sudo, reapply after rebuild

# Verify I/O helper status
just verify-io-helper

# Source workspace (when running commands outside of just)
. install/setup.bash
```

### Building Wheel for Distribution

```sh
# Build with colcon first
just build

# Build wheel (copies binaries to python package)
just build-wheel

# Wheel will be in dist/play_launch-*.whl
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

## Configuration

### CLI Flags

- `--config <PATH>` (`-c`): Runtime config YAML
- `--verbose` (`-v`): Show per-node progress (default: summary only)
- `--enable-monitoring`: Enable resource monitoring
- `--monitor-interval-ms <MS>`: Sampling interval (default: 1000ms)
- `--standalone-composable-nodes`: Run composable nodes standalone
- `--load-orphan-composable-nodes`: Load nodes without matching containers
- `--disable-respawn`: Disable automatic respawn even if configured in launch file

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
play_log/YYYY-MM-DD_HH-MM-SS/
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
- Flat 1-level structure for easy scripting
- Short directory names (e.g., `control_evaluator`) with deduplication (`_2`, `_3`)
- Each node directory is self-contained with all data

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

## Dependencies

**Rust**: tokio, rayon, eyre, clap, serde/serde_json/serde_yaml, bincode, rclrs, composition_interfaces, rcl_interfaces, sysinfo, nvml-wrapper, csv

**Python**: pyyaml (standard), lark, packaging, hatchling, pytest, ruff

## Testing

Autoware planning simulator integration test in `test/autoware_planning_simulation/`:
- `just start-sim`: Start simulator with play_launch
- `just drive`: Run autonomous driving test
- `just plot`: Generate resource plots
- Tested with 52 composable nodes, 15 containers

## Distribution & Packaging

### Debian Package

Build Debian package for Ubuntu 22.04:

```bash
just build-deb  # Creates dist/play-launch_0.2.0-3_amd64.deb
```

Packages are built to `dist/` directory.

**Package details:**
- **Naming convention**: Package name uses `play-launch` (Debian requirement), installed files use `play_launch` (source code consistency)
- Built with standard Debian tools (`dpkg-buildpackage`)
- Uses single-stage `colcon build` with colcon-cargo-ros2
- Uses `debian/` directory for metadata (control, changelog, rules, postinst)
- Installs to dual locations for FHS compliance and ROS2 integration:
  - `/usr/bin/play_launch`, `dump_launch` - main executables (in PATH)
  - `/usr/lib/play_launch/play_launch_io_helper` - I/O monitoring helper
  - `/opt/ros/humble/*` - Full ROS2 package structure with symlinks to /usr binaries
  - `/usr/share/doc/play_launch/` - Documentation (README.md, CLAUDE.md)
  - `/usr/share/play_launch/examples/` - Test cases
- Postinst script automatically sets CAP_SYS_PTRACE on I/O helper
- Size: ~2.0 MB compressed (5.4 MB play_launch + 1.7 MB play_launch_io_helper + Python packages)

**Build process:**
- `override_dh_auto_configure`: Install Rust (if needed), colcon-cargo-ros2, Python dependencies
- `override_dh_auto_build`: Single `colcon build` command (no --symlink-install for proper packaging)
- `override_dh_auto_install`: Copy install/* to /opt/ros/humble, move binaries to /usr, create symlinks
- Build artifacts are moved to `dist/` directory after successful build

**Binary optimization** (completed 2025-11-03):
- Release profile with `strip = true` and `lto = "thin"`
- 94% size reduction: 137MB → 8.7MB (before packaging)
- See `Cargo.toml` [profile.release] section

### Build System

- **justfile**: Task runner (replaced Makefile 2025-11-04)
- Use `just --list` to see all available recipes
- Main recipes:
  - `install-deps`: Install colcon-cargo-ros2, run rosdep (interactive prompts for conflicts)
  - `build`: Single-stage colcon build (sources /opt/ros/humble/setup.bash automatically)
  - `build-wheel`: Build Python wheel (copies binaries, outputs to dist/)
  - `run *ARGS`: Run play_launch with arguments (e.g., `just run launch demo_nodes_cpp talker_listener.launch.py`)
  - `setcap-io-helper`: Apply CAP_SYS_PTRACE to I/O helper (requires sudo)
  - `verify-io-helper`: Verify I/O helper capability status
  - `clean`: Remove build/install/log/dist directories and copied binaries
  - `build-deb`: Build Debian package (outputs to dist/)
  - `test-wheel`: Test wheel installation in fresh venv
  - `publish-pypi`: Publish wheel to PyPI (requires PYPI_TOKEN env var)
  - `publish-testpypi`: Publish wheel to TestPyPI (requires TESTPYPI_TOKEN env var)
  - `test`: Run package tests
  - `lint`: Run clippy and ruff
  - `format`: Format Rust and Python code

**debian/justfile**: Debian packaging recipes
- `build`: Build Debian package to dist/ directory
- `clean`: Remove dist/ and build artifacts

### GitHub Workflows

**CI Workflow** (`.github/workflows/ci.yml`):
- Triggers on push to main and pull requests
- Runs on Ubuntu 22.04 with ROS2 Humble
- Executes: `just build`, `just test`, `just lint`
- Verifies code quality and functionality

**Release Workflow** (`.github/workflows/release.yml`):
- Triggers on version tags (`v*`, e.g., `v0.2.0`)
- Builds Debian packages for both amd64 and arm64 architectures
- Uses cross-compilation with QEMU emulation:
  - **amd64**: Native build on `ubuntu-22.04` (~3min)
  - **arm64**: Cross-compilation with QEMU on `ubuntu-22.04` (~20-30min)
- Works on GitHub free tier (no special runners required)
- Creates GitHub release with both .deb packages as downloadable assets
- Release packages follow naming: `play-launch_<version>_<arch>.deb`

**Build process**:
1. Checkout code
2. Set up QEMU for ARM64 emulation (arm64 only)
3. Install ROS2 Humble and build dependencies
4. Cross-compile with `dpkg-buildpackage -aarm64` (arm64 only)
5. Upload artifacts
6. Create GitHub release with both packages

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
- Uploads wheels as GitHub release assets
- Note: Wheels are platform-specific due to embedded Rust binaries

**PyPI Publishing** (manual via justfile):
- `just publish-testpypi`: Publish to TestPyPI (requires TESTPYPI_TOKEN env var)
- `just publish-pypi`: Publish to PyPI (requires PYPI_TOKEN env var)
- Obtain tokens from https://pypi.org/manage/account/token/

## Key Recent Fixes

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
