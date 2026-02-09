# Autoware Planning Simulator Test

This directory contains test recipes for `play_launch` with Autoware planning simulator.

## Setup

1. **Configure `activate_autoware.sh`** to source your Autoware workspace:
   ```bash
   # Edit activate_autoware.sh and uncomment/modify the source line:
   source ~/autoware/install/setup.bash
   ```

2. **Ensure you have the required map data**:
   ```bash
   # The script expects the sample map at:
   $HOME/autoware_map/sample-map-planning
   ```

## Directory Structure

```
test/autoware/
├── justfile              # Main test automation
├── README.md             # This file
├── activate_autoware.sh  # Source this to enter Autoware env (edit to point to your install)
├── cyclonedds.xml        # CycloneDDS configuration
├── poses_config.yaml     # Test poses configuration
├── record.json           # Launch execution record (generated)
├── play_log/             # Execution logs and metrics (generated)
└── scripts/
    ├── test_autonomous_drive.py    # Autonomous driving test
    ├── plot_resource_usage.py      # Plot generation tool
    └── kill_orphan_nodes.sh        # Cleanup utility
```

## Usage

```bash
# Show available recipes
just --list

# Run simulator (foreground, Ctrl-C to stop)
just run-sim

# Run demo (simulator + autonomous test, Ctrl-C to stop)
just run-demo

# Dump and compare parser outputs
just dump-rust
just dump-python
just dump-both
just compare-dumps

# Clean up orphan ROS nodes
just kill-orphans
```

The justfile automatically sources the Autoware environment from `activate_autoware.sh`.

### Launch Methods

**`just run-sim`** - Foreground simulator:
- Web UI at http://localhost:8080
- Resource monitoring and diagnostics enabled by default
- Ctrl-C to stop

**`just run-demo`** - Simulator + autonomous test:
- Starts simulator in background
- Waits 60s for initialization
- Runs autonomous driving test
- Keeps simulator running after test (Ctrl-C to stop)

## Autonomous Driving Test

### Quick Start

```bash
just run-demo
```

This will:
1. Start Autoware with `play_launch launch`
2. Wait 60s for system initialization
3. Set initial pose
4. Set goal pose
5. Engage autonomous mode
6. Monitor for 60 seconds

### Manual Test (Step-by-step)

1. Start Autoware planning simulator:
   ```bash
   just run-sim
   ```

2. Wait ~60 seconds for system to initialize

3. In another terminal, run the autonomous driving test:
   ```bash
   python3 scripts/test_autonomous_drive.py
   ```

### Test Sequence Details

The autonomous driving test performs:

1. **Wait for Autoware** - Checks that critical topics are active:
   - `/map/vector_map`
   - `/api/operation_mode/state`
   - `/api/routing/state`

2. **Set Initial Pose** - Publishes to `/initialpose` topic to initialize localization

3. **Wait for Localization** - Waits 5s for localization to stabilize and checks:
   - `/tf` transforms
   - `/localization/kinematic_state`

4. **Set Route** - Calls `/api/routing/set_route_points` service

5. **Engage Autonomous** - Calls `/api/operation_mode/change_to_autonomous` service

6. **Monitor Progress** - Monitors vehicle state for specified duration

### Pose Configuration

Poses are loaded from `poses_config.yaml` in this directory.
These poses are validated for `sample-map-planning`.

## Troubleshooting

### Error: activate_autoware.sh not found
Ensure the file exists in the `test/autoware/` directory.

### Error: activate_autoware.sh has no active 'source' line
Edit `activate_autoware.sh` and uncomment/modify the source line to point to your Autoware install:
```bash
source ~/autoware/install/setup.bash
```

## Configuration

### justfile Variables

```bash
MAP_PATH=/path/to/map just run-sim
```

Available variables:
- `MAP_PATH` - Path to map data (default: `$HOME/autoware_map/sample-map-planning`)

### DDS Configuration

All recipes use `cyclonedds.xml` which configures:
- Localhost-only communication (loopback interface)
- 65.5KB max message size
- 10MB socket buffer size

The `CYCLONEDDS_URI` environment variable is set before launching to ensure all nodes use this configuration.
