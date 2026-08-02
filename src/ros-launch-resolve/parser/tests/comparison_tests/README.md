# Comparison Tests

This directory contains test scripts and launch files for comparing Rust and Python parser outputs.

## Files

### Comparison Script

**`compare_rust_python.py`** - Main comparison tool

Comprehensive comparison script that:
- Runs both Rust and Python parsers on the same launch file
- Compares all record types (nodes, containers, composable nodes)
- Provides color-coded terminal output with detailed diffs
- Saves JSON outputs for manual inspection

Usage:
```bash
# From this directory
python3 compare_rust_python.py test_simple.launch.xml

# From project root
python3 tests/comparison_tests/compare_rust_python.py tests/comparison_tests/test_simple.launch.xml

# With autoware symlink
cd tests/autoware_test/scripts
python3 compare_rust_python.py autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml
```

### Test Launch Files

**`test_simple.launch.xml`** - Basic comparison test

Tests:
- Regular node with namespace
- Node container with namespace scoping
- Composable nodes inside container
- Group with push-ros-namespace
- Namespace inheritance

**`test_group_namespace.launch.xml`** - Namespace scoping test

Tests:
- Nested groups with push-ros-namespace
- Container namespace inheritance from context
- Accumulated namespace paths

## Running Tests

### Simple Test

```bash
cd /path/to/play_launch_parser
python3 tests/comparison_tests/compare_rust_python.py tests/comparison_tests/test_simple.launch.xml
```

Expected output:
```
✓ ALL CHECKS PASSED
Rust and Python implementations produce identical outputs!
```

### Verify Namespace Scoping

```bash
python3 tests/comparison_tests/compare_rust_python.py tests/comparison_tests/test_group_namespace.launch.xml
```

This verifies that containers properly inherit namespaces from GroupActions and push-ros-namespace.

## Output Files

After running, the script creates:
- `rust_output.json` - Full Rust parser output
- `python_output.json` - Full Python dump_launch output

These are saved in the same directory as the script.

## Comparison Categories

The script compares:

1. **Counts** - Total nodes, containers, composable nodes, lifecycle nodes
2. **Containers** - Names and namespaces
3. **Composable Nodes** - Grouped by target container
4. **Regular Nodes** - With Python's container duplication filtered out

## Requirements

- Rust parser built: `cargo build --release --features python`
- Python dump_launch module available (searches in ../play_launch/python)
- ROS 2 Humble sourced
- For Autoware tests: Autoware workspace symlink at `../autoware_test/autoware`

## Path Resolution

The script automatically discovers:
- Project root (relative to script location)
- Rust binary location
- Autoware workspace (via symlink if available)
- play_launch module (checks common locations)

No absolute paths are hardcoded (except for /opt/ros/humble and /tmp).

## Test Results

As of 2026-01-20 with namespace scoping fix:

### test_simple.launch.xml
- ✅ Containers: 1 match
- ✅ Container namespace: `/planning` (correct inheritance from group)
- ✅ Composable nodes: 2 matches
- ✅ Target container name: `/planning/planning_container` (full path)

### test_group_namespace.launch.xml
- ✅ Container namespace: `/planning/scenario_planning` (nested)
- ✅ Target container name: `/planning/scenario_planning/test_container`

Both implementations produce identical outputs with proper namespace scoping.
