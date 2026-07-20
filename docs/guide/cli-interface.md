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
# Dump launch execution without replaying (dump-level options go BEFORE
# the `launch`/`run` subcommand — they belong to `dump`, not to the
# launch-argument list, which greedily captures everything after it)
play_launch dump [dump_options...] launch <package_name> <launch_file> [key:=value...]
play_launch dump [dump_options...] launch <launch_file_path> [key:=value...]

# Dump node execution
play_launch dump [dump_options...] run <package_name> <executable> [args...]
```

**Dump-Specific Options:**
- `--output <file>` or `-o <file>`: Output file (default: `system_model.yaml`
  for `dump launch`'s default `--format model`; `record.json` for
  `--format record` and for `dump run`, which has no launch scope tree to
  build a model from)
- `--format <model|record>`: Dump format (default `model`, the SystemModel —
  the one user-facing dump). `record` emits the legacy `record.json` — kept
  as a deprecated dev/parser-parity escape hatch (`scripts/compare_records.py`,
  `just compare-dumps`), not the user-facing default (Phase 46.5)
- `--debug`: Enable debug output during dump

**Examples:**
```bash
# Dump to a custom SystemModel file (default format)
play_launch dump --output autoware.yaml launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning

# Dump the legacy record.json (deprecated; parser-parity tooling only)
play_launch dump --format record --output autoware_dump.json \
    launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning

# Dump with debug output
play_launch dump --debug launch demo_nodes_cpp talker_listener.launch.py
```

### Replay Only

```bash
# Replay from a SystemModel (primary path, Phase 46)
play_launch replay --model <system_model.yaml> [options...]

# Replay from a legacy record.json without --model (deprecated, warns,
# spawns the same as before — Phase 46.5 compat)
play_launch replay --input-file <record.json> [options...]
```

**Examples:**
```bash
# Replay with monitoring config (monitoring is enabled by default; see --disable-monitoring)
play_launch replay --model autoware.yaml --config myconfig.yaml --monitor-interval-ms 500

# Replay with a custom model + log dir
play_launch replay --model autoware.yaml --log-dir logs/autoware_run1
```

## Common Options

These options apply to `launch`, `run`, and `replay` subcommands:

### Output Configuration
- `--log-dir <path>`: Log directory for execution outputs (default: `play_log`)
- `--model <path>`: SystemModel to replay from (primary source since Phase
  46 — spawns from `structure.nodes`, no companion record required)
- `--input-file <path>`: Legacy record.json to replay when `--model` is
  absent (default: `record.json`). Deprecated (Phase 46.5): prints a
  one-time warning and still spawns, kept for one release's rollback grace

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

### Automatic Dump-Resolve-Replay (`launch` and `run`)

When you use `play_launch launch` (Phase 43/46 — the SystemModel path is
the default):

1. **Dump phase**: records launch execution to a `record.json` internally
   (nodes, composable nodes, containers, parameters, remappings) — this is
   an internal working file for the `launch` flow, not the artifact you're
   expected to keep or hand-edit.

2. **Resolve phase**: builds `system_model.yaml` from that record + contract
   manifests + scheduling platform file (the same pipeline as
   `play_launch resolve`)
   - Contract **errors refuse the launch** — the model is checked by
     construction; warnings embed in the model
   - Since Phase 46.5 there is no cryptographic model↔record binding —
     the model is a complete, self-sufficient artifact on its own

3. **Replay phase**: executes the launch **from the SystemModel**
   - Spawns nodes and containers from `structure.nodes`; loads composable
     nodes via RCL services
   - The rule engine, blocking allowlist, and lifecycle wiring read the
     model's contracts; the scheduling plan reads the model's execution
     layer (no re-derivation at spawn time)
   - Monitors resources if enabled; saves all outputs to the log directory

`play_launch run` (single node) skips the resolve phase — it has no
contracts to check.

### The three-verb split

| Verb | Role |
|---|---|
| `check` | diagnostic front-end — full source excerpts, `--explain`, rule filters |
| `resolve` / `dump` | the build step — both emit the same checked `system_model.yaml` (the one user-facing artifact, Phase 46.5) |
| `replay --model` | the runtime — spawns and applies contracts/scheduling from the model, no companion record required |

`replay --input-file <record.json>` without `--model` remains a **deprecated**
legacy path (re-loads manifests and re-derives the sched plan from the
platform file at replay time) — it still spawns, prints a one-time
deprecation warning, and is kept for one release's rollback grace. There is
no model↔record binding to go stale: since 46.5 the two are independent
artifacts, not a checked pair.

### Manual Workflow

For more control, you can separate the dump and replay steps:

Dump-level options (`--output`, `--format`) go BEFORE the `launch`/`run`
subcommand — they belong to `dump`, not to the trailing launch-argument
list, which greedily captures every token after it (including ones that
look like flags):

```bash
# Step 1: Dump the SystemModel directly (default format — no record.json involved)
play_launch dump --output autoware.yaml launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning

# Step 2: Replay from the model (monitoring is on by default; container
# readiness settings like wait_for_service_ready live in --config, not CLI flags)
play_launch replay \
    --model autoware.yaml \
    --log-dir logs/run1
```

Equivalently, `resolve` builds the same model straight from the launch file
(skipping a separate dump step) and can add a scheduling platform file. Like
`dump`, put `resolve`'s own flags (`-o`, `--sched`, `--contracts`, ...)
before any `KEY:=VALUE` launch argument, for the same reason:

```bash
play_launch resolve -o autoware.yaml --sched system.posix.yaml \
    autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning
play_launch replay --model autoware.yaml
```

**Legacy record.json path (deprecated, dev/parser-parity tooling only):**

```bash
# Dump the legacy record.json
play_launch dump --format record --output autoware_planning.json \
    launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning

# Resolve reuses an existing record.json instead of re-parsing the launch file
play_launch resolve --record autoware_planning.json \
    --sched system.posix.yaml -o autoware.yaml

# Replay from the legacy record without --model (warns, still spawns)
play_launch replay --input-file autoware_planning.json --log-dir logs/run1
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

