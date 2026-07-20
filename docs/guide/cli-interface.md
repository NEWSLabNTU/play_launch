# play_launch CLI Interface Specification

## Overview

`play_launch` provides a unified command-line interface that mimics ROS 2 standard commands while enabling launch inspection, recording, and replay capabilities. The tool automatically dumps (parses) the launch execution into a SystemModel and then replays it using the optimized Rust runtime.

## Command Syntax

### Launch Files

```bash
# Launch from a ROS package
play_launch launch <package_name> <launch_file> [key:=value...] [options...]

# Launch from a file path
play_launch launch <launch_file_path> [key:=value...] [options...]
```

Flags may be placed before or after the `KEY:=VALUE` launch arguments — clap
parses flags in any position (47.A1). Use `--` to force any remaining token
to be treated as a positional launch argument instead.

**Examples:**
```bash
# Launch Autoware planning simulator
play_launch launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning

# Launch from absolute path
play_launch launch /path/to/my_launch.py use_sim_time:=true

# Launch with a custom monitoring interval (monitoring is on by default;
# see --disable-monitoring to turn it off). The flag works just as well
# after the launch arguments as before them.
play_launch launch demo_nodes_cpp topics/talker_listener.launch.py \
    --monitor-interval-ms 500
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
# `dump`'s own flags (--output/-o, --format, --debug) may be given anywhere
# on the command line — before `launch`/`run`, or after, or interleaved
# with the KEY:=VALUE launch arguments (47.A1: they're declared `global`,
# so clap recognizes them at any position in the nested subcommand).
play_launch dump [dump_options...] launch <package_name> <launch_file> [key:=value...] [dump_options...]
play_launch dump [dump_options...] launch <launch_file_path> [key:=value...] [dump_options...]

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
# Dump to a custom SystemModel file (default format) — flags before the
# KEY:=VALUE launch arguments
play_launch dump --output autoware.yaml launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning

# Equivalently, flags after the launch arguments — both orders work
play_launch dump launch autoware_launch planning_simulator.launch.xml \
    vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit \
    map_path:=$HOME/autoware_map/sample-map-planning \
    --output autoware.yaml

# Dump the legacy record.json (deprecated; parser-parity tooling only)
play_launch dump --format record --output autoware_dump.json \
    launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning

# Dump with debug output
play_launch dump --debug launch demo_nodes_cpp topics/talker_listener.launch.py
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
- Monitoring, diagnostics, and the web UI are all **on by default** — there
  is no `--enable-monitoring` flag. Turn features off individually
  (`--disable-monitoring`, `--disable-diagnostics`, `--disable-web-ui`,
  `--disable-all`), or flip to an allow-list with `--enable <FEATURE>`
  (repeatable; values: `monitoring`, `diagnostics`, `web-ui` — when `--enable`
  is used at all, only the listed features are on).
- `--monitor-interval-ms <ms>`: Sampling interval in milliseconds

### Composable Node Loading
These are **not CLI flags** — they're fields of the `composable_node_loading`
section in the `--config`/`-c` YAML file (`ComposableNodeLoadingSettings` in
`src/play_launch/src/cli/config.rs`):
- `delay_load_node_millis` (default: 2000): Delay before loading composable nodes
- `load_node_timeout_millis` (default: 30000): Timeout for loading each composable node
- `load_node_attempts` (default: 3): Max retry attempts for loading
- `max_concurrent_load_node_spawn` (default: 10): Concurrent loading limit

The following two ARE CLI flags (shared with the config file's semantics):
- `--standalone-composable-nodes`: Run composable nodes in standalone mode
- `--load-orphan-composable-nodes`: Load composable nodes without matching containers

### Container Readiness
Also **not CLI flags** — fields of the `container_readiness` section in the
`--config` YAML (`ContainerReadinessSettings`), enabled by default:
- `wait_for_service_ready` (default: true): Wait for container services via ROS service discovery
- `service_ready_timeout_secs` (default: 120, 0=unlimited): Max wait time for container services
- `service_poll_interval_ms` (default: 500): Polling interval for service discovery

See `tests/fixtures/autoware/autoware_config.yaml` for a full config YAML example.

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

`dump`'s own flags (`--output`, `--format`) and `resolve`'s own flags (`-o`,
`--sched`, `--contracts`, ...) may be given before or after the `launch`/`run`
subcommand and the `KEY:=VALUE` launch arguments — clap parses flags in any
position (47.A1). Putting them first, as below, is just a style choice:

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
(skipping a separate dump step) and can add a scheduling platform file:

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

`play_launch` must be available in PATH (there is no separate `dump_launch`
binary — Python-parser dumping is embedded in `play_launch` itself via PyO3,
selected with `--parser python`; see `docs/guide/parser-migration.md`):

```bash
# Build the workspace
cd /path/to/play_launch
just build

# Source the workspace
source install/setup.bash

# Verify the binary is available
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
play_launch launch demo_nodes_cpp topics/talker_listener.launch.py \
    --config config.yaml
```

### Container Service Readiness

For large launches like Autoware, container-readiness and composable-node-loading
timing are config-file settings, not CLI flags — service readiness checking is
already enabled by default. Tune it via `--config`:

```bash
# tuned_config.yaml
container_readiness:
  wait_for_service_ready: true
  service_ready_timeout_secs: 300
composable_node_loading:
  load_node_timeout_millis: 60000
```

```bash
play_launch launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning \
    --config tuned_config.yaml
```

## Troubleshooting

### play_launch not found

Ensure the workspace is sourced:
```bash
source install/setup.bash
which play_launch
```

### Composable nodes fail to load

1. Service readiness checking is on by default; if it's been disabled in a
   `--config` file, re-enable `container_readiness.wait_for_service_ready`
2. Increase `composable_node_loading.load_node_timeout_millis` in `--config`
   (default: 30000)
3. Check container logs in `play_log/node/` directories

### Orphan composable nodes warning

If you see warnings about orphan composable nodes:
- Use `--load-orphan-composable-nodes` to attempt loading them anyway
- Check that container names in launch files match actual container node names
- Review namespace resolution (relative vs absolute names)

