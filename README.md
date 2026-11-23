# play_launch

Resolve, run, and analyze ROS 2 launch executions with resource monitoring and interactive management.

[![Watch the demo](assets/demo.png)](assets/demo.mp4)

Upgrading? [CHANGELOG.md](CHANGELOG.md) lists the breaking changes — 0.9.0 has
several, and a few of them change behaviour rather than erroring.

## Installation

Requires Ubuntu 22.04+ with ROS 2 (Humble, Jazzy, Kilted, Lyrical or Rolling), Python 3.10+, and:

```bash
sudo apt install libz3-dev
sudo apt install ros-${ROS_DISTRO}-rclcpp-components ros-${ROS_DISTRO}-class-loader
```

Install from PyPI:

```bash
pip install play_launch
```

One wheel serves every supported ROS distribution that shares your Python
(Humble on 3.10; Jazzy and Kilted on 3.12; Lyrical and Rolling on 3.14), for
x86_64 and aarch64. It uses the distribution you sourced (`ROS_DISTRO`), or
the only one under `/opt/ros` when nothing is sourced;
`PLAY_LAUNCH_ROS_DISTRO` overrides both, and `python3 -m play_launch.payload`
says which one it picked and why. On Ubuntu 24.04 and later add
`--break-system-packages` (or use a virtual environment with
`--system-site-packages`, so ROS's Python packages stay visible).

This installs the `play_launch` command, which carries the whole surface:
`launch`, `run`, `up`, `resolve`, `dump`, `check`, `plot`, `contract`,
`context`, `setcap`, `verify`, `measure`.

Optional: grant the helper binaries the capabilities they need for per-process I/O
monitoring and non-root RT scheduling (requires sudo):

```bash
play_launch setcap
```

> This enables per-process I/O monitoring **and** non-root RT scheduling (`--sched`):
> `play_launch setcap` grants `CAP_SYS_PTRACE` to the I/O helper and `CAP_SYS_NICE` to
> the RT helper — both are small ROS-free binaries. The main `play_launch` binary is
> never given a file capability — that would put it in secure-execution mode and stop
> it finding its ROS libraries.
>
> See the [Real-Time Scheduling Guide](docs/guide/rt-scheduling.md) for the scheduling
> spec format and how it maps onto processes, threads, and the kernel.

## Quick Start

Launch any ROS 2 package with monitoring and Web UI enabled by default:

```bash
play_launch launch demo_nodes_cpp topics/talker_listener.launch.py
```

Access Web UI at `http://127.0.0.1:8080` for real-time node management and log streaming.

The Rust parser is used by default for speed. For maximum compatibility, use `--parser python`.

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

Resolve once, bring the system up as often as you like:

```bash
# Resolve once — writes system_model.yaml
play_launch dump launch <package> <launch_file> [arguments...]

# Bring it up, as often as you like
play_launch up system_model.yaml
```

`up` takes the model path positionally or as `--model <path>`; it is required
either way, since the model is the only thing `up` spawns from.

### Checking Contracts

`check` parses a launch file and checks the contracts shipped beside it
(`<stem>.contract.yaml`). It needs no ROS graph and is meant for CI:

```bash
play_launch check my_pkg bringup.launch.xml               # 0 clean, 1 errors, 3 a contract refused
play_launch check bringup.launch.xml --expect degradation-level-time   # a negative test
play_launch check bringup.launch.xml --explain --width 100         # the fault-reaction table, wrapped
play_launch --version   # play_launch 0.13.0 (v0.13.0-9-gabc1234, rlm v0.1.47)
```

Exit status 3 means a contract file did not parse (`error[manifest-parse]`),
so it was not checked at all; the refusal names the checker's version and the
grammar (rlm tag) it reads. Through a pipe the output is ASCII without colour
(`NO_COLOR` is honoured on a terminal too). Exit codes, `--expect`, the JSON
report and the `--explain` table: [Checking contracts in CI](docs/guide/check-in-ci.md).

## Features

All features enabled by default:
- **Resource monitoring**: CPU, memory, I/O, GPU (2s interval)
- **Diagnostic monitoring**: the `/diagnostics` topic (add more via `diagnostics.topics`)
- **Web UI**: Interactive management at `http://127.0.0.1:8080`
- **Observable containers**: Composable nodes share their container's process, as under `ros2 launch`, with `ComponentEvent` load reporting (default; `--container-mode isolated` fork+execs each composable instead)

### Disable Features

Disable specific features:

```bash
play_launch launch <package> <launch_file> --disable-monitoring
play_launch launch <package> <launch_file> --disable-diagnostics
play_launch launch <package> <launch_file> --disable-web-ui
play_launch launch <package> <launch_file> --disable-all
```

### Container Mode

Control how composable nodes are managed (default: `observable`):

```bash
# Observable: ComponentEvent publishing, shared process -- as `ros2 launch` runs it (default)
play_launch launch <pkg> <file> --container-mode observable

# Isolated: fork+exec per-node process isolation (a crash takes down one node)
play_launch launch <pkg> <file> --container-mode isolated

# Stock: use original container from launch file, no override
play_launch launch <pkg> <file> --container-mode stock
```

### Adjust Monitoring

Change sampling interval (default: 2000ms):

```bash
play_launch launch <package> <launch_file> --monitor-interval-ms 500
```

### Configure Web UI

Change address or port (default: `127.0.0.1:8080`):

```bash
play_launch launch <package> <launch_file> --web-addr 0.0.0.0:8080 --web-token "$(openssl rand -hex 16)"
```

The UI can stop and start nodes, so on an address other hosts can reach,
give it a token (`--web-token`, or `$PLAY_LAUNCH_WEB_TOKEN`) and open it
once as `http://<host>:8080/?token=<token>`; the browser keeps it in a
cookie from then on. Without a token, anyone who can reach the port can
control the launch. On any address, the UI accepts changes only from its
own page, so another website open in the same browser cannot send them.

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
play_launch launch <package> <launch_file> --config config.yaml
play_launch up system_model.yaml --config config.yaml
```

## Visualization

Generate interactive plots from monitoring data:

```bash
# Plot latest execution
play_launch plot

# Plot specific log directory
play_launch plot --log-dir play_log/2025-10-28_16-17-56

# Plot specific metrics
play_launch plot --metrics cpu --metrics memory

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
- **Real-time logs**: Stream stdout/stderr with log level coloring and filtering
- **Diagnostics panel**: View `/diagnostics` messages with level filtering
- **Status monitoring**: Color-coded node states
- **Auto-restart**: Per-node automatic restart configuration
- **Search & filter**: Find nodes in large deployments

## Output Structure

```
play_log/<timestamp>/
├── node/<node_name>/
│   ├── metadata.json
│   ├── metrics.csv       # Resource metrics (when enabled)
│   ├── out/err           # Process logs (the live process)
│   ├── pid/status/cmdline
│   ├── out.N/err.N       # Earlier processes of a respawned member, N = 1, 2, ...
│   ├── pid.N/status.N    #   in start order (the plain names are the newest)
│   └── params_files/     # ROS parameter files
├── load_node/<name>/
│   └── out/err           # Per-composable-node logs (isolated mode)
├── system_stats.csv      # System-wide metrics
├── diagnostics.csv       # Diagnostic level changes (see below)
└── plot/                 # Generated visualizations
```

### Disk usage

A run's largest files are the processes' own `out`/`err`, which grow as fast
as the nodes print. `diagnostics.csv` records one row when a diagnostic first
appears and each time its level changes. `diagnostics.record: all` records
every status with a row per key/value, which on Autoware is about 0.5 GB an
hour; `off` writes no file. The web UI shows live diagnostics either way.

Earlier runs are kept until you remove them. When the log directory holds
more than 5 GB, each run says so at startup (`play_log.remind_mb` sets the
threshold; 0 turns it off). To see and remove old runs:

```bash
play_launch log list                                    # each run's size, by kind of file
play_launch log prune --before 7d                       # show what it would remove
play_launch log prune --before 7d --yes                 # remove runs older than a week
play_launch log prune --keep 20 --yes                   # keep the newest 20 runs
play_launch log prune --before 2026-11-01 --class diagnostics --class events --yes
```

`--before` takes a date, a run name or an age (`12h`, `7d`, `2w`). With
`--class`, only those kinds of file are removed and the runs stay: `diagnostics`,
`metrics`, `events`, `interception`, `node-logs`, `launcher-log` and `params`.
Nothing is removed without `--yes`.

To have play_launch remove the oldest runs at the start of each run, set
either limit in `--config`:

```yaml
play_log:
  keep_runs: 20          # at most 20 runs, the new one included
  max_total_mb: 10000    # earlier runs under 10 GB in total
```

A run that a play_launch process is still writing is never removed.

## Command Reference

```bash
# Launch (all features enabled by default)
play_launch launch <package> <launch_file> [args...]
play_launch run <package> <executable> [args...]

# Dump and up (dump emits the SystemModel by default — one artifact)
play_launch dump launch <package> <launch_file> [args...]
play_launch up system_model.yaml            # or: play_launch up --model system_model.yaml

# Parser selection (Rust is default)
play_launch launch <pkg> <file> --parser rust     # Default, fast
play_launch launch <pkg> <file> --parser python   # Maximum compatibility

# Old runs
play_launch log list
play_launch log prune --before 7d --yes

# Container mode
play_launch launch <pkg> <file> --container-mode observable  # Default
play_launch launch <pkg> <file> --container-mode isolated
play_launch launch <pkg> <file> --container-mode stock

# Disable features
play_launch launch <pkg> <file> --disable-monitoring
play_launch launch <pkg> <file> --disable-diagnostics
play_launch launch <pkg> <file> --disable-web-ui
play_launch launch <pkg> <file> --disable-all
play_launch launch <pkg> <file> --disable-respawn

# Reload a composable node whose process crashed (default: off)
play_launch launch <pkg> <file> --composable-respawn on-crash  # always
play_launch launch <pkg> <file> --composable-respawn inherit   # only under a respawn="true" container

# Enable only specific features
play_launch launch <pkg> <file> --enable monitoring
play_launch launch <pkg> <file> --enable web-ui --enable diagnostics

# Adjust settings
play_launch launch <pkg> <file> --monitor-interval-ms 500
play_launch launch <pkg> <file> --web-addr 0.0.0.0:8080 --web-token <token>
play_launch launch <pkg> <file> --config config.yaml

# Logging
play_launch launch <pkg> <file> --verbose              # Enable INFO level
RUST_LOG=play_launch=debug play_launch launch <pkg> <file>  # DEBUG level

# Contracts
play_launch check <pkg> <file> [args...]           # exit 0 / 1 errors / 3 refused
play_launch check <pkg> <file> --expect <rule-id>  # pass only on exactly these errors
play_launch check <pkg> <file> --format json       # diagnostics as JSON on stdout
play_launch check <pkg> <file> --explain --ascii --width 120

# Visualization
play_launch plot
play_launch plot --log-dir <dir>
play_launch plot --metrics cpu --metrics memory --metrics io --metrics gpu
play_launch plot --list-metrics
```

## Development

### Prerequisites

- ROS 2 (Humble on Ubuntu 22.04; Jazzy or Kilted on 24.04; Lyrical or Rolling on 26.04), sourced: the build targets `$ROS_DISTRO`
- libclang (`libclang-dev`): rcl_interception_sys generates its bindings from the ROS headers
- Rust toolchain (stable)
- Python 3.10+
- [just](https://github.com/casey/just) command runner
- [cargo-nextest](https://nexte.st/) test runner
- [uv](https://github.com/astral-sh/uv) Python package manager

### Install Dependencies

```bash
just install-deps
```

This installs `colcon-cargo-ros2` and runs `rosdep install` for all ROS dependencies.

### Build

```bash
just build          # Full build: colcon + interception .so + wheel
just build-cpp      # C++ only (msgs + container)
just build-rust     # Rust only (assumes C++ install/ exists)
just build-wheel    # Bundle + wheel only (no colcon rebuild)
```

Optional: grant the I/O helper its monitoring capability and the RT helper its
scheduling capability (requires sudo). After this, RT scheduling (`--sched`) works
**without root**:

```bash
play_launch setcap
```

**Re-run it after every rebuild or upgrade.** A file capability is bound to the
exact contents of a binary, so replacing that binary drops it — deliberately,
since carrying privilege across a change would grant it to code nobody vetted.
play_launch says so when it happens, naming the grant it is replacing rather
than telling you to run a command you already ran.

(A `just setcap` recipe also exists. It is a DEVELOPER convenience that avoids
the password prompt via a throwaway container, it needs a source checkout, and
it does not work under rootless Docker. `play_launch setcap` is the supported
path.)

### Run

```bash
just run launch <package> <launch_file> [arguments...]
just run run <package> <executable> [arguments...]
```

### Test

```bash
just test               # Parser unit + fast integration (~3s)
just test-all           # Parser unit + all integration (~30s)
just test-unit          # Parser unit tests only
just test-integration   # All integration tests
just test-autoware      # Autoware integration tests
```

### Code Quality

```bash
just check      # clippy + rustfmt + ruff + cpplint + clang-format
just format     # Auto-format Rust + Python + C++
just quality    # Format then check
```

### Install Wheel Locally

```bash
just install-wheel
```

## License

MIT License. See [LICENSE.txt](LICENSE.txt).
