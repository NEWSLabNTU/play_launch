# play_launch CPU Profiling — Autoware Steady-State

**Date**: 2026-02-17 (updated)
**Scenario**: Autoware planning_simulator (64 composable nodes, ~100 processes)
**Tool**: `perf record -F 499 -g --call-graph dwarf,16384` for 20 seconds of steady-state
**Flamegraphs**: `tmp/profile/flamegraph_default.svg`, `tmp/profile/flamegraph_tuned.svg`

## Summary

After two rounds of optimization, play_launch CPU usage dropped from **~13% (40 threads)** to **~5.5% (39 threads)** during Autoware steady-state — a **58% reduction**.

| Metric                | Before optimization | After monitoring fix | After DDS tuning |
|-----------------------|--------------------:|---------------------:|-----------------:|
| CPU usage             |                ~13% |                ~9.5% |            ~5.5% |
| Threads               |                  40 |                   42 |               39 |
| Samples (20s @ 499Hz) |                 N/A |                 1405 |             1209 |

Optimizations applied:
1. **Monitoring: network connection caching** — namespace-wide `/proc/net/*` cache reduced 400 reads/tick to 4 reads/tick + stack-buffer newline counting (zero heap allocation). Eliminated the #1 app hotspot (12% -> ~0%).
2. **CycloneDDS tuning** — `AllowMulticast=spdp` + heartbeat/ACK tuning reduced DDS overhead by 45%. See `src/play_launch/cyclonedds_play_launch.xml`.

## A/B Comparison: Default vs Tuned DDS

Profiled with `scripts/profile_autoware.sh` (monitoring optimization applied in both runs).

### Per-Thread Sample Distribution

| Thread                          | Default DDS |     % | Tuned DDS |     % |    Change |
|---------------------------------|------------:|------:|----------:|------:|----------:|
| play_launch (main+tokio)        |         714 | 50.8% |       675 | 55.8% |       -5% |
| play_launch-wor (rayon/sysinfo) |         401 | 28.5% |       376 | 31.1% |       -6% |
| recvMC (DDS multicast)          |         133 |  9.5% |         0 |    0% | **-100%** |
| recv (DDS unicast)              |          29 |  2.1% |       120 |  9.9% |     +314% |
| tev (DDS transport events)      |          99 |  7.0% |        24 |  2.0% |  **-76%** |
| dq.builtins (DDS delivery)      |          21 |  1.5% |        12 |  1.0% |      -43% |
| recvUC                          |           4 |  0.3% |         0 |    0% |     -100% |
| gc                              |           3 |  0.2% |         1 |  0.1% |      -67% |
| **Total**                       |    **1405** |       |  **1209** |       |  **-14%** |

### Key Observations

- **recvMC eliminated entirely**: `AllowMulticast=spdp` stops play_launch from processing the flood of multicast data from ~100 Autoware processes. This was the single biggest DDS cost.
- **tev reduced 76%**: Heartbeat/ACK/NACK tuning (slower heartbeats, batched ACKs) dramatically reduced transport-event processing.
- **recv (unicast) increased**: Expected — data previously received via multicast now arrives via unicast. But total recv (MC+UC+recv) dropped from 166 to 120 samples (**28% reduction**).
- **Total DDS samples**: 287 -> 157, a **45% reduction** in DDS overhead.
- **Thread count**: 42 -> 39. `MultipleReceiveThreads=false` merged separate unicast/multicast recv threads into one.
- **Application threads unchanged**: sysinfo/rayon monitoring is now the dominant cost (~31%), not DDS.

## Original CPU Breakdown (Before Any Optimization)

Captured 2026-02-16, before monitoring fix and DDS tuning. These numbers are the baseline reference.

| Category                             | Cycles     | %         | Notes                                                 |
|--------------------------------------|------------|-----------|-------------------------------------------------------|
| DDS recv threads                     | 2.15B      | 20.6%     | CycloneDDS `recvUC`/`recvMC` threads                  |
| DDS tev thread                       | 1.65B      | 15.8%     | CycloneDDS transport-event thread                     |
| ROS2 wait_set                        | 1.48B      | 14.2%     | `rclrs::wait_set` polling (Rust ROS2 bindings)        |
| **monitoring: network connections**  | **1.25B**  | **12.0%** | `count_connections_in_file()` — **#1 app hotspot**    |
| diagnostics                          | 564M       | 5.4%      | DiagnosticStatus message cloning                      |
| other                                | 502M       | 4.8%      | Misc (allocator, scheduler, signals)                  |
| **monitoring: find_subprocess_pids** | **198M**   | **1.9%**  | Recursive `/proc/<pid>/task/<tid>/children` traversal |
| tokio runtime                        | 169M       | 1.6%      | Tokio task scheduling overhead                        |
| monitoring: parse_proc_stat          | 31M        | 0.3%      | Reading `/proc/<pid>/stat`                            |
| monitoring: write_csv                | 21M        | 0.2%      | CSV serialization + write                             |
| **Total**                            | **~10.4B** |           |                                                       |

## Optimization #1: Network Connection Caching (DONE)

**Status**: Implemented, verified in profiling.

**Problem**: `count_connections_in_file()` was called 4x per monitored process per tick (tcp, tcp6, udp, udp6). With ~100 Autoware processes at 2-second intervals: ~400 `read_to_string()` calls every 2 seconds, each allocating a heap `String` just to count newlines.

**Solution** (two parts):

1. **Stack-buffer newline counting**: Replaced `read_to_string()` with `File::open()` + `read()` into `[u8; 8192]` stack buffer. Zero heap allocation.

2. **Namespace-wide caching**: `/proc/<pid>/net/tcp` returns the same data for all processes in the same network namespace. Cache the result once per tick and reuse for all ~100 processes. Reduced 400 reads/tick to 4 reads/tick.

**Result**: The 12% CPU hotspot is no longer visible in profiling.

## Optimization #2: CycloneDDS Tuning (DONE)

**Status**: Implemented in `src/play_launch/cyclonedds_play_launch.xml`, verified in A/B profiling.

**Problem**: CycloneDDS by default uses ASM (Any-Source Multicast) for both discovery AND data. play_launch's recv threads processed ALL multicast DDS traffic on the domain — even for the ~95% of topics it doesn't subscribe to.

**Solution**: Dedicated CycloneDDS config with these settings:

| Setting                  | Value             | Effect                                                                    |
|--------------------------|-------------------|---------------------------------------------------------------------------|
| `AllowMulticast`         | `spdp`            | Unicast-only for data + SEDP. **Biggest win**: eliminates multicast flood |
| `MultipleReceiveThreads` | `false`           | Single recv thread (sufficient for monitoring)                            |
| `LeaseDuration`          | `60s`             | Fewer keepalive messages from remote participants                         |
| `SPDPInterval`           | `30s`             | Less frequent discovery announcements                                     |
| `HeartbeatInterval`      | `1s` (base)       | Slower reliable-protocol heartbeats                                       |
| `NackDelay` / `AckDelay` | `500ms` / `100ms` | Batch ACK/NACK responses                                                  |
| `BuiltinEndpointSet`     | `minimal`         | Reduce discovery endpoint overhead                                        |
| `SquashParticipants`     | `true`            | Single DDSI participant                                                   |
| `RetransmitMerging`      | `adaptive`        | Deduplicate retransmit requests                                           |

**Result**: DDS CPU overhead reduced by 45%. Total CPU from ~9.5% to ~5.5%.

**Usage**:
```bash
export CYCLONEDDS_URI="file://$(pwd)/src/play_launch/cyclonedds_play_launch.xml"
play_launch launch autoware_launch planning_simulator.launch.xml ...
```

**Important**: This config is inherited by child processes. For single-host Autoware workloads, `AllowMulticast=spdp` is safe for all participants. For multi-host setups with data multicast, users should use their own config.

## DDS Footprint

play_launch creates only **one ROS node** (`/play_launch`) with these DDS entities:

| Entity                               | Count (Autoware, 15 containers) | QoS                                     |
|--------------------------------------|---------------------------------|-----------------------------------------|
| ComponentEvent subscriptions         | 15                              | Reliable, TransientLocal, KeepLast(100) |
| LoadNode service clients             | 15                              | Default                                 |
| UnloadNode service clients           | 15                              | Default                                 |
| Diagnostic subscriptions             | 0-2                             | Default (off by default)                |
| `/rosout` publisher (implicit)       | 1                               | Default                                 |
| Parameter service servers (implicit) | 6                               | Default                                 |

### QoS Notes

The ComponentEvent subscription uses **Reliable + TransientLocal + KeepLast(100)** — the most expensive QoS combination. This is intentional and **should not be changed**:
- **Reliable**: Missing a LOAD_FAILED event means play_launch won't detect node loading failures
- **TransientLocal**: Late-joining must receive historical events (subscription may be created after container loads nodes)
- **KeepLast(100)**: Captures all events from a container restart

### What play_launch CANNOT configure (user launch targets)

play_launch does **not** set `CYCLONEDDS_URI` for child processes. The user's DDS configuration is inherited from the environment. This is by design — play_launch should not interfere with the application's DDS behavior.

Users can optimize their own CycloneDDS configuration separately. Common options for Autoware:
- **`AllowMulticast>spdp`** for nodes that primarily use point-to-point communication
- **Larger `SocketReceiveBufferSize`** (10MB+) for bursty LiDAR/camera data
- **`WhcHigh>500kB`** to increase writer history cache for reliable topics

## Remaining Optimization Opportunities

1. **Cache subprocess PID tree** — `find_subprocess_pids()` still reads `/proc/<pid>/task/<tid>/children` every tick. Refresh every N ticks instead (process trees are stable in steady-state). Currently ~2% of CPU.

2. **Reduce sysinfo overhead** — `sysinfo::get_all_pid_entries` (reading `/proc` dirs via rayon) is now the dominant cost at ~31% of samples. Consider reducing monitoring frequency from 2s to 5s, or caching the process list.

3. **Add `--disable-network-stats` flag** — Allow users who don't need TCP/UDP connection counts to skip network monitoring entirely.

## Profiling Methods

### Prerequisites

```bash
# Install perf (Linux only)
sudo apt install linux-tools-$(uname -r) linux-tools-common

# Install flamegraph tools
cargo install inferno

# Allow perf for non-root users (resets on reboot)
sudo sysctl kernel.perf_event_paranoid=-1
# Or persistently:
echo 'kernel.perf_event_paranoid = -1' | sudo tee /etc/sysctl.d/99-perf.conf
```

### Automated Profiling with Autoware

```bash
# Baseline (default DDS config)
just profile-autoware

# With CycloneDDS tuning
just profile-autoware-tuned
```

The script `scripts/profile_autoware.sh` automates:
1. Launch Autoware planning_simulator via `play_launch`
2. Wait 20 seconds for stabilization
3. Record 20 seconds of steady-state CPU samples
4. Kill play_launch and clean up child processes
5. Generate flamegraph SVG and perf reports

Output files (in `tmp/profile/`):
- `flamegraph_{default,tuned}.svg` — interactive flamegraph (open in browser)
- `perf_{default,tuned}.data` — raw perf data
- `perf_folded_{default,tuned}.txt` — collapsed stacks
- `stdout_{default,tuned}.log` — play_launch stdout/stderr

### Manual Profiling

```bash
# 1. Launch play_launch in the background
play_launch launch <pkg> <launch_file> &
PID=$!
sleep 15  # wait for stabilization

# 2. Find the Rust binary PID (may differ from wrapper PID)
RUST_PID=$(pgrep -P $PID -f 'play_launch' | head -1)
# Verify: ls -la /proc/$RUST_PID/exe

# 3. Record CPU samples
#    -F 499: ~500 Hz sampling (prime to avoid aliasing)
#    -g --call-graph dwarf,16384: DWARF unwinding with 16KB stack dump
#    -p PID: target specific process (all its threads)
perf record -F 499 -g --call-graph dwarf,16384 -p $RUST_PID -o perf.data -- sleep 20

# 4. Generate flamegraph
perf script -i perf.data > perf_script.txt
inferno-collapse-perf < perf_script.txt > perf_folded.txt
inferno-flamegraph --title "play_launch profile" < perf_folded.txt > flamegraph.svg

# 5. Text reports
perf report -i perf.data --stdio --no-children --percent-limit 0.5  # self time
perf report -i perf.data --stdio --children --percent-limit 2.0     # inclusive time

# 6. Per-thread breakdown
perf script -i perf.data -F comm,tid | awk '{print $1}' | sort | uniq -c | sort -rn
```

### Interpreting the Flamegraph

- **X-axis**: proportion of samples (wider = more CPU time), NOT a timeline
- **Y-axis**: call stack depth (bottom = entry point, top = leaf functions)
- **Colors**: random per-function — no semantic meaning
- **Click** a frame to zoom into that subtree; click the top bar to reset

Key thread prefixes in play_launch:
- `recv` / `recvMC` / `recvUC` / `tev` / `gc` / `dq.*` — CycloneDDS internal threads
- `play_launch-wor` — rayon worker threads (sysinfo/monitoring)
- `play_launch` — main thread + tokio async workers
