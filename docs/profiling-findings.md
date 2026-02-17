# play_launch CPU Profiling — Autoware Steady-State

**Date**: 2026-02-16
**Scenario**: Autoware planning_simulator (64 composable nodes, ~100 processes)
**Tool**: `perf record -F 997 -g --call-graph dwarf` for 20 seconds of steady-state
**Flamegraph**: `tmp/flamegraph.svg`

## Summary

play_launch uses ~13% CPU with 40 threads during Autoware steady-state. Over half of that CPU is consumed by CycloneDDS internal threads (inherent cost of being a DDS participant). The largest actionable hotspot is the monitoring subsystem's `/proc` file reading.

## CPU Breakdown by Category

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

## Hotspot #1: `count_connections_in_file()` (12% of total CPU)

**File**: `src/play_launch/src/monitoring/resource_monitor.rs:189`

```rust
fn count_connections_in_file(path: &str) -> u32 {
    match std::fs::read_to_string(path) {
        Ok(content) => {
            let line_count = content.lines().count();
            if line_count > 0 { (line_count - 1) as u32 } else { 0 }
        }
        Err(_) => 0,
    }
}
```

**Why it's expensive**: Called 4 times per monitored process per tick (tcp, tcp6, udp, udp6). With ~100 Autoware processes at 2-second intervals, that's ~400 `read_to_string()` calls every 2 seconds. Each call:
1. Opens the procfs file (kernel generates content on-the-fly)
2. Allocates a `String` and reads the entire file contents
3. Iterates the string to count newlines
4. Drops the allocation

The function only needs the **line count**, but reads and allocates the entire file content.

**Optimization opportunities**:
- **Read into a reusable buffer** instead of allocating a new `String` per call
- **Count newlines via `read()` into a fixed stack buffer** (no heap allocation, no full-file buffering)
- **Aggregate subprocess network connections**: `/proc/<pid>/net/tcp` already shows connections for all threads in the namespace, so reading it once per process group may suffice
- **Skip network connection counting entirely** if the user hasn't enabled network monitoring (add a config flag)

## Hotspot #2: `find_subprocess_pids()` (1.9% of total CPU)

**File**: `src/play_launch/src/monitoring/resource_monitor.rs:206`

```rust
fn find_subprocess_pids(parent_pid: u32) -> Vec<u32> {
    let mut pids = Vec::new();
    let task_dir = format!("/proc/{}/task", parent_pid);
    if let Ok(entries) = std::fs::read_dir(task_dir) {
        for entry in entries.flatten() {
            let children_path = entry.path().join("children");
            if let Ok(content) = std::fs::read_to_string(&children_path) {
                for pid_str in content.split_whitespace() {
                    if let Ok(child_pid) = pid_str.parse::<u32>() {
                        pids.push(child_pid);
                        let mut grandchildren = find_subprocess_pids(child_pid);
                        pids.append(&mut grandchildren);
                    }
                }
            }
        }
    }
    pids
}
```

**Why it's expensive**: For each monitored process, iterates every thread's `/proc/<pid>/task/<tid>/children` file, then recurses into each child. A process with 40 threads means 40 `read_dir` + 40 `read_to_string` calls per tick, multiplied across all monitored processes.

**Optimization opportunities**:
- **Cache the subprocess tree** and only refresh it periodically (processes don't spawn/die every 2 seconds)
- **Use `/proc/<pid>/children`** (single file) instead of iterating all task dirs — available on Linux 4.2+ (kernel 6.8 qualifies)
- **Read children non-recursively** and only go one level deep (Autoware containers don't have deep process trees)

## Non-Actionable Overhead (DDS + ROS2): ~50%

About half of CPU time is consumed by CycloneDDS middleware threads and ROS2 wait-set polling. These are inherent costs of being a DDS participant:

- **DDS recv threads (20.6%)**: Receiving multicast/unicast DDS packets from ~100 Autoware processes
- **DDS tev thread (15.8%)**: Transport-event processing (heartbeats, ACKs, NAKs, retransmits)
- **ROS2 wait_set (14.2%)**: Polling for incoming messages on subscribed topics (ComponentEvent, diagnostics, etc.)

These cannot be reduced without reducing the number of DDS subscriptions or switching RMW implementations.

## Recommendations (Priority Order)

1. **Optimize `count_connections_in_file()`** — Use a stack-allocated buffer + `read()` syscall to count newlines without heap allocation. Expected ~80% reduction in the 12% budget (~10% total savings).

2. **Cache subprocess PID tree** — Refresh every N ticks instead of every tick. Process trees are stable in steady-state.

3. **Add `--disable-network-stats` flag** — Allow users who don't need TCP/UDP connection counts to skip the most expensive monitoring operation entirely.

4. **Consider reducing monitoring frequency** — The 2-second default may be unnecessarily frequent for steady-state. A 5-second interval would cut monitoring overhead by 60%.

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

The script `tmp/profile_autoware.sh` automates the full workflow:

```bash
bash tmp/profile_autoware.sh
```

It performs these steps:
1. Launches Autoware planning_simulator via `play_launch`
2. Waits 15 seconds for process stabilization
3. Finds the Rust binary PID (pip entry point execs into Rust binary)
4. Records 20 seconds of steady-state CPU samples
5. Kills play_launch and cleans up child processes
6. Generates flamegraph SVG and perf reports

Output files (in `tmp/`):
- `flamegraph.svg` — interactive flamegraph (open in browser)
- `perf.data` — raw perf data (~130MB)
- `perf_folded.txt` — collapsed stacks for flamegraph
- `perf_script.txt` — human-readable perf script output
- `perf_stdout.log` — play_launch stdout/stderr during profiling

### Manual Profiling

For profiling specific scenarios or non-Autoware workloads:

```bash
# 1. Launch play_launch in the background
play_launch launch <pkg> <launch_file> &
PID=$!
sleep 15  # wait for stabilization

# 2. Find the Rust binary PID (may differ from wrapper PID)
RUST_PID=$(pgrep -f 'site-packages/play_launch/bin/play_launch' | head -1)
# Verify: ls -la /proc/$RUST_PID/exe

# 3. Record CPU samples
#    -F 997: ~1000 Hz sampling (prime to avoid aliasing with periodic tasks)
#    -g --call-graph dwarf,32768: DWARF unwinding with 32KB stack dump
#    -p PID: target specific process (all its threads)
perf record -F 997 -g --call-graph dwarf,32768 -p $RUST_PID -o perf.data -- sleep 20

# 4. Generate flamegraph
perf script -i perf.data > perf_script.txt
inferno-collapse-perf < perf_script.txt > perf_folded.txt
inferno-flamegraph --title "play_launch profile" < perf_folded.txt > flamegraph.svg

# 5. Text reports
perf report -i perf.data --stdio --no-children --percent-limit 0.5  # self time
perf report -i perf.data --stdio --children --percent-limit 2.0     # inclusive time
```

### Interpreting the Flamegraph

- **X-axis**: proportion of samples (wider = more CPU time), NOT a timeline
- **Y-axis**: call stack depth (bottom = entry point, top = leaf functions)
- **Colors**: random per-function — no semantic meaning
- **Click** a frame to zoom into that subtree; click the top bar to reset

Key thread prefixes in play_launch:
- `dds:recv*` / `dds:tev` / `dds:gc` / `dds:dq.*` — CycloneDDS internal threads
- `tokio-runtime-*` — Tokio async worker threads
- Thread names matching function names (e.g., `monitoring`, `diagnostics`) — play_launch application threads

### Analyzing perf Data Further

```bash
# Per-thread breakdown
perf report -i perf.data --stdio --sort comm,dso --percent-limit 1.0

# Annotate a specific function (source-level hotspots)
perf annotate -i perf.data -s count_connections_in_file

# Export to Firefox Profiler (alternative visualization)
perf script -i perf.data -F +pid > profile_for_firefox.txt
```
