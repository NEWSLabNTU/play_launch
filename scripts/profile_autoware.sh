#!/usr/bin/env bash
# Profile play_launch during Autoware steady-state execution.
#
# Usage:
#   scripts/profile_autoware.sh [--dds-tuned] [--duration SECS] [--output-dir DIR]
#
# Options:
#   --dds-tuned     Apply cyclonedds_play_launch.xml (optimized DDS config)
#   --duration N    Perf recording duration in seconds (default: 20)
#   --output-dir D  Output directory for perf data and flamegraph (default: tmp/profile)
#
# Prerequisites:
#   - perf (linux-tools-$(uname -r))
#   - inferno (cargo install inferno)
#   - kernel.perf_event_paranoid <= 1
#   - Autoware workspace at $AUTOWARE_WS
#   - play_launch installed via pip (just build && just install-wheel)
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
AUTOWARE_WS="${AUTOWARE_WS:-/home/aeon/repos/autoware/1.5.0-ws}"

# Defaults
DDS_TUNED=false
DURATION=20
OUTPUT_DIR="$PROJECT_ROOT/tmp/profile"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dds-tuned)    DDS_TUNED=true; shift ;;
        --duration)     DURATION="$2"; shift 2 ;;
        --output-dir)   OUTPUT_DIR="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# Label for output files
if $DDS_TUNED; then
    LABEL="tuned"
else
    LABEL="default"
fi

mkdir -p "$OUTPUT_DIR"

# --- Preflight checks ---
if ! command -v perf &>/dev/null; then
    echo "ERROR: perf not found. Install with: sudo apt install linux-tools-$(uname -r)"
    exit 1
fi
if ! command -v inferno-collapse-perf &>/dev/null; then
    echo "ERROR: inferno not found. Install with: cargo install inferno"
    exit 1
fi
PARANOID=$(cat /proc/sys/kernel/perf_event_paranoid)
if [ "$PARANOID" -gt 1 ]; then
    echo "ERROR: kernel.perf_event_paranoid=$PARANOID (need <= 1)"
    echo "  Fix: sudo sysctl kernel.perf_event_paranoid=-1"
    exit 1
fi
if [ ! -d "$AUTOWARE_WS/install" ]; then
    echo "ERROR: Autoware workspace not found at $AUTOWARE_WS"
    echo "  Set AUTOWARE_WS to your Autoware workspace path"
    exit 1
fi

# --- Environment setup ---
source "$AUTOWARE_WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY="${DISPLAY:-:1}"

if $DDS_TUNED; then
    DDS_XML="$PROJECT_ROOT/src/play_launch/cyclonedds_play_launch.xml"
    if [ ! -f "$DDS_XML" ]; then
        echo "ERROR: DDS config not found at $DDS_XML"
        exit 1
    fi
    export CYCLONEDDS_URI="file://$DDS_XML"
    echo "=== DDS config: $DDS_XML ==="
else
    unset CYCLONEDDS_URI
    echo "=== DDS config: CycloneDDS default ==="
fi

LAUNCH_ARGS=(
    launch
    --disable-web-ui
    autoware_launch planning_simulator.launch.xml
    "map_path:=$HOME/autoware_map/sample-map-planning"
    vehicle_model:=sample_vehicle
    sensor_model:=sample_sensor_kit
)

# --- Step 1: Launch ---
echo "=== Step 1: Launch play_launch ==="
cd "$AUTOWARE_WS"
play_launch "${LAUNCH_ARGS[@]}" > "$OUTPUT_DIR/stdout_${LABEL}.log" 2>&1 &
WRAPPER_PID=$!
echo "Wrapper PID: $WRAPPER_PID"

sleep 3

# Find the Rust binary PID (child of our wrapper, not a stale process)
RUST_PID=$(pgrep -P $WRAPPER_PID -f 'play_launch' | head -1)
if [ -z "$RUST_PID" ]; then
    RUST_PID=$WRAPPER_PID
fi
echo "Rust binary PID: $RUST_PID"
ls -la /proc/$RUST_PID/exe 2>/dev/null || true

# --- Step 2: Wait for stabilization ---
echo "=== Step 2: Wait for stabilization (20s) ==="
sleep 20

if ! kill -0 $RUST_PID 2>/dev/null; then
    echo "ERROR: play_launch died before profiling"
    tail -20 "$OUTPUT_DIR/stdout_${LABEL}.log"
    # Clean up
    kill -TERM $WRAPPER_PID 2>/dev/null || true
    exit 1
fi

THREAD_COUNT=$(ls /proc/$RUST_PID/task/ 2>/dev/null | wc -l)
echo "Thread count: $THREAD_COUNT"
echo "CPU snapshot (before):"
ps -p $RUST_PID -o pid,pcpu,pmem,nlwp,comm --no-headers
echo ""

# --- Step 3: perf record ---
PERF_DATA="$OUTPUT_DIR/perf_${LABEL}.data"
rm -f "$PERF_DATA"  # perf fails with exit 255 if data file already exists
echo "=== Step 3: perf record for ${DURATION}s (steady state) ==="
perf record -F 499 -g --call-graph dwarf,16384 -p $RUST_PID -o "$PERF_DATA" -- sleep "$DURATION" || true
echo "perf record complete: $(du -h "$PERF_DATA" | cut -f1)"

# CPU snapshot after recording
echo "CPU snapshot (after):"
ps -p $RUST_PID -o pid,pcpu,pmem,nlwp,comm --no-headers 2>/dev/null || echo "(process exited)"
echo ""

# --- Step 4: Kill play_launch ---
echo "=== Step 4: Kill play_launch ==="
# Cleanup is handled by the EXIT trap, but we do it here explicitly
# so flamegraph generation doesn't compete with play_launch for CPU.
kill -TERM $WRAPPER_PID 2>/dev/null || true
wait $WRAPPER_PID 2>/dev/null || true
pkill -TERM -f 'component_container|component_node' 2>/dev/null || true
sleep 2
pkill -9 -f 'component_container|component_node' 2>/dev/null || true
unset WRAPPER_PID  # prevent EXIT trap from double-killing

# --- Step 5: Generate flamegraph ---
echo "=== Step 5: Generate flamegraph ==="
PERF_SCRIPT="$OUTPUT_DIR/perf_script_${LABEL}.txt"
PERF_FOLDED="$OUTPUT_DIR/perf_folded_${LABEL}.txt"
FLAMEGRAPH="$OUTPUT_DIR/flamegraph_${LABEL}.svg"

perf script -i "$PERF_DATA" > "$PERF_SCRIPT" 2>/dev/null
echo "perf script: $(wc -l < "$PERF_SCRIPT") lines"

inferno-collapse-perf < "$PERF_SCRIPT" > "$PERF_FOLDED"
echo "collapsed stacks: $(wc -l < "$PERF_FOLDED") lines"

TITLE="play_launch Autoware steady-state (${DURATION}s, $LABEL)"
inferno-flamegraph --title "$TITLE" < "$PERF_FOLDED" > "$FLAMEGRAPH"

# --- Step 6: Summary report ---
echo ""
echo "=== Summary ($LABEL) ==="
echo "Threads: $THREAD_COUNT"
echo "Samples: $(perf report -i "$PERF_DATA" --stdio 2>/dev/null | grep -c '^#.*events'  || echo 'N/A')"
echo ""

echo "--- Top functions (self time, >0.5%) ---"
perf report -i "$PERF_DATA" --stdio --no-children --percent-limit 0.5 2>/dev/null | head -80
echo ""

echo "--- Top functions (inclusive, >2%) ---"
perf report -i "$PERF_DATA" --stdio --children --percent-limit 2.0 2>/dev/null | head -80
echo ""

echo "Flamegraph: $FLAMEGRAPH"
echo "Perf data:  $PERF_DATA"
echo "Stdout log: $OUTPUT_DIR/stdout_${LABEL}.log"
echo "Done ($LABEL)."
