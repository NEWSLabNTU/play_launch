#!/usr/bin/env bash
# Benchmark Rust parser performance with Autoware launch file

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(cd "$TEST_DIR/../.." && pwd)"
OUTPUT_DIR="$TEST_DIR/output"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "================================"
echo "Autoware Parser Performance Benchmark"
echo "================================"
echo ""

# Check autoware symlink
if [ ! -L "$TEST_DIR/autoware" ]; then
    echo "ERROR: autoware symlink not found"
    exit 1
fi

AUTOWARE_PATH=$(readlink -f "$TEST_DIR/autoware")
LAUNCH_FILE="$AUTOWARE_PATH/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml"

if [ ! -f "$LAUNCH_FILE" ]; then
    echo "ERROR: Launch file not found: $LAUNCH_FILE"
    exit 1
fi

echo "Launch file: $(basename "$LAUNCH_FILE")"
echo "Size: $(wc -l < "$LAUNCH_FILE") lines"
echo ""

# Build parser
cd "$PROJECT_ROOT/src/play_launch_parser"
echo "Building Rust parser (release mode)..."
cargo build --release --quiet
echo -e "${GREEN}✓${NC} Built"
echo ""

RUST_PARSER="$PROJECT_ROOT/src/play_launch_parser/target/release/play_launch_parser"
MAP_PATH="${HOME}/autoware_map/sample-map-planning"

# Source Autoware setup to get package paths
if [ -f "$AUTOWARE_PATH/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$AUTOWARE_PATH/install/setup.bash" > /dev/null 2>&1
    echo -e "${GREEN}✓${NC} Sourced Autoware environment"
else
    echo "WARNING: Autoware setup.bash not found"
    exit 1
fi
echo ""

mkdir -p "$OUTPUT_DIR"

# Warmup
echo "Warming up..."
"$RUST_PARSER" file "$LAUNCH_FILE" \
    "map_path:=$MAP_PATH" \
    -o "$OUTPUT_DIR/warmup.json" &>/dev/null
echo ""

# Run benchmark
ITERATIONS=100
echo "Running benchmark (${ITERATIONS} iterations)..."

TOTAL_TIME=0
for i in $(seq 1 $ITERATIONS); do
    START=$(date +%s%N)

    "$RUST_PARSER" file "$LAUNCH_FILE" \
        "map_path:=$MAP_PATH" \
        -o "$OUTPUT_DIR/bench.json" &>/dev/null

    END=$(date +%s%N)
    TIME_NS=$((END - START))
    TOTAL_TIME=$((TOTAL_TIME + TIME_NS))

    if [ $((i % 10)) -eq 0 ]; then
        echo -n "."
    fi
done
echo ""
echo ""

# Calculate statistics
AVG_TIME_NS=$((TOTAL_TIME / ITERATIONS))
AVG_TIME_MS=$((AVG_TIME_NS / 1000000))
AVG_TIME_DECIMAL=$((AVG_TIME_NS / 100000))
AVG_TIME_FORMATTED="$((AVG_TIME_DECIMAL / 10)).$((AVG_TIME_DECIMAL % 10))ms"

THROUGHPUT=$((1000000000 / AVG_TIME_NS))

echo "${BLUE}Benchmark Results:${NC}"
echo "  Iterations: $ITERATIONS"
echo "  Average time: $AVG_TIME_FORMATTED"
echo "  Throughput: ~$THROUGHPUT parses/sec"
echo ""

# Load output and count nodes
NODE_COUNT=$(jq '.node | length' "$OUTPUT_DIR/bench.json")
echo "Parsed content:"
echo "  Nodes: $NODE_COUNT"
echo ""

echo "================================"
echo -e "${GREEN}Benchmark completed!${NC}"
echo "================================"
