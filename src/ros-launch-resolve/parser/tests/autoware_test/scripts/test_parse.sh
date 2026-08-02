#!/usr/bin/env bash
# Test Rust parser against Autoware planning simulator launch file

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(cd "$TEST_DIR/../.." && pwd)"
OUTPUT_DIR="$TEST_DIR/output"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "================================"
echo "Autoware Launch File Parser Test"
echo "================================"
echo ""

# Check if autoware symlink exists
if [ ! -L "$TEST_DIR/autoware" ]; then
    echo -e "${RED}ERROR: autoware symlink not found${NC}"
    echo "Create symlink: cd $TEST_DIR && ln -s /path/to/autoware autoware"
    exit 1
fi

AUTOWARE_PATH=$(readlink -f "$TEST_DIR/autoware")
if [ ! -d "$AUTOWARE_PATH" ]; then
    echo -e "${RED}ERROR: Autoware path invalid: $AUTOWARE_PATH${NC}"
    exit 1
fi

# Find the planning simulator launch file
LAUNCH_FILE="$AUTOWARE_PATH/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml"

if [ ! -f "$LAUNCH_FILE" ]; then
    echo -e "${RED}ERROR: Launch file not found: $LAUNCH_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} Found Autoware launch file"
echo "  Path: $LAUNCH_FILE"
echo "  Size: $(wc -l < "$LAUNCH_FILE") lines"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Build Rust parser if needed
echo "Building Rust parser (with Python support)..."
cd "$PROJECT_ROOT/src/play_launch_parser"
cargo build --release --features python --quiet
echo -e "${GREEN}✓${NC} Rust parser built"
echo ""

RUST_PARSER="$PROJECT_ROOT/src/play_launch_parser/target/release/play_launch_parser"

# Source Autoware setup to get package paths
if [ -f "$AUTOWARE_PATH/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$AUTOWARE_PATH/install/setup.bash" > /dev/null 2>&1
    echo -e "${GREEN}✓${NC} Sourced Autoware environment"
else
    echo -e "${YELLOW}⚠${NC} Autoware setup.bash not found, package resolution may fail"
fi
echo ""

# Test Rust parser
echo "Testing Rust parser..."
RUST_OUTPUT="$OUTPUT_DIR/rust_output.json"

# Set required map_path argument (Autoware requires this)
MAP_PATH="${HOME}/autoware_map/sample-map-planning"

# Time the execution
START_TIME=$(date +%s%N)

"$RUST_PARSER" file "$LAUNCH_FILE" \
    "map_path:=$MAP_PATH" \
    -o "$RUST_OUTPUT" 2>&1 | tee "$OUTPUT_DIR/rust_stderr.log"

# Check the parser exit code (from PIPESTATUS, not tee)
PARSER_EXIT_CODE=${PIPESTATUS[0]}

END_TIME=$(date +%s%N)
RUST_TIME_MS=$(( (END_TIME - START_TIME) / 1000000 ))

if [ "$PARSER_EXIT_CODE" -ne 0 ]; then
    echo -e "${RED}✗ Rust parser failed (exit code: $PARSER_EXIT_CODE)${NC}"
    echo "  See error log: $OUTPUT_DIR/rust_stderr.log"
    exit 1
fi

echo -e "${GREEN}✓${NC} Rust parser succeeded"
echo "  Output: $RUST_OUTPUT"
echo "  Parse time: ${RUST_TIME_MS}ms"
echo ""

# Validate JSON output
echo "Validating JSON output..."
if ! jq empty "$RUST_OUTPUT" 2>/dev/null; then
    echo -e "${RED}✗ Invalid JSON output${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} JSON output valid"
echo ""

# Count nodes in output
NODE_COUNT=$(jq '.node | length' "$RUST_OUTPUT")
CONTAINER_COUNT=$(jq '.container | length' "$RUST_OUTPUT")
LOAD_NODE_COUNT=$(jq '.load_node | length' "$RUST_OUTPUT")

echo "Parsed results:"
echo "  Nodes: $NODE_COUNT"
echo "  Containers: $CONTAINER_COUNT"
echo "  Composable nodes: $LOAD_NODE_COUNT"
echo ""

# Test with Python dump_launch if available
if command -v dump_launch &> /dev/null; then
    echo "Comparing with Python dump_launch..."

    PYTHON_OUTPUT="$OUTPUT_DIR/python_output.json"

    # Source Autoware setup to get package paths
    if [ -f "$AUTOWARE_PATH/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "$AUTOWARE_PATH/install/setup.bash" > /dev/null 2>&1
    fi

    START_TIME=$(date +%s%N)

    if dump_launch file "$LAUNCH_FILE" \
        --args "map_path:=$MAP_PATH" \
        --output "$PYTHON_OUTPUT" 2>"$OUTPUT_DIR/python_stderr.log"; then

        END_TIME=$(date +%s%N)
        PYTHON_TIME_MS=$(( (END_TIME - START_TIME) / 1000000 ))

        echo -e "${GREEN}✓${NC} Python dump_launch succeeded"
        echo "  Output: $PYTHON_OUTPUT"
        echo "  Parse time: ${PYTHON_TIME_MS}ms"
        echo ""

        # Performance comparison
        if [ "$RUST_TIME_MS" -gt 0 ]; then
            SPEEDUP=$((PYTHON_TIME_MS / RUST_TIME_MS))
            echo "Performance:"
            echo "  Rust:   ${RUST_TIME_MS}ms"
            echo "  Python: ${PYTHON_TIME_MS}ms"
            echo -e "  ${GREEN}Speedup: ${SPEEDUP}x${NC}"
            echo ""
        fi

        # Compare node counts
        PYTHON_NODE_COUNT=$(jq '.node | length' "$PYTHON_OUTPUT")

        if [ "$NODE_COUNT" -eq "$PYTHON_NODE_COUNT" ]; then
            echo -e "${GREEN}✓${NC} Node count matches: $NODE_COUNT"
        else
            echo -e "${YELLOW}⚠${NC} Node count mismatch:"
            echo "  Rust:   $NODE_COUNT"
            echo "  Python: $PYTHON_NODE_COUNT"
        fi
        echo ""
    else
        echo -e "${YELLOW}⚠${NC} Python dump_launch failed (see $OUTPUT_DIR/python_stderr.log)"
    fi
else
    echo -e "${YELLOW}⚠${NC} Python dump_launch not found (skipping comparison)"
    echo ""
fi

echo "================================"
echo -e "${GREEN}Test completed successfully!${NC}"
echo "================================"
echo ""
echo "Output files:"
echo "  - $RUST_OUTPUT"
if [ -f "$PYTHON_OUTPUT" ]; then
    echo "  - $PYTHON_OUTPUT"
fi
echo ""
