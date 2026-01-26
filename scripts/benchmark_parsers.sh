#!/bin/bash
# Benchmark Rust vs Python parser performance
# Part of Phase 13: Rust Parser Migration

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Configuration
ITERATIONS=${ITERATIONS:-5}
WARMUP_ITERATIONS=1

# Color output
BOLD='\033[1m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test cases: "package:launch_file:description"
TEST_CASES=(
    "test/simple_test/launch/pure_nodes.launch.xml::Simple"
    "test/simple_test/launch/composition.launch.xml::Composable"
)

# Note: Autoware test requires external symlink, skip for now

echo -e "${BOLD}========================================"
echo "Parser Performance Benchmark"
echo "========================================${NC}"
echo ""
echo "Iterations per test: $ITERATIONS"
echo "Warmup iterations: $WARMUP_ITERATIONS"
echo "Total test cases: ${#TEST_CASES[@]}"
echo ""

# Results file
RESULTS_FILE="benchmark_results_$(date +%Y%m%d_%H%M%S).txt"

# Function to run benchmark
benchmark_parser() {
    local file_path="$1"
    local parser_flag="$2"
    local iterations="$3"

    local times=()

    for i in $(seq 1 "$iterations"); do
        # Clean up
        rm -f record.json

        # Time the command
        start_time=$(date +%s.%N)

        if [ "$parser_flag" = "rust" ]; then
            play_launch dump launch "$file_path" --parser rust > /dev/null 2>&1
        else
            play_launch dump launch "$file_path" --parser python > /dev/null 2>&1
        fi

        end_time=$(date +%s.%N)

        # Calculate elapsed time
        elapsed=$(echo "$end_time - $start_time" | bc)
        times+=("$elapsed")

        # Show progress
        if [ "$iterations" -gt 1 ]; then
            echo -n "." >&2
        fi
    done

    if [ "$iterations" -gt 1 ]; then
        echo "" >&2
    fi

    # Calculate statistics
    local sum=0
    local min=999999
    local max=0

    for time in "${times[@]}"; do
        sum=$(echo "$sum + $time" | bc)

        if (( $(echo "$time < $min" | bc -l) )); then
            min=$time
        fi

        if (( $(echo "$time > $max" | bc -l) )); then
            max=$time
        fi
    done

    local avg=$(echo "scale=3; $sum / ${#times[@]}" | bc)

    # Calculate standard deviation
    local variance_sum=0
    for time in "${times[@]}"; do
        local diff=$(echo "$time - $avg" | bc)
        local squared=$(echo "$diff * $diff" | bc)
        variance_sum=$(echo "$variance_sum + $squared" | bc)
    done

    local variance=$(echo "scale=6; $variance_sum / ${#times[@]}" | bc)
    local stddev=$(echo "scale=3; sqrt($variance)" | bc)

    # Return statistics as JSON-like string
    echo "$avg $min $max $stddev"
}

# Print table header
echo -e "${BOLD}| Test Case | Parser | Avg (s) | Min (s) | Max (s) | StdDev | Speedup |${NC}"
echo "|-----------|--------|---------|---------|---------|--------|---------|"

# Run benchmarks
for test_case in "${TEST_CASES[@]}"; do
    IFS=':' read -r file_path _empty desc <<< "$test_case"

    echo ""
    echo -e "${BLUE}Testing: $desc ($file_path)${NC}" >&2

    # Warmup
    if [ "$WARMUP_ITERATIONS" -gt 0 ]; then
        echo "  Warmup..." >&2
        benchmark_parser "$file_path" "rust" "$WARMUP_ITERATIONS" > /dev/null
        benchmark_parser "$file_path" "python" "$WARMUP_ITERATIONS" > /dev/null
    fi

    # Benchmark Rust parser
    echo "  Benchmarking Rust parser..." >&2
    rust_stats=$(benchmark_parser "$file_path" "rust" "$ITERATIONS")
    read -r rust_avg rust_min rust_max rust_stddev <<< "$rust_stats"

    # Benchmark Python parser
    echo "  Benchmarking Python parser..." >&2
    python_stats=$(benchmark_parser "$file_path" "python" "$ITERATIONS")
    read -r python_avg python_min python_max python_stddev <<< "$python_stats"

    # Calculate speedup
    speedup=$(echo "scale=2; $python_avg / $rust_avg" | bc)

    # Format speedup with color
    if (( $(echo "$speedup >= 5.0" | bc -l) )); then
        speedup_color="${GREEN}"
    elif (( $(echo "$speedup >= 3.0" | bc -l) )); then
        speedup_color="${YELLOW}"
    else
        speedup_color=""
    fi

    # Print results
    printf "| %-9s | %-6s | %7.3f | %7.3f | %7.3f | %6.3f |         |\n" \
        "$desc" "Rust" "$rust_avg" "$rust_min" "$rust_max" "$rust_stddev"
    printf "| %-9s | %-6s | %7.3f | %7.3f | %7.3f | %6.3f | ${speedup_color}%6.2fx${NC}  |\n" \
        "$desc" "Python" "$python_avg" "$python_min" "$python_max" "$python_stddev" "$speedup"

    # Save to results file
    {
        echo "Test: $desc ($file_path)"
        echo "  Rust:   avg=$rust_avg, min=$rust_min, max=$rust_max, stddev=$rust_stddev"
        echo "  Python: avg=$python_avg, min=$python_min, max=$python_max, stddev=$python_stddev"
        echo "  Speedup: ${speedup}x"
        echo ""
    } >> "$RESULTS_FILE"
done

echo ""
echo -e "${BOLD}========================================"
echo "Benchmark Complete"
echo "========================================${NC}"
echo ""
echo "Results saved to: $RESULTS_FILE"
echo ""

# Summary statistics
echo -e "${BOLD}Summary:${NC}"
echo "  - All benchmarks completed successfully"
echo "  - See detailed results in $RESULTS_FILE"
echo ""

# Cleanup
rm -f record.json
