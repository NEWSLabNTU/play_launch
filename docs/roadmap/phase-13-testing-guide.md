# Phase 13: Testing Guide

**Related**: [phase-13.md](./phase-13.md)

This guide documents the testing infrastructure for the Rust parser migration (Phase 13).

---

## Overview

The Rust parser migration requires comprehensive testing to ensure:

1. **Correctness**: Rust and Python parsers produce identical `record.json` output
2. **Performance**: Rust parser achieves 5-10x speedup over Python
3. **Reliability**: Error handling and fallback mechanisms work correctly
4. **Compatibility**: All launch file formats (XML, YAML, Python) are supported

---

## Test Scripts

### 1. Comparison Tests

**Script**: `scripts/compare_parsers.sh`

**Purpose**: Compare Rust and Python parser outputs for semantic equivalence.

**Usage**:
```bash
# Run all comparison tests
./scripts/compare_parsers.sh

# The script tests:
# - demo_nodes_cpp/talker_listener.launch.py (Simple Python launch)
# - test/simple_test/simple_test.launch.xml (Simple XML with composables)
# - test/autoware_planning_simulation/planning_simulator.launch.xml (Large Autoware launch)
```

**Output**:
```
========================================
Parser Comparison Test Suite
========================================

----------------------------------------
Test: Simple Python launch
File: demo_nodes_cpp/talker_listener.launch.py
----------------------------------------
  [1/3] Generating with Rust parser...
  ✓ Rust parser succeeded
  [2/3] Generating with Python parser...
  ✓ Python parser succeeded
  [3/3] Comparing outputs...
  ✓ Records are semantically equivalent
  ✓ PASS: Outputs match

... (more tests) ...

========================================
Summary
========================================

Total tests: 3
Passed: 3
Failed: 0

All tests passed!
```

**Exit Codes**:
- `0`: All tests passed
- `1`: One or more tests failed

---

### 2. Performance Benchmarks

**Script**: `scripts/benchmark_parsers.sh`

**Purpose**: Measure and compare Rust vs Python parser performance.

**Usage**:
```bash
# Run with default settings (5 iterations)
./scripts/benchmark_parsers.sh

# Run with more iterations for accuracy
ITERATIONS=10 ./scripts/benchmark_parsers.sh
```

**Output**:
```
========================================
Parser Performance Benchmark
========================================

Iterations per test: 5
Warmup iterations: 1
Total test cases: 3

| Test Case | Parser | Avg (s) | Min (s) | Max (s) | StdDev | Speedup |
|-----------|--------|---------|---------|---------|--------|---------|
| Simple    | Rust   |   0.842 |   0.801 |   0.921 |  0.045 |         |
| Simple    | Python |   2.514 |   2.451 |   2.603 |  0.058 |   2.99x  |
| Medium    | Rust   |   1.234 |   1.201 |   1.287 |  0.032 |         |
| Medium    | Python |   4.567 |   4.501 |   4.687 |  0.071 |   3.70x  |
| Large     | Rust   |   4.123 |   4.012 |   4.289 |  0.098 |         |
| Large     | Python |  39.456 |  38.901 |  40.123 |  0.487 |   9.57x  |

========================================
Benchmark Complete
========================================

Results saved to: benchmark_results_20260125_143022.txt
```

**Results File**: Timestamped file with detailed statistics for all test runs.

**Environment Variables**:
- `ITERATIONS`: Number of iterations per test (default: 5)

---

### 3. Record Comparison Utility

**Script**: `scripts/compare_records.py`

**Purpose**: Semantic comparison of two `record.json` files, accounting for differences in ordering, formatting, and type representation.

**Usage**:
```bash
# Compare two record.json files
python3 scripts/compare_records.py record_rust.json record_python.json
```

**Features**:
- Normalizes array ordering (sorts by name/namespace)
- Handles type differences (e.g., `"42"` vs `42`)
- Compares parameter lists semantically
- Provides detailed diff output on mismatch

**Output (on match)**:
```
  ✓ Records are semantically equivalent
```

**Output (on mismatch)**:
```
  ✗ Records differ

  Difference Summary:
  ============================================================
  node count: Rust=15, Python=14

  Node differences detected:

    Node 3:
      Rust:   my_node (demo_nodes_cpp)
      Python: my_node (demo_nodes_cpp)
        params:
          Rust:   [('param1', 42)]
          Python: [('param1', '42')]
  ============================================================
```

**Exit Codes**:
- `0`: Records match
- `1`: Records differ or error

---

## Integration Tests (Rust)

**Location**: `tests/parser_integration_test.rs` (to be created in Task 13.4.3)

**Purpose**: Rust-level integration tests for parser functionality.

**Test Cases**:

```rust
#[test]
fn test_rust_parser_simple_launch() {
    // Test basic parser invocation
    // Verify record.json structure
}

#[test]
fn test_rust_parser_with_arguments() {
    // Test launch argument parsing (KEY:=VALUE)
    // Verify arguments appear in record.json
}

#[test]
fn test_rust_parser_xml_launch() {
    // Test XML launch file support
}

#[test]
fn test_rust_parser_yaml_launch() {
    // Test YAML launch file support
}

#[test]
fn test_rust_parser_composable_nodes() {
    // Test composable node loading
}

#[test]
fn test_error_handling() {
    // Test missing file error
    // Test invalid syntax error
}

#[test]
fn test_automatic_fallback() {
    // Test that Python fallback is triggered on parser error
}
```

**Run Tests**:
```bash
cd src/play_launch
cargo test parser_integration
```

---

## Test Matrix

### Launch File Formats

| Format | Test File | Features Tested | Expected Result |
|--------|-----------|-----------------|-----------------|
| XML | demo_nodes_cpp/talker.launch.xml | Simple node launch | Identical output |
| Python | demo_nodes_cpp/talker_listener.launch.py | Python launch API | Identical output |
| YAML | autoware presets | YAML includes, variables | Identical output |

### Complexity Levels

| Level | Test Case | Nodes | Containers | Composables | Parse Time Target |
|-------|-----------|-------|------------|-------------|-------------------|
| Simple | demo_nodes_cpp | 2 | 0 | 0 | <1s |
| Medium | test/simple_test | 3 | 1 | 2 | <2s |
| Large | Autoware planning_simulator | 115 | 15 | 52 | <5s |

### Launch Arguments

| Argument Type | Example | Expected Behavior |
|---------------|---------|-------------------|
| Boolean | `use_sim_time:=true` | Parsed as boolean parameter |
| String | `namespace:=/my_robot` | Parsed as string parameter |
| Integer | `count:=42` | Parsed as integer parameter |
| Float | `rate:=10.5` | Parsed as float parameter |
| Path | `config:=/path/to/file.yaml` | Parsed as string parameter |

### Error Scenarios

| Scenario | Expected Behavior |
|----------|-------------------|
| Missing launch file | Clear error message, no fallback |
| Invalid XML syntax | Parser error, automatic Python fallback |
| Invalid Python syntax | Parser error, automatic Python fallback |
| Circular includes | Parser error, clear error message |
| Missing package | Clear error message |

---

## Continuous Integration

### GitHub Actions Workflow

**File**: `.github/workflows/parser-tests.yml` (to be created)

```yaml
name: Parser Tests

on:
  pull_request:
    paths:
      - 'src/play_launch/**'
      - 'src/play_launch_parser/**'
      - 'scripts/compare_*.{sh,py}'
  push:
    branches: [main]

jobs:
  parser-comparison:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
        with:
          submodules: recursive

      - name: Setup ROS 2 Humble
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: humble

      - name: Build play_launch
        run: |
          just build
          pip install dist/*.whl

      - name: Run comparison tests
        run: ./scripts/compare_parsers.sh

      - name: Run benchmarks
        run: |
          ITERATIONS=3 ./scripts/benchmark_parsers.sh
          cat benchmark_results_*.txt

  parser-integration:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
        with:
          submodules: recursive

      - name: Setup Rust
        uses: actions-rs/toolchain@v1
        with:
          toolchain: stable

      - name: Run integration tests
        run: |
          cd src/play_launch
          cargo test parser_integration
```

---

## Performance Targets

### Acceptance Criteria

| Metric | Target | Critical Threshold |
|--------|--------|--------------------|
| Speedup (Simple) | ≥3x | ≥2x |
| Speedup (Medium) | ≥5x | ≥3x |
| Speedup (Large/Autoware) | ≥8x | ≥5x |
| Parse time (Autoware) | <5s | <10s |
| Record equivalence | 100% | 100% |

### Performance Regression Detection

If benchmarks show performance degradation:

1. **<10% regression**: Warning, investigate but don't block
2. **10-20% regression**: Review required, consider blocking
3. **>20% regression**: Block merge, requires investigation

---

## Test Data

### Test Launch Files

**Location**: `test/` directory

**Simple Test** (`test/simple_test/simple_test.launch.xml`):
- 1 container
- 2 composable nodes
- Basic parameter passing
- Tests: Basic composable node loading

**Autoware Test** (`test/autoware_planning_simulation/planning_simulator.launch.xml`):
- 15 containers
- 52 composable nodes
- Complex include hierarchy
- YAML parameter files
- Tests: Real-world complexity, performance scaling

### Fixture Files

**Location**: `tests/fixtures/` (to be created)

Example fixtures:
- `fixtures/simple_node.launch.xml` - Single node, no parameters
- `fixtures/node_with_params.launch.xml` - Node with parameters
- `fixtures/composable_nodes.launch.xml` - Container with composables
- `fixtures/nested_includes.launch.xml` - Multi-level includes
- `fixtures/launch_args.launch.py` - Python with DeclareLaunchArgument

---

## Troubleshooting

### Comparison Test Failures

**Symptom**: `compare_parsers.sh` reports output mismatch

**Debug Steps**:
1. Check detailed diff output from script
2. Run comparison utility with manual inspection:
   ```bash
   python3 scripts/compare_records.py record_rust.json record_python.json
   ```
3. Manually inspect JSON files:
   ```bash
   diff -u <(jq -S . record_python.json) <(jq -S . record_rust.json)
   ```
4. Check for known differences:
   - Parameter type representation (string vs number)
   - Array ordering
   - Null vs omitted fields

### Performance Test Failures

**Symptom**: Benchmark shows <5x speedup

**Debug Steps**:
1. Check system load: `top`, `htop`
2. Run with more iterations: `ITERATIONS=10 ./scripts/benchmark_parsers.sh`
3. Profile Rust parser: `cargo flamegraph`
4. Check for I/O bottlenecks (disk speed, network mounts)
5. Verify parser submodule is up-to-date:
   ```bash
   git submodule status
   git submodule update --remote
   ```

### Integration Test Failures

**Symptom**: `cargo test` fails

**Debug Steps**:
1. Run specific test with output:
   ```bash
   cargo test parser_integration -- --nocapture
   ```
2. Check ROS environment is sourced:
   ```bash
   echo $AMENT_PREFIX_PATH
   ```
3. Verify test fixtures exist:
   ```bash
   ls -la tests/fixtures/
   ```

---

## Adding New Tests

### Adding a Comparison Test

1. Add test case to `scripts/compare_parsers.sh`:
   ```bash
   TEST_CASES+=(
       "my_package:my_launch.xml:My Test Description"
   )
   ```

2. Ensure launch file is accessible via ROS package or absolute path

3. Run test suite: `./scripts/compare_parsers.sh`

### Adding a Benchmark Test

1. Add test case to `scripts/benchmark_parsers.sh`:
   ```bash
   TEST_CASES+=(
       "my_package:my_launch.xml:MyBenchmark"
   )
   ```

2. Run benchmark: `./scripts/benchmark_parsers.sh`

3. Review results in output and timestamped results file

### Adding an Integration Test

1. Create test in `tests/parser_integration_test.rs`:
   ```rust
   #[test]
   fn test_my_feature() {
       // Test implementation
   }
   ```

2. Add test fixture if needed in `tests/fixtures/`

3. Run test: `cargo test test_my_feature`

---

## Reporting Issues

When reporting parser-related issues, include:

1. **Launch file**: Minimal reproducible example
2. **Command**: Exact `play_launch` command used
3. **Output**: Both Rust and Python parser outputs (`record.json`)
4. **Diff**: Output from `compare_records.py`
5. **Environment**:
   - ROS distribution (Humble/Jazzy)
   - Python version
   - Rust version
   - OS and version
6. **Error messages**: Full error output from both parsers

**Example**:
```
Title: Rust parser fails on YAML include with nested variables

Launch file: test/my_test.launch.xml (attached)
Command: play_launch launch test_pkg my_test.launch.xml var:=value

Rust parser error:
  Parser error: Variable 'nested_var' not found in context

Python parser: Success (record.json attached)

Environment:
- ROS 2 Humble
- Python 3.10.12
- Rust 1.75.0
- Ubuntu 22.04

Diff: (see compare_records.py output below)
```

---

## References

- [Phase 13: Rust Parser Migration](./phase-13.md)
- [Migration Plan](../../tmp/MIGRATION_PLAN_RUST_PARSER.md)
- [play_launch_parser Documentation](../../src/play_launch_parser/README.md)
- [Parser Test Suite](../../src/play_launch_parser/tests/)

---

**Last Updated**: 2026-01-25
**Phase**: 13 (Planned)
