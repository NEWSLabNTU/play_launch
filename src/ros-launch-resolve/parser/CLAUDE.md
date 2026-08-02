# Claude Development Guidelines

This document contains essential practices and guidelines for Claude (AI assistant) when working on this project.

## Quality Assurance

### Always Run Quality Checks Before Finishing

**CRITICAL PRACTICE**: Before marking any task as complete, ALWAYS run quality checks and fix errors.

**Quick Command**:
```bash
just quality
```

This runs:
1. `just check` - Linters and formatters (clippy, rustfmt)
2. `just test-rust` - All Rust unit tests

**Required Steps**:
1. Run clippy: `cargo clippy --all-targets --all-features -- -D warnings`
2. Fix ALL warnings and errors (no exceptions)
3. Check formatting: `cargo fmt -- --check`
4. Run all tests: `cargo test --all`
5. Verify all tests pass (371 tests expected without IR; 413 with `--features ir`)

**Never**:
- Mark a task complete with failing tests
- Leave clippy warnings unfixed
- Skip quality checks "for later"
- Commit code that doesn't compile

## Development Workflow

### File Creation Best Practices

**CRITICAL**: Always use Write tool for creating files, never use cat with heredoc pattern.

**Correct**:
```rust
Write tool with file_path and content parameters
```

**Incorrect**:
```bash
cat > file.txt << 'EOF'
content
EOF
```

**Reasons**:
- Write tool is clearer and more reliable
- Avoids shell escaping issues
- Better error handling
- Consistent with other file operations

### Python Support (Mandatory)

**Python support is now mandatory** (as of Session 13):
- pyo3 is a required dependency (no longer optional)
- All `#[cfg(feature = "python")]` gates have been removed
- Python launch file support is always enabled
- No need to build with `--features python`

**Why Python is mandatory**:
- Required for 100% Autoware compatibility (46 nodes vs 31 without Python)
- Many ROS 2 packages use Python launch files
- Python files contain critical nodes (ADAPI, MRM operators, control nodes)
- Python includes are processed and tracked just like XML includes

**Build commands** (Python automatically included):
```bash
cargo build --profile dev-release
just build-rust
just test
```

### When Making Changes

1. **Read files first**: Always use the Read tool before modifying
2. **Run tests after changes**: After modifications, run `cargo test` or `just test-rust`
3. **Fix all errors**: Never leave compilation errors or failing tests
4. **Update tests**: When changing functionality, update relevant tests
5. **Add tests for new features**: New functionality requires comprehensive test coverage

### Test-Driven Development

Write tests covering:
- Happy path (normal operation)
- Edge cases (boundary conditions)
- Error cases (invalid input, failures)
- Integration scenarios (features working together)

### Code Quality Standards

- **No warnings**: Fix all clippy warnings
- **Formatted code**: Always format with `cargo fmt`
- **No unused code**: Remove unused imports, variables, functions
- **Clear naming**: Use descriptive names
- **Documentation**: Add doc comments for public APIs

### Optimization Best Practices

**When implementing Phase 7 optimizations**:

1. **Measure First**: Benchmark before optimizing to establish baseline
   - Use `hyperfine` for timing
   - Use `heaptrack` for memory profiling
   - Use `cargo flamegraph` for CPU profiling

2. **Follow the Roadmap**: `docs/roadmap/phase-7-performance_optimization.md` has detailed implementation steps
   - Each phase has specific tasks with checkboxes
   - Full code examples provided
   - Success criteria defined

3. **Incremental Implementation**: Implement one phase at a time
   - Phase 7.1 (caching) before Phase 7.2 (context refactoring)
   - Phase 7.2 required before Phase 7.3 (parallelization)
   - Run benchmarks after each phase

4. **Test After Each Change**: All 371 tests must pass (413 with `--features ir`)
   - Optimizations should not change behavior
   - Use regression tests to validate output matches

5. **Reference Analysis Documents**: Detailed analysis in `/tmp/`
   - `optimization_opportunities.md` - Why each optimization works
   - `context_cloning_best_practices.md` - Compiler pattern details
   - `parallelism_strategy_analysis.md` - Why rayon over async
   - `cache_strategy_dashmap.md` - Why DashMap over LRU

6. **Standard Patterns**: Use well-known compiler/systems patterns
   - Context: Hybrid Arc + Local (V8, Python, Rust compiler, LLVM)
   - Parallelism: rayon work-stealing (Rust compiler, ripgrep, fd)
   - Caching: DashMap for bounded, thread_local LRU for unbounded

## Project-Specific Practices

### Temporary Files and Scripts

**IMPORTANT**: Always create temporary files and scripts in `$project/tmp/` directory instead of `/tmp/`

- **Location**: `$project/tmp/` (i.e., `/home/aeon/repos/play_launch_parser/tmp/`)
- **Create if needed**: `mkdir -p tmp/`
- **Clean up**: Remove when done if appropriate
- **Applies to**: Debug files, analysis scripts, test outputs, comparison tools, etc.

Examples:
- Temporary data: `tmp/debug_output.json`, `tmp/test_results.txt`
- Analysis scripts: `tmp/compare_outputs.py`, `tmp/analyze_nodes.sh`
- Test data: `tmp/sample_launch.xml`, `tmp/captured_data.md`

### File Creation

**CRITICAL RULE**: Always use the `Write` tool for creating files. Never use Bash commands.

Rules:
- ✅ **Correct**: Use `Write` tool for ALL file creation
- ❌ **Wrong**: `cat > file << 'EOF'`, `echo > file`, Bash redirection
- ✅ **Correct**: Use `Edit` tool for modifying existing files
- ❌ **Wrong**: `sed`, `awk`, or any Bash text processing

**Why this matters**:
- Write tool has better error handling
- Write tool requires reading file first (prevents accidental overwrites)
- Write tool is explicit and clear in the tool call log
- Bash heredoc can have quoting/escaping issues

**No exceptions**: Use Write even for:
- Long files (100+ lines)
- Files with complex content
- Multiple files in sequence
- Temporary test scripts

### Crate Layout

```
play_launch_parser/
├── crates/
│   ├── play_launch_parser/     # Rust library + CLI binary
│   │   ├── src/lib.rs          # pub fn parse_launch_file(...)
│   │   └── src/main.rs         # CLI: play_launch_parser launch|file
│   └── python/                 # PyO3 extension module (excluded from workspace)
│       ├── Cargo.toml          # cdylib, own [workspace], pyo3 extension-module
│       ├── src/lib.rs          # #[pymodule] + _cli_main()
│       ├── pyproject.toml      # maturin build config
│       ├── play_launch_parser/ # Python source (__init__.py, _cli.py, stubs)
│       └── examples/           # Usage examples
├── src/python_dump/            # Python fallback parser (not a Rust crate)
├── Cargo.toml                  # workspace: members = [crates/play_launch_parser]
│                               # exclude = [crates/python]
└── justfile
```

**Key**: The `crates/python/` extension crate is **excluded** from the parser workspace because PyO3 `extension-module` and `auto-initialize` are mutually exclusive features. It has its own `[workspace]` marker and is built independently by maturin.

### Test Organization

Test files are organized by category in `crates/play_launch_parser/tests/`:

```
crates/play_launch_parser/tests/
├── edge_cases.rs           # Edge case tests (18 tests)
├── xml_tests.rs            # XML parsing tests (25 tests)
├── yaml_tests.rs           # YAML launch file tests (50 tests)
├── python_tests.rs         # Python launch tests (38 tests)
├── ir_tests.rs             # IR builder tests (22 tests)
├── ir_eval_tests.rs        # IR evaluation tests (20 tests)
├── integration_tests.rs    # Performance tests (3 tests)
└── fixtures/
    ├── launch/             # Main test launch files
    └── includes/           # Files to be included by tests
```

**Test Fixtures Location**: `crates/play_launch_parser/tests/fixtures/`
- Helper functions use `env!("CARGO_MANIFEST_DIR")/tests/fixtures/launch`
- Fixtures are self-contained within the crate

**Total Test Count**: 413 tests with `--features ir` (371 without IR).

### Python Test Requirements

**CRITICAL**: Python tests MUST use serialization to prevent race conditions.

**Problem**: Python's global interpreter state causes test contamination when tests run in parallel.

**Solution**: All Python tests must acquire `python_test_guard()` at the start:

```rust
#[test]
#[cfg(feature = "python")]
fn test_my_python_feature() {
    let _guard = python_test_guard();  // ← REQUIRED for all Python tests
    // ... test code ...
}
```

**Why This Matters**:
- Python's global capture storage (CAPTURED_NODES, etc.) is shared across all tests
- Parallel test execution causes data races and false failures
- The guard ensures Python tests run sequentially
- Without the guard, tests will fail intermittently

**When Adding New Python Tests**:
1. Add `#[cfg(feature = "python")]` attribute
2. Add `let _guard = python_test_guard();` as first line in test function
3. Place test in `python_tests.rs` file

See `src/play_launch_parser/tests/python_tests.rs` for examples.

## Common Commands

```bash
# Build everything (Rust + colcon + Python wheel)
just build

# Build Rust only
just build-rust

# Build Python wheel (creates venv in crates/python/.venv if needed)
just build-python

# Run all quality checks (linters + Rust tests)
just quality

# Run only Rust unit tests (413 with --features ir)
just test-rust

# Run Python smoke test
just test-python

# Run ALL tests (Rust + comparison + Autoware if available)
just test

# Run linters and formatters
just check

# Format code
just format

# Clean artifacts
just clean
```

**Build notes**:
- `just build` includes `build-rust build-colcon build-python`
- `just build-python` creates a venv with system Python, installs maturin, builds the wheel
- The wheel path is printed at the end of `build-python`
- The Python venv (`crates/python/.venv`) uses system Python to avoid version mismatches with PyO3's `auto-initialize` in the parser crate

### CLI

Two binaries with identical functionality:

- **Rust binary** (`play_launch_parser`): built by `just build-rust`, lives at `target/dev-release/play_launch_parser`
- **Python entry point** (`play-launch-parser`): installed by `just build-python` into `crates/python/.venv/bin/`

```bash
# Parse by file path (JSON to stdout)
play_launch_parser file /path/to/launch.xml

# Parse by ROS package name
play_launch_parser launch <package> <file>

# Pass launch arguments
play_launch_parser file /path/to/launch.xml vehicle_model:=sample

# Output formats
play_launch_parser file /path/to/launch.xml --format summary   # human-readable table
play_launch_parser file /path/to/launch.xml --format names     # one FQN per line

# Write to file instead of stdout
play_launch_parser file /path/to/launch.xml -o record.json
```

Exit codes: `0` success, `1` parse error, `2` file/package not found.

## Before Completing a Task

Checklist:
- [ ] All code compiles without errors
- [ ] All tests pass (`just test` or `just quality`)
- [ ] Code is formatted (`just format`)
- [ ] No clippy warnings (`just check`)
- [ ] Quality checks pass (`just quality`)
- [ ] New functionality has tests
- [ ] Documentation is updated if needed
- [ ] If working on optimization: Follow Phase 7 roadmap (`docs/roadmap/phase-7-performance_optimization.md`)

## Environment Setup

### direnv Configuration

The project uses `.envrc` to automatically source ROS 2 environment:

```bash
# .envrc sources:
# - /opt/ros/humble/setup.bash (if exists)
# - install/setup.bash (if exists, watched for changes)
```

After creating/modifying `.envrc`, run `direnv allow` to enable it.

## Project Overview

**Goal**: Fast Rust implementation of ROS 2 launch file parser to replace slow Python `dump_launch`

**Current Status**: ✅ **PRODUCTION READY** - Autoware Compatibility Complete
- **Test Coverage**: 413 tests passing (with `--features ir`)
- **Autoware Compatibility**: ✅ **100%** (46 nodes, 15 containers, 54 composable nodes)
- **Python bindings**: ✅ `pip install play-launch-parser` (Phase 9 complete)
- **CLI**: ✅ `play-launch-parser` with `--format json|summary|names`

### Current Capabilities

- ✅ Complete XML / Python / YAML launch file parsing
- ✅ All core substitutions, eval expressions, conditions
- ✅ Container and composable node support (XML + Python)
- ✅ Launch tree scoping (which file defines each node)
- ✅ Python bindings (`parse_file()`, `parse_package()`) via PyO3 + maturin
- ✅ CLI with JSON / summary / names output formats
- ✅ Type stubs (`.pyi`) for IDE autocompletion
- ✅ Autoware production workload validated

### Completed Phases

- **Phase 7**: Performance optimization (DashMap caching, hybrid context, rayon parallelization) — see `docs/roadmap/phase-7-performance_optimization.md`
- **Phase 8**: ROS API completeness (50/56 features, 89% coverage) — see `docs/roadmap/phase-8-ros_api_completeness.md`
- **Phase 9**: Python bindings & CLI (PyO3 + maturin, `parse_file()`/`parse_package()`, `--format json|summary|names`, type stubs) — see `docs/roadmap/phase-9-python_bindings_and_cli.md`

### Detailed Documentation

For detailed information, see:
- **Feature tracking**: `docs/feature_list.md`
- **Implementation status**: `docs/roadmap/implementation_status.md`
- **Python bindings & CLI**: `docs/roadmap/phase-9-python_bindings_and_cli.md`
- **Phase 7 roadmap**: `docs/roadmap/phase-7-performance_optimization.md`

### Key Architectural Notes

- Substitution system uses recursive `Vec<Substitution>` for nesting
- Parser uses character-by-character parsing with depth counting
- Include arguments use `Vec` (not `HashMap`) to preserve order
- Python API uses capture-on-construction pattern
- Python extension crate (`crates/python/`) is excluded from workspace — PyO3 `extension-module` vs `auto-initialize` conflict
- The extension crate's venv must use system Python (same version PyO3 links against in the parser crate)

## ROS 2 Include Scoping Behavior

**Critical**: YAML and XML includes have different scoping semantics:
- **XML includes**: isolated child scope (safe for parallel processing)
- **YAML includes**: modify parent scope directly (must be sequential)

This is undocumented in ROS 2 but essential for Autoware's preset system. See `docs/include_scoping_behavior.md`.
