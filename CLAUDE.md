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
5. Verify all tests pass (413 tests expected)

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

4. **Test After Each Change**: All 413 tests must pass
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

### Test Organization

**Test Structure** (Session 12 - Reorganized):

Test files are organized by category in `src/play_launch_parser/tests/`:

```
src/play_launch_parser/tests/
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

**Test Fixtures Location**: `src/play_launch_parser/tests/fixtures/`
- Helper functions use `env!("CARGO_MANIFEST_DIR")/tests/fixtures/launch`
- Fixtures are self-contained within the crate

**Total Test Count**: 413 tests (237 unit + 18 edge + 25 XML + 50 YAML + 38 Python + 22 IR + 20 IR-eval + 3 performance)

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
# Run ALL tests (Rust + comparison + Autoware if available)
just test

# Run only Rust unit tests (413 tests)
just test-rust

# Run comparison tests (Rust vs Python parser)
just test-compare

# Run Autoware validation tests
just test-autoware

# Run colcon tests (ROS 2 integration tests)
just test-colcon

# Run all quality checks (linters + Rust tests)
just quality

# Run linters and formatters
just check

# Format code
just format

# Build the project
just build

# Clean artifacts
just clean
```

**Test Organization**:
- `just test` - Runs ALL tests in the project (recommended for CI/validation)
- `just test-rust` - Fast Rust unit tests only (recommended during development)
- `just test-compare` - Comparison tests between Rust and Python parsers
- `just test-autoware` - Full Autoware validation (requires Autoware symlink)
- `just quality` - Linters + Rust tests (recommended before commits)

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
- **Test Coverage**: 413 tests passing (~95% code coverage)
- **Feature Completion**: 95% (Phase 5 complete, Phase 6 planned, **Phase 7 roadmap ready**)
- **Autoware Compatibility**: ✅ **100%** - Full planning_simulator test passes (46 nodes, 15 containers, 54 composable nodes)
- **Performance**: ~5s to parse full Autoware launch tree (vs ~10-15s with Python)
  - **Phase 7 Target**: <1s (5-7x improvement planned)

### Current Capabilities

- ✅ Complete XML launch file parsing
- ✅ All core substitutions (`$(var)`, `$(env)`, `$(find-pkg-share)`, etc.)
- ✅ Eval expressions (arithmetic & string comparisons)
- ✅ Python launch file support (complete core + extended API)
- ✅ Container and composable node support (XML + Python)
- ✅ YAML launch file support
- ✅ Boolean attribute substitutions (respawn, respawn_delay)
- ✅ Python API: launch, launch_ros, launch_xml, launch.frontend, launch.utilities
- ✅ Autoware production workload validated

### Next Phase: Performance Optimization (Phase 7)

**Status**: 📋 Planned (comprehensive roadmap created in Session 12)

**Goal**: Achieve **5-7x performance improvement** for complex launch files

**Current Performance**: ~5s for Autoware planning_simulator.launch.xml
**Target Performance**: <1s (5-7x improvement)

**Three-Phase Approach**:

1. **Phase 7.1: DashMap Caching** (2-3 days, Low Risk, **Highest Impact**)
   - Package resolution cache: 80-95% improvement for lookups
   - File content cache: 30-50% improvement for repeated includes
   - Python mutex upgrade (parking_lot): 15-25% improvement
   - **Expected**: Autoware ~5s → ~3s (60-80% improvement)

2. **Phase 7.2: Hybrid Arc + Local Context** (1 week, Medium Risk, **Enables Parallelization**)
   - Standard compiler pattern (used by V8, Python, Rust compiler, LLVM)
   - Parent scope frozen in `Arc<ParentScope>`, child has local HashMap
   - 500-1000x faster context creation (O(1) vs O(n))
   - **Expected**: Autoware ~3s → ~2s (20-40% improvement + thread-safe for Phase 7.3)

3. **Phase 7.3: rayon Parallel Processing** (2-3 days, Low Risk, **Final Multiplier**)
   - Work-stealing threadpool (NOT async, NOT manual task queue)
   - Change `.iter()` to `.par_iter()` for include processing
   - rayon implements dynamic task spawning automatically
   - **Expected**: Autoware ~2s → ~1s (2.1x improvement on 8 cores)

**Why These Approaches**:
- **DashMap over LRU+RwLock**: 7.5x vs 1.3x speedup on 8 threads, lock-free reads
- **Hybrid context over simple Arc or CoW**: Standard compiler pattern, natural scope semantics, enables parallelization
- **rayon over async**: Workload is 80%+ CPU-bound after caching, no tokio overhead, proven and simple

**Key Dependencies**:
```toml
dashmap = "5.5"           # Concurrent HashMap
parking_lot = "0.12"      # Faster mutexes
rayon = "1.8"             # Parallel processing
lru = "0.12"              # LRU cache (optional)
```

**Analysis Documents** (created Session 12, in `/tmp/`):
- `optimization_opportunities.md` - Complete 8-category analysis (800+ lines)
- `context_cloning_best_practices.md` - 4 compiler patterns compared
- `parallelism_strategy_analysis.md` - Async vs rayon vs task queue
- `cache_strategy_dashmap.md` - DashMap vs LRU detailed comparison

### Detailed Documentation

For detailed information, see:
- **Feature tracking**: `docs/feature_list.md`
- **Implementation status**: `docs/roadmap/implementation_status.md`
- **Phase 5 roadmap**: `docs/roadmap/phase-5-python_support.md`
- **Phase 7 roadmap**: `docs/roadmap/phase-7-performance_optimization.md` ⭐
- **Test organization**: `tests/README.md`

### Quick Reference

**Test Count Baseline**: 413 tests (237 unit + 18 edge + 25 XML + 50 YAML + 38 Python + 22 IR + 20 IR-eval + 3 performance)

**Key Architectural Notes**:
- Substitution system uses recursive `Vec<Substitution>` for nesting
- Parser uses character-by-character parsing with depth counting
- Include arguments use `Vec` (not `HashMap`) to preserve order
- Python API uses capture-on-construction pattern

See implementation docs for detailed architecture and design decisions.

## Recent Session Work (Latest)

### Session 12: Test Organization, Documentation & Optimization Planning ✅

**Work Completed**:

**Part 1: Test Organization & Documentation**
1. ✅ Cleaned up debug code - Converted all `eprintln!` to structured logging (`log::trace!`, `log::debug!`, `log::error!`)
2. ✅ Fixed test script bugs - `compare_rust_python.py` now correctly validates all metrics including node count adjustments for containers
3. ✅ Created comprehensive edge case tests - 4 new fixtures testing Autoware-derived patterns (OpaqueFunction conditional logic, list concatenation, ParameterFile, nested substitutions)
4. ✅ Reorganized test structure:
   - Moved fixtures from `/tests/fixtures` to `/src/play_launch_parser/tests/fixtures`
   - Split 2191-line `integration_tests.rs` into 3 focused files:
     - `xml_tests.rs` (20 tests, 36KB)
     - `python_tests.rs` (15 tests, 35KB)
     - `integration_tests.rs` (3 performance tests, 3.5KB)
5. ✅ Fixed Python test race conditions - Added `python_test_guard()` to serialize Python tests and prevent global state contamination
6. ✅ Updated documentation:
   - `docs/feature_list.md` - Added edge case testing section, test coverage matrix, and Autoware validation results
   - Test counts updated to 260 total tests
   - Rewrote `README.md` in minimalist style (266 lines → 70 lines)
   - Revised justfile test recipes for better workflow

**Part 2: Performance Optimization Analysis**
7. ✅ Comprehensive optimization analysis - Identified 8 major categories with 5-7x total improvement potential
8. ✅ Created Phase 7 roadmap - Detailed 3-4 week implementation plan with code examples and success criteria
9. ✅ Context cloning best practices - Analyzed 4 compiler approaches, recommended Hybrid Arc + Local pattern
10. ✅ Parallelism strategy analysis - Compared async vs rayon vs manual task queue, recommended rayon (Approach 2)
11. ✅ Cache strategy documentation - DashMap vs LRU analysis for different cache types

**Key Technical Decisions**:
- **Logging**: All debug output now uses proper log levels for RUST_LOG control
- **Test Serialization**: Python tests run sequentially to avoid race conditions in global capture storage
- **Test Organization**: Logical separation of XML, Python, and performance tests
- **Context Pattern**: Hybrid Arc + Local (used by V8, Python, Rust compiler, LLVM) - 500-1000x faster child creation
- **Parallelization**: rayon threadpool (NOT async) - 80%+ CPU-bound workload, work-stealing queue
- **Caching**: DashMap for bounded caches (packages, files, commands), thread_local LRU for unbounded (substitutions)

**Documentation Created**:
- `/tmp/test_coverage_update.md` - Comprehensive test coverage documentation
- `/tmp/optimization_opportunities.md` - Complete 8-category optimization analysis (800+ lines)
- `/tmp/context_cloning_best_practices.md` - Compiler pattern analysis (4 approaches)
- `/tmp/parallelism_strategy_analysis.md` - Async vs rayon vs task queue comparison
- `/tmp/cache_strategy_dashmap.md` - DashMap vs LRU detailed comparison
- `/tmp/package_resolution_context_analysis.md` - Context awareness analysis
- `docs/roadmap/phase-7-performance_optimization.md` - Implementation roadmap with tasks and code examples
- Edge case test fixtures validating Autoware patterns

**Validation**:
- ✅ 413 tests passing (218 unit + 18 edge + 20 XML + 15 Python + 3 performance + 6 base)
- ✅ 100% Autoware compatibility maintained (46 nodes, 15 containers, 54 composable nodes)
- ✅ Zero clippy warnings
- ✅ All quality checks passing

**Next Phase Preview**: Phase 7 Performance Optimization
- **Goal**: 5-7x improvement (Autoware: ~5s → ~0.7-1s)
- **Phase 7.1**: DashMap caching (60-80% improvement in 2-3 days)
- **Phase 7.2**: Hybrid Arc + Local context (20-40% improvement in 1 week)
- **Phase 7.3**: rayon parallelization (2.1x improvement in 2-3 days)
- **Phase 7.4**: Additional optimizations (+10-20% in 1 week)

## ROS 2 Include Scoping Behavior (Session 13 Findings)

### Critical Discovery: YAML vs XML Include Semantics

**Problem Identified**: Parallel include processing broke YAML preset files in Autoware.

**Root Cause**: ROS 2 has different scoping semantics for YAML vs XML includes:

#### XML Includes (Isolated Scope)
- Create child context: `include_context = self.context.child()`
- Variables declared inside stay in child scope
- Context changes are discarded after processing
- **Safe for parallel processing** ✅

#### YAML Includes (Parent Scope Modification)
- Modify parent context directly: `self.process_yaml_launch_file()`
- Variables declared are visible to subsequent includes
- Used for preset files that declare configuration variables
- **Must be processed sequentially** ⚠️

### Real-World Example from Autoware

```xml
<!-- tier4_planning_component.launch.xml -->
<include file=".../preset/default_preset.yaml"/>  <!-- Declares velocity_smoother_type -->

<include file=".../planning.launch.xml">
  <arg name="param" value="...$(var velocity_smoother_type)..."/>  <!-- Uses it -->
</include>
```

If processed in parallel:
- YAML modifies clone context (discarded) ✗
- XML sees original context (missing variable) ✗
- Result: "Undefined variable: velocity_smoother_type"

If processed sequentially:
- YAML modifies self.context ✓
- XML sees updated self.context ✓
- Result: Variable resolved correctly

### Implementation Solution

**Code Change** (src/lib.rs:624-680):
- Detect YAML includes during batch collection
- Process any collected XML includes in parallel
- Process YAML include sequentially
- Break batching to restart after YAML

**Result**:
- XML includes: parallel for performance
- YAML includes: sequential for correctness
- Maintains both performance and correctness

### ROS 2 Documentation Analysis

**Official Docs Reviewed**:
- [ROS 2 Launch XML Format](https://design.ros2.org/articles/roslaunch_xml.html)
- [ROS 2 Launch System](https://design.ros2.org/articles/roslaunch.html)
- [Using Python, XML, and YAML for ROS 2 Launch Files](https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html)

**Key Findings**:
1. XML format doc: "included launch file description has its own scope"
2. Main launch doc: "Changes to the local state by included launch descriptions persist"
3. **YAML scoping behavior is completely undocumented** ❌

**Observed Behavior** (reverse-engineered from Autoware):
- YAML preset files modify parent context
- Subsequent includes see YAML-declared variables
- This is critical for Autoware's preset system to work

**Documentation Created**:
- `docs/include_scoping_behavior.md` - Comprehensive analysis with sources

### Key Takeaway for Development

When implementing include processing:
1. ✅ XML includes can be parallelized safely
2. ⚠️ YAML includes must respect execution order
3. ⚠️ Include arg values may reference variables from previous YAML includes
4. ✅ Don't assume all includes can be parallelized - check file type

This behavior is essential for Autoware compatibility and likely affects other large ROS 2 projects using preset files.
