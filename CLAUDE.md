# CLAUDE.md

Guide for Claude Code when working with this repository.

## Project Overview

ROS2 Launch Inspection Tool - Records and replays ROS 2 launch executions for performance analysis:
- **play_launch_parser** (Rust): Parses launch files to `record.json` (default, 3-12x faster)
- **dump_launch** (Python): Alternative parser for maximum compatibility
- **play_launch** (Rust): Replays with resource monitoring
- **play_launch_analyzer** (Python): Analyzes and visualizes logs

## Launch File Parsing

**Default**: Rust parser (`play_launch_parser` library)
- **Performance**: 3-12x faster than Python parser
- **Formats**: XML, YAML, Python launch files
- **Compatibility**: 100% Autoware tested (311 parser unit tests)
- **Behavior**: Fails immediately on error (no automatic fallback)
- **Known Issues**: Minor discrepancies in exec_name, global parameters, and params_files (Phase 17 in progress)

**Alternative**: Python parser (for maximum compatibility)
```bash
play_launch launch <package> <file> --parser python
play_launch dump launch <package> <file> --parser python
```

### Performance Comparison

| Test Case | Rust Parser | Python Parser | Speedup |
|-----------|-------------|---------------|---------|
| Simple nodes (pure_nodes.launch.xml) | 0.133s | 0.390s | 2.93x |
| Composable nodes (simple_test.launch.xml) | 0.136s | 1.570s | 11.54x |

**Benchmark command**:
```bash
just benchmark-parsers 5  # Run 5 iterations
```

### When to Use Python Parser

Use `--parser python` when:
- Rust parser fails with a parsing error
- Testing compatibility between parsers
- Debugging launch file issues
- Maximum compatibility is required

The Rust parser is the default and recommended for all normal use cases.

### Python Parser as Ground Truth

**CRITICAL RULE**: When Rust and Python parser behaviors differ, **Python's behavior is always correct**.

- Python parser (`dump_launch`) is the authoritative reference implementation
- Rust parser must match Python's output exactly
- If a test expects different behavior than Python produces, **the test is wrong**
- Always validate Rust parser changes against Python parser output on real launch files

**Validation workflow**:
1. Parse with Python: `play_launch dump launch <package> <file> --parser python -o record_python.json`
2. Parse with Rust: `play_launch dump launch <package> <file> --parser rust -o record_rust.json`
3. Compare outputs: Check that Rust matches Python for all fields (node names, parameters, container targets, etc.)
4. If Rust differs: Fix Rust, not Python

**Example cases**:
- Substitution resolution in `target_container_name`: Python resolves to actual container names (e.g., `/pointcloud_container`), so Rust must too
- Namespace handling: Python's namespace concatenation rules are authoritative
- Parameter type preservation: Match Python's type conversions (int vs float)

### Parser Architecture: Conditional Evaluation

**Important**: The parser does not process every conditional path. Instead, it:
1. **Evaluates conditions during parsing** using real LaunchConfiguration values
2. **Parses only the selected path** based on the condition result
3. **Preserves LaunchConfiguration substitutions** as `$(var name)` for later resolution

**Implementation Details** (`src/python/api/utils.rs`):
- **Evaluating substitutions** (call `perform()` with real LaunchContext):
  - Conditional: `EqualsSubstitution`, `IfElseSubstitution`, `NotEqualsSubstitution`, `AndSubstitution`, `OrSubstitution`, `NotSubstitution`
  - Content: `FileContent` (reads files, resolves nested substitutions), `PathJoinSubstitution` (joins paths, resolves nested)
  - Expression: `PythonExpression` (evaluates Python code like `'true' if 'offline' == 'realtime' else 'false'`)
- **LaunchConfiguration substitutions** call `__str__()` to preserve as `$(var name)` for replay-time resolution
- **Other substitutions** call `__str__()` to preserve their format
- **Float parameters** always include decimal point (e.g., `0.0` not `0`) to preserve type information for ROS

This design allows:
- Conditional expressions to evaluate correctly and select the right execution path
- Python expressions in parameters to evaluate instead of being passed as raw strings
- LaunchConfiguration values to be re-resolved at replay time with different values
- Complex nested conditionals to work properly
- ROS parameter type checking to work correctly (INTEGER vs DOUBLE distinction preserved)

**Examples**:
```python
# Example 1: IfElseSubstitution in namespace
# With cli_args: mode='debug'
namespace = IfElseSubstitution(
    EqualsSubstitution(LaunchConfiguration('mode'), 'debug'),
    '/debug_ns',    # Parser takes this path (condition evaluates to true)
    '/release_ns'   # This path is not processed
)

# Example 2: PythonExpression in parameter
# With cli_args: mode='offline'
use_sensor_data_qos = PythonExpression([
    "'true' if '", LaunchConfiguration('mode'), "' == 'realtime' else 'false'"
])
# Parser evaluates to: "false" (not the raw Python expression string)

# Example 3: Float parameter preservation
parameters=[{'sync_tolerance_ms': 0.0}]
# Parser outputs: "0.0" (not "0") to preserve DOUBLE type for ROS
```

### `<let>` Statement Temporal Semantics

**Critical**: `<let>` statements use **sequential parse-time resolution** with proper temporal ordering:

1. **Immediate Context Update**: When `<let name="var" value="new_value"/>` is encountered, the context is updated immediately
2. **Subsequent Resolution**: Any `$(var var)` substitution after the `<let>` sees the new value
3. **Pre-Resolved Storage**: Values are resolved at parse time and stored in record.json
4. **Runtime Usage**: Runtime uses pre-resolved values directly (no variable lookup needed)

**Example**:
```xml
<arg name="x" default="initial"/>
<node name="n1"><param value="$(var x)"/></node>  <!-- Gets "initial" -->
<let name="x" value="changed"/>
<node name="n2"><param value="$(var x)"/></node>  <!-- Gets "changed" -->
```

**Test Coverage**: `test_let_ordering.launch.xml` validates this behavior matches Python parser exactly.

**Why Parse-Time**: Runtime resolution would lose temporal ordering (all nodes would get final value).

### Runtime Substitution Resolution (Fallback)

**Problem**: Python launch files using `LaunchConfiguration("executable")` may result in unresolved substitutions like `$(var container_executable)` in the record.json when the variable is defined after the node is parsed.

**Solution**: Runtime fallback resolution in `play_launch` replay (src/execution/node_cmdline.rs):
```rust
// Resolve any substitutions in the executable name before ament index lookup
let resolved_executable = substitute_variables(executable, variables);
let exe_path = find_executable(package, &resolved_executable)?;
```

**How It Works**:
1. Parser stores: `executable: "$(var container_executable)"` + `variables: {"container_executable": "component_container"}`
2. Runtime resolves: `$(var container_executable)` → `"component_container"` using variables from record.json
3. Ament index lookup: `find_executable("rclcpp_components", "component_container")` succeeds

**Coverage**: Currently applied to executable names. Can be extended to other fields if needed (parameters, remaps, etc.).

**Backward Compatible**: Only adds capability, doesn't change existing behavior.

## Installation & Usage

```sh
# Install from PyPI (requires ROS2 Humble)
pip install play_launch

# Or build from source
just build                          # Build wheel
just run launch <pkg> <launch_file> # Run with colcon build
play_launch launch <pkg> <launch>   # Run if installed via pip

# Optional: I/O monitoring (requires sudo)
just setcap-io-helper

# Analysis
play_launch plot
```

## Architecture

**Execution Flow:**
1. Load `record.json`, copy parameter files
2. Classify nodes (containers vs regular)
3. Spawn async background tasks (monitoring, service discovery, web UI)
4. Actor-based lifecycle management:
   - Regular nodes and containers: separate actors
   - Composable nodes: virtual members managed by parent containers
5. Logs saved to `play_log/<timestamp>/`

**Key Concepts:**
- **Async/Tokio**: All background services run as async tasks
- **Actor Pattern**: Self-contained lifecycle management with respawn support
- **Virtual Members**: Composable nodes managed as internal state by container actors (LoadNode/Unload operations)
- **ListNodes Verification**: Automatic verification if LoadNode calls timeout (>30s)

## Performance Characteristics

play_launch is designed to be lightweight, with Phase 1 optimizations (2026-01-18) achieving:
- **CPU**: ~19% on large deployments (115 nodes) = 0.17% per managed node
- **Memory**: ~116 MB RSS
- **Threads**: ~60 total
  - 8-19 async worker threads (adaptive tokio pool)
  - ~40 ROS/DDS threads
  - ~2 other threads

**Tuning Options:**
- `--monitor-interval-ms <MS>`: Adjust sampling frequency (default: 2000ms)
- `--disable-monitoring`: Disable resource monitoring entirely (~5% CPU savings)
- `--disable-diagnostics`: Disable diagnostic subscriptions (~2% CPU savings)
- `--disable-all`: Disable all features for minimal overhead

**Technical Details:**
- Tokio runtime: 8 worker threads, 16 max blocking threads
- ROS executor: 50ms spin interval (acceptable latency for management operations)
- Monitoring interval: 2000ms default (configurable via CLI or config file)

## Configuration

**Key CLI Flags:**
- `--config <PATH>`: Runtime config YAML
- `--enable <FEATURE>`: Enable only specific features (can be repeated)
  - Values: `monitoring`, `diagnostics`, `web-ui`
  - Conflicts with `--disable-*` flags
- `--disable-monitoring`: Disable resource monitoring (enabled by default)
- `--disable-diagnostics`: Disable diagnostic monitoring (enabled by default)
- `--disable-web-ui`: Disable web UI (enabled by default)
- `--disable-all`: Disable all features at once
- `--web-addr <IP:PORT>`: Web UI address (default: 127.0.0.1:8080)
- `--disable-respawn`: Disable automatic respawn

**Default Behavior** (as of 2026-01-18):
All features are **enabled by default** for better out-of-box experience:
- Resource monitoring: ✅ enabled
- Diagnostic monitoring: ✅ enabled
- Web UI: ✅ enabled (http://127.0.0.1:8080)

**Config YAML (see `test/autoware/autoware_config.yaml`):**
```yaml
composable_node_loading:
  load_node_timeout_millis: 30000
  max_concurrent_load_node_spawn: 10

list_nodes:
  loading_timeout_secs: 30    # Trigger verification after timeout
  rate_limit_secs: 5          # Min time between queries per container

monitoring:
  enabled: false
  sample_interval_ms: 2000  # Default: 2000ms (reduced from 1000ms for lower CPU overhead)

diagnostics:
  enabled: false               # Enable ROS2 diagnostic monitoring
  topics:                      # Topics to subscribe to
    - /diagnostics
    - /diagnostics_agg
  debounce_ms: 100             # Min time between diagnostic updates
  filter_hardware_ids: []      # Filter by hardware_id (empty = all)

processes:
  - node_pattern: "NODE 'rclcpp_components/component_container*"
    cpu_affinity: [0, 1]
    nice: 5
```

## Log Directory Structure

```
play_log/
├── latest -> 2025-12-21_09-44-52/  # Symlink to most recent
└── 2025-12-21_09-44-52/
    ├── params_files/
    ├── system_stats.csv           # System-wide metrics
    ├── diagnostics.csv            # ROS2 diagnostic messages (when enabled)
    └── node/<node_name>/          # Flat structure (no composable subdirs)
        ├── metadata.json          # For containers: includes composable_nodes array
        ├── metrics.csv            # Per-process metrics (when enabled)
        ├── out/err/pid/status
        └── cmdline
```

**Notes**:
- Composable nodes don't have separate directories. Metadata appears in parent container's `metadata.json`.
- `diagnostics.csv` contains all ROS2 diagnostic messages with columns: timestamp, hardware_id, diagnostic_name, level, level_name, message, key, value (each key-value pair is a separate row).

## Web UI

**Enabled by default** at http://127.0.0.1:8080 (disable with `--disable-web-ui`). Features:
- Two-panel layout with light/dark theme
- Node list with status colors, search/filter, clickable namespaces
- Per-node controls (Start/Stop/Restart) + bulk operations
- Container controls for composable nodes (Load/Unload)
- Real-time log streaming (stdout/stderr) with auto-reconnect
- Stderr activity indicator
- Auto-restart checkbox per node
- **Diagnostics page** (enabled by default, disable with `--disable-diagnostics`):
  - Real-time ROS2 diagnostic status monitoring
  - Sortable table by hardware_id, name, level, or timestamp
  - Color-coded by severity (OK/WARNING/ERROR/STALE)
  - Visual indicators for fresh diagnostics (<10s old)
  - Dashboard badge showing diagnostic counts
  - Auto-refresh every 5 seconds

Security: Binds to `127.0.0.1:8080` by default (localhost only). Use `--web-addr 0.0.0.0:8080` only on trusted networks.

## Development Practices

### Building
- **ALWAYS** use `just build` (NEVER `colcon build` directly)
- After code changes, run `just build` before testing

### Bash Tool Usage
- **ALWAYS** use Bash tool's `timeout` parameter: `Bash(command="...", timeout=15000)`
- **NEVER** prefix commands with `timeout`: ~~`timeout 15 play_launch replay`~~

### Avoiding Orphan Processes
When killing play_launch or ros2 launch:
- **NEVER** use `kill -9` on individual processes
- **ALWAYS** kill the entire process group (PGID):
  ```bash
  PGID=$(ps -o pgid= -p $PID | tr -d ' ')
  kill -TERM -$PGID
  sleep 2
  kill -9 -$PGID
  ```
- Use `just kill-orphans` to clean up stray ROS processes

### Temporary Files and Scripts
- **ALWAYS** store temp files in `/home/aeon/repos/play_launch/tmp/` (gitignored)
- **NEVER** use system `/tmp` for project-related files
- **Create temporary scripts using Write tool**: Use `Write` to create scripts in `tmp/`, never inline Python/shell scripts in Bash commands
- **Script naming**: Use descriptive names like `compare_parameters.py`, `analyze_nodes.sh`

### Logging Practices

**Log Levels:**
- `error!`: Unrecoverable errors
- `warn!`: Recoverable errors/unexpected conditions
- `info!`: User-facing events (startup, shutdown, Start/Stop commands, major lifecycle events)
- `debug!`: Technical details (state transitions, service readiness, timing)

**Default `RUST_LOG=play_launch=info` should show only essential user-facing information.**

Examples:
```rust
// User-facing (info!)
info!("{}: Received Stop command, killing container (PID: {})", name, pid);

// Technical details (debug!)
debug!("{}: LoadNode service available after {}ms", name, elapsed);
debug!("{}: Transitioning {} blocked nodes to Unloaded", name, count);
```

Enable debug logs: `RUST_LOG=play_launch=debug play_launch replay`

## PyO3 and ROS2 Compatibility Notes

### PyO3 0.23 API Changes

When calling Python module-level functions from Rust, use `getattr` + `call0`:

```rust
// CORRECT (PyO3 0.23+)
let main_fn = module.getattr("main")?;
let result = main_fn.call0()?;

// WRONG - call_method0 is for object methods, not module functions
let result = module.call_method0("main")?;  // May cause TypeError
```

### launch_ros Property Access

Some `launch_ros` attributes are properties, not methods:

```python
# CORRECT - expanded_node_namespace is a property
namespace = target_container.expanded_node_namespace

# WRONG - don't call it as a method
namespace = target_container.expanded_node_namespace(context)
```

### Node Name Handling

ROS2 node names may already be fully qualified (with leading `/`). Always check before constructing paths:

```python
# Handle already-qualified node names to avoid double slashes
if node_name and node_name.startswith("/"):
    full_name = node_name  # Already qualified
else:
    full_name = f"{namespace}/{node_name}"
```

### YAML Parameter File Substitution Resolution

**CRITICAL**: YAML parameter files may contain substitutions that must be resolved before passing to ROS nodes.

**Problem**: ROS nodes expect typed parameter values (bool, int, float) but cannot parse launch file substitutions:
```yaml
# BAD - unresolved substitution causes type error
ekf_enabled: $(var ekf_enabled)  # ROS receives string "$(var ekf_enabled)" instead of boolean
```

**Solution**: The parser must resolve all substitutions in YAML file contents and convert to proper types:
```yaml
# GOOD - resolved and typed
ekf_enabled: false  # Boolean, not string
gnss_enabled: true
update_rate: 10.0  # Float
```

**Implementation** (`src/params.rs`):
1. `load_and_resolve_param_file()`: Loads YAML, resolves substitutions, returns resolved YAML string
2. `resolve_yaml_substitutions()`: Recursively walks YAML structure, resolves all `$(...)` patterns
3. `string_to_yaml_value()`: Converts resolved strings to proper YAML types:
   - `"true"` / `"false"` → `Value::Bool`
   - `"123"` → `Value::Number` (i64)
   - `"123.45"` → `Value::Number` (f64)
   - Everything else → `Value::String`

**Usage in generator.rs**:
```rust
// DON'T: Read raw file (leaves substitutions unresolved)
let contents = std::fs::read_to_string(&path)?;

// DO: Load and resolve substitutions
let resolved = load_and_resolve_param_file(Path::new(&path), context)?;
```

**Common errors this fixes**:
- `parameter 'X' has invalid type: ... setting it to {string} is not allowed` (ROS type checking)
- `Failed to parse parameter override rule` (unresolved substitutions in `-p` args)

### Python API Type Handling Pattern (Phase 15)

When implementing ROS 2 launch API classes in PyO3, parameters that accept `SomeSubstitutionsType` in the Python API should accept `PyObject` in our PyO3 bindings:

**ROS 2 Type Definition:**
```python
SomeSubstitutionsType = Union[
    Text,                              # Plain string
    Substitution,                      # LaunchConfiguration, FindPackageShare, etc.
    Iterable[Union[Text, Substitution]], # List of strings and/or substitutions
]
```

**Implementation Pattern:**
```rust
// In struct: store resolved String
pub struct Action {
    param: String,
}

// In constructor: accept PyObject and convert
#[pymethods]
impl Action {
    #[new]
    fn new(py: Python, param: PyObject) -> PyResult<Self> {
        // Use shared utility function
        let param_str = crate::python::api::utils::pyobject_to_string(py, &param)?;

        Ok(Self {
            param: param_str,
        })
    }
}
```

**Shared Helper:** `src/python/api/utils.rs::pyobject_to_string()`
- Handles strings, substitutions (LaunchConfiguration), and lists
- **Conditional substitutions** (EqualsSubstitution, IfElseSubstitution, etc.): Calls `perform()` with real LaunchContext to evaluate to "true"/"false"
- **LaunchConfiguration substitutions**: Calls `__str__()` to preserve as `$(var name)` for replay-time resolution
- **Other substitutions**: Calls `__str__()` to preserve their format
- **Lists**: Recursively concatenates elements
- Used by: SetEnvironmentVariable, AppendEnvironmentVariable, ExecuteProcess, Node.arguments, and others

**When to Use:**
- Any parameter documented in ROS 2 as `SomeSubstitutionsType`
- Parameters that should accept LaunchConfiguration or other substitutions
- Command arrays, paths, environment variables, node arguments, etc.

**Examples:**
```python
# All these should work:
SetEnvironmentVariable('VAR', 'plain_string')
SetEnvironmentVariable('VAR', LaunchConfiguration('var'))
SetEnvironmentVariable('VAR', [LaunchConfiguration('prefix'), '/suffix'])
```

## Key Recent Changes

- **2026-02-07**: Nextest-based integration test infrastructure - Created standalone `play-launch-tests` crate (`tests/`) with `ManagedProcess` RAII guard (setsid + PR_SET_PDEATHSIG + PGID kill on Drop) for guaranteed process cleanup. All process spawning goes through `ManagedProcess` — no raw `.status()` calls. Nextest config with test groups (Python serialization) and timeout overrides. Reorganized justfile: `just test` (~3s, 317 tests), `just test-all` (~30s, 323 tests). Deep Autoware parity comparison via `compare_records.py`.
- **2026-02-06**: Context unification investigation and namespace fix - **Fixed namespace propagation** by identifying and resolving incompatible semantics between LaunchContext (fully-qualified stack) and ParseContext (segment-based stack). Implemented bidirectional namespace synchronization in `lib.rs::execute_python_file()`. Namespace now correct: `/perception/traffic_light_recognition/camera6/classification` ✅. **Architectural issue identified**: Two separate context structures cause synchronization complexity. Created Phase 17 roadmap (`docs/roadmap/phase-17-context_unification.md`) for unifying contexts into single structure. **Remaining discrepancies**: (1) exec_name using node name instead of executable, (2) namespace not used in `__ns:=` cmd argument, (3) global parameters missing in Rust output, (4) params_files mismatch, (5) executable path resolution differences. See Phase 17 roadmap for detailed work items and success criteria.
- **2026-02-06**: Parser comparison and executable path resolution - Modified `./tmp/run_all_checks.sh` to dump and compare both Rust and Python parser outputs in Autoware test stage. Implemented `find_package_executable()` in bridge.rs to resolve full executable paths (searches AMENT_PREFIX_PATH and ROS_DISTRO locations) instead of using `ros2 run` commands. Fixed `exec_name` field to use executable name instead of node name.
- **2026-02-06**: XML composable node YAML parameter loading - Fixed massive parameter loss in composable nodes (behavior_path_planner: 15 → 813 params). Modified `ComposableNodeAction::from_entity` in container.rs to call `extract_params_from_yaml()` when encountering `<param from="file.yaml"/>` elements. Loads and flattens ROS 2 YAML parameter files with `/**` wildcard and `ros__parameters` wrapper support. All three LoadNodeRecord creation paths now merge global parameters correctly. Achieves 100% parameter count parity with Python parser (813 params). Minor formatting differences remain (array spacing, string quoting).
- **2026-02-04**: Composable node namespace normalization fix - Fixed LoadNode service "Couldn't parse remap rule: '-r __ns:=adapi/node'" errors in Autoware by ensuring all composable node namespaces have leading slashes. Modified `ComposableNode::capture_as_load_node` in launch_ros.rs to normalize namespaces for RCL compatibility. Fixed 16 load_node entries (adapi/node → /adapi/node, pointcloud_container → /pointcloud_container, etc.). All namespace-related LoadNode errors resolved, composable nodes now load successfully.
- **2026-02-04**: Debug logging cleanup - Converted all 13 `eprintln!` debug statements in library code to proper `log::debug!` and `log::trace!` calls. Library code now uses structured logging via `RUST_LOG` environment variable. User-facing error messages in main.rs and test debug output preserved. All 310 tests pass.
- **2026-02-04**: Runtime substitution resolution - Fixed pre-existing Autoware container startup issue where Python `LaunchConfiguration` objects created unresolved `$(var container_executable)` substitutions. Added runtime fallback in `node_cmdline.rs` to resolve substitutions before ament index lookup. Simple 3-line fix using existing `substitute_variables()` function. All 15 Autoware containers now start successfully. Zero breaking changes, backward compatible.
- **2026-02-04**: `<let>` statement temporal ordering test - Added comprehensive test fixture (`test_let_ordering.launch.xml`) validating sequential parse-time resolution semantics. Test verifies that `<let>` statements immediately update context and subsequent `$(var)` substitutions see the new value. Confirms both Python and Rust parsers use correct temporal ordering. Documentation created explaining parse-time vs runtime resolution architecture. Test count: 310 (was 308).
- **2026-02-04**: Namespace test fixes - Fixed 4 test assertions (3 in xml_tests.rs, 1 in python_tests.rs) to expect null namespace for root/unspecified instead of "/". Matches actual parser behavior where null indicates root namespace.
- **2026-02-03**: YAML parameter file substitution resolution - Fixed Autoware node failures caused by unresolved substitutions in YAML parameter files (e.g., `ekf_enabled: $(var ekf_enabled)` passed as literal string instead of boolean). Implemented `load_and_resolve_param_file()` in params.rs to recursively resolve all substitutions in YAML structure and convert resolved strings to proper YAML types (boolean, integer, float). Type conversion: `"true"/"false"` → bool, `"123"` → i64, `"123.45"` → f64, others → string. Fixes `parameter 'ekf_enabled' has invalid type` errors. All 308 tests pass, Autoware nodes start successfully.
- **2026-02-01**: Phase 14 complete - Substitution Context Unification: Fixed nested substitution resolution by extracting launch_configurations from Python context and populating Rust LaunchContext. Enables proper resolution of `$(var other_var)` patterns where `other_var` contains substitutions. Helper functions: `create_context_from_python()`, `resolve_substitution_string()`. All 297 tests pass, Autoware validated.
- **2026-02-01**: Security updates - Upgraded `lru` from 0.12 to 0.16.3 (fixes RUSTSEC-2026-0002 IterMut soundness issue). PyO3 upgrade to 0.24.1 deferred due to 268 breaking API changes; vulnerable function `PyString::from_object` not used in codebase.
- **2026-02-01**: Comprehensive test script - Added `tmp/run_all_checks.sh` with timeouts on all tests: (1) Root quality (5min), (2) Parser quality (3min), (3) Simple test (10s), (4) Autoware parser comparison (dumps both parsers and compares outputs, 60s), (5) LCTK demo (30s). Prevents infinite hangs, proper exit code handling. Updated 2026-02-06 to use parser comparison instead of runtime simulation test.
- **2026-02-01**: Substitution evaluation improvements - (1) Added PythonExpression.perform() to evaluate Python conditional expressions in parameters (fixes "Couldn't parse parameter override rule" errors with IfElse expressions). (2) Fixed float type preservation: floats now output with decimal point (e.g., "0.0" not "0") to prevent ROS INTEGER vs DOUBLE type errors. (3) Extended evaluating substitutions to include FileContent and PathJoinSubstitution for proper nested substitution resolution. (4) Parser evaluates conditional logic and processes only the selected path, not all paths. LaunchConfiguration substitutions preserved as $(var name) for replay-time resolution.
- **2026-01-31**: Phase 14 complete - Python launch file execution through ROS 2 launch system (LaunchConfiguration resolution, global parameters, SetLaunchConfiguration support, Autoware validation passes)
- **2026-01-27**: Phase 13 complete - Rust parser as default (3-12x speedup), Python parser as optional fallback

## Testing

### Test Infrastructure

Tests use [nextest](https://nexte.st/) as the test runner, with two separate crates:

1. **Parser unit tests** (`src/play_launch_parser/`): 311 tests for the Rust parser
2. **Integration tests** (`tests/`): Standalone crate (`play-launch-tests`) for end-to-end testing

The integration test crate is excluded from the cargo workspace (`exclude = ["tests"]` in root `Cargo.toml`) because `play_launch` has ROS dependencies that only resolve under colcon.

### Test Recipes

```bash
just test              # Parser unit (311) + fast integration (6), ~3s
just test-all          # Parser unit (311) + all integration (12), ~30s
just test-unit         # Parser unit tests only
just test-integration  # All integration tests (simple + Autoware)
just test-simple       # simple_workspace binary only
just test-autoware     # Autoware binary only
```

All recipes use `--no-fail-fast --failure-output immediate-final`.

### Nextest Configuration

- **Parser** (`.config/nextest.toml`): Python tests serialized via test group (max 1 thread)
- **Integration** (`tests/.config/nextest.toml`): 60s slow-timeout for simple_workspace and Autoware binaries

### Process Cleanup (ManagedProcess)

All integration tests spawn processes through `ManagedProcess` (`tests/src/process.rs`), an RAII guard that guarantees cleanup:

- **`setsid()`**: Child runs in its own process group — `kill(-pgid, ...)` kills the entire tree
- **`PR_SET_PDEATHSIG(SIGKILL)`**: Kernel kills child if the test runner process dies unexpectedly
- **`Drop`**: Sends SIGTERM then SIGKILL to the process group if the child is still running
- **`wait_with_timeout()`**: Polls with deadline, kills group on timeout

This prevents orphan ROS processes from lingering after test panics, assertion failures, or timeouts.

### Test Workspaces

- `test/autoware/`: Full Autoware test (46 nodes, 15 containers, 54 composable nodes)
- `test/simple_test/`: Basic container with 2 nodes
- `test/sequential_loading/`: FIFO queue testing
- `test/concurrent_loading/`: Parallel loading testing

Each test workspace has `justfile` with:
- `just run`, `just run-debug`: Run tests
- `just dump-rust`, `just dump-python`, `just dump-both`: Generate record.json with Rust/Python parsers
- `just compare-dumps`: Compare Rust vs Python parser outputs using `scripts/compare_records.py`

## Distribution

- **Primary**: PyPI (`pip install play_launch`)
- Wheels for x86_64 and aarch64 (Ubuntu 22.04+)
- Binary optimization: 94% size reduction (strip + LTO)
- Build: `just build` → `dist/play_launch-*.whl`
- Publish: `just publish-pypi` or GitHub Actions on version tags

## Documentation

- **Migration Guide**: `docs/parser-migration-guide.md` - Guide for users migrating to Rust parser (v0.6.0+)
- **Roadmap**: `docs/roadmap/README.md` - Implementation phases and progress tracking (Phase 18 complete)
- **Phase 13 (Complete)**: `docs/roadmap/phase-13.md` - Rust parser integration (3-12x speedup)
- **Phase 14 (Complete)**: `docs/roadmap/phase-14-python_execution.md` - Python launch file execution with LaunchConfiguration resolution

## Parser Parity Status

**Autoware Validation** (Phase 17.9 complete):
- Entity counts: 46 nodes, 15 containers, 54 load_nodes — all match Python parser
- Functional equivalence: 45/45 nodes (100%)
- Process count: 61/61 (both parsers)

**Remaining cosmetic (non-functional) differences:**
- CMD param deduplication: Rust deduplicates, Python preserves duplicates from nested includes
- params_files expansion: Rust keeps YAML file references, Python inlines parameters (3 nodes)
- CMD ordering: same parameters in different order (2 nodes)
- XML whitespace: robot_state_publisher xacro newlines vs spaces (1 node)
- Array quoting: `[a, b]` vs `['a', 'b']` (1 node)
- Load node boolean case: `False` vs `false`
- Load node extra_args: Rust omits default `use_intra_process_comms` when not set (18 nodes)
