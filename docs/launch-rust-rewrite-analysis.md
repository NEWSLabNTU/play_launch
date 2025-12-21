# ROS Launch to Rust Rewrite Analysis

**Date**: 2025-12-21
**Purpose**: Analysis for rewriting dump_launch from Python to Rust for performance

## Current Architecture

### dump_launch (Python)

**What it does**:
1. **Injects code** into the ROS 2 Python launch process
2. **Copies LaunchService** behavior with added inspection/recording
3. **Visits entities** during launch execution to extract metadata
4. **Records to JSON** (`record.json`) for later replay by play_launch (Rust)

**Key Components**:

```
python/play_launch/dump/
├── __init__.py           # Entry point (main function)
├── inspector.py          # LaunchInspector (fork of LaunchService)
├── launch_dump.py        # Data structures for recording
├── ros_cmdline.py        # ROS argument parsing
├── utils.py              # YAML handling, parameter conversion
├── event_handlers.py     # Custom event handlers
└── visitor/              # Entity visitor pattern
    ├── entity.py         # Generic entity visitor
    ├── action.py         # Action visitor dispatcher
    ├── node.py           # Regular Node visitor
    ├── composable_node_container.py
    ├── load_composable_nodes.py
    ├── lifecycle_node.py
    ├── execute_process.py
    ├── execute_local.py
    └── include_launch_description.py
```

### ROS Launch Architecture (Python)

**Core Classes** (from `external/launch/`):

1. **LaunchService** - Main event loop and execution manager
   - Manages async event loop
   - Handles event emission and handling
   - Tracks entity futures
   - Provides shutdown mechanism

2. **LaunchContext** - Execution context
   - Stores launch arguments (`argv`)
   - Manages launch configurations (key-value pairs)
   - Tracks event handlers
   - Handles event emission

3. **LaunchDescription** - Container for entities
   - List of launch description entities
   - Can be nested via IncludeLaunchDescription

4. **LaunchDescriptionEntity** - Base for all entities
   - Has `visit(context)` method
   - Returns sub-entities or None
   - Has `get_asyncio_future()` for async tracking

5. **Action** - Subclass of entity for executable actions
   - ExecuteProcess (base for running processes)
   - ExecuteLocal (extended version with respawn, etc.)
   - Node (ROS-specific, inherits ExecuteProcess)
   - ComposableNodeContainer
   - LoadComposableNodes
   - LifecycleNode
   - SetParameter, DeclareLaunchArgument, etc.

6. **Frontend Parsers** (launch/launch/frontend/)
   - XML parser
   - YAML parser
   - Python parser (imports .py files)
   - Parses to LaunchDescription with entities

**Execution Flow**:

```
1. Parse launch file (XML/YAML/Python) → LaunchDescription
2. LaunchService.include_launch_description(desc)
3. Event emission → IncludeLaunchDescription event
4. Event handlers process entities
5. Entity.visit(context) → returns sub-entities
6. Recursively visit all entities (depth-first)
7. Actions execute (spawn processes, etc.)
8. Async futures track completion
9. Event loop runs until idle or shutdown
```

**What dump_launch Intercepts**:

- **Node actions**: Extracts package, executable, namespace, name, params, remaps, args, env vars, respawn config
- **ComposableNodeContainer**: Records container info
- **LoadComposableNodes**: Records which nodes to load into which container
- **LifecycleNode**: Records lifecycle node info
- **Parameters**: Reads param files, extracts parameter values
- **Global params**: From SetParameter actions (via launch_configurations)
- **Environment**: From `<env>` tags or additional_env

## Performance Issues

### Why It's Slow

1. **Python Launch System**:
   - Python interpreter overhead
   - Async event loop in Python
   - Lots of object creation/destruction
   - Dynamic typing overhead

2. **dump_launch Overhead**:
   - Runs entire Python launch process just to extract metadata
   - No actual process spawning needed for dump phase
   - All the async machinery runs even though we just want to traverse entities
   - Heavy use of Python introspection (accessing private `__` attributes)

3. **Two-Phase Execution**:
   - Phase 1: dump_launch (Python) - just to get metadata
   - Phase 2: play_launch (Rust) - actual execution
   - Phase 1 is wasteful since we don't actually run anything

### Measurements Needed

- [ ] Profile dump_launch execution time for large launch files (e.g., Autoware)
- [ ] Identify bottlenecks (parsing vs traversal vs I/O)
- [ ] Compare with native `ros2 launch` execution time

## Rust Rewrite Options

### Option 1: Rust Launch Parser + Visitor (Recommended)

**Approach**: Parse launch files directly in Rust and traverse to extract metadata

**Pros**:
- **Fast**: Native Rust performance
- **Single-phase**: Extract metadata without running Python at all
- **Type-safe**: Compile-time guarantees
- **No Python dependency** for dump phase

**Cons**:
- **Large effort**: Need to reimplement launch file parsing
- **Maintenance**: Must keep up with ROS launch changes
- **Complexity**: Launch system is complex (frontends, substitutions, actions, etc.)

**What needs to be implemented**:

1. **Launch File Parsers**:
   - XML parser (using `quick-xml` or similar)
   - YAML parser (using `serde_yaml`)
   - Python parser (this is tricky - might need to keep Python for `.launch.py` files)

2. **Substitution Engine**:
   - $(find package)
   - $(env VAR)
   - $(arg name)
   - $(eval expression)
   - $(dirname), $(anon name), etc.
   - PathJoinSubstitution, TextSubstitution, etc.

3. **Entity System**:
   - Action trait
   - Node, ComposableNodeContainer, LoadComposableNodes
   - ExecuteProcess, ExecuteLocal
   - IncludeLaunchDescription
   - DeclareLaunchArgument, SetParameter, etc.

4. **Context Management**:
   - Launch arguments
   - Launch configurations (key-value store)
   - Scope management for parameters

5. **Visitor Pattern**:
   - Visit entities recursively
   - Extract metadata without executing
   - Handle conditionals, loops, etc.

**Estimated Effort**: 4-6 weeks for MVP, 8-12 weeks for feature parity

### Option 2: Python AST Analysis (Hybrid)

**Approach**: Use Python's `ast` module to analyze `.launch.py` files without executing them

**Pros**:
- Faster than executing Python
- No need to reimplement substitutions
- Leverages Python's built-in parsing

**Cons**:
- Only works for Python launch files (not XML/YAML)
- Still requires Python
- AST analysis can be brittle
- Won't handle dynamic logic well

**Estimated Effort**: 2-3 weeks

### Option 3: Optimize dump_launch (Quick Win)

**Approach**: Keep Python but optimize the hot paths

**Ideas**:
- Cache parsed launch files
- Skip unnecessary async machinery (use synchronous traversal where possible)
- Parallelize file I/O (param file reads)
- Profile and optimize bottlenecks
- Consider using PyPy for JIT compilation

**Pros**:
- Low effort
- Low risk
- Immediate improvements possible

**Cons**:
- Still fundamentally Python-limited
- Won't achieve Rust-level performance

**Estimated Effort**: 1-2 weeks

### Option 4: Hybrid - FFI to Rust Visitors

**Approach**: Keep Python launch system but call Rust for visitor callbacks

**Pros**:
- Leverage existing launch system
- Optimize hot paths in Rust
- Incremental migration

**Cons**:
- FFI overhead (PyO3)
- Complexity of Python/Rust boundary
- May not yield significant speedup

**Estimated Effort**: 2-4 weeks

## Recommendation

### Short Term (1-2 weeks)
**Option 3: Optimize dump_launch**
- Profile current performance
- Identify and fix bottlenecks
- Low risk, quick wins

### Medium Term (2-3 months)
**Option 1: Rust Launch Parser** - MVP focusing on:
- XML parser (most common for ROS packages)
- Basic substitutions
- Core actions (Node, ComposableNodeContainer, LoadComposableNodes)
- Keep Python fallback for complex `.launch.py` files

### Long Term (6+ months)
**Full Rust Implementation**
- All launch file formats
- Complete substitution engine
- All actions and entities
- Feature parity with ROS launch

## Key Challenges for Rust Rewrite

1. **Python Launch Files**:
   - `.launch.py` files can have arbitrary Python code
   - Would need to either:
     - Keep Python for these (hybrid approach)
     - Restrict to declarative subset and parse AST
     - Require users to convert to XML/YAML

2. **Substitution Engine**:
   - Complex recursive substitution system
   - Many built-in substitution types
   - Need to handle $(eval) which can have Python expressions

3. **ROS Integration**:
   - Need to find packages (`ament_index` in Python)
   - Resolve package paths
   - Handle ROS-specific substitutions

4. **Testing**:
   - Large test surface area
   - Need to verify compatibility with existing launch files
   - Regression testing against Python implementation

5. **Maintenance**:
   - ROS launch evolves (new actions, substitutions, etc.)
   - Need to keep Rust version in sync

## Dependencies for Rust Implementation

```toml
[dependencies]
# XML parsing
quick-xml = "0.31"

# YAML parsing
serde_yaml = "0.9"

# JSON output
serde_json = "1.0"
serde = { version = "1.0", features = ["derive"] }

# Path handling
pathdiff = "0.2"

# Regular expressions (for substitutions)
regex = "1.10"

# Python interop (for .launch.py files)
pyo3 = { version = "0.20", features = ["auto-initialize"] }

# Error handling
eyre = "0.6"
thiserror = "1.0"
```

## Success Metrics

1. **Performance**:
   - Dump phase < 1 second for Autoware-sized launch files (vs current)
   - 10x speedup over current Python implementation

2. **Compatibility**:
   - Parse 95%+ of existing ROS launch files
   - Byte-for-byte identical `record.json` output

3. **Maintainability**:
   - Clear separation between parser and visitor
   - Extensible for new actions/substitutions
   - Good test coverage (>80%)

## Next Steps

1. **Benchmark current dump_launch** with Autoware
2. **Prototype XML parser** in Rust (1 week)
3. **Implement core visitor for Node actions** (1 week)
4. **Compare performance** against Python version
5. **Decide** based on prototype results

## References

- ROS Launch: https://github.com/ros2/launch
- ROS Launch ROS: https://github.com/ros2/launch_ros
- Launch XML spec: https://github.com/ros2/launch/blob/humble/launch_xml/doc/architecture.md
- Launch YAML spec: https://github.com/ros2/launch/blob/humble/launch_yaml/doc/architecture.md
