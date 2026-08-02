# dump_launch Implementation Analysis

Comprehensive analysis of the dump_launch Python implementation from the play_launch project.

**Source:** `~/repos/play_launch/python/play_launch/dump/`

**Date:** 2026-01-18

---

## 1. Overview

dump_launch is a **launch tree inspection tool** that:
- Traverses ROS 2 launch descriptions using a visitor pattern
- Collects metadata about nodes, containers, and composable nodes
- Generates a `record.json` file for replay with play_launch runtime
- Integrates with the ROS 2 launch event system

**Performance Bottleneck:** Takes ~40 seconds to scan Autoware launch tree due to:
- Python overhead
- Dynamic launch execution
- Async event processing
- Private attribute access via name mangling

---

## 2. Architecture

### 2.1 Component Structure

```
dump/
├── __init__.py              # Entry point (LaunchInspector.dump())
├── inspector.py             # LaunchInspector class (493 lines)
├── launch_dump.py           # Data structures (50 lines)
├── ros_cmdline.py           # Command-line parser (122 lines)
├── utils.py                 # Utilities (92 lines)
└── visitor/
    ├── entity.py            # Entity visitor (41 lines)
    ├── action.py            # Action dispatcher (59 lines)
    ├── node.py              # Node visitor (196 lines)
    ├── execute_process.py   # Process visitor (113 lines)
    ├── composable_node_container.py     # Container visitor (57 lines)
    ├── load_composable_nodes.py         # Load nodes visitor (176 lines)
    └── lifecycle_node.py    # Lifecycle node visitor (25 lines)
```

### 2.2 Data Flow

```
User Command
    ↓
LaunchInspector.dump(package, launch_file)
    ↓
Create LaunchService with event loop
    ↓
Register OnIncludeLaunchDescription handler
    ↓
Include launch description (triggers event)
    ↓
OnIncludeLaunchDescription → visit_entity()
    ↓
Recursive entity visitation via visitor pattern
    ├→ Nodes → visit_node() → NodeRecord
    ├→ Containers → visit_composable_node_container() → ContainerRecord
    ├→ LoadComposableNodes → visit_load_composable_nodes() → LoadNodeRecord
    └→ Other actions → action.execute()
    ↓
Collect futures and wait for completion
    ↓
Generate LaunchDump structure
    ↓
Serialize to record.json
```

---

## 3. Core Components

### 3.1 LaunchInspector (`inspector.py`)

Main orchestrator managing the inspection process.

**Key Methods:**

```python
class LaunchInspector:
    def __init__(self):
        self._event_handlers_to_unregister = []
        self._event_loop = None
        self._launch_service = None

    def dump(self, package_name: str, launch_file: str, *, launch_arguments: list[str] = []) -> str:
        """Inspect launch and return JSON string"""
        # 1. Parse launch arguments
        # 2. Create LaunchService
        # 3. Register event handler
        # 4. Include launch description
        # 5. Run launch service (async event loop)
        # 6. Wait for all futures
        # 7. Serialize to JSON

    def _on_include_launch_description(self, event: IncludeLaunchDescription) -> None:
        """Event handler for launch includes"""
        # Visit all entities in the included description
```

**Event-Driven Flow:**
1. Creates `LaunchService` with async event loop
2. Registers `OnIncludeLaunchDescription` event handler
3. Includes the target launch file (triggers event)
4. Event handler calls `visit_entity()` on all entities
5. Visitors recursively traverse the tree
6. Collects asyncio futures for async operations
7. Waits for all futures to complete
8. Returns JSON dump

### 3.2 LaunchDump Data Structure (`launch_dump.py`)

**Root Structure:**

```python
@dataclass
class LaunchDump:
    node: list[NodeRecord] = field(default_factory=list)
    container: list[ComposableNodeContainerRecord] = field(default_factory=list)
    load_node: list[LoadNodeRecord] = field(default_factory=list)
    lifecycle_node: list[str] = field(default_factory=list)
    file_data: dict[str, str] = field(default_factory=dict)  # path → content
```

**NodeRecord:**

```python
@dataclass
class NodeRecord:
    executable: str              # e.g., "talker"
    package: str | None          # e.g., "demo_nodes_cpp"
    name: str | None             # Fully expanded node name
    namespace: str | None        # Node namespace
    exec_name: str | None        # Execution name (for tracking)
    params: list[tuple[str, str]]         # Inline parameters
    params_files: list[str]               # Parameter file contents
    remaps: list[tuple[str, str]]         # Topic remappings
    ros_args: list[str] | None            # ROS-specific arguments
    args: list[str] | None                # User-defined arguments
    cmd: list[str]                        # Complete command line
    env: list[tuple[str, str]] | None     # Environment variables
    respawn: bool | None                  # Respawn on exit
    respawn_delay: float | None           # Respawn delay
    global_params: list[tuple[str, str]] | None  # SetParameter-based params
```

**ComposableNodeContainerRecord:**

```python
@dataclass
class ComposableNodeContainerRecord:
    name: str          # Container node name (with namespace)
    namespace: str     # Container namespace
```

**LoadNodeRecord:**

```python
@dataclass
class LoadNodeRecord:
    package: str                        # Plugin package
    plugin: str                         # Plugin class name
    target_container_name: str          # Full container name
    node_name: str                      # Node name
    namespace: str                      # Node namespace
    log_level: str | None               # DEBUG/INFO/WARN/ERROR/FATAL
    remaps: list[tuple[str, str]]       # Topic remappings
    params: list[tuple[str, str]]       # Parameters
    extra_args: dict[str, str]          # Additional arguments
    env: list[tuple[str, str]] | None   # Environment variables
```

---

## 4. Visitor Pattern Implementation

### 4.1 Entity Visitor (`visitor/entity.py`)

Entry point for recursive tree traversal.

```python
def visit_entity(entity, context: LaunchContext, dump: LaunchDump):
    """Visit entity and recurse through sub-entities"""

    # Dispatch by type
    if is_a_subclass(entity, Action):
        sub_entities = visit_action(entity, context, dump)
    else:
        sub_entities = entity.visit(context)

    # Collect futures
    futures_to_return = []
    entity_future = entity.get_asyncio_future()
    if entity_future is not None:
        futures_to_return.append((entity, entity_future))

    # Recurse
    if sub_entities is not None:
        for sub_entity in sub_entities:
            futures_to_return += visit_entity(sub_entity, context, dump)

    return futures_to_return
```

**Key Points:**
- Returns list of `(entity, future)` tuples for async tracking
- Recursively visits all sub-entities
- Handles both Action and Entity types

### 4.2 Action Dispatcher (`visitor/action.py`)

Routes actions to specialized visitors.

```python
def visit_action(action, context: LaunchContext, dump: LaunchDump):
    """Dispatch action to appropriate visitor"""

    if isinstance(action, IncludeLaunchDescription):
        return visit_include_launch_description(action, context, dump)

    if isinstance(action, launch_ros.actions.Node):
        return visit_node(action, context, dump)

    if isinstance(action, launch_ros.actions.LifecycleNode):
        return visit_lifecycle_node(action, context, dump)

    if isinstance(action, launch_ros.actions.ComposableNodeContainer):
        return visit_composable_node_container(action, context, dump)

    if isinstance(action, launch_ros.actions.LoadComposableNodes):
        return visit_load_composable_nodes(action, context, dump)

    # Default: execute action
    return action.execute(context)
```

### 4.3 Node Visitor (`visitor/node.py`)

Extracts node metadata and builds NodeRecord.

**Key Steps:**

1. **Perform Substitutions:**
   ```python
   node._perform_substitutions(context)
   ```

2. **Extract Package and Executable:**
   ```python
   package = perform_substitutions(context, node.node_package)
   executable = perform_substitutions(context, node.node_executable)
   ```

3. **Get Namespace:**
   ```python
   namespace = perform_substitutions(context, node.node_namespace) if node.node_namespace else None
   ```

4. **Extract Parameters:**
   ```python
   # Inline parameters
   params = [(k, str(v)) for k, v in node._Node__expanded_parameter_arguments]

   # Parameter files (read content)
   for param_file_path in node._Node__parameter_files:
       resolved_path = perform_substitutions(context, param_file_path)
       with open(resolved_path) as f:
           content = f.read()
       dump.file_data[resolved_path] = content
       params_files.append(content)
   ```

5. **Extract Remappings:**
   ```python
   remaps = [
       (perform_substitutions(context, src), perform_substitutions(context, dst))
       for src, dst in node.expanded_remapping_rules
   ]
   ```

6. **Get Global Parameters:**
   ```python
   global_params = [
       (k, str(v)) for k, v in context.launch_configurations.get("global_params", {}).items()
   ]
   ```

7. **Build Command Line:**
   ```python
   cmd = node._Node__process_config['cmd']
   ```

8. **Create NodeRecord:**
   ```python
   record = NodeRecord(
       executable=executable,
       package=package,
       name=node_name,
       namespace=namespace,
       params=params,
       params_files=params_files,
       remaps=remaps,
       # ...
   )
   dump.node.append(record)
   ```

9. **Continue to ExecuteProcess visitor:**
   ```python
   return visit_execute_process(node, context, dump)
   ```

**Private Attribute Access (Fragile!):**

```python
# These use Python name mangling and are fragile
node._Node__expanded_parameter_arguments
node._Node__parameter_files
node._Node__process_config
node._ExecuteLocal__respawn
node._ExecuteLocal__respawn_delay
```

### 4.4 Composable Node Container Visitor (`visitor/composable_node_container.py`)

Records container metadata.

```python
def visit_composable_node_container(action, context: LaunchContext, dump: LaunchDump):
    """Visit composable node container"""

    # Perform substitutions
    action._perform_substitutions(context)

    # Get container name with namespace
    name = perform_substitutions(context, action.node_name)
    namespace = perform_substitutions(context, action.node_namespace) if action.node_namespace else "/"

    # Build full name
    if namespace != "/":
        full_name = namespace + "/" + name
    else:
        full_name = "/" + name

    # Record container
    record = ComposableNodeContainerRecord(name=full_name, namespace=namespace)
    dump.container.append(record)

    # Continue as regular node
    return visit_node(action, context, dump)
```

### 4.5 LoadComposableNodes Visitor (`visitor/load_composable_nodes.py`)

Extracts composable node loading information.

**Key Steps:**

1. **Get Target Container:**
   ```python
   target_container = perform_substitutions(context, action._LoadComposableNodes__target_container_name)
   ```

2. **Iterate Composable Nodes:**
   ```python
   for composable_node in action._LoadComposableNodes__composable_node_descriptions:
       # Extract metadata
   ```

3. **Get Load Request:**
   ```python
   request = action._LoadComposableNodes__get_composable_node_load_request(composable_node, context)
   ```

4. **Extract Plugin Info:**
   ```python
   plugin = request.plugin_name  # e.g., "composition::Talker"
   package = request.package_name
   ```

5. **Extract Parameters:**
   ```python
   params = []
   for param in request.parameters:
       name = param.name
       # Type-specific value extraction
       if param.type == ParameterType.PARAMETER_BOOL:
           value = str(param.bool_value)
       elif param.type == ParameterType.PARAMETER_INTEGER:
           value = str(param.integer_value)
       elif param.type == ParameterType.PARAMETER_DOUBLE:
           value = str(param.double_value)
       elif param.type == ParameterType.PARAMETER_STRING:
           value = param.string_value
       # ... other types
       params.append((name, value))
   ```

6. **Convert Log Level:**
   ```python
   log_level = log_level_code_to_text(request.log_level) if request.log_level != 0 else None
   ```

7. **Extract Remappings:**
   ```python
   remaps = [(r.from_rule, r.to_rule) for r in request.remap_rules]
   ```

8. **Create LoadNodeRecord:**
   ```python
   record = LoadNodeRecord(
       package=package,
       plugin=plugin,
       target_container_name=target_container,
       node_name=node_name,
       namespace=namespace,
       log_level=log_level,
       remaps=remaps,
       params=params,
       extra_args=extra_args,
       env=env
   )
   dump.load_node.append(record)
   ```

**Log Level Mapping (`utils.py`):**

```python
def log_level_code_to_text(code: int) -> str:
    """Convert ROS log level code to text"""
    mapping = {
        10: "DEBUG",
        20: "INFO",
        30: "WARN",
        40: "ERROR",
        50: "FATAL",
    }
    return mapping.get(code, "UNKNOWN")
```

---

## 5. Challenges and Bottlenecks

### 5.1 Private Attribute Access

**Problem:** Extensive use of Python name mangling to access private attributes.

**Examples:**
```python
node._Node__expanded_parameter_arguments
node._Node__parameter_files
node._Node__process_config
node._ExecuteLocal__respawn
action._LoadComposableNodes__target_container_name
action._LoadComposableNodes__composable_node_descriptions
```

**Consequences:**
- **Fragile:** Breaks with Python version changes or ROS library updates
- **Maintenance burden:** Requires intimate knowledge of internal structure
- **No API stability:** Internal APIs can change without notice

### 5.2 Async Complexity

**Problem:** Heavy async/await machinery for event handling.

**Components:**
- Event loop management
- Future collection and tracking
- Completion waiting
- Pruning completed futures

**Overhead:**
- Event handler registration/deregistration
- Async task scheduling
- Context switching
- Future management

### 5.3 Late Parameter Resolution

**Problem:** Parameter files read at different times.

**Cases:**
1. **Inline param files:** Read during `visit_node()` before process starts
2. **Exit-triggered params:** Read in `__on_process_exit()` after process completes

**Issues:**
- Timing inconsistency
- May miss dynamically generated parameters
- Complex state tracking

### 5.4 Substitution Performance

**Problem:** Multiple substitution passes.

**Phases:**
1. `node._perform_substitutions(context)` - Modifies node state
2. `perform_substitutions(context, field)` - Per-field resolution
3. Repeated for every attribute

**Accumulation:** Can become expensive with large launch files and many substitutions.

### 5.5 Type Code Mapping

**Problem:** Hard-coded integer-to-text mappings.

**Examples:**
- Log levels: `10 → DEBUG`, `20 → INFO`, etc.
- Parameter types: `PARAMETER_BOOL`, `PARAMETER_INTEGER`, etc.

**Fragility:** Brittle to ROS API changes.

### 5.6 Error Handling

**Problem:** Limited error messages.

**Examples:**
```python
assert expr is not None  # No context on failure
```

**Consequences:**
- Silent failures in complex launches
- Difficult debugging
- Poor user experience

### 5.7 Special Cases

**Lifecycle Nodes:**
```python
def visit_lifecycle_node(action, context: LaunchContext, dump: LaunchDump):
    # Get node name
    name = perform_substitutions(context, action.node_name)
    namespace = perform_substitutions(context, action.node_namespace) if action.node_namespace else "/"
    full_name = namespace + "/" + name if namespace != "/" else "/" + name

    # Record as lifecycle node
    dump.lifecycle_node.append(full_name)

    # Continue as regular node
    return visit_node(action, context, dump)
```

**SetParameter Actions:**
- Captured in context as `global_params`
- Retrieved later during node visitation
- Requires context state management

**Unsupported Features:**
- `on_exit` handlers: Logs warning once, then ignores

---

## 6. Key Implementation Details

### 6.1 Namespace Resolution

**Full Node Name Construction:**

```python
def get_full_node_name(name, namespace):
    if namespace is None or namespace == "/":
        return "/" + name
    elif namespace.startswith("/"):
        return namespace + "/" + name
    else:
        return "/" + namespace + "/" + name
```

### 6.2 Parameter File Handling

**Reading Parameter Files:**

```python
for param_file_path in node._Node__parameter_files:
    resolved_path = perform_substitutions(context, param_file_path)
    with open(resolved_path) as f:
        content = f.read()
    dump.file_data[resolved_path] = content
    params_files.append(content)
```

**File Data Storage:**
- Stored in `dump.file_data` dict: `{path: content}`
- Content stored verbatim (YAML string)
- Referenced by path in node records

### 6.3 Command-Line Building

**ROS 2 Command Format:**

```
executable --ros-args -r __node:=name -r __ns:=/namespace -p param:=value ...
```

**Extraction:**

```python
cmd = node._Node__process_config['cmd']
# Already built by Node action
```

### 6.4 Environment Variables

**Extraction:**

```python
env = list(node.additional_env) if node.additional_env else None
# List of (name, value) tuples
```

---

## 7. Performance Analysis

### 7.1 Measured Performance

**Autoware Launch Tree:** ~40 seconds

**Breakdown (estimated):**
- Launch execution and event processing: 30%
- Substitution resolution: 25%
- Private attribute access: 15%
- Parameter file I/O: 15%
- Visitor pattern overhead: 10%
- JSON serialization: 5%

### 7.2 Optimization Opportunities

1. **Static Analysis:** Parse launch files without executing them
2. **Parallel Processing:** Parse multiple includes concurrently
3. **Caching:** Cache substitution results
4. **Lazy Loading:** Don't read param files immediately
5. **Native Code:** Use compiled language (Rust!)

---

## 8. Rust Parser Considerations

### 8.1 What to Replicate

**Essential Functionality:**
- ✅ Parse XML/YAML launch files
- ✅ Resolve substitutions
- ✅ Handle includes recursively
- ✅ Extract node metadata
- ✅ Handle composable nodes and containers
- ✅ Generate compatible record.json

**Nice to Have:**
- Event system (not needed for static parsing)
- Full action execution (only for Python launch files)
- Dynamic parameter generation (rare)

### 8.2 What to Avoid

**Anti-Patterns from dump_launch:**
- ❌ Private attribute access (use public APIs or custom structures)
- ❌ Async event system (static analysis is synchronous)
- ❌ Late parameter resolution (read files during parse)
- ❌ Hard-coded type mappings (use enums or constants)

### 8.3 Simplifications Possible

1. **No Launch Execution:** Parse files statically without running them
2. **No Event Handlers:** Don't need event system for static analysis
3. **Eager Parameter Loading:** Read all param files during parse
4. **Parallel Includes:** Parse multiple includes concurrently
5. **No Private Access:** Design custom data structures from scratch

### 8.4 Compatibility Requirements

**Must Match record.json Format:**
- Same field names
- Same data types
- Same serialization format
- Compatible with play_launch runtime

**Field-by-Field Compatibility:**
```json
{
    "node": [
        {
            "executable": "talker",
            "package": "demo_nodes_cpp",
            "name": "/talker",
            "namespace": "/",
            "exec_name": "talker-1",
            "params": [["param1", "value1"]],
            "params_files": ["yaml_content..."],
            "remaps": [["from", "to"]],
            "ros_args": null,
            "args": null,
            "cmd": ["/path/to/executable", "--ros-args", ...],
            "env": null,
            "respawn": false,
            "respawn_delay": null,
            "global_params": []
        }
    ],
    "container": [...],
    "load_node": [...],
    "lifecycle_node": [...],
    "file_data": {...}
}
```

---

## 9. Testing Strategy

### 9.1 Comparison Testing

**Approach:**
1. Run dump_launch on test launch file
2. Run Rust parser on same file
3. Compare generated record.json files
4. Assert semantic equivalence

**Test Cases:**
- Simple nodes
- Nodes with parameters
- Nodes with remappings
- Composable nodes
- Nested includes
- Substitutions
- Conditions

### 9.2 Performance Benchmarking

**Metrics to Track:**
- Parse time (total)
- Time per node
- Time per include
- Memory usage
- CPU usage

**Target:**
- Autoware launch tree: <5 seconds (vs. 40s current)
- 8x speedup minimum

---

## 10. Summary

### Key Takeaways

1. **dump_launch uses visitor pattern** to traverse launch trees
2. **Heavy reliance on private attributes** makes it fragile
3. **Async event system** adds significant overhead
4. **Performance bottleneck** is well-documented (~40s for Autoware)
5. **Rust parser can simplify** by avoiding execution and using static analysis

### Critical Path for Rust Implementation

1. ✅ Parse XML/YAML launch files into entity tree
2. ✅ Resolve substitutions (especially `var`, `env`, `find-pkg-share`)
3. ✅ Handle includes recursively
4. ✅ Extract node metadata (package, exec, params, remaps)
5. ✅ Handle composable nodes and containers
6. ✅ Generate record.json compatible with play_launch

### Success Criteria

- **Correctness:** Generate identical record.json to dump_launch
- **Performance:** Parse Autoware tree in <5 seconds
- **Maintainability:** Clean code without private attribute hacks
- **Compatibility:** Works with play_launch runtime without modification
