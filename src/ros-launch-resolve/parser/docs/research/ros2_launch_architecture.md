# ROS 2 Launch System Architecture

Comprehensive analysis of the ROS 2 launch system based on exploration of the official repositories.

**Sources Analyzed:**
- `external/launch` (launch, launch_xml, launch_yaml packages)
- `external/launch_ros` (ROS-specific launch extensions)
- `external/demos` (Example launch files)

**Date:** 2026-01-18

---

## 1. Overview

The ROS 2 launch system is a **plugin-based, extensible framework** for describing and executing complex robot launch configurations. It uses:

- **Visitor Pattern** for entity traversal
- **Event-Driven Architecture** for async execution
- **Plugin System** for extensibility (entry points)
- **Multiple Frontend Parsers** (XML, YAML, Python)

---

## 2. Package Structure

### Core Package: `launch`

Main launch framework providing:
- Base classes (`Action`, `Condition`, `Substitution`, `LaunchDescriptionEntity`)
- Frontend parser infrastructure
- Event system
- Launch context and execution engine

**Key Modules:**
```
launch/
├── actions/          # 26+ action types (ExecuteProcess, Node, Group, etc.)
├── conditions/       # IfCondition, UnlessCondition, etc.
├── substitutions/    # 20+ substitution types
├── events/           # ExecutionComplete, ProcessExit, etc.
├── event_handlers/   # OnProcessExit, OnShutdown, etc.
├── frontend/         # Parser infrastructure
│   ├── parser.py        # Abstract Parser base class
│   ├── entity.py        # Abstract Entity intermediate representation
│   ├── expose.py        # @expose_action/@expose_substitution decorators
│   └── parse_substitution.py  # Lark-based grammar parser
└── share/launch/frontend/grammar.lark  # Substitution grammar
```

### Frontend Parsers: `launch_xml` and `launch_yaml`

Plugin packages that register parsers via entry points:

**launch_xml:**
- Uses `xml.etree.ElementTree` (Python stdlib)
- Wraps `ET.Element` as `Entity`
- Supports type coercion

**launch_yaml:**
- Uses `yaml.safe_load()` for security
- Wraps `dict` as `Entity`
- Type checking only (no coercion)

### ROS Extensions: `launch_ros`

ROS-specific actions and substitutions:
```
launch_ros/
├── actions/
│   ├── node.py              # Node action (package/executable resolution)
│   ├── lifecycle_node.py    # Lifecycle-aware nodes
│   ├── composable_node_container.py  # Component containers
│   ├── load_composable_nodes.py      # Dynamic composition
│   └── set_parameter.py     # Global parameter setting
├── descriptions/
│   └── composable_node.py   # ComposableNode description
└── substitutions/
    ├── find_package.py      # Find package paths
    └── executable_in_package.py  # Resolve executables
```

---

## 3. Core Concepts

### 3.1 Launch Description

**LaunchDescription** is a collection of entities to execute:

```python
class LaunchDescription:
    entities: List[LaunchDescriptionEntity]
```

Entities can be:
- **Actions**: Things to execute (nodes, processes, etc.)
- **Event Handlers**: React to events
- **Other LaunchDescriptions**: Nested descriptions

### 3.2 Actions

Actions represent user intentions. Key action types:

| Action | Purpose | ROS-Specific |
|--------|---------|--------------|
| `ExecuteProcess` | Run arbitrary processes | No |
| `Node` | Run ROS nodes with package/executable lookup | Yes |
| `LifecycleNode` | Run lifecycle-aware ROS nodes | Yes |
| `ComposableNodeContainer` | Container for composable nodes | Yes |
| `LoadComposableNodes` | Dynamically load nodes into containers | Yes |
| `IncludeLaunchDescription` | Include other launch files | No |
| `DeclareLaunchArgument` | Declare arguments with defaults | No |
| `SetLaunchConfiguration` | Set configuration variables | No |
| `SetEnvironmentVariable` | Modify environment | No |
| `GroupAction` | Group and scope actions | No |
| `RegisterEventHandler` | Register event handlers | No |

**Action Base Class:**
```python
class Action(LaunchDescriptionEntity):
    def __init__(self, *, condition: Optional[Condition] = None):
        self.__condition = condition

    @staticmethod
    def parse(entity: Entity, parser: Parser) -> Tuple[Type, Dict[str, Any]]:
        """Parse entity and return (ActionClass, kwargs)"""
        raise NotImplementedError()

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """Execute action and return sub-entities"""
        raise NotImplementedError()
```

### 3.3 Substitutions

Substitutions enable **runtime value interpolation** in strings:

**Pattern:** `$(substitution_name arg1 arg2 ...)`

**Common Substitutions:**

| Substitution | Syntax | Example |
|--------------|--------|---------|
| `LaunchConfiguration` | `$(var name)` | `$(var file_path)` |
| `EnvironmentVariable` | `$(env VAR)` or `$(env VAR default)` | `$(env HOME)` |
| `Command` | `$(command cmd)` | `$(command pwd)` |
| `FileContent` | `$(file path)` | `$(file /etc/hostname)` |
| `FindExecutable` | `$(find-exec name)` | `$(find-exec python3)` |
| `FindPackage` | `$(find-pkg-share pkg)` | `$(find-pkg-share demo_nodes_cpp)` |
| `ThisLaunchFile` | `$(this-launch-file)` | Path to current launch file |
| `ThisLaunchFileDir` | `$(this-launch-file-dir)` | Directory of current file |
| `AnonName` | `$(anon name)` | `$(anon talker)` → `talker_12345` |

**Grammar (Lark):**
```lark
?start: fragment_list

fragment_list: (quoted_fragment | unquoted_fragment)*

quoted_fragment: QUOTE_CHAR (ESCAPED_CHAR | substitution | text_fragment)* QUOTE_CHAR
unquoted_fragment: (ESCAPED_CHAR | substitution | text_fragment)+

substitution: "$(" substitution_name (WS+ argument)* ")"

substitution_name: /[a-z0-9_-]+/
argument: /[^\s()]+/
```

**Substitution Base Class:**
```python
class Substitution:
    def perform(self, context: LaunchContext) -> str:
        """Resolve substitution to string"""
        raise NotImplementedError()
```

### 3.4 Conditions

Conditions enable **conditional execution** of actions:

```python
class Condition:
    def __init__(self, predicate: Optional[Callable[[LaunchContext], bool]] = None):
        self.__predicate = predicate

    def evaluate(self, context: LaunchContext) -> bool:
        """Evaluate condition"""
        if self.__predicate is not None:
            return self.__predicate(context)
        return True
```

**Concrete Conditions:**
- `IfCondition(predicate_expression)`: Evaluate expression like `$(var debug)`
- `UnlessCondition(predicate_expression)`: Negate condition
- `LaunchConfigurationEquals(name, value)`: Compare config value
- `LaunchConfigurationNotEquals(name, value)`: Inverse comparison

### 3.5 Launch Context

The runtime context holds state during execution:

```python
class LaunchContext:
    __launch_configurations: Dict[str, str]  # Launch variables
    __environment_stack: List[Dict[str, str]]  # Environment management
    __event_handlers: Deque[EventHandler]  # Registered handlers
    __locals_stack: List[Dict[str, Any]]  # Scoped variables
    # ... async event loop support
```

**Key Operations:**
- `get_launch_configurations()`: Access variables
- `perform_substitution(substitution)`: Resolve substitutions
- `extend_globals()` / `extend_locals()`: Scoped variables
- `register_event_handler()`: Add event handlers

### 3.6 Event System

**Events** signal occurrences during launch:

| Event | When Triggered |
|-------|----------------|
| `ExecutionComplete` | Action finished executing |
| `ProcessStarted` | Process started |
| `ProcessExited` | Process exited |
| `ProcessIO` | Process stdout/stderr |
| `IncludeLaunchDescription` | Launch file included |
| `Shutdown` | Launch shutdown requested |

**Event Handlers** react to events:
- `OnProcessExit(target_action, on_exit=[...])`: React to process exit
- `OnProcessStart(target_action, on_start=[...])`: React to process start
- `OnShutdown(on_shutdown=[...])`: React to shutdown
- `OnIncludeLaunchDescription(on_include=[...])`: React to includes

---

## 4. Frontend Parser Architecture

### 4.1 Plugin System

Parsers are discovered via **entry points**:

```python
# setup.py
entry_points={
    'launch.frontend.parser': [
        'xml = launch_xml:Parser',
        'yaml = launch_yaml:Parser',
    ],
}
```

### 4.2 Parser Base Class

```python
class Parser:
    @classmethod
    def load(cls, file: Union[str, TextIO]) -> Tuple[Entity, 'Parser']:
        """Load file and return (root_entity, parser_instance)"""
        # Detects file type and loads appropriate parser
        pass

    def parse_description(self, entity: Entity) -> LaunchDescription:
        """Parse entity tree into LaunchDescription"""
        pass

    def parse_entity(self, entity: Entity, *, context: Optional[LaunchContext] = None) -> LaunchDescriptionEntity:
        """Parse single entity into Action/Handler"""
        pass
```

### 4.3 Entity Abstraction

**Entity** is an intermediate representation that abstracts XML/YAML differences:

```python
class Entity:
    @property
    def type_name(self) -> str:
        """Entity type (e.g., 'node', 'executable')"""
        raise NotImplementedError()

    def get_attr(self, name: str, *, data_type=str, optional=False, can_be_str=True):
        """Get typed attribute value"""
        raise NotImplementedError()

    @property
    def children(self) -> Optional[List['Entity']]:
        """Get child entities"""
        raise NotImplementedError()
```

**XML Entity Implementation:**
```python
class Entity(launch.frontend.Entity):
    """Wraps xml.etree.ElementTree.Element"""

    def __init__(self, element: ET.Element):
        self.__element = element

    @property
    def type_name(self) -> str:
        return self.__element.tag

    def get_attr(self, name, ...):
        value = self.__element.attrib.get(name)
        # Type coercion with get_typed_value()
        return coerced_value

    @property
    def children(self):
        return [Entity(child) for child in self.__element]
```

**YAML Entity Implementation:**
```python
class Entity(launch.frontend.Entity):
    """Wraps dict/list from yaml.safe_load()"""

    def __init__(self, data: Union[Dict, List], type_name: str):
        self.__data = data
        self.__type_name = type_name

    @property
    def type_name(self) -> str:
        return self.__type_name

    def get_attr(self, name, ...):
        value = self.__data.get(name)
        # Type checking with is_instance_of()
        return value

    @property
    def children(self):
        children_data = self.__data.get('children', [])
        # Each child is dict with single key (type name)
        return [Entity(child_dict[key], key) for child_dict in children_data for key in child_dict]
```

### 4.4 Action Exposure System

Actions register themselves with the parser via decorators:

```python
@expose_action('node')
class Node(ExecuteProcess):
    @staticmethod
    def parse(entity: Entity, parser: Parser) -> Tuple[Type['Node'], Dict[str, Any]]:
        kwargs = {}
        kwargs['package'] = parser.parse_substitution(entity.get_attr('pkg'))
        kwargs['executable'] = parser.parse_substitution(entity.get_attr('exec'))
        kwargs['name'] = parser.parse_substitution(entity.get_attr('name', optional=True))
        # ... extract other attributes
        return Node, kwargs
```

**Flow:**
1. Decorator stores `parse` method in registry: `action_parse_methods['node'] = Node.parse`
2. Parser encounters entity with `type_name='node'`
3. Calls `instantiate_action(entity, parser)`
4. Looks up `action_parse_methods['node']`
5. Calls `Node.parse(entity, parser)` → returns `(Node, kwargs)`
6. Instantiates: `Node(**kwargs)`

---

## 5. XML Launch File Format

### 5.1 File Extensions

`.launch.xml`, `.xml`, `.launch`

### 5.2 Basic Structure

```xml
<launch>
  <!-- Actions and entities -->
</launch>
```

### 5.3 Common Elements

#### Node

```xml
<node pkg="package_name" exec="executable_name"
      name="node_name" namespace="/"
      output="screen" respawn="true" respawn_delay="2.0">
  <param name="param1" value="value1"/>
  <param name="param2" value="42"/>
  <remap from="old_topic" to="new_topic"/>
  <env name="ENV_VAR" value="value"/>
</node>
```

**Attributes:**
- `pkg`: Package name
- `exec`: Executable name
- `name`: Node name (optional, defaults to executable)
- `namespace`: Node namespace (default: `/`)
- `output`: `screen`, `log`, or `both`
- `respawn`: Boolean (default: false)
- `respawn_delay`: Seconds (default: 0.0)

**Child Elements:**
- `<param>`: Inline parameters
- `<remap>`: Topic/service remappings
- `<env>`: Environment variables

#### Executable / Process

```xml
<executable cmd="ls -l -a" cwd="/" name="my_ls"
            shell="true" output="log" emulate_tty="true"
            launch-prefix="$(env LAUNCH_PREFIX '')">
  <env name="var" value="1"/>
</executable>
```

**Attributes:**
- `cmd`: Command with arguments (space-separated or list)
- `cwd`: Working directory
- `name`: Process name
- `shell`: Execute via shell (default: false)
- `output`: Output destination
- `emulate_tty`: Emulate TTY (default: false)
- `launch-prefix`: Prefix for command (e.g., `gdb --args`)

#### Composable Node Container

```xml
<composable_node_container pkg="rclcpp_components"
                           exec="component_container"
                           name="my_container" namespace="/">
  <composable_node pkg="composition" plugin="composition::Talker" name="talker"/>
  <composable_node pkg="composition" plugin="composition::Listener" name="listener">
    <param name="rate" value="10.0"/>
    <remap from="chatter" to="/chat"/>
  </composable_node>
</composable_node_container>
```

**Container Attributes:**
- Same as `<node>` (it's a special node)

**Composable Node Attributes:**
- `pkg`: Package containing plugin
- `plugin`: Plugin class name (e.g., `namespace::ClassName`)
- `name`: Node name
- `namespace`: Node namespace

#### Arguments

```xml
<arg name="file_path" default="/tmp/test.txt" description="Path to file"/>
<arg name="debug" default="false"/>
```

**Attributes:**
- `name`: Argument name
- `default`: Default value (optional)
- `description`: Help text (optional)

**Usage:**
```xml
<node ... name="$(var node_name)"/>
```

#### Set Configuration

```xml
<set_launch_configuration name="use_sim_time" value="true"/>
```

#### Include

```xml
<include file="$(find-pkg-share pkg_name)/launch/other.launch.xml">
  <arg name="param1" value="value1"/>
</include>
```

#### Group

```xml
<group scoped="true">
  <push-ros-namespace namespace="ns"/>
  <node .../>
  <node .../>
</group>
```

**Attributes:**
- `scoped`: Create new scope (default: true)
- `if`: Condition expression
- `unless`: Negated condition

#### Let (Local Variable)

```xml
<let name="var_name" value="$(env HOME)/config"/>
<node ... args="--config $(var var_name)"/>
```

### 5.4 Attribute Separators

XML supports list separation with `-sep` suffix:

```xml
<node pkg="demo" exec="node" args-sep=";" args="arg1;arg2;arg3"/>
```

Equivalent to:
```xml
<node pkg="demo" exec="node" args="arg1 arg2 arg3"/>
```

### 5.5 Type Coercion

XML parser automatically coerces types:
- `"true"` / `"false"` → `bool`
- `"42"` → `int` (if requested)
- `"3.14"` → `float` (if requested)

---

## 6. YAML Launch File Format

### 6.1 File Extensions

`.launch.yaml`, `.launch.yml`, `.yaml`, `.yml`, `.launch`

### 6.2 Basic Structure

Root element must be a dictionary with **single key** (typically `launch`):

```yaml
launch:
  - action1:
      attribute: value
  - action2:
      attribute: value
```

**Important:** YAML format expects **lists** as children, where each item is a dict with a single key (the action type).

### 6.3 Common Elements

#### Node

```yaml
launch:
  - node:
      pkg: package_name
      exec: executable_name
      name: node_name
      namespace: /
      output: screen
      respawn: true
      respawn_delay: 2.0
      param:
        - name: param1
          value: value1
        - name: param2
          value: 42
      remap:
        - from: old_topic
          to: new_topic
      env:
        - name: ENV_VAR
          value: value
```

#### Executable

```yaml
launch:
  - executable:
      cmd: ls -l -a -s
      cwd: /
      name: my_ls
      shell: true
      output: log
      emulate_tty: true
      launch-prefix: $(env LAUNCH_PREFIX '')
      env:
        - name: var
          value: '1'
```

#### Arguments

```yaml
launch:
  - arg:
      name: file_path
      default: /tmp/test.txt
      description: 'Path to file'
```

#### Set Configuration

```yaml
launch:
  - set_launch_configuration:
      name: use_sim_time
      value: 'true'
```

#### Include

```yaml
launch:
  - include:
      file: $(find-pkg-share pkg_name)/launch/other.launch.yaml
      arg:
        - name: param1
          value: value1
```

#### Group

```yaml
launch:
  - group:
      scoped: true
      children:
        - push_ros_namespace:
            namespace: ns
        - node:
            pkg: demo
            exec: talker
```

### 6.4 Type Handling

YAML does **NOT** perform type coercion. Values must be correct types in YAML:

```yaml
# Correct
respawn: true       # Boolean
rate: 10.0          # Float
count: 42           # Integer
name: 'my_node'     # String

# Incorrect (would be strings)
respawn: 'true'     # String, not boolean!
rate: '10.0'        # String, not float!
```

### 6.5 Substitutions in YAML

Substitutions must be **quoted strings**:

```yaml
launch:
  - node:
      pkg: demo_nodes_cpp
      exec: talker
      name: $(var node_name)  # Correct
```

---

## 7. Composable Nodes (Component System)

### 7.1 Concept

Composable nodes are **shared library plugins** loaded into a container process at runtime. This enables:
- **Reduced overhead**: Multiple nodes in single process
- **Intra-process communication**: Zero-copy message passing
- **Dynamic composition**: Load/unload nodes at runtime

### 7.2 Container Launch

**XML:**
```xml
<composable_node_container pkg="rclcpp_components"
                           exec="component_container_mt"
                           name="my_container" namespace="/">
  <!-- Composable nodes here -->
</composable_node_container>
```

**YAML:**
```yaml
launch:
  - composable_node_container:
      pkg: rclcpp_components
      exec: component_container_mt
      name: my_container
      namespace: /
      composable_node:
        - pkg: composition
          plugin: composition::Talker
          name: talker
```

**Python:**
```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name='my_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='composition',
            plugin='composition::Talker',
            name='talker'
        ),
        ComposableNode(
            package='composition',
            plugin='composition::Listener',
            name='listener'
        )
    ]
)
```

### 7.3 Component Description

**ComposableNode Structure:**
- `package`: Package containing plugin
- `plugin`: Fully qualified class name (e.g., `namespace::ClassName`)
- `name`: Node name
- `namespace`: Node namespace
- `parameters`: List of parameter dicts or YAML file paths
- `remappings`: List of (from, to) tuples
- `extra_arguments`: Additional plugin-specific args

### 7.4 Dynamic Loading

Use `LoadComposableNodes` action to load nodes into existing containers:

**XML:**
```xml
<load_composable_nodes target_container="/my_container">
  <composable_node pkg="composition" plugin="composition::Talker" name="talker"/>
</load_composable_nodes>
```

**Python:**
```python
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

load_nodes = LoadComposableNodes(
    target_container='/my_container',
    composable_node_descriptions=[
        ComposableNode(package='composition', plugin='composition::Talker', name='talker')
    ]
)
```

---

## 8. Key Implementation Details

### 8.1 Parser Selection

File extension determines parser:
- `.xml` → XML parser
- `.yaml`, `.yml` → YAML parser
- `.py` → Python (dynamic execution, not parsed)
- `.launch` → Try all parsers (XML first, then YAML)

### 8.2 Substitution Resolution

**Two-Phase Processing:**
1. **Parse Time**: Substitutions parsed into `List[Substitution]` objects
2. **Execution Time**: `perform(context)` called to resolve values

**Example:**
```python
# Parse time
cmd_attr = entity.get_attr('cmd')  # "cat $(var file_path)"
cmd_substitutions = parser.parse_substitution(cmd_attr)
# → [TextSubstitution('cat '), LaunchConfiguration('file_path')]

# Execution time
cmd_str = perform_substitutions(context, cmd_substitutions)
# → "cat /tmp/myfile.txt"
```

### 8.3 Parameter Handling

**Parameter Sources:**
1. Inline parameters: `<param name="x" value="1"/>`
2. Parameter files: `<param from="/path/to/params.yaml"/>`
3. Global parameters: `<set_parameter name="use_sim_time" value="true"/>`

**Parameter File Format (ROS 2 YAML):**
```yaml
/**:
  ros__parameters:
    param1: value1
    param2: 42
    nested:
      param3: true
```

### 8.4 Error Handling

**Validation:**
- `assert_entity_completely_parsed(entity)`: Ensures all attrs/children consumed
- Raises `ValueError` for unexpected attributes
- Raises `InvalidFrontendLaunchFileError` for parse errors

**Common Errors:**
- Missing required attributes
- Invalid type coercion
- Unknown action type
- Malformed substitutions

---

## 9. Critical Considerations for Rust Parser

### 9.1 Must-Have Features

1. **Substitution Grammar Parser**: Need Lark equivalent or regex-based parser
2. **Entity Abstraction**: Unified interface for XML/YAML
3. **Type System**: Handle type coercion (XML) and checking (YAML)
4. **Attribute Handling**: Support `-sep` suffixes for lists
5. **Children Handling**: Different semantics for XML vs YAML

### 9.2 Semantic Differences

| Aspect | XML | YAML |
|--------|-----|------|
| Type Coercion | Yes | No |
| Attribute Lists | `-sep` suffix | Native YAML lists |
| Children | Direct elements | `children` key or list items |
| Defaults | More forgiving | Stricter types |

### 9.3 Minimal Viable Parser

For initial implementation, focus on:
1. **Node action**: Most common use case
2. **Basic substitutions**: `var`, `env`, `find-pkg-share`
3. **Include action**: For modular launches
4. **Argument declarations**: For parameterization
5. **Simple conditions**: `if` / `unless`

Defer for later:
- Event handlers
- Complex substitutions (`command`, `file-content`)
- Lifecycle nodes
- All exotic actions

### 9.4 Testing Strategy

**Comparison Testing:**
1. Parse same launch file with ROS 2 launch (Python)
2. Parse same launch file with Rust parser
3. Compare generated `record.json` structures
4. Verify semantic equivalence

**Unit Testing:**
- Substitution parsing
- XML entity handling
- YAML entity handling
- Type coercion/checking
- Error cases

---

## 10. References

- **ROS 2 Launch Docs**: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/
- **launch Package**: `/home/aeon/repos/play_launch_parser/external/launch`
- **launch_ros Package**: `/home/aeon/repos/play_launch_parser/external/launch_ros`
- **Demo Examples**: `/home/aeon/repos/play_launch_parser/external/demos`
