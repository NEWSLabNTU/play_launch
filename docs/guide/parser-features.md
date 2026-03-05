# Parser Feature Coverage

Comprehensive reference for the three launch file formats supported by `play_launch_parser`.

## Action Testing Status

Legend: **T** = integration tests, **U** = unit tests, **TODO** = supported by ROS 2 but not yet implemented in our parser.

All YAML action types below are supported by the official ROS 2 YAML frontend via the shared `@expose_action` registration system (source: `external/launch/` and `external/launch_ros/`).

| Action                 | XML                 | Python             | YAML                        | IR / WASM |
|------------------------|---------------------|--------------------|-----------------------------|-----------|
| `arg` / declare        | `<arg>` T U         | `DeclareArg` T     | `- arg:` T                  | T U       |
| `node`                 | `<node>` T U        | `Node()` T         | `- node:` T                 | T U       |
| `include`              | `<include>` T U     | `IncludeLaunch` T  | `- include:` T              | T U       |
| `group`                | `<group>` T U       | `GroupAction` T    | `- group:` T                | T U       |
| `let`                  | `<let>` T U         | `SetLaunchConf` T  | `- let:` T                  | T U       |
| `set_env`              | `<set_env>` T U     | `SetEnvVar` T      | `- set_env:` T              | T U       |
| `unset_env`            | `<unset_env>` T     | `UnsetEnvVar` T    | `- unset_env:` T            | T U       |
| `push-ros-namespace`   | `<push-ros-ns>` T U | `PushROSNs` T      | `- push-ros-ns:` T          | T U       |
| `set_parameter`        | `<set_param>` T U   | `SetParameter` T   | `- set_parameter:` T        | T U       |
| `set_remap`            | `<set_remap>` T U   | `SetRemap` T       | `- set_remap:` T            | T U       |
| `executable`           | `<executable>` T U  | `ExecuteProcess` T | `- executable:` T           | T U       |
| `node_container`       | `<node_cont>` T U   | `ComposableNC` T   | `- node_container:` T       | T U       |
| `load_composable_node` | `<load_cn>` T U     | `LoadCN` T         | `- load_composable_node:` T | T U       |
| `if`/`unless`          | attrs T U           | `condition=` T     | keys T                      | T         |
| `OpaqueFunction`       | —                   | `OpaqueFunc` T     | —                           | T         |
| **Substitutions**      | all 10 types T U    | all types T        | via XML engine              | T U       |
| **Scoping**            | group/include T     | isolated ctx T     | parent scope T              | T         |

### Additional YAML actions in official ROS 2 (not yet implemented)

These actions are registered in the official ROS 2 frontend but not currently needed by Autoware/AutoSDV:

| Action                   | Source package | ROS 2 class                 |
|--------------------------|----------------|-----------------------------|
| `lifecycle_node`         | `launch_ros`   | `LifecycleNode`             |
| `set_parameters_from_file` | `launch_ros` | `SetParametersFromFile`     |
| `set_use_sim_time`       | `launch_ros`   | `SetUseSimTime`             |
| `set_ros_log_dir`        | `launch_ros`   | `SetROSLogDir`              |
| `ros_timer`              | `launch_ros`   | `ROSTimer`                  |
| `bool_arg`               | `launch`       | `DeclareBooleanLaunchArgument` |
| `append_env`             | `launch`       | `AppendEnvironmentVariable` |
| `rep_env` / `replace_env` | `launch`      | `ReplaceEnvironmentVariables` |
| `reset_env`              | `launch`       | `ResetEnvironment`          |
| `reset`                  | `launch`       | `ResetLaunchConfigurations` |
| `for` / `for_each`       | `launch`       | `ForLoop` / `ForEach`       |
| `log` / `log_info` / etc. | `launch`      | `Log` / `LogInfo` / etc.    |
| `timer`                  | `launch`       | `TimerAction`               |
| `shutdown`               | `launch`       | `Shutdown`                  |

### Substitution types (shared across formats)

| Substitution             | Unit tests | Integration tests |
|--------------------------|------------|-------------------|
| `$(var name)`            | 8          | 12+               |
| `$(env VAR)`             | 3          | 4                 |
| `$(optenv VAR default)`  | 4          | 3                 |
| `$(find-pkg-share pkg)`  | 2          | 2                 |
| `$(find-pkg-prefix pkg)` | 1          | 1                 |
| `$(dirname)`             | 2          | 1                 |
| `$(filename)`            | 2          | 0                 |
| `$(anon name)`           | 3          | 1                 |
| `$(eval 'expr')`         | 16         | 2                 |
| `$(command 'cmd')`       | 10         | 2                 |

## XML Parser

**Entry point**: `traverse_entity()` in `src/traverser/entity.rs`

Handles the full ROS 2 XML launch specification. All elements support `if=` and `unless=` condition attributes evaluated via `should_process_entity()`.

### Substitutions

| Substitution             | Example                          |
|--------------------------|----------------------------------|
| `$(var name)`            | Launch configuration variable    |
| `$(env VAR)`             | Environment variable (required)  |
| `$(optenv VAR default)`  | Environment variable (optional)  |
| `$(find-pkg-share pkg)`  | Package share directory          |
| `$(find-pkg-prefix pkg)` | Package prefix directory         |
| `$(dirname)`             | Directory of current launch file |
| `$(filename)`            | Path of current launch file      |
| `$(anon name)`           | Anonymous unique name            |
| `$(eval 'expr')`         | Python expression evaluation     |
| `$(command 'cmd')`       | Shell command output             |

### Scoping

- `<group>`: saves/restores scope; optional `ns=` pushes namespace
- `<include>`: creates isolated child context (variables don't leak to parent)
- `<let>`: sets variable in current scope (sequential resolution)
- `<push-ros-namespace>` / `<pop-ros-namespace>`: modifies namespace stack

## Python Parser

**Entry point**: `execute_python_file()` in `src/traverser/python_exec.rs`

Executes Python launch files via PyO3. Uses a capture-on-construction pattern: Python objects register themselves with global state, which is collected after execution and propagated through the include chain.

### Supported APIs

- `launch`: `LaunchDescription`, `DeclareLaunchArgument`, `SetLaunchConfiguration`, `GroupAction`, `IncludeLaunchDescription`, `ExecuteProcess`, `SetEnvironmentVariable`, `UnsetEnvironmentVariable`, `OpaqueFunction`, `Condition`, `IfCondition`, `UnlessCondition`
- `launch_ros`: `Node`, `ComposableNodeContainer`, `ComposableNode`, `LoadComposableNode`, `PushROSNamespace`, `SetParameter`, `SetParametersFromFile`, `SetRemap`
- `launch.substitutions`: `LaunchConfiguration`, `EnvironmentVariable`, `FindPackageShare`, `FindPackagePrefix`, `PathJoinSubstitution`, `TextSubstitution`, `PythonExpression`, `Command`, `AnonName`
- `launch.frontend`: `Parser` (for XML includes from Python)
- `launch.utilities`: `perform_substitutions`, `normalize_to_list_of_substitutions`

### Scoping

- Python launch files create an isolated execution context
- Captures (nodes, containers, includes) are merged into the parent traverser after execution
- `SetLaunchConfiguration` in Python writes to the child context; propagated back via capture merge

## YAML Parser

**Entry point**: `process_yaml_launch_file()` in `src/traverser/yaml.rs`

Handles the ROS 2 YAML launch format. Each YAML launch file has a top-level `launch:` key containing a sequence of action mappings.

### Format

```yaml
launch:
- arg:
    name: my_arg
    default: "value"
    description: "optional description"

- include:
    file: "$(find-pkg-share pkg)/launch/file.launch.xml"
    arg:
    - name: param1
      value: value1

- group:
    if: "$(var condition)"
    ns: "/namespace"
    children:
    - node:
        pkg: my_package
        exec: my_node
        name: node_name

- node:
    pkg: my_package
    exec: my_executable
    name: my_node
    namespace: /ns
    param:
    - name: param_name
      value: param_value
    - from: /path/to/params.yaml
    remap:
    - from: input
      to: /remapped_input
    args: "--extra-flag"
    output: screen
    respawn: "true"
    respawn_delay: "2.0"

- let:
    name: variable
    value: "resolved_value"

- set_env:
    name: MY_VAR
    value: "my_value"

- unset_env:
    name: MY_VAR

- push-ros-namespace:
    namespace: /my_ns
```

### Conditions

Any YAML action mapping can include `if:` or `unless:` keys:

```yaml
- group:
    if: "$(eval '\"$(var mode)\" == \"sim\"')"
    children:
    - include:
        file: "$(find-pkg-share pkg)/launch/sim.launch.xml"
```

### Scoping

**YAML modifies parent scope** — unlike XML `<include>` which creates isolated child contexts, YAML launch files operate directly on the caller's context. This is critical for the preset pattern used by Autoware:

```xml
<!-- XML file includes YAML preset, then uses variables it declared -->
<include file="preset.yaml"/>  <!-- declares perception_mode -->
<include file="perception.launch.xml">
  <arg name="mode" value="$(var perception_mode)"/>
</include>
```

Groups within YAML use save/restore scope (same as XML groups).

### Value types

YAML values may be strings, booleans, or numbers. The parser converts non-string types to their string representation (`true` → `"true"`, `42` → `"42"`, `5.0` → `"5.0"`) before substitution resolution.

## IR and WASM Pipeline

**Entry point**: `analyze_launch_file()` → `LaunchProgram` IR → `compile_to_wasm()` → WASM → `execute_wasm()` → `RecordJson`

The parser has a two-stage architecture. The IR preserves full structure (conditions, substitution expressions, groups, includes) without evaluating them. The WASM pipeline compiles IR to wasmtime-executable bytecode.

## Test Summary

| Category                                         | Count   |
|--------------------------------------------------|---------|
| Parser unit tests (`src/`)                       | 237     |
| XML/YAML tests (`tests/xml_tests.rs`)            | 63      |
| Python tests (`tests/python_tests.rs`)           | 36      |
| Edge case tests (`tests/edge_cases.rs`)          | 18      |
| IR builder tests (`tests/ir_tests.rs`)           | 20      |
| IR eval tests (`tests/ir_eval_tests.rs`)         | 22      |
| Performance tests (`tests/integration_tests.rs`) | 3       |
| WASM round-trip tests                            | 18      |
| **Total**                                        | **417** |

## Autoware Validation

The parser is validated against the full Autoware planning_simulator stack:

- **46/46** nodes captured (100%)
- **15/15** containers captured (100%)
- **54/54** composable nodes captured (100%)
- AutoSDV `logging_simulation.launch.yaml`: 44 nodes, 15 containers, 84 load_nodes

Test workspaces under `tests/fixtures/`: `autoware/`, `simple_test/`, `sequential_loading/`, `concurrent_loading/`, `container_events/`, `parallel_loading/`.
