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
| `timer`                | `<timer>` T U       | `TimerAction` ¹    | `- timer:` T                | —         |
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
| `shutdown`               | `launch`       | `Shutdown`                  |

¹ **`<timer>` on the Python frontend carries the delay too, by object
identity.** Python constructs a `Node(...)` — and the parser captures it —
before the enclosing `TimerAction` ever sees it, so the delay cannot be read
off capture ORDER: a node built outside the argument list and passed in by
name breaks any "last N captures" rule. It is read off the objects instead.
Each capturing mock records the span of captures its own constructor
appended, and the timer walks its `actions` and stamps exactly those, through
`GroupAction`, nested timers (which add), helper functions and
`OpaqueFunction`. `launch_ros`'s `RosTimer` is handled the same way; it used
to discard its period with no diagnostic at all.

Four shapes still cannot be attributed, and each is reported by name in
`dropped_actions` (so `check` refuses) rather than guessed at: the same action
object in two unrelated timers, or both in a timer and started directly
(`ros2 launch` starts it twice, the model holds it once); a `period` that is
not a number when the file is read; and a child this parser cannot see into,
such as an `IncludeLaunchDescription`. See
`docs/design/python-timer-delay-attribution.md`.

A timer's delay reaches the SystemModel as
`structure.nodes.<fqn>.start_delay_secs` (seconds before the FIRST start,
accumulated across nested timers, distinct from `respawn_delay`), and
`play_launch up` honours it for every kind of member. A node or a container
waits out the deadline before it is spawned. A composable node has no process
of its own, so its container holds back that one LoadNode request until the
same deadline and issues it then — the container itself, and its undelayed
composables, come up on the usual path meanwhile. Nothing about a `<timer>` is
reported in `meta.diagnostics` any more; the note that used to name every
delayed composable was there for exactly as long as that last case was
unhonoured.

Every deadline is measured from ONE instant per launch, as ROS measures a
`TimerAction`, so members sharing a `<timer>` start together however much
startup bookkeeping separated them — and a respawn or a container restart does
not wait the delay a second time.

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

## Where this parser accepts more than `ros2 launch` does

`play_launch` is meant to be a drop-in for `ros2 launch`, so a divergence in
either direction matters. This one runs the dangerous way round: a launch file
that `resolve`, `dump` and `check` all accept can still fail under `ros2 launch`.

`respawn_delay` is the known case. ROS 2's own XML frontend parses it eagerly as
a float, so a substitution reaches `float()` as a `Substitution` object and the
launch dies at start-up:

```xml
<arg name="d" default="2.0"/>
<node pkg="demo_nodes_cpp" exec="talker" name="t"
      respawn="true" respawn_delay="$(var d)"/>
```

```
$ ros2 launch gap.launch.xml
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
 - TypeError: '<' not supported between instances of 'str' and 'float'

$ play_launch dump launch gap.launch.xml
SystemModel: system_model.yaml (1 nodes, 0 topics, 0 tier(s), 0 warning(s))
  respawn_delay: 2.0        # evaluated correctly, as a float

$ play_launch check gap.launch.xml ; echo $?
0
```

This parser evaluates the substitution first and then converts, which is the
behaviour one would want — but it means `check` passing is **not** sufficient
evidence that `ros2 launch` will start the file. Note `respawn` itself takes a
substitution under both parsers; it is only the typed numeric attribute that
diverges.

Consequence for anyone using `check` as a CI gate: it verifies this parser can
resolve the tree, not that every other launch implementation can. Where a
project must run under both, it is worth keeping a `ros2 launch --show-args`
or a real start-up in the same gate.

Found while migrating a robot's launch tree to `play_launch`, where the file
dumped clean seventeen nodes and then failed to start.

### A second case: a lazily-evaluated `<timer period>` in an included file

Related, and worse, because nothing reports it at all.

`TimerAction` in ROS 2 resolves `period` when the timer *fires*, not when the
launch description is built. If the period is a substitution defined by an
`<arg>` in an included file, that file's scope has usually been popped by then:

```xml
<!-- navigation.launch.xml, included from a parent -->
<arg name="server_delay" default="6.0"/>
<timer period="$(var server_delay)">
  <node pkg="nav2_controller" exec="controller_server" name="controller_server"/>
</timer>
```

```
$ ros2 launch parent.launch.xml
Task exception was never retrieved
future: <Task finished coro=<TimerAction._wait_to_fire_event() ...>
launch.substitutions.substitution_failure.SubstitutionFailure:
    launch configuration 'server_delay' does not exist
```

asyncio swallows the exception, `ros2 launch` reports success, and the timer's
children never start — indefinitely. The same file launched directly works,
because its scope is still alive.

This parser models those children as present, which is defensible: the launch
file does declare them, and a static model cannot know the scope will be gone
at fire time. But the consequence for a user is that `dump` and `check` agreeing
is **not** evidence the nodes will start.

Two things would help, neither implemented here:

- A diagnostic when a `<timer period>` is a substitution referring to a
  configuration declared in the same included file — the shape that fails.
- A note in `check`'s output that timer children are modelled statically.

Found on a robot where the model showed seventeen nodes, `check` exited 0, and
eight of them never started.
