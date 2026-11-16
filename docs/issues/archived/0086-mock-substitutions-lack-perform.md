---
id: 86
title: "The Rust parser's mock substitutions have no `perform()`, so `context.perform_substitution(EnvironmentVariable(...))` fails"
status: resolved
type: bug
severity: high
---

# 0086 - Mock substitutions lack `perform()`; Autoware 2.1.0 does not load

**Repo:** `play_launch` at `ed2b53f8` (installed 0.20.0)
**Affects:** `pyexec`, the mock `launch` module, and every Autoware 2.1.0
launch under the default Rust parser
**Found by:** the golf cart's Autoware 1.5.0 → 2.1.0 migration, 2026-11-17

## Symptom

    Error: Rust parser error while parsing <file>: Python error: Python error:
    AttributeError: 'launch.substitutions.EnvironmentVariable' object has no
    attribute 'perform'

Autoware 2.1.0's stock `autoware_launch planning_simulator.launch.xml` fails
this way before anything spawns, as does any stack built on 2.1.0. The
triggers are 2.1.0's new agnocast files, which almost every 2.1.0 launch
includes:

- `autoware_component_state_monitor/launch/component_state_monitor.launch.py:64`:
  `context.perform_substitution(EnvironmentVariable("ENABLE_AGNOCAST", default_value="0"))`
- `autoware_agnocast_wrapper/launch/agnocast_env.launch.py`: `EnvironmentVariable("LD_PRELOAD", default_value="")`
  and `EnvironmentVariable("ENABLE_AGNOCAST", ...)` as a `DeclareLaunchArgument` default.

`--parser python` loads the same launch: the stock planning simulator ran
97 entities (48 nodes, 8 containers, 41 composable nodes) for 4 minutes.

## Reproduction

```python
# env_perform.launch.py
from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from launch.substitutions import EnvironmentVariable


def _setup(context):
    enabled = context.perform_substitution(
        EnvironmentVariable("ENABLE_AGNOCAST", default_value="0"))
    return [LogInfo(msg="agnocast=" + enabled)]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
```

```
$ play_launch resolve env_perform.launch.py -o out.yaml            # rc=1, the error above
$ play_launch resolve --parser python env_perform.launch.py -o out.yaml   # rc=0
```

## Cause

`LaunchContextView::perform_substitution`
(`src/ros-launch-resolve/parser/crates/pyexec/src/api/utils.rs`) calls
`sub.perform(context)`. The mock `EnvironmentVariable` has no `perform`
method; it is only converted to a string where `pyobject_to_string` meets it.
The other mock substitutions (`LaunchConfiguration`, `FindPackageShare`,
`PathJoinSubstitution`, `TextSubstitution`, ...) probably have the same gap
when handed to `perform_substitution` or `perform_substitutions`: worth
checking all of them, not just this one.

Related: #0073 (user-defined `launch.Substitution` subclasses) is the other
half of "a substitution performed from Python".

## Direction

Give every mock substitution a `perform(context)` that resolves it the way
`pyobject_to_string` already does, the environment read from the
traverser's environment, including any `SetEnvironmentVariable` in scope.
Then add a test that loads the snippet above plus 2.1.0's
`agnocast_env.launch.py` pattern: a `DeclareLaunchArgument` whose default is
an `EnvironmentVariable`.

## Workaround

`--parser python`. The golf cart has to use it for 2.1.0 until this is fixed.

## Resolution (2026-11-17)

Every mock substitution in the Python half (`pyexec`) now has a
`perform(context)` with stock `launch`/`launch_ros` semantics, and
`context.perform_substitution` performs strictly as stock does (a string or a
list is accepted too, performed as `perform_substitutions` would). The value
comes from the parser's own substitution engine
(`Substitution::resolve`, the evaluator `$(...)` in XML and YAML uses), via
`utils::resolve_with_engine`/`resolve_call`, so a Python and an XML spelling
of one substitution cannot disagree:

- `EnvironmentVariable`: the launch environment (process environment plus
  every `SetEnvironmentVariable` executed so far), else the default; unset
  with no default raises `SubstitutionFailure`. `name` and `default_value`
  are now `SomeSubstitutionsType`, as in `launch`.
- `TextSubstitution`: its text. `ThisLaunchFileDir`/`ThisLaunchFile`: the
  file being run (it crosses the C ABI in `ContextState.current_file`),
  falling back to the host-rewritten token only when none is set.
- `AnonName`: the engine's `$(anon)`, one name per launch.
- `Not`/`And`/`Or` (and the new `AnySubstitution`/`AllSubstitution`),
  `EqualsSubstitution`/`NotEqualsSubstitution`, `IfElseSubstitution`: the
  engine's boolean rules (`true`/`false`/`1`/`0`, anything else
  `SubstitutionFailure`; equality of booleans and numbers by value).
- `PathJoinSubstitution`: `str(pathlib.Path(*components))`, so an absolute
  component restarts the path and a list component is concatenated.
- `FindPackage` (now also `FindPackagePrefix`), `ExecutableInPackage`,
  `FindExecutable`, `Parameter`, `LaunchLogDir`: resolved by the engine
  instead of returning a `$(...)` token. Two of those tokens
  (`$(find-executable)`, `$(launch-log-dir)`) named nothing the grammar knew
  and reached the record literally.
- `LocalSubstitution` exists (it was an `ImportError`); it evaluates
  `context.locals.<expression>` as stock does.
- `SubstitutionFailure` is exported from `launch.substitutions` (a
  `RuntimeError`, as in `launch`); `LaunchConfiguration` raises it too.

No C ABI change: nothing new crosses the boundary.

Tests: `execution_semantics.rs`
(`an_environment_variable_performs_from_an_opaque_function`, the
reproduction above; `every_substitution_performs_from_python`, 24 nodes
including the `DeclareLaunchArgument(default_value=EnvironmentVariable(...))`
pattern read by a later node and an included file), and the stock-parity case
`tests/stock_parity/cases/env_perform`. Expectations are stock Humble's
(`stock_oracle.py`). With the source change stashed, the reproduction fails
with this issue's `AttributeError`.
