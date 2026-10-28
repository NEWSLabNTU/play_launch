---
id: 77
title: "The Python dumper drops <set_env> / SetEnvironmentVariable: a node gets only its own <env>"
status: resolved
type: defect
severity: high
---

# 0077 - The Python dumper drops `<set_env>`

**Repo:** `play_launch` at `92d75000` (seen on 0.16.0 and 0.20.0)
**Affects:** `python/play_launch/dump/visitor/node.py`,
`composable_node_container.py`, `bare_execute_process.py`

## Symptom

A node launched with `--parser python` runs without the variables a
`<set_env>` (`SetEnvironmentVariable`) before it set; only its own `<env>`
(`additional_env`) reaches the process. `ros2 launch` and the default Rust
parser both pass them.

Found with SROS2: a robot gives each part its enclave with
`ROS_SECURITY_ENCLAVE_OVERRIDE`, one node by `<env>`, the Nav2 includes by a
`<set_env>` in a `<group>` around them. Under `--parser python` every Nav2
process ran in the default enclave, where the permissions refused its
velocity topics.

## Reproduce

`enclaves.launch.xml` (Jazzy; `ROS_SECURITY_ENCLAVE_OVERRIDE=/robot` exported
before the launch, so a lost value reads `/robot`):

```xml
<launch>
  <node pkg="tf2_ros" exec="static_transform_publisher" name="gate_node" args="--frame-id a --child-frame-id b">
    <env name="ROS_SECURITY_ENCLAVE_OVERRIDE" value="/gate"/>
  </node>
  <group>
    <set_env name="ROS_SECURITY_ENCLAVE_OVERRIDE" value="/navigation"/>
    <node pkg="tf2_ros" exec="static_transform_publisher" name="nav_node" args="--frame-id a --child-frame-id e"/>
  </group>
  <group>
    <set_env name="ROS_SECURITY_ENCLAVE_OVERRIDE" value="/navigation"/>
    <include file="inner.launch.xml"/>   <!-- one node, nav_xml_node -->
  </group>
  <group>
    <set_env name="ROS_SECURITY_ENCLAVE_OVERRIDE" value="/navigation"/>
    <include file="inner.launch.py"/>    <!-- one Node, nav_py_node -->
  </group>
  <node pkg="tf2_ros" exec="static_transform_publisher" name="robot_node" args="--frame-id a --child-frame-id c"/>
</launch>
```

The value each process got, read from `/proc/<pid>/environ`:

| Node | set by | `ros2 launch` | `play_launch launch` (Rust) | `--parser python` |
|---|---|---|---|---|
| gate_node | `<env>` | `/gate` | `/gate` | `/gate` |
| nav_node | `<set_env>` in the group | `/navigation` | `/navigation` | `/robot` |
| nav_xml_node | `<set_env>`, included XML | `/navigation` | `/navigation` | `/robot` |
| nav_py_node | `<set_env>`, included Python | `/navigation` | `/navigation` | `/robot` |
| robot_node | nothing | `/robot` | `/robot` | `/robot` |

`play_launch dump --parser python` agrees: only `gate_node` carries an `env`
entry in the model.

## Cause

`_execute_logging_configurations` runs a `SetEnvironmentVariable` with
stock `execute()`, which writes `context.environment`, as launch does. But
`visit_node` (and the container and bare-process visitors) record only
`node.additional_env`; launch's `ExecuteLocal` starts the process with
`context.environment` updated by `additional_env`, and nothing reads
`context.environment` at dump time.

The Rust parser does what launch does: `record/generator.rs` starts from
`context.environment()` (the `<set_env>`s in scope) and lays the node's own
environment over it.

## Fix

`LaunchDump` keeps the dumper's environment from before the launch ran;
`set_env_in_scope(context)` returns what differs from it in
`context.environment` (on Jazzy `os.environ` itself, which `<set_env>` writes
and a scoped group restores). The node, container and bare-process visitors
start from that and lay the process's own `<env>` over it, as launch's
`Executable.prepare` does. A variable the dumper already had with the same
value is not recorded: the replayed process inherits it anyway, and the Rust
parser records only its `<set_env>`s too. An `<unset_env>` of an inherited
variable still cannot be represented (the `setscope` stock-parity case).

Checked on Jazzy:

- the table above with `--parser python`: every node got its value in
  `/proc/<pid>/environ`;
- `tests/fixtures/simple_test/launch/set_env.launch.xml` (seven processes:
  the group, both include kinds, an own `<env>` winning, after the group, an
  `<executable>`): the values match stock `ros2 launch`, both parsers' models
  give them (`scripts/expect_set_env.py`) and `compare_models.py` passes;
  the dumper before the fix: 0 of 7, 7 env differences;
- `tests/stock_parity/cases/setscope` through both parsers: the same env on
  every member.

## Workaround

Use the default Rust parser, or `ros2 launch`, for a launch file whose
processes depend on `<set_env>`. (Since 0.20.0 the Rust parser also loads
`nav2_bringup`'s `RewrittenYaml`, #0073, which was a reason to fall back to
`--parser python` on such files.)
