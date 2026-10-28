---
id: 77
title: "The Python dumper drops <set_env> / SetEnvironmentVariable: a node gets only its own <env>"
status: open
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

## Fix direction

In each visitor that records a process, start `env` from the entries of
`context.environment` that differ from (or are absent in) the environment the
dumper started with, then lay `additional_env` over it; an `<unset_env>` of a
variable the dumper's own environment has needs the same treatment as the
Rust side gives it. Add the table above as a parity fixture
(`just compare-dumps`), with an `<unset_env>` case.

## Workaround

Use the default Rust parser, or `ros2 launch`, for a launch file whose
processes depend on `<set_env>`. (Since 0.20.0 the Rust parser also loads
`nav2_bringup`'s `RewrittenYaml`, #0073, which was a reason to fall back to
`--parser python` on such files.)
