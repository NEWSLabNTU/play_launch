---
id: 104
title: "A container started as a plain Node was not a container"
status: resolved
type: defect
severity: high
---

# 0104 - A container started as a plain Node was not a container

**Repo:** `play_launch` up to 0.41.2
**Affects:** the model's `is_container` bit
(`ros-launch-resolve` `ros/model_builder.rs`)
**Found:** 2026-11-21, running Nav2's `tb3_simulation_launch.py
use_composition:=True` on Humble under play_launch

## What happened

Nav2's `bringup_launch.py` on Humble starts its container as a plain node,
`Node(package='rclcpp_components', executable='component_container_isolated',
name='nav2_container')`, and loads every server into it with
`LoadComposableNodes(target_container='nav2_container')`. `ros2 launch`
serves that through the process's LoadNode service and loaded all eleven.

The model set `is_container` only for a `ComposableNodeContainer` (or
`<node_container>`), so play_launch spawned `nav2_container` as an ordinary
node, registered no container, and every composable was
`Blocked(ContainerNotFound)`. Startup never completed:

    Startup still incomplete after 39s: 11 composable(s) are BLOCKED because
    their container is not running

Both parsers recorded the node the same way, so the parity gate could not
see it.

## Fix

A named plain node whose package is `rclcpp_components` and whose executable
is `component_container`, `component_container_mt` or
`component_container_isolated` is a container in the model. The rule reads
what the process runs, never composable `container=` references, so a
regular node whose name collides with a dangling composable target is still
a node (the 46.3b rule). An unnamed one stays a node, since composables find
a container by name. Being a container, it is replaced under
`--container-mode observable|isolated` as any other is.

Nav2's composed bringup now loads 11/11 in about 3 s.

## Tests

- `model_builder::tests::plain_node_running_a_component_container_is_a_container`
- `tests/tests/load_policy.rs::test_composables_load_into_a_container_started_as_a_plain_node`
  (fixture `parallel_loading/launch/plain_node_container.launch.xml`), in
  `observable`, `isolated` and `stock`.
