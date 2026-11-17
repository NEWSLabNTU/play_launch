---
id: 95
title: "Rust parser drops Autoware 2.1.0's `<autoware_node>` with a WARN, so its nodes are missing from the model"
status: open
type: defect
severity: high
---

# 0095 - `<autoware_node>` is not supported by the Rust parser

**Repo:** `play_launch` 0.37.0 (PyPI)
**Affects:** the Rust parser's action dispatch (the path that logs
`Unsupported action type: ...`)
**Found:** golf cart, Autoware 2.1.0 migration, 2026-11-17

## What happens

Autoware 2.1.0 adds a launch action, `<autoware_node>`, registered by
`autoware_agnocast_wrapper`
(`/opt/autoware/2.1.0/lib/python3.10/site-packages/autoware_agnocast_wrapper/actions/autoware_node.py`,
`@expose_action("autoware_node")`). The Rust parser does not know it and
skips each one:

```
WARN Unsupported action type: autoware_node
```

`resolve` exits 0 with the nodes absent. On the golf cart's 2.1.0 perception
launch (`lidar_only`) the Rust model has 133 nodes and the Python model 136;
the three missing are
`/perception/obstacle_segmentation/crop_box_filter`,
`/perception/obstacle_segmentation/common_ground_filter` and
`/perception/object_recognition/detection/voxel_grid_downsample_filter`.
`logging_simulation`: 122 vs 125. A launch started from the Rust model runs
without them, and downstream nodes wait on topics nobody publishes.

Upstream files using it (2.1.0), among others:
`autoware_sensing_launch/launch/pointcloud_crop_boxes.launch.xml`,
`autoware_sensing_launch/launch/blockage_diagnostics.launch.xml`,
`autoware_trajectory_gate/launch/trajectory_gate.launch.xml`,
`autoware_redundancy_switcher_interface/launch/redundancy_switcher_interface.launch.xml`,
`autoware_redundancy_adapi_switcher/launch/sub_default_adapi.launch.xml`.

## What the action means

From its docstring and `parse()`:

- It accepts everything `<node>` accepts (its parser hands the remainder to
  `Node.parse`), plus `mode` (`auto` | `rclcpp`), `target` and
  `<extra_arg>` children.
- `target` given: a composable node loaded into that container (the plugin
  class comes from the ament resource `autoware_node_plugins`, entry
  `<package>__<executable>`, content `<component class>;<ENABLE_AGNOCAST 0|1>`).
  `target` omitted: a standalone process, like `<node>`.
- `extra_arg` applies to the container form only; process attributes
  (`output`, `respawn`, `additional_env`, ...) to the standalone form only.
- `mode` matters only when the node was built with Agnocast and the launch
  enables it (`use_agnocast`, or `ENABLE_AGNOCAST=1`); otherwise rclcpp.

## Reproduce

```xml
<launch>
  <node_container pkg="rclcpp_components" exec="component_container" name="c" namespace=""/>
  <autoware_node pkg="autoware_pointcloud_preprocessor" exec="crop_box_filter_node"
                 name="crop" target="/c">
    <param name="negative" value="true"/>
  </autoware_node>
  <autoware_node pkg="autoware_pointcloud_preprocessor" exec="crop_box_filter_node" name="crop_proc"/>
</launch>
```

with `/opt/autoware/2.1.0/setup.bash` sourced: `--parser python` gives the
container with one composable node plus one standalone process; the Rust
parser gives the container alone and two WARNs.

## Expected

The Rust parser models `<autoware_node>` as the Python action does
(composable with `target`, process without, plugin from the
`autoware_node_plugins` resource, rclcpp unless Agnocast is enabled), so both
parsers agree. Separately, an unknown action type in a launch file is worth
more than a WARN: the model silently loses whatever it would have started.
