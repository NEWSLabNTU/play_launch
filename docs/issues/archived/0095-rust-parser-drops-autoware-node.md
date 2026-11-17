---
id: 95
title: "Rust parser drops Autoware 2.1.0's `<autoware_node>` with a WARN, so its nodes are missing from the model"
status: resolved
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

## Resolution (2026-11-17)

**XML and YAML.** `actions/autoware_node.rs` parses `<autoware_node>` as
`AutowareNode.parse` does — `mode`, `target` and the `<extra_arg>` children,
then everything else through `NodeAction`'s own parse — and decides the form
when the action executes, as `AutowareNode.execute` does:

- `registration` reads `autoware_node_plugins/<pkg>__<exec>` from the first
  prefix on `AMENT_PREFIX_PATH` that has it (`<class>;<0|1>`);
- Agnocast is enabled by the `use_agnocast` configuration if set, else by
  `ENABLE_AGNOCAST=1`, both read through the context, which on Humble is
  `os.environ` with `<set_env>` applied; it is used when the node was built
  with it and `mode` is not `rclcpp`, and then any `target` is dropped;
- with a target: a `LoadComposableNodeAction` (plugin from the resource;
  name, namespace, parameters including files, remappings and extra
  arguments) handed to the path `<load_composable_node>` takes; no resource
  is upstream's error;
- without: the `NodeAction` handed to `<node>`'s path, with
  `ENABLE_AGNOCAST` and `LD_PRELOAD` (heaphook first, or the heaphook
  removed) replacing what the launch file's `<env>` said; a heaphook path
  that does not exist is upstream's error;
- an unknown `mode` is upstream's error, in every build.

`attr_spec` has an `autoware_node` table (`<node>`'s plus `mode` and
`target`; children plus `extra_arg`), checked against stock in
`attr_differential.rs`, whose probe now has the extension importable.

**Python.** `from autoware_agnocast_wrapper.actions import AutowareNode`
works unchanged. The class subclasses `launch.Action` and does its work in
`execute(context)`; pyexec's walk skipped any class it did not know. It now
calls a `launch.Action` subclass's own `execute` with the walk's context and
walks what it returns, as `launch` does, so the upstream code runs on the
stand-ins and returns their `Node` / `LoadComposableNodes`. Importing the
module also needed `launch.logging.get_logger` and `expose_action` as a real
decorator (it was `None`).

**Unknown actions are refused.** A tag that is no stock `launch`/
`launch_ros` frontend action (`STOCK_FRONTEND_ACTIONS`, Humble and Jazzy)
and that the parser does not dispatch is now
`Unrecognized entity of the type: <tag>`, launch's own wording, in both
frontends and the IR builder, with a hint to use `--parser python` for a
launch extension. A stock action the parser does not implement (`<log>`,
`<set_parameters_from_file>`, ...) is still dropped, warned and recorded
for `check` to refuse, as before. Measured first: Autoware 1.5.0's planning
simulator drops nothing with either parser (Rust and Python models
equivalent), no `.launch.xml` under `/opt/ros/humble/share` or
`/opt/autoware/1.5.0/share` uses a tag outside the dispatched set (the only
hits are ROS 1 `.launch` files), and the one test that used a made-up tag
for a dropped action (`ir_eval_tests`) now uses `<log>` and also asserts the
refusal.

**Tests.** `tests/autoware_node_tests.rs` (parser crate, 8 tests: both
forms in XML, YAML and Python; Agnocast by environment and by
`use_agnocast` both ways; `mode="rclcpp"`; LD_PRELOAD handling;
`extra_arg`; the missing resource and unknown mode errors; unknown versus
stock tags). Stock parity cases `awnode`, `awnode_yaml`, `awnode_py`,
`awnode_agnocast`, `awnode_use_agnocast`, `awnode_noplugin`, `awnode_mode`
run stock Humble with a copy of the upstream extension
(`tests/fixtures/autoware_node/python`, commit 34c25ee5) and a fake ament
prefix; `run.sh` gained per-case `ENV` and `PYTHON_PARITY` files, the
latter also requiring the Rust and `--parser python` models to match. With
the source reverted, all eight crate tests, the differential and the IR test
fail, and all seven stock cases fail (nodes MISSING, the Python case on
`launch.logging`).

Not verified: the golf cart's 2.1.0 perception launch itself, since
Autoware 2.1.0 is not installed here.
