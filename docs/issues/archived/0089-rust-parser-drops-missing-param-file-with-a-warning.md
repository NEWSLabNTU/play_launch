---
id: 89
title: "Rust parser drops a node's missing parameter file with a WARN and resolves on, where `ros2 launch` and `--parser python` fail"
status: resolved
type: defect
severity: medium
---

# 0089 - a missing parameter file is a warning, not an error

**Repo:** `play_launch` 0.36.0 at `89b70f92`
**Affects:** the Rust parser's parameter file loading (the path that logs
`Failed to load YAML parameter file ...`)
**Found:** golf cart, Autoware 2.1.0 migration, 2026-11-17

## What happens

A node whose `<param from="...">` names a file that does not exist resolves
anyway under the default parser. The file is skipped with one log line:

```
WARN Failed to load YAML parameter file ./data/autoware_data/lidar_centerpoint/tiny/ml_package.param.yaml: IO error: No such file or directory (os error 2)
```

and the model carries the node with that file absent (`params_files: None`
for `/perception/object_recognition/detection/centerpoint/lidar_centerpoint`).
`resolve` exits 0 with 130 nodes.

The same tree under `--parser python` fails:

```
FileNotFoundError: [Errno 2] No such file or directory: 'data/autoware_data/lidar_centerpoint/tiny/ml_package.param.yaml'
```

which is what `ros2 launch` does too: launch_ros reads parameter files when
it builds the node's arguments, so the launch does not start.

## Why it matters

The two parsers disagree on whether the launch is valid. Under the Rust
parser a launch with a wrong data path (here a model directory for another
Autoware version) starts, and the node comes up without the parameters the
file held: for CenterPoint that is the model's class list and thresholds,
which the node then declares without defaults and dies on, far from the
cause. A `resolve` used as a pre-flight check (golf cart does this before
every migration step) reports a clean model for a launch that cannot run.

## Reproduce

```xml
<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t">
    <param from="/nonexistent/params.yaml"/>
  </node>
</launch>
```

`play_launch resolve <file>` exits 0 with one WARN; `--parser python` and
`ros2 launch` fail on the missing file.

## Expected

Same as `ros2 launch`: an error naming the file and the node, and no model.
If a lenient mode is wanted for inspection, make it a flag, off by default.

## Resolution (2026-11-17)

Checked against stock `ros2 launch` (Humble) first, and the issue's
reproduction turned out to be the one case where launch does NOT fail: a
plain `<node>`'s missing parameter file is a warning there
(`launch_ros.actions.node: Parameter file path is not a file: ...`), and the
node starts without it. A composable node's missing file fails the launch
(`[Errno 2] No such file or directory`), because `LoadComposableNodes` opens
it to build the request. CenterPoint is a composable, so the golf cart case
is the second. The Rust parser had both backwards: a plain node's missing
file was a hard error, a composable's a warning.

Now, on both frontends:
- a composable node's missing parameter file is
  `ParseError::ParamFileNotFound` (XML, `actions/container.rs`) or a Python
  `FileNotFoundError` (`pyexec` `ComposableNode`), naming the file and the
  node; `resolve` exits 1 with no model;
- a plain node's missing file is warned about in launch's words and left out
  of the record and the command line (`record/generator.rs`, `pyexec`
  `Node`), except with `allow_substs="true"`, where launch opens the file and
  fails, and so does the parser.

`--parser python` already behaved as `ros2 launch` in all four cases.

Tests: `tests/missing_files.rs` (composable error, plain warning, the
`allow_substs` error). `python_tests.rs::test_parse_python_parameters`
asserted a path in `params_files` only because its fixture named a file that
does not exist, so the unread path stood in for the content; the fixture now
names a real file and the test checks its content.

Not fixed here: the plain-node warning from a `.launch.py` is not printed,
because the dlopen'd `pyexec` object has no logger (#0093).
