---
id: 109
title: "A .launch.py ParameterFile(allow_substs=True) was read as written"
status: resolved
type: defect
severity: high
---

# 0109 - A `.launch.py` `ParameterFile(…, allow_substs=True)` was read as written

**Repo:** `play_launch` 0.41.3
**Affects:** the Python frontend's `Node` and `ComposableNode` stand-ins
(pyexec `api/launch_ros/node.rs::parse_parameter_item`)
**Found:** amr 175, 2026-11-23, on Jazzy: Nav2's `bringup_launch.py
use_composition:=False`, with a params file holding `$(var max_linear)`
set by a `<let>` in the including XML

## What happened

nav2_bringup hands every server
`ParameterFile(RewrittenYaml(...), allow_substs=True)`. `ros2 launch`
performs the `$(var …)`s in the file; play_launch 0.41.3 wrote them raw into
`params_files/0.yaml`, and controller_server died:

    failed to initialize rcl: Couldn't parse params file: '... params_files/0.yaml'.
    Error: Sequence should be of same type. Value type 'double' do not belong

Nav2 never activated.

## Cause

The stand-in read a `ParameterFile` through `__str__`, which is the path and
nothing else. Its `allow_substs` was dropped there, so the record carried the
file with no opt-in, and #0106 (0.41.3) then read every such file as written.
Before #0106 every file was substituted, which hid this.

## Fix

The node takes the file `ParameterFile.evaluate` gives, as launch_ros's
`Node` does: with `allow_substs=True`, a temporary copy (`launch_params_*`)
with its `$(…)` performed in the walk's context at the point the node
executes. An evaluation failure (an undeclared variable, a missing file to
open) fails the file, as in launch, instead of falling back to the path.
Composables share the helper.

## Tests

`execution_semantics.rs::a_python_parameter_file_that_opts_in_is_substituted`
(fixture `param_file_allow_substs_py`): an XML `<let>` reaches an opted-in
file and a second node without the opt-in gets the text. It fails on the
previous code.
