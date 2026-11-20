---
id: 102
title: "A composable's parameter file without a .yaml extension was not expanded (Nav2's RewrittenYaml)"
status: resolved
type: defect
severity: high
---

# 0102 - A composable's parameter file without a `.yaml` extension

**Repo:** `play_launch` 0.41.1 (PyPI, cp312 x86_64)
**Affects:** the default parser's composable-node parameter files, in
the Python frontend (`pyexec/src/api/launch_ros/composable_node.rs`) and
the XML/YAML frontends (`actions/container.rs::composable_param_file`)
**Found:** amr 175 (Jazzy, Ubuntu 24.04), 2026-11-20, after #0101, with
`nav2_bringup/navigation_launch.py use_composition:=True`

`navigation_launch.py` gives every server
`parameters=[ParameterFile(RewrittenYaml(...), allow_substs=True)]`.
`RewrittenYaml` performs to `tempfile.NamedTemporaryFile(mode='w',
delete=False)`, a path such as `/tmp/tmplgwjsy4a` with no extension.
`LoadComposableNodes` opens any parameter file as YAML and sends the
values matched to the node, so under `ros2 launch` the controller server
gets the file's MPPI controller.

The parser expanded a composable's parameter file only when its name
ended in `.yaml` or `.yml`. Any other file was recorded as one parameter,
`__param_file: /tmp/tmp...`, which the component ignores. Every Nav2
server therefore ran on its built-in defaults; the controller server
loaded DWB with no critics and the lifecycle manager aborted bringup.
A plain path, or `ParameterFile('/path/x.yaml')`, worked, which is why
the substitution form looked like the cause.

Fixed in both frontends: a composable's parameter file is read as YAML
whatever its name, as `LoadComposableNodes` reads it. A missing file is
still refused (#0089). A plain node is unaffected: its file is passed to
the process as `--params-file`, which rcl reads whatever its name.

Fixture `execution_semantics/param_file_substitution` uses a substitution
that copies a YAML file to an extension-less temporary file, in both the
`ParameterFile(...)` and bare forms; test
`a_composable_param_file_without_extension_is_expanded` fails on 0.41.1's
code and passes now. `--parser python` (stock `launch_ros`) gives the same
model (`compare_models.py` PASS). The earlier `user_substitution` fixture
(#0073) did not catch this because its stand-in `RewrittenYaml` wrote a
`.yaml` suffix that the real one does not.
