---
id: 107
title: "A YAML-frontend composable's param file is dropped"
status: open
type: defect
severity: medium
---

# 0107 - A YAML-frontend composable's `from:` parameter file is dropped

**Repo:** `play_launch`
**Affects:** `play_launch_parser` `traverser/yaml.rs::parse_yaml_composable_nodes`
**Found:** 2026-11-22, while fixing #0106 (code reading, not a report)

A composable node in a `.launch.yaml` file reads only the inline `name:`
entries of its `param:` list. An entry with `from:` is skipped, so the
LoadNode request carries none of the file's values. The XML frontend's
`<composable_node><param from=…/>` and the Python frontend both load the
file (`actions/container.rs::composable_param_file`).

Not yet checked against stock `ros2 launch`'s YAML frontend; that is the
first step. The fix would route `from:` through `composable_param_file`
with the composable's FQN and its `allow_substs`, as the XML path does.
