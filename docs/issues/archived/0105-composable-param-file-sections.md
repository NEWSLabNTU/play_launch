---
id: 105
title: "A composable got the whole of a Nav2-style parameter file"
status: resolved
type: defect
severity: high
---

# 0105 - A composable got the whole of a Nav2-style parameter file

**Repo:** `play_launch` up to 0.41.2
**Affects:** the parameters both frontends put in a composable's LoadNode
request from a parameter file (`play_launch_parser` `params.rs`,
`actions/container.rs`, `actions/autoware_node.rs`; pyexec
`api/launch_ros/helpers.rs`)
**Found:** 2026-11-21, right after #0104, on the same Nav2 run

## What happened

With #0104 fixed, all eleven Nav2 servers loaded and then failed to
configure: map_server with `parameter 'yaml_filename' is not initialized`,
controller_server with `No critics defined for FollowPath`.

`LoadComposableNodes` builds the request with launch_ros's
`to_parameters_list`, which finds each `ros__parameters` key at any depth,
names its section by the keys above it joined with `/` (each stripped of a
leading `/`), and sends a node only the sections whose name matches it.
Nav2's `nav2_params.yaml` writes `amcl:` and `map_server:` with no leading
slash, and nests a costmap's section as `global_costmap: global_costmap:`.

- The Python frontend counted only top-level keys starting with `/` as
  sections. Nav2's file had none, so it flattened the whole file onto every
  composable, as `amcl.ros__parameters.alpha1` and the like. map_server got
  221 parameters, none of them `yaml_filename`.
- The XML/YAML frontend merged every top-level section's
  `ros__parameters` into every composable, with no node selection at all,
  and missed nested sections.

The Python dumper (`--parser python`), which runs launch_ros itself, gave
map_server its two parameters.

## Fix

One selection, in `play_launch_parser::params`: `node_param_sections`
(launch_ros's `__normalize_parameters_dict`), `sections_for_node` (matching
sections in file order, then the exact one again, as `to_parameters_list`
does) and `is_node_name_matched`. The Python frontend calls it and drops its
own copy. The XML/YAML frontend now passes the composable's FQN
(`container::composable_fqn`, the launch context's namespace per #0021) to
`extract_params_for_node`. Each frontend keeps its own value spelling.

On Humble, Nav2's localization lifecycle manager now reports its nodes
active, and navigation goes as far as stock does with no robot spawned
(activating controller_server, waiting for TF).

## Tests

- `execution_semantics.rs::a_composable_gets_its_own_section_of_a_nav2_style_param_file`
  (fixtures `param_file_sections_xml`, `param_file_sections_py`), which
  fails on the previous code.
- pyexec `helpers::tests::sections_without_a_slash_and_nested_sections_are_selected`.
