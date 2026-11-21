---
id: 106
title: "A <param from> file was substituted without allow_substs"
status: resolved
type: defect
severity: high
---

# 0106 - A `<param from>` file was substituted without `allow_substs`

**Repo:** `play_launch` up to 0.41.2
**Affects:** the Rust parser's parameter-file loading for nodes, containers
and composables (`play_launch_parser` `params.rs`, `record/generator.rs`,
`actions/container.rs`, `actions/autoware_node.rs`, `traverser/yaml.rs`)
**Found:** amr 175, 2026-11-22: a robot with `localization:=fused` loads
`nav2.yaml` into a standalone AMCL with `<param from>`, and that file holds
`$(var …)` placeholders only the Nav2 include fills (with
`ParameterFile(…, allow_substs=True)`)

## What happened

    Error: Rust parser error while parsing raw.launch.xml: IO error: Failed
    to load and resolve parameter file '.../raw.yaml': Invalid substitution
    syntax: Undefined variable: 'not_declared'.

launch_ros's `ParameterFile` defaults to `allow_substs=False`, and
`<param from>` uses it, so stock `ros2 launch` passes the file to the node
as written: on Humble, `ros2 param get /talker note` answers
`$(var not_declared)`. `--parser python` keeps the file raw too.

The Rust parser substituted every file. Issue #8 had noticed and kept that
on purpose, warning instead, because honouring the flag changes what a
launch resolves to. That is the point: stock is the oracle, and a file that
does not opt in is not substituted there either.

## Fix

`load_and_resolve_param_file_checked` returns the file as written unless
`allow_substs` is `Some(true)`. Each path carries the authored flag:

- a plain node's ordered sources already did; the split file list takes
  the flag of the matching ordered entry, since both lists push files in
  the same order;
- a container's `param_files` now pairs each path with its flag (XML and
  YAML frontends);
- a composable's `<param from>` reads the attribute, and
  `extract_params_for_node` resolves the file's values only on the opt-in;
  `<autoware_node>`'s component form passes its source's flag.

`allow_substs` is now a supported attribute of `<param>`, and the issue #8
warning and divergence note are gone. An `allow_substs` written as a
substitution (`$(var …)`) is not evaluated and reads as absent, so the file
is raw.

## Tests

`tests/param_file_substs.rs`: node, container and composable, without the
flag and with `"false"` (raw, an undeclared variable included) and with
`"true"` (substituted). It fails on the previous code. The `params.rs` unit
test that pinned the old behaviour is replaced.
