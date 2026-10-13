---
id: 74
title: "The Python dumper merges every node's section of a substituted parameter file into each node, and drops files with lists"
status: resolved
type: defect
severity: high
---

# 0074 - The Python dumper flattens parameter files into one node

**Repo:** `play_launch` at `c1f76c72`
**Affects:** `python/play_launch/dump/visitor/node.py`

## Symptom

`nav2_bringup` under `--parser python`: the costmaps aborted with

    parameter 'observation_sources' has invalid type: ... of type {string},
    setting it to {string_array} is not allowed

and the lifecycle managers with `parameter 'node_names' is not initialized`.

## Cause

Two faults in how the dumper reads the files launch writes to
`/tmp/launch_params_*`:

1. It took every such file for a node's inline `parameters=[{...}]` and
   merged all its sections into that node. But
   `ParameterFile(..., allow_substs=True)` (nav2_bringup's
   `RewrittenYaml`) also writes the WHOLE substituted file there, with a
   section per node: every Nav2 node got every other node's keys, the last
   one winning (`observation_sources: [scan]` from collision_monitor on the
   costmaps).
2. launch_ros dumps a list value as a tuple, which yaml writes as
   `!!python/tuple`. `yaml.safe_load` refused the tag and the file was
   dropped with a warning, so `node_names` (a list) was lost.

## Fix

A file with a section that does not address this node (rcl's matching:
`/**`, the node's name, `*` and `**` wildcards) stays a parameter file, as
the Rust path already keeps them; the node reads its own sections. Files
are loaded with a safe loader that reads `!!python/tuple` as a sequence.
Checked with the whole ceilbot robot (Nav2 included) on Jazzy: 32 of 32
processes up, an operator shift of 7 steps passed.
