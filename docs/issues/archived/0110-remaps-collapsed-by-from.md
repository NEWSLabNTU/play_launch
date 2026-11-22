---
id: 110
title: "A node's command line kept one remap rule per `from`, in hash order"
status: resolved
type: defect
severity: high
---

# 0110 - A node's command line kept one remap rule per `from`, in hash order

**Repo:** `play_launch` up to 0.41.3
**Affects:** every process play_launch spawns as a ROS node
(`src/play_launch/src/execution/node_cmdline.rs`)
**Found:** amr 175, 2026-11-23: a group's
`<set_remap from="/tf" to="/tf"/>` around Nav2's bringup, whose nodes add
`('/tf', 'tf')` of their own

## What happened

launch_ros passes every rule in order, the context's `SetRemap` rules first
and then the node's own, and rcl applies the first rule that matches a name.
So the group's `/tf:=/tf` wins and slam_toolbox uses `/tf`. Under
play_launch its command line had only the node's `/tf:=tf`, and
slam_toolbox used `/planning/tf`.

## Cause

The parser and the model kept every rule in order. `NodeCommandLine.remaps`
was a `HashMap`, so building the command line kept the last rule for each
`from` (the node's) and emitted them in hash order, which also put
`__node`/`__ns` among the others.

## Fix

`remaps` is an ordered `Vec`: `__node`, `__ns`, then each rule as the model
carries it. Composables were not affected; their rules were already a list.

## Tests

`node_cmdline.rs::every_remap_reaches_the_command_line_in_order`: the five
rules of the report, repeated `from`s included, in order. Checked end to end
with `up` on a model of the same shape.
