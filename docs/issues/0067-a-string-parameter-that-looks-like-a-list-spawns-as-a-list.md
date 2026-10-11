---
id: 67
title: "A string parameter whose text looks like a list (`type=\"str\"`, `value=\"[a, b]\"`) is spawned as a list: the model has no array type"
status: open
type: defect
severity: low
---

# 0067 - A string parameter that looks like a list spawns as a list

**Repo:** `play_launch` at `785cddb7`
**Affects:** `src/play_launch/src/execution/node_cmdline.rs`
(`param_value_to_record_string`), `src/ros-launch-resolve/resolve/src/ros/model_builder.rs`
(`lower_scalar`), and the `ros-launch-manifest` model's `ParamValue`
**Found:** review of the commits pulled 2026-10-11; reproduced 2026-10-11.

## What happens

```xml
<node pkg="demo_nodes_cpp" exec="talker" name="t">
  <param name="as_str"  value="[a, b]" type="str"/>
  <param name="as_list" value="[a, b]"/>
</node>
```

`play_launch resolve` writes both to the model as strings:

```yaml
params:
  as_list: '[''a'', ''b'']'
  as_str: '[a, b]'
```

At spawn, both go out unquoted (`-p as_str:=[a, b]`), so the node gets a
string array for `as_str`, which the launch file typed as a string. Stock
launch passes `as_str` as the string `"[a, b]"`.

## Why

The model's `ParamValue` is `Bool | Int | Float | Str | StrList`, and nothing
produces `StrList`. An array parameter therefore reaches the model as a `Str`
holding its YAML flow text. `param_value_to_record_string` has to pass any
`[...]` string through unquoted, or every real array would become a string.
The dump did distinguish the two: the parser quotes a string that would
re-type (`'[a, b]'`). `lower_scalar` unquotes it into `Str("[a, b]")`, and
from then on it can't be told apart from an array.

## Fix

The model needs to say which one it holds. The options, each a
`ros-launch-manifest` change that nano-ros also reads:

1. A list variant, `ParamValue::List(Vec<ParamValue>)`, lowered from the
   dump's unquoted flow text. Exact, but it adds an enum variant, which breaks
   every exhaustive `match` on `ParamValue` downstream.
2. Make `StrList` real for string arrays and add an `IntList`/`FloatList`/`BoolList`
   family, mirroring `rcl_interfaces/ParameterType`. Closer to ROS's own
   types, and the same breaking-change cost.
3. Keep `Str`, but record the YAML text verbatim for arrays and the string's
   own value for strings, with a flag. Cheapest on the wire, and the least
   honest.

Option 2 matches what a ROS parameter actually is. It needs nano-ros to agree,
because its entry codegen reads `params`.

## Verification, when fixed

A `resolve_launch_fields.rs` case with the two parameters above: the model
distinguishes them, and the spawned command line carries `as_str` quoted and
`as_list` as a list. A negative control, reverting the lowering, must fail
it.
