---
id: 68
title: "A list parameter whose string elements hold `: ` or `, ` reaches the node re-shaped: Autoware's component_state_monitor refuses to construct"
status: resolved
type: defect
severity: high
---

# 0068 - A list element holding flow syntax is re-shaped

**Repo:** `play_launch` at `476f8a86`, and `ros-launch-manifest` v0.1.51
**Affects:** `ros-launch-manifest` `model/src/lib.rs` (`list_element_text`),
`src/play_launch/src/execution/node_cmdline.rs` (`str_to_yaml`)
**Found:** 2026-10-12, in `just test-all`'s Autoware run, the first full run
after #0067.
**Regression from:** #0067 (schema 2).

## What happens

Autoware 1.5.0's `component_state_monitor.launch.py` builds this parameter:

```python
{"launch": {"map": ["topic_state_monitor_map: map_topic_status", ...]}}
```

The composable fails to load:

```
Component constructor threw an exception: parameter 'launch.map' has invalid
type: Wrong parameter type, parameter {launch.map} is of type {string_array},
setting it to {string} is not allowed.
```

## Why

Schema 2 carries a list as a `StrList` whose elements are spelled as YAML
scalars, and play_launch renders it as flow text, `[e1, e2]`. v0.1.51 quoted
an element only by the scalar rule: would it read as a number, a bool or
nothing. Inside `[...]`, flow syntax re-shapes more than that. `a: b` reads
back as a mapping, `a, b` as two elements, `a #b` loses its tail. So the
element went out bare, as `[topic_state_monitor_map: map_topic_status]`.
The LoadNode path parses that as YAML, gets a sequence holding a mapping, and
falls back to a `string` parameter, which the node refuses.

In schema 1 the parser's own flow text was passed through, already quoted, so
this did not happen before #0067.

The plain-node path survived by accident. `str_to_yaml` splits a sequence on
commas, which happens to keep `a: b` whole, but it would cut a quoted
`'a, b'` in two.

Nothing caught it. The Autoware process-count tests count processes, not
loaded composables, so a composable that never constructs still passes. The
fast suite excludes Autoware, and the #0067 verification did not run
`test-all`.

## Fix

- **`ros-launch-manifest` v0.1.52:** an element is quoted unless it reads back
  as itself between `[` and `]` (`reads_back_in_flow`). The schema-1 upgrade
  goes through the same spelling.
- **play_launch:** `str_to_yaml` reads a flow sequence as YAML first, and
  falls back to the comma split only when it does not parse.

## Verification

- `model/tests/param_lists.rs` in the manifest repo: elements holding `: `,
  `, `, ` #`, `{`, `]`, `"`, a leading `- ` or a leading space are quoted, and
  each spelling reads back as its element.
- `parameter_conversion::tests` covers the composable path from model to
  `ParameterValue`, and gets a `string_array` of the two elements.
- `node_cmdline::tests` covers the plain path: a quoted `'a, b'` stays one
  element.
- `tests/tests/param_lists.rs` spawns a node with
  `['mon: map_status', 'a, b', plain]` and reads the params file it was given.
- Negative controls: without the manifest fix the composable test fails with
  type 4 (`string`) against 9 (`string_array`), Autoware's error. Without the
  YAML read, the plain-path test fails.
- `test_autoware_process_count_python` no longer logs the `launch.map`
  failure.
