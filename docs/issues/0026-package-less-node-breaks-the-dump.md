---
id: 26
title: "A `Node` with no package makes `dump_launch` fail with a bare TypeError"
status: resolved
type: correctness
severity: low
---

# 0026 — a package-less `Node` kills the dump

**Repo:** `play_launch`
**Affects:** the dump visitor (`dump/visitor/node.py`) or one of its callers

## What happens

`launch_ros.actions.Node` accepts an absolute `executable` with no `package` — it is how
you launch a plain program under the launch system. play_launch cannot dump it:

```python
Node(executable="/bin/sleep", name="solo", arguments=["5"], output="screen")
```

```
Error: Failed to call dump_launch main()

Caused by:
    0: TypeError: 'NoneType' object is not iterable
```

The error names neither the node nor the field, so on a large launch file there is
nothing to go on.

## Where it came from

Found while writing a minimal reproduction for issue 0025, which needed two trivial
nodes and could not use `/bin/sleep` for either. The workaround was to borrow real
package executables (`demo_nodes_cpp`), which is fine for a test but means a legitimate
launch idiom is unsupported.

## Root cause

`visit_node` substituted the package unconditionally:

```python
package = substitute(node.node_package)     # None -> TypeError, deep inside launch
```

`normalize_to_list_of_substitutions(None)` is what raised. The record type already models
the field as optional (`package: str | None`), and so does the Rust spawn path, which
routes a package-less record to `from_raw_executable` — only the dump disagreed.

## Fix

- `visit_node` substitutes the package only when there is one.
- `visit_composable_node_container` raises a `ValueError` naming the container instead. A
  container genuinely needs a package (`ComposableNodeContainerRecord.package: str`, and
  LoadNode requests are addressed by it), so the right answer there is a clear message,
  not a `None`.
- Regression test: `tests/simple_workspace.rs::test_resolve_package_less_node_python`
  over a new Python fixture. Python, not XML — the XML frontend requires a package, so
  the shape is unreachable from XML.

## Also found

`launch_ros` appends `--ros-args` to every `Node` command line unconditionally, named or
not. `/bin/sleep` rejects the flag, so a package-less node pointing at a non-ROS program
still cannot *run* — it just dumps and spawns correctly now instead of aborting the whole
dump. Wrapping in a shell (`/bin/sh -c ...`) works, since a shell takes the stray argument
as `$0` and ignores it. This is upstream launch_ros behaviour, faithfully replayed; not a
play_launch bug, but worth knowing before reaching for this shape.
