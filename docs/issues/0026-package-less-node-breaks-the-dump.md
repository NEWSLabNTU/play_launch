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

## Second half: the node dumped, but still could not run

Fixing the crash was not enough. `launch_ros` appends `--ros-args` to every `Node`
unconditionally, even when there is nothing to put after it, so the spawned command was

```
/bin/sleep 3600 --ros-args        ->  exits 1: unrecognized option '--ros-args'
```

An empty ROS args section is a no-op for rcl, which is why no real ROS node ever noticed.
A package-less node usually is not a ROS program, and it is fatal there.

Both command builders now drop a trailing `--ros-args` when nothing follows it — the
Python dump visitor and the Rust parser's `generate_node_command`. A section with
arguments in it is load-bearing and is left alone. Trimming in only one of them would have
split the two parsers on every argument-less node, which is why both changed together; the
Rust golden test that pinned the empty marker was updated, since its own comment said it
was matching the Python parser.

Still true, and not a play_launch bug: a package-less node that *names* itself gets
`-r __node:=<name>` appended, which a non-ROS program will still reject. Naming a non-ROS
process has no meaning, so this is left as is.
