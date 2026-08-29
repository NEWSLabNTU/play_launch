---
id: 26
title: "A `Node` with no package makes `dump_launch` fail with a bare TypeError"
status: open
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

## Not investigated

Which `None` is being iterated. The traceback is swallowed by the "Failed to call
dump_launch main()" wrapper; running the visitor directly would show it. `package=None`
reaches `NodeRecord.package` as an `Option`, so the record type is not the problem —
something upstream of it assumes a package string.
