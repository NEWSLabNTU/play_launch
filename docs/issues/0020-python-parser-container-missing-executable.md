---
id: 20
title: "A stale `play_launch` Python install breaks `--parser python` with an unrecognisable error"
status: resolved
type: correctness
severity: high
---

# 0020 — `--parser python` failed on a real Autoware stack

**Repo:** `play_launch`
**Affects:** `python/play_launch/dump/__init__.py`,
`ros-launch-resolve/resolve/src/python/python_bridge.rs`,
`ros-launch-resolve/resolve/src/ros/launch_dump.rs`

## Symptom

    $ ros-launch-resolve resolve --parser python golfcart_launch golfcart.launch.yaml …
    Error: loading Python-parsed record /tmp/play_launch-parse-…record.json
    Caused by:
        0: missing field `executable` at line 6 column 9

The record the Python parser had just written could not be read back. The
offending entries carried only a name and a namespace:

```json
{ "container": [ { "name": "pointcloud_container", "namespace": "/" }, … ] }
```

## What it actually was

**Not a defect in this source tree.** `python/play_launch/dump/visitor/
composable_node_container.py` builds a full container record — executable,
package, cmd, params, scope. The package being IMPORTED was a different one:

    imported from: /home/aeon/.local/lib/python3.10/site-packages/play_launch/dump/__init__.py
    fields: ['name', 'namespace']

A stale pip install, shadowing the worktree on `sys.path`. The fixture gate
never caught it because `tests/fixtures/*/justfile` prepends `$REPO_ROOT/
python` to `PYTHONPATH` for exactly this reason — so `just compare-dumps`
passed while any invocation without that line failed.

The first diagnosis in this file — "the dump side and the loader disagree about
the shape of a container" — was wrong in a specific and instructive way. Two
container-record definitions exist in the tree, and the stale one
(`src/ros-launch-resolve/parser/src/python_dump/launch_dump.py`, an unused
vendored fork carrying `name`/`namespace` only) matched the broken output
exactly. Matching the symptom is not the same as being the code that ran.

## The real defect: the failure was unrecognisable

A stale install shadowing the current source is the most common way
`--parser python` breaks, and every symptom of it surfaces somewhere that
cannot name the cause:

- **Container records without `executable`** — a serde error naming a line and
  column in a temporary file, on a path where nothing suggests looking at
  `sys.path`.
- **File scopes without `ScopeOrigin.path`** — no error at all. Contract and
  platform-file sidecar resolution silently finds nothing, and the degraded
  model scores as PASS.

The second already had a guard, `ensure_python_scope_paths` (Phase 46.5). It
could not fire here: it runs on the loaded dump, and the stale install kills
the load itself. A guard placed after the thing it is meant to explain only
covers the symptoms that let execution reach it.

## Fix

Ask the module what it is, at import, before it parses anything.

`play_launch.dump` now exports `DUMP_FORMAT_VERSION` (currently `2`);
`ensure_python_dump_version` reads it through PyO3 in `python_bridge.rs`
immediately after the import and refuses anything older, **or absent** — which
is how every pre-existing install answers, so the check cannot degrade into a
silent pass. The message names the offending `__file__`:

    Error: The `play_launch` Python package being imported is too old for this
    binary: it reports no version marker, so it predates the check, and version
    2 or newer is required.
      imported from: /home/aeon/.local/lib/python3.10/site-packages/play_launch/dump/__init__.py
    This is a stale install shadowing the current source on PYTHONPATH. …
    Fix: prepend the current source's `python/` dir to PYTHONPATH …

Naming the file is the part that matters. The pre-existing guard already gave
the remediation; what it never gave was WHICH of several installs was in the
way, and on a machine with a pip install, a colcon `install/` tree and a
worktree, that is the whole question.

Bump `DUMP_FORMAT_VERSION` and `MIN_DUMP_FORMAT_VERSION` together whenever the
record shape gains a field the Rust side requires.

## Verified

With the stale install on the path — refused at import, before parsing.
With `python/` prepended — `container_events` resolves, and the golf cart
stack resolves to **145 nodes** under `--parser python`.

## What it unblocked, and what that found immediately

Parser parity had only ever been checkable on the fixtures whose justfiles
carried the `PYTHONPATH` line. Running `scripts/compare_models.py` across both
parsers on the golf cart stack — 145 nodes, the first real-corpus parity run —
gives 137/137 matched nodes functionally equivalent and **8 nodes whose FQNs
disagree**:

    Rust:   /system/system_monitor/system_monitor/cpu_monitor
    Python: /system/system_monitor/cpu_monitor

`system_monitor` appears twice on the Rust side, for every composable node in
`autoware_system_monitor`'s container — the one container in the stack with an
explicit `namespace=` attribute. Python is authoritative, so the Rust parser
(the DEFAULT) is wrong here. Filed separately as #0021.

## Not the same as the failure it was mistaken for

Before this, `--parser python` failed on the same stack with `KeyError: ''`,
reported here as a play_launch bug more than once. It was not: `gnss_receiver`
defaulted to `""` in the launch file and reached a dict lookup in the sensor
kit, which broke stock `ros2 launch` too. Three distinct faults surfaced
through one command in sequence — a launch-file bug, a stale install, and a
parser bug — and each was assumed to be the previous one.
