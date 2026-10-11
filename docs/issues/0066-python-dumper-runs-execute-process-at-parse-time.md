---
id: 66
title: "`--parser python` RUNS every plain `ExecuteProcess` at parse time and records none of them"
status: resolved
type: defect
severity: high
---

# 0066 - The Python dumper executes `ExecuteProcess` instead of recording it

**Repo:** `play_launch` at `785cddb7`
**Affects:** `python/play_launch/dump/visitor/action.py:39-59`
(`visit_action_by_class`); `python/play_launch/dump/visitor/execute_process.py`,
which is never called
**Found:** 2026-10-11, by tightening `tests/tests/on_exit_shutdown.rs` (one of
the vacuous tests from the same review). Its Python case had been passing for
the wrong reason.

## What happens

```python
ExecuteProcess(cmd=['sleep', '300'], name='bystander'),
ExecuteProcess(cmd=['sleep', '2'], name='required', on_exit=Shutdown()),
```

`play_launch launch --parser python` on that file (`resolve --parser python`
uses the same dumper):

```
[INFO] [bystander-1]: process started with pid [1770890]
[INFO] [required-2]: process started with pid [1770892]
[INFO] [required-2]: process has finished cleanly [pid 1770892]
[INFO] [bystander-1]: sending signal 'SIGINT' to process[bystander-1]
INFO  Parsed: 0 node(s), 0 container(s), 0 composable node(s)
```

The DUMPER started both commands, ran the launch's own `on_exit` handler,
recorded nothing, and play_launch then replayed an empty system and exited 0.

## Why it matters

- **Parse has side effects.** Whatever an `ExecuteProcess` runs (a
  simulator, a recorder, a script that writes files) runs during `resolve` or
  `dump`. Those verbs are meant to be inert.
- **The process is missing from the model.** It isn't monitored, logged under
  `play_log`, respawned or scheduled, and the parity gate can't see it, since
  the Python side simply has fewer entities.
- `on_exit_shutdown.rs`'s Python case passed only because the dump took about
  two seconds, which met the old "ended in time" check.

## Why

`visit_action_by_class` handles `LoadComposableNodes`,
`ComposableNodeContainer`, `LifecycleNode`, `Node` and
`IncludeLaunchDescription`. Everything else falls through to
`action.execute(context)`, which for `ExecuteProcess` is stock launch's
implementation and spawns the process. `visit_execute_process`, which records
through `visit_execute_local`, exists but has no caller. `Node` reaches
`visit_execute_local` from `visit_node`, so nodes are recorded. A bare
`ExecuteProcess` never reaches it.

## Fix

Add the branch, after the `Node` cases, since `Node` subclasses
`ExecuteProcess`:

```python
elif is_a(action, ExecuteProcess):
    return visit_execute_process(action, context, dump)
```

Then check that `visit_execute_local` records an `ExecuteProcess` with no ROS
package in the shape the Rust parser's `<executable>` record has
(`package: None`). CLAUDE.md's 2026-03-10 entry covers what goes wrong if
ROS-only fields leak into such a record. Also check it records
`on_exit_shutdown`, as 779d68e5 intended.

## Verification, when fixed

- Remove the `#[ignore]` on `on_exit_shutdown.rs::python_parser_shutdown_action`;
  it must pass all four checks.
- A stock-parity case with a bare `ExecuteProcess`, compared under both
  parsers.
- `resolve --parser python` on a launch whose `ExecuteProcess` touches a
  file: the file must not exist afterwards.

## Resolution (2026-10-11)

`visit_action_by_class` gained an `ExecuteProcess` branch, after the `Node`
branches since `Node` subclasses it. The branch goes to
`bare_execute_process.py`, which prepares the process (registering its
handlers and starting nothing, as for a `Node`) and records it the way the
Rust parser's capture does: no package, executable plus arguments, the given
name, `additional_env`, `respawn` and `on_exit`. `resolve --parser rust` and
`--parser python` now produce equivalent models for it (`compare_models.py`:
PASS), and the Python dump starts no process.

Tests: `python_dump_inert.rs` (a `touch` that must not run, and must be in
the model). Negative control: without the branch it fails. Also
`on_exit_shutdown.rs::python_parser_shutdown_action`, no longer ignored.
