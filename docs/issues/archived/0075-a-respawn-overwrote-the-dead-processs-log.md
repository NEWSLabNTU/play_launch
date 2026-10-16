---
id: 75
title: "A respawn overwrote the dead process's `out` and `err`: the log that said why it died was lost"
status: resolved
type: defect
severity: medium
---

# 0075 - A respawn overwrote the dead process's log

**Repo:** `play_launch` at `23e6e9bf`
**Affects:** `src/play_launch/src/execution/context.rs`
(`NodeContext::to_exec_context`), and through it every respawning node and
container (`member_actor/regular_node_actor.rs::handle_pending`,
`member_actor/container_actor/process_lifecycle.rs::spawn_process`)
**Found:** 2026-10-16, by carla-scenario-bridge roadmap 034. Its bridge
(`respawn="true"`) drains and `_exit(75)`s after a CARLA call that never
returns; play_launch respawned it, and the dying bridge's own account of the
hang -- the one log that explained it -- was gone. It had to be captured with
`tail -F` running beside the launch.

## What happened

Every spawn of a process member goes through `to_exec_context`, which opened
`out` and `err` with `File::create`. A respawn (and a restart from the web UI)
is a spawn, so the new process's first act was to truncate the files the old
one had written. `pid` and `status` were rewritten the same way, so nothing
in the member's directory said there had ever been an earlier process.

Nothing reported it: the respawned member looked healthy, and its `err` was
simply short.

## Fix

Before the files are opened, `rotate_previous_incarnation` renames the
previous process's `out`, `err`, `pid` and `status` to `out.N`, `err.N`,
`pid.N`, `status.N`, N being the first number none of them uses (1 for the
first process, 2 for the second, ...). The live process always writes the
plain names, so `play_log/latest/node/<name>/err` still names the current
process for every reader. Nothing is renumbered later, so a path once handed
out stays right. `cmdline`, `metadata.json`, `metrics.csv` and
`params_files/` describe the member and stay shared. A first spawn finds no
`out`/`err` and renames nothing: a member that never restarts has exactly
the layout it had.

Rename, not copy: a grandchild that inherited the old descriptors keeps
writing into the renamed file, where its lines belong.

Two readers followed the old file by descriptor and had to learn about the
rename:

- the web UI's log stream (`web/sse.rs::tail_lines_to_channel`) kept
  reading the renamed inode and would have shown the dead process forever;
  it now reopens the path when it names a different inode
  (`reopen_if_replaced`) and streams the new file from its start;
- the Health page's log watcher (`health/logs.rs::read_new`) reopens by
  path each tick but kept its byte offset unless the file shrank, so a new
  process that out-wrote the old one within a tick lost its first lines; it
  now tracks the inode too.

Per-composable logs under `load_node/` (isolated container mode) are
written by the container, not by `to_exec_context`, and are not covered.

## Tests

- `execution::context::incarnation_tests` (unit): first spawn renames
  nothing; each restart takes the next number and keeps every file's
  content; a partial set (spawn failure: `err`, no `pid`) is kept as is;
  `to_exec_context` twice on one context keeps the first process's `err`.
- `web::sse::tests::a_file_replaced_by_a_respawn_is_followed`,
  `health::logs::tests::a_file_replaced_by_a_respawn_is_read_from_its_start`.
- `tests/tests/respawn_logs.rs` (integration): a `respawn="true"` script
  that prints its PID and exits 3 runs three times; `out.1`/`err.1`/`pid.1`/
  `status.1` and `.2` each hold exactly one process's output and PID, the
  plain names the live one's; a sibling without respawn has no numbered
  files.
