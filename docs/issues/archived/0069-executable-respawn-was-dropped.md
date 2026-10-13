---
id: 69
title: "`<executable respawn=\"true\">` was accepted and dropped: a process that died stayed dead"
status: resolved
type: defect
severity: medium
---

# 0069 - `respawn` on `<executable>` was dropped

**Repo:** `play_launch` at `150536c4`
**Affects:** `src/ros-launch-resolve/parser/crates/play_launch_parser/src/actions/executable.rs`,
`record/generator.rs` (`generate_executable_record`), `traverser/yaml.rs`
**Found:** 2026-10-13, by the crash-loop test of the Health page
(`docs/design/health-page.md` W3), whose respawned segfaulting process never
came back.

## What happened

`launch`'s `ExecuteProcess` takes `respawn` and `respawn_delay`, and both
`launch_xml` and `launch_yaml` read them for `<executable>` exactly as for
`<node>`. The Rust parser had both on `<executable>`'s attribute allowlist,
so they parsed without a warning, but `ExecutableAction` had no field for
them and `generate_executable_record` wrote `respawn: None`. The SystemModel
carried no `respawn` for the process, so play_launch never restarted it.

The allowlist made the drop silent: an attribute that is accepted and then
ignored looks, from outside, exactly like one that works.

## Fix

`ExecutableAction` carries `respawn` / `respawn_delay` (XML and YAML), and the
record generator resolves them the way it does for `<node>`.
`tests/executable_respawn_tests.rs` in the parser crate checks both
frontends, with substitutions; play_launch's
`test_health_reports_crash_loops_and_members_that_stay_down` checks the
running behaviour.

The Python frontend has the same gap for `Node` and `ExecuteProcess`: see
#0070.
