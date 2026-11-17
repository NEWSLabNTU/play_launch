---
id: 93
title: "The dlopen'd pyexec object has no logger, so every `log::warn!` in the Python frontend is dropped"
status: open
type: defect
severity: low
---

# 0093 - pyexec's log lines go nowhere

**Repo:** `play_launch` at main after #0089
**Affects:** `src/ros-launch-resolve/parser/crates/pyexec/` (six `log::warn!`
and `log::info!` call sites)
**Found:** while fixing #0089, 2026-11-17

## What happens

`pyexec` is a cdylib the driver `dlopen`s (0897 W3). It links its own copy
of the `log` crate and nothing installs a logger in it, so its `log::` calls
are no-ops. The plain-node warning added for #0089
(`Parameter file path is not a file: ...`) prints for an XML or YAML launch
and not for a `.launch.py`; `--parser python` prints launch_ros's own.

## Expected

Forward pyexec's records to the host's logger: hand the object a log
callback over the C ABI at load time (a version bump), or return warnings in
`ExecCaptures` for the driver to log. A test should assert the #0089 warning
appears for a `.launch.py`.
