---
id: 71
title: "A `shell=True` process runs with its parts as argv instead of through the shell"
status: resolved
type: defect
severity: high
---

# 0071 - A `shell=True` process runs without a shell

**Repo:** `play_launch` at `ca5abc2d`
**Affects:** `pyexec/src/api/actions/simple_actions.rs` (`ExecuteProcess`),
`play_launch_parser/src/record/generator.rs` (`<executable>`, XML and YAML),
`python/play_launch/dump/visitor/bare_execute_process.py`

## Symptom

`ros_gz_sim`'s `gz_sim.launch.py` starts Gazebo with

    ExecuteProcess(cmd=['ruby $(which gz) sim', gz_args, '--force-version', '8'], shell=True)

Under play_launch, `gz sim` received the whole `gz_args` (`-r -s world.sdf`)
as ONE argument and exited at once:

    cmdsim8.rb:369:in `parse': invalid option: - -s --headless-rendering world.sdf

## Cause

launch hands a `shell=True` process to osrf_pycommon's
`async_execute_process`, which joins the command with spaces and runs it with
`/bin/sh -c`. Neither parser recorded `shell`: the mocks dropped the kwarg, the
XML/YAML `<executable shell=...>` was parsed and never used, and the Python
dumper read `final_cmd` only. The parts became argv.

## Fix

Both parsers record a shell process as launch runs it:
`['/bin/sh', '-c', '<parts joined with spaces>']`; its name stays the one
of the command. Tests: `play_launch_parser/tests/executable_shell_tests.rs`
(XML, YAML, `.launch.py`); the Python dumper gives the same record.
