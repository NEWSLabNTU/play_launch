---
id: 87
title: "`<executable cwd=…>` is ignored, so a process runs in play_launch's directory, and the parser warns that `shell=` is ignored although it is honoured"
status: resolved
type: defect
severity: medium
---

# 0087 - `<executable cwd=…>` is ignored; the `shell=` warning is false

**Repo:** `play_launch` at `89b70f92` (0.36.0 + #0086)
**Reported:** GitHub NEWSLabNTU/play_launch#27, AutoSDV Lab 3 launch files
(Humble), 2026-11-17
**Affects:** the Rust parser's `<executable>`/`<node>` `cwd` and the
attribute table in `xml/attr_spec.rs`

## Symptom

```xml
<launch>
  <executable cmd="ros2 bag record /tf_static" cwd="$(env HOME)/bags" output="screen"/>
</launch>
```

```console
$ play_launch launch ./record.launch.xml
WARN <executable cwd=…> is valid ROS 2 but not supported by the Rust parser; the value is ignored. Use --parser python if you need it.
```

The bag lands in the directory play_launch was started from. `ros2 launch`
puts it in `~/bags`. `ros2 bag record` with no `-o` writes into its working
directory, so the divergence changes where data goes, not just a log line.

`shell="true"` prints the same warning:

```
WARN <executable shell=…> is valid ROS 2 but not supported by the Rust parser; the value is ignored.
```

## Cause

- `cwd` is parsed into `ExecutableAction::cwd` and never read; no record or
  model field carries a working directory, and the spawn never sets one.
- `shell` IS honoured since #0071 (`record/generator.rs` runs the command as
  `/bin/sh -c "<joined cmd>"`), but `attr_spec.rs` still lists `shell`, `cwd`
  and `launch-prefix` under `known_unsupported` for `<executable>` and the
  `<node>` family, so the parser warns that a value it uses is ignored.

## Expected

`cwd` (after substitution) is the process's working directory under every
frontend and both start paths (`launch`, `up <model.yaml>`), as `ros2
launch` does; a `cwd` that does not exist fails the process start, as it
does in launch. The table stops warning about what is honoured. Related:
#0085 (`launch-prefix`, `<unset_env>`), the same class of attribute.

## Not a divergence

`$(command 'date +%Y%m%d_%H%M%S')` keeps the trailing newline, so a name
built from it ends in `\n`. `ros2 launch` does the same.

## Resolution (2026-11-17)

Fixed with #0085. `cwd` (after substitution) on `<executable>`, `<node>`,
`<node_container>` and their YAML spellings, and `cwd=` on Python
`ExecuteProcess`/`Node`/`ComposableNodeContainer`, is carried on the process
record and the model (`NodeInstance::cwd`, ros-launch-manifest v0.3.28) and
set as the child's working directory at spawn, for `launch`, `run` and
`up <model.yaml>`. A `cwd` that is not a directory fails the start, as in
launch; the node's `err` file and the "Unable to start" error name the path.

The attribute table now lists `shell`, `cwd` and `launch-prefix` as
supported on `<executable>`, and `cwd`/`launch-prefix` on the `<node>`
family, so none of them warns. What still warns, and why
(`xml/attr_spec.rs`):

| Attribute | Status |
|---|---|
| `<node shell=…>` | not carried (a node's command line is built at spawn); `false` is silent |
| `emulate_tty` | N/A: output goes to `play_log` files, never a tty; `false` is silent |
| `exec_name` | dropped; launch uses it only to label the process in its log |
| `respawn_max_retries` (Jazzy) | dropped |
| `sigterm_timeout`, `sigkill_timeout` (Jazzy) | dropped; open as #0084 |
| `node-name` (Humble) | N/A: crashes stock `Node` construction |

Honoured and unchanged: `<env>` children, Python `additional_env`,
`respawn`/`respawn_delay`, `on_exit`, `ros_args`. The YAML frontend now also
reads a native `shell: true`, which it read as false.

Test: `tests/tests/process_spawn.rs` (`cwd_exe`, `node_wrap`, `shell_exe`
against `ros2 launch`).
