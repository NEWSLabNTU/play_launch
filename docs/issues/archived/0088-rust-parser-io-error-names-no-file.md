---
id: 88
title: "Rust parser reports a missing include as `IO error: No such file or directory (os error 2)` without naming the file or the including file"
status: resolved
type: friction
severity: low
---

# 0088 - a missing include file is reported without its path

**Repo:** `play_launch` 0.36.0 at `89b70f92`
**Affects:** `src/ros-launch-resolve/parser/crates/play_launch_parser/src/error.rs`
(`ParseError::IoError(#[from] std::io::Error)`, line 60; the `IoError(String)`
variant at line 144), and every `?` on a file read in the include path that
converts through `From<std::io::Error>`
**Found:** golf cart, Autoware 2.1.0 migration, 2026-11-17

## What happens

A launch tree that includes a file which no longer exists stops with:

```
Error: Rust parser error while parsing golfcart_launch: IO error: No such file or directory (os error 2)
Hint: if this is a parser limitation rather than a bad launch file, re-run the same command with `--parser python` (slower, maximum compatibility).
```

Nothing says which file was missing or which file included it. On the same
tree `--parser python` names it:

```
FileNotFoundError: [Errno 2] No such file or directory: '/opt/autoware/2.1.0/share/autoware_launch/launch/components/tier4_map_component.launch.xml'
```

The hint is also misleading here: the launch file IS bad (a dead include
after Autoware 2.1.0 renamed its component files), and the suggestion to
switch parsers sends the reader looking for a parser limitation.

## Reproduce

Any include of a path that does not exist, e.g.

```xml
<launch>
  <include file="/nonexistent/x.launch.xml"/>
</launch>
```

`play_launch resolve <that file>`: the default parser prints the bare IO
error; `--parser python` prints the path.

## Expected

The error names the missing path and the including file (and line, if the
parser has it), as `ParseError::FileNotFound` already can:
`File not found: /opt/autoware/2.1.0/share/.../tier4_map_component.launch.xml (included from .../golfcart_autoware.launch.xml)`.
Likely fix: map `io::ErrorKind::NotFound` on the include read to
`FileNotFound(path)` instead of the blanket `#[from]`, and wrap other IO
errors with the path. Skip the "--parser python" hint for a not-found file.

## Resolution (2026-11-17)

`process_include_target` checks the resolved path before the frontend split
and returns `ParseError::IncludeNotFound { file, includer }`, so an XML, YAML
or `.launch.py` target reports
`Included launch file not found: <path> (included from <includer>)`. Every
other read through `read_file_cached` names its path
(`ParseError::FileRead`). `ParseError::is_missing_file()` tells the resolver
the launch tree is at fault, and it then leaves out the `--parser python`
hint, since that parser fails on the same file.

Tests: `tests/missing_files.rs`
(`a_missing_include_names_the_file_and_the_includer`, XML and YAML
includers); `edge_cases.rs::test_nonexistent_include_file`, which accepted a
bare `IO error`, now requires the path.
