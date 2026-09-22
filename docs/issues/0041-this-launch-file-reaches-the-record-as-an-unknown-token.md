---
id: 41
title: "`ThisLaunchFile()` in a `.launch.py` reaches the record as `$(this-launch-file)`, a token the substitution grammar does not know"
status: open
type: correctness
severity: medium
---

# 0041 - a substitution the parser emits and cannot read

**Repo:** `play_launch` (0.11.0, found at `3b01e54d`)
**Affects:** `src/ros-launch-resolve/parser/crates/pyexec/src/api/substitutions/package.rs:535`
and `:545`; `.../play_launch_parser/src/substitution/parser.rs:349-350`

## Symptom

`ThisLaunchFile()`'s stand-in in the Python half returns a literal string:

```rust
fn __str__(&self) -> String {
    // Return placeholder - actual path would be set during parsing
    "$(this-launch-file)".to_string()
}
```

and `perform()` at `:545` returns the same. The host's substitution grammar
knows exactly two file-location tokens (`substitution/parser.rs:349-350`):

```rust
"dirname" => Ok(Substitution::Dirname),
"filename" => Ok(Substitution::Filename),
```

There is no `this-launch-file`. So a `.launch.py` that uses `ThisLaunchFile()`
for a parameter, an argument, a remap or a parameter-file path puts the literal
string `$(this-launch-file)` into the record, the model and the spawned command
line, on every path. `ros2 launch` substitutes the launch file's absolute path.

## Cause

The comment at `:536` says the placeholder is resolved "during parsing", and
nothing does. This is the same shape as the `$(dirname)` residual closed in
`archived/0034-*`: the mock cannot resolve a file-location substitution itself,
because the dlopen'd `pyexec` object has its own `LaunchContext` and is never
told the current file, so it emits a token for the host to rewrite - and for
`$(dirname)`/`$(filename)` the host now does exactly that
(`traverser/python_exec.rs`, `FileSubstitutions`). `this-launch-file` was never
added to either side.

Found while fixing 0034's Python-capture residual; not in that issue's scope.

## Impact

Smaller than 0034's, because `ThisLaunchFile()` is rarer than
`ThisLaunchFileDir()` - but the same class, and the parameter-FILE case has the
same silent shape: `NodeCapture::to_record` stores a params file's CONTENT via
`fs::read_to_string(path).unwrap_or(path)`, so an unresolved path is stored
where content belongs and the file is never loaded, with no diagnostic.

## Fix direction

One decision has to be made first: **ROS 2's `ThisLaunchFile` returns the full
PATH, and our `$(filename)` returns the basename**
(`LaunchContext::current_filename()`), so the existing token is not a correct
target for it. Either

- add a `Substitution::ThisLaunchFile` to the grammar (parser plus whatever
  renders it) and have `FileSubstitutions` rewrite it beside the other two -
  the honest option, since the concept is genuinely a third one; or
- decide `$(filename)` should be the full path (it is a parser-internal token,
  so the blast radius is small - but check every reader first, and note that
  XML `$(filename)` is a documented ROS 2 substitution whose meaning is fixed,
  so this likely fails).

Then a fixture using `ThisLaunchFile()` in a parameter value AND as a
`--params-file` path, asserting an absolute path in the record and in `cmd`,
with the same negative control the 0034 tests use.

## Provenance

Found 2026-09-22 while closing 0034's Python-capture residual; the mock and the
grammar were read at `3b01e54d`.
