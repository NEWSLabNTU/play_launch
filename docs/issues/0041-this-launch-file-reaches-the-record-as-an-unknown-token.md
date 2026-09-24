---
id: 41
title: "`ThisLaunchFile()` in a `.launch.py` reaches the record as `$(this-launch-file)`, a token the substitution grammar does not know"
status: resolved
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

## Resolved 2026-09-24 — and this issue's own premise was wrong

The report above, and the decision it framed, assumed `ThisLaunchFile` and
`$(filename)` are two concepts — a full path versus a basename — and warned
that redefining `$(filename)` "probably fails on compatibility grounds". That
is backwards, and checking ROS 2 rather than reasoning about it settles it in
one line. `/opt/ros/humble/.../launch/substitutions/this_launch_file.py`:

```python
@expose_substitution('filename')
class ThisLaunchFile(Substitution):
    """Substitution that returns the absolute path to the current launch file."""
```

`$(filename)` in launch XML **is** `ThisLaunchFile`, and it is the ABSOLUTE
PATH. `perform()` returns `context.locals.current_launch_file_path`. Confirmed
empirically against a seeded `LaunchContext`, not only read:

```
filename -> '/abs/dir/probe.launch.xml' | ThisLaunchFile
dirname  -> '/abs/dir'                  | ThisLaunchFileDir
```

So there was a SECOND divergence hiding behind this one: this parser's
`$(filename)` returned the basename on every frontend, XML included. Adding a
third token would have invented a spelling ROS 2 does not have, unwritable in
XML, while leaving `$(filename)` permanently wrong and still holding ROS 2's
name for it.

Fixed as: `Substitution::Filename` resolves from `current_file()` rather than
`current_filename()`; `ThisLaunchFile`'s stand-in in the Python half returns
`$(filename)`; and `FileSubstitutions` in `python_exec.rs` — the helper
0034's residual added — rewrites it with the full path, still after
`backend.exec_file()` returns, so an INCLUDED `.launch.py` resolves against
its own file. A test pins that case.

Blast radius checked before the change: `$(filename)` had one resolution site,
one render site, one rewrite site, and **zero** uses in any fixture or launch
file in the repository. `docs/guide/parser-features.md:119` already documented
it as "Path of current launch file", so the implementation was the thing out
of step with this repo's own documentation.

One pre-existing test asserted the old basename
(`pyexec/tests/eval_with_via_python.rs::test_filename_substitution`) and was
corrected — the fourth test this session found pinning a defect.

Non-vacuity: with the three source files reverted and every test kept, 3 of
the 4 new tests fail (`got "test_this_launch_file.launch.xml"`, `got
"$(this-launch-file)"`, `$(this-launch-file): No such file or directory`) and
the `$(var ...)` preservation control passes either way, as a control should.
Parser 461 -> 465, IR 504 -> 508.

**Narrower than the report claimed:** the silent params-file shape is not
reachable for a bare `ThisLaunchFile()`. A bare substitution is classified as
a parameter FILE only when its unresolved string ends in `.yaml`, which
neither token does, and any `.yaml` path built from it would treat the launch
file as a directory. It is reachable only through
`ParameterFile(ThisLaunchFile())`, which is what the fixture uses.
