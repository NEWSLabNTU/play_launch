---
id: 34
title: "`$(dirname)` is the empty string when the launch file is given as a bare filename"
status: resolved
type: correctness
severity: medium
---

# 0034 - `play_launch check safety_island.launch.xml` cannot find `$(dirname)/../..`

**Repo:** `play_launch` (0.11.0, `5eaa3191`)
**Affects:** `src/ros-launch-resolve/parser/crates/play_launch_parser/src/substitution/context.rs:188-192`;
`.../play_launch_parser/src/lib.rs:127` and `:161`; `.../substitution/types.rs:122-129`

## Symptom

From inside the island's launch directory, with the 0.10.0 binary at
`155ed78b` (2026-09-21; non-ASCII transcribed):

```
$ cd src/safety_island_bringup/launch
$ play_launch check safety_island.launch.xml
Parsing launch file: safety_island.launch.xml
Error: Parser error: IO error: IO error: Failed to load and resolve parameter file '/../../autoware_mrm_emergency_stop_operator/config/mrm_emergency_stop_operator.param.yaml': IO error: No such file or directory (os error 2)

$ play_launch check ./safety_island.launch.xml
Parsing launch file: ./safety_island.launch.xml
Parsed: 1 scopes, 4 nodes, 0 containers, 0 composable nodes
2026-09-21T02:54:58.073446Z  INFO Loaded 1 manifest(s) [0 overlay, 1 provider] (0 scopes without manifests, 0 errors, 0 warnings)
[24 info[derivable-rate]/info[derivable-min-rate] lines]
1 manifest(s) checked: 1 clean, 0 with errors (0 errors, 0 warnings)
1 contract(s): 0 overlay, 1 provider
```

The launch file declares `<param from="$(dirname)/../../autoware_mrm_emergency_stop_operator/config/...">`.
With a bare filename `$(dirname)` expands to `""`, so the path becomes
`/../../...`, which is `/autoware_mrm_.../...` and does not exist. Any
directory component (`./`, relative, absolute) fixes it. `ros2 launch` runs
the same file from the same directory.

## Cause

`lib.rs:161` stores the path exactly as typed:

```rust
self.context.set_current_file(path.to_path_buf());
```

and `context.rs:188-192` derives the directory from it with `Path::parent()`:

```rust
pub fn current_dir(&self) -> Option<PathBuf> {
    self.current_file.as_ref().and_then(|p| p.parent().map(|p| p.to_path_buf()))
}
```

`Path::new("safety_island.launch.xml").parent()` is `Some("")`, not `None`
and not `.`, so `types.rs:122-129` happily returns the empty string for
`$(dirname)`. Thirty-four lines earlier (`lib.rs:127`) the same path IS
canonicalized, but only for the scope table's `canonical_path`; the
substitution context never sees the canonical form.

## Impact

Every `$(dirname)`-relative reference (param files, includes, config dirs)
in a launch file breaks when the file is named without a directory, which
is the natural thing to type from inside a `launch/` directory. The error
names a path the user never wrote, with no hint that the cwd form of the
invocation is what differs. Autoware-style launch trees use
`$(dirname)/../config/` pervasively.

## Fix direction

- Resolve the root launch path to an absolute path once at the entry point
  (the same `canonicalize_path` `lib.rs:127` already uses, with its
  cwd-join fallback) and store THAT in the substitution context; included
  files are already joined onto the including file's directory and inherit
  the fix.
- Alternatively make `current_dir()` map `Some("")` to `.`; this is the
  smaller change but leaves `$(dirname)` relative to the cwd, which is only
  right by coincidence.
- Regression test: parse a fixture that uses `$(dirname)` from within its
  own directory with a bare filename; today's behaviour is a parse error.

Note: brief C reported a different bare-filename oddity on 2026-09-18 (the
"Cross-scope diagnostics" header printing with no diagnostics under it).
That did not reproduce on 2026-09-21 on the brief's own copy with absolute
param paths (24 infos either way, `render_cross_scope_diagnostics` at
`verbs/check.rs:481-505` prints the header only when its list is
non-empty); what does reproduce on the original file is the failure above,
with the same cause (an empty `$(dirname)`).

## Provenance

Brief C (`brief-C-playlaunch-usage.md`, section 0, "A quirk found while
capturing"), 2026-09-18; the reproduction above is from 2026-09-21 with the
same 0.10.0 binary; cause located in the 0.11.0 source at `5eaa3191`.

## Resolution 2026-09-21

Fixed by absolutizing the launch file's path where it enters the parser, which
is what `launch` does: `IncludeLaunchDescription._get_launch_file_directory()`
takes `os.path.abspath(location)` FIRST and only then `os.path.dirname`, so
`$(dirname)` is absolute in ROS 2 for every invocation form. That makes the
divergence wider than this issue's title - a bare filename gave `""`, and
`./f.launch.xml` gave `"."` where ROS 2 gives an absolute directory. Both are
closed.

New `record::absolute_path` (cwd-join plus the existing lexical `.`/`..`
collapse, sharing one implementation with `canonicalize_path`'s fallback) is
applied in `LaunchContext::set_current_file` - the choke point every frontend
funnels through - and once at `LaunchTraverser::traverse_file`, before the
`.py`/`.yaml`/XML branch split, so the Python path's `path.parent()` include
joining is absolute too.

`fs::canonicalize` was deliberately NOT used: it resolves symlinks, and under
`colcon build --symlink-install` a launch file installed into `share/` points
back into the source tree, so `$(dirname)/../config/x.yaml` would resolve into
`src/` where `launch` stays in `install/`. `ScopeOrigin.path` keeps its
canonicalized spelling, so the two now differ under symlink-install; nothing
compares them today.

Found while fixing, not in the report above: **the Python frontend never set a
current file at all**. `ThisLaunchFileDir()` is captured as the string
`$(dirname)` and resolved by the host, so from a root `.launch.py` it was the
hard error `dirname: no current file set`, and from an included one it silently
meant the INCLUDING XML file's directory. `execute_python_file` now sets and
restores the current file around execution.

## The Python-capture residual, closed the same day

The note here first said a literal `$(dirname)` in a `.launch.py` node
parameter "reaches the record unresolved" and left it out of scope. Measured
rather than read, it was wider and worse than that. For a root `.launch.py`,
EVERY capture carried the literal token into the record AND into `cmd` - the
spawned command line:

```
"args":        ["$(dirname)/arg.txt"]
"params":      [["launch_dir","$(dirname)"], ["config_path","$(dirname)/sub/thing.txt"]]
"params_files":["$(dirname)/../includes/test_dirname_params.yaml"]
"remaps":      [["in","$(dirname)/topic"]]
```

The params-file case is not cosmetic: `NodeCapture::to_record` stores the
file's CONTENT (`fs::read_to_string(path).unwrap_or(path)`), so the read
failed silently and the unresolved path was stored where content belongs - the
parameter file was never loaded, and nothing said so. The oracle
(`python3 -m play_launch.dump` under `/opt/ros/humble`) gives the absolute
directory in `params`, `args`, `remaps` and `cmd`.

Fixed in `python_exec.rs`, host-side: after `backend.exec_file()` returns, the
captures that execution just appended have `$(dirname)`/`$(filename)` - and
only those two argument-less tokens - rewritten from the context, whose
current file is that `.launch.py`. Not in `to_record`, because by
`into_record_json` the context has been restored to the ROOT file, so every
capture from an INCLUDED `.launch.py` would resolve against the wrong
directory; a test pins that case. Not in the `pyexec` mock either, because the
dlopen'd object has its own `LaunchContext` with no current file, so it would
need ABI 5 -> 6. `$(var ...)` is untouched by construction and a test asserts
a `LaunchConfiguration` parameter still arrives as `$(var name)`.

Negative control: with the fixed source swapped for HEAD's and the tests kept,
2 of 3 new tests fail (`got "$(dirname)"`, and `$(dirname): No such file or
directory`). Parser suite 461 passed, IR suite 504 passed.

Neighbouring gap, filed separately as #0041: `ThisLaunchFile()`'s mock returns
`$(this-launch-file)`, a token the substitution grammar does not know.

Tests: `tests/dirname_tests.rs`, 7 cases - bare/`./`/absolute for the XML and
YAML frontends, a root `.launch.py`, and a unit check on `current_dir()`.
Verified as a negative control by stashing the source change and keeping the
tests: 6 of 7 fail, the bare-filename XML one with this issue's message
verbatim. Parser suite 458 passed, 0 failed.
