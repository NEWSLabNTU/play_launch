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
restores the current file around execution. Still open and out of scope: a
literal `$(dirname)` in a node parameter inside a `.launch.py` reaches the
record unresolved, because `NodeCapture::to_record` takes no context.

Tests: `tests/dirname_tests.rs`, 7 cases - bare/`./`/absolute for the XML and
YAML frontends, a root `.launch.py`, and a unit check on `current_dir()`.
Verified as a negative control by stashing the source change and keeping the
tests: 6 of 7 fail, the bare-filename XML one with this issue's message
verbatim. Parser suite 458 passed, 0 failed.
