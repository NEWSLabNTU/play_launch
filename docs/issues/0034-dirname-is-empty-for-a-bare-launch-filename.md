---
id: 34
title: "`$(dirname)` is the empty string when the launch file is given as a bare filename"
status: open
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
