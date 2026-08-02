# Phase 55 — Launch toolchain consolidation

**Design:** `docs/design/launch-toolchain-topology.md`
**W0/W1/W2 landed** 2026-08-02/03. RFC-0060 amended by nano-ros `9baebb2eb`.
**Related:** issue 0013 (resolved — deleted the dead CLI surface this phase inherits)

## Problem

`play_launch` → `ros-launch-resolve` → {`play_launch_parser`,
`ros-launch-manifest`} is three levels of submodule nesting. Every
cross-layer change needs two or three pointer-bump commits in a fixed order,
and concurrent work on two layers races on the same tree.

The nesting buys nothing. The isolation everyone actually depends on comes
from the **Cargo workspace** boundary (`exclude` keeps `rclrs`/`rosidl` out of
layer 2's graph) and the **process** boundary (the resolver is a binary, so
`libpython` never enters a consumer's link). Both survive a merge; see the
design doc for the measurements.

## Scope

Fold layer 2 into this repository as plain directories, keeping its separate
workspace. Keep `ros-launch-manifest` independent — it is the genuine shared
contract, the only thing both projects link.

Explicitly NOT in scope: the layering itself, the direction of dependency,
what any consumer links, and the verb reshape (`check` → argument, `replay` →
model-based + rename). That last one is user-facing and breaking; it gets its
own phase.

## Status (2026-08-02)

**W0, W1 and W2 all done. Submodule nesting under layer 2 is gone entirely.**

W0: nano-ros accepted the consolidation and amended RFC-0060 to two
repositories, keeping the three layers, the linking rule and the process
boundary. All four reject-reasons (release cadence, access control, CI cost,
bigger pinned tree) were considered and cleared; RFC status stays Stable.

W1: `src/ros-launch-resolve` and its nested `parser` are plain directories in
this repository, folded in with `git subtree` so both histories survive —
ros-launch-resolve's own commits are now reachable from `git log` here. One
submodule level remained at that point (`third-party/ros-launch-manifest`);
W2 removed it too.

W2: `ros-launch-manifest` is a git dependency pinned to **v0.1.0**, named
identically by `src/play_launch/Cargo.toml` and
`src/ros-launch-resolve/Cargo.toml` so cargo resolves one instance. Layer 2
has no submodules left. Three levels of nesting → zero.

Landed ahead of the merge, because it is worth having whether or not the
merge happens:

- **W1's acceptance criterion is now an executable gate**, not a one-off
  measurement — `ros-launch-resolve/scripts/check-layer2-isolation.sh`,
  `just check-layer2-isolation` (folded into `just check`), and a CI job on a
  bare runner with no ROS at all. Details and the two findings it turned up
  are in the design doc's "Enforcing it" section.
- **`just check` passes end to end again.** It had been failing since
  `adc33a7`: that commit moved `dump` to ros-launch-resolve and dropped
  `Command::Dump`, but left `DumpArgs`, `DumpSubcommand` and two tests
  destructuring the removed variant, so `cargo check --all-targets` did not
  compile. Removing the residue turned up `PlotArgs` and the three
  `Contract*` structs in the same state, and the mirror-image defect in the
  extracted binary, whose `--help` still introduced itself as `play_launch`
  and advertised `launch`, `run` and `replay`. Both halves of the split had
  drifted, in opposite directions, and neither warned: **`pub` items in a
  `pub` module are never reported as `dead_code`**, so an extracted verb's
  leftovers are invisible to the compiler. Both sides now have a test that
  fails if `--help` names a verb the binary lacks.

The relevance to this phase is not incidental. Every one of those defects is
a seam artifact — things left behind on one side of a split that no build on
either side could see. That is the cost being weighed in W0.

## W0 — Ratify the boundary change — DONE

RFC-0060 was Stable and specified three repositories. That was nano-ros's RFC
to amend, and layer 3 did not get to redraw a boundary its consumers depend
on.

- [x] nano-ros amended RFC-0060 (`9baebb2eb`, phase-332 W0): three layers, the
      linking rule and the process boundary all kept; two repositories plus the
      shared schema repository. Status stays Stable.
- [x] All four reject-reasons considered and cleared by the maintainer —
      release cadence, access control, CI cost, bigger pinned tree.

Rejection was a live option and is recorded as such; it was not taken.

## W1 — Merge layer 2 in — DONE

- [x] `git submodule deinit` + remove `src/ros-launch-resolve`; re-added at the
      **same path** via `git subtree add`, history preserved — its own commits
      are reachable from `git log` here. Same path means every `../ros-launch-
      resolve/...` path dep, the justfile and CI needed no edit.
- [x] Same for the nested `parser` submodule → a plain directory
- [x] `exclude` kept. Comment rewritten: it no longer describes a submodule,
      and it now says this line is the ONLY thing keeping the `rclrs` patches
      out of layer 2's graph, naming the test that catches its removal.
- [x] `ros-launch-manifest` stays a submodule, re-registered in the ROOT
      `.gitmodules`. The nested `.gitmodules` was deleted rather than left: git
      reads only the root one, so a nested copy looks authoritative without
      being so — it still declared `parser` as a submodule after the subtree
      add had made it plain directories.

**Acceptance — the gate for this whole phase.** From a clean clone, with no
`install/` tree, no `build/` tree, and the ROS environment stripped:

```sh
env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u PYTHONPATH -u COLCON_PREFIX_PATH \
    cargo build -p ros-launch-resolve-cli
```

must succeed, and the resulting binary must show:

- `cargo tree -p ros-launch-resolve-cli` → **0** `rclrs`/`rosidl` entries
- `ldd` → **0** ROS/rcl/rmw/ament libraries, `libpython` present
- `resolve` works on both a `.launch.xml` and a `.launch.py` with no ROS
  sourced, and the declared node appears in the model — check the model
  contents, not just the exit code

These all pass on the current layout (2026-08-02), so a failure means the
merge broke the workspace boundary, not that the property was never there.

**Result — all green on the merged tree.**

| Check | Result |
|---|---|
| Content preserved | sha256 of all 391 tracked files across the three trees, before vs after: **one intended difference** (the deleted nested `.gitmodules`) |
| Isolation gate, in-tree | pass |
| Isolation gate, `--standalone` | pass |
| Fresh `git clone --recurse-submodules` + gate | pass, **8.9s cold**, and the parent-cargo-config NOTE disappears (nothing above it to inherit) |
| Submodule depth | 3 levels → **1** |
| `just build` / `just check` | pass |
| `just test-all` | 110/110, 0 skipped |
| `just test-unit` | 462/462 |

The clean-clone run is the one that matters: it is exactly what nano-ros W1
will do — clone this repo recursively, `cargo build -p
ros-launch-resolve-cli`, no ROS anywhere.

One test failure during verification was chased down and was **not** the
merge: `container_events::test_isolated_external_subscriber` hard-codes
`ROS_DOMAIN_ID=199` while the rest of the suite assigns a unique domain per
invocation, so a stale `ros2-daemon` on domain 199 from an earlier run made it
fail. Its diagnostics pipe both `ros2` calls through `2>/dev/null`, so the
real error was invisible and the assertion message blamed DDS cross-process
communication. Worth fixing on its own; filed separately from this phase.

## W2 — `ros-launch-manifest` by tag — DONE

- [x] Tagged `ros-launch-manifest` **v0.1.0**; `play_launch` and the layer-2
      workspace both depend on it by that tag
- [x] Submodule dropped — **layer 2 now has no submodules at all**, and the
      three remaining in this repository are play_launch's own (vendored
      ros2_rust, rosidl_runtime_rs, rcl_interception_sys) plus the two C++
      packages
- [x] Coordination with nano-ros turned out to be unnecessary *if ordered this
      way round* — see below

### The lockstep turned out to be avoidable

The earlier reading was that three repositories had to convert together:
play_launch, the layer-2 workspace, and nano-ros in four manifests. Convert
one and cargo resolves two same-named packages from different sources; they do
not unify, so `SystemModel` becomes two incompatible types.

That is still true of the *simultaneous* state — but W1 removed the
constraint. play_launch and layer 2 are one repository now, so their two
conversions are a single atomic commit. And nano-ros still pins the OLD
standalone `ros-launch-resolve` repository, so it consumes none of this yet:
it is insulated until it does its own W1. Doing W2 **before** nano-ros
repoints means they land on a tree that is already tag-based and convert once
instead of twice.

Verified, not assumed: `cargo tree -i ros-launch-manifest-model` shows a
single node with both `play_launch` and `ros-launch-resolve` as dependents,
and `Cargo.lock` has one `source = "git+...?tag=v0.1.0"` entry.

### The blocker W2's plan did not anticipate

Four consumers read the manifest repository's `tests/fixtures` **at runtime**,
by relative path: three unit-test modules in `resolve/src/ros/` and
`tests/tests/manifest_check.rs`. 24 of the 29 fixture packages are used. A git
dependency is checked out under `~/.cargo/git/checkouts/<repo>-<hash>/<rev>/`,
so no relative path from a consumer can reach them.

Copying the fixtures into the consumer was the obvious fix and the wrong one:
24 packages duplicated, going stale silently the first time a rule or schema
changed upstream — the same seam-drift class this phase exists to remove.

Instead the owning repository hands the path out:
`ros_launch_manifest_check::fixture_dir()`, behind a default-off `testdata`
feature (`ros-launch-manifest` `172aa53`). `CARGO_MANIFEST_DIR` resolves
correctly for a path dependency, a git dependency and a local `cargo test`
alike. Consumers enable the feature in dev-dependencies, so with resolver 2 it
never reaches a non-test build.

Proven non-vacuous by a negative control: moving `tests/fixtures` aside in the
cargo checkout fails 6 tests; restoring it passes them. The tests genuinely
read from the git checkout rather than finding a leftover directory.

**A note on how the fourth consumer was nearly missed.** The first exhaustive
grep for the fixture path ended in `head -20`, which truncated the output at
exactly the point the `tests/` hit would have appeared — so a search meant to
be exhaustive silently was not, and the omission surfaced only when
`just test-all` failed. Worth naming, because it is the same shape as the
silent-skip defects catalogued in issues 0008 and 0012: a check that looks
complete and is not.

### Residual: git dependencies need network

A submodule is on disk after clone; a git dependency needs network on first
build (thereafter `~/.cargo/git` caches it). `cargo vendor` covers offline
distribution if nano-ros needs it for embedded targets. Flagged rather than
solved — it has not bitten anything here, and CI has network.

Unrelated but worth passing on: nano-ros has an untracked
`packages/cli/third-party/ros-launch-manifest/` directory on disk that git
does not know about — not a submodule, not vendored, just a stale copy left
from before RFC-0060 moved it. It confuses exactly this analysis. Their
cleanup, not ours.

## W3 — Documentation

- [ ] `CLAUDE.md`: the repo layout section, and the "Standalone crates" note
- [ ] `docs/guide/parser-migration.md` if any path changes
- [ ] Record in the design doc which alternative was chosen and why, if W0
      lands on something other than the proposal

## Risks

**The merge breaks the workspace boundary silently.** The whole isolation
rests on `exclude`. If layer 2 accidentally becomes a workspace member, the
`rclrs` patches enter its graph and nano-ros's build starts needing a ROS
toolchain. W1's acceptance test is precisely this check, which is why it runs
from a stripped environment rather than a developer shell that happens to
have ROS sourced.

**History loss.** `ros-launch-resolve` and `play_launch_parser` have their own
histories. A naive copy discards them. Prefer `git subtree`/unrelated-history
merge; if the history is judged not worth the merge complexity, say so
explicitly rather than losing it silently.

**nano-ros breakage.** Their setup script builds the resolver from a pinned
`ros-launch-resolve`. After W1 that path changes. Their side is nano-ros
phase-332; the two must land in a known order — W1 here first, since their
phase pins a play_launch commit that must already contain the merged tree.
