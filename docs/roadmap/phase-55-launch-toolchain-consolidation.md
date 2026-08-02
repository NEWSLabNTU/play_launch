# Phase 55 — Launch toolchain consolidation

**Design:** `docs/design/launch-toolchain-topology.md`
**Blocked on:** nano-ros amending RFC-0060 (Stable) — see W0
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

**Waiting on W0.** Everything that does not depend on the RFC decision is
done; nothing that does has been started.

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

## W0 — Ratify the boundary change (BLOCKING)

RFC-0060 is Stable and specifies three repositories. This phase proposes two.
That is nano-ros's RFC to amend, and layer 3 does not get to redraw a
boundary its consumers depend on.

- [ ] nano-ros amends RFC-0060: keep the layering, the linking rule and the
      process boundary; replace "three repositories" with "three workspaces,
      two repositories, one shared schema repository"
- [ ] Their `ARCHITECTURE.md` updated in the same commit (their drift rule)
- [ ] If nano-ros prefers the repository boundary for reasons outside this
      repo's view — release cadence, access control, CI cost — **stop here**
      and close this phase as wontfix. That is a sufficient answer.

Nothing below starts until W0 lands.

## W1 — Merge layer 2 in

- [ ] `git submodule deinit` + remove `src/ros-launch-resolve`; add its
      contents as plain directories, preserving history where practical
      (`git subtree add` or a merge with `--allow-unrelated-histories`)
- [ ] Same for the nested `parser` submodule → a plain directory
- [ ] Keep `src/ros-launch-resolve` (or its new path) in the root workspace's
      `exclude`, with the existing comment intact — it is the load-bearing bit
- [ ] `ros-launch-manifest` stays a submodule for now; W2 converts it

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

## W2 — `ros-launch-manifest` by tag

- [ ] Tag `ros-launch-manifest`; `play_launch` depends on it by git tag rather
      than through the nested submodule path
- [ ] Drop the nested submodule
- [ ] Coordinate the tag with nano-ros — they consume the same crates and must
      move together or pin the older tag deliberately

**This cannot be done one repository at a time.** Measured 2026-08-02: three
repositories reach the manifest crates by path through the nested submodule —
`play_launch/src/play_launch/Cargo.toml`
(`../ros-launch-resolve/third-party/ros-launch-manifest/...`),
`ros-launch-resolve` itself (workspace deps), and nano-ros in four manifests
(`nros-cli-core`, `nros-orchestration-ir`, `nros-macros`, `nros-tests`, via
`.../third-party/ros-launch-resolve/third-party/ros-launch-manifest/...`).

Convert one and not the others and cargo resolves two distinct packages with
the same name from different sources. They do not unify, so every type that
crosses the boundary — `SystemModel` above all — becomes two incompatible
types and the consumer stops compiling. The three moves have to land
together.

That coupling is independent of W0: it is true whether or not the merge
happens, and it is the strongest argument that the *repository* count is not
what makes these layers separable.

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
