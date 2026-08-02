# Launch toolchain topology: three boundaries, not three repositories

**Status:** Proposed (2026-08-02)
**Relates to:** nano-ros RFC-0060 (Stable) — this proposes an amendment
**Supersedes:** nothing

## Summary

The launch toolchain has three layers, and they are correct. What is not
carrying its weight is the assumption that each layer needs its own
**repository**.

Three different boundaries are in play, and they do different jobs:

| Boundary | What it buys | Verdict |
|---|---|---|
| **Cargo workspace** | keeps `rclrs`/`rosidl` out of the resolver's dependency graph | load-bearing — keep |
| **Process** (resolver is a binary) | keeps `libpython` out of the consumer's link, while the resolver still embeds CPython for `.launch.py` | load-bearing — keep |
| **Repository** | nothing the other two do not already provide | costs submodule nesting — drop |

The proposal: fold `ros-launch-resolve` and `play_launch_parser` into the
`play_launch` repository as plain directories, **keeping their separate Cargo
workspace**, and keep `ros-launch-manifest` as an independent repository
consumed by tag.

## The layers are not in dispute

nano-ros RFC-0060 draws the layering along "what a consumer must be able to
link", and that line is right:

```
ros-launch-manifest    spec + algorithms      serde/toml/yaml. no ROS, no Python.
       ↑
ros-launch-resolve     launch tree → model    + CPython. no rclrs, no colcon.
       ↑
play_launch            Linux runtime          + rclrs, generated msgs, colcon.
```

Nothing here changes the layering, the direction of dependency, or what any
consumer links. nano-ros continues to depend on layers 1–2 and never on
layer 3.

## What the repository split does not deliver

**It does not deliver the Python isolation.** That is a property of the
process boundary. `ros-launch-resolve` has a *mandatory* `pyo3` dependency
with `auto-initialize`, and so does `play_launch_parser` — the split never
removed CPython from layer 2, and was never intended to. nano-ros's own
Cargo comment states the actual mechanism: the resolver "embeds CPython via
pyo3 `auto-initialize`, and abi3 cannot unpin an embedding binary's
libpython… Keeping it out of the **link** makes the shipped `nros` a portable
libc-only binary." Layer 2 needs Python because scanning `.launch.py` requires
an interpreter. That is a feature, not a leak.

**It does not deliver the graph isolation either.** That is a property of the
Cargo workspace. `play_launch`'s root manifest already says so:

```toml
exclude = [
    # RFC-0060 layer 2 — its own workspace, vendored as a submodule. Listing it
    # as a member here would merge the two graphs and defeat the split.
    "src/ros-launch-resolve",
]
[patch.crates-io]
rclrs = { path = "src/vendor/ros2_rust/rclrs" }
rosidl_runtime_rs = { path = "src/vendor/rosidl_runtime_rs/rosidl_runtime_rs" }
```

The `exclude` is what keeps the `rclrs`/`rosidl` patches out of layer 2's
graph. A directory inside the repository can be excluded exactly as a
submodule can.

## What it costs

Three-level submodule nesting: `play_launch` → `ros-launch-resolve` →
{`play_launch_parser`, `ros-launch-manifest`}. Every change that spans layers
needs two or three pointer-bump commits in a fixed order.

This is not hypothetical. During the 2026-07-31 `machine=` removal, that
nesting produced: a pointer bump dropped as "already upstream" after being
resolved during a rebase, two agents racing on the same submodule tree (one
detached and re-pinned mid-task to avoid clobbering the other), and three
separate commits whose only content was moving a pointer. RFC-0060 cites
"the submodule drift that bit three times during the 0285 work" as a
motivation for the split — the same class of failure the split then
reproduced one level deeper.

## Evidence the merge is safe

Measured 2026-08-02 against the current layout, before proposing anything.

| Check | Result |
|---|---|
| `rclrs`/`rosidl` in `cargo tree -p ros-launch-resolve-cli` | **0** of 294 deps |
| Build from a clean clone — no `install/`, no `build/`, ROS env stripped | **succeeds, 11.6s** |
| ROS/rcl/rmw/ament shared libraries in the binary | **0** |
| `libpython3.10` linked | **yes** — required |
| Resolves XML with no ROS sourced | **yes** |
| Resolves **`.launch.py`** with no ROS sourced | **yes** — node captured, verified in the model |

The last row was better than expected and is worth recording: `import launch`
**fails** in that stripped environment, yet `.launch.py` resolves and the
declared node appears in the output. The parser supplies its own
`launch`/`launch_ros` API through pyo3 mock modules in `src/python/api/`. So
layer 2 needs **CPython, not a ROS installation**.

Caveat, so this is not read as more than it is: the test used a
self-contained launch file. Anything using `$(find-pkg-share …)` still needs
`AMENT_PREFIX_PATH` at runtime. "No ROS needed" holds for building and
running the resolver, not for every launch file it may be handed.

## Consequences

- `play_launch` and nano-ros each depend on `ros-launch-manifest` by git tag,
  not through a submodule of a submodule.
- nano-ros pins the `play_launch` repository and builds
  `cargo build -p ros-launch-resolve-cli` from the nested workspace. It gets
  CPython and not `rclrs`, exactly as today.
- One pointer to bump instead of three.
- `ros-launch-manifest` stays independent because it is the genuine shared
  contract — the only thing both projects *link*.

## The open conflict

**RFC-0060 is Stable and specifies three repositories.** This proposes two
plus the schema crate. That is a real contradiction, not a detail, and it is
nano-ros's RFC to amend — layer 3 does not get to unilaterally redraw a
boundary layer 2's consumers depend on.

The amendment being proposed is narrow: keep the layering, keep the linking
rule, keep the process boundary; replace "three repositories" with "three
workspaces, two repositories, one of them shared". If nano-ros prefers the
repository boundary for reasons outside play_launch's view — independent
release cadence, access control, CI cost — that is a sufficient answer and
this proposal should be dropped rather than argued.

## Alternatives considered

**Promote `ros-launch-resolve` to a top-level repository.** Removes one level
of nesting instead of two, and adds a third repository to version-coordinate.
Strictly worse than merging unless the resolver acquires consumers other than
these two.

**Move `ros-launch-resolve` into nano-ros.** Ruled out on a fact:
`play_launch` *links* the resolve library (`ros-launch-resolve = { path = … }`
in its manifest), so this inverts the dependency — layer 3 would depend on a
consumer. It only works if play_launch re-absorbs its own copy of the
resolver, which restores the duplication the split removed.

**Feature-gate `pyo3` so layer 2 can build without Python.** Ruled out: nano-ros
requires `.launch.py` support at scan time. The 29-of-30 pyo3-using files
under `src/python/` make the gating mechanically easy, and it would still be
wrong — `$(eval …)` routes unconditionally to CPython
(`substitution/eval.rs:75`; the Rust evaluator `needs_python_eval` no longer
exists), and nano-ros's own multi-host launch files use `$(eval …)`.
