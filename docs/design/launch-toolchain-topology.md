# Launch toolchain topology: three boundaries, not three repositories

**Status:** Accepted and implemented (proposed 2026-08-02, shipped 2026-08-03)
**Relates to:** nano-ros RFC-0060 (Stable), amended by `9baebb2eb` to match
**Implemented by:** play_launch phase-55 W1/W2
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

(That comment is quoted as it stood when this was proposed. Post-merge it
reads differently — the directory is no longer a submodule, and the comment
now says this line is the *only* thing preventing the graphs from merging,
naming the gate that fails if it is dropped.)

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

## Enforcing it

The measurements above were a snapshot. As of 2026-08-02 they are a gate:
`ros-launch-resolve/scripts/check-layer2-isolation.sh`, wired into
`just check` and into a CI job that runs on a bare runner with no ROS
installed at all rather than in the builder container.

The invariant was already written down — the layer-2 workspace manifest says
this workspace must build "with no ROS install, no ament environment and no
colcon" — but nothing checked it, and the failure mode is silent: every
developer shell and every CI job in this repo has ROS sourced, so a resolver
that started needing `rclrs` would keep passing the whole suite while becoming
unbuildable for nano-ros. The gate asserts its preconditions and *fails* when
they are not met rather than skipping, because issue 0012 was precisely a
check that reported success on the machines where the drift it looked for was
guaranteed.

Two things surfaced while building it.

**The `exclude` is necessary but not sufficient.** Cargo walks parent
directories for `.cargo/config.toml`, and that walk does not stop at a
workspace boundary or at the `exclude` list. Checked out inside `play_launch`,
layer 2 inherits its colcon-generated `[patch.crates-io]` and
`-L native=.../install/...` rustflags. This is inert today — the patches land
in `Cargo.lock` as `[[patch.unused]]`, which is itself evidence the graph does
not want them — and it is unchanged by the merge, since the directory sits in
the same place either way. But it means an in-tree run is not clean-clone
equivalent, so the gate reports the inheritance and offers `--standalone`,
which copies the workspace outside every contributing parent and builds there:
12.1s cold, matching the 11.6s measured independently. CI needs no such trick,
because `.cargo/config.toml` is gitignored and never checked out.

**A zero here is a measurement, not a broken grep.** The same detector run
against `play_launch` — layer 3 — flags `ament_rs`, `rclrs`, `rosidl_cargo`
and `rosidl_runtime_rs`.

## What the seam costs, observed

While wiring the gate, `just check` turned out to have been failing since
`adc33a7`, the commit that extracted the CLI. It had left `DumpArgs`,
`DumpSubcommand` and two tests referring to a `Command::Dump` variant it had
just deleted, so `cargo check --all-targets` did not compile. Cleaning that up
found `PlotArgs` and three `Contract*` structs in the same state, and the
mirror-image defect on the other side: the extracted binary's `--help`
introduced itself as `play_launch` and advertised `launch`, `run` and
`replay` — verbs it has never had.

None of it warned, for one reason: **`pub` items in a `pub` module are never
reported as `dead_code`**. The residue of a verb moved across a repository
boundary is invisible to the compiler on both sides.

This is evidence for the argument above rather than a digression. The claim is
not that repository splits cause bugs; it is that *this* seam has no
mechanical check across it, so drift accumulates until something unrelated
trips over it. The layering has such a check — that is what the isolation gate
is. The repository boundary has none, and it is not obvious what one would
look like.

## Consequences

- `play_launch` and nano-ros each depend on `ros-launch-manifest` by git tag,
  not through a submodule of a submodule.
- nano-ros pins the `play_launch` repository and builds
  `cargo build -p ros-launch-resolve-cli` from the nested workspace. It gets
  CPython and not `rclrs`, exactly as today.
- One pointer to bump instead of three.
- `ros-launch-manifest` stays independent because it is the genuine shared
  contract — the only thing both projects *link*.

## The conflict, and how it was resolved

RFC-0060 was Stable and specified three repositories; this proposed two plus
the schema crate. A real contradiction, not a detail, and nano-ros's RFC to
amend — layer 3 does not get to unilaterally redraw a boundary its consumers
depend on. So it was put to them as a blocking gate, with rejection recorded
in advance as a legitimate outcome rather than an obstacle to argue past.

They accepted (`9baebb2eb`, phase-332 W0): three layers kept, the linking rule
kept, the process boundary kept, "three repositories" replaced. Status stays
Stable. The four reasons that would have justified rejection — independent
release cadence, access control, CI cost, and not wanting nano-ros's build to
pin a repository that also carries a C++ container and a web UI — were
considered and cleared.

The last of those was weakened by measurement after the fact: a plain `git
clone` of play_launch with **no submodules initialised at all** builds layer 2
in 10.9s, because the C++ packages and vendored ROS crates are submodules
nobody needs to fetch to use the resolver.

**Shipped:** W1 folded layer 2 in as plain directories via `git subtree`, both
histories intact. W2 replaced the last submodule with a git dependency on
`ros-launch-manifest` pinned to `v0.1.0`. Three levels of nesting → zero.

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
