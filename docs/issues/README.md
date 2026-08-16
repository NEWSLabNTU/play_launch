# play_launch Issues

Bugs and frictions to fix — one file per issue, `NNNN-slug.md` with YAML
frontmatter (`id`, `title`, `status: open|resolved|wontfix`, `type`,
`severity`). These files ARE the issue tracker; numbering is local and
sequential, allocated by taking the next free `NNNN`. GitHub issues are not
used. (`0001`–`0006` carry a trailing `(GH #N)` from when numbering mirrored
github.com/NEWSLabNTU/play_launch/issues — historical, not a live link.)
Resolved issues move to `archived/`. Design direction lives in
`docs/design/`; implementation plans in `docs/roadmap/` phase docs.

An issue may also cover `ros-launch-manifest`, the one remaining external
repository (a git dependency pinned by tag since phase-55 W2) — it has no
tracker of its own. Name the repo in the issue body. `ros-launch-resolve` and
`play_launch_parser` are no longer separate repositories; they live in
`src/ros-launch-resolve/` here.

## Open

**#0017** — the model's node FQN is not the node's ROS name. For a `<node>` with
no `name=`, `structure.nodes` keys by the EXECUTABLE while the node registers
its compiled-in name (`…/autoware_ekf_localizer_node` vs `…/ekf_localizer`);
19 of 144 FQNs on the golf cart stack never appear in `ros2 node list`.
play_launch matches stock `launch_ros` here — it emits `__node` only when a name
was declared, and forcing it was bug `af7c524` — so the fix is not to rename the
nodes but to stop presenting a synthetic key as a ROS name. Full mapping,
options and the one node that is genuinely missing for other reasons in
`0017-*`.

**#0015** — a file capability lives on the inode, so `just build` replaces
`play_launch_rt_helper` and drops the `cap_sys_nice` that `just setcap`
granted; the failure surfaces on the next RT run. The guide documents this,
so the gap is timing rather than knowledge: nothing says it at the moment of
loss. Friction, not a correctness hole — `--sched-apply strict` errors clearly
and `rt_av_demo`'s `ab` recipe refuses rather than reporting a vacuous
comparison. See `0015-*`.

## Resolved

**#0019** — a composable whose CONSTRUCTOR ran past 30 s was SIGKILLed by the
isolated container mid-construction, and play_launch reported it as loaded
anyway. Both halves mattered: first-run TensorRT engine builds live in that
constructor (measured: Autoware's traffic light classifier ~33 s cold, ~45 s
even cached), and the `.engine` is written last, so the kill discarded the work
and recurred every launch. Meanwhile a LoadNode response means only "spawn
requested" under `isolated`, so `Startup complete: all nodes ready` was printed
over a dead node — which is what `84/84 loaded` meant on the golf cart, and the
explanation for #0017's unresolved `motion_velocity_planner`. Timeout made
raisable in the container submodule (`PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS`,
default still 30 s); promotion to `Loaded` now requires ListNodes confirmation.
See `0019-*`.

**#0018** — the two parsers gave un-named nodes different model keys, and the
reference one was the unstable one: Python keys them by `launch`'s global
console process label (`talker-1`), so inserting any node earlier renumbers
every un-named node after it (measured: 2 of 5 identities survived), while
Rust's per-collision ordinal survived 5 of 5. Python keeps its scheme — it is
ported from ROS launch and reflects the original behaviour — and Rust keeps its
stable numbering but switches `#N` to `-N`, numbering un-named nodes from `-1`
so the suffix marks a key as executable-derived rather than authored. `-` is as
collision-proof as `#` was, since `validate_node_name` rejects both. Stability
is enforced by three tests: same-input determinism, unrelated-insertion
invariance, and an explicit assertion of the one case that DOES shift.
See `0018-*`.

**#0016** — `Startup complete: all nodes ready (nodes 12/44, containers 0/16,
composable 0/84)` two tenths of a second into a three-minute startup, followed
by no progress at all. The completion test asked "is nothing in flight?"
(`composable_pending == 0`) when it meant "is everything done?", and those
differ exactly at t=0 — a composable whose container has not started is
`Blocked`, counting as neither pending nor loaded nor failed. It survived on a
race (every container used to spawn within 100 ms) that phase-61's spawn gate
made it lose every time; any slow container reproduces it. Fixed by
`HealthSummary::startup_complete()`. See `0016-*`.

**#0014** — `test_isolated_external_subscriber` hard-coded `ROS_DOMAIN_ID=199`
while every other test gets a unique domain per invocation, so a stale
`ros2-daemon` failed it — and both diagnostic `ros2` calls piped stderr to
`/dev/null`, so the assertion confidently blamed DDS cross-process isolation
for something that was not it. Fixed with the shared domain allocator,
`--no-daemon`, captured stderr, and an assertion that reports observations
instead of naming a cause. Proven by planting daemons on the failing domains:
3/3 PASS where the old test was 3/3 FAIL. See `0014-*`.

**#0013** — orphaned `ros-launch-resolve` CLI helpers. Deleted: the crate is
`[[bin]]`-only with no `[lib]` and nothing can link it, and the verbs they
served stay in play_launch, which has live copies. Measurement found FOUR
dead `Args` structs, not the two annotated. 1934 → 1033 lines, no
`allow(dead_code)` left anywhere. See `0013-*`.

**#0007** — parameter source ORDERING was lost, so an inline `<param>` beat a
later `<param from=>`. Marked resolved by phase-54 in 2026-07, but the
ordering was discarded THREE times after the parser built it (model_builder,
`from_node_record`'s own return struct, and the YAML frontend never built it)
— the shipped behaviour was still the bug. Verified by reading the spawned
command line, fixed end to end 2026-08-02, regression-tested on both
frontends. Composable nodes remain unfixed and are called out at the site.
See `0007-*`.

**#0012** — the allowlists encoded Humble's surface exactly (so valid Jazzy
files hard-errored) and the differential oracle hard-coded `/opt/ros/humble`,
treating its absence as a skip — disabling drift detection on the machines
where drift was guaranteed. Fixed by making the tables the UNION across
supported distros, sourcing whatever ROS 2 is present, and making sanctioned
divergences DIRECTIONAL (more permissive than the oracle is allowed; less
permissive never is). See `0012-*`.

**#0010** — YAML validated only the top-level action mapping, so nested
`param`/`remap`/`env`/`composable_node` keys went unchecked and
`composable_node` had no YAML enforcement path at all. Fixed with
`validate_yaml_child_seq()` at all seven nesting sites; a non-mapping action
body now errors instead of silently no-op'ing. See `0010-*`.

**#0011** — `declare_argument`, `unset_env`, `pop-ros-namespace` fell through
`spec_for`'s None-means-skip and accepted any attribute. Fixed with measured
specs; the measurement also revealed two of them are not ROS 2 elements at
all. See `0011-*`.

**#0009** — PID-named scratch fixtures collided under `cargo test`. Fixed with
`tempfile`; proven by a before/after where the failure COUNT varied between
runs. See `0009-*`.

**#0008** — six integration tests used stale member names after phase-50's
canonical `<kind>:/<name>` ids, and 27 more silently skipped on unbuilt
fixtures (concealing 4 real failures). Originally misfiled as a real
unload-path 500; the unload path was never broken. Fixed by name-aware
helpers, `bare_member_name()` in `is_healthy`, repointing tests + docs at the
relocated `ros-launch-resolve` CLI, and making `just test-all` build the
fixtures and report remaining skips. See `0008-*`.

**#0006** — web UI organization. Fixed by phases 50+53 (nested ns tree,
facets, health strip, banner, backend dedup); membership SSE + CSS
bundling deferred by design. See `0006-*`. (GH #6)

**#0004** — hardcoded timeouts / flat CLI. Fixed by phase-52 LoadTimings
config+CLI knobs and grouped CommonOptions. See `0004-*`. (GH #4)

**#0005** — failure swallowing. Fixed by phase-52 --on-startup-failure,
typed LoadError, bounded event bridge (+51's emit). See `0005-*`. (GH #5)

**#0002** — composable state triple-write + duplicated BlockReason.
Fixed by phase-51 state reducer + model/ unification. See `0002-*`. (GH #2)

**#0003** — ContainerActor god struct. Fixed by phase-51 split
(actor/supervisor/ros_client/timing). See `0003-*`. (GH #3)

**#0001** — node page misses nodes (bare-name key collisions; orphaned
composables dropped). Fixed by phase-50 canonical member ids
(`kind:/ns/name[#N]`) end-to-end + model-key ordinal dedup. See `0001-*`.
(GH #1)
