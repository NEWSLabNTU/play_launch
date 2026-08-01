# play_launch Issues

Bugs and frictions to fix — one file per issue, `NNNN-slug.md` with YAML
frontmatter (`id`, `title`, `status: open|resolved|wontfix`, `type`,
`severity`, `github`). Numbering tracks the GitHub issue ids
(github.com/NEWSLabNTU/play_launch/issues). Resolved issues move to
`archived/`. Design direction lives in `docs/design/`; implementation
plans in `docs/roadmap/` phase docs.

## Open

**#0012** — attribute allowlists are pinned to Humble's exact surface, so a
valid launch file on Jazzy/Rolling (`sigkill_timeout`, `sigterm_timeout`) hits
a hard `UnexpectedAttribute`. Worse, `attr_differential.rs` hard-codes
`/opt/ros/humble` and treats a missing one as its "no ROS 2" skip sentinel —
so the test built to catch table drift silently passes on exactly the machines
where drift is guaranteed. See `0012-*`.

**#0011** — `declare_argument`, `unset_env`/`unset-env`, and
`pop-ros-namespace` are dispatched by the traverser but have no `AttrSpec`, so
unknown attributes on them are accepted silently. The design's scope said
"every element the parser handles"; failing open produced no signal that it
was not met. See `0011-*`.

**#0010** — YAML attribute validation runs only on the top-level action
mapping, so nested `param:`/`remap:`/`env:`/`composable_node:` keys are
unchecked (the `composable_node` spec has no YAML enforcement path at all).
XML validates two levels. Also: a non-mapping action body (`- node: null`) is
a silent no-op. See `0010-*`.

**#0009** — `attr_strictness.rs` names scratch fixtures by
`std::process::id()`, which is constant across tests sharing a process, so
bare `cargo test` collides and looks flaky. `cargo nextest` (what the gates
use) is unaffected. Four separate agents hit it independently. See `0009-*`.

**#0013** — four orphaned `ros-launch-resolve` CLI helpers carry
`#[allow(dead_code)]` + an "UNRESOLVED DISPOSITION" comment pending a
wire-up/relocate/delete decision. Verified orphaned-but-live (they compile;
they just have no caller) rather than genuinely dead. See `0013-*`.

**#0007** — parameter source ORDERING is lost: the parser forks sibling
`<param name=>` / `<param from=>` children into two vectors, so inline values
always win even when the launch file wrote a file last. ROS 2 treats
`parameters=[dict|file, …]` as ONE ordered list where the later entry wins.
Silent wrong values on the spawn path; the nano-ros bake inherits the same
divergence through the split model. Fix direction: one ordered
`param_sources` vec at the fork in `actions/node.rs`, with `params`/
`params_files` kept as derived views for migration. See `0007-*`.

## Resolved

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
