# play_launch Issues

Bugs and frictions to fix — one file per issue, `NNNN-slug.md` with YAML
frontmatter (`id`, `title`, `status: open|resolved|wontfix`, `type`,
`severity`, `github`). Numbering tracks the GitHub issue ids
(github.com/NEWSLabNTU/play_launch/issues). Resolved issues move to
`archived/`. Design direction lives in `docs/design/`; implementation
plans in `docs/roadmap/` phase docs.

## Open

**#0002** — composable state triple-write (actor entry + shared_state +
ignored StateEvent), duplicated BlockReason enums. See `0002-*`. (GH #2)

**#0003** — ContainerActor god struct: lifecycle + supervision + ROS
clients + web state in one 20-field struct. See `0003-*`. (GH #3)

**#0004** — 12+ hardcoded timeout Durations (LOAD_TOTAL_BUDGET 600s,
200ms warmup sleep), none configurable; flat 59-flag CLI. See `0004-*`. (GH #4)

**#0005** — "Startup complete with failures" warn-and-continue; swallowed
sends; unbounded ComponentEvent channel; stringly eyre. See `0005-*`. (GH #5)

**#0006** — web UI organization: bare-name-first list, container-only
tree, no status facets, three freshness mechanisms. See `0006-*`. (GH #6)

## Resolved

**#0001** — node page misses nodes (bare-name key collisions; orphaned
composables dropped). Fixed by phase-50 canonical member ids
(`kind:/ns/name[#N]`) end-to-end + model-key ordinal dedup. See `0001-*`.
(GH #1)
