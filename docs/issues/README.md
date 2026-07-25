# play_launch Issues

Bugs and frictions to fix — one file per issue, `NNNN-slug.md` with YAML
frontmatter (`id`, `title`, `status: open|resolved|wontfix`, `type`,
`severity`, `github`). Numbering tracks the GitHub issue ids
(github.com/NEWSLabNTU/play_launch/issues). Resolved issues move to
`archived/`. Design direction lives in `docs/design/`; implementation
plans in `docs/roadmap/` phase docs.

## Open

**#0006** — web UI organization: bare-name-first list, container-only
tree, no status facets, three freshness mechanisms. See `0006-*`. (GH #6)

## Resolved

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
