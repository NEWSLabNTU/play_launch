# Unified SystemModel — One Complete Artifact — Design

**Date:** 2026-07-20
**Status:** Approved (direction), pending implementation (Phase 46)
**Revises:** Phase 43's two-artifact decision (`phase-43-runtime_consumes_system_model.md`
§"Decision: two-artifact runtime") — that decision is superseded.
**Cross-track:** the `model`-crate field additions are shared with the nano-ros
track (RFC-0050/0052) — coordinate; the shared model gains launch fields
nano-ros vendors.

## Principle

**The SystemModel is the one complete artifact.** It carries ALL
launch-derived information (+ optional contract + optional system config) —
the full resolved launch, not a portable subset. Both consumers — the
play_launch Linux runtime and the nano-ros build system — read the entire
model and each resolves *its own* platform specifics from it. Neither rescans
the launch files; there is no separate `record.json`.

This corrects the Phase 43 split, which assumed most spawn detail was
Linux-only. In fact most of it (package, node name, params, remaps, ros_args,
respawn) is launch-derived and wanted by both runtimes — only the *resolution*
of a few things (exec path, argv) is platform-specific, and each consumer does
that itself.

## Layering

Not "shared base + Linux annotations." One model; the platform-specific parts
are **derived at use time, never stored**:

- **In the model (launch inputs, shared):** everything the parser resolves —
  `structure.nodes[]` gains `remaps`, `ros_args`, `respawn`/`respawn_delay`,
  launch-declared `env`; already carries `pkg`, `exec` (name), `plugin`/
  `container`, `lifecycle`, resolved `params` values, `criticality`; plus the
  `contracts` and `execution` (sched) layers.
- **Derived per-consumer, not stored:**
  - resolved exec **path** — play_launch via ament_index; nano-ros links its
    own binary.
  - assembled **cmd / argv** — play_launch builds it at spawn from
    exec-path + ros_args + remaps + a materialized params file; nano-ros
    doesn't use argv.
  - runtime-injected **env** (LD_PRELOAD interception, fd vars) — play_launch
    spawn-time only.
  - materialized **param files** — play_launch writes temp YAML from the
    model's resolved `params` values (it already does this for containers).
- **Dropped from the artifact entirely:**
  - `file_data` cache — a parser read-cache; params are already resolved.
  - `variables` — the launch-arg binding, already captured in `meta.args`.
  - `meta.record` + `verify_model_record_binding` — one file, nothing to
    mismatch.

## Consequence: play_launch derives cmd at spawn

Today the parser assembles the Linux command line at parse time and pre-bakes
it into `record.json` (`node_cmdline.rs`). Under this design the model carries
the *inputs* (exec name, ros_args, remaps, params values, env); play_launch's
spawn path must *derive* the exec path (ament) + assemble the argv + inject
runtime env at spawn, consuming the model. This relocates cmdline assembly from
parse-time-into-record.json to spawn-time-from-model. It is the main
engineering cost of the unification, and it is the same "each consumer resolves
its own platform specifics" step nano-ros already performs.

## The artifact and commands

- One artifact: `system_model.yaml` (the model). `record.json` retires.
- `play_launch resolve` emits the model (unchanged command; now complete).
- `play_launch dump` emits the model (was: `record.json`; now the same one
  artifact — the user perceives one kind of dump).
- `play_launch replay <model.yaml>` reads the model and spawns (deriving
  cmd/exec-path); `--input-file` accepts the model.
- `launch` = resolve-then-replay over the one model (Phase 43.5 flow, minus the
  record companion).

## Cross-track (nano-ros) — corrected 2026-07-20 after studying their bake

Studying nano-ros's model bake (`.superpowers/sdd/nano-ros-study.md`)
**corrected an assumption**: nano-ros does NOT need `remaps`/`ros_args`/
`respawn`/`env`. It bakes from *resolved topic names* (`structure.topics`),
not raw remaps (its own finding: "remaps NOT a gap"); embedded targets have no
argv/process/respawn model, and `ros_args`/`respawn`/`env` appear nowhere in
its code or docs. So those four fields are **Linux-serving launch info, not a
shared need**. Two options, user's call:
- (a) Per the "all launch info in the shared model" principle, still put them
  in `NodeInstance` (additive; nano-ros ignores them — no
  `deny_unknown_fields`). Simplest artifact story; slightly heavier shared
  model.
- (b) Keep them play_launch-side (the model carries what's genuinely shared;
  these ride in a play_launch-owned section). Leaner shared model.

**The one field nano-ros genuinely needs and can't get: `<node machine=>`
(multihost).** It's standard ROS 2, mature in nano-ros since phase-263, and
the shared model ALREADY has its home — `execution.deploy[fqn].host`. But
play_launch **drops it**: the parser captures `machine` (ir.rs), yet the
`LaunchDump` `NodeRecord` has no `machine` field, so it's lost before
`model_builder` and `execution.deploy.host` stays empty (nano-ros issue #236 —
blocks their multihost workspace migration). **Fixing this drop is the real
cross-track win of Phase 46** and belongs here since Phase 46 reworks the
launch→model field-population path anyway.

**Decision (2026-07-20): option (a).** `remaps`, `ros_args`, `respawn`/
`respawn_delay`, and launch-declared `env` are added to the shared
`NodeInstance` per the all-launch-info principle. nano-ros ignores what it
doesn't consume (no `deny_unknown_fields`); the model is the one complete
record of the launch. This keeps a single artifact story and lets any future
consumer read any launch fact without a play_launch-specific side channel.

## Coordination (nano-ros / RFC-0050)

The shared `model` crate is vendored by nano-ros; every `NodeInstance` /
`Deploy` addition here is a schema change they inherit. Same handshake as the
sched-SSoT reconciliation:
1. **Note them first** (nano-ros issue #236 + RFC-0050): our unified-model
   design, the `machine=`→`deploy.host` fix landing now, and the additive
   launch fields coming. Confirm they've seen the design and flag any field
   they read that the model still omits.
2. Land additive schema changes (backward-compat; old models parse).
3. nano-ros consumes the new fields on their track (they ignore the
   Linux-serving ones by design).

## Migration (Phase 46) — additive-first, no flag day

1. **Coordinate** (46.0): note nano-ros, confirm the field set.
2. **Fix #236 first** (46.1, independent quick win): `<node machine=>` →
   `execution.deploy.host`; unblocks nano-ros's paused multihost migration.
   Ships ahead of everything else.
3. **Shared launch fields** (46.1b/46.2): `NodeInstance` gains
   `remaps`/`ros_args`/`respawn`/`env` (cross-track, additive); parser +
   `model_builder` populate them.
4. **Spawn-from-model** (46.3, the load-bearing item): relocate
   `node_cmdline.rs` so play_launch derives exec path (ament) + argv +
   injected env + materialized param files from the model at spawn; `replay`
   reads the model, not `record.json`. Regression-gated against current
   `record.json`-driven spawns (Autoware + rt_workspace must match).

## Retirement (of the two-artifact runtime)

Only after (46.3) proves spawn-from-model matches the `record.json` era:
5. `dump` emits the one model; **`record.json` retired**; the `meta.record`
   hash-binding + `verify_model_record_binding` + the `resolve` record
   companion removed.
6. Drop the LaunchDump-only artifacts (`file_data` cache, `variables`) from
   the emitted model; keep whatever the parser needs internally.
7. Docs + guide: one-artifact story across `dump`/`resolve`/`replay`.
Rollback safety: through step 4 both paths coexist (the model is additive,
`record.json` still written); retirement (5+) is the only non-additive step
and is gated on the spawn-from-model regression suite.

## Non-goals

- Storing platform-resolved outputs (exec path, argv) in the shared artifact —
  those stay derived per-consumer, by principle.
- Changing the contract or sched layers (Phases 44/45 stand).
- nano-ros's own consumption of the new fields (its track).
