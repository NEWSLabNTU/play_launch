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

## Cross-track (nano-ros)

The shared `model` crate's `NodeInstance` gains launch fields (`remaps`,
`ros_args`, `respawn`/`respawn_delay`, `env`) — additive, nano-ros vendors them
and uses what it needs (it *wants* remaps; it was previously unable to get them
since they lived only in Linux `record.json`). No `deny_unknown_fields`, so
staged rollout is safe. Coordinate the field set + semantics with RFC-0050
before landing (same handshake as the sched-SSoT reconciliation).

## Migration (Phase 46)

Additive-first, no flag day:
1. `model` crate `NodeInstance` gains the launch fields (cross-track).
2. `resolve`/parser populate them into the model.
3. play_launch spawn path derives cmd/exec-path/env from the model (the
   relocation of `node_cmdline.rs`); `replay` reads the model, not `record.json`.
4. `dump` emits the model; `record.json` retired; `meta.record` binding removed.
5. Docs + guide; nano-ros consumes the new fields (their track).

## Non-goals

- Storing platform-resolved outputs (exec path, argv) in the shared artifact —
  those stay derived per-consumer, by principle.
- Changing the contract or sched layers (Phases 44/45 stand).
- nano-ros's own consumption of the new fields (its track).
