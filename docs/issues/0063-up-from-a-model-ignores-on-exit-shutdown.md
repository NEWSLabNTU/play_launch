---
id: 63
title: "`play_launch up <model.yaml>` ignores `on_exit=Shutdown()`: the SystemModel does not carry it"
status: resolved
type: defect
severity: medium
---

# 0063 - `up` from a resolved model ignores `on_exit=Shutdown()`

**Repo:** `play_launch` at `785cddb7`
**Affects:** `src/play_launch/src/commands/up.rs:159` and `:1107-1126`; the
SystemModel schema (`ros-launch-manifest`'s `model` crate); the resolve
pipeline in `src/ros-launch-resolve/resolve`
**Found:** review of the commits pulled 2026-10-11, confirmed by reading.

## What happens

```sh
play_launch resolve scenario.launch.py -o m.yaml
play_launch up m.yaml
```

The process declared `on_exit=Shutdown()` exits, and the rest of the stack
keeps running. `play_launch launch scenario.launch.py` stops correctly.

fcd2188a ("the Rust parser honours on_exit=Shutdown") gives the motivating
case as "a scenario stack resolved with it stayed up after its runner exited".
`resolve` followed by `up` is that case, and it still stays up.

## Why

`up.rs:1107` builds the set of required processes from
`launch_dump.node[].on_exit_shutdown`. On the `launch` path the dump is the
one just parsed, held in memory. Standalone `up` passes `LaunchDump::empty()`
(`up.rs:159`), because since phase 47 the model is the only artifact, and
nothing carries `on_exit` from the dump into the model. Neither the model
crate nor `ros-launch-resolve` mentions it. The comment above the read even
says why: "on the model path `context.record` is rebuilt from a NodeInstance
that carries no launch-level handlers."

Same family as the documented gap for `GET /api/launch-tree` on standalone
`up`. A consumer reads the in-memory dump, so it works under `launch` and
silently degrades under `up`. The difference is that this one changes what
the running system does.

## Fix

Carry it on the model: a `required: bool` (or `on_exit: shutdown`) on the
node and container instances, lowered by `model_builder` from
`on_exit_shutdown`. `up` then reads the model, and `launch` reads the same
field, so the dump read at `up.rs:1112` can go. This is a schema change in
`ros-launch-manifest`, so it needs a tag bump (`just bump-manifest`).

## Verification, when fixed

`tests/tests/on_exit_shutdown.rs` gains a case that runs `resolve -o m.yaml`
then `up m.yaml`, with the same four checks the `launch` cases make: exit 0,
not before the required process exits, the on_exit log line present, and the
bystander gone.

## Resolution (2026-10-11)

The model carries it. `ros-launch-manifest` v0.1.50 adds
`NodeInstance.on_exit: Option<OnExit>` (`OnExit::Shutdown`, serialised
`shutdown`) beside `respawn`. It is a launch fact, so it passes
`docs/model-boundary.md`. `model_builder` lowers it from the dump for nodes
and containers. `node_record_from_instance` puts it back on the record each
spawn context is built from, so `launch` and `up <model.yaml>` read the same
field. The dump read in `up.rs` is gone.

Tests (`on_exit_shutdown.rs`): `up_from_a_resolved_model` and
`up_from_a_resolved_xml_model`, under the same four checks as the `launch`
cases. Negative control: without the model-to-record line, both hang to the
60 s timeout.
