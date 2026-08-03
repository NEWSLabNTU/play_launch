# CLI migration: 0.8.x → 0.9.0

0.9.0 reshapes the verbs. Every removed verb still parses and errors with
its replacement spelled out, echoing your own arguments back — you will not
get `unrecognized subcommand`.

| 0.8.x | 0.9.0 |
|---|---|
| `play_launch replay m.yaml` | `play_launch up m.yaml` |
| `play_launch check <pkg> <f>` | `play_launch launch <pkg> <f> --check` |
| `play_launch check <pkg> <f> --format json` | `ros-launch-resolve check <pkg> <f> --format json` |
| `play_launch resolve <pkg> <f> -o m.yaml` | `ros-launch-resolve resolve <pkg> <f> -o m.yaml` |

## Why

**`replay` → `up`.** It replayed nothing. It loads a resolved SystemModel and
supervises what it spawns. The name was a fossil of `record.json`, retired in
Phase 47.

**`check` → a flag plus a layer-2 verb.** `launch` already validated contracts
and scheduling on every run, so `check` was the same pipeline minus the spawn.
The pass/fail gate is now `--check`. The diagnostics moved to
`ros-launch-resolve check`, where they run **without a ROS install** — the
checker never needed one.

**`resolve` removed.** It had been a delegate to `ros-launch-resolve` since
RFC-0060, printing a deprecation warning on every call.

## `run --check`

`run` validates the scheduling platform file only, and says so. Contracts are
keyed by launch file and `run` has none, so no contract sidecar can apply.

## Compatibility

The hidden verbs are removed at 1.0.0. Migrate rather than relying on them.
