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

## Where `ros-launch-resolve` comes from

`pip install play_launch` installs it. The 0.9.0 wheel ships both binaries —
`play_launch` (launch, run, up; needs a ROS install) and `ros-launch-resolve`
(resolve, dump, check, contract, plot; needs no ROS install at all). Nothing
extra to install, no second package. In a source checkout, `just build`
builds it into `src/ros-launch-resolve/target/release/`.

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

"Validates" means it loads and derives the plan, the same way `run` without
`--check` does — a `--sched` path that does not exist or does not parse fails
the check and exits non-zero. It never reports OK for a file it merely
located.

## Compatibility

The hidden verbs are removed at 1.0.0. Migrate rather than relying on them.
