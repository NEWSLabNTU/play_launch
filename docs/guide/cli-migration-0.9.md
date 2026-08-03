# CLI migration: 0.8.x → 0.9.0

One verb was renamed, and two verbs gained a flag. Everything else is
unchanged — `launch`, `run`, `resolve`, `dump`, `check`, `plot`, `contract`,
`context`, `setcap` and `verify` all spell and behave as they did in 0.8.x.

`replay` still parses and errors with its replacement spelled out, echoing
your own arguments back — you will not get `unrecognized subcommand`.

| 0.8.x | 0.9.0 |
|---|---|
| `play_launch replay m.yaml` | `play_launch up m.yaml` |

## Why `replay` → `up`

It replayed nothing. It loads a resolved SystemModel and supervises what it
spawns. The name was a fossil of `record.json`, retired in Phase 47.

`up` takes the model path positionally or as `--model <path>`; it is required
either way, since the model is the only thing `up` spawns from. Produce one
with `play_launch resolve <pkg> <file> -o m.yaml` or `play_launch dump launch
<pkg> <file>`.

## New: `--check` on `launch` and `run`

`launch` already validated contracts and scheduling on every run, so a
validate-only pass was the same pipeline minus the spawn. `--check` runs the
validation and exits with its verdict instead of spawning:

```bash
play_launch launch <pkg> <file> --check
play_launch run <pkg> <exe> --sched system.posix.yaml --check
```

`play_launch check <pkg> <file>` is still there and still the richer tool —
`--format json`, `--explain`, `--contracts <root>`. `--check` is the
one-line gate for the command you were already going to run.

`run --check` validates the scheduling platform file only, and says so.
Contracts are keyed by launch file and `run` has none, so no contract sidecar
can apply. "Validates" means it loads and derives the plan, the same way `run`
without `--check` does — a `--sched` path that does not exist or does not
parse fails the check and exits non-zero. It never reports OK for a file it
merely located.

## Compatibility

The hidden `replay` verb is removed at 1.0.0. Migrate rather than relying on
it.
