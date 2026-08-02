# CLI verb reshape — `check` becomes a flag, `replay` becomes `up` (0.9.0)

- **Date**: 2026-08-03
- **Status**: Approved, not yet implemented
- **Repos touched**: `play_launch` and `src/ros-launch-resolve` (one repository since phase-55 W1)
- **Deferred from**: phase-55, which scoped this out as "user-facing and breaking; it gets its own phase"
- **Breaking**: yes — version goes 0.8.2 → 0.9.0

## Problem

`play_launch` has eight verbs. Three of them do no process spawning at all,
and one of those has been a deprecated delegate since RFC-0060:

| verb | what it does | needs a ROS runtime? |
|---|---|---|
| `launch` | parse + resolve + spawn | yes |
| `run` | spawn one node | yes |
| `replay` | spawn from a SystemModel | yes |
| `setcap` / `verify` | capability management | yes |
| `context` | read a SystemModel, print the launch tree | **no** |
| `check` | contract-check a launch file | **no** |
| `resolve` | delegate to layer 2, printing a deprecation line | **no** |

Three specific problems:

**`replay` is a lie.** It replays nothing. It loads a declarative artifact and
spawns from it. The name is a fossil of `record.json`, which Phase 47 removed
as a user-facing artifact; `replay` outlived the thing it was named after.

**`check` duplicates `launch`.** `handle_launch` already calls
`build_checked_model` — contracts and scheduling are validated on every launch.
`check` is that same pipeline minus the spawn, wearing a separate verb.

**`resolve` is dead weight.** `commands/resolve_compat.rs` prints
"`resolve` is deprecated and will be removed" and delegates. It has survived
because removing it silently is what caused nano-ros 0285.

## Decisions

Made during brainstorming on 2026-08-03. Each was a real fork; recording the
rejected options matters more than the chosen ones.

### D1 — Hard cut at 0.9.0, not a deprecation window

Same posture Phase 47 took with `record.json`. The rejected alternative was one
release of dual surface.

**This decision is only safe because of D4.** The argument against a hard cut is
nano-ros 0285: a subcommand vanished, and clap's `unrecognized subcommand
'resolve'` surfaced from inside a cmake configure, taking down every platform's
fixture build. That failure was about the *error*, not the removal. D4 fixes the
error.

Known casualty, accepted: simple-autoware-safety-island embeds
`play_launch resolve` in `sentinel_bringup/launch/pilot.launch.xml`. It breaks
on upgrade, with a message naming the replacement.

### D2 — `check` splits: gate here, diagnostics in layer 2

`play_launch launch --check` is a plain pass/fail gate — one new flag, no
options. The four diagnostic-only options (`--format`, `--rule`, `--explain`,
`--export-graph`) go to a **new `ros-launch-resolve check`**, where the checker
already lives (`ros-launch-manifest-check`, reached through
`ros_launch_resolve::model`) and where it runs with no ROS install.

Rejected: moving all four onto `launch` and `run`. That adds five flags to each
run verb, four of which are inert without `--check`, and the diagnostics would
still need ROS sourced because play_launch links `rclrs`.

Rejected: keeping `check` alongside `--check`. Two ways to do one thing, and it
contradicts D1.

### D3 — `replay` → `up`

`play_launch up system_model.yaml`. Behaviour unchanged.

Rejected: `start`, which conventionally implies fire-and-forget (`systemctl
start`, `npm start`) while this blocks in the foreground supervising children
until Ctrl-C — the name would fight the behaviour.

Rejected: folding into `launch` with dispatch on input type. `launch` takes
`<pkg> <file> KEY:=VALUE...`; a model takes exactly one path and accepts no
launch arguments, because they are already baked in. One verb with two
grammars, and `launch model.yaml x:=1` could only fail at runtime rather than
at parse.

**Known weakness, accepted:** `docker compose up` daemonizes by default and
ours does not. The compose analogy is imperfect in exactly the way the `start`
objection describes, just less severely.

### D4 — Removed verbs error helpfully; they do not vanish

Each removed verb stays in the `Command` enum as a `#[command(hide = true)]`
variant that accepts the old arguments, prints guidance naming the replacement,
and exits non-zero. Accepting the old arguments is what lets the error echo the
user's own invocation back in the new form.

This is the mechanism that makes D1 safe, and it follows the precedent already
in the tree: `fc99205` added exactly this for `replay --input-file` when Phase
47 removed the record.json path.

**These hidden verbs are deleted at 1.0.0.** Written down so they do not become
permanent, which is the failure mode of every "temporary" compatibility shim.

### D5 — `resolve` removed, `context` stays

`resolve` goes; it already delegates and already warns.

`context` stays in `play_launch` even though it needs no ROS. This is a
deliberate exception to "if it doesn't spawn a process, it isn't in
play_launch", not an oversight: `context` is used while debugging a launch you
have just run, where ROS is already sourced, and moving it would cost more in
ergonomics than the layering purity is worth.

### D6 — `run --check` validates the scheduling platform file only

`run` cannot contract-check. `run.rs:204` says so in a comment, and the reason
is structural: contracts are keyed by launch file
(`<pkg>/launch/<stem>.contract.yaml`), and `run` has a package and an
executable but no launch file, so no sidecar can be located and there is no
scope table to check against.

So `run --check` resolves and validates the `--target` platform file, exits on
that result alone, and **must** print:

```
no contracts checked: `run` has no launch file, so no contract sidecar
can apply. Platform file: <path> (target: posix) — OK
```

Rejected: a uniform `--check` that silently skips the contract stage on `run`.
It would exit 0 having checked nothing checkable — the vacuous-pass pattern
this codebase spent issues 0008, 0012 and 0014 removing.

That mandatory line is a tested requirement, not a nicety.

## Target surface

```
play_launch (needs ROS runtime)        ros-launch-resolve (no ROS)
  launch  [+ --check]                    resolve
  run     [+ --check]                    dump
  up          (was: replay)              check      ← new
  context                                contract
  setcap  verify                         plot
        6 verbs (was 8)                        5 verbs (was 4)
```

## Error text (normative)

```
$ play_launch replay system_model.yaml
error: `replay` was renamed to `up` in 0.9.0.
       play_launch up system_model.yaml

$ play_launch check demo_pkg a.launch.xml --format json
error: `check` was removed in 0.9.0. Two replacements:
       play_launch launch demo_pkg a.launch.xml --check   (pass/fail gate)
       ros-launch-resolve check demo_pkg a.launch.xml --format json
                                                         (diagnostics, no ROS needed)

$ play_launch resolve demo_pkg a.launch.xml -o m.yaml
error: `resolve` was removed in 0.9.0 — it delegated to layer 2 since RFC-0060.
       ros-launch-resolve resolve demo_pkg a.launch.xml -o m.yaml
```

Echoing the user's own arguments back is the point. A generic "see --help"
would be the 0285 failure with better grammar.

## Blast radius (measured 2026-08-03)

| Reference | Count | Action |
|---|---|---|
| `play_launch resolve` | 56 | → `ros-launch-resolve resolve` |
| `play_launch check` | 66 | → `launch --check` or layer-2 `check` |
| `play_launch replay` | 16 | → `up` |
| `ros-launch-resolve resolve` | 3 | unchanged |
| Test files needing edits | 7 | — |

The `resolve` and `check` migrations are the bulk of the work and are
mechanical churn in tests and docs. Integration tests already have a
`cargo_bin` helper for the layer-2 binary (`tests/tests/manifest_check.rs`), so
driving it is not new infrastructure.

## Testing

- Parse tests per verb on both binaries.
- **Every removed verb gets a test asserting the error names its replacement**,
  not merely that the command fails. "Fails" is what 0285 did.
- A test that `run --check` emits the no-contracts line (D6). Without it, a
  later refactor can quietly turn `run --check` into an always-pass.
- The existing `help_advertises_only_verbs_this_binary_has` tests on both
  binaries already cover hidden verbs staying out of `--help`; extend the
  banned list.
- Full `just test-all`, `just check`, and the layer-2 isolation gate must stay
  green — the new `ros-launch-resolve check` must not pull ROS into layer 2's
  graph.

## Out of scope

Behaviour of `launch`, `run` or `up` beyond the new flag. The SystemModel
format. `context`'s own argument surface. The pyo3 0.24 → 0.29 security port.

## Migration note for downstream

To be published in the 0.9.0 release notes and `docs/guide/`:

| 0.8.x | 0.9.0 |
|---|---|
| `play_launch replay m.yaml` | `play_launch up m.yaml` |
| `play_launch check <pkg> <f>` | `play_launch launch <pkg> <f> --check` |
| `play_launch check <pkg> <f> --format json` | `ros-launch-resolve check <pkg> <f> --format json` |
| `play_launch resolve <pkg> <f> -o m.yaml` | `ros-launch-resolve resolve <pkg> <f> -o m.yaml` |

simple-autoware-safety-island is affected — see D1.

**nano-ros never invokes these verbs**, verified 2026-08-03: it builds its own
`nros-launch-resolve` binary, and `nros sync` only *used to* shell out to
`play_launch resolve`. Its build is unaffected.

Its **user-facing text is not**. Two diagnostics tell users to run a command
that will not exist:

- `packages/cli/nros-cli-core/src/orchestration/planner.rs:74` — "re-run
  `play_launch resolve` with the desired `KEY:=VALUE` bindings instead"
- `packages/cli/nros-cli-core/src/orchestration/planner.rs:486` — "Resolve a
  SystemModel (`play_launch resolve …`)"

Both should become `ros-launch-resolve resolve`. Their repository, their fix —
flagged here so it is not discovered by a user following the advice. This is
the same drift class as the archived parser README that still documented
`record.json`: guidance outliving the thing it describes.
