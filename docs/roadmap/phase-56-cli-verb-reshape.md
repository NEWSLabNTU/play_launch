# Phase 56: CLI verb reshape for 0.9.0 (complete)

**Design of record:** [docs/superpowers/specs/2026-08-03-cli-verb-reshape-design.md](../superpowers/specs/2026-08-03-cli-verb-reshape-design.md).
**Migration guide:** [docs/guide/cli-migration-0.9.md](../guide/cli-migration-0.9.md).
**Breaking:** yes — version 0.8.2 → 0.9.0.

## Summary

`play_launch` had eight verbs; three did no process spawning at all, and one
of those (`resolve`) had been a deprecated delegate to `ros-launch-resolve`
since RFC-0060. Phase 56 cuts the surface down to the verbs that actually
spawn processes, moving everything else to the ROS-free `ros-launch-resolve`
binary, and does it as a hard cut rather than a deprecation window — made
safe by making every removed verb error with its replacement spelled out
instead of vanishing.

```
play_launch (needs ROS runtime)        ros-launch-resolve (no ROS)
  launch  [+ --check]                    resolve
  run     [+ --check]                    dump
  up          (was: replay)              check      ← new
  context                                contract
  setcap  verify                         plot
        6 verbs (was 8)                        5 verbs (was 4)
```

## The six decisions

**D1 — Hard cut at 0.9.0, not a deprecation window.** Same posture Phase 47
took with `record.json`. Safe only because of D4: nano-ros issue 0285 showed
that the failure mode to avoid is the *error*, not the removal — a vanished
subcommand surfacing `unrecognized subcommand 'resolve'` from inside a cmake
configure took down every platform's fixture build. **Accepted casualty:**
simple-autoware-safety-island embeds `play_launch resolve` in
`sentinel_bringup/launch/pilot.launch.xml`; it breaks on upgrade, with a
message naming the replacement.

**D2 — `check` splits: a gate here, diagnostics in layer 2.**
`play_launch launch --check` is a plain pass/fail flag — `launch` already
calls `build_checked_model` on every run, so `check` was that same pipeline
minus the spawn, wearing a separate verb. The four diagnostic-only options
(`--format`, `--rule`, `--explain`, `--export-graph`) moved to a new
`ros-launch-resolve check`, which runs with no ROS install. Rejected: putting
all four flags on `launch`/`run` (inert without `--check`, and still needs
ROS sourced because play_launch links `rclrs`); keeping `check` alongside
`--check` (two ways to do one thing).

**D3 — `replay` → `up`.** `replay` replayed nothing — it loaded a resolved
SystemModel and spawned from it; the name was a fossil of `record.json`,
retired in Phase 47. Rejected: `start` (implies fire-and-forget, but this
blocks in the foreground supervising children); folding into `launch` with
dispatch on input type (different argument grammars — `launch` takes
`<pkg> <file> KEY:=VALUE...`, a model takes exactly one path with no launch
arguments).

**D4 — Removed verbs error helpfully; they do not vanish.** Each removed
verb stays in the `Command` enum as a hidden (`#[command(hide = true)]`)
variant that still parses its old arguments, so the error can echo the
user's own invocation back in the new form. This is the mechanism that makes
D1 safe. Implementation: `src/play_launch/src/commands/migrated.rs`.
**These hidden verbs are deleted at 1.0.0** — written down so they do not
become permanent.

**D5 — `resolve` removed, `context` stays.** `resolve` already delegated and
already warned; it goes. `context` stays in `play_launch` despite needing no
ROS — a deliberate exception, since it's used while debugging a launch you
just ran, where ROS is already sourced, and moving it would cost more in
ergonomics than the layering purity is worth.

**D6 — `run --check` validates the scheduling platform file only.** `run`
has a package and executable but no launch file, so no contract sidecar can
be located and there's no scope table to check contracts against. `run
--check` resolves and validates only the `--target` platform file, and
**must** print a line saying so explicitly (`no contracts checked: ...`) —
a uniform `--check` that silently skipped the contract stage would exit 0
having checked nothing checkable, the vacuous-pass pattern issues 0008,
0012 and 0014 already removed elsewhere in this codebase.

## Verification

`just test-all`, `just test-unit`, `just check`, and both binaries' `--help`
output were used as the final gate — see
`.superpowers/sdd/2026-08-03-cli-verb-reshape/task-7-report.md` for the
recorded output. Every removed verb has a test asserting its error names the
replacement (not merely that the command fails, which was the shape of the
nano-ros 0285 failure). `run --check`'s no-contracts line is a tested
requirement (D6).

## Known follow-ups (not this phase's responsibility)

- **simple-autoware-safety-island** breaks on upgrade (D1's accepted
  casualty) — its own repo's fix.
- **nano-ros** never invokes these verbs programmatically (it builds its own
  `nros-launch-resolve` binary), but two of its user-facing diagnostics
  (`packages/cli/nros-cli-core/src/orchestration/planner.rs:74,486`) tell
  users to run `play_launch resolve`, which no longer exists. Flagged in the
  design doc so it isn't discovered by a user following stale advice — fix
  belongs to nano-ros's repository.
