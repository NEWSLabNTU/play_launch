# Phase 56: CLI verb reshape for 0.9.0 (complete)

**Design of record:** [docs/superpowers/specs/2026-08-03-cli-verb-reshape-design.md](../superpowers/specs/2026-08-03-cli-verb-reshape-design.md).
**Migration guide:** [docs/guide/cli-migration-0.9.md](../guide/cli-migration-0.9.md).
**Breaking:** yes — version 0.8.2 → 0.9.0.

## Summary

Phase 56 shipped in two parts, and the second reverses half of the first.

**As designed**, the phase cut `play_launch` down to the verbs that spawn
processes and moved `resolve`, `dump`, `check`, `contract` and `plot` onto the
ROS-free `ros-launch-resolve` binary, as a hard cut made safe by D4 (removed
verbs error with their replacement spelled out).

**As shipped (2026-08-03 amendment)**, D2 and D5 are REVERSED. All five verbs
are back on `play_launch`. The split was wrong on a fact the design never
checked: **`ros-launch-resolve` is a developer/integration binary a user never
installs.** It exists so nano-ros and similar consumers can resolve launch
trees without linking a ROS runtime; `pip install play_launch` is the whole
product as far as a user is concerned. Two consequences the amendment fixed:

1. With `resolve` AND `dump` both gone, **no `play_launch` verb could write a
   `system_model.yaml` — and `up` requires one.** The two-step workflow the
   README documents was unreachable from an installed wheel.
2. README, the migration guide and three error messages told users to run a
   binary they did not have. The interim fix — shipping the developer binary
   as a second console script (`932c63b`) — was itself reverted, because
   making bad advice true is not the fix.

The layering survives, relocated: all five verbs live in
`ros_launch_resolve::verbs::*` (one implementation), and both CLIs are thin
argument-mapping wrappers over them. `ros-launch-resolve` keeps its no-ROS
guarantee and its `check-layer2-isolation.sh` gate.

```
play_launch (needs ROS runtime; THE product, ships in the wheel)
  launch [+ --check]   run [+ --check]   up (was: replay)
  resolve   dump   check   plot   contract   context   setcap   verify
        11 verbs (was 8)

ros-launch-resolve (no ROS; developer/integration binary, NOT in the wheel)
  resolve   dump   check   contract   plot
        the same ros_launch_resolve::verbs::* implementations
```

D1, D3, D4 and D6 stand as designed. `replay` is the one verb that stayed
removed.

## The six decisions

**D1 — Hard cut at 0.9.0, not a deprecation window.** Same posture Phase 47
took with `record.json`. Safe only because of D4: nano-ros issue 0285 showed
that the failure mode to avoid is the *error*, not the removal — a vanished
subcommand surfacing `unrecognized subcommand 'resolve'` from inside a cmake
configure took down every platform's fixture build. **Accepted casualty (now moot):**
simple-autoware-safety-island embeds `play_launch resolve` in
`sentinel_bringup/launch/pilot.launch.xml`. The D5 reversal reinstates that
verb, so nothing breaks there after all.

**D2 — `check` splits: a gate here, diagnostics in layer 2.** **REVERSED** — see the Summary. The `--check` gate stands; the removal of `play_launch check` does not. `play_launch check` is a live verb again with all four diagnostic options, sharing one implementation with `ros-launch-resolve check`. Original reasoning:
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

**D5 — `resolve` removed, `context` stays.** **REVERSED in its first half** — `resolve` (and `dump`, `plot`, `contract`) are back on `play_launch`; see the Summary. The `context` half stands unchanged. Original reasoning: `resolve` already delegated and already warned; it goes. `context` stays in `play_launch` despite needing no
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
`.superpowers/sdd/2026-08-03-cli-verb-reshape/task-7-report.md` (original) and
`.superpowers/sdd/2026-08-03-cli-verb-reshape-amendment/task-10-report.md`
(amendment) for the recorded output. The amendment adds the layer-2 isolation
gate, a `--help` guard that walks *every* subcommand's long help asserting the
developer binary is never named, and a check that the built wheel contains no
`ros-launch-resolve`. Every removed verb has a test asserting its error names the
replacement (not merely that the command fails, which was the shape of the
nano-ros 0285 failure). `run --check`'s no-contracts line is a tested
requirement (D6).

## Known follow-ups (not this phase's responsibility)

- **nano-ros**'s user-facing diagnostics
  (`packages/cli/nros-cli-core/src/orchestration/planner.rs:74,486`) tell users
  to run `play_launch resolve`. The amendment's reinstatement of that verb
  makes this advice correct again — no fix needed.
