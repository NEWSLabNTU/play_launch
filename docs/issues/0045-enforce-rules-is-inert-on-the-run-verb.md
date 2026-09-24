---
id: 45
title: "`--enforce-rules` is accepted by `run` and does nothing — `run` never builds a RuleEngine"
status: resolved
resolved_in: fix(#0045) on main, 2026-09-24
type: correctness
severity: medium
---

# 0045 - a gate that is silent on one verb and load-bearing on the others

**Repo:** `play_launch` at `af0aeab1`
**Affects:** `src/play_launch/src/commands/run.rs`;
`src/play_launch/src/commands/up.rs` (`check_enforcement_has_event_source`,
the only call site, in `play()`)

## Symptom

`play_launch run --enforce-rules strict <pkg> <exe>` starts the node and
exits normally whatever the contracts say. There is no `Interception:` line,
no precondition warning, no `runtime_violations.jsonl`, and no
`interception/` directory in the bundle — measured on 2026-09-23 with
`run --enforce-rules strict --interception off demo_nodes_cpp talker`, which
on `launch` or `up` is refused before spawn.

## Cause

`run.rs` calls `load_runtime_config` and never uses the interception half of
the result: it sets up no child interception, builds no `RuleEngine`, and
never calls `check_enforcement_has_event_source`, which has exactly one call
site — in `up::play`. The clap flag is on `CommonOptions`, so every verb
ACCEPTS it; only two act on it.

Phase 79 made a non-`Off` mode with no event source refuse under `strict`
(issue #0031), which sharpened this: on `launch`/`up` a strict run can no
longer pass with nothing measured, while on `run` it still can — and now does
so without even the warning, because the check that would warn is not
reached.

## Impact

`run` is the single-node verb, which is what a developer reaches for when
iterating on ONE node against its contract — the case where a runtime rule is
most useful and cheapest to evaluate. A CI job written around `run` inherits
exactly the shape phase 79 removed from the other verbs: green because
nothing was observed.

## Fix direction

Either make it work or make it say it does not:

- **Wire it**: `run` already spawns a child through the same execution layer,
  so the missing pieces are the interception setup and the engine, both of
  which `up::play` builds from `runtime_config` and `contract_view`. The
  precondition check is one call. The awkward part is that `run` takes a
  package and executable rather than a launch file, so the contract channel
  has no scope to resolve against — decide what a contract means for a single
  node named on the command line before wiring anything.
- **Refuse it**: if `--enforce-rules` is meaningless on `run`, reject a
  non-`Off` value with a message saying so, the way a strict run with no
  event source is now refused. A flag that is accepted and ignored is the
  defect; either answer removes it.

`docs/guide/runtime-enforcement.md` states the inertness under Limits, so a
reader who finds the guide is not misled today. The flag itself still is.

## Fix

**Refused**, because the awkward part turned out to be the whole question and
it has a structural answer. A contract is LOCATED by launch file —
`<overlay>/<pkg>/launch/<stem>.contract.yaml`, or the sidecar beside the
launch file (`manifest_loader::ContractChannel`, and there are only those
two) — and `run` names a package and an executable, so neither channel has a
key to look one up with. `run` resolves no `ContractView` today and could not
be given one without inventing a third channel that `check`, `resolve` and
`contract eject` know nothing about: a contract enforceable at runtime and
unverifiable statically. `run --check` already prints this same fact about
its own contract step.

So `commands/run.rs` refuses an EXPLICIT non-`off` mode before anything is
created — ahead of `create_log_dir`, so a refusal leaves no half-written
bundle and no moved `latest` (issue #0023), the placement phase 79 chose for
the same reason. The message names the mode as the CLI spells it, the reason,
and the two ways out (a one-node launch file under `launch`/`up`, or
`--enforce-rules off`).

Explicit is the load-bearing word: `enforce_rules` defaults to `Warn`, so
refusing the VALUE would reject every plain `play_launch run`. The question
"did the user type it?" is put to clap (`ValueSource` from a re-parse of the
same argv), not to a scan of `std::env::args()` — after a `--`, `run pkg exe
-- --enforce-rules strict` passes those tokens to the NODE, and a textual
scan would refuse a command that never addressed play_launch. (Without the
`--`, clap takes the flag for itself even when it follows the executable;
measured, and pinned by a test.)

Pinned by `commands::run::enforcement_refusal_tests` (4, the predicate and
the message) and `tests/tests/run_enforce.rs` (6): the issue's own
invocation exits non-zero having created no `play_log`, all three enforcing
modes and the `=` spelling are refused, and a plain run, an explicit `off`
and a flag passed through to the node are not.

Two corrections to this report. `check_enforcement_has_event_source` does not
exist under that name — phase 79's precondition is an inline block in
`up::play` (`up.rs:297-375`) with no separate function, which is why it has
one call site. And `run` does not merely skip the interception half of
`load_runtime_config`: it never reads `runtime_config.interception` at all,
so `--interception on` is inert there too. That one is left open on purpose —
interception needs no contract, so wiring it into `run` would ADD a
capability rather than remove a false claim, and is a separate change.

## Provenance

Found 2026-09-22 while implementing phase 79 W1, confirmed 2026-09-23 by
running it and by reading `run.rs` for the call sites.
