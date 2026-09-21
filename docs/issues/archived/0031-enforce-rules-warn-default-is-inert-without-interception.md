---
id: 31
title: "`--enforce-rules warn` is the default and does nothing unless `--config` turns interception on, silently"
status: resolved
resolved_in: fix(#0031) on main, 2026-09-21
type: correctness
severity: high
---

# 0031 - the default runtime enforcement mode enforced nothing, and said so nowhere

**Repo:** `play_launch` (0.11.0, `5eaa3191`)
**Affected:** `src/play_launch/src/commands/up.rs`,
`src/play_launch/src/cli/options.rs`, `src/play_launch/src/cli/config.rs`

## Symptom

`play_launch launch --contracts contracts pure_nodes.launch.xml` ran with
the documented default `--enforce-rules warn`, reported no runtime
violation, wrote no `runtime_violations.jsonl` and exited 0. The same
command with a `--config` file containing `interception: {enabled: true}`
reported the `rate-hierarchy-runtime` violation the contract was written
for (2026-09-18, 0.10.0 binary at `155ed78b`).

## Cause

Three facts, each fine alone: `enforce_rules` defaulted to `Warn`;
`InterceptionSettings::default()` had `enabled: false` and the key could
only be set from the YAML; `up.rs` built the `RuleEngine` only when
interception consumers existed. Nothing checked "enforce mode is not Off
but interception is off" and told the user. A `--enforce-rules strict` CI
gate passed trivially on a run that observed nothing.

## Fix

The issue's first-preference direction, as phase 79 W1 specifies it: a
non-`Off` `--enforce-rules` implies interception.

- `interception.enabled` is tristate (`Option<bool>`, unset by default).
  `InterceptionSettings::decide(enforce)` returns `Configured`,
  `ImpliedByEnforcement(mode)`, `RefusedByConfig(mode)` or `Off`; `up` logs
  which input decided ("Interception: enabled, implied by --enforce-rules
  Warn ..."). The `interception_on.yaml` workaround in the integration
  harness is gone: the tests pass `--enforce-rules` and get an engine.
- An explicit `enabled: false` (or `--interception off`) is honoured. Under
  `warn`/`record-only` the run warns once that it has no event source and
  no runtime rule can fire; under `strict` it refuses to start rather than
  pass green with no measurement.
- `--interception on|off` switches interception without authoring a YAML
  file; `--enforce-rules`'s help text names the precondition. The missing
  `.so` warning now says that no runtime rule can fire either.

Pinned by `cli::config::tests::interception_follows_enforcement_unless_configured`
(the decision table) and `interception_enabled_is_tristate_in_yaml`, and by
`tests/tests/runtime_enforcement.rs::default_invocation_intercepts_once_a_contract_resolves`
(no `--config`, no `--enforce-rules`, a 1 Hz talker against
`min_rate_hz: 1000`: the violation is written and the log says what
switched interception on) and `strict_refuses_to_start_without_an_event_source`.

## Provenance

Brief C (`brief-C-playlaunch-usage.md`, sections 2.5 and 3.1), runs of
2026-09-18 with the 0.10.0 binary at `155ed78b`; re-verified against the
0.11.0 source at `5eaa3191` on 2026-09-21.
