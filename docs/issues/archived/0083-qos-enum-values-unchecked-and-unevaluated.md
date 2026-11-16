---
id: 83
title: "A QoS enum value the contract does not know (a typo, or a launch reference) passes qos-match silently"
status: resolved
type: defect
severity: medium
---

# 0083 - QoS enum values are neither validated nor evaluated

**Repo:** `ros-launch-manifest` (types, check) and `play_launch`
(`ros-launch-resolve`); code read at rlm `87c10cd` (v0.3.25) and play_launch
`ae09fd9e`. Not run.
**Affects:** rlm `types/src/parse.rs` (the `qos` section, `yaml_string` for
`reliability`, `durability`, `history`, `liveliness`);
`check/src/rules/qos_match.rs`;
`src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs`
(`durability_compatible`, "Unknown values pass").
**Found:** 2026-11-16, planning the autoware-safety-island port to declare
its input QoS in `launch/system.contract.yaml` (its phase 15, step P3).

## What happens

The four QoS enum fields are read as any string. Nothing checks the value
against `volatile | transient_local` (or the other enums), and the match
treats an unknown pair as compatible. So:

- `durability: transient-local` (a typo) parses, and every pub/sub pair on
  the topic passes `qos-match`;
- `durability: $(var operation_mode_durability)` parses too. Numeric
  references (`$(param ...)` with a unit, `types/src/values.rs`) are
  evaluated per scope; a reference in a string field is not, so it reaches
  the match as the literal `$(var ...)` and passes.

The consumer case: the island's controller picks VOLATILE or
TRANSIENT_LOCAL for `/system/operation_mode/state` from a parameter
(`operation_mode_durability`) to match whichever bridge is deployed.
Declaring that in the contract today gives no check at all, which is the
silent non-match the contract was meant to catch.

## Expected

- An unknown enum value is a parse error naming the allowed values.
- A `$(var ...)` / `$(param ...)` in an enum field is either evaluated by the
  consumer that holds the launch tree (as numeric references are) and then
  checked, or refused at parse time with a pointer to per-launch contracts.
  Which one is a grammar decision for rlm.

## Resolution (2026-11-16)

rlm v0.3.26 took the grammar decision: a QoS enum takes a literal from its
ROS 2 set or exactly `$(var <arg>)` naming an arg of the contract's
`args:`, evaluated per scope; any other reference is refused at parse.

- **One definition of the sets and the match** (rlm `types/src/qos.rs`):
  `reliability` (`reliable`, `best_effort`, `system_default`,
  `best_available`), `durability` (`volatile`, `transient_local`,
  `system_default`, `best_available`), `history` (`keep_last`, `keep_all`,
  `system_default`), `liveliness` (`automatic`, `manual_by_topic`,
  `system_default`, `best_available`). `qos::compatible` answers
  compatible, incompatible, RMW-defined (`system_default` on either side:
  a warning that the pair is not checked) or an error for a value outside
  the set. `best_available` matches.
- **Parse (rlm):** a value outside its set is a parse error naming the key
  path, the value and the set. A `$(var <arg>)` must name a declared arg;
  a `type: bool` arg, or a `choices:` entry outside the set, is refused, so
  an arg with `choices:` checks every launch configuration at parse time.
  `$(param ...)`, `$(var x @n)` and text around a reference are refused.
- **Evaluate (rlm):** `substitute_manifest` substitutes the four fields on
  topic and endpoint blocks (it skipped them) and refuses a free-string arg
  that evaluates outside the set (`SubstError::InvalidQosValue`, with the
  key path).
- **Match (both layers):** rlm `qos-match` and play_launch's cross-scope
  `check_cross_scope_qos_match` both call `qos::compatible`; play_launch's
  own `reliability_compatible`/`durability_compatible` and the inline
  liveliness test are gone. `qos-compat` names the allowed set and warns on
  a `$(var ...)` read without a launch scope.
- **Report (play_launch):** an args failure (a required arg the launch
  file does not pass, a value outside `choices:`, a QoS value outside the
  set) was a `warn!` and a silently dropped contract. It is now an
  `error[manifest-args]` naming the file, the line of the key when there is
  one, and the scope, and `check` counts it as a refused contract (exit
  `EXIT_REFUSED`), as it does `manifest-parse`.

Tests. rlm: `qos::tests` (sets, references, match),
`parse::tests::a_qos_enum_typo_is_refused`,
`..::a_qos_enum_reference_must_name_a_checked_arg`,
`subst::tests::a_qos_var_is_evaluated_per_arg_value`,
`..::a_free_string_qos_var_is_tested_once_evaluated`,
`qos_match::tests::an_unknown_durability_is_not_compatible`,
`..::best_available_matches_and_system_default_is_reported`, and three in
`qos_compat`. play_launch (`manifest_loader`):
`test_qos_var_is_evaluated_per_scope_arg`,
`test_qos_var_outside_the_set_refuses_the_contract`,
`test_qos_enum_typo_refuses_the_contract`,
`test_cross_scope_qos_match_unknown_durability_is_not_compatible`.

Downstream: every QoS value in the autoware-safety-island contracts and
the ASI controller contract is a literal from the sets, so none refuses.

