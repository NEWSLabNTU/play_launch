---
id: 83
title: "A QoS enum value the contract does not know (a typo, or a launch reference) passes qos-match silently"
status: open
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
