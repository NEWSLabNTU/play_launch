---
id: 54
title: "four `manifest_check` tests fail against a current binary — three phase-70 W2 model facts and one missing rate-contradiction warning"
status: open
type: correctness
severity: medium
---

# 0054 - the first honest run of this suite since 1c27ba5c

**Repo:** `play_launch` at `d20936d2`
**Affects:** `tests/tests/manifest_check.rs:941` (`w2_derived_overrun_reaches_the_model`),
`:695` (`check_legacy_toml_with_contradicting_contract_facts_warns_but_succeeds`),
plus `w2_reservation_is_refused_for_a_node_that_claims_concurrency` and
`w2_an_unenforceable_miss_action_is_reported_not_downgraded`

## Why this is only being filed now

The suite has not compiled on `main` since `1c27ba5c` (phase 82 W1-W3) left a
test without its closing brace, and before that its spawns inherited the
ambient environment, so most of it failed on the library path wherever the
invoking shell had not been sourced (issue #0051). Both are fixed as of
`d20936d2`, and the binary was rebuilt so `install/` matches the source. This
is what the suite says once it can actually run: **30 tests, 26 passed, 4
failed.**

Three of the four were previously dismissed as "pre-existing" in this
session's reports, which was true but incomplete — they are pre-existing AND
real. They were verified against two different manifest tags (v0.1.41 and
v0.1.42) with identical output, so they are not a pin artifact, and they
survive a full `just build`, so they are not issue #0020's stale-binary shape
either.

## The failures

**1-3. The three `w2_*` tests — a `miss:` declaration does not reach the model.**

```
a `miss:` declaration must reach the model as deadline_policy:
meta:
  version: 1
  ...
```

`w2_derived_overrun_reaches_the_model` asserts a declared `miss:` arrives as
`deadline_policy` in the emitted model; it does not.
`w2_an_unenforceable_miss_action_is_reported_not_downgraded` and
`w2_reservation_is_refused_for_a_node_that_claims_concurrency` are the same
family — model facts phase 70 W2 said it carried.

This matters beyond the tests: phase 68 W5's whole point was that a fact which
never reaches `model::PathContract` cannot be seen by nano-ros at all, because
it builds its `MapperPath` from the model rather than from the manifest. If
`miss` is not arriving, that seam is open again for exactly the reason W5
closed it.

**4. A rate contradiction between a legacy `system.toml` and a contract
produces no warning.**

```
expected a rate contradiction warning citing both nodes, got:
  ... Loaded 1 manifest(s) [1 overlay, 0 provider] (0 scopes ..., 0 errors, 2 warnings)
  Scheduling platform file [explicit]: /tmp/.../system.toml
  Scheduling (posix, mapper=manual): 2 tier(s)
```

Two warnings are emitted and neither is the contradiction the test asks for.
The legacy TOML bridge is deprecated but supported until nano-ros migrates
(phase 41.6), so a contradiction between it and a contract is exactly the case
the bridge needs to report.

## Fix direction

Take the three `w2_*` together — they are one question (does a `miss:`
declaration survive lowering?) and the answer is in `model_builder.rs`'s path
lowering, which is where `max_jitter` and `miss` were added in manifest
v0.1.19. Check whether the field is lowered at all, and whether the
`effective_trigger()` correction from phase 58 W2 moved the branch that reads
it.

The fourth is separate: find which two warnings ARE emitted and whether the
contradiction check runs at all on the `manual` mapper path.

Neither should be closed by adjusting the test until the product behaviour is
understood — three of these assert facts that shipped phases claim to carry.

## Provenance

Observed 2026-09-25 after fixing #0051 and rebuilding; the `w2_*` three were
also seen at v0.1.41 and v0.1.42 by an earlier agent, with identical output.
