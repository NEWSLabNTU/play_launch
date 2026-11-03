# Phase 98 - standard names: the vocabulary in the terms its readers know

**Status (2026-11-03): planned.** Design of record: rlm
`docs/design/standard-names.md` (design issue 57). Spans rlm, play_launch,
the safety island's contracts and nano-ros (pull requests from
`~/repos/nano-ros-pl`).

## Why

The contract, the model and the diagnostics grew words of their own for
concepts the standards already name, and one standard word names the wrong
thing:

- `hazards.<h>.severity: ASIL_D`. ISO 26262's severity is the class S0–S3;
  the ASIL is derived from severity, exposure and controllability. An
  assessor reads the key as a category error.
- rung / ladder for a degradation level and the degradation chain; floor for
  the minimal risk condition (SAE J3016, ISO 21448, UL 4600, UNECE R157);
  `window:` for a dwell time; "scope path" for an end-to-end event chain
  (AUTOSAR TIMEX), where "scope" already means a launch-file scope here.
- Imprecise: `mapper:` (priority assignment), `guards` (monitored signals),
  `budget` used for both CPU time and time limits, `ingress` (arrival
  curve), `on_demand` (aperiodic), play_launch's carried edges (triggered),
  tier (scheduling context or task).

The full table, the sources, the alternatives considered and what is kept
are in the design doc. This phase is the work and its order.

## Units

| Unit | Work | Acceptance |
|---|---|---|
| N0 | Fix the final name for each proposal (design doc §2, §3) | the design doc's proposals become a decided table; its status line says so |
| N1 | **Release R for S1–S5** in rlm: new keys, old keys as `deprecated alias` rows, types and model fields renamed, old field names rewritten in `SystemModel::from_yaml_str`, schema bumped, rule ids renamed with the old ids accepted in `acknowledged:`, `migrate` taught every rename | rlm release; one test per old key and per old rule id showing it still works and warns, naming the new one; an old saved model loads and resolves the same |
| N2 | play_launch at R: `just bump-manifest`; resolver rule ids and `--explain` text (`budget_book.rs`, `verbs/check.rs`); every fixture and contract migrated; guides, CLAUDE.md, help text rewritten | `just check` and `just test-all` green; zero `deprecated-spelling` warnings across all fixtures (no shipped file leans on an alias) |
| N3 | Safety island contracts and nano-ros at R: contracts migrated; nano-ros pull request renaming the mirrored types (`rungs[]` and its route types) and bumping the rlm tag | island `check` verdicts and totals unchanged; nano-ros PR merged |
| N4 | **Release R+1 (refuse)**, only after R is on PyPI and N3 is merged: each alias row becomes a `removed` row with its own message naming the new key; an old rule id in `acknowledged:` is an error naming the new id; alias handling deleted from the parser | one negative test per old key and per old rule id (a removal that forgot one fails); the model rewrite still loads an R-1 model |
| N5 | **Release R+2 (purge)**: `check-terms` gate (fails on a retired word in user-facing text, allowlisted by file) in `just check` and rlm's gate; old-id message dropped | the gate lists every S1–S5 old word; it was checked by injecting one (a negative control) |
| N6 | M1–M6 (mapper, guards, budget-as-limit, ingress, on_demand, carried edges), each through its own R / R+1 / R+2 when its area is next touched; M6 is play_launch only (`chains.json` with a version field, `/api/chains`, the Chains page) | per name, as N1–N5 |
| N7 | Decide M7 (tier): rename, or only gloss it in the docs' prose ("a tier, i.e. a scheduling context") | a recorded decision; ~14.5k nano-ros mentions weighed against the gain |

## Retiring the old names

An alias left in place is a second spelling for good, so every rename ends
on a schedule (design doc §6):

| Release | Old key in a contract | Old rule id in `acknowledged:` | Old field in a saved model |
|---|---|---|---|
| R | accepted, warned, `migrate` rewrites | matches, warned | rewritten on read |
| R+1 | parse error naming the new key | error naming the new id | rewritten on read |
| R+2 | still the R+1 error | old-id message dropped; word in `check-terms` | rewritten on read |

Kept for good, because the past needs them: the model rewrite (a saved model
must load), the `removed` rows (an old contract must be told where its key
went, not "unknown key"), and `migrate`. Everything else carrying an old name
is gone by R+2.

R+1 is a separate release on purpose: phase 68's deprecation window was
never tagged, so no user saw it and 0.9 to the next release was a hard cut.
Here R+1 waits until R has been published and nano-ros has followed.

## Order

N0, then N1 + N2 + N3 as release R (one rlm release, one play_launch
release). N4 one release later, N5 the release after. N6 as areas are
touched; N7 on its own.

## Notes for whoever does it

- `acknowledged:` entries name rule ids, so a rule renamed without the alias
  turns every acknowledged finding into `ack-stale` plus a fresh warning on
  upgrade. That is why rule ids get the same R / R+1 / R+2 treatment as keys.
- `chain-sampling-feasibility` is this rule's id from before phase 68, when
  `chains:` was a written route. Reusing it for the event-chain rule is
  right, but the `chains:` removal message must keep saying it removed a
  written route and point at `paths:`.
- History entries (CLAUDE.md "Key Recent Changes", closed phase docs, design
  issues) keep the words of their time; they are allowlisted in
  `check-terms`, and a status line names the rename where it matters.
- `ros-launch-resolve` must not appear in any guide text this phase
  rewrites.
