# Phase 98 - standard names: the vocabulary in the terms its readers know

**Status (2026-11-04): in progress -- N0-N4 done (R = rlm v0.3.3 /
play_launch 0.24.0, R+1 = rlm v0.3.6); N5's `check-terms` gate is in both
repos, the old-id message is dropped at the next release; N6: M6 done,
M1-M5 at R (rlm v0.3.7, v0.3.8), released in play_launch 0.25.0; N7
open.** Design
of record: rlm
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

## Progress

- **N0** (2026-11-03): the table in the design doc's section 2.6. One
  change from the first proposal: the ordered list is the *fallback
  sequence*, not a "degradation chain", because S5 gives "chain" to the
  event chain. In the `--explain` hazard table the level column is
  DEGRADATION (LEVEL already heads the budget's evidence level).
- **N1** (rlm `c11aadb`, `a02feb3`, `ac2786c`, `527dd14`; v0.3.3): every
  S1-S5 key, type, model field and rule id renamed, the old spellings read
  for one release; model schema 3 reads schema 1 and 2. New in rlm:
  `types::rule_names` (renamed rule ids and keys, `current`,
  `current_path`), so an `acknowledged:` entry written with an old id or
  through an old key still matches, and the `rule-id-renamed`,
  `severity-integrity` and `window-min-dwell` rewrites. The bundled
  profiles are 1.1.0 (their bytes changed with `integrity_scale:`).
- **N2** (play_launch `c2928054`, `879c8e78`, `eb1e0dd2`, `55ba4aef`):
  resolver messages, `--explain`, the runtime's mode message, every
  fixture, the guides. `contract_spelling_deprecated_n1` and `_s4` prove an
  old spelling checks to the same verdict and table and migrates; the
  ranked plans are unchanged (the two new fixtures were added).
  Code-internal identifiers that never reach a user (`LevelBudget.rung`,
  local variables) were left; the R+2 gate reads user-facing text only.
- **N3**: the safety island's CI checks with the play_launch it pins from
  PyPI (0.20.1), so its contracts and its `check-contracts.sh` rule ids
  (`ladder-rung-budget`, `ladder-unterminated`) move when it bumps that pin
  to the release carrying R; until then the old keys keep working. Run
  against the R build its verdicts are unchanged apart from the renamed ids
  (and `l3_takeover`'s `call-unbounded`, which is phase 95's, not this
  phase's). nano-ros: the pull request bumping the rlm tag and the mirrored
  `rungs`/`RungRole` reads is still to open.

- **N4** (rlm v0.3.6, 2026-11-04): `severity_levels:`, a hazard's
  `severity:` and a mode's `window:` are `removed` rows whose messages name
  the new key and `migrate`; a profile's `severity_scale:` has its own
  refusal; the `spelled_*` fields and the resolver's `deprecated-spelling`
  blocks are gone. `rule_names` marks every rename `refused`, and
  `accepted()`/`accepted_path()` (what `acknowledged:` matching reads) map
  none of them, so an old id or path is `error[ack-renamed]` naming the
  new one instead of a silent `ack-stale`. `contract_spelling_deprecated_n1`
  and `_s4` now prove the refusal (exit 3, the message) and that
  `migrate --write` still reaches the reference verdict.
- **N5** (gate): `scripts/check_terms.py` in both repos -- play_launch's
  `just check-terms` (a `just check` step), rlm's
  `types/tests/retired_terms.rs`. Markdown lines, Rust string literals
  (placeholders removed, `#[cfg(test)]` skipped), clap help comments and
  web UI literals; `--self-test` injects one word per kind (the negative
  control) and runs first. Its first run found the `--explain` help still
  naming RUNG/WINDOWS columns, "node/scope paths" in `--export-graph`'s
  help, "Contract scope path" on the Chains page, and stale v0.1.43 links
  in the RT guide. The floor-as-MRC sense is not mechanically separable
  from a rate's or `min_latency`'s floor, so the gate does not list it.
  Still to do at the next release: drop `ack-renamed` (an old id then reads
  as `ack-stale`), per the schedule.
- **N6, M6** (R, 2026-11-04): play_launch's carried edges are triggered
  edges -- `kind: "triggered"` on a hop, `triggered_hops` on a path,
  `TriggeredEdge`, the Chains page. `chains.json` and `/api/chains/paths`
  carry `version: 2` (format 1 had none); `carried_hops` is emitted beside
  `triggered_hops` for one release and then goes.
- **N6, M1/M2/M4/M5** (R, rlm v0.3.7, 2026-11-04): `priority_assignment:`
  (platform file, was `mapper:`), `monitors` (was a hazard's `guards`),
  `arrival_curve` (was a subscriber's `ingress`), `aperiodic` (was a
  publisher's `on_demand` and `trigger: spontaneous`). The old spellings
  read with one `deprecated-spelling` warning each (rlm
  `Manifest::deprecated_spellings`, `PlatformFile::spelled_mapper`);
  `migrate` rewrites the contract keys; every fixture, example and guide
  uses the new ones, and `contract_spelling_deprecated_n6` proves the old
  ones warn and migrate to the same verdict. The model's field names and
  the Rust types did not move: nano-ros reads them, so they go with its
  pull request (the model's schema 4).
- **N6, M3** (R, rlm v0.3.8, 2026-11-04): the time-limit sense of
  "budget" -- `fault-reaction-time` (was `fault-reaction-budget`),
  `degradation-level-time` (`degradation-level-budget`),
  `latency-bound-overflow` (`budget-overflow`). An `acknowledged:` entry
  naming an old id still matches this release, warned as a deprecated
  spelling (`rule_names::accepted`); `migrate` rewrites it. `budget`
  keeps its reservation sense.

- **Release** (play_launch 0.25.0, 2026-11-04): the release carrying N4
  (rlm v0.3.6), N5's gate, M6 and M1-M5 at R (rlm v0.3.7, v0.3.8). The
  island and nano-ros follow it (nano-ros phase 534); the M1-M5 R+1
  refusals wait until they have.

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
