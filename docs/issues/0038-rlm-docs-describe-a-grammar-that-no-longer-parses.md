---
id: 38
title: "rlm: the hand-written docs teach spellings that are parse errors at HEAD and rules that were deleted"
status: open
type: tech-debt
severity: low
---

# 0038 - `docs/launch-manifest.md`, `slides.md`, `scheduling.md`, `contract-verification.md` and the README are behind phases 68-72

**Repo:** `ros-launch-manifest` (rlm), `origin/main` at `ea5cbea`; filed here
because rlm has no tracker. `docs/format-reference.md` is generated and is
NOT affected; everything below is hand-written prose the source contradicts.

## Symptom

Every line was re-read at `ea5cbea` on 2026-09-21. "Parse error" means the
closed grammar in `types/src/field_table.rs` rejects the key: `_ms` names
are `removed(...)` entries with a hint (`field_table.rs:398-401`,
`:667-670`), and `max_drop_rate` / topic-level `max_consecutive` are absent
altogether (the only drop vocabulary is `drop: { max_count: "N / W",
max_consecutive: N, ... }`, `:785-796`).

`docs/launch-manifest.md`

- `:579-580`, `:609-612` (and `:949-950`, `:1244-1245`, `:1258-1268`,
  `:1382-1383`, `:1400-1409`): `max_drop_rate: 0.08` and top-level
  `max_consecutive: 3` on topics and scope paths. Parse errors.
- `:1187`: `criticality` "Advisory, not schema-enforced - unrecognized
  values are ignored, never a parse error". It is a closed set
  (`types/src/parse.rs:564-579`) and, since phase 72, a CONSEQUENCE of the
  hazards a node guards or feeds (`field_table.rs:362-366`).
- `:662-669`: the QoS table has no `deadline` and no `lease_duration` rows,
  says `lifespan` is "integer", and its "ROS 2 default (when omitted)"
  column implies defaults the checker does not assume
  (`check/src/rules/qos_match.rs:16-19`).
- `:719-721`: "`history`, `depth`, `lifespan`, and `liveliness` are not
  checked by `qos-match` in v1. `liveliness`, `deadline`, and `lifespan`
  compatibility are deferred". `liveliness` and `lease_duration` have been
  checked pairwise since phase 70 (`qos_match.rs:247-282`).
- `:1390`, `:1404`: path `input` "Empty = periodic (timer-driven)". An
  empty input is `Unclassified` (`types/src/types.rs:714-728`); the doc's
  own `:1528-1534` says so.
- `:1678-1713` rule table: omits `jitter-range`; `rate-hierarchy` omits the
  `max_rate_hz` upper bound; `dangling-entity` omits the service
  `external:` escape.
- Retired spellings in prose: `max_age_ms` (`:481`), `max_transport_ms`
  (`:504-506`).

`docs/slides.md`

- `:82-83`: `max_drop_rate: 0.01`, `max_consecutive: 3` at topic level.
  Parse errors.
- `:115`, `:126`: `max_latency_ms`, `max_age_ms`. Parse errors.
- `:134-151` "Scope Interface and Composition": top-level `sub:` / `pub:` /
  `srv:` blocks and `child_name/group_name` wiring. Removed by design issue
  33; a top-level `sub:` is a parse error.
- `:154-172` "Verification Rules": names `chain-shape`, `chain-link`,
  `chain-budget`, all deleted with `chains:` (commit `9881fbe`).

`docs/scheduling.md`

- `:66`, `:114`: `rr_timeslice_us: 100000`. Canonical is
  `rr_timeslice: 100ms`; `_us` is a deprecated alias (`sched/src/platform.rs:79-90`).
- `:81-84`: `PosixOverride` lists `budget_us: Option<u64>`; the field is
  `budget: Option<Duration>` with `budget_us` as alias (`platform.rs:184-198`).
- `:294-295`: diagnostics list only `ChainInfeasible` and `BandTooNarrow`;
  `ChainFeasibleWithoutWcet` and `UnmitigatedPriorityTie` exist
  (`sched/src/chain.rs:317-346`, `chain_aware_mapper.rs:524`).

`docs/contract-verification.md`

- `:98`: rule 20 is `chain-shape`; it is `jitter-range`.
- `:144-153`: division-of-labour table lists `chain-link`, `chain-budget`,
  `chain-sampling-feasibility` (deleted), uses `max_transport_ms` and
  `max_latency_ms` (retired).

`README.md`

- `:19-22`: "Vocabulary v2: `trigger:`, `sync:`, `buffer:`, `chains:`" and
  `:47-50`: "Cross-scope checks (`consistency`, `budget-overflow`,
  `chain-link`, ...)". `chains:` and `chain-link` are gone.

`docs/contract-theory.md:416-417`, `:549-575`: `max_transport_ms`,
`max_age_ms`, and `chain-link` / `chain-budget` /
`chain-sampling-feasibility` as live rules.

## Impact

A contract author who copies the first example under "drops" or "timing"
in the spec gets a parse error whose hint names a key the doc never shows.
The slide deck, which is the artefact people read first, teaches a scope
interface that was removed by design. Nano-ros and the island contract
were written against `format-reference.md` and the source, which is why
they parse; the prose is what a newcomer reads.

## Fix direction

- Mechanical pass with `format-reference.md` as the oracle: replace every
  `max_drop_rate` / `max_consecutive` example with a `drop:` block; every
  `_ms` / `_us` spelling with the typed duration; delete the scope-interface
  and chain material or mark it historical.
- Regenerate the rule tables from `default_rules()` (19 or 20 per #0037)
  and the consumer's list; cite `jitter-range`, `once-durability`,
  `sync-feasibility`, `queue-drain-rate`, `inherited-rate`,
  `explicit-trigger` by name.
- Add a doc test that parses every fenced `yaml` block in `docs/*.md` with
  `parse_manifest_str`; the blocks that are meant to fail get a marker.
  This is the only thing that keeps the prose from drifting again.

## Provenance

Brief A (`brief-A-rlm-grammar.md`, section 6), 2026-09-18, which has
file:line for every item; each re-verified at rlm `origin/main` `ea5cbea`
(worktree `rlm-gaps`) on 2026-09-21.
