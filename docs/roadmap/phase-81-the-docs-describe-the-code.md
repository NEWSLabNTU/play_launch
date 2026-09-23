# Phase 81 - the documentation describes the code that exists

Status: **complete** (2026-09-23), shipped as `ros-launch-manifest` **v0.1.39**
and pinned here. Follows phase 69 (the grammar became enumerable) and phase 70
(the consumer census), which between them made it possible to ASK what is
legal and what is read; this phase asks the third question — what the prose
CLAIMS — and reconciles the three.

## Why

v0.1.38 fixed the hand-written docs' EXAMPLES (issue #0038) and left their
prose and their arithmetic unverified, on the argument that a parse error is
the urgent half. That argument was wrong in one direction and right in the
other: the examples were indeed the part a reader copies, but the formulas are
the part a reader RECONSTRUCTS THE TOOL FROM, and two of them were stated
backwards.

The measurement that made this a phase rather than a cleanup: a mechanical
diff of the generated `docs/format-reference.md` against
`docs/launch-manifest.md` found **six live fields the specification never
mentioned** (`params:`, `concurrency.exclusive:`, the whole `functions:` /
`modes:` vocabulary, `severity_levels:`, and a subscriber's `buffer` and
`on_violation`). A grammar nobody can read from the spec is not a specified
grammar, whatever the field table knows.

## What it did

Four documents in parallel, one agent each, in four git worktrees so the file
sets could not collide: the specification, the theory pair, the scheduling
schema, and the entry points plus the design log.

### The two formulas that were backwards

Both read as correct, which is why they survived four phases of review:

- **Fan-in rate.** `contract-theory.md` asserted `f = min(f_A, f_B)`
  unconditionally, and a slide table said the same. The truth is the SUM of
  the input rates WITHOUT `sync:` — a callback fires once per message on
  *each* topic it is registered for — and the min only WITH it. Taking the min
  in both cases understates a fan-in node's load by exactly the factor that
  decides whether it fits, which is the worst possible direction for a bound.
- **Sampling cost.** The theory gave `S = Sum(P_i + C_i)` and attributed the
  verdict to `scope-sampling-feasibility`. `sampling_cost_ms` is the sum of
  the sampling PERIODS alone, which is what that rule judges; `P_i + C_i` is
  the traversal cost, which is what the mapper's feasibility check sums. Two
  different sums, one name.

### Claims with no implementation behind them

- A `max_age`-versus-budget consistency check ("if a subscriber has
  `max_age: 200ms` and the scope path `max_latency: 50ms`, the upstream must
  deliver data with age <= 150ms"). No such arithmetic exists anywhere;
  `max_age` is read statically in exactly two places.
- `qos-match` "runs per satisfiable arg model, sharing infrastructure with
  `satisfiability`". It has no arg logic at all.
- `consistency` merges three fields. It merges five.
- The example diagnostics throughout the spec were invented rather than the
  strings the rules emit.
- A capture mode, `--save-manifest-dir`, that no binary accepts (issue #0044).
- Per-child conditions on an `include:`, which are unrepresentable — the type
  has no field for one and the filter never filters includes (issue #0043).

### The scheduling document had gone stale at a crate boundary

It still said "this derivation is per-consumer", which v0.1.37 made false when
`derive/` became the one derivation both consumers call; and its fact table
listed rules that derivation does not use. It also claimed `sched` has no
`types` dependency (it does, for `Duration`), described a submodule that no
longer exists, called `ResolvedTier` a 13-field record (14), and gave the apply
layer as `sched_setscheduler(2)` rather than `sched_setattr(2)` — which is the
entire reason `SCHED_DEADLINE` and uclamp are expressible at all.

### The design log is history, and was being read as truth

`docs/design-issues.md` keeps every entry's body and vocabulary — an entry
explaining why `chains:` was removed must keep saying `chains:` — and gained a
status legend plus per-entry status lines. Two entries were contradicted by
the code rather than merely dated: **#50** records a decision to drop
`min_latency` that was REVERSED in phase 67 and never written down anywhere,
and **#52** still said "no code yet" while R1-R4 had all landed and `derive`
had shipped as the fifth workspace member at v0.1.37.

## The finding that outlives the phase

**The guard could not see a class of example.** `sched/tests/docs_yaml.rs`,
added in v0.1.38 precisely to stop the prose drifting again, matched a fence
with `trim_end()` and no `trim_start()` — so every INDENTED fence was
invisible to it. Three existed; two taught a spelling the parser rejects.

A guard that cannot see a class of input is worse than no guard, because it
reads as coverage: the count it printed (40 blocks) looked like the whole
document set and was five short. Fixed here, with bodies dedented by the
fence's own indentation so an indented block is judged on its content rather
than its leading spaces. **49 blocks now: 45 parse, 2 expect-error, 2 skipped.**

This is the same family as the three defects this session found PINNED by
tests that asserted them (`strict_mode_trips_atomic_flag`,
`test_interception_default_disabled`,
`from_model_degrades_colocation_warning_to_empty`) and as issue #0040, where a
feature-gated module had not compiled for weeks because no recipe built it.
The pattern is worth naming: **in this tree, a silent behaviour usually
arrives with something that blesses it** — a test, a comment, a doc line, or a
gate that cannot reach it. Finding the defect and finding its blessing are the
same job.

## What it did not do

- **Issue #0042 is left open deliberately.** A subscriber's `max_transport` is
  legal grammar, the checker honours it (preferring it over the topic's, for
  the heterogeneous-transport case Issue #44 added it for), and
  `model::SubContract` has no transport field — so `derive` reads the topic's
  value alone and the two copies of one derivation compute different route
  totals wherever a contract uses the override. Fixing it needs a model field,
  a lowering and a pin bump, which is a release of its own, and it is the
  first thing phase 78's successor should take.
- Five stale strings in code were fixed (a user-facing `scope-budget`
  diagnostic printing the retired `max_transport_ms`, and four module docs
  naming retired field spellings), but no rule, no formula and no grammar
  changed. The workspace version stayed `0.1.4`: a contract that parsed at
  v0.1.38 parses at v0.1.39.

## Gates

- `cargo test --workspace` in rlm: 551 passed, 0 failed, before and after.
- `docs_yaml`: 49 blocks, 45 parsed, 2 expect-error (each with a reason), 2
  skipped (each with a reason).
- Here, at the new pin: play_launch lib 346 passed, resolver 215 passed.
- A re-scan for retired vocabulary across `README.md` and `docs/*.md` returns
  only historical statements, the retired-spelling tables themselves, and one
  Rust struct field (`chains: Vec<ResolvedChain>` on `MapperInput`, which is
  not contract YAML).
