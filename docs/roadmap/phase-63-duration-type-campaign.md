# Phase 63 — the duration type campaign

**Status:** 📋 planned. Foundation built (`ros-launch-manifest` `ca027aa`,
unpushed).
**Executes:** [Phase 59](./phase-59-timing-vocabulary.md), which decided the
vocabulary. This decides how to land it.
**Spans:** `ros-launch-manifest`, `play_launch`, and every contract on disk.
nano-ros pins the same types and inherits the result.

## Why this is a separate document

Phase 59 answers *what the vocabulary should be* — units on values, old names
as deprecated aliases, canonical emission — and its reasoning is unchanged. It
does not answer *how to change 455 call sites across two repositories that are
pinned to each other by tag*. Splitting that out keeps the decision stable
while the execution plan takes revisions.

## The choice this phase makes

Two ways to deliver Phase 59's benefit:

| | approach | 1000x error stopped? | sites touched |
|---|---|---|---|
| **B** | parse-only: accept `12ms`, keep `max_latency_ms: f64` in the struct | in authored files, yes | ~15 |
| **A** | **type change: `max_latency: Duration` end to end** | **everywhere, including code** | **~455** |

**A is chosen**, deliberately and with the cost visible. B stops an author
writing the wrong thing; it leaves every consumer holding a bare `f64` whose
unit lives in a field name, so the next arithmetic mistake is as available as
it ever was — and `sched_derive.rs` substituting a *deadline* for a *cost* was
exactly that kind of mistake. A removes the class from the code, not only from
the files.

The cost is real and is the reason for a campaign rather than a commit.

## Measured inventory

Not estimated — counted, 2026-08-21.

| | `ros-launch-manifest` | `play_launch` |
|---|---|---|
| Rust sites | ~238 | 217 |
| of which `src/` | ~131 | ~194 |
| of which tests/fixtures | ~107 | ~23 |

Contract and platform files carrying a renameable key **in this repo: ~10**,
under `tests/fixtures/` and `examples/rt_av_demo/`. An earlier count of 445
files was wrong — it matched unrelated `_ms` keys in Autoware parameter files.
Phase 59's "two field names, mechanically" holds for DATA; it is the Rust that
is large.

**112 sites do arithmetic** on these values (`*`, `/`, comparisons, `as f64`).
Those are the ones that can change behaviour rather than merely fail to
compile, and they are where the phase's risk actually lives.

Field concentration, highest first: `max_latency_ms` (92 manifest / 67
play_launch), `deadline_us` (37/30), `budget_us` (17/35), `max_transport_ms`
(22/38), `max_interval_ms` (19/0). The long tail — `jitter_ms`,
`tolerance_ms`, `spin_period_us`, `time_slice_us` — is under 10 each.

## The ordering constraint that shapes everything

`ros-launch-manifest` is a git dependency pinned **by tag** from three
manifests here, and nano-ros pins it too. So:

- a type change in the manifest repo cannot land "a bit at a time" for
  consumers — the tag moves and everything pinned to it moves together
- but it CAN land a bit at a time *inside* the manifest repo, on `main`,
  before any tag exists
- and play_launch can only follow a tag

That gives the wave structure below its shape: everything that can be done
without moving the tag is done first, the tag moves once, and the consumer
migration happens against a tag that already works.

## Waves

### W1 — the type ✅ (`ca027aa`, `913e945`, on `main`)

`types/src/duration.rs`: nanoseconds in `i64`, parses `12ms`, canonical form
is the coarsest integral unit.

**One type, two entry points** — a correction to this plan's first draft, which
assumed a single mechanism. There is one `Duration`; what differs is how a
field is *read*:

- **serde** (`compat::opt_millis` / `compat::opt_micros`, applied with
  `#[serde(alias = "…_us", deserialize_with = "…")]`) for everything
  deserialized normally.
- **`from_legacy_scalar`** for the hand-rolled contract reader, which exists
  because `parse_manifest_with_spans` produces the `SpanIndex` the checker uses
  for source excerpts, and serde gives no line or column into the original
  YAML. That is a constraint, not an inconsistency.

`the_two_entry_points_agree` asserts the two cannot drift.

**A documented asymmetry.** Through serde, a bare number under the NEW name is
read in the legacy unit rather than rejected: `alias` gives the deserializer
one field and cannot report which spelling matched, so refusing would need a
hand-written `Deserialize` per containing struct. Reading it as the legacy unit
is conservative — an un-migrated file cannot change meaning — and the leniency
is removed with the alias at W6. The hand-rolled side has no such limit and
rejects properly. Pinned by
`a_bare_number_takes_the_legacy_unit_through_serde`.

An earlier revision of this wave introduced a second type
(`sched::compat::PlatformDuration`) to paper over the mechanism split. Two
types for one concept was the wrong answer and it was removed in `913e945`.

### W2 — migrate the manifest repo's own fields (platform half ✅ `4e922a3`)

Struct fields become `Duration`, `parse.rs` uses `yaml_duration`, the 112
arithmetic sites become explicit conversions. Internal to one repo, no tag
moves, `cargo test` is the gate.

Order within the wave: platform side first (`deadline_us`, `budget_us`,
`period_us`, `spin_period_us`, `time_slice_us` — 5 fields, no `f64`, and the
v2 schema has none of them so the surface is the deprecated TOML bridge), then
the contract side (9 fields, `f64` today).

**Acceptance:** every existing test passes unchanged except where it asserts a
serialized string, and each such change is a unit suffix appearing — never a
value moving.

**Platform half done.** Five fields; 443 tests pass, 0 fail. Two findings worth
carrying into the contract half:

- **grep overstates the work by an order of magnitude.** It reported 67
  consumer sites for these fields; the compiler found **five**. The rest were
  test fixtures and doc comments. The campaign's ~455 figure is an upper bound
  on the same basis.
- **`ResolvedTier` was left speaking microseconds on purpose.** It is the wire
  to play_launch and nano-ros, so both sides move together in W4 under one tag;
  converting at that seam is what keeps this wave inside one repository.

### W3 — emit and lint

Canonical emission everywhere a manifest is written back, and a checker lint
naming the deprecated spelling with its replacement. Phase 59 requires the
lint; the `explicit-trigger` lint is the precedent to follow.

**Acceptance:** re-emitting an unchanged migrated contract diffs empty; a
contract using the old spelling still validates and produces exactly one
warning per field.

### W4 — tag, then migrate play_launch

Tag `v0.1.9`, `just bump-manifest v0.1.9`, then the 217 sites here. The
compiler finds all of them; the 194 in `src/` are the work and the ~23 in
tests follow.

Highest-risk files, from the inventory: `ros/sched_derive.rs`,
`ros/chain_checks.rs`, `execution/sched_apply.rs`, `commands/measure.rs` —
they compute with these values rather than passing them through.

**Acceptance:** `just test-all` green including the parity gates; and
`examples/rt_av_demo/`'s `just measure` still reports detect 8.0 declared ->
~8.08 ms measured, because that fixture is an oracle and a unit slip anywhere
in the chain moves that number by 1000x.

### W5 — migrate the data

The ~10 contract files here, then Autoware's ~75 in the vehicle repo. Both
spellings parse throughout, so this wave cannot break a build; it is a
readability change and can trail the rest.

**Acceptance:** no file here uses a deprecated spelling; the lint from W3
reports zero on this repo's own contracts.

### W6 — sunset

Phase 59 requires a stated removal, because "deprecated" without a version
means "forever" and the alias burden is two spellings in the docs, the checker
and the tests. The `version:` field is parsed and never branched on
(`yaml_u32(doc, "version").unwrap_or(1)`), so the lever exists unimplemented.

Not scheduled here: it depends on nano-ros and Autoware finishing W5, which is
not this repo's call.

## Risks, and what each is guarded by

**A unit slip during migration is silent.** The type makes future slips
unrepresentable, but the migration itself converts `f64` ms into `Duration` by
hand 112 times, and `from_millis_f64` where `from_micros` was meant is exactly
the 1000x error this phase is about — committed by the phase that removes it.
Guard: `rt_av_demo` is an oracle (nodes burn exactly `burn_ms`), so W4's
acceptance catches a 1000x slip anywhere in the chain. Run it per wave, not
only at the end.

**The two repos drift while the tag is unmoved.** W2 and W3 land on the
manifest's `main` with play_launch still pinned to v0.1.8. That is fine and
intended, but the window must be short and the tag must be cut before anyone
starts W4.

**nano-ros inherits the tag.** It pins the same types and its own consumers
break the moment it bumps. The deprecated aliases mean its *data* keeps
working; its *code* has the same ~200-site problem. That is their migration to
schedule, and W4's tag is what starts their clock. Tell them before tagging,
not after.

**Test churn hides real changes.** ~130 of the sites are test fixtures and
expected strings. A diff of that size makes a genuine behaviour change easy to
miss in review. Guard: W2's acceptance says value changes are not allowed —
only unit suffixes appearing — so any diff hunk where a NUMBER changes is a
defect to explain, not a rename to skim.

## What this phase does not do

Move fields between files (Phase 58 W1/W3), change any field's meaning, or
touch rate fields (`rate_hz`, `min_rate_hz`, `max_rate_hz`) — Phase 59's scope
boundary stands: frequency is not a duration, needs its own unit set, and the
error it prevents is far cheaper.
