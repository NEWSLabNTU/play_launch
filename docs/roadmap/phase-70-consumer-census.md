# Phase 70 — the consumer census: which fields are actually read

Status: **complete** (W1–W4; manifest crate `v0.1.29`).

Phase 69 made the contract grammar enumerable — every key a contract may carry
is a row in the manifest crate's `types/src/field_table.rs`. That says what is
**legal**. It does not say what is **read**, and the difference is where this
campaign's recurring defect lives.

## Why

Four fields were retired in phases 67/68 — a chain's `semantics`, an
endpoint's `jitter`, and `lifespan`/`max_response` before rules were written
for them. Every one was found the same way: grep the field, read each hit,
notice that all of them are *transport*. `jitter` had three read sites for its
whole life and its own documentation row said **"Not checked"**.

That is a repeatable procedure, so it should not be a manual one. The test is
not "is this field mentioned?" but "is it mentioned anywhere that CONSUMES
it?":

| class | sites | is it a reason for the field to exist? |
|---|---|---|
| **transport** | parse, serialize, model lowering, merge equality, deprecation lint, graph export, CLI reporting | no |
| **consuming** | a check rule, mapper arithmetic, the executor, a runtime monitor | yes |

A field with zero consuming reads is write-only: delete it, or write the rule
it is waiting for. Both are decisions. The point is that neither gets made by
accident.

## W1 — the census, and a gate

`scripts/field_census.py` walks the struct fields of `types/src/types.rs` and
`model/src/lib.rs`, greps every `.<field>` read across the manifest crate,
`ros-launch-resolve`, `play_launch`'s runtime and **nano-ros**, and classifies
each hit by the file it lands in.

`scripts/field_census.py --check` fails when a field goes unread that is not
already in `scripts/field_census_baseline.txt`. It also fails when a baselined
field acquires a reader, because a baseline listing fields someone has since
wired up hides the next one that goes unread. Wired into `just check` as
`check-field-census`.

Three decisions that make the verdict trustworthy rather than merely
automatic:

- **nano-ros is in scope, and its vendored copy of this repo is not.** It
  builds its `MapperPath` from the MODEL, never from the manifest, so a model
  field only it reads is consumed even though nothing here touches it — a
  census stopping at the repo boundary would report `node_concurrency`, added
  in phase 68 W5 *for them*, as write-only. But nano-ros also **vendors a copy
  of play_launch** under `packages/cli/third-party/`; counting that would let
  a field look consumed downstream on the strength of the very lowering code
  that is transport here. Excluded by path.
- **Tests are ignored.** A test can exercise a field the product never reads,
  which is exactly how a vacuous test looks from the outside — and phase 68
  found three of those.
- **The grep errs toward reporting MORE reads.** A field read through a
  pattern match is missed, so a "write-only" verdict is conservative and a
  "consumed" one is not proof. Every finding below was confirmed by hand.

Result: **140 fields — 131 consumed, 9 unread.**

### Verified findings

Two were already known (`min_latency`, and the model-side copies). Four are
new.

**`exclude_patterns` — three mentions in the entire codebase.** The table row,
the struct field, and the line that parses it. It is documented as "node-name
globs this manifest deliberately does not describe", i.e. a suppression — so a
manifest that excludes a node still gets `dangling-entity` for it. The
suppression suppresses nothing.

**`correlation` — parsed, exported, lowered, never branched on.** Exactly the
shape `semantics: age` had when phase 68 deleted it: a `model::Correlation`
enum exists, `causal_graph` carries it, and no arithmetic reads it. `sync:` is
what states fan-in policy now, and `sync-feasibility` reads *that*.

**`lease_duration` — the liveliness half of QoS is dropped.** `qos_match` and
`qos_compat` check reliability, durability, depth and history. Liveliness kind
and its lease parse and go nowhere.

**`max_rate_hz` — only the lower bound is checked.** `min_rate_hz` has
`rate-hierarchy` and feeds rate derivation. The upper bound has neither, which
is backwards for queue overrun: the *over-fast* publisher is the one that
overruns a subscriber, and that is the case `queue-drain-rate` is about.

### One finding that is a cross-repo correction

`CLAUDE.md` records phase 68 W5 as having closed both contract seams, with
nano-ros reading `node_concurrency` and `max_jitter_ms` from the model.
**As of 2026-09-05 that is not true of nano-ros in any branch**: nothing there
mentions `node_concurrency` or `claims_concurrency`, its `phase-379` is a
different phase (API parity with the ROS 2 client libraries), and its actual
reads of `model.contracts` are `node_paths`, `pub_endpoints` and
`sub_endpoints` only.

So the seam that wave was about is still open, and open in the way that has
teeth: an absent `concurrency:` means every path serialises — what `rclcpp`
and nano-ros's `default_cbg_type` both do — while nano-ros's
`PlanCallbackGroup` INFERS groups from causal coupling and gives an uncoupled
callback its own `Reentrant` group. Opposite defaults, each picked silently,
deciding whether summing a chain's latencies is sound and whether a per-thread
reservation is.

This is the census earning its place: the claim was in a document, the code
disagreed, and nothing before now compared them.

## W2 — the rulings

One per unread field. The rule was the campaign's: a write-only field is a
comment with a schema, so either write the rule it waits for or delete it.
Which of the two was decided by asking what the field would *check* if it
were read — a field with no checkable claim behind it is a deletion.

| field | ruling | what changed |
|---|---|---|
| `exclude_patterns` | **deleted** | Three mentions in the codebase, and two different documented meanings (node globs in the table, topic prefixes in the guide) — neither implemented. `external:` is the read way to mark an expected-absent side. Parse error names it. |
| `correlation` | **deleted** | `timestamp` vs `latest` is exactly `sync:` present vs absent, and `sync:` is read by three rules and by rate derivation. Parse error names `sync:`. Old models still load; the golden fixture keeps `correlation: timestamp` on disk. `tolerance:` stays — `sync-budget` reads it. |
| `max_rate_hz` | **implemented** | `rate-hierarchy` checks the upper bounds: `pub.max_rate_hz >= topic.rate_hz >= sub.max_rate_hz`. A topic faster than its subscriber drains is an error regardless of scheduling. |
| `lease_duration` | **implemented** | `qos-match` applies the DDS matrix to `liveliness` (`manual_by_topic` ≥ `automatic`) and to the lease: a publisher asserting less often than the subscriber's lease is one the subscriber will periodically declare dead. |
| `min_latency` | **implemented** | New rule `jitter-range`: with both bounds declared, `max_latency − min_latency > max_jitter` is an error; `min > max` is a contradiction on its own. With `min_latency` ABSENT the bound is unverifiable and the rule says so at info level — **the first draft read absence as zero**, which turned every jitter requirement on a wide-budget path into a hard error and blocked model emission on `contract_w2`. An upper bound of 40ms says nothing about whether latencies cluster at 38..40 or range over 0..40. That is the absent-versus-zero confusion phase 60 removed from the chain checker, caught here by a fixture. |
| `node_concurrency`, `srv_endpoints`, `max_response_ms`, `tolerance_ms` | **kept, baselined** | Model-side copies of facts the contract declares and this repo consumes. They exist for nano-ros, which reads the model — and W1 established it does not read them *yet*. Deleting them would re-open the unobservable seam phase 68 W5 closed; the debt is theirs to pay by reading, and the baseline says so. |

Every ruling landed with a test that fails without it. Corpus impact:
`correlation: timestamp` appeared in four fixtures and one golden model; no
fixture used `exclude_patterns`, `max_rate_hz` or `lease_duration` in a way
the new rules reject.

## W2 follow-through (2026-09-06)

- **`measure` now produces the floor.** `jitter-range`'s info said
  "`play_launch measure` produces the floor" and, when it shipped, that was
  false — `measure` had no notion of a minimum. `Dist` gained `min`, and
  the fragment prints one comment line per measured path
  (`nodes.<n>.paths.<p>.min_latency: <best response>ms`) under a header
  saying it belongs in the CONTRACT, not the platform file. Comments only,
  so stdout stays pasteable under `overrides:`.
- **Wrong types are errors** (manifest `v0.1.26`) — the other half of phase
  69's finding, recorded there.

## W3 — the agreement metric

Phase 68 retired `chains:` on a provenance argument made by hand: the
derivation reproduced every authored route, so the copy was redundant. Right,
and unrepeatable — nothing counted, so nothing could say when the next field
had earned the same. `scripts/derivation_census.py` counts: for each field
the resolver derives, it runs `check --format json` over every launch fixture
and tallies the resolver's own verdicts.

A second derivable field landed with it. A publisher's `min_rate_hz` is the
topic rate one hop earlier — five of `rt_workspace`'s nine copies of `100`
were this field — so `derive_and_check_endpoint_rates` emits
`derivable-min-rate` (equal: a copy) and `min-rate-mismatch` (a promise
above what the timers driving it can produce). Attribution is made only
where the topic has one publisher; with several the derived rate is their
sum and dividing it back out would present a bound as a rate. The
subscriber side is a requirement, never a copy, but deleting
`topics.<t>.rate_hz` would leave it unchecked (`rate-hierarchy` reads only
the declared topic rate), so `derived-rate-hierarchy` is the form that
survives the retirement.

**Result, 2026-09-06** (8 launch fixtures, 9 contract files):

| field | authored | agree | disagree | underivable |
|---|---|---|---|---|
| `topics.<t>.rate_hz` | 24 | 10 | 3 | 11 |
| `nodes.<n>.pub.<e>.min_rate_hz` | 22 | 12 | 1 | 9 |

All four disagreements are in fixtures that say so in their header
(`contract_error`, `contract_rates`) — they are the tests that keep the rules
honest, and the census marks them. Outside those, **every authored value the
graph could derive, it derived to the same number.**

The number that changes the ruling is the third column. Eleven of twenty-four
authored topic rates sit on chains the graph cannot derive — `contract_w2`
and `contract_concurrency` are driven by an `external: pub` source, so nothing
inside the tree says how fast it ticks. There the declaration is not a copy;
it is the only place that number can come from. So the retirement W3 licenses
is **narrower than `chains:`'s was**: delete the derivable copies (the
`derivable-*` infos name each one), keep the declaration where the graph
returns `Unknown`, and let the census say which is which. A blanket deletion
would remove a fact, not a consequence.

## W4 — the leftovers, closed (2026-09-06)

- **The `kind` column.** `Kind::{Meta, Fact, Requirement, ByEndpoint,
  Consequence}` on every live row of `field_table.rs`, rendered in the
  format reference. `contract-primitives.md`'s rule is now data a test can
  hold: the live consequences are pinned to exactly `topics.<t>.rate_hz`, so
  a new key of that kind cannot land without the census being told.
  `ByEndpoint` exists for `min_rate_hz`/`max_rate_hz`, the one pair whose
  kind depends on which side of the endpoint map they sit under.
- **Cross-scope `qos-match`.** The resolver's merged-graph copy now checks
  liveliness and the lease. This is the copy that matters for a lease: the
  publisher that asserts and the subscriber that times out are rarely in
  one launch file. Fixture pair `manifest_qos_liveliness_{pub,sub}`.
- **`criticality` is a closed set.** It accepted any string, and
  `sched_derive::parse_criticality` answered an unknown one with a debug log
  and `None` — `criticality: urgent` scheduled a node exactly as if nothing
  had been declared. `high | medium | low` at parse time now.
- **The `_ms` spellings are retired.** Nine aliases are parse errors naming
  the canonical form. The census had them as the *majority* spelling (14
  files to 10); 135 occurrences were migrated across both repositories'
  fixtures, test literals and guides. `deprecated-unit-suffix` is deleted
  with them — a lint for a spelling that cannot parse has nothing to say.
  One retirement condition met the way phase 68's was not: this one shipped
  behind a lint for the whole of 0.10's preparation, so the window existed
  even if no release carried it.

## Not done

