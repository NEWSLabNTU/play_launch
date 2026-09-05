# Phase 70 — the consumer census: which fields are actually read

Status: **W1, W2 complete** (manifest crate `v0.1.24`).

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

## Not done
- **W3 — the derivation/agreement corpus metric.** `rate-mismatch` /
  `derivable-rate` generalised: for each derivable field, count agreements and
  disagreements over the whole corpus. Zero disagreements is the evidence for
  retirement, replacing the by-hand provenance argument that carried `chains:`.
- **The `kind` column** (fact / requirement / consequence) on the field table.
  W1 gives the `consumer` half; `kind` is a judgment per field. W2 made five
  of those judgments implicitly and wrote none of them down in the table —
  the column is still the right home, and still empty.
- **Cross-scope `qos-match`.** The resolver's merged-graph copy of the rule
  still checks reliability and durability only; liveliness and the lease are
  checked per manifest. Same gap the per-manifest rule had until W2, one
  layer up.
