---
id: 42
title: "a subscriber's `max_transport` never reaches the model, so the shared derivation and the checker compute different route totals"
status: resolved
type: correctness
severity: medium
---

# 0042 - one derivation, two answers, wherever a contract overrides transport per subscriber

**Repos:** `ros-launch-manifest` (rlm) at `v0.1.38`, and `play_launch` at
`af0aeab1`. Filed here because rlm has no tracker.
**Affects:** `model/src/lib.rs` (`SubContract`, which has no transport field;
`TopicContract.max_transport_ms` at `:1297`); `derive/src/view.rs:229-234`;
`derive/src/graph.rs:90`; `src/ros-launch-resolve/resolve/src/ros/
manifest_graph.rs:337-339`

## Symptom

A contract may declare `max_transport` on a SUBSCRIBER endpoint, and the
grammar accepts it: `types/src/field_table.rs` carries `max_transport` twice
(topic context and endpoint context, both `Kind::Requirement`) and
`types/src/parse.rs` reads it at `:549` and `:706`.

The checker honours it. `manifest_graph.rs:337-339` prefers the subscriber's
value and falls back to the topic's, under a comment naming the reason:

```rust
// Per-subscriber transport latency override (Issue #44):
// the same ROS topic can have heterogeneous transport across
// subscribers (intra-process ~0ms vs cross-network ~10ms).
let max_transport_ms = sub_props
    .and_then(|p| p.max_transport.map(|d| d.as_millis_f64()))
    .or(topic.max_transport_ms);
```

The shared derivation cannot. `derive/src/graph.rs:90` sets an edge's
transport from `topic.max_transport_ms` alone, because that is all its view
has: `derive/src/view.rs:229-234` builds `TopicView` from
`model.contracts.topics`, and **`model::SubContract` has no transport field at
all**. So on any contract that overrides transport per subscriber, the two
derivations produce different route totals, with no diagnostic from either.

## Cause

The fact is declarable and read by the checker, but it is not carried on the
model, and `derive` reads the model. This is the phase-70 census shape (a
declared fact nothing downstream can see), except that here one consumer CAN
see it and the other cannot, which is worse: it is not an unread field, it is
a field that makes two copies of one derivation disagree.

`derive/src/graph.rs`'s own header says "The arithmetic is unchanged -- series
hops sum, a fork-join ...", which is true of the graph algebra and not of the
edge weights it runs on.

Phase 78 moved the derivation into rlm precisely so both toolchains would
schedule from the same facts. This is a hole left in that move, and it is
silent: the numbers merely differ.

## Impact

Bounded but real: it moves route totals, and route totals decide
`scope-budget`, `jitter-feasibility`, the mapper's `chain_feasibility` and
every derived deadline. A contract using the intra-process/cross-network
distinction that Issue #44 added the field for gets one answer from
`play_launch check` and another from anything driving the mapper through
`derive`. Nothing reports the divergence, so the first symptom is two tools
disagreeing about whether a system fits.

No fixture in either repository declares endpoint-level `max_transport`, which
is why nothing caught it.

## Fix direction

- Carry it: add `max_transport_ms` to `model::SubContract`, lower it in
  `model_builder.rs` beside the other endpoint facts (the `buffer` field
  landed the same way in v0.1.37), and have `derive/src/view.rs` build the
  per-edge value with the resolver's own precedence -- subscriber first,
  topic as fallback. That is additive on the wire and needs a manifest
  release plus a pin bump.
- Then delete the precedence from `manifest_graph.rs` in favour of the shared
  one, or the two copies will drift again the moment either is edited.
- A fixture that declares endpoint-level `max_transport` on a two-subscriber
  topic, asserting both consumers produce the same total, is what would have
  caught this and what should gate the fix.

## Provenance

Found 2026-09-23 while refreshing rlm's documentation: the theory pass
reported that `derive` and the resolver disagreed about the source of an
edge's transport. Both sites, the field table and `model::SubContract` were
read at the versions above to confirm it is a model gap rather than a coding
slip in `derive`.

## Ruling 2026-09-24 — what the number is

The project owner settled the modelling question the report left open:

> The ability to ensure the guarantee depends on the practical construction.
> A pub/sub on the same host can be managed; a topic connecting two machines
> cannot.

So transport latency is mostly a CONSEQUENCE of placement, and placement is a
fact the launch file already states. Three classes, and they differ in whether
a bound is a promise or an assumption:

- **same process** (one container, `use_intra_process_comms`) — a pointer
  handoff; the bound is structural;
- **same host, different process** — an RMW hop, MANAGED: play_launch places
  the processes and sets their priorities, cgroups and container membership,
  so a bound is something the toolchain can be held to;
- **different hosts** — the network, managed by nothing here. A number written
  there is an assumption about the environment, not a promise the system
  makes, and belongs with `external:` and phase 71's reaction vocabulary
  rather than with budgets.

That does not change this issue's fix, but it changes what the fix is FOR, so
the staging is now:

1. **This issue**: carry the declared endpoint value onto the model, have
   `derive` read it with the resolver's precedence, then delete the precedence
   from `manifest_graph.rs` so one copy remains. Gate: a fixture declaring
   endpoint-level `max_transport` on a two-subscriber topic, asserting both
   consumers produce the same total. Unblocked, and independent of the rest.
2. Derive the class from the model's placement facts; the per-class figure
   becomes a PLATFORM fact beside `rr_timeslice`, so a declared
   `max_transport` is a requirement on a link rather than an estimate of one.
3. Falsify by measurement — `InterceptionEvent` carries the topic, the header
   stamp and `monotonic_ns` at both hooks, so the delta for one message is the
   transport. `CLOCK_MONOTONIC` is comparable across processes on a host and
   not across hosts, so the class that cannot be managed is also the one that
   cannot be measured this way.

Recorded as design issue #55 in the manifest repository's
`docs/design-issues.md`, which is where the reasoning lives.

## Resolved 2026-09-24 — carried, gated, and still two copies

**Wave A** (rlm v0.1.42): `model::SubContract` gained `max_transport_ms`,
`TopicView` gained a per-subscriber map plus `transport_ms(sub_ep)` (override
else topic), and `derive`'s graph reads it per edge.

**Wave B** (here): `model_builder.rs` lowers it, and the all-absent early
return in `sub_contract` needed the new field added to it — not hypothetically:
the gate's `/et/fast/objects` declares `max_transport` and nothing else, and
without that line the whole endpoint is dropped from
`contracts.sub_endpoints`.

### The second copy is still there, and this is the evidence

`manifest_graph.rs:337` cannot call `TopicView::transport_ms`, for two
independent reasons:

1. **Visibility** — `TopicView` is `pub(crate)` in `derive`, and the crate
   re-exports only `parse_criticality_label`. It is neither namable nor
   constructible from outside.
2. **Input shape** — `build_global_graph(index: &ManifestIndex)` runs at LOAD
   time from types-level declarations, while `TopicView::from_model` is built
   from a `SystemModel`. The resolver's graph exists before any model does, so
   a public `TopicView` would still have nothing to be built from there.

Deleting the resolver's copy therefore means deleting the resolver's graph,
which is a larger change. Both sites now name the other and the test that
fails when they disagree, so the cross-reference is bidirectional. Closing it
properly means either making `TopicView` public and having the resolver build
one, or extracting a two-argument `transport_ms(sub, topic)` helper in rlm
that both sides call — both rlm changes.

### The gate reproduces the bug rather than asserting around it

`tests/fixtures/endpoint_transport/` adds what rlm's fixture structurally
cannot: a FORK. On a single linear route the transport only moves a total, and
a total can be right by accident; here it decides WHICH BRANCH is the critical
path, so the route and the total are both asserted.

One topic (`max_transport: 10ms`), two subscribers — one overriding to `0ms`,
one inheriting — rejoining at a sink:

| hypothesis | fast | slow | route | total |
|---|---|---|---|---|
| correct | 10+0+20 = 30 | 10+10+15 = **35** | producer → **slow** → sink | **40ms** |
| override ignored | 10+10+20 = **40** | 35 | producer → fast → sink | 45ms |
| override applied to all | 10+0+20 = **30** | 25 | producer → fast → sink | 35ms |

Both wrong precedences name the OTHER branch, and all three totals differ. The
scope budget is 1ms on purpose so `scope-budget` always fires and PRINTS the
resolver's route, rather than the test inferring it from silence.

Non-vacuity, with the reported defect live: reverting the lowering gives

```
left:  ["/et/producer", "/et/fast", "/et/sink"]
right: ["/et/producer", "/et/slow", "/et/sink"]
```

— the resolver printing `producer → slow → sink = 40.00ms` while `derive`
walked `producer → fast → sink`. That is this issue, observed rather than
argued.

play_launch lib 350, resolver 215, `endpoint_transport` 2 — all passing at
the v0.1.42 pin, with every lockfile naming exactly one revision and the
conflict-marker grep clean.
