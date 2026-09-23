---
id: 42
title: "a subscriber's `max_transport` never reaches the model, so the shared derivation and the checker compute different route totals"
status: open
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
