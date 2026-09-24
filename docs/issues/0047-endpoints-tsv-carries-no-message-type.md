---
id: 47
title: "`endpoints.tsv` records no message type, so a capture can only describe topics the run exercised"
status: open
type: enhancement
severity: medium
---

# 0047 - the type is resolved at the init hook and thrown away

**Repo:** `play_launch` at `ccd4adc5`
**Affects:** `src/play_launch_interception/src/endpoints.rs:82`, `:93`;
`src/play_launch_interception/src/lib.rs:285-303` (the recorder) and the
publisher/subscription init hooks around `:699`;
`src/play_launch_interception/src/introspection.rs:95` (`find_type_identity`)

## Symptom

An endpoint record is five columns and none of them is the type:

```rust
let key  = format!("{node_fqn}\t{direction}\t{topic}");
let line = format!("{}\t{}\t{}\n", sink.member, std::process::id(), key);
```

A message type DOES reach disk, but only through
`stats_summary.json` / `frontier_summary.json`, which are keyed by **traffic**
— a topic appears there when a message crossed it. `discovered_topic_types.tsv`
is written only by the `RuleEngine`, so it exists only when contracts already
exist, which is circular for anything trying to produce a first contract.

`type:` is mandatory in the manifest grammar (a topic without one is a parse
error that drops the whole file), so a created-but-never-exercised endpoint
cannot be described at all. Phase 77 measured the size of that gap on one
Autoware run: **982 endpoints created, 63 carrying a message.**

## Cause

The type is in scope exactly where the endpoint is recorded and is then
dropped. The init hooks take `type_support: *const rosidl_message_type_support_t`
(`lib.rs:~699`), and `find_type_identity` already turns that into
`pkg/msg/Name` — it is what the publish/take path uses. The recorder helper
they call (`lib.rs:285-303`) takes `node`, `topic_name` and `direction`, and
never receives the type support, so `endpoints::observe` cannot write it.

## Impact

It bounds `scripts/capture_manifest.py` (issue #0044) to the traffic a run
happened to exercise, which is the difference between a capture that describes
a system and one that describes a sample of it. It is also why that script
stays a script: a verb implies completeness, and this is the one thing
preventing it.

More generally, every consumer that wants "what endpoints exist and what type
are they" has to join two artifacts with different keying rules and accept
that one of them only knows about topics that moved.

## Fix direction

Thread the type support through to the recorder and add a sixth column,
defaulting to empty where introspection cannot resolve it (the same tolerance
`find_type_identity` already has — it tries the C then the C++ identifier and
returns `None` if neither answers).

Both init hooks already hold everything needed; the helper's signature is the
only thing in the way. Keep the write allocation-free and failure-tolerant as
it is today (`endpoints.rs:99` — "a failed write must never disturb the node
it is describing").

Gate: a fixture where a node creates a publisher that never publishes, with
the capture asserting the topic is emitted with its type. That is exactly the
case the summaries cannot see.

## Provenance

Found 2026-09-24 while building the capture script for #0044; the record
format and the hook signatures were read at `ccd4adc5`.

## Renumbered 2026-09-24

Filed as #0046 and renumbered to #0047: another session pushed its own
#0046 first, so two issues briefly shared the id. Commit messages written
before the collision (`d5ae3f9b`) refer to the old numbers.
