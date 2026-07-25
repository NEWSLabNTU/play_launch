# Design: Member Identity — FQN keys end-to-end

**Status:** Draft (2026-07-25). Closes issue 0001 (GH #1); prerequisite for
web-ui-organization.md. Implementation: roadmap phase-50.

## Problem

Every member registry keys on a bare display name (`HashMap<String, _>`),
with silent overwrite. Names are not unique: namespaces repeat node names,
`name=null` nodes fall back to shared `exec_name`s, composables repeat
node_names across containers, and regular/container/composable members
share one keyspace. Colliding members vanish from the web UI, corrupt
health counts, and misroute control actions.

## Decision

Introduce one identity type and use it as the key EVERYWHERE:

```rust
/// Collision-proof member identity.
/// display: what the UI shows (bare name);
/// fqn:     "/ns/node_name" — the ROS-graph identity;
/// ordinal: disambiguates true duplicates (same fqn launched twice).
pub struct MemberId { kind: MemberKind, fqn: String, ordinal: u16 }
// canonical string form: "{kind}:{fqn}[#{ordinal>0}]"
```

- `MemberKind ∈ {Node, Container, Composable}` — prefixing the kind removes
  cross-type collisions without merging state machines.
- `fqn` built by ONE constructor (`MemberId::new(kind, ns, name)`) that owns
  the namespace-join rules (root "/", trailing-slash trim) — replaces the
  scattered `format!("{}/{}")` sites. The SAME helper replaces the two
  existing joiners (`model_builder::fqn` and `manifest_loader::
  qualify_name`) so the emitted SystemModel and the runtime can never
  disagree on join rules.
- `ordinal` assigned by the builder on true duplicates (extends the current
  container-only `_N` counter to all kinds); display strings render
  `name (2)` rather than mutating the key.

## Interning — FQN is identity, u32 is the runtime key

Concern: FQN strings are long, and frequent messages shouldn't pay for
them. The member set is FIXED once the builder finishes (all registration
precedes web-server start), so intern:

```rust
pub struct MemberIdx(u32);                    // Copy, 4 bytes
pub struct MemberRegistry {
    infos:  Vec<MemberInfo>,                  // idx → {fqn, kind, display, ordinal, …}
    by_fqn: HashMap<Arc<str>, MemberIdx>,     // boundary lookups only
}
```

- HOT paths (StateEvent, load completions, control events, metrics ticks,
  `shared_state`) carry `MemberIdx` — no string hashing, no clones
  (removes today's pervasive `name.clone()`); `shared_state` becomes
  `DashMap<MemberIdx, _>` or a plain `Vec` slot per member.
- BOUNDARIES carry the string once: REST paths + the initial `/api/nodes`
  payload include the canonical FQN; subsequent SSE events may carry just
  the idx (the frontend holds idx→node from the initial fetch). Logs
  resolve idx→fqn at format time.
- ROS boundary is inherently stringly (`ComponentEvent.full_node_name`,
  service names): resolve to idx once at ingestion.

Rate honesty: current event rates (state transitions + ~1 Hz metrics ×
O(100) members) would tolerate string keys; interning is adopted for
Copy-semantics correctness and to keep the door open for high-rate
per-topic metrics.

## Keyed structures to convert

`metadata_map`, `shared_state`, `control_channels`, `tasks`
(coordinator/builder + handle), `composable_nodes` (container supervisor),
frontend `nodes` Map + Preact list keys + REST paths
(`/api/nodes/:id` takes the canonical string; display name stays a field).

## Orphaned composables

A composable whose `target_container_name` matches no container is
REGISTERED (state `Blocked{ContainerNotFound}`) instead of warn-and-drop —
visible with a badge, recoverable if the container appears later.

## Non-goals

No change to ROS-side naming or LoadNode semantics; ComponentEvent
name-fallback matching keeps using the ROS full node name (it now equals
`MemberId.fqn` by construction).
