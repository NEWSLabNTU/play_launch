---
id: 1
title: "Node page misses nodes — bare-name keys silently collide; orphaned composables dropped"
status: resolved
type: bug
severity: high
github: https://github.com/NEWSLabNTU/play_launch/issues/1
---

The member registry (`metadata: HashMap<String, MemberMetadata>`, and with it
`shared_state`, `control_channels`, `tasks`) is keyed by a bare,
non-namespaced member name with silent-overwrite inserts; only containers
get `_N` dedup (`coordinator/builder.rs:233-244` vs plain inserts at
`:217` regular and `:416` composable).

Drop mechanisms observed:
1. Same node name across namespaces → last writer wins.
2. `name=null` nodes fall back to `exec_name` (`replay.rs:693-699`,
   Autoware-common) → duplicates collapse; literal `"unknown"` bucket.
3. Frontend re-collapses by name (`store.js:151-157`; Preact
   `key=${node.name}`).
4. Composables with an unmatched `target_container_name` are skipped with
   only a `warn!` (`builder.rs:425-441`) — never registered at all.

Side effects beyond the node page: health counts shrink; start/stop routes
to the shadowing member.

Fix: FQN identity end-to-end + register orphans as
`Blocked{ContainerNotFound}` — design `docs/design/member-identity.md`,
plan `docs/roadmap/phase-50-fqn-identity-and-web-visibility.md`.

## Resolution (phase-50, 2026-07-25)

Canonical member id `kind:/ns/name[#N]` (`member_actor/member_id.rs`)
keys every registry (`metadata_map`, `shared_state`, `control_channels`,
`tasks`, `virtual_member_routing`, actor names, `node_fqn_map`), the REST
routes, and the frontend store/Preact keys. Orphaned composables register
as `Blocked{ContainerNotFound}`. A FIFTH drop mechanism surfaced during
acceptance: `structure.nodes` (the sole spawn source since 47.B3) is a
`BTreeMap<FQN, _>` that silently overwrote true duplicates in
`model_builder::insert_node` — duplicates now keep both instances under
ordinal `#N` keys (the ROS `__node` name comes from `inst.node_name`, so
the suffix never reaches the process args).

Acceptance: synthetic collision launch (cross-ns pair, 2× anonymous
listeners, node+container name share, orphaned composable) → 8/8 on the
node page, `Registered 8 == Spawning 8`, stop routed only to the targeted
duplicate.
