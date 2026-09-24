---
id: 55
title: "an action's `server:`/`client:` refs are never reconciled against the launch dump — #0048 without even the fallback"
status: open
type: correctness
severity: medium
---

# 0055 - topics and services resolve against the launch tree; actions do not

**Repo:** `play_launch` at `599f9f1b`
**Affects:** `src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs:4446`
(`resolve_actions`); compare `resolve_topics` and `resolve_services`, which
route through `resolve_endpoint_ref`

## Symptom

```rust
fn resolve_actions(manifest: &Manifest, scope: &ScopeEntry, index: &mut ManifestIndex) {
    let ns = &scope.ns;
    for (action_name, action_decl) in &manifest.actions {
        let fqn = qualify_name(ns, action_name);
        let servers: Vec<String> = action_decl
            .server
            .iter()
            .map(|ep_ref| qualify_endpoint_ref(ns, ep_ref))   // <-- naive
            .collect();
```

`qualify_endpoint_ref` prepends the scope namespace and stops. It never
consults `index.node_identity`, the launch dump's map from a bare node name to
the FQN it actually got — which `resolve_endpoint_ref` exists to do and which
every topic and service ref goes through.

So an action ref is scope-qualified and never checked against the launch tree
at all.

## Why this is worse than #0048

#0048 was about the FALLBACK being silent: `resolve_node_fqn` consults the
identity map first and only qualifies naively when the lookup misses, and the
complaint was that the miss produced no diagnostic. That is now an error
(`node-identity-unknown`).

Here there is no lookup to miss. An action ref that names a node the launch
file DID declare — under a different namespace, or with the `-N` ordinal a
node the launch file did not name receives (#0017/#0018) — resolves to the
wrong FQN even when the right answer was available. The identity map is right
there and nothing reads it.

Consequence: every requirement written on an action endpoint is attached to a
vertex that may not exist, exactly as in #0048, and #0048's new diagnostic does
not fire because the site never calls the function that emits it.

## Impact

Bounded by how much anyone uses `actions:` in a contract today — which is
little, and is the only reason this has not bitten. It is the last of the four
entity kinds (`topics`, `services`, `actions`, and the `nodes:` keys) not
reconciled, so the inconsistency is the defect: an author has no way to know
that the same `node/endpoint` spelling means one thing under `topics:` and
another under `actions:`.

## Fix direction

Route `resolve_actions` through `resolve_endpoint_ref` like its two siblings,
and include its refs in `check_contract_node_identity`'s site sweep (#0048's
diagnostic) so an unknown node in an action ref is reported at
`actions.<a>.server` / `.client` rather than silently mis-qualified.

This is a behaviour change to action resolution, not only a new diagnostic:
a contract whose action refs currently resolve to a wrong-but-consistent FQN
will start resolving to the right one, or start erroring. Check the fixtures
that declare `actions:` before and after and state the difference.

Gate: a fixture where an action's server is a node whose FQN the scope
namespace does not predict (the `unnamed_node` shape is the cheapest), with
the ref asserted to resolve to the launch tree's FQN — plus the #0048
diagnostic asserted on a genuinely unknown name.

## Provenance

Found 2026-09-25 while fixing #0048, by the agent that implemented it, and
confirmed by reading `resolve_actions` at `599f9f1b`.
