---
id: 48
title: "a contract node key that matches no node resolves by namespace fallback, and nothing says the contract describes something that is not there"
status: resolved
type: correctness
severity: medium
---

# 0048 - the contract is checked against a node the launch tree does not have

**Repo:** `play_launch` at `ccd4adc5`
**Affects:** `src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs:4178`
(`resolve_node_fqn`) and `:4197` (`resolve_endpoint_ref`)

## Symptom

Measured on 2026-09-24 against `tests/fixtures/simple_test/launch/unnamed_node.launch.xml`:
a contract keyed `talker-1` resolved to `/talker-1` where the running node is
`/identity_test/talker-1`, and `play_launch check` reported **`1 clean, 0
errors, 0 warnings`**, exit 0. `--export-graph` shows the wrong FQN, which is
the only way to see it.

So a contract can describe a node that does not exist, and the tool agrees
with it.

## Cause

```rust
pub(crate) fn resolve_node_fqn(index, scope_id, ns, node_name) -> String {
    if node_name.starts_with('/') { return node_name.to_string(); }
    if let Some(fqn) = index.node_identity.get(&(scope_id, node_name.to_string())) {
        return fqn.clone();
    }
    qualify_name(ns, node_name)
}
```

The middle branch is the real lookup — the launch dump's identity map. When it
misses, the fallback qualifies the bare name against the SCOPE's namespace,
which is not necessarily the node's: a node whose namespace comes from its own
attribute rather than from the scope lands somewhere the scope namespace does
not predict. The result is a syntactically fine FQN that names nothing, and no
rule asks whether a declared node exists in the launch tree.

The fallback itself is defensible — a contract may legitimately describe a
scope resolved without a dump. What is missing is the diagnostic when the
identity map was available and the name was not in it.

## Impact

Silent and total: every requirement on that node — rates, budgets, QoS,
hazards it guards — is checked against a vertex nothing runs, and reports
clean. A contract can rot against a renamed node and keep passing, which is
the failure mode contracts exist to prevent.

It also interacts badly with issue #0017's keying: a node the launch file did
not name is keyed by its EXECUTABLE plus an ordinal, so the bare key an author
would naturally write is exactly the one most likely to miss.

## Fix direction

When `index.node_identity` is non-empty for the scope (i.e. a dump WAS
available) and a bare key misses it, emit a diagnostic naming the key, the
FQN it fell back to, and the nearest candidates from the identity map. Error
or warning is a judgement call — a warning risks being ignored, an error
breaks contracts written against a scope with no dump, so gate it on the map
being present rather than on severity alone.

Do not remove the fallback; it is what lets a contract be checked without a
launch dump at all.

Consider also emitting absolute FQNs from tooling that generates contracts
(`scripts/capture_manifest.py` already does, for this reason), since an
absolute key passes through verbatim and cannot be mis-qualified.

## Provenance

Found 2026-09-24 while validating the capture script for #0044, by comparing
`--export-graph` output against the running graph; `resolve_node_fqn` was read
at `ccd4adc5`.

## Renumbered 2026-09-24

Filed as #0047 and renumbered to #0048: another session pushed its own
#0046 first, so two issues briefly shared the id. Commit messages written
before the collision (`d5ae3f9b`) refer to the old numbers.

## Resolved 2026-09-25

`node-identity-unknown` at Error severity, gated on the scope having at least
one entry in `node_identity` — which is exactly the statement that the dump
describes this scope's nodes and a lookup was possible. A contract checked with
no launch tree stays silent, so the diagnostic cannot break a legitimate
contract shape.

Error rather than Warning was justified against the corpus rather than by
preference: 14 contract fixtures and Autoware 1.5.0's planning_simulator give
ZERO hits, and misspelling `mrm_handler` in a copy of the Autoware contract
fires it with the right suggestion — so the zero is a result, not a disabled
rule. A warning was rejected because cross-scope warnings do not affect the
exit code, which would have left the defect standing: a clean report is the
symptom.

Also corrected a detail of this report: the running node is `talker`, not
`/identity_test/talker-1`. The `-1` is the model key's ordinal (#0017/#0018),
so the key missed for two compounding reasons and the diagnostic names the
real bare name.

Extended to action refs by #0055, which found `resolve_actions` never consulted
the identity map at all.
