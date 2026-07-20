# Phase 48: Detangle Namespace from the Scope Tree

**Status:** 📋 Planned. A **minimal** alignment fix already shipped (see
"Shipped: minimal fix" below); this phase is the **full** detangle.
**Motivated by:** the 8 Autoware "scope advisories" (`/control` vs `/`)
surfaced while fixing the parser param-FQN bugs (2026-07-20). Root-cause
discussion: the parser conflates two orthogonal axes — **scope** (structural:
which `<include>`/`<group>` a node came from) and **namespace** (a sequential
`<push-ros-namespace>` accumulator).

## The problem

`ScopeEntry.ns` tries to store "the namespace of this scope". But namespace is
NOT a scope property:

```xml
<group>
  <node a/>                       <!-- ns ""    -> /a       -->
  <push-ros-namespace ns="x"/>
  <node b/>                       <!-- ns /x    -> /x/b     -->
  <push-ros-namespace ns="y"/>
  <node c/>                       <!-- ns /x/y  -> /x/y/c   -->
</group>
```

`<push-ros-namespace>` is a **sequential, per-member** action — zero-or-many
per group, applying to everything that follows it in scope. a, b, c share ONE
scope but three namespaces. No single `scope.ns` can label them. The correct
per-node namespace already lives on each node (`node.namespace`, whence the
model builds FQNs). `scope.ns` is a lossy summary that is impossible to make
correct in the general case (push-after-members, double-push).

The two parsers picked different lossy summaries:
- **Rust**: created scopes for `<include>` AND `<group>`; a group scope's `ns`
  was the accumulated `current_namespace()` at group entry — so a group nested
  under a `<push-ros-namespace>` inherited it (`/control`).
- **Python** (`dump_launch`, ROS-standard): creates scopes ONLY for
  `<include>`; scope `ns` = `ros_namespace` at include time; no group scopes.
  So a node's scope-ns is the ns of its enclosing FILE (intra-file pushes don't
  change it) → `/`.

Neither is "namespace"; both are annotations. FQN resolution never used
`scope.ns` (model FQN = `fqn(node.namespace, name)`), and contract binding uses
the scope PARENT CHAIN + FQNs, not `scope.ns` — so this was always a
cosmetic/label divergence, not a fidelity bug (119/119 FQNs matched throughout).

## Shipped: minimal fix (parser alignment)

`traverser/entity.rs`: a group scope's `ns` now **inherits its parent scope's
ns** instead of the accumulated `current_namespace()`. File (include) scopes
keep `ns = current_namespace()` at include (already matched Python). Net: a
node's scope-ns equals its enclosing file scope's ns — identical to Python.
The 8 Autoware advisories are gone; both parsers agree. The `<group ns="…">`
push still happens (children's FQNs stay correct); only the SCOPE label changed.
Regression: `lib.rs::test_namespace_scoping_with_push_ros_namespace` now
asserts nested group scopes inherit `/`, not `/robot1/sensors`.

This is a faithful alignment, but it keeps `ScopeEntry.ns` — still a lossy
namespace summary that happens to be consistent now.

## The full detangle (this phase)

Make the model say what's true: scope = structure, namespace = per-node.

- **48.1 Model**: drop `ns` as scope IDENTITY. Scope entries carry structural
  identity only (file scopes: pkg/file/path; group scopes: a synthetic
  parent+index key or the group's own `ns=`/scoped flag). Keep the parent
  chain. Namespace stays exclusively on `NodeInstance.namespace` (already
  there). Decide: keep `ns` as optional *relative* metadata (the group's own
  increment) or remove entirely.
- **48.2 `scope_keys()`** (`model_builder.rs`): key scopes structurally, not by
  `ns`. Affects `structure.scopes` keys, the per-instance `scope:` label, and
  `contracts.scope_paths` keys — all internal-consistent, but a **shared-model
  wire change** (see coordination).
- **48.3 Launch tree** (`LaunchTreeView.js` / `context` command): render the
  include/group STRUCTURE as the tree skeleton, with each node's namespace as
  node metadata (both already available) — strictly richer than today's
  ns-labeled tree.
- **48.4 Both parsers**: with `scope.ns` no longer identity, the Rust group
  scope / Python include-only asymmetry stops mattering for the key; verify
  `compare_scopes.py` / `compare_models.py` compare structural scope identity
  (parent chain + origin) instead of ns.
- **48.5 Contracts**: confirm `find_node_decl` (parent-chain) and
  `resolve_node_ref` (FQN) are unaffected; re-key `scope_paths`.

## Coordination (nano-ros track)

`structure.scopes` and the per-instance `scope:` field are in the shared
SystemModel that nano-ros consumes. Changing the scope-key scheme
(ns-based → structural) is a wire change. The `scope` field is documented as
"just a label" (low semantic risk), but the change must be announced/sequenced
with the nano-ros model-ingest track before landing 48.2.

## Out of scope

Node FQN resolution (already correct + scope-independent); the param-FQN fixes
(shipped 2026-07-20); the contract layers.
