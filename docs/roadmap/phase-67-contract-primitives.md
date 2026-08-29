# Phase 67 — contract primitives: the vocabulary

Status: **planned.**

Design of record:
[docs/design/contract-primitives.md](../design/contract-primitives.md) (the
rule) and [docs/design/contract-axes.md](../design/contract-axes.md) (the
completeness survey, the rulings, and the phase arrangement this doc
implements). Read those first; this doc is the work, not the argument.

**This phase removes nothing.** Every field it adds is optional, every old
spelling keeps working, and nothing consumes the new facts yet. Retirement is
phase 68's last wave, gated on verification against a running system — see
`contract-axes.md` §5, phase C.

## Why

A contract should state **what the code does** and **what it must achieve**.
Anything computable from those two is a consequence, and a consequence written
by hand is a second copy of something the tool already knows.

Measured on `tests/fixtures/rt_workspace` — three nodes, ~95 lines — the number
**100 appears nine times**: once as the fact (`trigger: { timer: { rate_hz: 100
} }`), three times as `topics.*.rate_hz` that propagate from it, five times as
identical `min_rate_hz`. Plus five lines of `segments:` restating a route the
graph already defines. Roughly **a third** of that file is irreducible.

The completeness pass then found the opposite problem: six declared facts that
**no arithmetic reads at all** (`jitter`, `lifespan`, `max_response`, the sync
window, sampling cost, per-path cost), and five axes a safe system needs that
the vocabulary cannot express. Two of those are adopted here, two deferred by
decision, one kept on the list because ISO 26262 requires it.

Adding vocabulary while existing fields are decorative would make the ratio
worse, which is why this phase is paired with phase 68 rather than shipped
alone. A field you can write and nothing checks is indistinguishable from a
comment.

## Scope

Adopted (`contract-axes.md` §0):

| axis | this phase adds |
|---|---|
| jitter as a requirement (§3.6) | `PathDecl.max_jitter`, `PathDecl.min_latency` |
| deadline-miss handling (§3.2) | `PathDecl.miss { tolerate, consecutive, action }` |
| executor structure (§3.4) | `NodeDecl.concurrency.exclusive` |
| ROS 2 QoS (§4) | `QosDecl.deadline`, `QosDecl.lease_duration` |
| operational modes (§3.3) | **the mitigation only** — no modes |

Not in scope: consuming any of it (phase 68 W1–W2), verifying it on a running
system (68 W3), retiring anything (68 W4).

## The dependency that shapes the wave order

`ros-launch-manifest` is a **git dependency pinned by tag**, and **three**
manifests in this repo name the tag: `src/play_launch/Cargo.toml`,
`src/ros-launch-resolve/Cargo.toml`, `tests/Cargo.toml`. Naming the same tag is
what makes cargo resolve ONE instance; different revisions produce two
same-named packages from different sources and `SystemModel` becomes two
incompatible types.

So every schema change is: change the manifest crate, tag it, then
`just bump-manifest <tag>` here — which validates the tag on the remote before
editing anything, rewrites all three manifests, refreshes all three lockfiles,
and verifies each lock names exactly one revision.

`tests/Cargo.toml` is the one that drifts unnoticed; it sat four tags behind
once, because it uses the crate for a single call and drives `play_launch` as a
subprocess, so nothing failed to compile. **Commit the lockfiles.**

nano-ros pins the same crate from four manifests across two workspaces. W5
covers what has to be said to them.

---

## W1 — schema

In the manifest crate's `types` crate. All fields optional; `deny_unknown_fields`
means an unknown key is already an error, so nothing silently ignores a typo.

### `PathDecl.max_jitter` and `PathDecl.min_latency`

```yaml
paths:
  detect:
    trigger: { input: [image] }
    output: [boxes]
    max_latency: 20ms
    max_jitter: 4ms       # spread across invocations, not a second budget
    min_latency: 6ms      # optional; best case
```

`min_latency` exists because **jitter is a spread and every bound we carry today
is an upper one**:

```
route_jitter = route_worst_case − route_best_case
```

Shipping `max_jitter` without a lower bound gives a requirement nothing can
check — a seventh write-only field, which is the exact failure this pairing is
meant to avoid.

It is optional on purpose. Phase 68 can check the **sampling** half of jitter
with no new facts at all (a timer boundary contributes the whole period,
regardless of callback speed), and the **execution** half only where a best case
is known. `play_launch measure` already computes per-invocation distributions
and can emit a measured minimum; whether that minimum is stable enough to budget
against is 68's first question, not this phase's.

The same two fields go on scope paths, where the end-to-end jitter requirement
is what a controller actually cares about.

### `PathDecl.miss`

```yaml
paths:
  control:
    max_latency: 10ms
    miss:
      tolerate: { n: 2, w: 100 }   # weakly-hard (m,K)
      consecutive: 1
      action: continue             # continue | skip_next | abort
```

Two independent things, deliberately not conflated: **how many misses are
tolerable**, and **what to do about one**.

`tolerate`/`consecutive` reuse `DropSpec`'s shape — same mathematics on a
different event — but must not reuse its *type*: a dropped message and a late
message are different failures and will want different fields. Same spelling
for the author, distinct types in the schema.

`action`'s three values come from the weakly-hard literature. Phase 68 W2 finds
out what they cost; the expectation going in is that `continue` is what CBS
already does and the other two describe an obligation on the node rather than
something play_launch can enforce.

### `NodeDecl.concurrency`

```yaml
nodes:
  detector:
    paths:
      to_boxes: { trigger: { input: [image] }, output: [boxes] }
      to_masks: { trigger: { input: [image] }, output: [masks] }
      health:   { trigger: { timer: { rate_hz: 1 } }, output: [diag] }
    concurrency:
      exclusive: [[to_boxes, to_masks]]
```

An **exclusion relation between paths**, not a group. A maximal mutually
exclusive set *is* a group, derived the way a route is derived from
`trigger`/`output`.

**The default does more work than the syntax.** Absent `concurrency:` means
every path of the node serializes — which is what both realizations already do
(`rclcpp`'s implicit per-node callback group is `MutuallyExclusive`; nano-ros's
`default_cbg_type` is the same string). Nothing is written unless the author
claims *more* concurrency than that, which is exactly the claim worth reviewing.

### `QosDecl.deadline` and `QosDecl.lease_duration`

ROS 2 supports eight QoS policies; we model six. The two missing are precisely
the runtime-enforced timing contracts. `liveliness` also stops being a bare
`String` and gains its lease duration.

### The modes mitigation

Modes are deferred (`contract-axes.md` §3.3). The mitigation is taken now,
because it costs nothing while the schema is open and converts a later rewrite
into a later addition:

- **Requirements stay addressable, not bare scalars.** A requirement reached
  through a lookup that happens to have one entry survives a mode dimension; a
  scalar does not.
- **`if`/`unless` are not bent into approximating modes.** They are launch-time
  conditions resolved once. Using them for modes would produce contracts that
  look mode-aware and are not — harder to migrate than contracts with no modes.

**Acceptance:** the crate builds, round-trips every new field, and rejects an
unknown key. A contract using none of the new fields deserializes to a
byte-identical `Manifest`.

---

## W2 — lowering

Carry the new fields through `model_builder` into the `SystemModel`. Present in
the model, read by nothing.

This wave exists separately from W1 because the model is the boundary nano-ros
consumes, and a field that stops at the manifest crate is invisible to them.

**Acceptance:** `resolve` emits the new fields when declared and omits them
otherwise (`skip_serializing_if`), under **both** parsers — the Rust default and
`--parser python`. Parity is checked by `just test-parity`, which `just test-all`
runs; a gate nobody runs is a gate that rots, and in that state one of them once
found #0021.

---

## W3 — migrate the tree's own contracts — DONE (phase 68 W4)

`tests/fixtures/rt_workspace`, `tests/fixtures/contract_merge`,
`tests/fixtures/contract_error`, `examples/rt_av_demo`, and the Autoware
contract set if present.

Migration here means **adopting the new spellings where they say something
true**, not rewriting for its own sake. A fixture with no jitter requirement
does not gain one.

The one real conversion is `rt_workspace`, which carried `chains:` and
`segments:`. The plan here was to keep them and gain the scope-path form
alongside, so W4's gate had both spellings to compare. That is not how it went:
the comparison was made against a SEPARATE fixture
(`tests/fixtures/contract_derived_chain`, the same system with `chains:`
removed), and the tree's own contracts were converted in one step at W4 —
`rt_workspace`'s provider sidecar and overlay and `rt_av_demo`'s, verified by
provenance rather than by the resulting priorities.

**Acceptance:** every fixture resolves; `just test-all` green including the
parity gates and the `rt_workspace` integration tests.

---

## W4 — the equivalence gate

The wave that makes this phase safe, and the reason it is additive.

**Acceptance, all of it mechanical:**

1. Every contract in the tree resolves to a model **byte-identical** to the one
   at this phase's base commit, modulo the presence of new optional fields.
   `scripts/compare_models.py` is the tool.
2. Both parsers agree (`just test-parity`).
3. `just test-all` green, with the silently-skipped-tests summary showing no new
   skips — a guarded test that skips still reports as PASSED, and 27 of 108
   integration tests once skipped silently while concealing 4 real failures.

**A phase 67 that changes any derived value has a defect in it.** That is the
whole criterion. The precedent is the Duration migration, whose acceptance was
*"a unit suffix appearing, never a value moving"*, verified by resolving the same
launch under both spellings and diffing the models — the only difference being
the platform file's own `sha256`.

### Mapper: minimum correctness

The mapper's job this phase is to **not get worse**. The new facts reach it and
are ignored. `check --sched --explain` output is unchanged for every fixture.

---

## W5 — tell nano-ros — NOT DONE

Two seams where the toolchains would otherwise drift, both recorded in
`contract-axes.md` §5:

- **Platform-file granularity.** Ours assigns per node; nano-ros assigns per
  `(node, callback_group)` — `ResolvedTier::members` is
  `Vec<(String, String)>`. Once the contract expresses exclusion, groups become
  derivable, and our platform file should agree on whether it can name one.
- **`deadline_policy` already exists on the RTOS side.** `ResolvedTier` carries
  `deadline_policy: Option<String>`, so nano-ros has a home for W1's
  `miss.action` that our platform file does not. Align before either side
  invents a second spelling.

Also worth sending: nano-ros **infers** callback groups from causal coupling and
treats declaration as an override (`PlanCallbackGroup::inferred`). W1's
exclusion relation is the shared source that inference could read instead of
each toolchain deriving from its own information.

**Acceptance:** a phase doc in nano-ros naming the pin bump and the two seams.
`~/repos/nano-ros/docs/roadmap/phase-378-duration-type-migration.md` is the
model for how that reads from their side.

---

## Risks

- **Half a manifest bump is worse than none.** Three manifests here, four in
  nano-ros. A partial bump fails as `expected TierDef, found
  ros_launch_manifest_model::TierDef` — a type mismatch pointing anywhere but at
  the pin. Use `just bump-manifest`; it refuses rather than half-applies.
- **`min_latency` may not survive contact with measurement.** It is optional
  precisely so that phase 68 can find the sampling half checkable and the
  execution half not, without this phase having promised anything.
- **`concurrency:` defaults to the safe answer, which means it changes nothing
  until someone writes it.** That is intended, and it means W4's byte-identical
  gate does not exercise it. Phase 68 W3 does, on Autoware under `observable`,
  where co-location is real — under the default `isolated` every node is its own
  process and the constraint is structurally vacuous.
