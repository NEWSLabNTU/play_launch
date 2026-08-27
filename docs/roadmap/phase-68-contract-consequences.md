# Phase 68 — contract consequences: analysis, mapper, verification, retirement

Status: **W1–W3 done. W4's gate is MET and its lint is live; only the
deprecation window itself remains, which is calendar time.** Depends on phase 67 for W2 onward;
W1 needs no new vocabulary and is proceeding first (see §W1.a).

Design of record:
[docs/design/contract-primitives.md](../design/contract-primitives.md) and
[docs/design/contract-axes.md](../design/contract-axes.md) (§5, phases B and C).

Phase 67 added vocabulary and consumed none of it. This phase makes the analysis
and the mapper read it, **verifies the result on running systems**, and only then
retires what it replaces.

## Why

Two kinds of defect, and the second is the reason retirement waits.

**The analysis reduces facts wrongly, and does it silently.** A node with two
causal outputs — one input image, two extractions, different costs — is charged
the maximum over *all* its paths on every route, including routes that never
touch the expensive one. Measured on a purpose-built fixture:

```
/image → detector ─(boxes, 20ms)→ tracker   (10ms) → /tracks
                  └(masks, 35ms)→ segmenter ( 5ms) → /seg
```

| `to_masks` | `to_tracks` charged | true (20+10) |
|---|---|---|
| 35 ms | **45 ms** | 30 ms |
| 100 ms | **110 ms** | 30 ms |
| 12 ms | silent (= 30 ms) | 30 ms |

One-for-one above the max, on a route that never traverses `masks`.
Overestimating is the safe direction, so this is not a hazard — it is a
**silent** wrong answer, and it makes a per-route budget ungradeable for exactly
the multi-output nodes that most need one.

Same root cause makes cost unavailable: `exec_ms` is refused outright when
`path_count > 1`, which is honest (*"Absent is the honest answer"*) and still
leaves the common perception front-end with no cost reaching the mapper.
**The graph is node-keyed; the facts are path-keyed.**

**And some facts are read by nothing.** `jitter`, `lifespan` and `max_response`
have three read sites each — a `model_builder` copy, a merge-equality check, a
deprecation lint. No rule reaches a verdict from them.

## Scope

| wave | what |
|---|---|
| W1 | analysis reads the facts correctly |
| W2 | mapper acts on them |
| W3 | verify on running systems |
| W4 | retire, gated on W3 |

---

## W1 — analysis

### W1.a — per-path attribution — DONE

One fix, two symptoms (§1.1, §1.2). Carry path identity onto the graph edge so
the critical-path DP charges the path that produced the traversed topic, and
`measure` can attribute per `(node, path)`.

Today's reduction:

```rust
pub fn max_latency_ms(&self) -> f64 {
    self.paths.values()
        .filter_map(|p| p.max_latency.map(|d| d.as_millis_f64()))
        .fold(0.0_f64, f64::max)   // max over ALL paths in the node
}
```

**Acceptance — the expected value was known in advance and is met:**
`to_tracks` is charged **30 ms, not 45 ms**, and varying `to_masks` over
12/35/100 ms leaves it unchanged.

**What shipped.** The root cause was granularity, not arithmetic: the facts a
contract declares (`trigger`, `output`, `max_latency`) all live on `PathDecl`,
while the dataflow graph was keyed by node — so a node with two causal outputs
had one vertex and two answers, and no patch to the reduction could fix that.

- `GlobalEdge` gained `pub_endpoint`, the counterpart of `sub_endpoint`. The
  publisher endpoint was already computed in `build_global_graph` and discarded
  as `_pub_ep`; it is what lets a hop be attributed to the path whose `output`
  names it.
- `build_path_graph` lowers a subgraph to **path granularity** — a vertex is a
  path, an edge joins the path that published a topic to the path whose
  *effective* trigger consumes it (`effective_trigger()`, so Vocabulary v2
  `trigger: { input: [...] }` matches, not just the legacy `input:`).
- `critical_path` runs its DP over that graph. Timer/once/spontaneous paths
  consume nothing and so can never be entered — which is what makes them chain
  boundaries, and is the hook §W1.b needs.
- A hop no declared path accounts for falls back to the node-wide maximum, so
  the change is **never less conservative** than before.

Four tests in `manifest_graph.rs`, built from hand-assembled graphs rather than
cross-repo fixtures. Verified non-vacuous by reverting the attribution and
confirming the two defect tests fail with exactly the measured numbers
(`left: 45.0, right: 30.0`); the two invariant tests — fork-join takes
max(50,30)+20=70, and the unattributable-hop fallback — correctly pass in both
states, being guards rather than the defect.

Gates, all green: clippy clean, `just check-layer2-isolation`, and `just
test-all` — **485 parser + 291 play_launch + 181 resolver + 142 integration,
1099 tests, 0 failed, 0 skipped**, including the cross-parser parity gates and
an empty silently-skipped report.

Open (`contract-axes.md` §6.4): whether per-path cost then needs a `costs:`
section, or whether `measure` can attribute it once path identity exists.
Prefer the second; not yet verified possible.

### W1.b — sampling cost and sampling jitter — DONE (cost; jitter deferred)

`scope-budget`'s subgraph starts **at** the input topic, so a timer boundary
upstream is outside the traced region by construction. On `rt_workspace` that is
10 ms of 25 ms — **40%**, and the part no priority assignment can remove.

```
chain form   warning[chain-budget]: total (25.00ms = 15.00ms event-segment
             + 10.00ms sampling_cost) exceeds the chain budget (20ms)
scope path   clean — 0 budget diagnostics
```

Both terms enter **per route and are maximised, not summed**: with fork-join two
branches may have different boundaries, and summing across branches is the same
error as summing node latencies across them.

Sampling jitter rides along, and is the half of §3.6 that needs no new facts — a
boundary contributes the **whole period** to end-to-end jitter regardless of how
fast its callback is.

**Acceptance — met at the value predicted in advance:** `points_to_cmd`
expressed as a scope path totals **25.00 ms** (10 ms sampling + 5 ms filter +
10 ms control), matching `chain-budget` on the same fixture, and the diagnostic
now names the split the way `chain-budget` does.

**The stated cause was wrong, and both design docs are corrected.** This wave's
premise — that the boundary is "outside the traced region by construction,
since the subgraph starts at the input topic" — does not survive reading
`subgraph_for_scope_path`, which seeds sources from the input topic's
**publishers as well as** its subscribers. The boundary was in the subgraph all
along. Two independent causes were found instead, each of which alone produces
the measured 15 ms:

1. **No sampling term.** A timer path was charged its declared `exec` like any
   other path, so the period never entered at all — and `rt_workspace`'s
   boundary declares no `exec`, so it contributed exactly 0.
2. **A source truncated the route.** Because a subscriber of the input topic is
   also seeded as a source, and a source was treated as having nothing
   upstream, the DP discarded the hop that produced its data — dropping the
   boundary from the route even once the boundary had a cost.

Reverting either fix independently restores 15.0 against the expected 25.0,
which is what makes the pair load-bearing rather than one fix and one guess.

**What shipped.** `traversal_latency_ms` charges a timer-triggered path
`period + exec` rather than `exec`, the same one-period-per-boundary bound
`chain_checks` uses, so the two forms of one system agree. `CriticalPath` gained
`sampling_cost_ms`, summed over the winning route rather than the whole
subgraph, and the `scope-budget` diagnostic reports the split — the part of a
total that no priority assignment can remove is exactly what an author told to
"reduce this" needs distinguished. Source vertices now take the larger of
"start here" and "arrive from upstream" instead of ignoring their incoming
edges.

**Sampling jitter is deliberately NOT shipped.** A boundary contributes its
whole period to end-to-end jitter, and that is computable today — but
`max_jitter` is phase 67 vocabulary and does not exist yet, so the number would
have nothing to be checked against. Adding it now would create a seventh
write-only field, which is the exact failure `contract-axes.md` §2 is about. It
lands with phase 67.

Gates, all green: clippy clean, `just check-layer2-isolation`, and `just
test-all` — **485 parser + 291 play_launch + 185 resolver + 142 integration,
1103 tests, 0 failed, 0 skipped**, with an empty silently-skipped report.

### W1.c — groups from exclusion, and the validity diagnostic — DONE

Derive callback groups from phase 67's exclusion relation: a maximal mutually
exclusive set is a group.

Then the diagnostic that is the point of the whole axis. Today's summing is
sound **only because `--container-mode isolated` is the default**; under
`observable`/`stock`, nodes share an executor and it is not, with nothing saying
so. With exclusion in the contract and executor sharing known from the platform,
the checker can decide whether **summing is valid** and report when it is not.

That needs no response-time analysis, no blocking arithmetic, no executor
simulation. Converting a silent unsound assumption into a diagnostic is the part
that matters; the full analysis stays a later option.

### W1.d — consume the write-only fields — DONE

`jitter` (now the path-level requirement), `lifespan`, `max_response`, and the
sync window (`tolerance`, `max_interval`, `timeout`).

**Acceptance met**, with `tests/fixtures/contract_w1d` written to violate each
one and an integration test asserting all of them fire:

| field | rule | what it catches |
|---|---|---|
| `max_jitter` (new) | `jitter-feasibility` | a route's sampling jitter — one whole period per clock boundary — already exceeds the declared bound |
| `lifespan` | `lifespan-age` | a subscriber accepts data older than DDS will keep it |
| `sync.timeout`, `sync.max_interval`, `tolerance` | `sync-budget` | the synchronisation window is wider than the path's own budget |

`sync-budget` is the **complement** of the manifest crate's existing
`sync-feasibility`, which bounds the same window from below (it must span the
slowest input's period). The fixture violates both at once, so the window must
be ≥100 ms and ≤20 ms — an empty interval that neither rule alone can report.
That is the argument for adding the upper bound rather than folding it into the
existing rule.

`max_jitter` is checked without `min_latency` because sampling jitter needs no
best-case fact: a boundary contributes its whole period regardless of what the
callback costs. Execution jitter still needs a best case and waits for
measurement (`contract-axes.md` §6.1).

**`max_response` is deliberately left unconsumed**, and that is a finding
rather than an omission. It sits on the SERVER (`srv:`), is documented "runtime
monitoring only", and nothing in the vocabulary binds a service call to the
path that makes it — so any static rule would have to guess whether the call is
synchronous and which callback pays for it. Absent is the honest answer; the
missing fact is a caller-to-path binding, which is vocabulary work, not a rule.

**Found on the way**: `sync-feasibility` had been printing a Rust expression at
the user since the phase-63 rename —
`sync.timeout.map(|d| d.as_millis_f64()) (60)` — because the rename replaced
the field name inside the format string, where it was being quoted for the
reader rather than evaluated. Both policy branches, so every diagnostic of that
rule since the migration carried it. Fixed in manifest v0.1.13, and the
integration test now asserts no diagnostic contains `as_millis_f64`.

Gates: clippy clean, `just check-layer2-isolation`, `just test-all` — **485 +
291 + 189 + 144 = 1109 tests, 0 failed, 0 skipped**.

---

## W2 — mapper — DONE

Not all derivations. **Two of these are refusals**, and the refusals are the
safety-relevant half.

| axis | job |
|---|---|
| exclusion | treat an exclusive group as **one** schedulable entity, not N nodes |
| exclusion | **refuse** a per-thread reservation for a node whose exclusive paths span threads |
| `miss.action` | derive `deadline_policy`; enable `SCHED_FLAG_DL_OVERRUN` |
| `max_jitter` | a declared jitter bound argues against best-effort placement — promote, or **report why it cannot** |
| per-path cost | reservation runtime from the traversed path's cost, not the node's sum |
| QoS | emit `qos_overrides.*` where accepted; report endpoints that did not opt in |

### On the reservation refusal

CLAUDE.md's phase 60 F2 already records that **a reservation is per-thread**:
sweeping one across a ROS node's ~11 threads would turn an 8 ms/100 ms budget
into 88% of a CPU at admission control, so the thread-group leader is reserved
and siblings take `SCHED_FIFO`. A node whose paths are declared mutually
exclusive but run on different threads is exactly the case where per-node
reservation derivation is unsound. Refuse it and say why.

### On `SCHED_FLAG_DL_OVERRUN`

Wired since phase 60 and **always off**, because `deadline_policy` was never
authorable. Phase 67 makes it authorable. Note the neighbouring trap already
recorded: `SCHED_FLAG_RESET_ON_FORK` is set **only** for `SCHED_DEADLINE`;
setting it on `SCHED_FIFO` silently breaks the per-TID sweep, because
`sched_fork()` runs for thread creation too. Measured, and locked by a test.

### On what `miss.action` can actually deliver

Expected before starting, so that the finding is a confirmation rather than a
surprise:

| action | mechanism | expectation |
|---|---|---|
| `continue` | CBS throttles to next replenishment | native; already what happens |
| `skip_next` | needs the node to drop the next release | no kernel mechanism |
| `abort` | needs an interruptible callback | no `rclcpp` hook |

Detection is available on both paths — `SCHED_FLAG_DL_OVERRUN` raises `SIGXCPU`
under `SCHED_DEADLINE`; under `SCHED_FIFO` there is no kernel deadline at all
and a miss is observable only by measuring response time, which the interception
layer already does per invocation.

**Decision this wave must settle:** ship `skip_next`/`abort` as
declarations-of-intent (checked, reported, unenforced), or omit them until there
is a mechanism. A vocabulary where selecting `abort` silently gets `continue` is
worse than one that offers less.

### On QoS

`rclcpp` lists `Deadline` and `LivelinessLeaseDuration` as overridable kinds,
settable from outside as `qos_overrides.<topic>.<endpoint>.<policy>` parameters.
But the gate is on the node's side:

```cpp
/// Default constructor, no overrides allowed.
QosOverridingOptions() = default;
```

Opt-in by the node author, and most nodes do not. So: **derive always, apply
where accepted, report where it cannot be.** A node carrying a rate requirement
whose endpoints did not opt in is checkable but never enforceable, and saying so
beats emitting parameters that silently do nothing.

**Acceptance:** `check --sched --explain` names the provenance of every changed
decision — a node whose placement moved says which new fact moved it.

---

## W3 — verify on running systems — DONE

The wave that gates retirement. A checker agreeing with itself is not evidence.

**Status, arm by arm:**

| arm | result |
|---|---|
| `rt_workspace` | **green** — vocabulary end to end, covered by `just test-all` |
| `rt_av_demo` derivation | **green** — plan unchanged: brake 40, detector 39, lidar 38, same `chain_aware` drain provenance |
| `rt_av_demo` measured A/B | **green** — 498/498 → 0/1033, p99 280.1 → 13.4 ms |
| Autoware / golf cart at scale | **green for regression, useless for verification** — see below |

**The measured A/B ran** once `docker` group membership made `just setcap`
password-free. Steady state, first 200 samples discarded:

| arm | p50 | p99 | missed |
|---|---|---|---|
| RT off | 225.3 ms | 280.1 ms | **498/498** |
| `SCHED_FIFO` | 13.2 ms | 13.4 ms | **0/1033** |

The RT-on arm — the one that tests the derivation — matches the published
figures almost exactly (0/1039, p99 13.3 ms). The baseline is harsher than
published (498/498 against ~217–272/1013, p99 280 against 106.6), consistent
with load average 4.59 and other agents working on the same box. A worse
baseline reaching the same outcome makes the result stronger, not weaker.

**Scale verifies no regression and cannot verify the vocabulary.** The golf-cart
stack resolves **159 nodes in 0.83 s** with no new diagnostics — but it reports
`0 topics, 0 contracts-carrying endpoints, 0 tier(s)`, because it ships **no
contracts at all**. Every rule W1 and W2 added has nothing to act on there. The
110 warnings it does emit are pre-existing provenance and identity notes, none
from this phase. So the arm intended to exercise the exclusion diagnostic at
scale cannot: that needs a large contract set, which the Autoware contract
corpus would provide and this checkout does not have.

| fixture | what it proves | gate |
|---|---|---|
| `tests/fixtures/rt_workspace` | vocabulary end to end, `resolve` → `check` → `up` | contract parity under both parsers |
| `examples/rt_av_demo` (`just ab`, `sudo -E just ab3`) | the derivation still produces a working schedule | **reproduces the published 217 → 9 missed-frame result** |
| Autoware (144 nodes, 17 containers) | no regression at scale; the only case that exercises W1.c | resolves; validity diagnostic fires where co-location is real |
| golf cart / AutoSDV | the launch behind phases 61 and 64 | resolves and launches |

`rt_av_demo` is load-bearing. Its numbers are published in
`docs/reports/rt-mixed-criticality/` and came from measurement, so a derivation
change that quietly degrades scheduling surfaces as **frames**, not as a passing
test. `just ab` refuses rather than reports when the baseline meets its deadline
or the helper lacks `CAP_SYS_NICE` — keep that.

Two operational notes, both from prior findings:

- Run anything on the golf cart **jailed in its own `ROS_DOMAIN_ID`**; another
  agent works in that repo.
- `just setcap` after every build. Every colcon build copies the helpers into
  `install/` with fresh inodes, and a capability lives on the inode (#0015).

**Acceptance:** every row green, with the `rt_av_demo` numbers recorded in the
phase report rather than summarised — the report regenerates its figures from
the run, and hand-typed values are how a regression hides.

---

## W4 — retire — GATE MET, lint live, removal waits a release

Condition 3 (the A/B) is met. Condition 2 — *old and new spellings resolve to
identical models* — was run, **failed**, was fixed, and now passes for the right
reason. What follows is the record of that, because the failure is the more
instructive half.

`rt_workspace`'s chain was rewritten as the equivalent scope path and both
forms resolved with the same platform file. The derived schedules came out
**identical** — control_node 20/core 0, filter 39, sensor 38. Stopping there
would have retired `segments:` on a false pass. The provenance says why:

```
chain form:  derived(chain_aware: points_to_cmd segment drain 2/2) -> prio 39
             derived(chain_aware: points_to_cmd boundary RM period=10ms) -> prio 38

scope form:  derived(chain_aware: non-chain criticality=None budget_ms=5) -> prio 39
             derived(chain_aware: non-chain criticality=None budget_ms=10) -> prio 38
```

**`non-chain`.** With `chains:` removed the mapper finds no chain at all and
falls back to ranking nodes by budget, which on this three-node fixture happens
to produce the same two priorities. The agreement is a coincidence of the
fixture, not a property of the derivation — on any system where budget order
and drain order differ, retiring `segments:` would silently change the
schedule. The chain form also emits an `sched:override-inversion` warning that
the scope-path form does not, so the two are not equivalent in diagnostics
either.

### What was missing, and is now built

`contract-primitives.md`'s migration step 5 — *"Emit the derived routes for the
mapper; switch the mapper to read them"* — which the earlier waves had skipped.
`resolve_chains_derived` now builds `ResolvedChain`s from the routes
`check_scope_path_critical_path` already computes, and
`mapper_input_from_dump` prefers them, falling back to authored chains of a
name nothing derives so both spellings coexist through the window.

`tests/fixtures/contract_derived_chain` is `rt_workspace`'s system with
`chains:` and `segments:` removed entirely, and its provenance now reads

```
derived(chain_aware: points_to_cmd segment drain 2/2) -> prio 39
derived(chain_aware: points_to_cmd boundary RM period=10ms) -> prio 38
```

The `sched:override-inversion` warning, which the authored form emitted and the
scope-path form did not, now fires in both.

**The test asserts the provenance string, not the priorities**, and that is the
whole point of it: with the mapper falling back to budget ranking this
three-node system produced the *same two priorities by coincidence*, so a
numeric check passed while the derivation was not being used at all. A gate
that an accident of the fixture can satisfy is not a gate.

### Condition 1 and the deprecation lint

`derivable-chain` (Info, manifest v0.1.16) tells an author that an authored
route is derivable and shows the scope path that replaces it. Blast radius is
exactly one note per authored chain — `rt_workspace` and `rt_av_demo` — and
none on the migrated fixture.

Two deliberate silences in that lint, both because the replacement does not
exist yet:

- **`topics.<t>.rate_hz` is not deprecated.** The design lists it as derivable
  by propagation from the timer that starts the chain, but that propagation is
  **not implemented** — `inherited_rate` checks a stale legacy `input:` list,
  not rate derivation. Deprecating a field nothing would reconstruct is worse
  than leaving it alone.
- **A chain declaring `semantics: age` is skipped.** `PathDecl` has no
  `semantics` field (`contract-axes.md` §3.6), so migrating such a chain would
  lose a requirement.

### What remains

Condition 4 — one release shipped with the lint live — is calendar time, not
work. Removal of `chains:`/`segments:` follows it. Migrating the tree's own
contracts (condition 1) is now safe to do at any point and is deliberately
*not* done here: `rt_workspace` and `rt_av_demo` are the fixtures the published
measurements were taken on, and changing their spelling in the same phase that
changed the mapper would make a future regression harder to bisect.

- `chains:` / `segments:` — the derived route replaces them
- `topics.<t>.rate_hz` — propagates from the timer that starts the chain
- `EndpointProps.jitter` — superseded by the path/scope requirement

Deprecation lint first, naming the fact each is derivable from; removal a
release later. The manifest crate already has the pattern
(`check/src/rules/deprecated_unit_suffix.rs`), so this is a rule, not a
mechanism.

**Retirement gate — all four, not a majority:**

1. every fixture in the tree migrated to the new spellings
2. old and new spellings resolve to identical models
   (`scripts/compare_models.py`)
3. `rt_av_demo`'s A/B reproduces its published numbers under the new derivation
4. one release shipped with the deprecation lint live

Condition 2 is what makes the rest safe. While both spellings produce the same
model, retirement is **provably** lossless; if they ever diverge, the divergence
is a defect in the new path rather than a reason to keep the old one.

Phase 47 is the precedent for the removal itself — `record.json` went from
deprecated-with-a-warning to `error: unexpected argument`, a hard clap failure
rather than a silent misparse, and the dead code paths were deleted in the same
wave once nothing could reach them.

---

## What this phase does not do

- **No modes, no cross-chain synchronization or offset** — deferred by decision
  (`contract-axes.md` §3.3, §3.5). §3.5 in particular should wait for the
  derived route to be stable, since a synchronization constraint names two chain
  endpoints and would otherwise revive `segments:` under another name.
- **No FTTI.** Kept on the list because ISO 26262 requires it, not because it is
  optional. W2's QoS `lease_duration` is a fault-detection interval and W2's miss
  detection is the other half of the evidence, so by the time FTTI is written,
  detection may already exist and the new vocabulary is mostly the *reaction* and
  *safe state* halves.
- **No blocking model.** W1.c decides whether summing is valid; it does not
  compute what the blocking would be. Full response-time analysis under
  mutually-exclusive callback groups is a separate phase, and the published work
  reports **unbounded** worst-case response time for some chains under the
  default ROS 2 executor — which is a result about ROS 2, not about our
  arithmetic.
- **No performance claim.** This is about what an author writes and what a
  checker can prove.
