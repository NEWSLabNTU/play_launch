# Phase 68 — contract consequences: analysis, mapper, verification, retirement

Status: **planned.** Depends on phase 67.

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

### W1.a — per-path attribution

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

**Acceptance — the expected value is known in advance:** on the fixture above,
`to_tracks` is charged **30 ms, not 45 ms**, and varying `to_masks` from 12 to
100 ms leaves it unchanged. Commit the fixture as a test, not a shell transcript.

Open (`contract-axes.md` §6.4): whether per-path cost then needs a `costs:`
section, or whether `measure` can attribute it once path identity exists.
Prefer the second; not yet verified possible.

### W1.b — sampling cost and sampling jitter

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

**Acceptance:** `points_to_cmd` expressed as a scope path reports **25 ms
against a 20 ms budget**, matching `chain-budget` on the same fixture. Write
this test first; the value has already been measured both ways.

### W1.c — groups from exclusion, and the validity diagnostic

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

### W1.d — consume the write-only fields

`jitter` (now the path-level requirement), `lifespan`, `max_response`, and the
sync window (`tolerance`, `max_interval`, `timeout`).

**Acceptance:** each has at least one rule that can fail because of it, and a
fixture that makes it fail.

---

## W2 — mapper

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

## W3 — verify on running systems

The wave that gates retirement. A checker agreeing with itself is not evidence.

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

## W4 — retire

Only after W3.

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
