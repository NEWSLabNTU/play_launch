# Phase 59 — Timing vocabulary: units move onto the value

**Status:** 📋 planned
**Split from:** [Phase 58](./phase-58-scheduling-derivation.md), which surfaced
the problem but does not depend on the fix.
**Scope:** `ros-launch-manifest` schema + every contract and platform file.

## Decision

Time-valued fields carry their unit in the **value**, not the name:

```yaml
max_latency: 12ms        # was max_latency_ms: 12
budget: 8ms              # was budget_us: 8000
```

Existing `*_ms` / `*_us` names are accepted as deprecated aliases. Canonical
form is emitted; the old form is never written back.

## Why

### The error it prevents is a 1000x error that type-checks

`budget_us: 8` when the author meant 8 ms is off by three orders of magnitude,
and the value flows into a scheduling parameter — under Phase 58's W4, into a
reservation the kernel admits or rejects. Nothing in the schema can catch it:
both are valid `u64`.

That is the same species as the two defects Phase 57 found — a deadline used as
a cost, a `SCHED_OTHER` tier clamped into an RT band. In each case a *wrong*
value was **structurally representable**. A unit suffix makes this one
unrepresentable.

### It retires an inconsistency that was rationalised, not justified

An earlier draft of Phase 58 called the ms/µs split between the two files
deliberate: contracts being human-authored at millisecond granularity,
scheduling parameters needing sub-millisecond precision. That was a
justification invented for an inconsistency rather than a reason for it. With
units on values, each author writes what they mean and no reader converts.

### It is cheaper now than it will ever be again

Measured against the real corpus, not the schema. The full inventory has 14
timing fields on the contract side, but the contracts that actually exist use
four:

```
19  min_rate_hz     ← unchanged (rate, out of scope)
15  max_latency_ms  ← renamed
 5  rate_hz         ← unchanged (rate, out of scope)
 4  max_age_ms      ← renamed
```

So the real migration is **two field names**, mechanically, across ~75 Autoware
contracts plus a handful here. The platform side is cheaper still: v2 has *no*
timing fields at all, and the v1 fields are reachable only through the
deprecated TOML bridge.

Every contract written between now and then makes this more expensive. That is
the argument for doing it early, and it is the reason this is its own phase
rather than a Phase 58 work item — it should not wait on scheduling work it
does not depend on.

## The places

**Contract — platform-agnostic requirements** (`types/src/types.rs`), all
`f64` except `lifespan_ms`:

| struct | fields |
|---|---|
| `EndpointProps` | `min_rate_hz`, `max_rate_hz`, `jitter_ms`, `max_age_ms`, `max_transport_ms` |
| `SrvEndpointProps` | `max_response_ms` |
| `TopicDecl` | `rate_hz`, `max_transport_ms` |
| `QosDecl` | `lifespan_ms` (`u64`) |
| `PathDecl` | `max_latency_ms`, `tolerance_ms` |
| `Sync` | `max_interval_ms`, `timeout_ms` |
| `ChainDecl` | `max_latency_ms` |

**Platform / sched — machine facts** (`sched/src/types.rs`), all `u64`:

| struct | fields |
|---|---|
| `TierDef` | `deadline_us`, `period_us`, `budget_us`, `spin_period_us` |
| `TierPlatformSpec` | `deadline_us`, `budget_us`, `period_us`, `time_slice_us` |

**Platform v2** (`sched/src/platform.rs`): **none.** `PosixResources` is
`rt_priority_band` + `isolated_cpus`; `PosixOverride` is `priority` + `core` +
`sched_class`. Every platform-side timing field lives in a v1 structure.

Note also `f64` on the contract side against `u64` on the platform side — a
duration type unifies that too.

## Rename map

| today | planned | side |
|---|---|---|
| `max_latency_ms` | `max_latency` | contract (`PathDecl`, `ChainDecl`) |
| `max_age_ms` | `max_age` | contract (`EndpointProps`) |
| `max_transport_ms` | `max_transport` | contract (`EndpointProps`, `TopicDecl`) |
| `max_response_ms` | `max_response` | contract (`SrvEndpointProps`) |
| `max_interval_ms`, `timeout_ms` | `max_interval`, `timeout` | contract (`Sync`) |
| `jitter_ms`, `tolerance_ms` | `jitter`, `tolerance` | contract |
| `lifespan_ms` | `lifespan` | contract (`QosDecl`) |
| `deadline_us`, `period_us`, `budget_us` | `deadline`, `period`, `budget` | platform |
| `spin_period_us`, `time_slice_us` | `spin_period`, `time_slice` | platform |
| `min_rate_hz`, `max_rate_hz`, `rate_hz` | *unchanged* | contract |

## Options considered

| | approach | verdict |
|---|---|---|
| A | unit in the field name (today) | rejected: keeps the 1000x error representable |
| B | unit in value, flag day | rejected: ~75 Autoware contracts plus nano-ros on the same tag-pinned types |
| C | **unit in value, old names as deprecated aliases, emit canonical** | **chosen** |
| D | unit in value for new fields only | rejected: strictly increases inconsistency |

C needs no `version: 2` gate to *land*. The contract's `version:` field is
parsed and never branched on today (`yaml_u32(doc, "version").unwrap_or(1)`),
so the lever exists unimplemented — it becomes necessary only to *reject* the
old form, which is the sunset decision below.

## Rules

- **Grammar:** decimal number + unit from `ns | us | ms | s`. Decimals accepted
  on read (`1.5ms`) — forcing `1500us` is unit gymnastics.
- **Bare numbers rejected.** An unsuffixed value is an error, not a default.
  Guessing reintroduces exactly the ambiguity this removes.
- **Emit canonical only,** in the finest unit that keeps the value an integer,
  so `system_model.yaml` diffs stably and no float reaches a scheduling
  parameter.
- **Aliases warn** via a lint, following the `explicit-trigger` precedent that
  already nudges authors toward newer vocabulary.
- **State a sunset.** "Deprecated" without a removal version means "forever",
  and the alias burden is real: two spellings in the docs, the checker and the
  tests. The removal is what `version: 2` is for.
- `humantime-serde` is already a play_launch dependency
  (`diagnostics/diagnostic_data.rs`), so the pattern is not new here — though
  its grammar is not identical to the above.

## Scope boundary

Rate fields (`rate_hz`, `min_rate_hz`, `max_rate_hz`) are **out of scope**.
`50hz` is tempting and symmetric, but frequency is not a duration, it needs its
own unit set, and the error it prevents is far cheaper than a 1000x duration
slip.

The cost: `min_rate_hz: 50` will sit beside `max_latency: 12ms` in the same
block, so "unit in the name" does not fully disappear from the vocabulary.
That is a known, accepted wart — recorded so it is a decision rather than an
oversight.

## Non-goals

Moving fields between files (that is Phase 58's W1 for cost, and its W3 for
transport); changing any field's meaning; touching rate fields.
