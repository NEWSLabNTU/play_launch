# Criticality from hazards — deriving scheduling context from outcomes

Status: Draft (2026-08-07). Design of record for replacing the contract's
`criticality:` label with a derivation from declared hazards.
Feeds: [`phase-58`](../roadmap/phase-58-scheduling-derivation.md) (cost,
decomposition, reservations).
Related: [`scheduling-derivation-prior-art.md`](../research/scheduling-derivation-prior-art.md),
[`io-contract-prior-art.md`](../research/io-contract-prior-art.md).

## Problem

`nodes.<name>.criticality` is a free-text field parsed into a three-value enum:

```rust
pub enum Criticality { Low, Medium, High }
```

Its **only semantic content is the variant order**. `High` means "sorts above
`Medium`" and nothing else — no hazard, no consequence, no failure mode. It is
a bare label wearing the costume of a safety classification. Three specific
consequences:

1. **No meaning.** Nothing defines what makes a node `high`. Two engineers will
   not agree, and neither can be shown wrong.
2. **Fixed, arbitrary levels.** Teams working to ISO 26262, IEC 61508 or
   DO-178C have their own scales (ASIL A–D + QM, SIL 1–4, DAL A–E). Ours is
   three words chosen by us.
3. **Unknown values fail open.** `parse_criticality` maps anything outside
   `high|medium|low` to `None` at `debug!` level. A typo silently demotes a
   safety declaration to "no signal".

### The deeper issue: a missing ordering relationship

What the mapper actually needs from criticality is an **ordering** that timing
facts cannot supply. The Autoware study established that need empirically:
`vehicle_cmd_gate` (which emits the vehicle command) and a pure pipeline filter
both declare **10 Hz**, so "a rate-only or bucket-by-rate mapper cannot tell
these apart".

But criticality does **two** jobs, and an ordering only covers one. From the
`chain_aware` mapper's band-compression rule:

> adjacent ranks collapse to equal priority … **never across the chain/non-chain
> divide or a criticality bucket boundary**

An ordering says "A above B". That barrier says "A and B may **never be
merged**" — an equivalence-class constraint, not an order. It matters exactly
at the boundary, where the lowest safety node and the highest best-effort node
are adjacent and would otherwise collapse to the same priority.

So a replacement must supply **order + classes**, and both need a source with
meaning.

## What the standards do

Every safety standard answers this the same way: **criticality is never a
property of a component.** It is derived from an outcome and *allocated*
inward.

| | derives criticality from | allocates to components by |
|---|---|---|
| **ISO 26262** | a hazardous event: Severity × Exposure × Controllability (S3/E4/C3 → ASIL D; any S0/E0/C0 → QM) | the Functional Safety Concept; decomposition across independent elements |
| **STPA** (Leveson) | Losses → Hazards → Unsafe Control Actions | safety constraints on control actions |
| **AADL EMV2** | a `Hazards` property on the architecture model | OSATE plug-ins auto-generate FHA / FMEA / fault trees |
| **Contract-based safety** | safety goals as assume-guarantee contracts | modular safety cases composed from component contracts |

STPA deserves particular weight here. It is displacing FTA/FMEA for autonomous
vehicles because those are "grounded in reliability theory and designed to
prevent **component failure** accidents", whereas STPA addresses "**component
interaction** accidents caused by design control flaws or unsafe interaction
among operational components".

**A missed deadline is precisely a component-interaction accident.** Every node
computes the right answer; the composition delivers it too late. Component-failure
methods cannot express that. This is a strong argument that STPA's vocabulary —
outcomes and control actions — is the right one for a *scheduling* contract
specifically.

## Proposal

Declare the **outcome**, link it to the **control action**, derive the rest.

```yaml
# contract — reviewable by a safety engineer, product owner or regulator
hazards:
  vehicle_fails_to_stop:
    severity: ASIL_D            # from the team's HARA; we consume, never compute
    ref: "HARA-2026-014"        # traceability to the real analysis
    mitigated_by: /safety/brake_cmd     # the control action that prevents it

external_topics:
  /safety/brake_cmd: { side: sub }      # already means "consumer is external"
```

A node's criticality is then **derived**: the max severity over every hazard
whose control action it can causally reach. Nothing reaching a control action
is QM.

### Why the boundary is the right place

- **Literal meaning.** `ASIL_D` because S3/E4/C3 on "vehicle fails to stop" —
  not because someone typed `high`.
- **Few declarations.** One per system output rather than one per node: a
  handful against 100+ in Autoware.
- **Bidirectional traceability.** From a hazard, which nodes implement its
  mitigation. From a node, *why* it is critical. Neither is answerable today.
- **Checkability.** A hazard whose `mitigated_by` topic has no publisher is a
  finding. A node claiming criticality that reaches no control action is a
  contradiction. Both are undetectable today.

### Open vocabulary

The severity scale is **declared**, not baked in:

```yaml
severity_levels: [QM, ASIL_A, ASIL_B, ASIL_C, ASIL_D]   # ordered, ascending
```

Default stays `[low, medium, high]` so nothing breaks. This answers the
"many ways to define criticality" problem by not choosing one, makes the
ordering explicit rather than implied by enum variant order, and gives the
equivalence classes the compression barrier needs. A value outside the declared
vocabulary is an **error**, not a silent `None`.

## Composition rules

**R1 — Inheritance is MAX, never sum.** An element implementing requirements
from several safety goals is developed to the highest ASIL. A node reaching two
hazards takes the max. Consistent with `ResolvedChain.criticality`, already
documented as "max over the chain's member nodes".

**R2 — No decay with distance.** A sensor five hops upstream of the brake is
still ASIL D. The standard has no attenuation, and hop-count decay would be a
fabrication. Stated explicitly because it is the tempting shortcut.

**R3 — Propagation follows causal edges only.** `state: true` breaks it —
already the `causal-dag` rule's semantics. A node feeding only a buffered/state
input does not causally drive the control action.

**R4 — Decomposition may LOWER, given verified independence.** ISO 26262
permits ASIL D → B(D) + B(D), or D(D) + QM(D), across "sufficiently independent
elements". Independence means **disjoint ancestor sets**, and we own the graph:

```yaml
hazards:
  vehicle_fails_to_stop:
    severity: ASIL_D
    mitigated_by: [/safety/brake_cmd, /safety/emergency_stop]
```

If the two topics' ancestor sets are disjoint, each path carries ASIL B(D). If
they share `lidar_driver`, the decomposition is invalid and the checker says so,
naming the common ancestor. **This check exists in no tool we have, and is
normally a manual argument in a safety case.**

**R5 — Unreachable is QM**, but a node the author declared critical that
reaches no control action is a *finding*, never a silent downgrade.

## Worked example — `rt_av_demo`

Real graph:
`lidar_driver → /safety/scan → obstacle_detector → /safety/obstacles →
brake_controller → /safety/brake_cmd →` external.

One hazard declaration (above) derives:

| node | reaches a control action? | derived |
|---|---|---|
| `brake_controller` | publishes it | **ASIL D** |
| `obstacle_detector` | yes, causally | **ASIL D** |
| `lidar_driver` | yes, causally | **ASIL D** |
| `path_planner` | `/planning/path` → nothing external | QM |
| `map_loader`, `telemetry_logger`, 4 × tools | nothing external | QM |

This **reproduces the hand-written `criticality: high` on exactly those three
nodes and nothing on the rest**, from one declaration instead of three, with a
reason attached. Reproducing the existing human judgment is the test a
derivation must pass before it is trustworthy; it passes on the case we have.

## Deriving the scheduling context

ISO 26262 **requires** freedom from interference between components of
different ASIL sharing a controller. Temporal FFI is achieved through **OS
execution budgets** — the standard names "improper scheduling or budget
allocation" as a temporal failure mode.

```
hazard severity (ASIL D)
  └─ freedom from interference required vs. lower-ASIL components
      └─ temporal FFI ⇒ enforced execution budget
          └─ SCHED_DEADLINE reservation
             runtime  ← measured cost       (phase 58 W1/W2)
             period   ← 1 / min_rate_hz     (contract)
             deadline ← decomposed budget   (phase 58 W3)
```

QM nodes get `SCHED_OTHER` and the residual: no reservation, no guarantee.

This changes the standing of phase 58's W4. Reservations stop being "nice
isolation" and become **the mechanism the standard asks for**. Fixed priority
*orders* tasks; it does not *bound* them. Under `SCHED_FIFO` an ASIL D node
that overruns starves other ASIL D nodes, and the RT class as a whole is capped
by `sched_rt_runtime_us` — which Phase 57 measured as best-effort losing
26–37% of its throughput. That is temporal interference; it merely happened to
point in the benign direction.

### The division of labour

| role | source |
|---|---|
| **partition** — which class, and the barrier compression may never cross | criticality, derived from hazards |
| **order within a partition** | rate, deadline, chain position |
| **magnitude** — the budget enforced | measured cost |

Three inputs, three distinct roles. Today all three are muddled: criticality is
a meaningless label doing both partition and ordering work, and cost does not
exist.

## Scope boundary

**Do not own the hazard analysis — reference it.** `severity` + `ref` is
enough. The moment this computes S/E/C it becomes a HARA tool, which is not a
launch manifest's job and would be a poor one.

What the manifest is uniquely good at is the **linkage**: it owns the graph, so
it alone can answer "which nodes does this hazard depend on". Nothing else in
the toolchain can.

## Risks and limits

**Derivation over an incomplete graph silently misclassifies.** The Autoware
study found `/planning/trajectory` with five declared subscribers and **zero
declared publishers**. A node whose only route to a control action crosses that
hole derives as QM — a safety node silently demoted. Mitigation is the same
rule the chain discussion reached: derive a *proposal*, require confirmation,
and make "claims criticality but reaches nothing" a finding. Failing loud
matters most when the thing failing is a safety classification.

**We do not achieve ISO 26262 FFI, and must not claim it.** Reservations give
temporal isolation against CPU contention. They do not isolate cache, memory
bandwidth, or DDS internals. [Jiao (2026)](https://arxiv.org/html/2605.03641),
on mixed-criticality robotics, reaches FFI with a static partitioning
hypervisor (Jailhouse) — dedicated cores, memory and devices per partition,
cutting jitter σ from 12.58 µs to 1.95 µs. We operate one tier down, at OS
scheduling. The honest claim is: we provide the temporal-budget half and make
the ASIL allocation machine-checkable.

**This is not a safety case.** It is traceability plumbing that a safety case
could cite.

## Migration

Node-level `criticality:` stays, as an **override** on the derived value. Two
reasons: ~75 Autoware contracts use it today, and some nodes are critical for
reasons outside the graph — a watchdog acting out-of-band, a node whose failure
is caught by something we do not model. Derivation proposes; the author pins.

Fix regardless of whether the rest lands:

1. Unknown criticality is silently ignored at `debug!` — a safety declaration
   failing open. Make it an error.
2. Nothing validates criticality against structure.

## Open questions

1. Does `severity` belong on the hazard only, or may a control action carry its
   own (a topic that is ASIL D regardless of which hazard cites it)?
2. How are service and action endpoints handled? Control actions today are
   topics; a service call can equally be one.
3. Should `mitigated_by` accept a *path* rather than a topic, for a node whose
   safety role is internal (a watchdog that acts by not-publishing)?
4. Is QM the right floor, or is "no declaration" distinct from "declared QM"?
   The `chain-sampling-feasibility` precedent says absent and zero are
   different answers.

## References

- [ISO 26262 ASIL determination (S/E/C)](https://www.perforce.com/blog/qac/what-is-iso-26262) ·
  [Automotive Safety Integrity Level](https://en.wikipedia.org/wiki/Automotive_Safety_Integrity_Level) ·
  [Automated ASIL allocation and decomposition](https://www.sae.org/publications/technical-papers/content/07-11-02-0011/)
- [STPA for fully automated driving architectures](https://www.sciencedirect.com/science/article/pii/S1877705817312109) ·
  [Comparison of hazard analysis methods for AVs](https://ieeexplore.ieee.org/document/8916932/)
- [AADL Safety Analysis with Error Model V2 (OSATE)](https://github.com/osate/osate2/blob/master/emv2/org.osate.aadl2.errormodel.help/markdown/safetyanalysis.md) ·
  [AADL fault modeling within an ARP4761 safety assessment](https://apps.dtic.mil/sti/tr/pdf/ADA610294.pdf)
- [Freedom from interference — temporal](https://piembsystech.com/freedom-from-interference-iso-26262/) ·
  [Jiao: mixed-criticality robotics isolation](https://arxiv.org/html/2605.03641)
- [Modular safety cases from assume-guarantee contracts](https://link.springer.com/chapter/10.1007/978-3-030-26250-1_3) ·
  [Pacti: assume-guarantee contracts](https://dl.acm.org/doi/full/10.1145/3704736)
