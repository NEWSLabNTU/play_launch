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

## Cross-domain evidence: criticality is always *net of mitigation*

Surveying how four domains actually assign integrity levels turns up one shared
structure, and it is not raw reachability:

| domain | standard | level derived from | **narrowed by** |
|---|---|---|---|
| vehicles | ISO 26262 | hazardous event, S × E × C | ASIL **decomposition** across sufficiently independent elements |
| drones | SORA (JARUS/EASA) | ground risk class + air risk class | **mitigations** M1–M3 lower the class; robustness = integrity + assurance |
| industrial robots | ISO 13849 / ISO 10218 | required risk reduction → Performance Level (PLd required by 10218) | **safety functions** — monitored stop, speed & separation, power & force limiting — "part of the robot **or** provided by a protective device" |
| any CPS | Simplex / runtime assurance | the complex controller is unverified | a **high-assurance controller + decision module** bounds it |

In every one, the integrity a component needs is **what its failure can cause
given the mitigations in place** — never what it can reach. The Simplex
formulation is the clearest: a high-performance controller "not required to
provide guarantees" is paired with a verified baseline controller and a
switching decision module. The complex stack does not have to be high-integrity
*because the architecture bounds it*.

This is also how real systems avoid the outcome that pure reachability
produces. Nobody develops an entire autonomy stack to ASIL D. They build a
small, simple, independent channel that can force a safe state, certify *that*
high, and let the complex pipeline sit lower. Jiao's "Safe IO Cell" — an
independent supervision domain that "checks commanded values against predefined
envelopes" and can "deassert motor enable or engage brakes regardless of
primary controller behavior" — is the same architecture in a robotics setting.

**The design below therefore propagates severity from the control action
upstream, attenuated at declared mitigation barriers.** Reachability sets the
ceiling; mitigation brings it down.

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

**R3 — Propagation follows every data edge; `state:` breaks cycles, not
severity.** An earlier draft stopped propagation at `state: true` edges. That
was wrong, and the error is instructive: `state: true` means "polled, does not
create a causal *cycle*" — it was designed for cycle-breaking, and reusing it
for safety-irrelevance is a semantic overload. A node reading a stale pose can
absolutely produce hazardous output. Severity therefore traverses state edges
too; `state:` is used only to terminate the walk, exactly as `causal-dag` uses
it today.

The tempting alternative — propagate only over `required: true` edges — is
worse, and dangerously so. `required` defaults to **false**, and the spec says
"an endpoint with no properties is causal and optional — the most common case".
A required-only rule would derive QM for nearly everything in the existing
corpus: a safety classification **failing open**, silently. Over-assignment
fails safe; that rule fails open. Take the over-assignment.

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

**R6 — A mitigation barrier attenuates severity upstream of it.** This is the
rule the cross-domain survey demands, and the one that keeps a real stack from
collapsing into a single ASIL D class.

```yaml
mitigations:
  emergency_stop:
    bounds: vehicle_fails_to_stop
    residual: ASIL_B          # what remains when this channel works
    channel: /safety/estop_cmd
```

Semantics: severity propagates upstream from the hazard's control action at
full strength until the walk reaches a node that the mitigation channel bounds;
from there upstream it continues at `residual`. The **mitigation channel itself
inherits the full severity** — it is now the thing that must not fail. That is
Simplex expressed in a contract: the high-assurance controller becomes the
ASIL D element and the complex pipeline drops to `residual`.

The barrier is only valid if the channel is **independent** of what it bounds —
the same disjoint-ancestor test as R4. A mitigation whose channel shares an
ancestor with the path it claims to bound is a common-cause failure, and the
checker rejects it naming the shared node. This is the single most valuable
check in the design, because in a hand-written safety case it is an assertion
nobody can mechanically verify.

`residual` is authored, not computed. Deriving it would require quantitative
risk reduction (SORA's robustness levels, ISO 13849's PFHd), which is HARA
territory and out of scope per the boundary below.

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

### The same system with a safety channel (R6)

`rt_av_demo` has no mitigation, which is why every chain node lands at ASIL D.
Add the architecture a real vehicle would have:

```yaml
mitigations:
  independent_estop:
    bounds: vehicle_fails_to_stop
    residual: ASIL_B
    channel: /safety/estop_cmd       # from a monitor reading wheel speed + range directly
```

| node | before R6 | after R6 | why |
|---|---|---|---|
| `estop_monitor` | — | **ASIL D** | it is now what must not fail |
| `brake_controller` | ASIL D | ASIL B | bounded by the channel |
| `obstacle_detector` | ASIL D | ASIL B | upstream of the barrier |
| `lidar_driver` | ASIL D | ASIL B | upstream of the barrier |
| everything else | QM | QM | reaches no control action |

Valid only if `estop_monitor` shares no ancestor with the chain it bounds. If
it took its range input from `obstacle_detector`, the checker rejects the
mitigation — a common-cause failure, mechanically detected.

This is the whole point of the rule: it moves one small node up and a whole
pipeline down, which is what real safety architecture does and what pure
reachability cannot express.

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

### The division of labour — and how W4 changes it

| role | source |
|---|---|
| **partition** — which class, and the barrier compression may never cross | criticality, derived from hazards |
| **order within a partition** | rate, deadline, chain position |
| **magnitude** — the budget enforced | measured cost |

Three inputs, three distinct roles. Today all three are muddled: criticality is
a meaningless label doing both partition and ordering work, and cost does not
exist.

**Superseded once phase 58 W3+W4 land.** `SCHED_DEADLINE` is GEDF + CBS, and
per `sched(7)` deadline threads "preempt any thread scheduled under one of the
other policies" — they carry **no priority number and no band**. So:

- the *partition* row evaporates for reserved nodes: band compression is a
  `SCHED_FIFO` artifact, and CBS gives every reserved task a bandwidth
  guarantee by construction, so criticality is not needed to decide who gets
  isolation;
- the *order* row is fully covered by W3's decomposed deadlines under EDF.

With (C, T, D) complete, classical real-time theory schedules the system with
no criticality input at all. **Criticality's remaining scheduling job is
overload arbitration** — when admission control rejects an infeasible set,
deciding who is refused a reservation. Timing facts cannot answer "who matters
less"; that is precisely the question they do not contain.

This is Vestal's actual mixed-criticality premise: criticality is a
**degradation policy**, not a priority input. Today we use it as a priority
input and implement no degradation, which is backwards. After W3/W4 it should
stop feeding the ordering and start feeding admission — see phase 58 W4.

A residual need survives in hybrid systems: a node with no declared cost cannot
be reserved and falls back to FIFO/OTHER, where the band and its barrier still
apply.

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

## Issues found by writing contracts against this design

Recorded because a design that reads as settled is harder to correct than one
that carries its own defects.

**Resolved by R3/R6 above:**

1. *Reachability is not causal necessity.* `map_loader → path_planner → … →
   brake_cmd` derives ASIL D, but the planner works without fresh tiles. The
   fix is not a cleverer propagation rule — it is R6: if a mitigation bounds the
   planning path, everything upstream drops to `residual`. Where no mitigation
   exists, ASIL D is the correct and uncomfortable answer that ISO 26262 gives
   too.
2. *`state:` was overloaded for safety propagation.* Corrected in R3.
3. *Everything becomes ASIL D.* True without mitigations, and then it is a fact
   about the architecture rather than an artifact of the method. R6 is how a
   real system narrows it.

**Open, and not resolved by this design:**

4. **Ownership conflict.** `hazards:` and `mitigations:` must be
   integrator-owned (root contract or overlay), matching the `chains:`
   precedent — a package provider cannot know system hazards. But today
   `criticality:` lives in `nodes.<name>`, inside *provider-shipped* sidecars.
   The migration path below keeps node-level criticality as an override, which
   puts an integrator-level safety judgment in a provider-level file: the same
   misplacement class this document exists to fix. Either the override moves to
   the overlay, or the inconsistency is accepted and documented.
5. **ASIL is not priority.** ASIL is development rigour plus an FFI obligation;
   priority is execution order. This design keeps them separate (partition from
   criticality, order from timing), but nothing *enforces* the separation, and
   "higher ASIL ⇒ higher priority" is a plausible-looking next step that would
   be wrong.
6. **A control action is assumed to be a published topic.** A watchdog whose
   safety role is *withholding* `motor_enable`, or a control action delivered by
   a service call, cannot be expressed.
7. **`severity_levels` must be system-global.** Declared per contract, two
   contracts can disagree with no merge rule.
8. **No mode conditioning.** A hazard that exists only in autonomous mode
   cannot be expressed; vocabulary v2 deferred mode conditioning, so a node
   critical only in one mode is over-assigned in all of them.

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
- Cross-domain mitigation structure:
  [SORA (EASA)](https://www.easa.europa.eu/en/domains/drones-air-mobility/operating-drone/specific-category-civil-drones/specific-operations-risk-assessment-sora) ·
  [ISO 10218-1:2025 robotics safety requirements](https://www.iso.org/obp/ui/en/#!iso:std:73933:en) ·
  [Industrial robot safety standards overview](https://www.evsint.com/industrial-robot-safety-standards-iso-10218-ce-marking-2026/)
- Simplex / runtime assurance:
  [The Black-Box Simplex Architecture](https://arxiv.org/pdf/2102.12981) ·
  [Neural Simplex Architecture](https://www3.cs.stonybrook.edu/~stoller/papers/nfm2020.pdf) ·
  [Mission-level runtime assurance for autonomous driving](https://arxiv.org/pdf/2606.06996)
- [Modular safety cases from assume-guarantee contracts](https://link.springer.com/chapter/10.1007/978-3-030-26250-1_3) ·
  [Pacti: assume-guarantee contracts](https://dl.acm.org/doi/full/10.1145/3704736)
