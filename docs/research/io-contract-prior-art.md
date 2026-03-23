# I/O Contract Models: Prior Art Survey

**Status**: Research
**Date**: 2026-03-24

## Summary

Survey of how production systems and academic work model I/O contracts
for reactive/dataflow components. Each system separates trigger semantics
from data flow and requires explicit per-component timing bounds.

## Key Systems

### AUTOSAR Adaptive
- **Runnable entities** with typed Provide/Require ports
- **RTE Events**: `DataReceivedEvent` (data-driven), `TimingEvent` (periodic)
- **Parameter ports** distinct from sender-receiver data ports
- **TIMEX EventChains**: stimulus→response with LatencyTimingConstraint
- Multi-rate: implicit buffering (last-is-best or queued) between runnables

### Lingua Franca (Ptolemy II successor)
- **Reactions** with explicit `trigger set → effect set` declarations
- **Deadlines**: `deadline(5 ms)` with violation handler
- **Parameters**: immutable at instantiation vs **ports**: dynamic data
- **Deterministic by construction** via topological sort of reaction graph
- Cleanest contract model: `reaction(triggers) -> effects` is the gold standard

### AADL (Architecture Analysis & Design Language)
- **Flow specifications**: `flow source`, `flow sink`, `flow path` with latency ranges
- **Port types**: `data port` (sampled/latest), `event data port` (queued/triggering)
- **Dispatch protocol**: `periodic`, `aperiodic`, `sporadic`
- **End-to-end flow analysis** by composing per-component flow paths — tools compute
  total latency automatically. This is the gold standard for timing composition.

### ROS 2 Academic (Casini, Blass et al.)
- **Processing chains**: callback sequences through the ROS graph
- **Response-time analysis**: WCET + periods + chain structure → latency bounds
- **Activation patterns**: periodic (timer), sporadic (data-driven with min inter-arrival)
- Key gap: assume chain structure is known a priori (our manifest provides this)

### IEC 61499 (Function Blocks)
- **Event inputs/outputs** (trigger) separate from **data inputs/outputs** (payload)
- **WITH qualifiers**: `EVENT_IN WITH data1, data2` — declares which data is
  consumed with which trigger. Most explicit trigger↔data association.
- **ECC state machine**: internal state determines which algorithm runs per event

### Apache Beam / Flink
- **Watermark**: monotonic completeness assertion ("all data before time W processed")
- **Side inputs**: read-only lookup data, distinct from main data stream
- **Trigger policies**: `AfterWatermark`, `AfterCount(N)`, `AfterProcessingTime(d)`
- **Allowed lateness**: grace period for late-arriving data
- Watermark ≈ our frontier tracking from RCL interception

### Contract-Based Design (Benveniste et al.)
- **Assume-Guarantee**: contract C = (A, G) where A = input constraints, G = output promises
- **Refinement**: weaker assumption + stronger guarantee = valid upgrade
- **Composition**: composed contract assumption weakened (system self-provides)
- **Runtime monitoring**: checking traces against contracts = our interception + audit
- Theoretical foundation for everything we do

## Cross-System Primitive Comparison

| Concept        | AUTOSAR           | LF                   | AADL              | IEC 61499    | Beam                  | CBDe                |
|----------------|-------------------|----------------------|-------------------|--------------|-----------------------|---------------------|
| Trigger        | RTE Event         | reaction trigger set | dispatch protocol | event input  | trigger               | assumption          |
| Data vs config | param port vs S-R | parameter vs port    | convention        | WITH on INIT | side input            | degenerate contract |
| Timing bound   | TIMEX chain       | deadline             | flow latency      | —            | watermark             | guarantee           |
| Multi-rate     | buffering         | logical delay        | sampled/queued    | ECC          | watermark min         | channel contract    |
| Composition    | —                 | —                    | flow path sum     | —            | watermark propagation | contract algebra    |

## Validation of Our Design

Our four primitives map to the cross-system consensus:

| Our primitive | Industry standard equivalent |
|---------------|----------------------------|
| `inputs` with `role: trigger/state` | AUTOSAR param vs S-R port, IEC 61499 WITH, Beam side input |
| `trigger: on_arrival/all_ready/periodic` | AUTOSAR RTE Event, AADL dispatch protocol, LF reaction trigger |
| `latency_ms`, `rate_hz`, `freshness_ms` | AADL flow spec, AUTOSAR TIMEX, LF deadline, CBDe guarantee |
| `min_count` (readiness precondition) | IEC 61499 INIT event, Beam watermark hold |
| `consume` (SDF rate) | SDF token rate, Beam AfterCount(N), IEC 61499 ECC |

**Key insight from CBDe**: our manifest + runtime interception is essentially
contract-based runtime monitoring. We don't need formal verification now —
checking traces against declared contracts already provides enormous value.
The path to formalization (SMT checking, compositional verification) exists
when needed.

## Compositional Contract Rules

Detailed rules derived from the literature for our implementation.

### Composition

```
Series:    W(A → B) = W_A + W_B + comm_latency(A, B)
Parallel:  W(A ∥ B → C) = max(W_A, W_B) + W_C
Periodic:  W(timer → chain) = period + jitter + W_chain
Feedback:  Requires iterative fixed-point (not yet supported)
```

### Consistency Conditions

1. **Edge**: publisher latency + comm ≤ subscriber's assumed inter-arrival
2. **Scope**: scope.latency_ms ≥ critical_path(internal_graph)
3. **QoS**: pub.reliability ≥ sub.reliability ∧ pub.durability ≥ sub.durability

### Refinement

```
C' refines C  ⟺  A ⊇ A' (weaker assumption) ∧ G' ⊆ G (stronger guarantee)
```

Enables substitutability: faster node or more tolerant node is always valid.

### Runtime Monitoring

Monitor assumption and guarantee independently for 4-way diagnosis:
- (A ok, G ok) → nominal
- (A ok, G fail) → node bug
- (A fail, G ok) → node robust beyond contract
- (A fail, G fail) → upstream problem, not node's fault

### Empirical Derivation

From N traces: `guarantee = max(observed) × margin`, `assumption = min(observed) / margin`

## References

- Benveniste et al., "Contracts for System Design" (Foundations and Trends, 2018)
- de Alfaro & Henzinger, "Interface Automata" (POPL 2001)
- Henzinger & Matic, "Timed Interfaces" (EMSOFT 2006)
- Leucker & Schallhart, "A Brief Account of Runtime Verification" (JLAP 2009)
- Casini et al., "Response-Time Analysis of ROS 2 Processing Chains" (ECRTS 2019)
- Lohstroh et al., "Toward a Lingua Franca for Deterministic Concurrent Systems" (ACM TECS 2021)
- Akidau et al., "The Dataflow Model" (VLDB 2015)
- SAE AS5506C AADL Standard
- IEC 61499 Function Blocks Standard
- AUTOSAR TIMEX R22-11
