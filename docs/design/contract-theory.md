# Contract Theory for Launch Manifests

The `io:` blocks in launch manifests form an assume-guarantee contract
system that enables compositional reasoning and future formal verification.

See `docs/research/io-contract-prior-art.md` for the full survey of
related systems (AUTOSAR TIMEX, AADL flow specs, CBDe, Lingua Franca,
IEC 61499, Apache Beam).

## Contract Structure

Each node or scope has a contract C = (A, G):
- **Assumption (A)**: what the node expects of its inputs (rate, jitter)
- **Guarantee (G)**: what the node promises about its outputs (latency,
  rate, freshness) — valid only when assumptions hold

In the manifest, the assumption is implicit from input annotations
(`role`, `min_count`, expected rate via upstream contracts) and the
guarantee is the timing section (`latency_ms`, `rate_hz`, etc.).

## Composition Rules

For computing scope contracts from node contracts:

```
Series:    W(A → B) = W_A + W_B + comm_latency(A, B)
Parallel:  W(A ∥ B → C) = max(W_A, W_B) + W_C
Periodic:  W(timer → chain) = period + jitter + W_chain
Feedback:  Requires iterative fixed-point (not yet supported)
```

A scope's `latency_ms` must be ≥ the critical path through its
internal graph (sum of node latencies along the longest path).

**Jitter propagation**: If node A has output jitter J_A, node B sees
input jitter of J_A, increasing B's effective worst-case:
```
W_effective(B) = W_B + J_A
```
For a first implementation, the simple additive bound (without jitter
refinement) is sound but pessimistic.

## Consistency Conditions

Three conditions checkable statically from the manifest:

**1. Edge compatibility**: For each topic connecting node i → node j,
node i's worst-case output latency must not starve node j:
```
node_i.guarantee.latency + comm_latency ≤ node_j.assumption.max_input_delay
```

**2. Scope abstraction validity**: The scope's guarantee must be
sound with respect to its children:
```
scope.latency_ms ≥ critical_path_latency(internal_graph)
scope.assumption ⊇ ∪ { node.assumption : node is boundary input }
```

**3. QoS compatibility**: Publisher QoS must satisfy subscriber QoS
(reliability, durability, deadline). This is the base layer of contract
checking — necessary but not sufficient.

## Refinement

A new version of a node or scope refines its contract if it assumes
less and guarantees more:
```
C' refines C  ⟺  A ⊇ A'  ∧  G' ⊆ G
```

This enables substitutability: a faster node (lower latency guarantee)
or a more tolerant node (wider input rate assumption) is always a valid
upgrade. Backwards-compatible manifest changes follow the same rule:
- Adding a topic: safe (weaker assumption on consumers)
- Removing a topic: breaking
- Tightening a latency guarantee: safe
- Loosening a latency guarantee: breaking
- Widening an input rate range: safe (weaker assumption)

## Runtime Monitoring

Runtime monitoring checks contracts against observed traces. The monitor
tracks both assumption and guarantee violations independently:

| Assumption | Guarantee | Diagnosis |
|------------|-----------|-----------|
| satisfied | satisfied | Nominal |
| satisfied | violated | **Contract violation** — node is buggy or under-provisioned |
| violated | satisfied | Environment violation, node is robust beyond its contract |
| violated | violated | Environment violation, node affected — not the node's fault |

A guarantee violation with satisfied assumptions is a real bug. A
guarantee violation with violated assumptions is an environmental issue
(upstream node misbehaving, not the node's fault).

### Monitoring vs Static Verification

| Property        | Static Verification                  | Runtime Monitoring               |
|-----------------|--------------------------------------|----------------------------------|
| Coverage        | All executions                       | Observed executions only         |
| Guarantee       | Sound & complete (if model accurate) | Sound for observed trace         |
| False positives | Model inaccuracies                   | None (checks real system)        |
| False negatives | None (if model complete)             | Possible (unobserved violations) |
| Cost            | High (state space explosion)         | Low (linear in trace length)     |

Runtime monitoring is sufficient for our use case. The path to static
verification (SMT solvers, schedulability analysis) exists when needed.

## Empirical Contract Derivation

Capture mode derives contracts automatically from observed traces:
```
guarantee.latency_ms = max(observed_latencies) × safety_margin
assume.min_inter_arrival = min(observed_arrivals) / safety_margin
```

After N observations without violation, confidence bound:
```
P(violation) ≤ 1 - (1 - α)^(1/N)
```
For N=1000 observations at 99% confidence: P(violation) ≤ 0.0046.

This bridges the gap between "no contract" and "manually authored
contract" — capture mode provides a starting point that can be refined.

## QoS as Base Contract Layer

ROS 2 QoS compatibility is a special case of contract consistency:

| QoS Property                          | Contract Analog                                |
|---------------------------------------|------------------------------------------------|
| Reliability (RELIABLE/BEST_EFFORT)    | Guarantee strength (loss-free vs lossy)        |
| Durability (TRANSIENT_LOCAL/VOLATILE) | Assumption about subscriber arrival time       |
| Deadline                              | Timing guarantee (publisher commits to period) |
| Liveliness                            | Liveness guarantee (heartbeat contract)        |
| History depth                         | Buffer capacity assumption                     |

The QoS compatibility matrix is a contract compatibility check:
```
Compatible(pub_qos, sub_qos) ⟺
  pub.reliability ≥ sub.reliability  ∧
  pub.durability ≥ sub.durability    ∧
  pub.deadline ≤ sub.deadline
```

Timing contracts extend QoS: `pub.observed_period ≤ sub.assumed_period`
ensures the subscriber's processing pipeline isn't starved.

## References

- Benveniste et al., "Contracts for System Design" (Foundations and Trends, 2018)
- de Alfaro & Henzinger, "Interface Automata" (POPL 2001)
- Henzinger & Matic, "Timed Interfaces" (EMSOFT 2006)
- Leucker & Schallhart, "A Brief Account of Runtime Verification" (JLAP 2009)
- Casini et al., "Response-Time Analysis of ROS 2 Processing Chains" (ECRTS 2019)
- SAE AS5506C (AADL) — flow latency analysis
- AUTOSAR TIMEX R22-11 — event chain timing constraints
