# Contract Theory for Launch Manifests

Formal foundations for the manifest's contract system. Endpoint
properties (`min_rate_hz`, `state`, `required`), causal `paths:`,
and topic `rate_hz`/`drop` form an assume-guarantee contract system
that enables compositional verification.

See `docs/research/io-contract-prior-art.md` for the full survey of
related systems (AUTOSAR TIMEX, AADL flow specs, CBDe, Lingua Franca).

## Contract Structure

The manifest defines contracts at three levels:

| Level     | Manifest elements                            | Assumption ($A$)                                            | Guarantee ($G$)                                    |
|-----------|----------------------------------------------|-------------------------------------------------------------|----------------------------------------------------|
| **Topic** | `rate_hz`, `drop` on topic                   | Publisher `min_rate_hz` $\geq$ `rate_hz`                    | Channel runs at `rate_hz`, drops $\leq$ `drop`    |
| **Node**  | `paths:` on node, endpoint properties        | Sub endpoints receive per `state`, `required`, `min_rate_hz` | `max_latency_ms`, `min_latency_ms`, `max_age_ms`, `drop` per path |
| **Scope** | `paths:` on scope                            | Imports are wired by parent via topics                      | `max_latency_ms`, `max_age_ms` (E2E)                  |

These three levels compose hierarchically:
- Topic properties constrain the channels between nodes.
- Node paths describe per-node computation timing.
- Scope paths abstract the internal graph into an E2E budget.

### Formal Definition

A contract is a pair $C = (A, G)$ where:
- $A$ (assumption): constraints on inputs that must hold for the
  guarantee to be valid
- $G$ (guarantee): constraints on outputs that the component promises

**Satisfaction**: Component $M$ satisfies contract $C$ iff:

$$M \cap A \subseteq G$$

Whenever the assumption holds on the inputs, the guarantee holds on
the outputs.

In the manifest:

**Node assumption** — derived from `sub:` endpoint properties:
- `min_rate_hz` on sub endpoint: input must arrive at least this fast
- `max_rate_hz` on sub endpoint: input must not exceed this rate
- `state: true`: endpoint is read-latest (not a trigger)
- `required: true`: must receive at least once before operational
- Upstream topic `rate_hz`: expected channel rate

**Node guarantee** — from `paths:` and `pub:` endpoint properties:
- `paths.*.max_latency_ms`: max trigger-to-output time
- `paths.*.max_age_ms`: max data age from original source
- `paths.*.min_latency_ms`: best-case latency (anomaly detection)
- `paths.*.drop`: max missed outputs per input window
- `pub.min_rate_hz`: output rate floor
- `pub.jitter_ms`: max deviation from ideal period

**Scope assumption** — the scope's `imports:`: subscriber endpoints
that the parent must wire via topics.

**Scope guarantee** — the scope's `paths.*.max_latency_ms` (E2E budget)
and optionally `paths.*.max_age_ms` (data freshness from source).

## Composition Rules

Scope paths compose from internal node/scope paths. The composition
follows the dataflow graph topology.

### Series (pipeline)

Nodes $A \to B \to C$ connected by topics with transport latency $L(t)$:

$$L_{\max}(A \to B \to C) = L_{\max}(A) + L(t_{AB}) + L_{\max}(B) + L(t_{BC}) + L_{\max}(C)$$
$$L_{\min}(A \to B \to C) = L_{\min}(A) + L(t_{AB}) + L_{\min}(B) + L(t_{BC}) + L_{\min}(C)$$

Transport latency $L(t)$ is modeled but typically 0 for same-machine.

Drop (log-sum approximation):
$$\ln(1 - D_{\text{chain}}) = \sum_i \ln(1 - D_i)$$

Age: $A_{\max}(C) = A_{\max}(A) + L(t_{AB}) + L_{\max}(B) + L(t_{BC}) + L_{\max}(C)$

### Parallel (fork-join)

Two branches merging at a fusion node $C$:

$$L_{\max} = \max(L_{\max}(A), L_{\max}(B)) + L_{\max}(C)$$
$$L_{\min} = \max(L_{\min}(A), L_{\min}(B)) + L_{\min}(C)$$

```
      ┌→ A (W_A) →┐
in →  │            ├→ C (W_C)
      └→ B (W_B) →┘
```

Rate: $R = \min(R_A, R_B)$

Drop: all drop causes (upstream propagation, correlation mismatch,
computation failure) are combined into the node's single `drop:` value.

Age: $A_{\max} = \max(A_{\max}(A), A_{\max}(B)) + L_{\max}(C)$

### Periodic (timer-driven)

A timer-driven node with period $P$ and jitter $J$:

$$L_{\max}(\text{through periodic}) = L_{\max}(\text{upstream}) + P + J + C$$
$$L_{\min}(\text{through periodic}) = L_{\min}(\text{upstream}) + C$$

Best case: timer fires right as state updates (zero wait).
Worst case: state arrives just after timer, waits full period.

Rate through periodic: $R_{\text{out}} = 1000 / P$ (independent
of upstream). The path's `max_age_ms` constrains how old the
original source data can be at the output.

### Scope budget

A scope's `paths.*.max_latency_ms` must be $\geq$ the critical path
through its internal dataflow graph:

$$L_{\max}(\sigma) \geq \mathrm{longestPath}(\text{imports} \to \text{exports})$$

The critical path is the longest-weight path in a DAG — computable
in $O(|V|+|E|)$ via topological sort + dynamic programming.

## Data Age

The age of output $o$ is the elapsed time since the original source
data was created:

$$\text{age}(o) = t_{\text{pub}}(o) - t_{\text{creation}}(\text{source})$$

**Static computation**: Sum max latencies along the causal chain:

$$A_{\max}(o) = \sum_{\text{chain}} L_{\max}(p_i) + \sum_{\text{chain}} L(t_j)$$

For multi-input paths (barrier), the age is the maximum of all input
ages — the output is as stale as its oldest input:

$$A_{\max}(o) = \max_{i \in I(p)} A_{\max}(\text{input}_i) + L_{\max}(p)$$

The checker traces backward automatically through the manifest
topology to find the source. No explicit source declaration needed.

**`max_age_ms`**: Optional field on paths (node or scope). Allowed
on all scopes, not just top-level. The static checker verifies:

$$\text{declared } A_{\max} \geq \sum L_{\max}(p_i) + \sum L(t_j)$$

**Runtime measurement** (see `docs/research/caret-analysis.md`):

1. **Static bound only**: Verify `max_age_ms` against composition.
2. **`header.stamp`**: For stamp-copying chains, measure
   $t_{\text{pub}} - \text{header.stamp}$.
3. **Bracketing + backward trace**: Trace through interception data.
4. **Embedded tracing (future)**: Inject source timestamp via
   LD_PRELOAD.

## Consistency Conditions

Nine conditions checkable statically from the manifest.

### 1. Topic wiring completeness

Every node endpoint in a `paths:` entry must be wired by a topic in
the same scope or an ancestor scope.

### 2. Scope max budget validity

$$L_{\max}(\sigma) \geq \mathrm{criticalPath}_{\max}(\text{G})$$

### 3. Scope min bound validity (if declared)

$$L_{\min}(\sigma) \leq \mathrm{criticalPath}_{\min}(\text{G})$$

### 4. Age budget validity (if declared)

$$A_{\max}(\sigma) \geq \sum L_{\max}(p_i) + \sum L(t_j)$$

along the causal chain from source to scope export.

### 5. QoS compatibility

$$\forall (\text{pub}, \text{sub}) \text{ on same topic}: \quad \text{pub.reliability} \geq \text{sub.reliability} \;\wedge\; \text{pub.durability} \geq \text{sub.durability}$$

### 6. Rate hierarchy

$$R_{\min}(\text{pub}) \geq R(\text{topic}) \geq \max(R_{\min}(\text{sub}))$$

### 7. Endpoint uniqueness

Names unique per node across `pub:`, `sub:`, `srv:`, `cli:`.

### 8. Causal DAG

The causal path graph (excluding state edges) must be acyclic.

### 9. Rate chain feasibility

Scope export rate must be achievable from upstream: series preserves
rate, barrier takes min, periodic overrides to timer rate.

## Runtime Monitoring

Runtime monitoring checks contracts against observed traces from the
Phase 29 RCL interception infrastructure. The monitor tracks both
assumption and guarantee violations independently:

| Assumption | Guarantee | Diagnosis                                          |
|------------|-----------|----------------------------------------------------|
| satisfied  | satisfied | Nominal                                            |
| satisfied  | violated  | **Node bug** — computation exceeds declared bound  |
| violated   | satisfied | Environment violation, node robust beyond contract |
| violated   | violated  | Upstream problem — not this node's fault           |

### What the monitor checks

| Manifest field                 | Monitor logic                                                | Data source             |
|--------------------------------|--------------------------------------------------------------|-------------------------|
| `paths.*.max_latency_ms`       | $t_{\text{pub}} - t_{\text{take}}(\text{trigger}) \leq L$    | Interception timestamps |
| `paths.*.min_latency_ms`       | $L_{\text{actual}} \geq L_{\min}$ (anomaly if below)         | Interception timestamps |
| `paths.*.max_age_ms`           | $t_{\text{pub}} - t_{\text{creation}}(\text{source}) \leq A$ | Static or header.stamp  |
| `paths.*.correlation`          | $\max(\text{stamp}_i) - \min(\text{stamp}_i) \leq \tau$      | Interception            |
| topic `rate_hz`                | $t_{\text{pub}}[n] - t_{\text{pub}}[n-1] \approx 1/R$        | Stats plugin            |
| endpoint `min_rate_hz`         | actual rate $\geq R_{\min}$                                  | Stats plugin            |
| endpoint `max_rate_hz`         | actual rate $\leq R_{\max}$                                  | Stats plugin            |
| endpoint `jitter_ms`           | $\lvert t[n] - t[n-1] - P \rvert \leq J$                    | Stats plugin            |
| `paths.*.drop` ($N/W$)         | $\text{drops in window } W \leq N$                           | Stats plugin            |
| `paths.*.drop.max_consecutive` | $\text{consecutive drops} \leq K$                            | Stats plugin            |
| topic `drop` ($N/W$)           | $\text{transport drops in window } W \leq N$                 | Stats plugin            |
| srv endpoint `max_latency_ms`  | $t_{\text{response}} - t_{\text{request}} \leq L$            | Interception            |

### Monitoring vs Static Verification

| Property        | Static Verification       | Runtime Monitoring        |
|-----------------|---------------------------|---------------------------|
| Coverage        | All executions            | Observed traces only      |
| Guarantee       | Sound (if model accurate) | Sound for observed trace  |
| False positives | Model inaccuracies        | None (checks real system) |
| False negatives | None (if model complete)  | Possible (unobserved)     |
| Cost            | High (state space)        | Low (linear in trace)     |

Runtime monitoring is sufficient for Phase 31. The path to static
verification exists when needed (see `docs/design/contract-verification.md`).

## Empirical Contract Derivation

Capture mode (`--save-manifest-dir`) derives contracts from observed
traces:

$$\hat{G}_L = \max(\text{observed latencies}) \times \alpha$$

$$\hat{A}_R = \min(\text{observed inter-arrivals}) / \alpha$$

where $\alpha > 1$ is the safety margin.

After $N$ observations without violation, at confidence level $c$:

$$P(\text{violation per trial}) \leq 1 - (1 - c)^{1/N}$$

For $N = 1000$ at $c = 0.99$: $P(\text{violation}) \leq 0.0046$.

Capture provides a starting point. Users refine manually or tighten
margins over time.

## References

- Benveniste et al., ["Contracts for System Design"](https://doi.org/10.1561/2500000017) (Foundations and Trends in EDA, 2018)
- de Alfaro & Henzinger, ["Interface Automata"](https://doi.org/10.1145/366927.366984) (POPL 2001)
- Henzinger & Matic, ["Timed Interfaces"](https://doi.org/10.1145/1176887.1176896) (EMSOFT 2006)
- Leucker & Schallhart, ["A Brief Account of Runtime Verification"](https://doi.org/10.1016/j.jlap.2008.08.004) (JLAP 2009)
- Casini et al., ["Response-Time Analysis of ROS 2 Processing Chains"](https://doi.org/10.4230/LIPIcs.ECRTS.2019.6) (ECRTS 2019)
- Feiertag et al., ["A Framework for Measuring Data Age"](https://doi.org/10.1109/ETFA.2009.5347tried) (ETFA 2009)
- Becker et al., ["End-to-End Timing Analysis of Cause-Effect Chains"](https://doi.org/10.4230/LIPIcs.ECRTS.2017.9) (ECRTS 2017)
- [SAE AS5506C](https://www.sae.org/standards/content/as5506c/) (AADL) — flow latency analysis
- [AUTOSAR TIMEX R22-11](https://www.autosar.org/standards/r22-11) — event chain timing constraints
- [CARET](https://github.com/tier4/caret) — Chain-Aware ROS 2 Evaluation Tool (see `docs/research/caret-analysis.md`)
