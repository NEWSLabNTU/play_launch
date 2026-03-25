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

Delivery rate (see Drop Composition below):

$$\mathcal{R}_{\text{chain}} = \prod_i \mathcal{R}_i$$

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

Rate: $R_{\text{rate}} = \min(R_A, R_B)$

Drop: all drop causes (upstream propagation, correlation mismatch,
computation failure) are combined into the node's single `drop:` value.

Age: $A_{\max} = \max(A_{\max}(A), A_{\max}(B)) + L_{\max}(C)$

### Periodic (timer-driven)

A timer-driven node with period $P$ and jitter $J$:

$$L_{\max}(\text{through periodic}) = L_{\max}(\text{upstream}) + P + J + C$$
$$L_{\min}(\text{through periodic}) = L_{\min}(\text{upstream}) + C$$

Best case: timer fires right as state updates (zero wait).
Worst case: state arrives just after timer, waits full period.

Rate through periodic: $R_{\text{rate}} = 1000 / P$ (independent
of upstream). The path's `max_age_ms` constrains how old the
original source data can be at the output.

Periodic nodes **reset** the consecutive drop chain — upstream
consecutive drops don't propagate because the timer fires regardless.

### Drop Composition

Define **delivery rate** $\mathcal{R} = 1 - d$ where $d = N/W$ is
the drop rate from the manifest's `drop: N / W` declaration.

#### Series drop rate

$$\mathcal{R}_{\text{chain}} = \prod_i \mathcal{R}_i$$

In log form:

$$\ln \mathcal{R}_{\text{chain}} = \sum_i \ln \mathcal{R}_i$$

**Scope check**: given scope declares `drop: N_s / W_s`:

$$1 - N_s / W_s \leq \mathcal{R}_{\text{chain}}$$

Each node declares its own $N_i / W_i$ with its own window size.
The checker converts to per-message probability, composes, and
checks against the scope's declared window.

#### Consecutive drop composition

Given chain delivery rate $\mathcal{R}_{\text{chain}}$, the
probability of a run of $K$ or more consecutive drops in a window
of $W$ messages (Poisson approximation of the Erdos-Renyi longest
run distribution):

$$P(\text{max run} \geq K \mid W) \approx 1 - \exp\left(-(W - K + 1) \cdot (1 - \mathcal{R})^K \cdot \mathcal{R}\right)$$

where $\mathcal{R} = \mathcal{R}_{\text{chain}}$.

**Scope check**: given scope declares `max_consecutive: K_s` and
window $W_s$, verify at confidence $\epsilon = 0.01$:

$$(W_s - K_s + 1) \cdot (1 - \mathcal{R})^{K_s} \cdot \mathcal{R} \leq -\ln(1 - \epsilon)$$

For $\epsilon = 0.01$: right-hand side $\approx 0.01$.

**Necessary condition**: $K_s \geq \max_i K_i$ — the scope can't
be stricter than any individual node. Periodic nodes reset the
consecutive chain; check each segment independently.

#### Note: bursty drops

The Bernoulli model assumes independent drops. In practice, DDS
transport drops are often **bursty** due to network congestion,
OS scheduling, or queue overflow. The Gilbert-Elliott model (a
two-state HMM with Good/Bad states, mean burst length $= 1/r$)
better captures this behavior, but its composition rules are
complex (series produces a 4-state HMM, not another Gilbert-Elliott).

For Phase 31, the static checker uses the Bernoulli model. The
runtime monitor detects burstiness via diagnostics (see Burstiness
Diagnostics below) and warns the user when the Bernoulli model
is inadequate.

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

## Burstiness Diagnostics

The static checker uses the Bernoulli model for drop composition.
At runtime, the monitor detects whether observed drops are actually
independent (Bernoulli-like) or bursty. This is computed always-on
for all topics (negligible cost: ~5 ns/message, ~80 bytes/topic)
but reported selectively.

### Metrics

**Lag-1 autocorrelation**: Given binary trace $x[t] \in \{0, 1\}$
(1=delivered, 0=dropped):

$$\rho_1 = \frac{\sum_{t=1}^{N-1} (x[t] - \bar{x})(x[t+1] - \bar{x})}{\sum_{t=1}^{N} (x[t] - \bar{x})^2}$$

- $\rho_1 \approx 0$: independent drops (Bernoulli valid)
- $\rho_1 > 0.05$ with $N > 1000$: significant burstiness

**Dispersion index**: Divide trace into windows of size $W$ (default
100). Count drops per window $d_i$:

$$DI = \frac{\mathrm{Var}(d_i)}{E(d_i)}$$

- $DI \approx 1$: Poisson-like (Bernoulli)
- $DI > 1.5$: overdispersed (bursty)

**Observed max run**: Longest consecutive drop sequence $L_{\max}$.
Compare to Bernoulli prediction:

$$E[L_{\max}^{\text{Bernoulli}}] \approx \frac{\ln(N \cdot (1-d))}{\ln(1/d)}$$

If $L_{\max} \gg E[L_{\max}^{\text{Bernoulli}}]$, drops are burstier
than the Bernoulli model predicts.

**Estimated mean burst length** (reported only when $DI > 1.5$):

$$\hat{r} = 1 - P(\text{drop} \mid \text{previous drop})$$

$$\text{mean burst} = 1 / \hat{r}$$

### Reporting

The monitor always collects counters but reports selectively:

1. **Declared topics** (have `drop:` in manifest): Full diagnostic
   — rate check, consecutive check, burstiness, recommendation.
2. **Undeclared topics with anomalies** (drop rate > 1%, or $DI > 2$,
   or max run > 5): Discovery alerts with recommendation to add
   a `drop:` declaration.
3. **Undeclared topics with no issues**: Not shown.

### Recommendations

When burstiness is detected, the monitor suggests:

- Increase `max_consecutive` budget (with suggested value from
  observed burst lengths)
- Investigate burst cause (CPU contention, DDS queue overflow)
- Switch to RELIABLE QoS to prevent transport bursts
- Add `drop:` declaration for undeclared topics

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

## Appendix A: Drop Composition Derivation

### A.1 Delivery Rate Composition

Each node declares `drop: N_i / W_i`. Define **delivery rate**:

$$\mathcal{R}_i = 1 - \frac{N_i}{W_i}$$

For a series chain of independent components, a message must survive
every stage. The chain delivery rate is the product:

$$\mathcal{R}_{\text{chain}} = \prod_i \mathcal{R}_i$$

In log form (convenient for implementation):

$$\ln \mathcal{R}_{\text{chain}} = \sum_i \ln \mathcal{R}_i$$

The scope declares `drop: N_s / W_s`. The check:

$$\frac{N_s}{W_s} \geq 1 - \mathcal{R}_{\text{chain}}$$

### A.2 Consecutive Drop: Derivation

We model each message as an independent Bernoulli trial with drop
probability $d = 1 - \mathcal{R}_{\text{chain}}$. We want the
probability of seeing $K$ or more consecutive drops in $W$ messages.

**Step 1: Counting run starts.** A run of $K$ consecutive drops can
start at positions $1, 2, \ldots, W - K + 1$. There are $W - K + 1$
possible starting positions.

**Step 2: Per-position probability.** For a run starting at position
$i$ (where $i > 1$):
- Messages $i, i+1, \ldots, i+K-1$ must all drop: $d^K$
- Message $i-1$ must NOT drop (otherwise the run started earlier): $(1-d)$
- Combined: $d^K \cdot (1-d)$

For $i = 1$, there is no preceding message, so the probability is
$d^K$. Averaging over all positions gives approximately
$d^K \cdot (1-d)$ per position.

**Step 3: Poisson approximation.** When runs are rare events (small
$d^K$), the number of runs of length $\geq K$ is approximately
Poisson-distributed with mean:

$$\lambda = (W - K + 1) \cdot d^K \cdot (1-d)$$

The probability of at least one such run:

$$P(\text{max run} \geq K) = 1 - P(\text{Poisson}(\lambda) = 0) = 1 - e^{-\lambda}$$

Therefore:

$$P(\text{max run} \geq K \mid W) \approx 1 - \exp\left(-(W - K + 1) \cdot d^K \cdot (1-d)\right)$$

### A.3 Scope Consecutive Check

The scope declares `max_consecutive: K_s` over window $W_s$.
We require the probability of violation to be below confidence
threshold $\epsilon$ (default 0.01):

$$P(\text{max run} \geq K_s \mid W_s) \leq \epsilon$$

Substituting and rearranging:

$$1 - \exp(-(W_s - K_s + 1) \cdot d^{K_s} \cdot (1-d)) \leq \epsilon$$

$$\exp(-(W_s - K_s + 1) \cdot d^{K_s} \cdot (1-d)) \geq 1 - \epsilon$$

Taking $-\ln$ of both sides:

$$(W_s - K_s + 1) \cdot d^{K_s} \cdot (1-d) \leq -\ln(1 - \epsilon)$$

For $\epsilon = 0.01$: $-\ln(0.99) \approx 0.01$. The check becomes:

$$(W_s - K_s + 1) \cdot d^{K_s} \cdot (1-d) \leq 0.01$$

Additionally, the **necessary condition** must hold:
$K_s \geq \max_i K_i$ — the scope can't be stricter than any
individual node. Periodic nodes reset the consecutive chain;
segments are checked independently.

### A.4 Example: Three-Node Pipeline

**Setup**: Three nodes, each `drop: 2/100`.

Chain delivery rate:

$$\mathcal{R}_{\text{chain}} = 0.98^3 = 0.9412$$

Chain drop rate: $d = 1 - 0.9412 = 0.0588$.

**Scope declares** `drop: 12/200, max_consecutive: 3`.

**Rate check**:

$$\frac{12}{200} = 0.06 \geq d = 0.0588 \quad \checkmark$$

**Consecutive check** with $W_s = 200$, $K_s = 3$:

$$(200 - 3 + 1) \times 0.0588^3 \times (1 - 0.0588) = 198 \times 0.000203 \times 0.9412 = 0.0379$$

$0.0379 > 0.01$: **Fails.** Three consecutive drops are too likely
(3.8% probability per window of 200 messages).

**Try** `max_consecutive: 4`:

$$198 \times 0.0588^4 \times 0.9412 = 198 \times 0.0000120 \times 0.9412 = 0.00223$$

$0.00223 \leq 0.01$: **Passes.** Four consecutive drops are
sufficiently unlikely (0.2% probability).

The checker reports: "`max_consecutive: 3` is infeasible
(p=0.038 > 0.01). Minimum feasible: `max_consecutive: 4` (p=0.002)."

### A.5 Example: With Periodic Reset

**Setup**: cropbox (`drop: 1/100`) → centerpoint (`drop: 2/100`) →
tracker (periodic, `drop: 1/100`).

**Pre-tracker segment**: $\mathcal{R} = 0.99 \times 0.98 = 0.9702$,
$d = 0.0298$.

**Post-tracker segment**: tracker is periodic, resets the consecutive
chain. $d = 0.01$.

Scope declares `max_consecutive: 3` over $W_s = 200$.

**Pre-tracker check** ($d = 0.0298$, $K = 3$):

$$198 \times 0.0298^3 \times 0.9702 = 198 \times 0.0000265 \times 0.9702 = 0.00508$$

$0.00508 \leq 0.01$: **Passes.**

**Post-tracker check** ($d = 0.01$, $K = 3$):

$$198 \times 0.01^3 \times 0.99 = 198 \times 10^{-6} \times 0.99 = 0.000196$$

$0.000196 \leq 0.01$: **Passes.**

Both segments pass independently. The periodic node prevents
upstream bursts from propagating.

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
- Erdos & Renyi, "On a new law of large numbers" (*J. Analyse Math.* 1970) — longest runs in Bernoulli sequences
- Schilling, ["The Longest Run of Heads"](https://doi.org/10.1080/07468342.1990.11973306) (*College Math. J.* 1990) — finite-W approximation
- Gilbert, "Capacity of a Burst-Noise Channel" (*Bell System Technical J.* 1960) — burst error model
