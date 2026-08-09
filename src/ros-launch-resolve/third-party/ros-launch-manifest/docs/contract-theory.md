# Contract Theory for Launch Manifests

*This is the formal theory behind manifest timing contracts. For the
manifest format itself, see [launch-manifest.md](launch-manifest.md).*

## Motivation

Consider a perception pipeline with five nodes:

```
sensor → cropbox (5ms) → ground_filter (15ms) → detector (30ms) → tracker (20ms)
```

The pipeline has a 100ms latency budget. Today it takes 70ms — plenty of
headroom. Then a developer upgrades the detector model. It now takes 45ms.
The pipeline takes 85ms — still under budget. But add one more node to the
chain, or a bad scheduling day, and you silently break the 100ms contract.

**Without contracts**, you discover this at integration time (or in
production). **With contracts**, each node declares its own budget, and the
checker verifies that the sum fits the scope budget — at authoring time,
before any code runs.

The manifest's contract system makes these budgets explicit, composable,
and statically checkable. This document describes the formal foundations.

For the manifest format itself, see [launch-manifest.md](launch-manifest.md).

## Notation

Symbols used throughout this document:

| Symbol | Meaning |
|--------|---------|
| $C = (A, G)$ | Contract: assumption $A$ + guarantee $G$ |
| $M$ | Component (a node or scope that satisfies a contract) |
| $L_{\text{node}}(X)$ | Worst-case processing time of node $X$ (from `max_latency_ms`) |
| $L_{\text{transport}}(X \to Y)$ | Worst-case transport time between nodes $X$ and $Y$ (from topic's `max_transport_ms`; 0 when omitted) |
| $L_{\max}$, $L_{\min}$ | Worst / best case end-to-end latency of a path or scope |
| $A_{\max}$ | Maximum data age at a subscriber (ms) — runtime checked via `max_age_ms` |
| $f$ | Frequency (Hz) |
| $P$ | Timer period (ms) |
| $J$ | Jitter — max deviation from ideal period (ms) |
| $d$ | Drop rate: fraction of messages lost (from `max_drop_rate`, range 0-1) |
| $\mathcal{R}$ | Delivery rate: $\mathcal{R} = 1 - d$ (fraction that survives) |
| $K$ | Max consecutive drops (from `max_consecutive`) |
| $\ell_{\max}$ | Observed longest consecutive drop run (runtime) |
| `budget-overflow` | Verification check: descendant budget exceeds ancestor budget (error) |
| `scope-budget` | Verification check: sum of children exceeds scope budget (warning) |

## What is a Contract?

A **contract** is a pair $C = (A, G)$:

- **Assumption** ($A$) — constraints on inputs that must hold for the
  guarantee to be valid. "I need sensor data at 10 Hz or faster."
- **Guarantee** ($G$) — constraints on outputs that the component
  promises, given the assumption holds. "I will produce a result within
  30ms."

A component **satisfies** its contract when: if the assumption holds
on the inputs, the guarantee holds on the outputs. "If you give me what
I asked for, I'll deliver what I promised." *(Formally: $M \cap A \subseteq G$
— the component's behaviors, intersected with the assumed inputs, are
all within the guaranteed outputs.)*

### Three Levels

The manifest defines contracts at three levels. To make this concrete,
consider a perception pipeline:

```
pointcloud → [cropbox: 5ms] → [ground_filter: 15ms] → [detector: 30ms]
```

**Topic contract** — for the channel between cropbox and ground_filter:
- *Assumption:* cropbox publishes at ≥ 10 Hz
- *Guarantee:* the channel delivers at 10 Hz with `max_drop_rate: 0.01`

**Node contract** — for ground_filter:
- *Assumption:* receives filtered points at ≥ 10 Hz
- *Guarantee:* output within 15ms

**Scope contract** — for the whole perception pipeline:
- *Assumption:* parent scope wires the pointcloud input
- *Guarantee:* end-to-end latency ≤ 50ms

Summary:

| Level | What it describes | Assumption | Guarantee |
|-------|-------------------|------------|-----------|
| **Topic** | A communication channel | Publisher produces at `rate_hz` | Channel delivers at `rate_hz` with drops ≤ `max_drop_rate` |
| **Node** | A single computation | Inputs arrive per `min_rate_hz`, `state`, `required` | Output within `max_latency_ms` |
| **Scope** | An entire launch file | Topic declarations consistent across tree | E2E `max_latency_ms`, `max_drop_rate` |

These compose hierarchically: topic contracts constrain the channels,
node contracts describe per-node timing, scope contracts abstract the
internal graph into an end-to-end budget.

### Example: NDT Scan Matcher

Now that we've seen the three levels, let's look at one node's contract
in detail — both the YAML declaration and its formal interpretation:

```yaml
nodes:
  ndt_scan_matcher:
    sub:
      sensor_points:
        min_rate_hz: 10
      map:
        state: true
        required: true
    pub:
      ndt_pose:
        min_rate_hz: 10
    paths:
      localization:
        input: sensor_points
        output: [ndt_pose]
        max_latency_ms: 50
```

**Assumption** ($A$):
- `sensor_points` arrives at $f \geq 10$ Hz (causal trigger)
- `map` has been received at least once (`required`) and is polled (`state`)

**Guarantee** ($G$):
- `ndt_pose` published at $f \geq 10$ Hz
- $L_{\max} \leq 50$ ms (trigger to output)

(Drops are declared on the topics that carry `ndt_pose`, not on the
node path — see [Drop Budgets](#drop-budgets).)

The **assumption/guarantee separation** is what makes contracts useful
for diagnosis. At runtime:

| Assumption | Guarantee | Diagnosis |
|------------|-----------|-----------|
| met | met | Nominal |
| met | violated | **Node bug** — computation exceeds its declared budget |
| violated | met | Upstream problem, but this node is robust |
| violated | violated | Upstream problem — not this node's fault |

## Composition

A single node's contract says what it promises in isolation. But a
pipeline's guarantee depends on how nodes are connected. The composition
rules below show how to compute the pipeline's end-to-end latency
from the individual node contracts.

### Understanding Worst-Case Latency

A message traverses a pipeline by passing through nodes and the
transport channels (topics) between them. At each stage, time is spent:

- **Node processing** — the time a node takes to receive an input,
  compute, and publish the output. This is the node's `max_latency_ms`.
- **Transport** — the time for a published message to reach the next
  subscriber via DDS. On the same machine this is typically < 1ms;
  across machines it depends on the network.

The **worst-case latency** of a path is the total time assuming every
stage takes its maximum. This is a pessimistic bound — the real latency
is usually lower, but the bound is what the contract guarantees.

We write:
- $L_{\text{node}}(X)$ — worst-case processing time of node $X$
  (from the node's `max_latency_ms`)
- $L_{\text{transport}}(X \to Y)$ — worst-case transport time between
  node $X$ and node $Y$ (from the topic's `max_transport_ms`; 0 when
  omitted)

### Series (Pipeline)

Nodes connected in sequence. Each message passes through every node,
spending time in processing and transport at each hop:

```
in → [A: 5ms] —transport→ [B: 15ms] —transport→ [C: 30ms] → out
```

The worst case is when every node and every transport takes its maximum
time. Latencies simply add up:

$$L_{\max} = L_{\text{node}}(A) + L_{\text{transport}}(A \to B) + L_{\text{node}}(B) + L_{\text{transport}}(B \to C) + L_{\text{node}}(C)$$

In this example with same-machine transport (~0):

$$5 + 0 + 15 + 0 + 30 = 50 \text{ ms}$$

**Why it's the sum:** the message cannot be in two nodes at once. It
arrives at A, waits for A to finish, travels to B, waits for B to
finish, and so on. Each delay is sequential.

**Transport is declared per topic.** Each topic can declare
`max_transport_ms` — the worst-case time for a published message to
reach the subscriber via DDS. Topics without `max_transport_ms`
contribute 0 to the budget sum; their transport is absorbed into the
scope's residual headroom. On the same machine, transport is typically
< 1ms and can be omitted. For cross-machine hops (sensor ECUs, network
bridges), declare `max_transport_ms` to make the budget explicit.

**Age accumulates** along the chain — each node and transport hop adds
to the total time since the original sensor reading. Age is checked at
runtime on subscriber endpoints (`max_age_ms`), not statically composed.
See [Data Age](#data-age).

### Parallel (Fork-Join)

Two branches merging at a fusion node. The fusion node waits for inputs
from both branches before producing output:

```
      ┌→ A (50ms) →┐
in →  │             ├→ C (20ms) → out
      └→ B (30ms) →┘
```

The worst case: the **slower branch** determines how long the fusion
node waits. Then add the fusion node's own processing time:

$$L_{\max} = \max(L_{\max}(\text{branch } A),\; L_{\max}(\text{branch } B)) + L_{\text{node}}(C)$$

where $L_{\max}(\text{branch } A)$ is the end-to-end latency of branch A
— which may itself be a series pipeline of multiple nodes.

Example: $\max(50, 30) + 20 = 70$ ms.

Note: the best-case latency $L_{\min}$ also uses $\max$ — the fusion
barrier waits for the slowest branch even when both are fast.

**Rate = slowest branch:** $f = \min(f_A, f_B)$

**Age depends on correlation mode:**

- `correlation: timestamp` — the output stamp is the **oldest** input
  stamp. Age = max of branch ages + fusion processing:

$$A_{\max} = \max(A_{\max}(\text{branch } A),\; A_{\max}(\text{branch } B)) + L_{\text{node}}(C)$$

- `correlation: latest` — the output stamp is the **primary input's**
  stamp (first listed input). Age follows only the primary branch:

$$A_{\max} = A_{\max}(\text{primary branch}) + L_{\text{node}}(C)$$

The `latest` mode reflects how most Autoware fusion nodes work: the
primary input triggers the callback, secondary inputs are polled. The
output inherits the primary input's provenance.

Drop at the barrier is **not composed statically** — the user declares
the fusion node's observed drops directly, combining all causes
(upstream propagation, correlation mismatch, computation) into one value.

### Periodic (Timer-Driven)

A timer-driven node runs at fixed intervals (period $P$), reading the
latest state from a buffer. It does not react to individual messages —
it wakes up on the timer and processes whatever is available.

```
upstream → [state buffer] → [periodic node, P=100ms, J=5ms] → out
```

**Worst-case latency through a periodic node.** The worst case happens
when new data arrives just *after* the timer fires. The data sits in the
buffer for nearly one full period before the next timer tick processes it:

$$L_{\max}(\text{periodic}) = P + J + L_{\text{node}}(\text{periodic})$$

Where:
- $P$ — one full timer period of waiting (worst case)
- $J$ — timer jitter (the timer itself may be late)
- $L_{\text{node}}(\text{periodic})$ — the periodic node's processing time

Upstream processing time is **not** included — it is already accounted
for by the upstream node's own `max_latency_ms` in series composition.
The periodic node's contribution to the chain is the buffer wait plus
its own processing.

**Best case:** the timer fires right as data arrives (zero wait):

$$L_{\min}(\text{periodic}) = L_{\text{node}}(\text{periodic})$$

**Rate is independent of upstream:** $f = 1000 / P$ (where $P$ is in ms). The periodic node
produces output at its own timer rate regardless of how fast or slow
the upstream is.

**Periodic nodes reset the consecutive drop chain.** Upstream
consecutive drops don't propagate because the timer fires regardless
of whether new data arrived. Each segment (before and after the
periodic node) is checked independently.

### Drop Budgets

Drops are declared as `max_drop_rate` (a fraction, 0-1) on **topics**
(transport drops) and **scope paths** (E2E drops). Node paths do not
have drop fields — if a node internally drops messages, the effect is
reflected in a lower `pub.min_rate_hz` on its output.

**Static checking** validates local consistency only:

- Values in range: $0 \leq d \leq 1$, `max_consecutive` positive integer
- Scope drop rate sanity: scope `max_drop_rate` must not be tighter than
  any individual topic's `max_drop_rate` on the path (part > whole)
- Rate-drop compatibility: a topic's effective delivery rate must meet
  subscriber demand:

$$f_{\text{topic}} \cdot (1 - d_{\text{topic}}) \geq f_{\min}(\text{sub})$$

**Runtime monitoring** handles composition and consecutive checks —
these depend on actual transport conditions (burstiness, congestion)
that cannot be proven statically. The runtime monitor observes actual
drop patterns and checks `max_drop_rate` and `max_consecutive` against
observed values. See [Burstiness](#burstiness) for detection metrics
and Appendix A for the underlying theory.

### Composition Summary

| Topology | Latency | Rate | Age | Drop |
|----------|---------|------|-----|------|
| **Series** | sum of nodes + transport | preserved | sum along chain | runtime monitoring |
| **Parallel (`timestamp`)** | max(branches) + fusion | min of branches | max(branches) + fusion | runtime monitoring |
| **Parallel (`latest`)** | primary branch + fusion | primary branch | primary branch + fusion | runtime monitoring |
| **Periodic** | $P$ + $J$ + node | $1000/P$ (independent) | resets stamp chain | runtime monitoring |

## Verification Rules

The checker verifies that declared budgets are consistent across the
scope tree. Two separate checks apply to latency, drop, and age — each
with property-specific composition math but the same structural rules.

For precise measurement point definitions, see
[Latency and Data Freshness](launch-manifest.md#latency-and-data-freshness) in the manifest spec.

### Opaque vs Transparent Scopes

A scope's behavior in the budget check depends on whether it declares
its own budget:

- **Opaque** (has budget declared): the parent uses the declared value.
  The scope is a black box — its internal decomposition is its own
  responsibility.
- **Transparent** (no budget declared): the parent looks through it
  and sees the children directly. It's just organizational grouping
  (namespacing), not a timing boundary.

### Check 1: Budget Overflow (Error)

No descendant's budget may exceed any ancestor's budget. A node with
20ms inside a scope with 10ms is always wrong — the part cannot be
bigger than the whole, regardless of transport or topology.

The checker walks the scope tree from root to leaves, tracking the
tightest ancestor budget seen so far. At each node or scope with a
declared budget, it verifies the budget fits within the ancestor's.
Transparent scopes (no budget) don't tighten the constraint — the
walk passes through them.

```
Implementation sketch:

check_overflow(node, ancestor_budget):
  if node.budget > ancestor_budget → ERROR
  effective = min(node.budget, ancestor_budget)
  for each child:
    check_overflow(child, effective)
```

This check is **topology-unaware** — it doesn't distinguish series from
parallel. For parallel branches, each branch is individually less than
the scope budget (since max(branches) < scope is a weaker constraint
than sum). The overflow check catches the trivially wrong case; the
sum check (below) catches composition errors.

### Check 2: Budget Sum (Warning)

For each scope with a declared budget, the sum of its direct children's
budgets must fit within the scope budget. Children without budgets
contribute to the **residual** — the unallocated portion of the scope
budget that covers transport, scheduling variance, and undeclared nodes.

For transparent scopes (no budget), the parent looks through and
collects grandchildren directly.

```
Implementation sketch:

check_sum(scope):
  if scope has no budget → skip
  declared_nodes = collect children with budgets (look through transparent scopes)
  undeclared_nodes = children without budgets
  declared_transport = sum of max_transport_ms on topics within scope (0 when omitted)
  undeclared_transport_count = topics without max_transport_ms
  total = sum(declared_nodes) + declared_transport
  if total > scope.budget → WARNING
  residual = scope.budget - total
  if undeclared_nodes or undeclared_transport_count:
    → INFO: "Xms residual across N undeclared nodes and M topics without transport budget"
```

The sum check is a **warning** (not an error) because the sum is a lower
bound — topics without `max_transport_ms` contribute 0. The gap between
the sum and the scope budget is the allowance for undeclared transport,
scheduling variance, and undeclared nodes.

Like the overflow check, the sum check is **topology-unaware**. For
parallel branches, the sum is conservative (sum > max), so the check
may warn even when the actual composition fits. This is the right
direction — a false warning is better than a missed violation.

### Latency Example

```
scope S: max_latency_ms: 100
  ├── sub-scope P: max_latency_ms: 50    (opaque)
  │     ├── node A: max_latency_ms: 20
  │     ├── topic T1: max_transport_ms: 2  (A → B)
  │     └── node B: max_latency_ms: 25
  ├── sub-scope Q: (no budget)            (transparent)
  │     └── node C: max_latency_ms: 30
  ├── topic T2: (no transport budget)     (P → C)
  ├── topic T3: (no transport budget)     (C → E)
  └── node E: (no budget)
```

**Overflow check** — walk the tree with tightest ancestor:
- A(20) ≤ min(S:100, P:50) = 50. OK.
- B(25) ≤ 50. OK.
- C(30) ≤ min(S:100) = 100. OK. (Q has no budget, doesn't tighten.)

**Sum check on P** (opaque, checks its own children):
- A(20) + T1(2) + B(25) = 47 ≤ 50. Passes. 3ms residual.

**Sum check on S** (looks through transparent Q):
- P(50, opaque) + C(30, from Q look-through) = 80. E is undeclared.
  T2 and T3 have no transport budget (contribute 0).
- 80 ≤ 100. Passes. Residual: 20ms across 1 undeclared node (E) and
  2 topics without transport budget.

If E later gets a budget of 25ms: 50 + 30 + 25 = 105 > 100. Warning.

### Drop Example

Drop budgets are checked statically for local consistency (sanity) and
at runtime for actual behavior. Drops live on **topics** (transport)
and **scope paths** (E2E), not on node paths.

```
scope S path: max_drop_rate: 0.10
  topic T1: max_drop_rate: 0.03    (A → B transport)
  topic T2: max_drop_rate: 0.03    (B → C transport)
  topic T3: (no drop budget)       (C → D transport)
```

**Sanity check**: T1 drop (3%) < S drop (10%). T2 (3%) < 10%. OK.
No topic has a tighter drop budget than the scope.

**Rate-drop check**: if T1 has `rate_hz: 10` and subscriber B has
`min_rate_hz: 9`: effective delivery = $10 \times (1 - 0.03) = 9.7$ Hz ≥ 9. Passes.

**Runtime monitoring** observes actual E2E drop rates and checks them
against the scope's `max_drop_rate: 0.10` and `max_consecutive` (if
declared). See [Burstiness](#burstiness) and Appendix A for the theory.

### Partial Decomposition

A scope contract is valid without full node-level decomposition. Three
scenarios:

| Scope budget | Node budgets | Checker behavior |
|-------------|-------------|-----------------|
| Declared | All declared | Verify composition ≤ scope, check overflow |
| Declared | Some missing | Verify declared fit, report residual (latency) |
| Declared | None declared | Accept — runtime monitoring checks E2E |

This supports a **top-down workflow**: start with the E2E requirement,
fill in node budgets as you measure them. The residual (or gap report)
tells you what's unaccounted for.

## Data Age

The **age** of a message at a subscriber is the time since the original
sensor data was created:

$$\text{age} = t_{\text{take}} - \text{header.stamp}(\text{source})$$

This works because causal paths preserve `header.stamp` through the
chain — each node copies the input stamp to the output (see
[Timestamps and Data Flow](launch-manifest.md#timestamps-and-data-flow)).

**`max_age_ms`** is declared on **subscriber endpoints**, not on paths.
It constrains data freshness at the point of consumption:

```yaml
nodes:
  planner:
    sub:
      objects:
        max_age_ms: 200          # data must be fresher than 200ms
```

**Runtime checking:** the interception layer reads `header.stamp` on
every `rcl_take` and compares to current time. If
`now - stamp > max_age_ms`, a violation is flagged.

**Static checking** does not trace the full causal chain (which would
require every upstream node to have a latency budget). Instead, the
checker verifies local consistency: if a subscriber has `max_age_ms`
and the scope path feeding it has `max_latency_ms`, the checker can
verify that the age budget is feasible given the known latency budget.

**For multi-input nodes:** the age at a subscriber depends on the
correlation mode. With `correlation: timestamp`, age reflects the
oldest input. With `correlation: latest`, age follows the primary
(first listed) input only. See
[Parallel composition](#parallel-fork-join) for the formulas.

## Burstiness

The drop composition rules assume each drop is independent (Bernoulli
model). In practice, DDS transport drops are often **bursty** — network
congestion, scheduling jitter, or queue overflow cause drops to cluster.
When drops are bursty, the declared `max_drop_rate` and `max_consecutive`
thresholds may be violated more often than the Bernoulli model predicts.
The runtime monitor detects this gap.

The runtime monitor detects burstiness via two lightweight metrics
(~5 ns/message overhead):

- **Lag-1 autocorrelation** ($\rho_1$) — measures whether a drop
  predicts the next drop. $\rho_1 \approx 0$: independent. $\rho_1 > 0.05$
  with 1000+ samples: significant burstiness.

- **Dispersion index** ($DI$) — variance-to-mean ratio of drops per
  window. $DI \approx 1$: Poisson-like. $DI > 1.5$: overdispersed (bursty).

When burstiness is detected, the monitor recommends increasing
`max_consecutive` or investigating the burst cause. See Appendix B
for the detailed metric formulas.

## Appendix A: Drop Composition Theory

The formulas below describe the theoretical relationships between
per-topic drop rates and chain-level behavior. These are used by the
**runtime monitor** for analysis and alerting, not by the static
checker (which only validates local consistency).

### A.1 Delivery Rate Composition

Each topic on the critical path declares `max_drop_rate: d_i`. The
delivery rate:

$$\mathcal{R}_i = 1 - d_i$$

For a series chain of independent transports, a message must survive
every hop:

$$\mathcal{R}_{\text{chain}} = \prod_i \mathcal{R}_i = \prod_i (1 - d_i)$$

In log form (convenient for implementation):

$$\ln \mathcal{R}_{\text{chain}} = \sum_i \ln(1 - d_i)$$

The scope declares `max_drop_rate: d_s`. The check:

$$d_s \geq 1 - \mathcal{R}_{\text{chain}}$$

### A.2 Consecutive Drop: Poisson Derivation

Model each message as an independent Bernoulli trial with drop
probability $d$ (the composed drop rate for the chain, or a single
topic's `max_drop_rate`). We want the probability of $K$ or more
consecutive drops in $W$ messages, where $W$ is a runtime monitoring
window.

**Run starts.** A run of $K$ consecutive drops can start at positions
$1, 2, \ldots, W - K + 1$. For position $i > 1$: messages $i$ through
$i+K-1$ must drop ($d^K$) and message $i-1$ must not drop ($1-d$).
Per-position probability: $\approx d^K \cdot (1-d)$.

**Poisson approximation.** When runs are rare ($d^K$ small), the
count of runs $\geq K$ is approximately Poisson with mean:

$$\lambda = (W - K + 1) \cdot d^K \cdot (1-d)$$

Probability of at least one such run:

$$P(\text{max run} \geq K) \approx 1 - e^{-\lambda}$$

### A.3 Scope Consecutive Check

The scope declares `max_consecutive: K_s`. Using a monitoring window
of $W$ messages (runtime parameter), we require the probability of
violation to be below confidence threshold $\epsilon$ (default 0.01):

$$(W - K_s + 1) \cdot d^{K_s} \cdot (1-d) \leq -\ln(1 - \epsilon) \approx 0.01$$

### A.4 Example: Three-Topic Pipeline

Three topics in series, each `max_drop_rate: 0.02`.

Chain: $\mathcal{R} = (1 - 0.02)^3 = 0.98^3 = 0.941$, $d = 0.059$.

Scope declares `max_drop_rate: 0.06, max_consecutive: 3`.

**Rate check:** $0.06 \geq d = 0.059$. Passes.

**Consecutive check** (monitoring window $W = 200$, $K = 3$):

$$198 \times 0.059^3 \times 0.941 = 0.038$$

$0.038 > 0.01$: **Fails.** Three consecutive drops are too likely (3.8%).

**Try** `max_consecutive: 4`:

$$198 \times 0.059^4 \times 0.941 = 0.002$$

$0.002 \leq 0.01$: **Passes.**

### A.5 Example: With Periodic Reset

Topics: T1 (`max_drop_rate: 0.01`, cropbox→centerpoint) and
T2 (`max_drop_rate: 0.02`, centerpoint→tracker). Tracker is periodic
(`max_drop_rate: 0.01` on the output topic T3).

**Pre-tracker segment:** $\mathcal{R} = 0.99 \times 0.98 = 0.970$, $d = 0.030$.
**Post-tracker segment:** periodic resets the chain. $d = 0.01$ (T3 only).

Scope declares `max_consecutive: 3`, monitoring window $W = 200$.

Pre-tracker: $198 \times 0.030^3 \times 0.970 = 0.005 \leq 0.01$. Passes.
Post-tracker: $198 \times 0.010^3 \times 0.990 = 0.0002 \leq 0.01$. Passes.

Both segments pass independently. The periodic node prevents upstream
bursts from propagating.

## Appendix B: Burstiness Metric Formulas

**Lag-1 autocorrelation.** Given binary trace $x[t] \in \{0, 1\}$
(1=delivered, 0=dropped):

$$\rho_1 = \frac{\sum_{t=1}^{N-1} (x[t] - \bar{x})(x[t+1] - \bar{x})}{\sum_{t=1}^{N} (x[t] - \bar{x})^2}$$

**Dispersion index.** Divide trace into windows of size $W$. Count
drops per window $d_i$:

$$DI = \frac{\mathrm{Var}(d_i)}{E(d_i)}$$

**Observed max run** $\ell_{\max}$ (longest consecutive drop sequence).
Compare to Bernoulli prediction:

$$E[\ell_{\max}^{\text{Bernoulli}}] \approx \frac{\ln(N \cdot (1-d))}{\ln(1/d)}$$

If $\ell_{\max} \gg E[\ell_{\max}^{\text{Bernoulli}}]$, drops are burstier
than the model predicts.

**Estimated mean burst length** (reported when $DI > 1.5$):

$$\hat{r} = 1 - P(\text{drop} \mid \text{previous drop})$$
$$\text{mean burst} = 1 / \hat{r}$$

## Appendix C: Empirical Contract Derivation

When writing a manifest for an existing system without documented timing
requirements, you need initial values for `max_latency_ms`, `min_rate_hz`,
and `max_drop_rate`. Capture mode bootstraps these from runtime
measurements.

Capture mode (`--save-manifest-dir`) derives contracts from observed
traces:

$$\hat{G}_L = \max(\text{observed latencies}) \times \alpha$$
$$\hat{A}_R = \min(\text{observed inter-arrivals}) / \alpha$$

where $\alpha > 1$ is the safety margin (default 1.2).

After $N$ observations without violation, at confidence level $c$:

$$P(\text{violation per trial}) \leq 1 - (1 - c)^{1/N}$$

For $N = 1000$ at $c = 0.99$: $P \leq 0.0046$.

Capture provides a starting point. Users refine manually or tighten
margins as more data is collected.

## References

- Benveniste et al., ["Contracts for System Design"](https://doi.org/10.1561/2500000017) (Foundations and Trends in EDA, 2018)
- de Alfaro & Henzinger, ["Interface Automata"](https://doi.org/10.1145/366927.366984) (POPL 2001)
- Casini et al., ["Response-Time Analysis of ROS 2 Processing Chains"](https://doi.org/10.4230/LIPIcs.ECRTS.2019.6) (ECRTS 2019)
- Becker et al., ["End-to-End Timing Analysis of Cause-Effect Chains"](https://doi.org/10.4230/LIPIcs.ECRTS.2017.9) (ECRTS 2017)
- [SAE AS5506C](https://www.sae.org/standards/content/as5506c/) (AADL) — flow latency analysis
- [AUTOSAR TIMEX R22-11](https://www.autosar.org/standards/r22-11) — event chain timing constraints
- [CARET](https://github.com/tier4/caret) — Chain-Aware ROS 2 Evaluation Tool
- Erdos & Renyi, "On a new law of large numbers" (*J. Analyse Math.* 1970) — longest runs in Bernoulli sequences
- Gilbert, "Capacity of a Burst-Noise Channel" (*Bell System Technical J.* 1960) — burst error model
