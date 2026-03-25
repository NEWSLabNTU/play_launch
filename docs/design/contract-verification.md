# Contract Verification Implementation

Practical tooling for verifying manifest contracts. See
`docs/design/contract-theory.md` for the formal foundations.

## Architecture

```
YAML files ──→ parse with spans ──→ typed AST ──→ generate constraints ──→ check ──→ diagnostics
                (yaml-rust2)        (ManifestAst)    (rules)              (solve)   (codespan-reporting)
```

### Design goals

1. **Span tracking**: errors trace back to the offending YAML source location
2. **Extensibility**: each constraint rule is an independent module
3. **Readability**: constraints can be rendered in formal mathematical notation

## Parsing with Source Spans

`serde_yaml_ng` loses source locations during deserialization. Use
`yaml-rust2` which provides `MarkedYaml` — each YAML node carries
`Marker { line, col }`.

A thin manual deserialization layer (~200-300 lines) builds a typed
AST with spans attached:

```rust
/// A value paired with its source location.
struct Spanned<T> {
    value: T,
    file_id: FileId,
    span: Range<usize>,      // byte offsets into the source file
}

/// Typed manifest AST with full span information.
struct ManifestAst {
    version: Spanned<u32>,
    topics: Vec<Spanned<TopicDecl>>,
    nodes: Vec<Spanned<NodeDecl>>,
    includes: Vec<Spanned<IncludeDecl>>,
    imports: Vec<Spanned<GroupDecl>>,
    exports: Vec<Spanned<GroupDecl>>,
    io: Option<Spanned<IoContract>>,
}
```

`yaml-rust2`'s `Marker` gives line/col. Convert to byte offsets via
a `line_starts: Vec<usize>` table built by scanning the source for
newlines (~20 lines of code).

### Why not Serde?

Serde's data model has no concept of source locations — by the time
a struct is populated, span information is gone. Some workarounds
exist (custom `Deserialize` impls with span wrappers) but they are
fragile and not well-supported for YAML. The manual approach gives
full control and reliable spans.

## Constraint Types

Each constraint carries its formal expression, source span, and
severity:

```rust
/// A formal constraint extracted from the manifest.
struct Constraint {
    kind: ConstraintKind,
    span: SpanInfo,
    severity: Severity,
}

enum Severity { Error, Warning, Info }

/// Source location for diagnostic output.
struct SpanInfo {
    file_id: FileId,
    byte_range: Range<usize>,
    /// Human-readable YAML path, e.g. "topics.pointcloud.qos.reliability"
    path: String,
}
```

### Constraint kinds

```rust
enum ConstraintKind {
    /// $\text{QoS}(p) \succeq \text{QoS}(s)$ for all (pub, sub) on a topic
    QosCompatibility {
        topic: String,
        publisher: EntityRef,
        subscriber: EntityRef,
        field: String,        // "reliability", "durability"
    },

    /// $\exists p \in \text{Publishers}(t)$
    TopicHasPublisher { topic: String },

    /// $\text{scope.latency\_ms} \geq \text{critical\_path}(\text{internal\_graph})$
    ScopeBudgetValidity {
        scope: String,
        declared_ms: f64,
        critical_path_ms: f64,
    },

    /// $N \geq \text{rate\_hz}$ where $N$ is the trigger period
    RateFeasibility {
        topic: String,
        required_hz: f64,
        publisher_trigger: String,
    },

    /// Publisher and subscriber agree on message type
    TypeConsistency {
        topic: String,
        pub_type: String,
        sub_type: String,
    },

    /// Node endpoint is wired by a topic
    EndpointWired {
        node: String,
        endpoint: String,
    },

    /// Import is wired by parent scope
    ImportResolved {
        scope: String,
        import_name: String,
    },
}
```

## Rule System

Each validation rule is an independent module implementing a trait.
This follows the clippy/linter pattern — flat rules, not visitors.

```rust
/// A validation rule that checks one aspect of the manifest.
trait ValidationRule: Send + Sync {
    /// Unique identifier (e.g., "E001", "qos-compatibility")
    fn id(&self) -> &str;

    /// Human-readable description
    fn description(&self) -> &str;

    /// Default severity
    fn default_severity(&self) -> Severity;

    /// Check the manifest and emit constraints into the context.
    fn check(&self, manifest: &ManifestAst, ctx: &mut CheckContext);
}

/// Accumulates constraints from all rules.
struct CheckContext {
    constraints: Vec<Constraint>,
    files: SimpleFiles<String, String>,  // codespan-reporting file store
}

impl CheckContext {
    fn emit(&mut self, kind: ConstraintKind, span: SpanInfo, severity: Severity) {
        self.constraints.push(Constraint { kind, span, severity });
    }
}
```

### Rule registry

```rust
fn default_rules() -> Vec<Box<dyn ValidationRule>> {
    vec![
        Box::new(QosCompatibilityRule),
        Box::new(TopicWiringRule),
        Box::new(ScopeBudgetRule),
        Box::new(RateFeasibilityRule),
        Box::new(TypeConsistencyRule),
        Box::new(ImportResolutionRule),
        // easy to add new rules — just append here
    ]
}

fn run_checks(manifest: &ManifestAst, ctx: &mut CheckContext) {
    for rule in default_rules() {
        rule.check(manifest, ctx);
    }
}
```

### Example rule implementation

```rust
/// E001: QoS compatibility between publisher and subscriber
struct QosCompatibilityRule;

impl ValidationRule for QosCompatibilityRule {
    fn id(&self) -> &str { "E001" }
    fn description(&self) -> &str {
        "Publisher QoS must satisfy subscriber QoS requirements"
    }
    fn default_severity(&self) -> Severity { Severity::Error }

    fn check(&self, manifest: &ManifestAst, ctx: &mut CheckContext) {
        for topic in &manifest.topics {
            let t = &topic.value;
            for pub_ref in &t.publishers {
                for sub_ref in &t.subscribers {
                    // Check reliability: pub ≥ sub
                    if let (Some(pub_qos), Some(sub_qos)) = (&t.pub_qos, &t.sub_qos) {
                        if pub_qos.reliability < sub_qos.reliability {
                            ctx.emit(
                                ConstraintKind::QosCompatibility {
                                    topic: t.name.clone(),
                                    publisher: pub_ref.clone(),
                                    subscriber: sub_ref.clone(),
                                    field: "reliability".into(),
                                },
                                topic.span(),
                                self.default_severity(),
                            );
                        }
                    }
                }
            }
        }
    }
}
```

## Diagnostic Output

### Terminal diagnostics: `codespan-reporting`

Best fit for multi-file YAML diagnostics. Each unsatisfied constraint
becomes a `Diagnostic` with labeled source spans:

```rust
fn constraint_to_diagnostic(c: &Constraint) -> codespan_reporting::diagnostic::Diagnostic<FileId> {
    use codespan_reporting::diagnostic::{Diagnostic, Label};

    match &c.kind {
        ConstraintKind::QosCompatibility { topic, publisher, subscriber, field } => {
            Diagnostic::error()
                .with_code("E001")
                .with_message(format!("QoS incompatibility on topic '{}'", topic))
                .with_labels(vec![
                    Label::primary(c.span.file_id, c.span.byte_range.clone())
                        .with_message(format!("publisher declares {}", field)),
                    // secondary label on subscriber's span
                ])
                .with_notes(vec![
                    format!("publisher {} must be ≥ subscriber {}", field, field),
                ])
        }
        // ... other kinds
    }
}
```

Example output:

```
error[E001]: QoS incompatibility on topic 'pointcloud'
  ┌─ manifests/sensing/sensing.launch.yaml:12:5
  │
12│     reliability: best_effort
  │     ^^^^^^^^^^^^^^^^^^^^^^^^ publisher declares best_effort
  │
  ┌─ manifests/perception/lidar.launch.yaml:8:5
  │
 8│     reliability: reliable
  │     ^^^^^^^^^^^^^^^^^^^^ subscriber requires reliable
  │
  = note: publisher reliability must be ≥ subscriber reliability
```

### Formal notation: `FormalEmitter`

For academic-readable output and documentation:

```rust
impl fmt::Display for ConstraintKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::QosCompatibility { topic, publisher, subscriber, field } =>
                write!(f, "QoS({}, {}).{} ≥ QoS({}, {}).{}",
                    publisher, topic, field, subscriber, topic, field),

            Self::ScopeBudgetValidity { scope, declared_ms, critical_path_ms } =>
                write!(f, "scope({}).max_latency_ms = {} ≥ critical_path = {}",
                    scope, declared_ms, critical_path_ms),

            Self::TopicHasPublisher { topic } =>
                write!(f, "∃p ∈ Publishers : topic(p) = {}", topic),

            Self::RateFeasibility { topic, required_hz, .. } =>
                write!(f, "rate(publisher({})) ≥ {} Hz", topic, required_hz),

            Self::TypeConsistency { topic, pub_type, sub_type } =>
                write!(f, "type(pub, {}) = {} ≡ type(sub, {}) = {}",
                    topic, pub_type, topic, sub_type),
            // ...
        }
    }
}
```

Example output:

```
[E001] QoS(lidar_driver, pointcloud).reliability ≥ QoS(cropbox_filter, pointcloud).reliability  ✗
[E002] scope(perception).max_latency_ms = 85 ≥ critical_path = 70  ✓
[E003] ∃p ∈ Publishers : topic(p) = tracked_objects  ✓
[E004] rate(publisher(pointcloud)) = 10 Hz ≥ 10 Hz  ✓
```

### SMT-LIB output (optional)

For formal methods researchers or future Z3 integration:

```smt2
; E001: QoS compatibility
(assert (>= (qos-reliability "lidar_driver" "pointcloud")
            (qos-reliability "cropbox_filter" "pointcloud")))

; E002: Scope budget
(assert (>= 85.0 (+ (max 50.0 30.0) 20.0)))

; E003: Topic existence
(assert (> (count-publishers "tracked_objects") 0))
```

## File Structure

```
src/manifest/
  parse/
    mod.rs
    yaml_ast.rs              # yaml-rust2 → ManifestAst with Spanned<T>
    span.rs                  # Marker(line,col) → byte offset
  constraint/
    mod.rs
    types.rs                 # Constraint, ConstraintKind, SpanInfo, Severity
    check.rs                 # CheckContext, run_checks()
    emit/
      mod.rs
      diagnostic.rs          # → codespan-reporting terminal output
      formal.rs              # → Unicode mathematical notation
      smtlib.rs              # → SMT-LIB2 (optional)
  rules/
    mod.rs                   # ValidationRule trait, default_rules()
    qos_compatibility.rs     # E001
    topic_wiring.rs          # E002
    scope_budget.rs          # E003
    rate_feasibility.rs      # E004
    type_consistency.rs      # E005
    import_resolution.rs     # E006
```

## Crate Choices

| Concern                 | Crate                 | Why                                                 |
|-------------------------|-----------------------|-----------------------------------------------------|
| YAML parsing with spans | `yaml-rust2`          | `MarkedYaml` gives line/col per node                |
| Diagnostic rendering    | `codespan-reporting`  | Multi-file, FileId-based, stable, used by naga/Deno |
| Error types             | `thiserror`           | Already in our deps                                 |
| Graph analysis          | `petgraph`            | Critical path, cycle detection                      |
| SMT interaction (opt.)  | `rsmt2` or `easy-smt` | Pipe SMT-LIB2 to Z3 subprocess                      |

## Tiered Implementation

### Tier 1: Graph algorithms (implement now)

| Check                       | Algorithm                               | Complexity |
|-----------------------------|-----------------------------------------|------------|
| Critical path (E2E latency) | Topo sort + DP longest path             | $O(V+E)$   |
| Scope budget validity       | Tree walk: children max/sum ≤ parent    | $O(V)$     |
| Cycle detection             | `is_cyclic_directed()` / `tarjan_scc()` | $O(V+E)$   |
| Unreachable nodes           | `has_path_connecting()`                 | $O(V+E)$   |
| QoS compatibility           | Direct comparison per topic             | $O(E)$     |
| Type consistency            | String equality per topic               | $O(E)$     |
| Import/export completeness  | Set difference                          | $O(V)$     |
| Rate feasibility            | Arithmetic per topic                    | $O(E)$     |

This covers ~95% of contract checking with zero new dependencies
beyond `petgraph` (likely already in the dependency tree).

### Tier 2: Runtime monitors (add with audit feature)

Hand-rolled monitors for each contract field:

```rust
struct LatencyMonitor {
    bound_ms: f64,
    violations: u64,
    total: u64,
}

impl LatencyMonitor {
    fn check(&mut self, take_time: f64, pub_time: f64) -> bool {
        self.total += 1;
        if pub_time - take_time > self.bound_ms {
            self.violations += 1;
            return false;
        }
        true
    }
}
```

Each monitor maps to a contract field:

| Contract field                           | Monitor                               |
|------------------------------------------|---------------------------------------|
| `paths.*.max_latency_ms`                 | `LatencyMonitor`                      |
| `paths.*.min_latency_ms`                 | `LatencyAnomalyMonitor`               |
| `paths.*.max_age_ms`                     | `AgeMonitor` (static or header.stamp) |
| topic `rate_hz` / endpoint `min_rate_hz` | `RateMonitor`                         |
| endpoint `jitter_ms`                     | `JitterMonitor`                       |
| `paths.*.drop` / topic `drop`            | `DropMonitor` (sliding window)        |

Data source: Phase 29 RCL interception events via SPSC ring buffer.

Alternative: `rtlola-interpreter` (pure Rust, stream-based runtime
verification) for complex cross-topic correlations. Start with
hand-rolled monitors; migrate if needed.

### Tier 3: Constraint solvers (add if needed)

**`z3` crate** — encode timing constraints as SMT formulas:

```rust
// "Is there a consistent assignment satisfying all scope budgets?"
let solver = z3::Solver::new(&ctx);
for scope in scopes {
    let sum = Real::add(&ctx, &node_latencies);
    solver.assert(&sum.le(&budget));
}
match solver.check() {
    SatResult::Sat => { /* feasible */ }
    SatResult::Unsat => { /* conflicting constraints */ }
}
```

**`good_lp`** — linear programming for budget optimization:
"distribute 170ms E2E budget optimally across pipeline stages."

### Not recommended

- **UPPAAL**: external binary, academic license, XML format
- **SPIN**: wrong abstraction (protocol verification, not timing)
- **TLA+**: no Rust integration, not suited for quantitative timing

## Implementation Path

```
Phase 31 (now):      yaml-rust2 + petgraph + codespan-reporting
                     Parse with spans, graph checks, terminal diagnostics
                     New deps: yaml-rust2, codespan-reporting

Phase 31 (audit):    Hand-rolled monitors + interception events
                     Runtime checking of rate, deadline, latency, drops
                     Zero new deps

Future (if needed):  z3 for constraint satisfiability
                     good_lp for budget optimization
                     rtlola for complex runtime specs
```

## References

- Convent et al., "RTLola Specification Language" (ATVA 2019)
- `yaml-rust2` — https://docs.rs/yaml-rust2
- `codespan-reporting` — https://docs.rs/codespan-reporting
- `petgraph` — https://docs.rs/petgraph
- `z3` — https://docs.rs/z3
- `good_lp` — https://docs.rs/good_lp
- `rsmt2` — https://docs.rs/rsmt2
