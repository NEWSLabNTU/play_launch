# Shared Scheduling Crate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `ros-launch-manifest-sched`, a portable scheduling-spec crate shared by `play_launch` (Linux RT) and `nano-ros` (RTOS), and wire `play_launch check` to validate it.

**Architecture:** A new pure-host crate in the `src/ros-launch-manifest/` workspace defines a generic tier/deadline/binding schema (no priority numbers) plus per-platform placement sub-tables, and a selector-based resolver that maps nodes to tiers using launch-scope info. `play_launch` links it (validate-now: parse + resolve for `posix` + report; apply-later is a documented phase-2 hook). `nano-ros` migration is a separate follow-up plan.

**Tech Stack:** Rust (edition 2024), `serde`, `thiserror`, `toml`. No `no_std`, no runtime deps, and crucially NO dependency on `play_launch_parser` (it embeds CPython).

## Global Constraints

- Rust edition 2024; workspace `resolver = "3"` (matches `src/ros-launch-manifest/Cargo.toml`).
- New crate deps limited to `serde` (derive), `thiserror`, `toml`. It MUST NOT depend on `play_launch_parser`, `ros-launch-manifest-types`, or `ros-launch-manifest-check`.
- Pure host code — no `no_std`, no runtime/async deps (same constraint `nros-orchestration-ir` satisfies).
- On-disk format is **TOML**. Linux-RT platform key is exactly **`posix`** (no `linux` key).
- Tier priorities are `i64` (admits Zephyr negative coop priorities).
- The generic tier head carries NO priority number; `#[serde(deny_unknown_fields)]` enforces this.
- Design doc of record: `docs/superpowers/specs/2026-07-01-shared-scheduling-crate-design.md`.
- Build/test: use `cargo` directly for this crate (`cargo test -p ros-launch-manifest-sched`) since the manifest workspace is standalone. Do NOT use `colcon`/`just build` for the crate itself.

---

### Task 1: Scaffold the `ros-launch-manifest-sched` crate

**Files:**
- Create: `src/ros-launch-manifest/sched/Cargo.toml`
- Create: `src/ros-launch-manifest/sched/src/lib.rs`
- Create: `src/ros-launch-manifest/sched/src/types.rs`
- Create: `src/ros-launch-manifest/sched/src/parse.rs`
- Create: `src/ros-launch-manifest/sched/src/resolve.rs`
- Modify: `src/ros-launch-manifest/Cargo.toml` (add `"sched"` to `members`)

**Interfaces:**
- Produces: a buildable crate `ros-launch-manifest-sched` with empty modules `types`, `parse`, `resolve`.

- [ ] **Step 1: Create `sched/Cargo.toml`**

```toml
[package]
name = "ros-launch-manifest-sched"
version = "0.1.0"
edition = "2024"

[dependencies]
serde = { version = "1", features = ["derive"] }
thiserror = "2"
toml = "0.8"
```

- [ ] **Step 2: Add crate to the workspace members**

Modify `src/ros-launch-manifest/Cargo.toml`:

```toml
[workspace]
resolver = "3"
members = ["types", "check", "sched"]

[workspace.package]
edition = "2024"
```

- [ ] **Step 3: Create module stubs**

`src/ros-launch-manifest/sched/src/lib.rs`:

```rust
//! Shared, portable scheduling spec for launch-based systems.
//!
//! Consumed by both `play_launch` (Linux RT, target `posix`) and `nano-ros`
//! (RTOS targets). The generic layer (tier class, deadline, binding) is
//! platform-independent and carries no priority numbers; per-platform
//! placement lives in `[tiers.<name>.<target>]` sub-tables.

pub mod parse;
pub mod resolve;
pub mod types;

pub use parse::parse_system_sched;
pub use resolve::{DEFAULT_TIER, ResolvedTier, ResolvedTierTable, SchedError, SchedNode, resolve};
pub use types::{AssignRule, SystemSched, TierDef, TierPlatformSpec};
```

`src/ros-launch-manifest/sched/src/types.rs`:

```rust
//! On-disk scheduling schema (TOML).
```

`src/ros-launch-manifest/sched/src/parse.rs`:

```rust
//! TOML parsing for the system scheduling document.
```

`src/ros-launch-manifest/sched/src/resolve.rs`:

```rust
//! Selector-based tier resolution.
```

- [ ] **Step 4: Verify it builds**

Run: `cargo build -p ros-launch-manifest-sched --manifest-path src/ros-launch-manifest/Cargo.toml`
Expected: compiles (unused-module warnings are fine; the `pub use` lines will fail until Task 2/3 add the items — so for THIS step, temporarily comment the `pub use` lines, build, then uncomment before committing? No — instead keep the `pub use` out until items exist).

Correction: in Step 3, `lib.rs` should NOT re-export items that don't exist yet. Use this `lib.rs` for Task 1:

```rust
//! Shared, portable scheduling spec for launch-based systems.
pub mod parse;
pub mod resolve;
pub mod types;
```

Then re-run `cargo build -p ros-launch-manifest-sched --manifest-path src/ros-launch-manifest/Cargo.toml`
Expected: compiles cleanly.

- [ ] **Step 5: Commit**

```bash
git add src/ros-launch-manifest/Cargo.toml src/ros-launch-manifest/sched
git commit -m "feat(sched): scaffold ros-launch-manifest-sched crate"
```

---

### Task 2: Schema types + TOML parsing

**Files:**
- Modify: `src/ros-launch-manifest/sched/src/types.rs`
- Modify: `src/ros-launch-manifest/sched/src/parse.rs`
- Modify: `src/ros-launch-manifest/sched/src/resolve.rs` (add `SchedError` here so `parse` can use it)
- Modify: `src/ros-launch-manifest/sched/src/lib.rs` (add re-exports)

**Interfaces:**
- Produces:
  - `SystemSched { tiers: BTreeMap<String, TierDef>, assign: Vec<AssignRule> }`
  - `TierDef` with generic head fields + `posix/freertos/zephyr/threadx/nuttx: Option<TierPlatformSpec>` and `fn platform(&self, target: &str) -> Option<&TierPlatformSpec>`
  - `TierPlatformSpec { priority: i64, stack_bytes, core, sched_class, preempt_threshold, deadline_us }`
  - `AssignRule { tier: String, nodes: Vec<String>, scope: Option<String> }`
  - `SchedError` (thiserror) with variant `Parse(String)` (more variants added in Task 3/4)
  - `parse_system_sched(&str) -> Result<SystemSched, SchedError>`

- [ ] **Step 1: Write the schema types**

`src/ros-launch-manifest/sched/src/types.rs`:

```rust
//! On-disk scheduling schema (TOML).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// Top-level system scheduling document.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemSched {
    /// Symbolic tiers keyed by name.
    #[serde(default)]
    pub tiers: BTreeMap<String, TierDef>,
    /// Sparse node→tier binding rules.
    #[serde(default)]
    pub assign: Vec<AssignRule>,
}

/// `[tiers.<name>]` — the generic (portable) head plus per-platform sub-tables.
/// The head carries NO priority number; placement lives in the sub-tables.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TierDef {
    /// `best_effort | real_time | time_triggered | interrupt`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub class: Option<String>,
    /// Generic relative deadline (µs); a platform sub-table may tighten it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deadline_us: Option<u64>,
    /// Callback period (µs) for periodic / time_triggered.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub period_us: Option<u64>,
    /// Execution-time budget (µs) — EDF/sporadic.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub budget_us: Option<u64>,
    /// `ignore | warn | skip | fault`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deadline_policy: Option<String>,
    /// Executor spin period (µs).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spin_period_us: Option<u64>,

    // Per-platform placement sub-tables. `posix` is Linux RT.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub posix: Option<TierPlatformSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub freertos: Option<TierPlatformSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zephyr: Option<TierPlatformSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub threadx: Option<TierPlatformSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nuttx: Option<TierPlatformSpec>,
}

impl TierDef {
    /// Pick the placement sub-table for a resolve target.
    pub fn platform(&self, target: &str) -> Option<&TierPlatformSpec> {
        match target {
            "posix" | "native" => self.posix.as_ref(),
            "freertos" => self.freertos.as_ref(),
            "zephyr" => self.zephyr.as_ref(),
            "threadx" => self.threadx.as_ref(),
            "nuttx" => self.nuttx.as_ref(),
            _ => None,
        }
    }
}

/// `[tiers.<name>.<target>]` — concrete per-platform placement. Same shape on
/// every platform. `priority` is `i64` to admit Zephyr negative coop priorities.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TierPlatformSpec {
    pub priority: i64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stack_bytes: Option<u32>,
    /// CPU core to pin to (SMP); `None` ⇒ unpinned.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub core: Option<u32>,
    /// POSIX scheduler class (e.g. `"SCHED_FIFO"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sched_class: Option<String>,
    /// ThreadX preemption threshold (ignored elsewhere).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preempt_threshold: Option<i64>,
    /// Per-platform deadline override (µs), tighter than the generic head.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deadline_us: Option<u64>,
}

/// A sparse binding rule. A node matches if it is named in `nodes` OR falls
/// under `scope`. Explicit `nodes` matches win over `scope` matches.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AssignRule {
    pub tier: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub nodes: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scope: Option<String>,
}
```

- [ ] **Step 2: Add `SchedError` (Parse variant only for now)**

`src/ros-launch-manifest/sched/src/resolve.rs`:

```rust
//! Selector-based tier resolution.

/// Errors from parsing or resolving the scheduling spec.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SchedError {
    #[error("failed to parse system scheduling TOML: {0}")]
    Parse(String),
}
```

- [ ] **Step 3: Write the parse function**

`src/ros-launch-manifest/sched/src/parse.rs`:

```rust
//! TOML parsing for the system scheduling document.

use crate::resolve::SchedError;
use crate::types::SystemSched;

/// Parse a system scheduling document from a TOML string.
pub fn parse_system_sched(input: &str) -> Result<SystemSched, SchedError> {
    toml::from_str(input).map_err(|e| SchedError::Parse(e.to_string()))
}
```

- [ ] **Step 4: Add re-exports to `lib.rs`**

Replace `lib.rs` body with:

```rust
//! Shared, portable scheduling spec for launch-based systems.
pub mod parse;
pub mod resolve;
pub mod types;

pub use parse::parse_system_sched;
pub use resolve::SchedError;
pub use types::{AssignRule, SystemSched, TierDef, TierPlatformSpec};
```

- [ ] **Step 5: Write the failing tests**

Append to `src/ros-launch-manifest/sched/src/parse.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_generic_head_and_platform_subtable() {
        let src = r#"
[tiers.control]
class = "real_time"
deadline_us = 50000
period_us = 20000

[tiers.control.posix]
priority = 80
sched_class = "SCHED_FIFO"
core = 1

[tiers.control.freertos]
priority = 12
stack_bytes = 8192
deadline_us = 40000

[[assign]]
tier = "control"
nodes = ["ndt_localizer", "ekf_localizer"]

[[assign]]
tier = "perception"
scope = "/perception/lidar"
"#;
        let sched = parse_system_sched(src).expect("must parse");
        let control = sched.tiers.get("control").expect("control tier");
        assert_eq!(control.class.as_deref(), Some("real_time"));
        assert_eq!(control.deadline_us, Some(50000));
        assert_eq!(control.platform("posix").unwrap().priority, 80);
        assert_eq!(
            control.platform("posix").unwrap().sched_class.as_deref(),
            Some("SCHED_FIFO")
        );
        assert_eq!(control.platform("freertos").unwrap().deadline_us, Some(40000));
        assert_eq!(sched.assign.len(), 2);
        assert_eq!(sched.assign[0].nodes, vec!["ndt_localizer", "ekf_localizer"]);
        assert_eq!(sched.assign[1].scope.as_deref(), Some("/perception/lidar"));
    }

    #[test]
    fn priority_number_on_generic_head_is_rejected() {
        // deny_unknown_fields: a bare `priority` at the tier head must error —
        // this is what protects portability.
        let src = r#"
[tiers.control]
priority = 80
"#;
        let err = parse_system_sched(src).unwrap_err();
        assert!(matches!(err, SchedError::Parse(_)));
    }

    #[test]
    fn empty_document_is_valid_and_default() {
        let sched = parse_system_sched("").expect("empty parses");
        assert!(sched.tiers.is_empty());
        assert!(sched.assign.is_empty());
    }
}
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `cargo test -p ros-launch-manifest-sched --manifest-path src/ros-launch-manifest/Cargo.toml`
Expected: 3 tests pass.

- [ ] **Step 7: Commit**

```bash
git add src/ros-launch-manifest/sched src/ros-launch-manifest/sched/src/lib.rs
git commit -m "feat(sched): schema types + TOML parsing"
```

---

### Task 3: Selector binding (`bind_nodes`)

**Files:**
- Modify: `src/ros-launch-manifest/sched/src/resolve.rs`

**Interfaces:**
- Consumes: `AssignRule` (Task 2).
- Produces:
  - `SchedNode { name: String, scope: String }` — `name` is a fully-qualified node name, `scope` is the node's namespace path.
  - `const DEFAULT_TIER: &str = "default"`
  - `SchedError` gains `UnknownNodeSelector`, `UnknownScopeSelector`, `NodeMatchedByMultipleTiers`
  - `pub(crate) fn bind_nodes(assigns: &[AssignRule], nodes: &[SchedNode]) -> Result<BTreeMap<String, Vec<String>>, SchedError>` — returns tier-name → sorted member node names, with all unmatched nodes under `DEFAULT_TIER`. Two-pass precedence: explicit `nodes` selectors first, then `scope` selectors fill only still-unassigned nodes.

- [ ] **Step 1: Extend `SchedError` and add `SchedNode` + `DEFAULT_TIER`**

Replace the `SchedError` enum in `resolve.rs` and add types above it:

```rust
use std::collections::BTreeMap;

use crate::types::AssignRule;

/// The synthesized tier for nodes matched by no assign rule.
pub const DEFAULT_TIER: &str = "default";

/// One node as seen by the resolver — dependency-free (no parser types).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SchedNode {
    /// Fully-qualified node name (e.g. `/perception/lidar/ndt_localizer`).
    pub name: String,
    /// The node's namespace / scope path (e.g. `/perception/lidar`).
    pub scope: String,
}

/// Errors from parsing or resolving the scheduling spec.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SchedError {
    #[error("failed to parse system scheduling TOML: {0}")]
    Parse(String),
    #[error("assign rule node selector `{selector}` matches no node in the system")]
    UnknownNodeSelector { selector: String },
    #[error("assign rule scope selector `{selector}` matches no node's scope in the system")]
    UnknownScopeSelector { selector: String },
    #[error(
        "node `{node}` is matched by two tiers (`{tier_a}` and `{tier_b}`); \
         a node must belong to exactly one tier"
    )]
    NodeMatchedByMultipleTiers {
        node: String,
        tier_a: String,
        tier_b: String,
    },
}
```

- [ ] **Step 2: Write the selector matchers and `bind_nodes`**

Append to `resolve.rs`:

```rust
/// Normalize a namespace/scope path: ensure a single leading slash, no trailing.
fn norm_scope(s: &str) -> String {
    let trimmed = s.trim_matches('/');
    if trimmed.is_empty() {
        "/".to_string()
    } else {
        format!("/{trimmed}")
    }
}

/// A node selector matches a node by full name or by its bare (last-segment) name.
fn node_selector_matches(selector: &str, node: &SchedNode) -> bool {
    node.name == selector || node.name.rsplit('/').next() == Some(selector)
}

/// A scope selector matches a node whose scope equals it or is a descendant.
fn scope_selector_matches(selector: &str, node: &SchedNode) -> bool {
    let sel = norm_scope(selector);
    let ns = norm_scope(&node.scope);
    ns == sel || ns.starts_with(&format!("{}/", sel.trim_end_matches('/')))
}

/// Insert a node→tier assignment; error if it conflicts with a prior one.
fn assign_one<'a>(
    map: &mut BTreeMap<String, &'a str>,
    node: &str,
    tier: &'a str,
) -> Result<(), SchedError> {
    if let Some(prev) = map.insert(node.to_string(), tier) {
        if prev != tier {
            return Err(SchedError::NodeMatchedByMultipleTiers {
                node: node.to_string(),
                tier_a: prev.to_string(),
                tier_b: tier.to_string(),
            });
        }
    }
    Ok(())
}

/// Build tier-name → sorted member node names. Unmatched nodes → `DEFAULT_TIER`.
pub(crate) fn bind_nodes(
    assigns: &[AssignRule],
    nodes: &[SchedNode],
) -> Result<BTreeMap<String, Vec<String>>, SchedError> {
    let mut node_tier: BTreeMap<String, &str> = BTreeMap::new();

    // Pass 1: explicit node selectors (highest precedence).
    for rule in assigns {
        for sel in &rule.nodes {
            let hits: Vec<&SchedNode> =
                nodes.iter().filter(|n| node_selector_matches(sel, n)).collect();
            if hits.is_empty() {
                return Err(SchedError::UnknownNodeSelector {
                    selector: sel.clone(),
                });
            }
            for n in hits {
                assign_one(&mut node_tier, &n.name, &rule.tier)?;
            }
        }
    }

    // Pass 2: scope selectors — fill only nodes still unassigned by pass 1.
    for rule in assigns {
        if let Some(scope_sel) = &rule.scope {
            let hits: Vec<&SchedNode> = nodes
                .iter()
                .filter(|n| scope_selector_matches(scope_sel, n))
                .collect();
            if hits.is_empty() {
                return Err(SchedError::UnknownScopeSelector {
                    selector: scope_sel.clone(),
                });
            }
            for n in hits {
                if node_tier.contains_key(&n.name) {
                    continue; // explicit node rule wins
                }
                assign_one(&mut node_tier, &n.name, &rule.tier)?;
            }
        }
    }

    // Collect members, defaulting the unmatched.
    let mut members: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for n in nodes {
        let tier = node_tier
            .get(&n.name)
            .copied()
            .unwrap_or(DEFAULT_TIER)
            .to_string();
        members.entry(tier).or_default().push(n.name.clone());
    }
    for v in members.values_mut() {
        v.sort();
    }
    Ok(members)
}
```

- [ ] **Step 3: Write failing tests**

Append to `resolve.rs`:

```rust
#[cfg(test)]
mod bind_tests {
    use super::*;

    fn node(name: &str, scope: &str) -> SchedNode {
        SchedNode {
            name: name.to_string(),
            scope: scope.to_string(),
        }
    }

    fn rule_nodes(tier: &str, names: &[&str]) -> AssignRule {
        AssignRule {
            tier: tier.to_string(),
            nodes: names.iter().map(|s| s.to_string()).collect(),
            scope: None,
        }
    }

    fn rule_scope(tier: &str, scope: &str) -> AssignRule {
        AssignRule {
            tier: tier.to_string(),
            nodes: vec![],
            scope: Some(scope.to_string()),
        }
    }

    #[test]
    fn unmatched_nodes_go_to_default() {
        let nodes = vec![node("/a", "/"), node("/b", "/")];
        let m = bind_nodes(&[], &nodes).unwrap();
        assert_eq!(m.len(), 1);
        assert_eq!(m[DEFAULT_TIER], vec!["/a".to_string(), "/b".to_string()]);
    }

    #[test]
    fn explicit_node_selector_by_bare_name() {
        let nodes = vec![node("/ns/ndt_localizer", "/ns"), node("/ns/other", "/ns")];
        let m = bind_nodes(&[rule_nodes("control", &["ndt_localizer"])], &nodes).unwrap();
        assert_eq!(m["control"], vec!["/ns/ndt_localizer".to_string()]);
        assert_eq!(m[DEFAULT_TIER], vec!["/ns/other".to_string()]);
    }

    #[test]
    fn scope_selector_matches_subtree() {
        let nodes = vec![
            node("/perception/lidar/a", "/perception/lidar"),
            node("/perception/lidar/deep/b", "/perception/lidar/deep"),
            node("/control/c", "/control"),
        ];
        let m = bind_nodes(&[rule_scope("perception", "/perception/lidar")], &nodes).unwrap();
        assert_eq!(
            m["perception"],
            vec![
                "/perception/lidar/a".to_string(),
                "/perception/lidar/deep/b".to_string()
            ]
        );
        assert_eq!(m[DEFAULT_TIER], vec!["/control/c".to_string()]);
    }

    #[test]
    fn explicit_node_wins_over_scope() {
        let nodes = vec![node("/p/a", "/p"), node("/p/b", "/p")];
        let assigns = vec![
            rule_scope("bg", "/p"),
            rule_nodes("control", &["/p/a"]),
        ];
        let m = bind_nodes(&assigns, &nodes).unwrap();
        assert_eq!(m["control"], vec!["/p/a".to_string()]);
        assert_eq!(m["bg"], vec!["/p/b".to_string()]);
    }

    #[test]
    fn conflicting_node_rules_error() {
        let nodes = vec![node("/a", "/")];
        let assigns = vec![rule_nodes("hi", &["/a"]), rule_nodes("lo", &["/a"])];
        let err = bind_nodes(&assigns, &nodes).unwrap_err();
        assert!(matches!(err, SchedError::NodeMatchedByMultipleTiers { .. }));
    }

    #[test]
    fn unknown_node_selector_errors() {
        let nodes = vec![node("/a", "/")];
        let err = bind_nodes(&[rule_nodes("hi", &["ghost"])], &nodes).unwrap_err();
        assert!(matches!(err, SchedError::UnknownNodeSelector { .. }));
    }

    #[test]
    fn unknown_scope_selector_errors() {
        let nodes = vec![node("/a", "/")];
        let err = bind_nodes(&[rule_scope("hi", "/nowhere")], &nodes).unwrap_err();
        assert!(matches!(err, SchedError::UnknownScopeSelector { .. }));
    }
}
```

- [ ] **Step 4: Run tests**

Run: `cargo test -p ros-launch-manifest-sched --manifest-path src/ros-launch-manifest/Cargo.toml bind_tests`
Expected: 7 `bind_tests` pass (plus the 3 parse tests still pass).

- [ ] **Step 5: Commit**

```bash
git add src/ros-launch-manifest/sched/src/resolve.rs
git commit -m "feat(sched): selector-based node→tier binding"
```

---

### Task 4: Full resolver (`resolve` → `ResolvedTierTable`)

**Files:**
- Modify: `src/ros-launch-manifest/sched/src/resolve.rs`
- Modify: `src/ros-launch-manifest/sched/src/lib.rs` (export new items)

**Interfaces:**
- Consumes: `bind_nodes` (Task 3), `TierDef`/`TierPlatformSpec` (Task 2).
- Produces:
  - `ResolvedTier { name, priority: i64, stack_bytes, core, sched_class, preempt_threshold, class, period_us, budget_us, deadline_us, deadline_policy, spin_period_us, members: Vec<String> }` (effective `deadline_us` = platform override else generic head).
  - `ResolvedTierTable { tiers: Vec<ResolvedTier> }` with `fn is_single_tier(&self) -> bool`.
  - `SchedError` gains `UnknownTier` and `MissingPlatformSpec`.
  - `pub fn resolve(tiers: &BTreeMap<String, TierDef>, assigns: &[AssignRule], nodes: &[SchedNode], target: &str) -> Result<ResolvedTierTable, SchedError>` — tiers ordered highest-`priority` first (ties broken by name).

- [ ] **Step 1: Add the two new error variants**

Add to the `SchedError` enum in `resolve.rs`:

```rust
    #[error("assign rule references tier `{tier}`, which has no [tiers.{tier}] definition")]
    UnknownTier { tier: String },
    #[error("tier `{tier}` has no [tiers.{tier}.{target}] sub-table for the resolve target")]
    MissingPlatformSpec { tier: String, target: String },
```

- [ ] **Step 2: Add the resolved types**

Append to `resolve.rs`:

```rust
use crate::types::TierDef;

/// One resolved tier: generic policy + concrete placement for one target.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ResolvedTier {
    pub name: String,
    pub priority: i64,
    pub stack_bytes: Option<u32>,
    pub core: Option<u32>,
    pub sched_class: Option<String>,
    pub preempt_threshold: Option<i64>,
    pub class: Option<String>,
    pub period_us: Option<u64>,
    pub budget_us: Option<u64>,
    /// Effective deadline: platform override if present, else the generic head.
    pub deadline_us: Option<u64>,
    pub deadline_policy: Option<String>,
    pub spin_period_us: Option<u64>,
    /// Member node names, sorted.
    pub members: Vec<String>,
}

impl ResolvedTier {
    fn default_tier(members: Vec<String>) -> Self {
        ResolvedTier {
            name: DEFAULT_TIER.to_string(),
            members,
            ..Default::default()
        }
    }
}

/// The ordered tier table for one target.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ResolvedTierTable {
    /// Highest RTOS/OS priority first.
    pub tiers: Vec<ResolvedTier>,
}

impl ResolvedTierTable {
    /// True when the whole system collapsed to the single synthesized default.
    pub fn is_single_tier(&self) -> bool {
        self.tiers.len() == 1 && self.tiers[0].name == DEFAULT_TIER
    }
}
```

- [ ] **Step 3: Write `resolve`**

Append to `resolve.rs`:

```rust
/// Resolve tiers + assign rules against the system's nodes for one target.
pub fn resolve(
    tiers: &BTreeMap<String, TierDef>,
    assigns: &[AssignRule],
    nodes: &[SchedNode],
    target: &str,
) -> Result<ResolvedTierTable, SchedError> {
    // Every assign rule must name a real tier (the synthesized default excepted).
    for rule in assigns {
        if rule.tier != DEFAULT_TIER && !tiers.contains_key(&rule.tier) {
            return Err(SchedError::UnknownTier {
                tier: rule.tier.clone(),
            });
        }
    }

    let members_by_tier = bind_nodes(assigns, nodes)?;

    let mut out: Vec<ResolvedTier> = Vec::with_capacity(members_by_tier.len());
    for (name, members) in members_by_tier {
        // The default tier needs no [tiers.default] table.
        if name == DEFAULT_TIER && !tiers.contains_key(DEFAULT_TIER) {
            out.push(ResolvedTier::default_tier(members));
            continue;
        }
        let def = tiers
            .get(&name)
            .ok_or_else(|| SchedError::UnknownTier { tier: name.clone() })?;
        let spec = def.platform(target).ok_or_else(|| SchedError::MissingPlatformSpec {
            tier: name.clone(),
            target: target.to_string(),
        })?;
        out.push(ResolvedTier {
            name,
            priority: spec.priority,
            stack_bytes: spec.stack_bytes,
            core: spec.core,
            sched_class: spec.sched_class.clone(),
            preempt_threshold: spec.preempt_threshold,
            class: def.class.clone(),
            period_us: def.period_us,
            budget_us: def.budget_us,
            deadline_us: spec.deadline_us.or(def.deadline_us),
            deadline_policy: def.deadline_policy.clone(),
            spin_period_us: def.spin_period_us,
            members,
        });
    }

    // Highest priority first; ties by name for determinism.
    out.sort_by(|a, b| b.priority.cmp(&a.priority).then(a.name.cmp(&b.name)));
    Ok(ResolvedTierTable { tiers: out })
}
```

- [ ] **Step 4: Export the new items in `lib.rs`**

Set `lib.rs` re-exports to:

```rust
//! Shared, portable scheduling spec for launch-based systems.
pub mod parse;
pub mod resolve;
pub mod types;

pub use parse::parse_system_sched;
pub use resolve::{
    DEFAULT_TIER, ResolvedTier, ResolvedTierTable, SchedError, SchedNode, resolve,
};
pub use types::{AssignRule, SystemSched, TierDef, TierPlatformSpec};
```

- [ ] **Step 5: Write failing tests**

Append to `resolve.rs`:

```rust
#[cfg(test)]
mod resolve_tests {
    use super::*;
    use crate::parse::parse_system_sched;

    fn node(name: &str, scope: &str) -> SchedNode {
        SchedNode {
            name: name.to_string(),
            scope: scope.to_string(),
        }
    }

    #[test]
    fn no_assigns_degenerates_to_single_default_tier() {
        let sched = parse_system_sched("").unwrap();
        let table = resolve(&sched.tiers, &sched.assign, &[node("/a", "/")], "posix").unwrap();
        assert!(table.is_single_tier());
        assert_eq!(table.tiers[0].members, vec!["/a".to_string()]);
        assert_eq!(table.tiers[0].priority, 0);
    }

    #[test]
    fn resolves_platform_numbers_and_effective_deadline() {
        let src = r#"
[tiers.control]
class = "real_time"
deadline_us = 50000

[tiers.control.posix]
priority = 80
sched_class = "SCHED_FIFO"
core = 1
deadline_us = 40000

[[assign]]
tier = "control"
nodes = ["/a"]
"#;
        let sched = parse_system_sched(src).unwrap();
        let table = resolve(&sched.tiers, &sched.assign, &[node("/a", "/")], "posix").unwrap();
        let t = table.tiers.iter().find(|t| t.name == "control").unwrap();
        assert_eq!(t.priority, 80);
        assert_eq!(t.core, Some(1));
        assert_eq!(t.class.as_deref(), Some("real_time"));
        assert_eq!(t.deadline_us, Some(40000)); // platform override wins
        assert_eq!(t.members, vec!["/a".to_string()]);
    }

    #[test]
    fn orders_tiers_highest_priority_first() {
        let src = r#"
[tiers.hi.posix]
priority = 80
[tiers.lo.posix]
priority = 10

[[assign]]
tier = "hi"
nodes = ["/a"]
[[assign]]
tier = "lo"
nodes = ["/b"]
"#;
        let sched = parse_system_sched(src).unwrap();
        let nodes = vec![node("/a", "/"), node("/b", "/")];
        let table = resolve(&sched.tiers, &sched.assign, &nodes, "posix").unwrap();
        assert_eq!(table.tiers[0].name, "hi");
        assert_eq!(table.tiers[0].priority, 80);
        assert_eq!(table.tiers[1].name, "lo");
    }

    #[test]
    fn unknown_tier_in_assign_errors() {
        let src = r#"
[[assign]]
tier = "ghost"
nodes = ["/a"]
"#;
        let sched = parse_system_sched(src).unwrap();
        let err = resolve(&sched.tiers, &sched.assign, &[node("/a", "/")], "posix").unwrap_err();
        assert!(matches!(err, SchedError::UnknownTier { .. }));
    }

    #[test]
    fn missing_platform_spec_errors() {
        let src = r#"
[tiers.hi.freertos]
priority = 5

[[assign]]
tier = "hi"
nodes = ["/a"]
"#;
        let sched = parse_system_sched(src).unwrap();
        // resolving for posix must fail: only freertos declared.
        let err = resolve(&sched.tiers, &sched.assign, &[node("/a", "/")], "posix").unwrap_err();
        assert!(matches!(err, SchedError::MissingPlatformSpec { .. }));
    }
}
```

- [ ] **Step 6: Run tests**

Run: `cargo test -p ros-launch-manifest-sched --manifest-path src/ros-launch-manifest/Cargo.toml`
Expected: all tests pass (parse 3 + bind 7 + resolve 5 = 15).

- [ ] **Step 7: Commit**

```bash
git add src/ros-launch-manifest/sched/src/resolve.rs src/ros-launch-manifest/sched/src/lib.rs
git commit -m "feat(sched): full resolver with platform lowering + ordering"
```

---

### Task 5: Wire `play_launch check` to validate scheduling (Linux, validate-now)

**Files:**
- Modify: `src/play_launch/Cargo.toml` (add path dep on `ros-launch-manifest-sched`)
- Create: `src/play_launch/src/ros/sched_loader.rs` (dump→`SchedNode` mapping + load+resolve+report)
- Modify: `src/play_launch/src/ros/mod.rs` (register module) — confirm the module list file; it is the `mod.rs` under `src/play_launch/src/ros/`
- Modify: `src/play_launch/src/cli/options.rs` (add `--sched <PATH>` to `CheckArgs`)
- Modify: `src/play_launch/src/commands/manifest.rs` (call the loader when `--sched` given)
- Test: `src/play_launch/src/ros/sched_loader.rs` (unit test for the mapping)

**Interfaces:**
- Consumes: `LaunchDump` (`crate::ros::launch_dump::{LaunchDump, ScopeEntry, NodeRecord, NodeContainerRecord}`), and `ros_launch_manifest_sched::{SchedNode, SystemSched, ResolvedTierTable, parse_system_sched, resolve}`.
- Produces:
  - `pub fn sched_nodes_from_dump(dump: &LaunchDump) -> Vec<SchedNode>`
  - `pub fn check_sched(dump: &LaunchDump, sched_path: &std::path::Path) -> eyre::Result<()>` — loads the TOML, resolves for `"posix"`, prints the resolved table, and returns an error (surfaced as a diagnostic) on `SchedError`.

- [ ] **Step 1: Add the path dependency**

In `src/play_launch/Cargo.toml`, under `[dependencies]` (next to the existing `ros-launch-manifest-*` path deps):

```toml
ros-launch-manifest-sched = { path = "../ros-launch-manifest/sched" }
```

- [ ] **Step 2: Write the dump→SchedNode mapping with a failing test first**

Create `src/play_launch/src/ros/sched_loader.rs`:

```rust
//! Load + validate the shared scheduling spec against a parsed launch dump.
//!
//! Linux is "validate now, apply later": we resolve for the `posix` target and
//! report, but do not (yet) apply `sched_setscheduler`/affinity to processes.

use std::collections::HashMap;
use std::path::Path;

use eyre::{Result, WrapErr};
use ros_launch_manifest_sched::{ResolvedTierTable, SchedNode, parse_system_sched, resolve};

use crate::ros::launch_dump::LaunchDump;

/// The `posix` target key — Linux RT placement sub-table.
const TARGET: &str = "posix";

/// Resolve a scope id to its namespace path (falls back to `/`).
fn scope_ns(dump: &LaunchDump, scope: Option<usize>) -> String {
    let by_id: HashMap<usize, &str> =
        dump.scopes.iter().map(|s| (s.id, s.ns.as_str())).collect();
    scope
        .and_then(|id| by_id.get(&id).copied())
        .unwrap_or("/")
        .to_string()
}

/// Join a namespace with a bare node name into a fully-qualified name.
fn join_fqn(ns: &str, bare: &str) -> String {
    if bare.starts_with('/') {
        bare.to_string()
    } else if ns == "/" || ns.is_empty() {
        format!("/{bare}")
    } else {
        format!("{}/{bare}", ns.trim_end_matches('/'))
    }
}

/// Flatten a launch dump into the resolver's dependency-free node view.
/// Regular nodes and containers are both scheduled units on Linux.
pub fn sched_nodes_from_dump(dump: &LaunchDump) -> Vec<SchedNode> {
    let mut out = Vec::new();
    for n in &dump.node {
        let bare = n
            .name
            .clone()
            .or_else(|| n.exec_name.clone())
            .unwrap_or_default();
        if bare.is_empty() {
            continue;
        }
        let ns = scope_ns(dump, n.scope);
        out.push(SchedNode {
            name: join_fqn(&ns, &bare),
            scope: ns,
        });
    }
    for c in &dump.container {
        let ns = scope_ns(dump, c.scope);
        out.push(SchedNode {
            name: join_fqn(&ns, &c.name),
            scope: ns,
        });
    }
    out
}

/// Load the scheduling TOML, resolve for `posix`, and print the resolved table.
pub fn check_sched(dump: &LaunchDump, sched_path: &Path) -> Result<()> {
    let text = std::fs::read_to_string(sched_path)
        .wrap_err_with(|| format!("failed to read {}", sched_path.display()))?;
    let sched = parse_system_sched(&text)
        .map_err(|e| eyre::eyre!("scheduling spec error: {e}"))?;
    let nodes = sched_nodes_from_dump(dump);
    let table: ResolvedTierTable = resolve(&sched.tiers, &sched.assign, &nodes, TARGET)
        .map_err(|e| eyre::eyre!("scheduling resolve error: {e}"))?;

    eprintln!("Scheduling ({TARGET}): {} tier(s)", table.tiers.len());
    for t in &table.tiers {
        eprintln!(
            "  tier {:<16} prio={:<4} sched_class={:<10} core={:<4} members={}",
            t.name,
            t.priority,
            t.sched_class.as_deref().unwrap_or("-"),
            t.core.map(|c| c.to_string()).unwrap_or_else(|| "-".into()),
            t.members.len(),
        );
        for m in &t.members {
            eprintln!("      {m}");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::launch_dump::{NodeRecord, ScopeEntry};

    // Minimal NodeRecord/ScopeEntry construction: use serde_json to avoid
    // enumerating every field of these large records by hand.
    fn dump_with_one_node() -> LaunchDump {
        let json = serde_json::json!({
            "node": [{
                "name": "ndt_localizer",
                "exec_name": "ndt_localizer",
                "scope": 1
            }],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null},
                {"id": 1, "ns": "/localization", "parent": 0}
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    #[test]
    fn maps_node_scope_to_fqn() {
        let dump = dump_with_one_node();
        let nodes = sched_nodes_from_dump(&dump);
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].name, "/localization/ndt_localizer");
        assert_eq!(nodes[0].scope, "/localization");
        // Silence unused-import lint in case NodeRecord/ScopeEntry drift.
        let _ = (std::any::type_name::<NodeRecord>(), std::any::type_name::<ScopeEntry>());
    }
}
```

Note for the implementer: the JSON in the test must satisfy `NodeRecord`/`ScopeEntry`/`LaunchDump` serde requirements. If deserialization fails on a missing required field, add that field to the JSON with a null/empty value (check `src/play_launch/src/ros/launch_dump.rs` for `#[serde(default)]` vs required). `NodeRecord.name` and `exec_name` are `Option<String>`; `scope` is `Option<usize>`; `ScopeEntry` requires `id`, `ns`, `parent`.

- [ ] **Step 3: Register the module**

In `src/play_launch/src/ros/mod.rs`, add alongside the other `pub mod` lines:

```rust
pub mod sched_loader;
```

- [ ] **Step 4: Run the mapping unit test to verify it passes**

Run: `cargo build -p play_launch` first is unnecessary; run the test via colcon's cargo is not needed. Use:
`cargo test -p play_launch --manifest-path src/play_launch/Cargo.toml sched_loader`
Expected: `maps_node_scope_to_fqn` passes. If the crate does not build standalone (workspace deps on generated bindings), instead run `just build-rust` then the equivalent nextest filter — but prefer the direct cargo test if it compiles.

- [ ] **Step 5: Add the `--sched` CLI flag**

In `src/play_launch/src/cli/options.rs`, add a field to the `CheckArgs` struct (match the existing clap derive style used by its neighbors, e.g. `manifest_dir`):

```rust
    /// Path to a system scheduling spec (TOML). When given, `check` also
    /// resolves + validates tier assignments for the `posix` (Linux RT) target.
    #[arg(long)]
    pub sched: Option<std::path::PathBuf>,
```

- [ ] **Step 6: Call the loader from the check command**

In `src/play_launch/src/commands/manifest.rs`, inside `handle_check_manifest`, after the `dump` is built (right after the `eprintln!("Parsed: ...")` block) and before manifest loading, add:

```rust
    // Optional: validate the shared scheduling spec (Linux = validate-now).
    if let Some(sched_path) = &args.sched {
        crate::ros::sched_loader::check_sched(&dump, sched_path)?;
    }
```

- [ ] **Step 7: Build play_launch and run its tests**

Run: `just build-rust`
Expected: builds. Then:
Run: `cargo test -p play_launch --manifest-path src/play_launch/Cargo.toml sched_loader`
Expected: pass.

- [ ] **Step 8: Manual smoke check (fixture)**

Create `tmp/system_sched.toml`:

```toml
[tiers.control]
class = "real_time"
[tiers.control.posix]
priority = 80
sched_class = "SCHED_FIFO"

[[assign]]
tier = "control"
scope = "/"
```

Run (pick any small launch fixture that the existing `check` command accepts, e.g. one under `tests/fixtures/simple_test`):
`just run check <pkg> <launch_file> --sched tmp/system_sched.toml`
Expected: prints `Scheduling (posix): N tier(s)` with a `control` tier listing member nodes. Confirm no resolve error.

- [ ] **Step 9: Commit**

```bash
git add src/play_launch/Cargo.toml src/play_launch/src/ros/sched_loader.rs \
        src/play_launch/src/ros/mod.rs src/play_launch/src/cli/options.rs \
        src/play_launch/src/commands/manifest.rs
git commit -m "feat: play_launch check validates shared scheduling spec (posix)"
```

---

### Task 6: Document the crate + the validate-now / apply-later boundary

**Files:**
- Create: `src/ros-launch-manifest/docs/scheduling.md`
- Modify: `CLAUDE.md` (add a one-line pointer under the manifest section)

**Interfaces:** none (docs only).

- [ ] **Step 1: Write the crate doc**

Create `src/ros-launch-manifest/docs/scheduling.md` describing: the generic-vs-platform split, the TOML schema (copy the block from the design doc §Schema), the selector precedence (explicit node > scope > default), the `posix` target and the phase-2 apply-layer, and the cross-repo sharing model (nano-ros vendors the crate; generic layer byte-identical). Reference `docs/superpowers/specs/2026-07-01-shared-scheduling-crate-design.md` as the design of record.

- [ ] **Step 2: Add the CLAUDE.md pointer**

Under the "Launch File Parsing" / manifest area of `CLAUDE.md`, add:

```markdown
- **Scheduling spec** (`src/ros-launch-manifest/sched/`): portable tier/deadline/binding schema shared with nano-ros. Generic layer carries no priority numbers; per-platform placement in `[tiers.X.<target>]` (posix = Linux RT). `play_launch check --sched <file.toml>` validates for `posix` (validate-now; apply-later is phase 2). See `src/ros-launch-manifest/docs/scheduling.md`.
```

- [ ] **Step 3: Commit**

```bash
git add src/ros-launch-manifest/docs/scheduling.md CLAUDE.md
git commit -m "docs(sched): document scheduling crate + validate-now boundary"
```

---

## Follow-up (separate plan): nano-ros migration

This plan delivers the shared crate + the Linux (`play_launch`) consumer, which is independently working and testable. The `nano-ros` side is a distinct subsystem in a different repo/workspace with its own build + test cycle, so it gets its own plan. Scope of that follow-up:

1. Add `ros-launch-manifest-sched` as a vendored submodule under `packages/cli/third-party/` and pin it in `nros-sdk-index.toml` (mirror the existing `play_launch_parser` / `ros-launch-manifest` entries).
2. Move `TierDef` / `TierRtosSpec` (→ `TierPlatformSpec`) and the resolver out of `packages/core/nros-orchestration-ir` into the shared crate; keep `board_path_for` and re-export the shared types from `nros-orchestration-ir`.
3. Replace the callback-group authoring (`[package.metadata.nros.node].callback_groups` + `[[node_overrides]]`) with the central `[[assign]]` table; feed `resolve` a `Vec<SchedNode>` built from the `record.json` scope table (the CLI already parses it via the parser binary).
4. Point `codegen-system` at the shared `ResolvedTierTable` when baking `nros-plan.json` (one task/executor per tier), targeting `freertos`/`zephyr`/etc.
5. Verify with the existing `nros-cli-core` orchestration tests (`orchestration_e2e.rs`, `orchestration_generate.rs`, `orchestration_schema.rs`) and a cross-repo parity fixture: the same `system.toml` generic fields resolve identically under `posix` (play_launch) and an RTOS target (nano-ros).

---

## Self-Review

**Spec coverage:**
- Crate §Crate → Task 1. Schema §Schema → Task 2. Resolver §Resolver → Tasks 3+4. Validation §Validation (all `SchedError` variants) → Tasks 2/3/4. play_launch consumer §Consumers (validate-now) → Task 5; apply-later documented → Task 6. Distribution/migration §Migration → Follow-up section (separate plan, per scope-check). Testing §Testing → unit tests in Tasks 2-4, integration/mapping test + smoke in Task 5.
- `posix` key, TOML format, no-parser-dependency, i64 priority, deny_unknown_fields portability guard: all captured in Global Constraints and enforced by task code.

**Placeholder scan:** No TBD/TODO. Every code step shows full code. The one soft spot (Task 5 Step 2 test JSON must satisfy serde) carries an explicit implementer note with the exact required fields to check, not a vague "handle it".

**Type consistency:** `SchedNode { name, scope }`, `TierPlatformSpec` (crate) vs `TierRtosSpec` (nano-ros legacy name, only referenced in the migration follow-up), `resolve(tiers, assigns, nodes, target)` signature, `ResolvedTier.deadline_us` effective-value rule, `DEFAULT_TIER` — all consistent across Tasks 2-5 and the `lib.rs` re-exports.
