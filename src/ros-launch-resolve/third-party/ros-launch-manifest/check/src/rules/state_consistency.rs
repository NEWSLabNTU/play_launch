//! Rule: sibling `state:` tagging consistency within a single node
//! (Vocabulary v2, Phase 44.1 §5 — generalized, then noise-gated per the
//! 44.2 review).
//!
//! Motivated by a real Autoware contract bug (Phase 42 study, Q1/Q2):
//! `simple_planning_simulator` tags its `gear_cmd`, `turn_indicators_cmd`,
//! and `hazard_lights_cmd` subscribers `state: true` (all populated by the
//! same lambda-store-then-poll pattern), but leaves its behaviorally
//! identical `control_cmd` subscriber untagged — an inconsistent
//! application of the existing `state:` boolean within one contract file,
//! not a missing schema feature.
//!
//! A subscriber is **unaccounted** when it is neither
//! - tagged `state: true`, **nor**
//! - referenced by any of the node's paths' [`EffectiveTrigger::Input`]
//!   endpoint list (the *effective* trigger — explicit `trigger: {input:
//!   [...]}` or the legacy `input:` derivation both count).
//!
//! Two branches, calibrated against the real 75-file autoware-contract
//! corpus (P44 W2 review finding: the unqualified generalized rule
//! produced 36 warnings, most from nodes that simply hadn't authored any
//! `paths:` yet — migration noise the `explicit-trigger` lint already
//! covers at Info severity — drowning the 2 genuine smells):
//!
//! 1. **Node with ≥1 declared path** (has opted into path modeling): a
//!    single lone unaccounted sub warns — the node modeled its causality
//!    but left exactly one sub unexplained, the motivating-bug shape.
//!    2+ unaccounted subs are treated as a deliberate hybrid design
//!    (e.g. Autoware's `vehicle_cmd_gate`: event-driven for several
//!    independent control-cmd sources — auto/external/emergency — while
//!    polling everything else, with only the primary source modeled as a
//!    path) and not flagged.
//!
//! 2. **Node with zero declared paths**: the generalized rule does NOT
//!    apply — not-yet-migrated contracts fall under the
//!    `explicit-trigger`/migration lint family (Info severity), not a
//!    Warning here. The original shipped heuristic is kept for these
//!    nodes: ≥2 sibling subs tagged `state: true` (an established "this
//!    node polls its inputs" pattern) AND exactly 1 unaccounted sub →
//!    warn. This is the high-precision pre-44.2 rule that caught both
//!    real Autoware smells (`external_cmd_converter/heartbeat`,
//!    `lane_departure_checker/predicted_trajectory` — note the latter is
//!    a zero-path node, which is why this branch must survive the gate)
//!    with zero noise on the same corpus.

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::{EffectiveTrigger, Manifest};
use std::collections::HashSet;

pub struct StateConsistencyRule;

impl ValidationRule for StateConsistencyRule {
    fn id(&self) -> &str {
        "state-consistency"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        for (node_name, node) in &manifest.nodes {
            if node.subscribers.is_empty() {
                continue;
            }

            // Endpoint names referenced as the effective input trigger of
            // any of this node's paths (explicit trigger.input OR legacy
            // input: derivation — both count via effective_trigger()).
            let path_inputs: HashSet<String> = node
                .paths
                .values()
                .filter_map(|path| match path.effective_trigger() {
                    EffectiveTrigger::Input(eps) => Some(eps),
                    _ => None,
                })
                .flatten()
                .collect();

            let unaccounted: Vec<&str> = node
                .subscribers
                .iter()
                .filter(|(ep_name, props)| {
                    !props.state.unwrap_or(false) && !path_inputs.contains(ep_name.as_str())
                })
                .map(|(ep_name, _)| ep_name.as_str())
                .collect();

            // Both branches only flag a single lone stray sub — 2+
            // unaccounted subs looks like a deliberate hybrid design
            // (multiple independent causal inputs), not a one-off missed
            // tag. See module docs.
            let [ep_name] = unaccounted.as_slice() else {
                continue;
            };

            if node.paths.is_empty() {
                // Branch 2: unmigrated node (no paths declared) — keep the
                // original narrow heuristic: only flag when ≥2 siblings
                // are already state-tagged (established polling pattern).
                let state_count = node
                    .subscribers
                    .values()
                    .filter(|props| props.state.unwrap_or(false))
                    .count();
                if state_count < 2 {
                    continue;
                }
                ctx.warning(
                    self.id(),
                    &format!("nodes.{node_name}.sub.{ep_name}"),
                    format!(
                        "node '{node_name}' has {state_count} sibling subscriber(s) tagged \
                         'state: true', but '{ep_name}' is neither tagged 'state: true' nor \
                         referenced as the effective input trigger of any of this node's \
                         'paths:' entries — likely a missed 'state:' tag (see the \
                         simple_planning_simulator control_cmd bug in Phase 42's study for \
                         the motivating pattern)."
                    ),
                );
            } else {
                // Branch 1: node has opted into path modeling — a lone
                // unexplained sub warns regardless of how many siblings
                // are state-tagged.
                ctx.warning(
                    self.id(),
                    &format!("nodes.{node_name}.sub.{ep_name}"),
                    format!(
                        "node '{node_name}' declares 'paths:' but its subscriber '{ep_name}' \
                         is neither tagged 'state: true' nor referenced as the effective \
                         input trigger of any of those paths — likely a missed 'state:' tag \
                         or a missing causal path (see the simple_planning_simulator \
                         control_cmd bug in Phase 42's study for the motivating pattern)."
                    ),
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CheckContext;
    use ros_launch_manifest_types::parse::parse_manifest_str;

    fn run(yaml: &str) -> Vec<crate::Diagnostic> {
        let manifest = parse_manifest_str(yaml).unwrap();
        let graph = DataflowGraph::build(&manifest);
        let mut ctx = CheckContext::new();
        StateConsistencyRule.check(&manifest, &graph, &mut ctx);
        ctx.diagnostics
    }

    /// Branch 2 positive: the exact motivating pattern — a zero-path node
    /// with 2+ sibling subs tagged `state: true` and one behaviorally
    /// identical sub left untagged. Should warn on the untagged sub.
    #[test]
    fn untagged_sibling_of_state_subs_warns() {
        let yaml = r#"
version: 1
nodes:
  simple_planning_simulator:
    sub:
      gear_cmd:
        state: true
      turn_indicators_cmd:
        state: true
      control_cmd: {}
"#;
        let warnings: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Warning)
            .collect();
        assert_eq!(warnings.len(), 1, "got: {warnings:?}");
        assert!(warnings[0].path.ends_with("control_cmd"));
        assert!(warnings[0].message.contains("simple_planning_simulator"));
    }

    /// Branch 2 negative: zero-path node with fewer than 2 state-tagged
    /// siblings — heuristic doesn't fire.
    #[test]
    fn single_state_sub_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a:
        state: true
      b: {}
"#;
        assert!(run(yaml).is_empty());
    }

    /// Noise gate (44.2 review): a zero-path node whose subs are simply
    /// unannotated (no state tags at all) is migration territory — must
    /// NOT warn; the `explicit-trigger` lint covers this family at Info.
    #[test]
    fn zero_path_node_without_state_siblings_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a: {}
"#;
        assert!(run(yaml).is_empty());
    }

    /// Branch 1 negative: the untagged sub is accounted for via a node
    /// path's explicit `trigger: { input: [...] }`.
    #[test]
    fn untagged_sub_referenced_by_explicit_trigger_input_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a:
        state: true
      b:
        state: true
      c: {}
    pub:
      out: {}
    paths:
      main:
        trigger: { input: [c] }
        output: [out]
"#;
        assert!(run(yaml).is_empty());
    }

    /// Branch 1 negative: the untagged sub is accounted for via the legacy
    /// `input:` list (no explicit trigger — effective_trigger() still
    /// derives Input).
    #[test]
    fn untagged_sub_referenced_by_legacy_input_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a:
        state: true
      b:
        state: true
      c: {}
    pub:
      out: {}
    paths:
      main:
        input: [c]
        output: [out]
"#;
        assert!(run(yaml).is_empty());
    }

    /// Branch 1 positive: a path-declaring node with exactly one
    /// unaccounted sub warns even with ZERO state-tagged siblings — the
    /// generalization over the old heuristic (which required ≥2). The
    /// real-world shape: external_cmd_converter (paths: main, state subs,
    /// lone unaccounted 'heartbeat').
    #[test]
    fn path_node_with_lone_unaccounted_sub_warns_without_state_siblings() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      cause: {}
      heartbeat: {}
    pub:
      out: {}
    paths:
      main:
        trigger: { input: [cause] }
        output: [out]
"#;
        let warnings = run(yaml);
        assert_eq!(warnings.len(), 1, "got: {warnings:?}");
        assert!(warnings[0].path.ends_with("heartbeat"));
        assert!(warnings[0].message.contains("declares 'paths:'"));
    }

    /// Hybrid-design escape, path branch: 2+ unaccounted subs on a node
    /// that declares paths — the vehicle_cmd_gate shape (event-driven for
    /// several independent control sources, only the primary modeled as a
    /// path). Must not warn.
    #[test]
    fn path_node_with_multiple_unaccounted_subs_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  gate:
    sub:
      auto_cmd: {}
      external_cmd: {}
      emergency_cmd: {}
      operation_mode:
        state: true
    pub:
      out: {}
    paths:
      command:
        trigger: { input: [auto_cmd] }
        output: [out]
"#;
        // external_cmd + emergency_cmd unaccounted (2) → hybrid, no warn.
        assert!(run(yaml).is_empty());
    }

    /// Hybrid-design escape, zero-path branch: 2+ unaccounted subs — no
    /// warn (unchanged from the original heuristic).
    #[test]
    fn zero_path_node_with_multiple_unaccounted_subs_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a:
        state: true
      b:
        state: true
      c: {}
      d: {}
"#;
        assert!(run(yaml).is_empty());
    }

    /// Negative case: all subs consistently tagged `state: true` — nothing
    /// to flag.
    #[test]
    fn all_subs_tagged_state_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a:
        state: true
      b:
        state: true
      c:
        state: true
"#;
        assert!(run(yaml).is_empty());
    }
}
