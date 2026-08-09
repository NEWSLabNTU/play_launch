//! Rule: stale legacy `input:` list alongside a non-Input explicit
//! trigger (Vocabulary v2, Phase 44.1 §1, W1 carry-forward).
//!
//! W1 deliberately left this unchecked at parse time (see
//! `p44-w1-report.md` deviation 4): a path may declare
//! `trigger: { timer: {...} }` / `once` / `spontaneous` while *also*
//! carrying a non-empty legacy `input:` list left over from before the
//! author added the explicit trigger. Since a non-`Input` trigger never
//! consumes the legacy `input:` list ([`PathDecl::effective_trigger`]
//! ignores it once an explicit non-Input trigger is present), the
//! leftover list is dead/misleading data — warn "stale input list".
//!
//! The spec's second `inherited-rate` clause (`rate_hz` authored on an
//! input-triggered path) is **not implemented**: per W1's report, no
//! path-level `rate_hz` field exists in the schema (`Trigger::Timer` is
//! the only place `rate_hz` lives), so there is nothing to author "on
//! the path" that this rule could detect directly. The brief's suggested
//! proxy — "a node's ONLY paths are input-triggered yet a topic it
//! solely publishes declares rate_hz" — was judged too fuzzy to ship
//! confidently: `rate_hz` on a topic is a legitimate declared *channel*
//! rate independent of which node happens to be its sole publisher (e.g.
//! a `once`-triggered latched topic can validly declare an expected
//! consumer-side rate; a topic rate can be a measured/target fact
//! unrelated to "the author copy-pasted an inherited number"). Shipping
//! it would risk exactly the false-positive pattern (W2, "43/78 pairs")
//! the vocabulary v2 spec's own evidence base was designed to eliminate.
//! Left for a future pass if real-world signal justifies it.

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::{EffectiveTrigger, Manifest};

pub struct InheritedRateRule;

impl ValidationRule for InheritedRateRule {
    fn id(&self) -> &str {
        "inherited-rate"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        for (node_name, node) in &manifest.nodes {
            for (path_name, path) in &node.paths {
                check_path(self.id(), node_name, path_name, path, ctx);
            }
        }
        for (path_name, path) in &manifest.paths {
            check_path(self.id(), "<scope>", path_name, path, ctx);
        }
    }
}

fn check_path(
    rule_id: &str,
    node_name: &str,
    path_name: &str,
    path: &ros_launch_manifest_types::PathDecl,
    ctx: &mut CheckContext,
) {
    if path.input.is_empty() {
        return;
    }
    let is_non_input_explicit_trigger = matches!(
        path.effective_trigger(),
        EffectiveTrigger::Timer { .. } | EffectiveTrigger::Once | EffectiveTrigger::Spontaneous
    ) && path.trigger.is_some();
    if !is_non_input_explicit_trigger {
        return;
    }
    let loc = if node_name == "<scope>" {
        format!("paths.{path_name}")
    } else {
        format!("nodes.{node_name}.paths.{path_name}")
    };
    ctx.warning(
        rule_id,
        &loc,
        format!(
            "path '{path_name}' declares a non-'input' trigger but still carries a non-empty \
             legacy 'input:' list ({:?}) — stale input list, remove it (the explicit trigger \
             wins and the legacy list is no longer consumed)",
            path.input
        ),
    );
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
        InheritedRateRule.check(&manifest, &graph, &mut ctx);
        ctx.diagnostics
    }

    #[test]
    fn timer_trigger_with_stale_input_warns() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a: {}
    pub:
      out: {}
    paths:
      main:
        trigger: { timer: { rate_hz: 10 } }
        input: [a]
        output: [out]
"#;
        let warnings = run(yaml);
        assert_eq!(warnings.len(), 1, "got: {warnings:?}");
        assert!(warnings[0].message.contains("stale input list"));
    }

    #[test]
    fn once_trigger_with_stale_input_warns() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a: {}
    pub:
      out: {}
    paths:
      main:
        trigger: once
        input: [a]
        output: [out]
"#;
        assert_eq!(run(yaml).len(), 1);
    }

    #[test]
    fn spontaneous_trigger_with_stale_input_warns() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a: {}
    pub:
      out: {}
    paths:
      main:
        trigger: spontaneous
        input: [a]
        output: [out]
"#;
        assert_eq!(run(yaml).len(), 1);
    }

    #[test]
    fn input_trigger_with_matching_legacy_input_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a: {}
    pub:
      out: {}
    paths:
      main:
        trigger: { input: [a] }
        input: [a]
        output: [out]
"#;
        assert!(run(yaml).is_empty());
    }

    #[test]
    fn timer_trigger_without_legacy_input_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  n:
    pub:
      out: {}
    paths:
      main:
        trigger: { timer: { rate_hz: 10 } }
        output: [out]
"#;
        assert!(run(yaml).is_empty());
    }

    #[test]
    fn no_explicit_trigger_legacy_input_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      a: {}
    pub:
      out: {}
    paths:
      main:
        input: [a]
        output: [out]
"#;
        assert!(run(yaml).is_empty());
    }
}
