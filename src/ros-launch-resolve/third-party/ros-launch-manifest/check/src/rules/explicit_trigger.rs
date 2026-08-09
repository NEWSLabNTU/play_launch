//! Rule: nudge every path toward an explicit `trigger:` declaration
//! (Vocabulary v2, Phase 44.1 §5).
//!
//! Fires (info severity — authoring hygiene, not a defect) whenever a
//! path has no explicit `trigger:` field, regardless of whether the
//! legacy `input:` list derives `EffectiveTrigger::Input` or the path is
//! fully `Unclassified`. Migration aid: existing contracts stay valid
//! (info, not warning/error) while nudging authors toward the explicit
//! four-way taxonomy.

use super::ValidationRule;
use crate::{CheckContext, Severity, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;

pub struct ExplicitTriggerRule;

impl ValidationRule for ExplicitTriggerRule {
    fn id(&self) -> &str {
        "explicit-trigger"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        for (node_name, node) in &manifest.nodes {
            for (path_name, path) in &node.paths {
                if path.trigger.is_none() {
                    ctx.emit(
                        self.id(),
                        Severity::Info,
                        &format!("nodes.{node_name}.paths.{path_name}"),
                        format!(
                            "node '{node_name}' path '{path_name}' has no explicit 'trigger:' \
                             — add one of timer/input/once/spontaneous (Vocabulary v2)"
                        ),
                    );
                }
            }
        }
        // Scope-level (top-of-manifest) paths carry the same PathDecl shape.
        for (path_name, path) in &manifest.paths {
            if path.trigger.is_none() {
                ctx.emit(
                    self.id(),
                    Severity::Info,
                    &format!("paths.{path_name}"),
                    format!(
                        "scope path '{path_name}' has no explicit 'trigger:' — add one of \
                         timer/input/once/spontaneous (Vocabulary v2)"
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
        ExplicitTriggerRule.check(&manifest, &graph, &mut ctx);
        ctx.diagnostics
    }

    #[test]
    fn path_without_trigger_infos() {
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
        let diags = run(yaml);
        assert_eq!(diags.len(), 1, "got: {diags:?}");
        assert_eq!(diags[0].severity, Severity::Info);
        assert!(diags[0].path.ends_with("main"));
    }

    #[test]
    fn path_with_explicit_trigger_does_not_fire() {
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
        output: [out]
"#;
        assert!(run(yaml).is_empty());
    }

    #[test]
    fn unclassified_path_also_infos() {
        let yaml = r#"
version: 1
nodes:
  n:
    pub:
      out: {}
    paths:
      main:
        output: [out]
"#;
        let diags = run(yaml);
        assert_eq!(diags.len(), 1);
        assert_eq!(diags[0].severity, Severity::Info);
    }

    #[test]
    fn scope_path_without_trigger_infos() {
        let yaml = r#"
version: 1
paths:
  e2e:
    input: [a]
    output: [b]
"#;
        let diags = run(yaml);
        assert_eq!(diags.len(), 1);
        assert!(diags[0].path.starts_with("paths."));
    }
}
