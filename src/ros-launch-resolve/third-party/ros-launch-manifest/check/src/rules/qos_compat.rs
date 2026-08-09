//! Rule: QoS field value validity (topic-level and per-endpoint).
//!
//! Validates that declared QoS field values come from the allowed set
//! (e.g. `reliability` ∈ {reliable, best_effort}). Applies to both
//! topic-level `qos:` blocks and per-endpoint overrides on publishers
//! and subscribers. Pub/sub compatibility is handled by the separate
//! `qos-match` rule.

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::{Manifest, QosDecl};
use std::collections::BTreeMap;

pub struct QosCompatRule;

impl ValidationRule for QosCompatRule {
    fn id(&self) -> &str {
        "qos-compat"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        // Topic-level QoS blocks.
        for (topic_name, topic) in &manifest.topics {
            if let Some(qos) = &topic.qos {
                check_qos_values(self.id(), &format!("topics.{topic_name}.qos"), qos, ctx);
            }
        }

        // Per-endpoint QoS overrides on every node's pub/sub/cli endpoints.
        // `EndpointProps` applies to every endpoint kind that carries it,
        // and `qos:` on a `cli` is meaningful for service-client QoS profiles.
        for (node_name, node) in &manifest.nodes {
            check_node_endpoints(self.id(), node_name, "pub", &node.publishers, ctx);
            check_node_endpoints(self.id(), node_name, "sub", &node.subscribers, ctx);
            check_node_endpoints(self.id(), node_name, "cli", &node.cli, ctx);
        }
    }
}

fn check_node_endpoints(
    rule_id: &str,
    node_name: &str,
    kind: &str,
    eps: &BTreeMap<String, ros_launch_manifest_types::EndpointProps>,
    ctx: &mut CheckContext,
) {
    for (ep_name, props) in eps {
        if let Some(qos) = &props.qos {
            let path = format!("nodes.{node_name}.{kind}.{ep_name}.qos");
            check_qos_values(rule_id, &path, qos, ctx);
        }
    }
}

fn check_qos_values(rule_id: &str, path: &str, qos: &QosDecl, ctx: &mut CheckContext) {
    if let Some(rel) = &qos.reliability
        && rel != "reliable"
        && rel != "best_effort"
    {
        ctx.error(
            rule_id,
            &format!("{path}.reliability"),
            format!("invalid reliability value '{rel}', expected 'reliable' or 'best_effort'"),
        );
    }
    if let Some(dur) = &qos.durability
        && dur != "volatile"
        && dur != "transient_local"
    {
        ctx.error(
            rule_id,
            &format!("{path}.durability"),
            format!("invalid durability value '{dur}', expected 'volatile' or 'transient_local'"),
        );
    }
    if let Some(history) = &qos.history
        && history != "keep_last"
        && history != "keep_all"
    {
        ctx.error(
            rule_id,
            &format!("{path}.history"),
            format!("invalid history value '{history}', expected 'keep_last' or 'keep_all'"),
        );
    }
    if let Some(liveliness) = &qos.liveliness
        && liveliness != "automatic"
        && liveliness != "manual_by_topic"
    {
        ctx.error(
            rule_id,
            &format!("{path}.liveliness"),
            format!(
                "invalid liveliness value '{liveliness}', expected 'automatic' or 'manual_by_topic'"
            ),
        );
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
        QosCompatRule.check(&manifest, &graph, &mut ctx);
        ctx.diagnostics
    }

    #[test]
    fn topic_invalid_reliability_errors() {
        let yaml = r#"
version: 1
topics:
  /foo:
    type: std_msgs/msg/String
    qos:
      reliability: maybe
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert_eq!(errs.len(), 1);
        assert!(errs[0].message.contains("maybe"));
        assert!(errs[0].path.ends_with("reliability"));
    }

    #[test]
    fn endpoint_invalid_reliability_errors() {
        let yaml = r#"
version: 1
nodes:
  pub_node:
    pub:
      data:
        qos: { reliability: kinda_reliable }
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert_eq!(errs.len(), 1, "got: {errs:?}");
        assert!(errs[0].path.contains("pub_node.pub.data.qos.reliability"));
        assert!(errs[0].message.contains("kinda_reliable"));
    }

    #[test]
    fn endpoint_invalid_durability_errors() {
        let yaml = r#"
version: 1
nodes:
  sub_node:
    sub:
      data:
        qos: { durability: maybe }
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert_eq!(errs.len(), 1);
        assert!(errs[0].path.contains("sub_node.sub.data.qos.durability"));
    }

    #[test]
    fn invalid_history_errors() {
        let yaml = r#"
version: 1
topics:
  /foo:
    type: std_msgs/msg/String
    qos: { history: keep_some }
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert_eq!(errs.len(), 1);
        assert!(errs[0].message.contains("keep_some"));
    }

    #[test]
    fn invalid_liveliness_errors() {
        let yaml = r#"
version: 1
topics:
  /foo:
    type: std_msgs/msg/String
    qos: { liveliness: best_effort }
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert_eq!(errs.len(), 1);
        assert!(errs[0].path.ends_with("liveliness"));
    }

    #[test]
    fn valid_topic_and_endpoint_qos_no_errors() {
        let yaml = r#"
version: 1
nodes:
  pub_node:
    pub:
      data:
        qos: { reliability: reliable, durability: transient_local }
  sub_node:
    sub:
      data:
        qos: { reliability: best_effort }
topics:
  /channel:
    type: std_msgs/msg/String
    pub: [pub_node/data]
    sub: [sub_node/data]
    qos:
      reliability: reliable
      durability: volatile
      history: keep_last
      liveliness: automatic
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert!(errs.is_empty(), "expected no errors, got: {errs:?}");
    }
}
