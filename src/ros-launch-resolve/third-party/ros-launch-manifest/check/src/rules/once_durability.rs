//! Rule: a `once`-triggered path's output topic should be
//! `durability: transient_local` (Vocabulary v2, Phase 44.1 §1/§5).
//!
//! `once` marks a startup-latch publish (e.g. a map load). Late-joining
//! subscribers only ever see it if the topic is transient_local; a
//! volatile `once` publish is very likely a QoS authoring bug (the
//! message is gone forever the instant it's sent).

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::{EffectiveTrigger, Manifest, QosDecl};

pub struct OnceDurabilityRule;

impl ValidationRule for OnceDurabilityRule {
    fn id(&self) -> &str {
        "once-durability"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        for (node_name, node) in &manifest.nodes {
            for (path_name, path) in &node.paths {
                if path.effective_trigger() != EffectiveTrigger::Once {
                    continue;
                }
                for ep_name in &path.output {
                    let Some(ep_props) = node.publishers.get(ep_name) else {
                        continue;
                    };
                    let Some((topic_name, topic)) =
                        super::endpoint_topic::topic_for_pub(manifest, node_name, ep_name)
                    else {
                        continue;
                    };
                    let effective = QosDecl::effective(topic.qos.as_ref(), ep_props.qos.as_ref());
                    let durability = effective.durability.as_deref();
                    if durability != Some("transient_local") {
                        ctx.warning(
                            self.id(),
                            &format!("nodes.{node_name}.paths.{path_name}"),
                            format!(
                                "node '{node_name}' path '{path_name}' is 'once'-triggered and \
                                 publishes '{ep_name}' on topic '{topic_name}', but its \
                                 effective QoS durability is {} (not 'transient_local') — late \
                                 joiners will never see this message",
                                durability
                                    .map(|d| format!("'{d}'"))
                                    .unwrap_or_else(|| "unset".to_string())
                            ),
                        );
                    }
                }
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
        OnceDurabilityRule.check(&manifest, &graph, &mut ctx);
        ctx.diagnostics
    }

    #[test]
    fn once_output_without_transient_local_warns() {
        let yaml = r#"
version: 1
nodes:
  map_loader:
    pub:
      map: {}
    paths:
      publish_map:
        trigger: once
        output: [map]
topics:
  map:
    type: nav_msgs/msg/OccupancyGrid
    pub: [map_loader/map]
"#;
        let warnings = run(yaml);
        assert_eq!(warnings.len(), 1, "got: {warnings:?}");
        assert!(warnings[0].message.contains("transient_local"));
    }

    #[test]
    fn once_output_with_topic_level_transient_local_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  map_loader:
    pub:
      map: {}
    paths:
      publish_map:
        trigger: once
        output: [map]
topics:
  map:
    type: nav_msgs/msg/OccupancyGrid
    pub: [map_loader/map]
    qos:
      durability: transient_local
"#;
        assert!(run(yaml).is_empty());
    }

    #[test]
    fn once_output_with_endpoint_level_transient_local_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  map_loader:
    pub:
      map:
        qos:
          durability: transient_local
    paths:
      publish_map:
        trigger: once
        output: [map]
topics:
  map:
    type: nav_msgs/msg/OccupancyGrid
    pub: [map_loader/map]
    qos:
      durability: volatile
"#;
        assert!(run(yaml).is_empty());
    }

    #[test]
    fn non_once_path_does_not_fire() {
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
topics:
  out:
    type: std_msgs/msg/String
    pub: [n/out]
"#;
        assert!(run(yaml).is_empty());
    }
}
