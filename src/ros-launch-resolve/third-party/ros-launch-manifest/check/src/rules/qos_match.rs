//! Rule: QoS profile structural validity and pub/sub compatibility.
//!
//! Two checks:
//!
//! 1. **Structural invariants on declared QoS blocks** — combinations
//!    that are nonsensical or misconfigured (e.g. `keep_last` with
//!    `depth: 0`, `best_effort` + `transient_local` durability that
//!    breaks late-joining).
//!
//! 2. **Pub/sub compatibility (offered ≥ requested)** — for every
//!    publisher × subscriber pair on a topic, compute effective QoS
//!    (topic default overlaid with per-endpoint override) and apply
//!    the DDS compatibility matrix on `reliability` and `durability`.
//!    A field is checked only when both sides specify it directly or
//!    via inheritance — unspecified fields are skipped (no implicit
//!    ROS 2 default is assumed because real deployments use multiple
//!    profiles with different defaults).
//!
//! Cross-scope: this rule operates per-manifest. The `manifest_loader`
//! in the executing crate runs an equivalent check after cross-scope
//! merge, where publishers and subscribers from different scopes are
//! reconciled. Per-manifest checking catches the common case where
//! both endpoints live in the same scope.

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::{EndpointProps, Manifest, NodeDecl, QosDecl, TopicDecl};
use std::collections::BTreeMap;

pub struct QosMatchRule;

impl ValidationRule for QosMatchRule {
    fn id(&self) -> &str {
        "qos-match"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        // Per-topic structural checks on the declared topic-level QoS
        // and on any per-endpoint QoS override.
        for (topic_name, topic) in &manifest.topics {
            if let Some(qos) = &topic.qos {
                check_qos_profile(
                    self.id(),
                    topic_name,
                    &format!("topics.{topic_name}.qos"),
                    qos,
                    ctx,
                );
            }
            check_endpoint_qos_profiles(self.id(), topic_name, topic, &manifest.nodes, ctx);

            // Pub/sub compatibility check on merged endpoints within this manifest.
            check_pub_sub_compat(self.id(), topic_name, topic, &manifest.nodes, ctx);
        }
    }
}

/// Structural validation of a single QoS profile (topic-level or
/// endpoint-level): depth/history/durability sanity.
fn check_qos_profile(
    rule_id: &str,
    topic_name: &str,
    path: &str,
    qos: &QosDecl,
    ctx: &mut CheckContext,
) {
    if let Some(depth) = qos.depth
        && depth == 0
    {
        ctx.error(
            rule_id,
            path,
            format!(
                "topic '{topic_name}' qos.depth is 0; depth must be >= 1 \
                 to enable any message buffering"
            ),
        );
    }

    if let (Some(history), Some(_depth)) = (qos.history.as_deref(), qos.depth)
        && history == "keep_all"
    {
        ctx.warning(
            rule_id,
            path,
            format!(
                "topic '{topic_name}' qos uses history='keep_all' with depth set; \
                 depth is ignored when history is keep_all"
            ),
        );
    }

    if let (Some(rel), Some(dur)) = (qos.reliability.as_deref(), qos.durability.as_deref())
        && rel == "best_effort"
        && dur == "transient_local"
    {
        ctx.warning(
            rule_id,
            path,
            format!(
                "topic '{topic_name}' qos combines best_effort + transient_local; \
                 late-joining subscribers will not receive historical data over best_effort"
            ),
        );
    }
}

/// Run structural checks on every per-endpoint QoS override declared
/// for publishers and subscribers of this topic.
fn check_endpoint_qos_profiles(
    rule_id: &str,
    topic_name: &str,
    topic: &TopicDecl,
    nodes: &BTreeMap<String, NodeDecl>,
    ctx: &mut CheckContext,
) {
    for ep_ref in topic.publishers.iter().chain(topic.subscribers.iter()) {
        let Some((node_name, ep_name)) = split_endpoint_ref(ep_ref) else {
            continue;
        };
        let Some(node) = nodes.get(node_name) else {
            continue;
        };
        let kind_and_props = node
            .publishers
            .get(ep_name)
            .map(|p| ("pub", p))
            .or_else(|| node.subscribers.get(ep_name).map(|p| ("sub", p)));
        let Some((kind, props)) = kind_and_props else {
            continue;
        };
        let Some(ep_qos) = &props.qos else {
            continue;
        };
        let path = format!("nodes.{node_name}.{kind}.{ep_name}.qos");
        check_qos_profile(rule_id, topic_name, &path, ep_qos, ctx);
    }
}

/// Pub/sub compatibility check on every (pub, sub) pair of the topic.
fn check_pub_sub_compat(
    rule_id: &str,
    topic_name: &str,
    topic: &TopicDecl,
    nodes: &BTreeMap<String, NodeDecl>,
    ctx: &mut CheckContext,
) {
    let topic_qos = topic.qos.as_ref();
    let resolved_pubs: Vec<(String, String, QosDecl)> = topic
        .publishers
        .iter()
        .filter_map(|ep_ref| resolve_endpoint(ep_ref, nodes, |n| &n.publishers, topic_qos))
        .collect();
    let resolved_subs: Vec<(String, String, QosDecl)> = topic
        .subscribers
        .iter()
        .filter_map(|ep_ref| resolve_endpoint(ep_ref, nodes, |n| &n.subscribers, topic_qos))
        .collect();

    for (pub_node, pub_ep, pub_qos) in &resolved_pubs {
        for (sub_node, sub_ep, sub_qos) in &resolved_subs {
            check_compat_pair(
                rule_id, topic_name, pub_node, pub_ep, pub_qos, sub_node, sub_ep, sub_qos, ctx,
            );
        }
    }
}

/// Resolve an endpoint ref like `node/ep` to (node_name, ep_name,
/// effective_qos) by looking up the node, fetching the endpoint's QoS
/// override (if any), and overlaying it on the topic-level QoS.
fn resolve_endpoint<F>(
    ep_ref: &str,
    nodes: &BTreeMap<String, NodeDecl>,
    select: F,
    topic_qos: Option<&QosDecl>,
) -> Option<(String, String, QosDecl)>
where
    F: Fn(&NodeDecl) -> &BTreeMap<String, EndpointProps>,
{
    let (node_name, ep_name) = split_endpoint_ref(ep_ref)?;
    let node = nodes.get(node_name)?;
    let props = select(node).get(ep_name)?;
    let effective = QosDecl::effective(topic_qos, props.qos.as_ref());
    Some((node_name.to_string(), ep_name.to_string(), effective))
}

fn split_endpoint_ref(ep_ref: &str) -> Option<(&str, &str)> {
    let pos = ep_ref.rfind('/')?;
    let node = &ep_ref[..pos];
    let ep = &ep_ref[pos + 1..];
    if node.is_empty() || ep.is_empty() {
        return None;
    }
    Some((node, ep))
}

/// Apply the DDS compatibility matrix to a single (pub, sub) pair.
/// Emits one error per incompatible field.
#[allow(clippy::too_many_arguments)]
fn check_compat_pair(
    rule_id: &str,
    topic_name: &str,
    pub_node: &str,
    pub_ep: &str,
    pub_qos: &QosDecl,
    sub_node: &str,
    sub_ep: &str,
    sub_qos: &QosDecl,
    ctx: &mut CheckContext,
) {
    let path = format!("topics.{topic_name}");

    if let (Some(pub_rel), Some(sub_rel)) = (
        pub_qos.reliability.as_deref(),
        sub_qos.reliability.as_deref(),
    ) && !reliability_compatible(pub_rel, sub_rel)
    {
        ctx.error(
            rule_id,
            &path,
            format!(
                "incompatible QoS on topic '{topic_name}' field 'reliability': pub \
                 '{pub_node}/{pub_ep}' offers '{pub_rel}', sub '{sub_node}/{sub_ep}' \
                 requests '{sub_rel}'"
            ),
        );
    }

    if let (Some(pub_dur), Some(sub_dur)) =
        (pub_qos.durability.as_deref(), sub_qos.durability.as_deref())
        && !durability_compatible(pub_dur, sub_dur)
    {
        ctx.error(
            rule_id,
            &path,
            format!(
                "incompatible QoS on topic '{topic_name}' field 'durability': pub \
                 '{pub_node}/{pub_ep}' offers '{pub_dur}', sub '{sub_node}/{sub_ep}' \
                 requests '{sub_dur}'"
            ),
        );
    }
}

/// DDS reliability compatibility: offered must be at least as strong as
/// requested. `reliable` ≥ `best_effort`. Unknown values are accepted —
/// `qos-compat` reports invalid value tokens separately.
fn reliability_compatible(pub_v: &str, sub_v: &str) -> bool {
    match (pub_v, sub_v) {
        ("reliable", "reliable") => true,
        ("reliable", "best_effort") => true,
        ("best_effort", "reliable") => false,
        ("best_effort", "best_effort") => true,
        _ => true,
    }
}

/// DDS durability compatibility: offered must be at least as strong as
/// requested. `transient_local` ≥ `volatile`. Unknown values pass.
fn durability_compatible(pub_v: &str, sub_v: &str) -> bool {
    match (pub_v, sub_v) {
        ("transient_local", "transient_local") => true,
        ("transient_local", "volatile") => true,
        ("volatile", "transient_local") => false,
        ("volatile", "volatile") => true,
        _ => true,
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
        QosMatchRule.check(&manifest, &graph, &mut ctx);
        ctx.diagnostics
    }

    #[test]
    fn reliability_mismatch_best_effort_pub_reliable_sub() {
        let yaml = r#"
version: 1
nodes:
  lidar:
    pub: [pointcloud]
  perception:
    sub:
      pointcloud:
        qos: { reliability: reliable }
topics:
  /sensor/pointcloud:
    type: sensor_msgs/msg/PointCloud2
    pub: [lidar/pointcloud]
    sub: [perception/pointcloud]
    qos: { reliability: best_effort }
"#;
        let diags = run(yaml);
        let errs: Vec<_> = diags
            .iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert!(
            errs.iter().any(|d| d.message.contains("reliability")
                && d.message.contains("best_effort")
                && d.message.contains("reliable")),
            "expected reliability mismatch error, got: {errs:?}"
        );
    }

    #[test]
    fn reliability_compat_reliable_pub_best_effort_sub() {
        let yaml = r#"
version: 1
nodes:
  lidar:
    pub:
      pointcloud:
        qos: { reliability: reliable }
  viz:
    sub:
      pointcloud:
        qos: { reliability: best_effort }
topics:
  /sensor/pointcloud:
    type: sensor_msgs/msg/PointCloud2
    pub: [lidar/pointcloud]
    sub: [viz/pointcloud]
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert!(errs.is_empty(), "expected no errors, got: {errs:?}");
    }

    #[test]
    fn durability_mismatch_volatile_pub_transient_local_sub() {
        let yaml = r#"
version: 1
nodes:
  pub_node:
    pub: [data]
  sub_node:
    sub:
      data:
        qos: { durability: transient_local }
topics:
  /map:
    type: std_msgs/msg/String
    pub: [pub_node/data]
    sub: [sub_node/data]
    qos: { durability: volatile, reliability: reliable }
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert!(
            errs.iter().any(|d| d.message.contains("durability")
                && d.message.contains("volatile")
                && d.message.contains("transient_local")),
            "expected durability mismatch error, got: {errs:?}"
        );
    }

    #[test]
    fn endpoint_inherits_topic_qos_no_error() {
        let yaml = r#"
version: 1
nodes:
  lidar:
    pub: [pointcloud]
  recorder:
    sub:
      pointcloud: {}
topics:
  /sensor/pointcloud:
    type: sensor_msgs/msg/PointCloud2
    pub: [lidar/pointcloud]
    sub: [recorder/pointcloud]
    qos: { reliability: best_effort }
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert!(errs.is_empty(), "expected no errors, got: {errs:?}");
    }

    #[test]
    fn endpoint_override_takes_priority_over_topic_default() {
        // Topic says reliable; sub overrides to best_effort. Pub inherits
        // reliable. reliable pub × best_effort sub → compatible.
        let yaml = r#"
version: 1
nodes:
  lidar:
    pub: [pointcloud]
  viz:
    sub:
      pointcloud:
        qos: { reliability: best_effort }
topics:
  /sensor/pointcloud:
    type: sensor_msgs/msg/PointCloud2
    pub: [lidar/pointcloud]
    sub: [viz/pointcloud]
    qos: { reliability: reliable }
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert!(errs.is_empty(), "expected no errors, got: {errs:?}");
    }

    #[test]
    fn unspecified_field_is_skipped() {
        // Topic qos sets only reliability=reliable. Sub overrides
        // durability=transient_local but pub durability is unspecified
        // → durability check is skipped (no error).
        let yaml = r#"
version: 1
nodes:
  pub_node:
    pub: [data]
  sub_node:
    sub:
      data:
        qos: { durability: transient_local }
topics:
  /channel:
    type: std_msgs/msg/String
    pub: [pub_node/data]
    sub: [sub_node/data]
    qos: { reliability: reliable }
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert!(errs.is_empty(), "expected no errors, got: {errs:?}");
    }

    #[test]
    fn structural_check_depth_zero_on_endpoint() {
        let yaml = r#"
version: 1
nodes:
  pub_node:
    pub:
      data:
        qos: { depth: 0 }
topics:
  /channel:
    type: std_msgs/msg/String
    pub: [pub_node/data]
"#;
        let errs: Vec<_> = run(yaml)
            .into_iter()
            .filter(|d| d.severity == crate::Severity::Error)
            .collect();
        assert!(
            errs.iter().any(|d| d.message.contains("depth")),
            "expected depth=0 error on endpoint qos, got: {errs:?}"
        );
    }
}
