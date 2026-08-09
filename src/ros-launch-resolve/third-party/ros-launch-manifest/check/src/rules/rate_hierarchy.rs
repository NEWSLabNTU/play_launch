//! Rule: rate hierarchy — pub.min_rate_hz >= topic.rate_hz >= max(sub.min_rate_hz).

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;

pub struct RateHierarchyRule;

impl ValidationRule for RateHierarchyRule {
    fn id(&self) -> &str {
        "rate-hierarchy"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        for (topic_name, topic) in &manifest.topics {
            let topic_rate = match topic.rate_hz {
                Some(r) => r,
                None => continue,
            };

            // Check publisher min_rate_hz >= topic rate_hz
            for pub_ref in &topic.publishers {
                if let Some(rate) = resolve_pub_min_rate(pub_ref, manifest)
                    && rate < topic_rate
                {
                    ctx.error(
                            self.id(),
                            &format!("topics.{topic_name}"),
                            format!(
                                "publisher '{pub_ref}' min_rate_hz ({rate}) < topic rate_hz ({topic_rate})"
                            ),
                        );
                }
            }

            // Check topic rate_hz >= max(sub.min_rate_hz)
            let mut max_sub_rate: Option<(f64, String)> = None;
            for sub_ref in &topic.subscribers {
                if let Some(rate) = resolve_sub_min_rate(sub_ref, manifest)
                    && max_sub_rate.as_ref().is_none_or(|(r, _)| rate > *r)
                {
                    max_sub_rate = Some((rate, sub_ref.clone()));
                }
            }
            if let Some((sub_rate, sub_ref)) = max_sub_rate
                && topic_rate < sub_rate
            {
                ctx.error(
                        self.id(),
                        &format!("topics.{topic_name}"),
                        format!(
                            "topic rate_hz ({topic_rate}) < subscriber '{sub_ref}' min_rate_hz ({sub_rate})"
                        ),
                    );
            }
        }
    }
}

/// Resolve "node/endpoint" to the publisher's min_rate_hz.
fn resolve_pub_min_rate(pub_ref: &str, manifest: &Manifest) -> Option<f64> {
    let (node_name, ep_name) = pub_ref.split_once('/')?;
    let node = manifest.nodes.get(node_name)?;
    let props = node.publishers.get(ep_name)?;
    props.min_rate_hz
}

/// Resolve "node/endpoint" to the subscriber's min_rate_hz.
fn resolve_sub_min_rate(sub_ref: &str, manifest: &Manifest) -> Option<f64> {
    let (node_name, ep_name) = sub_ref.split_once('/')?;
    let node = manifest.nodes.get(node_name)?;
    let props = node.subscribers.get(ep_name)?;
    props.min_rate_hz
}
