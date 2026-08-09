//! Rule: sanity checks on drop tolerance values.
//!
//! - `max_drop_rate` (derived from max_count) must be in [0, 1]
//! - `max_consecutive` must be > 0
//! - effective delivery rate must satisfy subscriber min_rate_hz
//! - scope path drop rates must be in valid range

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;

pub struct DropSanityRule;

impl ValidationRule for DropSanityRule {
    fn id(&self) -> &str {
        "drop-sanity"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        // (a) + (b) Check node path drop specs
        for (node_name, node) in &manifest.nodes {
            for (path_name, path) in &node.paths {
                if let Some(drop) = &path.drop {
                    let loc = format!("nodes.{node_name}.paths.{path_name}.drop");
                    check_drop_values(drop, &loc, ctx, self.id());
                }
            }
        }

        // (a) + (b) Check topic drop specs
        for (topic_name, topic) in &manifest.topics {
            if let Some(drop) = &topic.drop {
                let loc = format!("topics.{topic_name}.drop");
                check_drop_values(drop, &loc, ctx, self.id());
            }
        }

        // (a) + (b) Check scope path drop specs
        for (path_name, path) in &manifest.paths {
            if let Some(drop) = &path.drop {
                let loc = format!("paths.{path_name}.drop");
                check_drop_values(drop, &loc, ctx, self.id());
            }
        }

        // (c) For each topic with rate_hz and drop, check that effective delivery
        // rate satisfies all subscribers' min_rate_hz
        for (topic_name, topic) in &manifest.topics {
            let topic_rate = match topic.rate_hz {
                Some(r) => r,
                None => continue,
            };
            let drop_rate = match &topic.drop {
                Some(d) => match &d.max_count {
                    Some(c) => c.drop_rate(),
                    None => continue,
                },
                None => continue,
            };

            let effective_rate = topic_rate * (1.0 - drop_rate);

            for sub_ref in &topic.subscribers {
                if let Some(min_rate) = resolve_sub_min_rate(sub_ref, manifest)
                    && effective_rate < min_rate
                {
                    ctx.error(
                        self.id(),
                        &format!("topics.{topic_name}"),
                        format!(
                            "effective delivery rate ({effective_rate:.2} Hz = {topic_rate} Hz * {:.4} delivery) \
                             < subscriber '{sub_ref}' min_rate_hz ({min_rate})",
                            1.0 - drop_rate
                        ),
                    );
                }
            }
        }
    }
}

fn check_drop_values(
    drop: &ros_launch_manifest_types::DropSpec,
    loc: &str,
    ctx: &mut CheckContext,
    rule_id: &str,
) {
    // (a) max_drop_rate in [0, 1]
    if let Some(count) = &drop.max_count {
        let rate = count.drop_rate();
        if !(0.0..=1.0).contains(&rate) {
            ctx.error(
                rule_id,
                loc,
                format!("max_drop_rate {rate:.4} is outside [0, 1] (from {count})"),
            );
        }
        if count.n > count.w {
            ctx.error(
                rule_id,
                loc,
                format!(
                    "drop count {} > window size {} — drop rate > 1.0",
                    count.n, count.w
                ),
            );
        }
    }

    // (b) max_consecutive > 0
    if let Some(k) = drop.max_consecutive
        && k == 0
    {
        ctx.error(
            rule_id,
            loc,
            "max_consecutive is 0 — this means zero drops are allowed, which is likely a mistake. \
             Use max_count: 0/N instead."
                .to_string(),
        );
    }
}

/// Resolve "node/endpoint" to the subscriber's min_rate_hz.
fn resolve_sub_min_rate(sub_ref: &str, manifest: &Manifest) -> Option<f64> {
    let (node_name, ep_name) = sub_ref.split_once('/')?;
    let node = manifest.nodes.get(node_name)?;
    let props = node.subscribers.get(ep_name)?;
    props.min_rate_hz
}
