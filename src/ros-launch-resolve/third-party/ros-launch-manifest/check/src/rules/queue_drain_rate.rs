//! Rule: a node's `buffer: queue` subscriptions must be drainable by its
//! consuming timer path (Vocabulary v2, Phase 44.1 §3/§5).
//!
//! `buffer: queue` (vs the default `latest`) means backlog, not
//! staleness, is the failure mode — the EKF pattern
//! (`ekf_localizer.cpp:137-215`, W3 census): a bounded queue drained
//! batch-wise by the node's timer callback. The timer's own rate must be
//! at least the sum of the producing topics' declared rates, or the
//! queue accumulates backlog every period.
//!
//! Checked per node: sum the declared `rate_hz` of every topic backing a
//! `buffer: queue` subscriber (skipped when a topic doesn't declare
//! `rate_hz` — nothing to sum for that one), then compare against every
//! `timer`-triggered path's `rate_hz` on the same node. A node with queue
//! subs but no timer path is not checked here (nothing to compare
//! against — not this rule's job to require one).

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::{Buffer, EffectiveTrigger, Manifest};

pub struct QueueDrainRateRule;

impl ValidationRule for QueueDrainRateRule {
    fn id(&self) -> &str {
        "queue-drain-rate"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        for (node_name, node) in &manifest.nodes {
            let queue_subs: Vec<&str> = node
                .subscribers
                .iter()
                .filter(|(_, props)| {
                    props.state.unwrap_or(false) && props.buffer == Some(Buffer::Queue)
                })
                .map(|(ep, _)| ep.as_str())
                .collect();
            if queue_subs.is_empty() {
                continue;
            }

            let producer_rate_sum: f64 = queue_subs
                .iter()
                .filter_map(|ep| {
                    super::endpoint_topic::topic_for_sub(manifest, node_name, ep)
                        .and_then(|(_, t)| t.rate_hz)
                })
                .sum();
            if producer_rate_sum <= 0.0 {
                continue;
            }

            for (path_name, path) in &node.paths {
                let EffectiveTrigger::Timer { rate_hz } = path.effective_trigger() else {
                    continue;
                };
                if rate_hz < producer_rate_sum {
                    ctx.warning(
                        self.id(),
                        &format!("nodes.{node_name}.paths.{path_name}"),
                        format!(
                            "node '{node_name}' timer path '{path_name}' rate_hz ({rate_hz}) is \
                             less than the sum of its 'buffer: queue' subscriptions' producer \
                             rates ({producer_rate_sum}, from {queue_subs:?}) — the queue will \
                             accumulate backlog every period"
                        ),
                    );
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
        QueueDrainRateRule.check(&manifest, &graph, &mut ctx);
        ctx.diagnostics
    }

    #[test]
    fn timer_slower_than_queue_producers_warns() {
        let yaml = r#"
version: 1
nodes:
  ekf:
    sub:
      twist:
        state: true
        buffer: queue
    pub:
      out: {}
    paths:
      predict:
        trigger: { timer: { rate_hz: 50 } }
        output: [out]
topics:
  twist:
    type: t
    pub: [x/twist]
    sub: [ekf/twist]
    rate_hz: 100
"#;
        let warnings = run(yaml);
        assert_eq!(warnings.len(), 1, "got: {warnings:?}");
    }

    #[test]
    fn timer_fast_enough_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  ekf:
    sub:
      twist:
        state: true
        buffer: queue
    pub:
      out: {}
    paths:
      predict:
        trigger: { timer: { rate_hz: 200 } }
        output: [out]
topics:
  twist:
    type: t
    pub: [x/twist]
    sub: [ekf/twist]
    rate_hz: 100
"#;
        assert!(run(yaml).is_empty());
    }

    #[test]
    fn no_queue_subs_does_not_fire() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      twist:
        state: true
    pub:
      out: {}
    paths:
      predict:
        trigger: { timer: { rate_hz: 1 } }
        output: [out]
topics:
  twist:
    type: t
    pub: [x/twist]
    sub: [n/twist]
    rate_hz: 100
"#;
        assert!(run(yaml).is_empty());
    }

    #[test]
    fn queue_subs_without_declared_rate_skips() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      twist:
        state: true
        buffer: queue
    pub:
      out: {}
    paths:
      predict:
        trigger: { timer: { rate_hz: 1 } }
        output: [out]
topics:
  twist:
    type: t
    pub: [x/twist]
    sub: [n/twist]
"#;
        assert!(run(yaml).is_empty());
    }

    #[test]
    fn queue_subs_without_timer_path_does_not_fire() {
        let yaml = r#"
version: 1
nodes:
  n:
    sub:
      twist:
        state: true
        buffer: queue
      cause: {}
    pub:
      out: {}
    paths:
      main:
        trigger: { input: [cause] }
        output: [out]
topics:
  twist:
    type: t
    pub: [x/twist]
    sub: [n/twist]
    rate_hz: 100
"#;
        assert!(run(yaml).is_empty());
    }
}
