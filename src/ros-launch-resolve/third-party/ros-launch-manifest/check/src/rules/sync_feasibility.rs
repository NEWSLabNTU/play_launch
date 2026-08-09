//! Rule: fan-in `sync:` window vs declared input rates (Vocabulary v2,
//! Phase 44.1 §2/§5).
//!
//! For an `exact`/`approximate` policy, `max_interval_ms` must be able to
//! span the slowest declared input's inter-arrival period — otherwise the
//! synchronizer can never find a match window wide enough to include that
//! input's messages. For `timeout_any`, `timeout_ms` must be at least the
//! slowest input's period — otherwise the collect-until-timeout window
//! always expires before the slowest input can contribute.
//!
//! Only checkable when at least one input endpoint's topic declares
//! `rate_hz` — with no declared rates, there is nothing to compare
//! against (silently skipped, not an error: rate declaration is
//! optional).

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::{EffectiveTrigger, Manifest, SyncPolicy};

pub struct SyncFeasibilityRule;

impl ValidationRule for SyncFeasibilityRule {
    fn id(&self) -> &str {
        "sync-feasibility"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        for (node_name, node) in &manifest.nodes {
            for (path_name, path) in &node.paths {
                let Some(sync) = &path.sync else {
                    continue;
                };
                let EffectiveTrigger::Input(endpoints) = path.effective_trigger() else {
                    continue;
                };

                let rates: Vec<f64> = endpoints
                    .iter()
                    .filter_map(|ep| {
                        super::endpoint_topic::topic_for_sub(manifest, node_name, ep)
                            .and_then(|(_, t)| t.rate_hz)
                    })
                    .collect();
                let Some(slowest_rate) = rates
                    .iter()
                    .cloned()
                    .fold(None, |acc, r| Some(acc.map_or(r, |a: f64| a.min(r))))
                else {
                    continue;
                };
                if slowest_rate <= 0.0 {
                    continue;
                }
                let slowest_period_ms = 1000.0 / slowest_rate;

                match sync.policy {
                    SyncPolicy::Exact | SyncPolicy::Approximate => {
                        let Some(window) = sync.max_interval_ms else {
                            continue;
                        };
                        if window < slowest_period_ms {
                            ctx.warning(
                                self.id(),
                                &format!("nodes.{node_name}.paths.{path_name}.sync"),
                                format!(
                                    "node '{node_name}' path '{path_name}' sync.max_interval_ms \
                                     ({window}) is less than the slowest input's period \
                                     ({slowest_period_ms:.2}ms, from rate {slowest_rate}Hz) — the \
                                     match window can never span the slowest input"
                                ),
                            );
                        }
                    }
                    SyncPolicy::TimeoutAny => {
                        let Some(timeout) = sync.timeout_ms else {
                            continue;
                        };
                        if timeout < slowest_period_ms {
                            ctx.warning(
                                self.id(),
                                &format!("nodes.{node_name}.paths.{path_name}.sync"),
                                format!(
                                    "node '{node_name}' path '{path_name}' sync.timeout_ms \
                                     ({timeout}) is less than the slowest input's period \
                                     ({slowest_period_ms:.2}ms, from rate {slowest_rate}Hz) — the \
                                     timeout always expires before the slowest input can \
                                     contribute"
                                ),
                            );
                        }
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
        SyncFeasibilityRule.check(&manifest, &graph, &mut ctx);
        ctx.diagnostics
    }

    #[test]
    fn approximate_window_too_small_warns() {
        let yaml = r#"
version: 1
nodes:
  fuse:
    sub:
      a: {}
      b: {}
    pub:
      out: {}
    paths:
      main:
        trigger: { input: [a, b] }
        sync:
          policy: approximate
          max_interval_ms: 5
        output: [out]
topics:
  ta:
    type: t
    pub: [x/a]
    sub: [fuse/a]
    rate_hz: 10
  tb:
    type: t
    pub: [x/b]
    sub: [fuse/b]
    rate_hz: 2
"#;
        // slowest input is 2Hz -> 500ms period, window 5ms is way too small
        let warnings = run(yaml);
        assert_eq!(warnings.len(), 1, "got: {warnings:?}");
    }

    #[test]
    fn approximate_window_sufficient_does_not_warn() {
        let yaml = r#"
version: 1
nodes:
  fuse:
    sub:
      a: {}
      b: {}
    pub:
      out: {}
    paths:
      main:
        trigger: { input: [a, b] }
        sync:
          policy: approximate
          max_interval_ms: 600
        output: [out]
topics:
  ta:
    type: t
    pub: [x/a]
    sub: [fuse/a]
    rate_hz: 10
  tb:
    type: t
    pub: [x/b]
    sub: [fuse/b]
    rate_hz: 2
"#;
        assert!(run(yaml).is_empty());
    }

    #[test]
    fn timeout_any_too_small_warns() {
        let yaml = r#"
version: 1
nodes:
  fuse:
    sub:
      a: {}
      b: {}
    pub:
      out: {}
    paths:
      main:
        trigger: { input: [a, b] }
        sync:
          policy: timeout_any
          timeout_ms: 10
        output: [out]
topics:
  ta:
    type: t
    pub: [x/a]
    sub: [fuse/a]
    rate_hz: 10
  tb:
    type: t
    pub: [x/b]
    sub: [fuse/b]
    rate_hz: 2
"#;
        assert_eq!(run(yaml).len(), 1);
    }

    #[test]
    fn no_declared_rates_skips_silently() {
        let yaml = r#"
version: 1
nodes:
  fuse:
    sub:
      a: {}
      b: {}
    pub:
      out: {}
    paths:
      main:
        trigger: { input: [a, b] }
        sync:
          policy: approximate
          max_interval_ms: 1
        output: [out]
"#;
        assert!(run(yaml).is_empty());
    }
}
