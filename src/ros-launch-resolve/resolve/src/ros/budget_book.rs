//! Every stated budget of the merged contracts, where it comes from, and
//! what checks it (phase 10 B7, B8).
//!
//! - `budget-rule` -- a budget whose `source:` says it is measured and states
//!   the observations must be what the rule gives: each term `period + max x
//!   (1 + margin)`, rounded up to the next millisecond, summed.
//! - [`stated_budgets`] -- one row per budget written in a contract, with its
//!   source and the checks that read it: the fault-reaction sum (a hazard
//!   charges it), a static rule that compares it with something
//!   (`scope-budget`, `reaction-within`, a parameter binding), a runtime
//!   monitor, or nothing. `--explain` prints the table, and
//!   `declared-not-charged` says the same thing about a `max_transport`.

use super::{
    manifest_graph::GlobalDataflowGraph,
    manifest_loader::{ManifestIndex, ResolvedManifest, TermKey},
    value_rules::site,
};
use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::{BudgetSource, DetectMechanism, Manifest, Measurement, SourceKind};
use std::collections::BTreeSet;

/// One budget as a contract states it.
#[derive(Debug, Clone)]
pub struct StatedBudget {
    pub scope_id: usize,
    /// The contract file's name.
    pub file: String,
    /// The dotted contract path (`nodes.h.paths.p.max_latency`).
    pub key: String,
    pub ms: f64,
    pub source: Option<BudgetSource>,
    /// What reads the number, in words; empty when nothing does.
    pub checked_by: Vec<String>,
}

/// Every budget a manifest states, as (dotted path, ms), in a stable order.
pub(crate) fn enumerate(m: &Manifest) -> Vec<(String, f64)> {
    let mut out: Vec<(String, f64)> = Vec::new();
    let mut put = |k: String, d: Option<ros_launch_manifest_types::duration::Duration>| {
        if let Some(d) = d {
            out.push((k, d.as_millis_f64()));
        }
    };
    for (h, d) in &m.hazards {
        put(format!("hazards.{h}.ftti"), d.ftti);
    }
    for (name, d) in &m.modes {
        put(
            format!("modes.{name}.window"),
            d.window.as_ref().map(|w| w.duration),
        );
    }
    for (n, nd) in &m.nodes {
        for (p, pd) in &nd.paths {
            let at = format!("nodes.{n}.paths.{p}");
            put(format!("{at}.max_latency"), pd.max_latency);
            put(format!("{at}.max_jitter"), pd.max_jitter);
            put(format!("{at}.trigger.timer.jitter"), pd.timer_jitter());
            if let Some(ss) = &pd.safe_state {
                match &ss.settle_profile {
                    Some(prof) => put(format!("{at}.safe_state.settle.response"), prof.response),
                    None => put(format!("{at}.safe_state.settle"), ss.settle),
                }
            }
        }
        for (side, eps) in [
            ("sub", &nd.subscribers),
            ("pub", &nd.publishers),
            ("cli", &nd.cli),
        ] {
            for (ep, props) in eps {
                let at = format!("nodes.{n}.{side}.{ep}");
                put(format!("{at}.max_age"), props.max_age);
                put(format!("{at}.max_transport"), props.max_transport);
                put(
                    format!("{at}.on_violation.within"),
                    props.on_violation.as_ref().and_then(|ov| ov.within),
                );
            }
        }
        for (ep, props) in &nd.srv {
            put(
                format!("nodes.{n}.srv.{ep}.max_response"),
                props.max_response,
            );
        }
    }
    for (t, d) in &m.topics {
        put(format!("topics.{t}.max_transport"), d.max_transport);
    }
    for (p, d) in &m.paths {
        put(format!("paths.{p}.max_latency"), d.max_latency);
        put(format!("paths.{p}.max_jitter"), d.max_jitter);
    }
    out
}

fn fmt_ms(v: f64) -> String {
    let s = format!("{v:.2}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// `100 + 14.84 x 1.2 = 117.81 -> 118, 25.04 x 1.2 = 30.05 -> 31`: the rule,
/// term by term.
pub fn rule_arithmetic(m: &Measurement) -> String {
    let f = 1.0 + m.margin;
    m.terms
        .iter()
        .map(|t| {
            let raw = t.period.map_or(0.0, |p| p.as_millis_f64()) + t.max.as_millis_f64() * f;
            let name = t.name.as_ref().map_or(String::new(), |n| format!("{n} "));
            let n = t.n.map_or(String::new(), |n| format!(" (n={n})"));
            let period = t.period.map_or(String::new(), |p| {
                format!("{} + ", fmt_ms(p.as_millis_f64()))
            });
            format!(
                "{name}{period}{} x {}{n} = {raw:.2} -> {}",
                fmt_ms(t.max.as_millis_f64()),
                fmt_ms(f),
                fmt_ms(t.budget_ms(m.margin))
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// `budget-rule` (phase 10 B7): a measured budget that states its
/// observations is what they give under the rule.
pub(crate) fn check_budget_rule(index: &ManifestIndex, diags: &mut Vec<Diagnostic>) {
    let mut ms: Vec<&ResolvedManifest> = index.manifests.values().collect();
    ms.sort_by_key(|m| m.scope_id);
    for m in ms {
        let stated = enumerate(&m.manifest);
        for (key, src) in &m.manifest.budget_sources {
            let Some(obs) = &src.measured else {
                continue;
            };
            let Some((_, v)) = stated.iter().find(|(k, _)| k == key) else {
                continue;
            };
            let rule = obs.budget_ms();
            if (rule - v).abs() <= 1e-6 {
                continue;
            }
            diags.push(Diagnostic {
                rule_id: "budget-rule".to_string(),
                severity: Severity::Error,
                message: format!(
                    "{}: `{key}: {}ms` is measured, but its observations give {}ms ({}; each \
                     term is the observed maximum plus {}%, rounded up to the next ms, and a \
                     timer's period takes no margin). State the maxima the budget was sized \
                     from, or a different source",
                    site(index, m.scope_id, key),
                    fmt_ms(*v),
                    fmt_ms(rule),
                    rule_arithmetic(obs),
                    fmt_ms(obs.margin * 100.0)
                ),
                path: key.clone(),
                span: None,
            });
        }
    }
}

/// The SOURCE cell of a stated budget.
pub fn source_label(src: Option<&BudgetSource>) -> String {
    match src {
        None => "unstated".to_string(),
        Some(BudgetSource {
            kind,
            measured: Some(m),
        }) => format!("{kind}: {}", rule_arithmetic(m)),
        Some(s) => s.kind.to_string(),
    }
}

/// The source of the stated budget `term`, when its manifest states one.
pub fn source_of<'a>(index: &'a ManifestIndex, term: &TermKey) -> Option<&'a BudgetSource> {
    index
        .manifests
        .get(&term.0)
        .and_then(|m| m.manifest.budget_sources.get(&term.1))
}

/// Whether a term is measured.
pub fn is_measured(index: &ManifestIndex, term: &TermKey) -> bool {
    source_of(index, term).is_some_and(|s| s.kind == SourceKind::Measured)
}

/// Every stated budget with its source and what checks it (phase 10 B8).
pub(crate) fn stated_budgets(
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
) -> Vec<StatedBudget> {
    let mut ms: Vec<&ResolvedManifest> = index.manifests.values().collect();
    ms.sort_by_key(|m| m.scope_id);
    let mut out = Vec::new();
    for m in ms {
        let file = m
            .contract_path
            .file_name()
            .map(|f| f.to_string_lossy().into_owned())
            .unwrap_or_default();
        for (key, v) in enumerate(&m.manifest) {
            let term: TermKey = (m.scope_id, key.clone());
            let checked_by = checks_of(index, graph, m, &term);
            out.push(StatedBudget {
                scope_id: m.scope_id,
                file: file.clone(),
                source: m.manifest.budget_sources.get(&key).cloned(),
                key,
                ms: v,
                checked_by,
            });
        }
    }
    out
}

/// What reads the stated budget `term` of manifest `m`.
fn checks_of(
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
    m: &ResolvedManifest,
    term: &TermKey,
) -> Vec<String> {
    let key = term.1.as_str();
    let parts: Vec<&str> = key.split('.').collect();
    let mut out: Vec<String> = Vec::new();
    // The fault-reaction arithmetic: the hazards whose rows contain it.
    let hazards: BTreeSet<&str> = index
        .budgets
        .iter()
        .filter(|r| r.terms.contains(term))
        .map(|r| r.hazard.as_str())
        .collect();
    if !hazards.is_empty() {
        out.push(format!(
            "fault-reaction sum ({})",
            hazards.into_iter().collect::<Vec<_>>().join(", ")
        ));
    }
    // A scope path's name may hold dots (`island.tor`).
    if key.starts_with("paths.") {
        if key.ends_with(".max_latency") {
            out.push("scope-budget".to_string());
            out.push("runtime monitor (max-latency-runtime)".to_string());
        } else if key.ends_with(".max_jitter") {
            out.push("jitter-feasibility".to_string());
        }
        return out;
    }
    // A scope-path budget charges the latencies and sampling waits of the
    // nodes its route crosses.
    const CROSSED: &str = "scope-budget (where a scope path's route crosses the node)";
    let scope_budgets = index
        .scope_paths
        .iter()
        .any(|p| p.path.max_latency.is_some());
    // An edge whose publisher is a contract node of this tree: what a scope
    // path can charge a `max_transport` on. An `external: pub` publisher
    // is outside it.
    let in_tree = |e: &&super::manifest_graph::GlobalEdge| {
        super::value_rules::contract_node(index, &e.from).is_some()
    };
    match parts.as_slice() {
        ["hazards", h, "ftti"] => {
            if index.budgets.iter().any(|r| &r.hazard == h) {
                out.push("fault-reaction sum (the interval)".to_string());
            }
        }
        ["modes", name, "window"] => {
            if m.manifest
                .modes
                .get(*name)
                .and_then(|d| d.window.as_ref())
                .is_some_and(|w| w.param.is_some())
            {
                out.push("window-param".to_string());
            }
        }
        ["nodes", n, "paths", p, "max_latency"] => {
            let reaction_within = m.manifest.nodes.get(*n).is_some_and(|nd| {
                nd.subscribers.values().any(|s| {
                    s.on_violation
                        .as_ref()
                        .is_some_and(|ov| ov.reaction == *p && ov.within.is_some())
                })
            });
            if reaction_within {
                out.push("reaction-within".to_string());
            }
            if scope_budgets {
                out.push(CROSSED.to_string());
            }
            out.push("runtime monitor (the realizer's path deadline)".to_string());
        }
        ["nodes", n, "paths", p, "trigger", "timer", "jitter"] => {
            // The window's owner reads its deadline on this tick.
            let owns_window = m.manifest.modes.values().any(|md| {
                md.window
                    .as_ref()
                    .and_then(|w| w.param.as_ref())
                    .is_some_and(|pr| pr.node == *n)
            });
            let timer = m
                .manifest
                .nodes
                .get(*n)
                .and_then(|nd| nd.paths.get(*p))
                .is_some();
            if owns_window && timer {
                out.push("window-expiry".to_string());
            }
            if scope_budgets {
                out.push(CROSSED.to_string());
            }
        }
        ["nodes", _, "paths", _, "max_jitter"] => out.push("jitter-feasibility".to_string()),
        ["nodes", _, "sub", _, "on_violation", "within"] => {
            out.push("reaction-within".to_string());
        }
        ["nodes", n, "sub", ep, "max_age"] => {
            let props = m
                .manifest
                .nodes
                .get(*n)
                .and_then(|d| d.subscribers.get(*ep));
            if props.is_some_and(|p| p.max_age_param.is_some()) {
                out.push("max-age-param".to_string());
            }
            match props
                .and_then(|p| p.on_violation.as_ref())
                .map(|ov| ov.mechanism)
            {
                Some(DetectMechanism::Application | DetectMechanism::Diagnostics) => {
                    out.push("the node itself (on_violation.mechanism)".to_string());
                }
                _ => out.push("runtime monitor (max-age-runtime)".to_string()),
            }
        }
        ["nodes", n, "sub", ep, "max_transport"] => {
            let fqn = super::manifest_loader::resolve_node_fqn(index, m.scope_id, &m.ns, n);
            if graph
                .edges
                .iter()
                .filter(in_tree)
                .any(|e| e.to == fqn && e.sub_endpoint == *ep)
            {
                out.push("scope-budget (an in-tree edge into it)".to_string());
            }
        }
        ["topics", _, "max_transport"] => {
            let t = parts[1..parts.len() - 1].join(".");
            let fqn = qualify(&m.ns, &t);
            if graph.edges.iter().filter(in_tree).any(|e| e.topic == fqn) {
                out.push("scope-budget (an in-tree edge on it)".to_string());
            }
        }
        ["nodes", _, "srv", _, "max_response"] => out.push("response-blocking".to_string()),
        _ => {}
    }
    out
}

fn qualify(ns: &str, name: &str) -> String {
    if name.starts_with('/') {
        name.to_string()
    } else if ns.is_empty() || ns == "/" {
        format!("/{name}")
    } else {
        format!("{}/{name}", ns.trim_end_matches('/'))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_launch_manifest_types::parse_manifest_str;

    /// The rule reproduces the island's numbers: the link from one maximum,
    /// the handler's hop from a tick and a work term.
    #[test]
    fn the_rule_reproduces_the_islands_budgets() {
        let m = parse_manifest_str(
            "version: 1\nnodes:\n  h:\n    sub:\n      a:\n        max_transport: { duration: \
             56ms, source: { measured: { max: 46.42ms, n: 12, margin: 0.2 } } }\n    paths:\n      \
             r:\n        trigger: { input: [a] }\n        max_latency: { duration: 149ms, source: \
             { measured: { margin: 0.2, terms: { tick: { period: 100ms, max: 14.84ms }, work: { \
             max: 25.04ms } } } } }\n",
        )
        .unwrap();
        let src = &m.budget_sources;
        let link = src["nodes.h.sub.a.max_transport"]
            .measured
            .as_ref()
            .unwrap();
        assert_eq!(link.budget_ms(), 56.0);
        assert_eq!(rule_arithmetic(link), "46.42 x 1.2 (n=12) = 55.70 -> 56");
        let hop = src["nodes.h.paths.r.max_latency"]
            .measured
            .as_ref()
            .unwrap();
        assert_eq!(hop.budget_ms(), 149.0);
        assert_eq!(
            rule_arithmetic(hop),
            "tick 100 + 14.84 x 1.2 = 117.81 -> 118, work 25.04 x 1.2 = 30.05 -> 31"
        );
        let stated = enumerate(&m);
        assert!(stated.contains(&("nodes.h.paths.r.max_latency".to_string(), 149.0)));
        assert!(stated.contains(&("nodes.h.sub.a.max_transport".to_string(), 56.0)));
    }
}
