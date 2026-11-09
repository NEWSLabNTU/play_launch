//! The static rules of a level's `commands` (rlm v0.3.13, play_launch phase
//! 100 F1; rlm `docs/design/safety-functions.md` §5).
//!
//! A topic named in any mode's `commands` is a FINAL ELEMENT: what that
//! level of the fallback sequence commands. Two facts follow from the
//! resolved graph with no further key:
//!
//! - `arbiter-bypass`: the topic has exactly one publisher, its arbiter.
//!   With one publisher every route that reaches the topic passes through
//!   it, so no separate bypass-route rule is needed (phase 102 finding 4).
//! - `commands-unreached`: the level's reaction reaches the topic, by
//!   ending on it or upstream of it. A level with no reaction of its own
//!   (a degraded mode that only limits) is not judged here.
//!
//! The predicates' fields are checked with every other predicate's, as
//! `commands-field-unknown` (`value_rules::check_predicates`).

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::ExternalSide;

use super::{
    manifest_graph::GlobalDataflowGraph, manifest_loader::ManifestIndex, value_rules::site,
};

fn diag(rule: &str, severity: Severity, message: String, path: String) -> Diagnostic {
    Diagnostic {
        rule_id: rule.to_string(),
        severity,
        message,
        path,
        span: None,
    }
}

/// Topics reachable from `start` through nodes: a node subscribing to a
/// reached topic reaches every topic it publishes. Node-level, so it may
/// call a topic reached that no path of the node actually produces from
/// that input; that errs toward "reached", never toward a false refusal.
fn downstream(graph: &GlobalDataflowGraph, start: &[String]) -> BTreeSet<String> {
    let mut publishes: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for (topic, pubs) in &graph.topic_publishers {
        for p in pubs {
            publishes
                .entry(p.as_str())
                .or_default()
                .push(topic.as_str());
        }
    }
    let mut seen: BTreeSet<String> = start.iter().cloned().collect();
    let mut queue: VecDeque<String> = start.iter().cloned().collect();
    while let Some(t) = queue.pop_front() {
        for node in graph.topic_subscribers.get(&t).into_iter().flatten() {
            for out in publishes.get(node.as_str()).into_iter().flatten() {
                if seen.insert(out.to_string()) {
                    queue.push_back(out.to_string());
                }
            }
        }
    }
    seen
}

/// `arbiter-bypass` and `commands-unreached` over every mode's `commands`.
pub(crate) fn check_commands(
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
    diags: &mut Vec<Diagnostic>,
) {
    // A topic two levels command is one final element: judge its arbiter once.
    let mut arbiters_judged: BTreeSet<&str> = BTreeSet::new();
    for m in &index.modes {
        for topic in m.commands.keys() {
            let at = format!("modes.{}.commands.{topic}", m.name);
            let where_ = site(index, m.scope_id, &at);
            if arbiters_judged.insert(topic.as_str()) {
                let mut pubs: Vec<String> = graph
                    .topic_publishers
                    .get(topic)
                    .cloned()
                    .unwrap_or_default();
                pubs.sort();
                pubs.dedup();
                let external = matches!(
                    index.externals.get(topic),
                    Some(ExternalSide::Pub | ExternalSide::Both)
                );
                match (pubs.len(), external) {
                    (1, false) | (0, true) => {}
                    (0, false) => diags.push(diag(
                        "arbiter-bypass",
                        Severity::Error,
                        format!(
                            "{where_}: mode '{}' commands '{topic}', which has no publisher in \
                             the resolved graph -- a final element has exactly one, its \
                             arbiter. Declare the node that publishes it",
                            m.name
                        ),
                        at.clone(),
                    )),
                    (n, ext) => diags.push(diag(
                        "arbiter-bypass",
                        Severity::Error,
                        format!(
                            "{where_}: mode '{}' commands '{topic}', which has {} publishers \
                             ({}{}) -- a final element has exactly one, its arbiter, or a \
                             command can reach it around the level's reaction",
                            m.name,
                            n + usize::from(ext),
                            pubs.join(", "),
                            if ext { ", and an external one" } else { "" }
                        ),
                        at.clone(),
                    )),
                }
            }
            let Some(reaction) = &m.decl.reaction else {
                continue;
            };
            let Some(sp) = index
                .scope_paths
                .iter()
                .find(|p| p.scope_id == m.scope_id && &p.path_name == reaction)
            else {
                // An unknown reaction is the mode rules' finding.
                continue;
            };
            if sp.output_topics.is_empty() {
                continue;
            }
            if !downstream(graph, &sp.output_topics).contains(topic) {
                diags.push(diag(
                    "commands-unreached",
                    Severity::Error,
                    format!(
                        "{where_}: mode '{}' commands '{topic}', but its reaction '{reaction}' \
                         ends at {} and nothing downstream of it publishes '{topic}' -- the \
                         level would command a topic its own reaction never reaches. End the \
                         reaction on the commanded topic, or on a topic upstream of it",
                        m.name,
                        sp.output_topics.join(", ")
                    ),
                    at.clone(),
                ));
            }
        }
    }
}

/// The topic an endpoint of `node` is wired to: a subscription when `sub`,
/// else a publication.
fn topic_of<'a>(index: &'a ManifestIndex, node: &str, ep: &str, sub: bool) -> Option<&'a str> {
    let full = format!("{node}/{ep}");
    index
        .topics
        .values()
        .find(|t| {
            if sub {
                t.subscribers.contains(&full)
            } else {
                t.publishers.contains(&full)
            }
        })
        .map(|t| t.fqn.as_str())
}

/// The output topics of every event chain a hazard's reaction runs: its
/// `reaction:` chain, or each level's reaction in the mode it enters and
/// that mode's fallback sequence.
fn reaction_outputs(
    index: &ManifestIndex,
    h: &super::manifest_loader::ResolvedHazard,
) -> Vec<String> {
    let Some(entry) = h.decl.entry() else {
        return Vec::new();
    };
    let mode = |name: &str| {
        index
            .modes
            .iter()
            .find(|m| m.scope_id == h.scope_id && m.name == name)
    };
    let mut chains: Vec<String> = Vec::new();
    match mode(entry) {
        Some(m) => {
            for name in std::iter::once(&m.name).chain(m.decl.fallback.iter()) {
                if let Some(r) = mode(name).and_then(|l| l.decl.reaction.clone()) {
                    chains.push(r);
                }
            }
        }
        None => chains.push(entry.to_string()),
    }
    index
        .scope_paths
        .iter()
        .filter(|p| p.scope_id == h.scope_id && chains.contains(&p.path_name))
        .flat_map(|p| p.output_topics.clone())
        .collect()
}

/// `reaction-waits-on-traffic` (play_launch phase 100 F4): a node reads a
/// hazard's monitored topic only as STATE, and the path that reaches the
/// reaction's output is triggered by another input. The stop then goes out
/// only when that input's next message arrives, which no safety
/// requirement bounds: a command stream can stop. Safe-ROS's
/// `cmd_vel_interceptor` is the shape (`contract_safe_ros`). A node that
/// also has a path triggered by the monitored topic reaching the output is
/// not this shape.
pub(crate) fn check_reaction_waits_on_traffic(
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
    diags: &mut Vec<Diagnostic>,
) {
    use ros_launch_manifest_types::EffectiveTrigger;
    for h in &index.hazards {
        let outputs: BTreeSet<String> = reaction_outputs(index, h).into_iter().collect();
        if outputs.is_empty() {
            continue;
        }
        let guards: BTreeSet<&str> = h
            .guards
            .iter()
            .flat_map(|g| g.members.iter().map(String::as_str))
            .collect();
        for resolved in index.manifests.values() {
            for (name, node) in &resolved.manifest.nodes {
                let fqn = super::manifest_loader::resolve_node_fqn(
                    index,
                    resolved.scope_id,
                    &resolved.ns,
                    name,
                );
                // The node's subscriptions to a monitored topic read as state.
                let state_eps: Vec<&str> = node
                    .subscribers
                    .iter()
                    .filter(|(ep, props)| {
                        props.state == Some(true)
                            && topic_of(index, &fqn, ep, true).is_some_and(|t| guards.contains(t))
                    })
                    .map(|(ep, _)| ep.as_str())
                    .collect();
                if state_eps.is_empty() {
                    continue;
                }
                let reaches = |p: &ros_launch_manifest_types::PathDecl| {
                    let outs: Vec<String> = p
                        .output
                        .iter()
                        .filter_map(|o| topic_of(index, &fqn, o, false).map(str::to_string))
                        .collect();
                    !outs.is_empty() && !downstream(graph, &outs).is_disjoint(&outputs)
                };
                let mut proper = false;
                let mut waits: Vec<(String, Vec<String>)> = Vec::new();
                for (pname, p) in &node.paths {
                    if !reaches(p) {
                        continue;
                    }
                    match p.effective_trigger() {
                        EffectiveTrigger::Input(inputs) => {
                            if inputs.iter().any(|i| state_eps.contains(&i.as_str())) {
                                proper = true;
                            } else {
                                waits.push((pname.clone(), inputs));
                            }
                        }
                        // A timer bounds the wait by its period, which the
                        // walk charges as a sampling hop (phase 82 W4).
                        _ => proper = true,
                    }
                }
                if proper || waits.is_empty() {
                    continue;
                }
                let at = format!("nodes.{name}.sub.{}", state_eps[0]);
                let where_ = site(index, resolved.scope_id, &at);
                for (pname, inputs) in waits {
                    let described: Vec<String> = inputs
                        .iter()
                        .map(|i| {
                            let topic = topic_of(index, &fqn, i, true);
                            let rate = topic
                                .and_then(|t| index.topics.get(t))
                                .and_then(|t| t.derived_rate_hz.or(t.rate_hz))
                                .filter(|r| *r > 0.0);
                            match (topic, rate) {
                                (Some(t), Some(r)) => format!(
                                    "'{t}' (up to {:.2}ms at its {r} Hz, and never if it stops)",
                                    1000.0 / r
                                ),
                                (Some(t), None) => format!("'{t}' (whose rate is unknown)"),
                                (None, _) => format!("'{i}'"),
                            }
                        })
                        .collect();
                    diags.push(diag(
                        "reaction-waits-on-traffic",
                        Severity::Error,
                        format!(
                            "{where_}: hazard '{}': {fqn} reads the monitored {} as state, and \
                             its path '{pname}' that reaches the reaction's output is triggered \
                             by {} -- the reaction goes out only when that traffic next arrives, \
                             which nothing bounds. Trigger a path on the monitored topic, or on a \
                             timer",
                            h.name,
                            state_eps.join(", "),
                            described.join(", ")
                        ),
                        at.clone(),
                    ));
                }
            }
        }
    }
}
