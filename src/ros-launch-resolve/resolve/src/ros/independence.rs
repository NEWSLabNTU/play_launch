//! Independence and freedom from interference (rlm v0.3.13, play_launch
//! phase 101 T1; rlm `docs/design/fault-tolerance.md` §4.5).
//!
//! `independence: [{ of, not_sharing }]` is a requirement checked against
//! the deployment facts this tree has:
//!
//! - `container`: two composables loaded into one container.
//! - `process`: the same, since composables in one container share a
//!   process under `observable` and `stock`. The model does not carry the
//!   container mode; `check` and `resolve` assume the shared case and say
//!   so, `--container-mode isolated` states the other, and `launch` passes
//!   the mode it runs (`ContractSources::process_per_composable`).
//! - `cpus`: overlapping CPU sets in the scheduling platform file. Checked
//!   only where a platform file resolved (`check_cpus`); without one the
//!   fact is reported unchecked, never passed.
//!
//! `interference-shared-process` is derived with no key: nodes of different
//! derived integrity in one container (one process, same assumption).

use std::collections::{BTreeMap, BTreeSet};

use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::SharedFact;

use super::{
    manifest_graph::GlobalDataflowGraph,
    manifest_loader::{ManifestIndex, ResolvedIndependence},
    value_rules::site,
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

/// The nodes one `of` entry names: a node FQN as written, or a function's
/// nodes, the publishers of its member topics.
fn nodes_of(
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
    ind: &ResolvedIndependence,
    entry: &str,
) -> BTreeSet<String> {
    if let Some(f) = index
        .functions
        .iter()
        .find(|f| f.scope_id == ind.scope_id && f.name == entry)
    {
        return f
            .group
            .members
            .iter()
            .flat_map(|t| graph.topic_publishers.get(t).cloned().unwrap_or_default())
            .collect();
    }
    BTreeSet::from([entry.to_string()])
}

/// The process a node runs in, on the shared-container assumption: its
/// container's, else its own.
fn process_of<'a>(index: &'a ManifestIndex, node: &'a str) -> &'a str {
    if index.process_per_composable {
        return node;
    }
    index
        .executor_of
        .get(node)
        .map(String::as_str)
        .unwrap_or(node)
}

/// One `of` entry as written, with the nodes it names.
type Entry = (String, BTreeSet<String>);

/// Every pair of distinct `of` entries, with each one's nodes.
fn pairs(
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
    ind: &ResolvedIndependence,
) -> Vec<(Entry, Entry)> {
    let sets: Vec<Entry> = ind
        .of
        .iter()
        .map(|e| (e.clone(), nodes_of(index, graph, ind, e)))
        .collect();
    let mut out = Vec::new();
    for i in 0..sets.len() {
        for j in i + 1..sets.len() {
            out.push((sets[i].clone(), sets[j].clone()));
        }
    }
    out
}

/// `independence-violated` for `process` and `container`, and
/// `interference-shared-process`. Called from `load_manifests`.
pub(crate) fn check_independence(
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
    diags: &mut Vec<Diagnostic>,
) {
    for ind in &index.independence {
        let at = format!("independence[{}]", ind.index);
        let where_ = site(index, ind.scope_id, &format!("independence.{}", ind.index));
        // play_launch phase 103 D6: an entry naming no node of this model
        // (another host's node, a typo, a function no node publishes) shares
        // nothing, so every pair with it passed. It is unchecked, and said.
        for entry in &ind.of {
            let nodes = nodes_of(index, graph, ind, entry);
            let absent: Vec<&String> = nodes
                .iter()
                .filter(|n| !graph.nodes.contains_key(n.as_str()))
                .collect();
            if nodes.is_empty() || !absent.is_empty() {
                let what = if nodes.is_empty() {
                    format!("'{entry}' names no node of this model")
                } else if nodes.len() == 1 && nodes.contains(entry) {
                    format!("'{entry}' is absent from this model")
                } else {
                    format!(
                        "'{entry}' names {}, absent from this model",
                        absent
                            .iter()
                            .map(|n| format!("'{n}'"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    )
                };
                diags.push(diag(
                    "independence-unchecked",
                    Severity::Warning,
                    format!(
                        "{where_}: {what} -- a node on another host, a misspelling, or a \
                         function no node publishes. A node absent from the model shares \
                         nothing with anything, so its pairs are UNCHECKED, not independent. \
                         One resolved model is one host"
                    ),
                    format!("{at}.of"),
                ));
            }
        }
        for ((a, na), (b, nb)) in pairs(index, graph, ind) {
            for fact in &ind.not_sharing {
                let shared: Vec<String> = na
                    .iter()
                    .flat_map(|x| nb.iter().map(move |y| (x, y)))
                    .filter_map(|(x, y)| {
                        let same = match fact {
                            SharedFact::Process => process_of(index, x) == process_of(index, y),
                            SharedFact::Container => {
                                x == y
                                    || matches!(
                                        (index.executor_of.get(x), index.executor_of.get(y)),
                                        (Some(cx), Some(cy)) if cx == cy
                                    )
                                    || index.executor_of.get(x) == Some(y)
                                    || index.executor_of.get(y) == Some(x)
                            }
                            SharedFact::Cpus => return None,
                        };
                        same.then(|| {
                            if x == y {
                                x.clone()
                            } else {
                                format!("{x} and {y} ({})", process_of(index, x))
                            }
                        })
                    })
                    .collect();
                if shared.is_empty() {
                    continue;
                }
                let assumed = if *fact == SharedFact::Process {
                    " (composables in one container share a process under `observable` and \
                     `stock`, the mode this check assumed; under `isolated` they do not -- pass \
                     `--container-mode isolated` if that is how the system runs)"
                } else {
                    ""
                };
                diags.push(diag(
                    "independence-violated",
                    Severity::Error,
                    format!(
                        "{where_}: '{a}' and '{b}' must not share a {}, and share one: {}{assumed}",
                        fact.as_str(),
                        shared.join("; ")
                    ),
                    format!("{at}.not_sharing"),
                ));
            }
        }
    }
}

/// `interference-shared-process`, derived: nodes of different derived
/// integrity in one container. Called once criticality is derived.
pub(crate) fn check_interference(index: &mut ManifestIndex) {
    // Under `isolated` a container shares no process with its composables.
    if index.process_per_composable {
        return;
    }
    let mut diags = Vec::new();
    let mut by_container: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for (node, container) in &index.executor_of {
        by_container
            .entry(container.as_str())
            .or_default()
            .push(node.as_str());
    }
    for (container, members) in by_container {
        let rank = |n: &str| index.derived_criticality.get(n).map_or(0, |c| c.rank);
        let level = |n: &str| {
            index
                .derived_criticality
                .get(n)
                .map_or("none".to_string(), |c| c.level.clone())
        };
        let ranks: BTreeSet<usize> = members.iter().map(|n| rank(n)).collect();
        if ranks.len() < 2 || ranks.iter().all(|r| *r == 0) {
            continue;
        }
        let listed: Vec<String> = members
            .iter()
            .map(|n| format!("{n} ({})", level(n)))
            .collect();
        diags.push(diag(
            "interference-shared-process",
            Severity::Warning,
            format!(
                "container {container} runs nodes of different derived integrity in one process: \
                 {} -- a fault in the lower one can take the higher one with it. Separate them, \
                 or run the container `isolated` (this check assumes `observable` or `stock`)",
                listed.join(", ")
            ),
            format!("containers.{container}"),
        ));
    }
    index.merge_diagnostics.extend(diags);
}

/// The CPUs each node may run on, from a resolved scheduling plan: `None`
/// for a node whose placement leaves affinity alone (every CPU).
pub(crate) type NodeCpus = BTreeMap<String, Option<BTreeSet<u32>>>;

/// `independence-violated` for `cpus`, where a platform file resolved; an
/// `independence-unchecked` info where none did. Called from `check`.
pub(crate) fn check_cpus(index: &ManifestIndex, cpus: Option<&NodeCpus>) -> Vec<Diagnostic> {
    let mut diags = Vec::new();
    if !index
        .independence
        .iter()
        .any(|i| i.not_sharing.contains(&SharedFact::Cpus))
    {
        return diags;
    }
    let graph = super::manifest_graph::build_global_graph(index);
    for ind in &index.independence {
        if !ind.not_sharing.contains(&SharedFact::Cpus) {
            continue;
        }
        let at = format!("independence[{}]", ind.index);
        let where_ = site(index, ind.scope_id, &format!("independence.{}", ind.index));
        let Some(cpus) = cpus else {
            diags.push(diag(
                "independence-unchecked",
                Severity::Info,
                format!(
                    "{where_}: `cpus` is checked against the scheduling platform file's CPU sets, \
                     and none resolved -- the CPU half of this requirement is UNCHECKED (pass \
                     --sched <path>)"
                ),
                format!("{at}.not_sharing"),
            ));
            continue;
        };
        for ((a, na), (b, nb)) in pairs(index, &graph, ind) {
            let mut shared = Vec::new();
            for x in &na {
                for y in &nb {
                    let cx = cpus.get(x).cloned().flatten();
                    let cy = cpus.get(y).cloned().flatten();
                    let overlap = match (&cx, &cy) {
                        (Some(sx), Some(sy)) => !sx.is_disjoint(sy),
                        // Every CPU overlaps any set.
                        _ => true,
                    };
                    if overlap {
                        let show = |s: &Option<BTreeSet<u32>>| match s {
                            Some(s) => format!("{s:?}"),
                            None => "every CPU".to_string(),
                        };
                        shared.push(format!("{x} on {}, {y} on {}", show(&cx), show(&cy)));
                    }
                }
            }
            if !shared.is_empty() {
                diags.push(diag(
                    "independence-violated",
                    Severity::Error,
                    format!(
                        "{where_}: '{a}' and '{b}' must not share a CPU, and do: {}",
                        shared.join("; ")
                    ),
                    format!("{at}.not_sharing"),
                ));
            }
        }
    }
    diags
}

/// Each member's CPUs from a resolved task table: a `cpus` mask, else the
/// legacy `core`, else every CPU (`None`). A cpuset partition is every CPU
/// of the partition, which this table does not name, so it reads as every
/// CPU too: an overlap is reported rather than a separation assumed.
pub(crate) fn node_cpus(plan: &ros_launch_manifest_sched::ResolvedTaskTable) -> NodeCpus {
    use ros_launch_manifest_sched::PosixAffinity;
    let mut out = NodeCpus::new();
    for task in &plan.tasks {
        let set = match task.posix.as_ref().map(|p| &p.affinity) {
            Some(PosixAffinity::Cpus { cpus }) => Some(cpus.iter().copied().collect()),
            Some(PosixAffinity::Cpuset { .. }) => None,
            Some(PosixAffinity::Inherit) | None => task.core.map(|c| BTreeSet::from([c])),
        };
        for m in &task.members {
            out.insert(m.clone(), set.clone());
        }
    }
    out
}
