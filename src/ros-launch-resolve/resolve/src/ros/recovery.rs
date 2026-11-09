//! Recovery credited, and not written (rlm v0.3.13, play_launch phase 101
//! T2; rlm `docs/design/fault-tolerance.md` §4.2).
//!
//! A restart is the launch file's `respawn` and `respawn_delay`, which the
//! dump carries; the contract never states it. A node that publishes a
//! hazard's monitored topic and respawns restores the topic after its
//! detection, its `respawn_delay` and its startup. That fits the FTTI or it
//! does not:
//!
//! - fits: `recovery-credited` (info), the restart is a recovery the hazard
//!   can count on;
//! - does not: `recovery-unfit` (warning), the hazard will degrade before
//!   the restart completes;
//! - no run (`check` without `--against`), or no startup time for the node
//!   in it: `recovery-uncredited` (info). A startup time comes only from a
//!   run (`fault-tolerance.md` §6 decision 2).
//!
//! The startup time is the node's first publish in the run, else its ready
//! or loaded time, measured from its spawn (`startup_timeline.json`).

use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
};

use ros_launch_manifest_check::{Diagnostic, Severity};

use super::{
    launch_dump::LaunchDump, manifest_loader::ManifestIndex, sched_loader::fqn_for,
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

/// Node FQN to `respawn_delay` in seconds (0 when unstated), for every
/// node and container that respawns.
fn respawning(dump: &LaunchDump) -> BTreeMap<String, f64> {
    let mut out = BTreeMap::new();
    for n in &dump.node {
        if n.respawn != Some(true) {
            continue;
        }
        let bare = n
            .name
            .as_deref()
            .or(n.exec_name.as_deref())
            .unwrap_or(&n.executable);
        out.insert(
            fqn_for(dump, n.namespace.as_deref(), bare, n.scope),
            n.respawn_delay.unwrap_or(0.0),
        );
    }
    for c in &dump.container {
        if c.respawn != Some(true) {
            continue;
        }
        out.insert(
            fqn_for(dump, Some(&c.namespace), &c.name, c.scope),
            c.respawn_delay.unwrap_or(0.0),
        );
    }
    out
}

/// Member FQN to its startup time in ms, from a run's
/// `startup_timeline.json`.
fn startup_times(run: &Path) -> Option<BTreeMap<String, f64>> {
    let path = run.join("startup_timeline.json");
    let text = std::fs::read_to_string(&path).ok()?;
    let doc: serde_json::Value = serde_json::from_str(&text).ok()?;
    let mut out = BTreeMap::new();
    for row in doc["rows"].as_array()? {
        let Some(id) = row["id"].as_str() else {
            continue;
        };
        let fqn = id.split_once(':').map_or(id, |(_, f)| f).to_string();
        let Some(spawned) = row["spawned_ms"].as_f64() else {
            continue;
        };
        let up = ["first_publish_ms", "ready_ms", "loaded_ms", "active_ms"]
            .iter()
            .find_map(|k| row[*k].as_f64());
        if let Some(up) = up {
            out.insert(fqn, (up - spawned).max(0.0));
        }
    }
    Some(out)
}

/// The recovery findings for every hazard whose monitored topics a
/// respawning node publishes. `run` is `check --against`'s run directory.
pub(crate) fn check_recovery(
    index: &ManifestIndex,
    dump: &LaunchDump,
    run: Option<&Path>,
) -> Vec<Diagnostic> {
    let mut diags = Vec::new();
    let respawns = respawning(dump);
    if respawns.is_empty() {
        return diags;
    }
    let graph = super::manifest_graph::build_global_graph(index);
    let startups = run.and_then(startup_times);
    for h in &index.hazards {
        let Some(ftti) = h.decl.ftti.as_ref().and_then(|b| b.ms()) else {
            continue;
        };
        let fdti = index
            .budgets
            .iter()
            .filter(|b| b.scope_id == h.scope_id && b.hazard == h.name)
            .filter_map(|b| b.fdti_ms)
            .fold(None, |acc: Option<f64>, v| {
                Some(acc.map_or(v, |a| a.max(v)))
            });
        let publishers: BTreeSet<String> = h
            .guards
            .iter()
            .flat_map(|g| g.members.iter())
            .flat_map(|t| graph.topic_publishers.get(t).cloned().unwrap_or_default())
            .collect();
        let at = format!("hazards.{}.ftti", h.name);
        let where_ = site(index, h.scope_id, &at);
        for node in publishers {
            // A composable restarts with its container.
            let unit = index
                .executor_of
                .get(&node)
                .cloned()
                .unwrap_or(node.clone());
            let Some(delay_s) = respawns.get(&unit) else {
                continue;
            };
            let delay_ms = delay_s * 1000.0;
            let startup = startups
                .as_ref()
                .and_then(|s| s.get(&node).or_else(|| s.get(&unit)).copied());
            let (Some(startup), Some(fdti)) = (startup, fdti) else {
                let why = match (&startups, fdti) {
                    (_, None) => "the hazard's detection time is unknown".to_string(),
                    (None, _) => "a startup time comes only from a run (`check --against \
                                  <run-dir>`)"
                        .to_string(),
                    (Some(_), _) => format!("the run records no startup for {node}"),
                };
                diags.push(diag(
                    "recovery-uncredited",
                    Severity::Info,
                    format!(
                        "{where_}: hazard '{}': {unit} respawns (after {delay_ms:.0}ms), but its \
                         restart is not credited: {why}",
                        h.name
                    ),
                    at.clone(),
                ));
                continue;
            };
            let total = fdti + delay_ms + startup;
            let terms = format!(
                "detection {fdti:.2}ms + respawn_delay {delay_ms:.2}ms + startup {startup:.2}ms \
                 = {total:.2}ms"
            );
            if total <= ftti {
                diags.push(diag(
                    "recovery-credited",
                    Severity::Info,
                    format!(
                        "{where_}: hazard '{}': a restart of {unit} restores {node}'s output in \
                         {terms}, inside the FTTI {ftti:.2}ms",
                        h.name
                    ),
                    at.clone(),
                ));
            } else {
                diags.push(diag(
                    "recovery-unfit",
                    Severity::Warning,
                    format!(
                        "{where_}: hazard '{}': a restart of {unit} takes {terms}, past the FTTI \
                         {ftti:.2}ms -- the hazard degrades before the restart completes. \
                         Shorten the restart, or add a redundant member",
                        h.name
                    ),
                    at.clone(),
                ));
            }
        }
    }
    diags
}
