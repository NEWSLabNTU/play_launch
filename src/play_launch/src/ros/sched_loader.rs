//! Load + validate the shared scheduling spec against a parsed launch dump.
//!
//! Linux is "validate now, apply later": we resolve for the `posix` target and
//! report, but do not (yet) apply `sched_setscheduler`/affinity to processes.

use std::collections::HashMap;
use std::path::Path;

use eyre::{Result, WrapErr};
use ros_launch_manifest_sched::{ResolvedTierTable, SchedNode, parse_system_sched, resolve};

use crate::ros::launch_dump::LaunchDump;

/// The `posix` target key — Linux RT placement sub-table.
const TARGET: &str = "posix";

/// Resolve a scope id to its namespace path (falls back to `/`).
fn scope_ns(dump: &LaunchDump, scope: Option<usize>) -> String {
    let by_id: HashMap<usize, &str> =
        dump.scopes.iter().map(|s| (s.id, s.ns.as_str())).collect();
    scope
        .and_then(|id| by_id.get(&id).copied())
        .unwrap_or("/")
        .to_string()
}

/// Join a namespace with a bare node name into a fully-qualified name.
fn join_fqn(ns: &str, bare: &str) -> String {
    if bare.starts_with('/') {
        bare.to_string()
    } else if ns == "/" || ns.is_empty() {
        format!("/{bare}")
    } else {
        format!("{}/{bare}", ns.trim_end_matches('/'))
    }
}

/// Flatten a launch dump into the resolver's dependency-free node view.
/// Regular nodes and containers are both scheduled units on Linux.
pub fn sched_nodes_from_dump(dump: &LaunchDump) -> Vec<SchedNode> {
    let mut out = Vec::new();
    for n in &dump.node {
        let bare = n
            .name
            .clone()
            .or_else(|| n.exec_name.clone())
            .unwrap_or_default();
        if bare.is_empty() {
            continue;
        }
        let ns = scope_ns(dump, n.scope);
        out.push(SchedNode {
            name: join_fqn(&ns, &bare),
            scope: ns,
        });
    }
    for c in &dump.container {
        let ns = scope_ns(dump, c.scope);
        out.push(SchedNode {
            name: join_fqn(&ns, &c.name),
            scope: ns,
        });
    }
    out
}

/// Load the scheduling TOML, resolve for `posix`, and print the resolved table.
pub fn check_sched(dump: &LaunchDump, sched_path: &Path) -> Result<()> {
    let text = std::fs::read_to_string(sched_path)
        .wrap_err_with(|| format!("failed to read {}", sched_path.display()))?;
    let sched = parse_system_sched(&text)
        .map_err(|e| eyre::eyre!("scheduling spec error: {e}"))?;
    let nodes = sched_nodes_from_dump(dump);
    let table: ResolvedTierTable = resolve(&sched.tiers, &sched.assign, &nodes, TARGET)
        .map_err(|e| eyre::eyre!("scheduling resolve error: {e}"))?;

    eprintln!("Scheduling ({TARGET}): {} tier(s)", table.tiers.len());
    for t in &table.tiers {
        eprintln!(
            "  tier {:<16} prio={:<4} sched_class={:<10} core={:<4} members={}",
            t.name,
            t.priority,
            t.sched_class.as_deref().unwrap_or("-"),
            t.core.map(|c| c.to_string()).unwrap_or_else(|| "-".into()),
            t.members.len(),
        );
        for m in &t.members {
            eprintln!("      {m}");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::launch_dump::{NodeRecord, ScopeEntry};

    // Minimal NodeRecord/ScopeEntry construction: use serde_json to avoid
    // enumerating every field of these large records by hand.
    // NodeRecord requires: executable (String), params_files (Vec), cmd (Vec).
    fn dump_with_one_node() -> LaunchDump {
        let json = serde_json::json!({
            "node": [{
                "executable": "ndt_localizer",
                "name": "ndt_localizer",
                "exec_name": "ndt_localizer",
                "params_files": [],
                "cmd": [],
                "scope": 1
            }],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null},
                {"id": 1, "ns": "/localization", "parent": 0}
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    #[test]
    fn maps_node_scope_to_fqn() {
        let dump = dump_with_one_node();
        let nodes = sched_nodes_from_dump(&dump);
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].name, "/localization/ndt_localizer");
        assert_eq!(nodes[0].scope, "/localization");
        // Silence unused-import lint in case NodeRecord/ScopeEntry drift.
        let _ = (std::any::type_name::<NodeRecord>(), std::any::type_name::<ScopeEntry>());
    }
}
