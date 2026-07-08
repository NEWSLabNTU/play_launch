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

/// Effective namespace: the record's own namespace if non-empty, else the scope's ns.
fn effective_ns(own: Option<&str>, scope: Option<usize>, by_id: &HashMap<usize, &str>) -> String {
    let own = own.map(str::trim).filter(|s| !s.is_empty() && *s != "/");
    let ns = own
        .or_else(|| scope.and_then(|id| by_id.get(&id).copied()))
        .unwrap_or("/");
    // normalize to a single leading slash, no trailing slash
    let t = ns.trim_matches('/');
    if t.is_empty() { "/".to_string() } else { format!("/{t}") }
}

/// Join a namespace with a bare node name into a fully-qualified name.
pub(crate) fn join_fqn(ns: &str, bare: &str) -> String {
    if bare.starts_with('/') {
        bare.to_string()
    } else if ns == "/" || ns.is_empty() {
        format!("/{bare}")
    } else {
        format!("{}/{bare}", ns.trim_end_matches('/'))
    }
}

/// Flatten a launch dump into the resolver's dependency-free node view.
/// Regular nodes, containers, and composable nodes are all scheduled units on Linux:
/// under isolated container mode each composable is a fork+exec process.
pub fn sched_nodes_from_dump(dump: &LaunchDump) -> Vec<SchedNode> {
    // Build scope-id → namespace map once; avoids O(N*M) per-record lookups.
    let by_id: HashMap<usize, &str> =
        dump.scopes.iter().map(|s| (s.id, s.ns.as_str())).collect();

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
        let ns = effective_ns(n.namespace.as_deref(), n.scope, &by_id);
        out.push(SchedNode {
            name: join_fqn(&ns, &bare),
            scope: ns,
        });
    }

    for c in &dump.container {
        if c.name.is_empty() {
            continue;
        }
        let ns = effective_ns(Some(c.namespace.as_str()), c.scope, &by_id);
        out.push(SchedNode {
            name: join_fqn(&ns, &c.name),
            scope: ns,
        });
    }

    for lc in &dump.load_node {
        if lc.node_name.is_empty() {
            continue;
        }
        // Under isolated container mode each composable is a fork+exec process,
        // so it is a schedulable unit and must be selectable.
        let ns = effective_ns(Some(lc.namespace.as_str()), lc.scope, &by_id);
        out.push(SchedNode {
            name: join_fqn(&ns, &lc.node_name),
            scope: ns,
        });
    }

    out
}

/// FQNs of composable (load_node) records — the units v1 cannot schedule.
///
/// Not yet called outside tests + `sched_plan` (actor wiring is a later
/// phase-38 task); allowed dead in non-consumer builds.
#[allow(dead_code)]
pub fn composable_fqns(dump: &LaunchDump) -> std::collections::HashSet<String> {
    let by_id: HashMap<usize, &str> =
        dump.scopes.iter().map(|s| (s.id, s.ns.as_str())).collect();

    dump.load_node
        .iter()
        .filter(|lc| !lc.node_name.is_empty())
        .map(|lc| {
            let ns = effective_ns(Some(lc.namespace.as_str()), lc.scope, &by_id);
            join_fqn(&ns, &lc.node_name)
        })
        .collect()
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
    use crate::ros::launch_dump::{ComposableNodeRecord, NodeRecord, ScopeEntry};

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

    #[test]
    fn includes_composable_nodes_and_uses_node_namespace() {
        // Node with an explicit namespace "/explicit" in scope 0 (scope ns = "/"):
        // effective_ns must prefer the node's own namespace over scope ns.
        // load_node entry with namespace "/composed": must appear in result.
        let json = serde_json::json!({
            "node": [{
                "executable": "some_exec",
                "name": "ndt_localizer",
                "exec_name": "ndt_localizer",
                "namespace": "/explicit",
                "params_files": [],
                "cmd": [],
                "scope": 0
            }],
            "load_node": [{
                "package": "my_pkg",
                "plugin": "my_pkg::MyNode",
                "target_container_name": "my_container",
                "node_name": "my_node",
                "namespace": "/composed",
                "scope": 0
            }],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null}
            ]
        });
        let dump: LaunchDump = serde_json::from_value(json).expect("valid LaunchDump");
        let nodes = sched_nodes_from_dump(&dump);

        assert_eq!(nodes.len(), 2, "expected node + composable");

        // Node with explicit namespace "/explicit" (scope ns is "/") → own ns wins.
        let n = nodes.iter().find(|n| n.name.contains("ndt_localizer")).unwrap();
        assert_eq!(n.name, "/explicit/ndt_localizer");
        assert_eq!(n.scope, "/explicit");

        // Composable node with namespace "/composed".
        let c = nodes.iter().find(|n| n.name.contains("my_node")).unwrap();
        assert_eq!(c.name, "/composed/my_node");
        assert_eq!(c.scope, "/composed");

        // Silence unused-import lint.
        let _ = std::any::type_name::<ComposableNodeRecord>();
    }

    #[test]
    fn composable_fqns_includes_load_node_excludes_regular_node() {
        let json = serde_json::json!({
            "node": [{
                "executable": "some_exec",
                "name": "ndt_localizer",
                "exec_name": "ndt_localizer",
                "namespace": "/explicit",
                "params_files": [],
                "cmd": [],
                "scope": 0
            }],
            "load_node": [{
                "package": "my_pkg",
                "plugin": "my_pkg::MyNode",
                "target_container_name": "my_container",
                "node_name": "my_node",
                "namespace": "/composed",
                "scope": 0
            }],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null}
            ]
        });
        let dump: LaunchDump = serde_json::from_value(json).expect("valid LaunchDump");
        let composables = composable_fqns(&dump);

        assert!(composables.contains("/composed/my_node"));
        assert!(!composables.contains("/explicit/ndt_localizer"));
        assert_eq!(composables.len(), 1);
    }
}
