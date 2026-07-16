//! Build a `SystemModel` (ros-launch-manifest-model) from the resolved
//! pipeline outputs: `LaunchDump` (structure), `ManifestIndex` (contracts),
//! and the derived scheduling plan (execution).
//!
//! This is the mapping half of `play_launch resolve` (RFC-0050 /
//! docs/design/system-model.md). The model is early-bound: everything here
//! is post arg-binding, post condition-filtering, post cross-scope merge —
//! no variables or conditions survive into the artifact.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};

use ros_launch_manifest_model as model;
use ros_launch_manifest_sched::{DEFAULT_TIER, TierDef, TierPlatformSpec};
use ros_launch_manifest_types::{DropSpec, EndpointProps, QosDecl};
use sha2::{Digest, Sha256};

use crate::ros::{
    launch_dump::LaunchDump, manifest_loader::ManifestIndex, sched_loader::DerivedSchedPlan,
};

/// Scheduling inputs for the execution layer.
pub struct SchedInputs<'a> {
    /// The derived, override-applied, flattened plan.
    pub derived: &'a DerivedSchedPlan,
    /// Declared tier table when the platform file carried one (legacy
    /// `system.toml` bridge); `None` for mapper-derived plans, where tier
    /// definitions are synthesized from the resolved table.
    pub declared_tiers: Option<BTreeMap<String, TierDef>>,
}

/// Join a namespace and a name into an FQN (`/ns/name`, root-safe).
fn fqn(ns: &str, name: &str) -> String {
    let ns = ns.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{name}")
    } else if ns.starts_with('/') {
        format!("{ns}/{name}")
    } else {
        format!("/{ns}/{name}")
    }
}

/// Deterministic, unique scope keys: the scope's namespace, disambiguated
/// with `#<id>` when several scopes share one namespace.
fn scope_keys(dump: &LaunchDump) -> BTreeMap<usize, String> {
    let mut count: BTreeMap<&str, usize> = BTreeMap::new();
    for s in &dump.scopes {
        *count.entry(s.ns.as_str()).or_default() += 1;
    }
    let mut keys = BTreeMap::new();
    for s in &dump.scopes {
        let base = if s.ns.is_empty() { "/" } else { s.ns.as_str() };
        let key = if count.get(s.ns.as_str()).copied().unwrap_or(0) > 1 {
            format!("{base}#{}", s.id)
        } else {
            base.to_string()
        };
        keys.insert(s.id, key);
    }
    keys
}

/// Walk the scope parent chain from `scope_id` and return the first scope
/// that has a loaded manifest containing a node declaration named `name`.
fn find_node_decl<'a>(
    index: &'a ManifestIndex,
    dump: &LaunchDump,
    scope_id: Option<usize>,
    name: &str,
) -> Option<&'a ros_launch_manifest_types::NodeDecl> {
    let mut cur = scope_id;
    while let Some(id) = cur {
        if let Some(m) = index.manifests.get(&id)
            && let Some(decl) = m.manifest.nodes.get(name)
        {
            return Some(decl);
        }
        cur = dump
            .scopes
            .iter()
            .find(|s| s.id == id)
            .and_then(|s| s.parent);
    }
    None
}

/// Reconcile a manifest-side node reference (scope-ns-qualified manifest
/// node name, e.g. `/control_node`) with the launch-side node FQNs in
/// `structure.nodes` (e.g. `/control/control_node`). Exact hit wins; else a
/// UNIQUE bare-name (last path segment) match — the same vocabulary the
/// sched crate's assign/override selectors use; else the ref is kept as-is.
fn resolve_node_ref(nodes: &BTreeMap<String, model::NodeInstance>, node_ref: &str) -> String {
    if nodes.contains_key(node_ref) {
        return node_ref.to_string();
    }
    let bare = node_ref.rsplit('/').next().unwrap_or(node_ref);
    let mut hits = nodes.keys().filter(|k| k.rsplit('/').next() == Some(bare));
    match (hits.next(), hits.next()) {
        (Some(k), None) => k.clone(),
        _ => node_ref.to_string(),
    }
}

/// Reconcile an endpoint ref (`<node ref>/<endpoint>`).
fn resolve_endpoint_ref(nodes: &BTreeMap<String, model::NodeInstance>, ep_ref: &str) -> String {
    match ep_ref.rsplit_once('/') {
        Some((node_ref, ep)) => format!("{}/{ep}", resolve_node_ref(nodes, node_ref)),
        None => ep_ref.to_string(),
    }
}

fn pub_contract(p: &EndpointProps) -> Option<model::PubContract> {
    if p.min_rate_hz.is_none() && p.max_rate_hz.is_none() && p.jitter_ms.is_none() {
        return None;
    }
    Some(model::PubContract {
        min_rate_hz: p.min_rate_hz,
        max_rate_hz: p.max_rate_hz,
        jitter_ms: p.jitter_ms,
    })
}

fn sub_contract(p: &EndpointProps) -> Option<model::SubContract> {
    let state = p.state.unwrap_or(false);
    let required = p.required.unwrap_or(false);
    if p.min_rate_hz.is_none()
        && p.max_rate_hz.is_none()
        && p.max_age_ms.is_none()
        && !state
        && !required
    {
        return None;
    }
    Some(model::SubContract {
        min_rate_hz: p.min_rate_hz,
        max_rate_hz: p.max_rate_hz,
        max_age_ms: p.max_age_ms,
        state,
        required,
    })
}

fn drop_contract(d: &DropSpec) -> model::DropContract {
    model::DropContract {
        max_drop_rate: d.max_count.as_ref().map(|c| c.drop_rate()),
        max_consecutive: d.max_consecutive,
    }
}

fn qos_contract(q: &QosDecl) -> model::Qos {
    model::Qos {
        reliability: q.reliability.clone(),
        durability: q.durability.clone(),
        history: q.history.clone(),
        depth: q.depth,
        lifespan_ms: q.lifespan_ms,
        liveliness: q.liveliness.clone(),
    }
}

fn correlation(c: Option<&str>) -> Option<model::Correlation> {
    match c {
        Some("timestamp") => Some(model::Correlation::Timestamp),
        Some("latest") => Some(model::Correlation::Latest),
        _ => None,
    }
}

fn path_contract(
    decl: &ros_launch_manifest_types::PathDecl,
    input: Vec<String>,
    output: Vec<String>,
) -> model::PathContract {
    model::PathContract {
        input,
        output,
        max_latency_ms: decl.max_latency_ms,
        correlation: correlation(decl.correlation.as_deref()),
        tolerance_ms: decl.tolerance_ms,
        drop: decl.drop.as_ref().map(drop_contract),
    }
}

/// sha256 the given files (skipping unreadable ones with a diagnostic).
fn hash_inputs(paths: &BTreeSet<PathBuf>, diagnostics: &mut Vec<String>) -> Vec<model::InputHash> {
    let mut out = Vec::new();
    for p in paths {
        match std::fs::read(p) {
            Ok(bytes) => {
                let digest = Sha256::digest(&bytes);
                let canon = std::fs::canonicalize(p).unwrap_or_else(|_| p.clone());
                out.push(model::InputHash {
                    path: canon.display().to_string(),
                    sha256: format!("{digest:x}"),
                });
            }
            Err(e) => diagnostics.push(format!(
                "provenance: could not hash input {}: {e}",
                p.display()
            )),
        }
    }
    out
}

/// Synthesize a `TierDef` table from the resolved plan when the platform
/// file declared none (mapper-derived plans): one entry per distinct
/// non-default tier name, with the placement recorded under the resolve
/// target's platform sub-table.
fn synthesize_tiers(derived: &DerivedSchedPlan) -> BTreeMap<String, TierDef> {
    let mut tiers = BTreeMap::new();
    for t in &derived.plan.tiers {
        if t.name == DEFAULT_TIER || tiers.contains_key(&t.name) {
            continue;
        }
        let spec = TierPlatformSpec {
            priority: t.priority,
            stack_bytes: t.stack_bytes,
            core: t.core,
            sched_class: t.sched_class.clone(),
            preempt_threshold: t.preempt_threshold,
            deadline_us: None,
        };
        let mut def = TierDef {
            class: t.class.clone(),
            deadline_us: t.deadline_us,
            period_us: t.period_us,
            budget_us: t.budget_us,
            deadline_policy: t.deadline_policy.clone(),
            spin_period_us: t.spin_period_us,
            ..Default::default()
        };
        match derived.target.as_str() {
            "posix" | "native" => def.posix = Some(spec),
            "freertos" => def.freertos = Some(spec),
            "zephyr" => def.zephyr = Some(spec),
            "threadx" => def.threadx = Some(spec),
            "nuttx" => def.nuttx = Some(spec),
            _ => {}
        }
        tiers.insert(t.name.clone(), def);
    }
    tiers
}

/// Map the pipeline outputs into a `SystemModel`.
///
/// `args` is the concrete launch-argument binding; `input_paths` is every
/// file that fed the resolution (launch files, contract files, platform
/// file) — hashed into `meta.inputs` as the provenance/cache key.
pub fn build_system_model(
    dump: &LaunchDump,
    index: &ManifestIndex,
    sched: Option<&SchedInputs>,
    args: BTreeMap<String, String>,
    input_paths: &BTreeSet<PathBuf>,
) -> model::SystemModel {
    let keys = scope_keys(dump);
    let scope_key = |id: Option<usize>| -> String {
        id.and_then(|i| keys.get(&i).cloned())
            .unwrap_or_else(|| "/".to_string())
    };

    let mut diagnostics: Vec<String> = Vec::new();

    // --- structure -------------------------------------------------------
    let mut structure = model::Structure::default();

    for s in &dump.scopes {
        structure.scopes.insert(
            keys[&s.id].clone(),
            model::ScopeInfo {
                parent: s.parent.map(|p| keys[&p].clone()),
                manifest: index
                    .manifests
                    .get(&s.id)
                    .map(|m| m.contract_path.display().to_string()),
            },
        );
    }

    let mut contracts = model::Contracts::default();

    let mut insert_node =
        |structure: &mut model::Structure, node_fqn: String, inst: model::NodeInstance| {
            if structure.nodes.insert(node_fqn.clone(), inst).is_some() {
                diagnostics.push(format!("structure: duplicate node instance {node_fqn}"));
            }
        };

    for n in &dump.node {
        let Some(name) = n.name.as_deref() else {
            continue;
        };
        let ns = n.namespace.as_deref().unwrap_or("/");
        let node_fqn = fqn(ns, name);
        let decl = find_node_decl(index, dump, n.scope, name);
        insert_node(
            &mut structure,
            node_fqn,
            model::NodeInstance {
                scope: scope_key(n.scope),
                pkg: n.package.clone(),
                exec: Some(n.executable.clone()),
                plugin: None,
                container: None,
                lifecycle: decl.and_then(|d| d.lifecycle).unwrap_or(false),
                criticality: decl.and_then(|d| d.criticality.clone()),
            },
        );
    }
    for c in &dump.container {
        let node_fqn = fqn(&c.namespace, &c.name);
        insert_node(
            &mut structure,
            node_fqn,
            model::NodeInstance {
                scope: scope_key(c.scope),
                pkg: Some(c.package.clone()),
                exec: Some(c.executable.clone()),
                plugin: None,
                container: None,
                lifecycle: false,
                criticality: None,
            },
        );
    }
    for l in &dump.load_node {
        let node_fqn = fqn(&l.namespace, &l.node_name);
        let decl = find_node_decl(index, dump, l.scope, &l.node_name);
        insert_node(
            &mut structure,
            node_fqn,
            model::NodeInstance {
                scope: scope_key(l.scope),
                pkg: Some(l.package.clone()),
                exec: None,
                plugin: Some(l.plugin.clone()),
                container: Some(l.target_container_name.clone()),
                lifecycle: decl.and_then(|d| d.lifecycle).unwrap_or(false),
                criticality: decl.and_then(|d| d.criticality.clone()),
            },
        );
    }

    for (fqn_t, t) in &index.topics {
        structure.topics.insert(
            fqn_t.clone(),
            model::TopicWiring {
                msg_type: t.msg_type.clone(),
                publishers: t
                    .publishers
                    .iter()
                    .map(|e| resolve_endpoint_ref(&structure.nodes, e))
                    .collect(),
                subscribers: t
                    .subscribers
                    .iter()
                    .map(|e| resolve_endpoint_ref(&structure.nodes, e))
                    .collect(),
            },
        );
        let tc = model::TopicContract {
            rate_hz: t.rate_hz,
            max_transport_ms: t.max_transport_ms,
            drop: t.drop.as_ref().map(drop_contract),
            qos: t.qos.as_ref().map(qos_contract),
        };
        if tc != model::TopicContract::default() {
            contracts.topics.insert(fqn_t.clone(), tc);
        }
    }
    for (fqn_s, s) in &index.services {
        structure.services.insert(
            fqn_s.clone(),
            model::ServiceWiring {
                srv_type: s.srv_type.clone(),
                server: s
                    .servers
                    .iter()
                    .map(|e| resolve_endpoint_ref(&structure.nodes, e))
                    .collect(),
                client: s
                    .clients
                    .iter()
                    .map(|e| resolve_endpoint_ref(&structure.nodes, e))
                    .collect(),
            },
        );
    }
    // NOTE: manifest `actions:` are not merged into ManifestIndex yet;
    // structure.actions stays empty until the loader grows that pass.

    // --- contracts: endpoints (from each scope's node declarations) -------
    for m in index.manifests.values() {
        for (name, decl) in &m.manifest.nodes {
            let node_fqn = resolve_node_ref(&structure.nodes, &fqn(&m.ns, name));
            for (ep, props) in &decl.publishers {
                if let Some(c) = pub_contract(props) {
                    contracts
                        .pub_endpoints
                        .insert(format!("{node_fqn}/{ep}"), c);
                }
            }
            for (ep, props) in &decl.subscribers {
                if let Some(c) = sub_contract(props) {
                    contracts
                        .sub_endpoints
                        .insert(format!("{node_fqn}/{ep}"), c);
                }
            }
            for (ep, props) in &decl.srv {
                if let Some(ms) = props.max_response_ms {
                    contracts.srv_endpoints.insert(
                        format!("{node_fqn}/{ep}"),
                        model::SrvContract {
                            max_response_ms: Some(ms),
                        },
                    );
                }
            }
        }
    }

    for p in &index.node_paths {
        let node_fqn = resolve_node_ref(&structure.nodes, &p.node_fqn);
        let input = p
            .path
            .input
            .iter()
            .map(|e| format!("{node_fqn}/{e}"))
            .collect();
        let output = p
            .path
            .output
            .iter()
            .map(|e| format!("{node_fqn}/{e}"))
            .collect();
        contracts.node_paths.insert(
            format!("{node_fqn}/{}", p.path_name),
            path_contract(&p.path, input, output),
        );
    }
    for p in &index.scope_paths {
        contracts.scope_paths.insert(
            format!("{}/{}", scope_key(Some(p.scope_id)), p.path_name),
            path_contract(&p.path, p.input_topics.clone(), p.output_topics.clone()),
        );
    }

    // --- execution ---------------------------------------------------------
    let mut execution = model::Execution::default();
    if let Some(s) = sched {
        execution.tiers = s
            .declared_tiers
            .clone()
            .unwrap_or_else(|| synthesize_tiers(s.derived));
        for t in &s.derived.plan.tiers {
            if t.name == DEFAULT_TIER {
                continue;
            }
            for m in &t.members {
                execution.bindings.insert(m.clone(), t.name.clone());
            }
        }
        diagnostics.extend(s.derived.warnings.iter().cloned());
    }
    // deploy: integrator-owned placement lands here once the system config
    // grows a [deploy] section (RFC-0050 open question); empty = all-linux.

    // --- embedded checker warnings -----------------------------------------
    for m in index.manifests.values() {
        for d in &m.diagnostics {
            if !matches!(d.severity, ros_launch_manifest_check::Severity::Error) {
                diagnostics.push(format!("{}: {d}", m.file));
            }
        }
    }
    for d in &index.merge_diagnostics {
        if !matches!(d.severity, ros_launch_manifest_check::Severity::Error) {
            diagnostics.push(d.to_string());
        }
    }

    let inputs = hash_inputs(input_paths, &mut diagnostics);

    model::SystemModel {
        meta: model::Meta {
            version: model::SCHEMA_VERSION,
            args,
            inputs,
            resolver: Some(model::ResolverInfo {
                tool: "play_launch".to_string(),
                version: env!("CARGO_PKG_VERSION").to_string(),
            }),
            diagnostics,
            record: None,
        },
        structure,
        contracts,
        execution,
    }
}
