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
        // R1-M5 — manifest-side per-endpoint QoS lands here when the
        // loader surfaces it (P-item); None until then.
        qos: None,
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
        qos: None,
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

    // R1-M4 producer side — lower the record's string params into typed
    // ParamValues (bool/int/float sniffed, else string). The embedded
    // consumer reads params from the model (it has no record.json).
    fn lower_params(
        params: &[(String, String)],
    ) -> std::collections::BTreeMap<String, model::ParamValue> {
        params
            .iter()
            .map(|(k, v)| {
                let pv = match v.as_str() {
                    "true" => model::ParamValue::Bool(true),
                    "false" => model::ParamValue::Bool(false),
                    s => {
                        if let Ok(i) = s.parse::<i64>() {
                            model::ParamValue::Int(i)
                        } else if let Ok(f) = s.parse::<f64>() {
                            model::ParamValue::Float(f)
                        } else {
                            model::ParamValue::Str(s.to_string())
                        }
                    }
                };
                (k.clone(), pv)
            })
            .collect()
    }

    // Phase 46.3a (GAP-4) — merge scope-wide `SetParameter`/global params
    // (lower precedence) with node-specific inline `<param>` values (higher
    // precedence) BEFORE lowering into typed `ParamValue`s, matching
    // `node_cmdline.rs`'s `chain!(global, node_specific).collect()` order
    // (later entries win on key collision). `global_params` has no
    // separate model field — it's folded into `NodeInstance::params` here,
    // per that field's Phase 46.3a doc comment.
    fn lower_params_merged(
        global: Option<&[(String, String)]>,
        node_specific: &[(String, String)],
    ) -> std::collections::BTreeMap<String, model::ParamValue> {
        let combined: Vec<(String, String)> = global
            .unwrap_or(&[])
            .iter()
            .cloned()
            .chain(node_specific.iter().cloned())
            .collect();
        lower_params(&combined)
    }

    let mut insert_node =
        |structure: &mut model::Structure, node_fqn: String, inst: model::NodeInstance| {
            if structure.nodes.insert(node_fqn.clone(), inst).is_some() {
                diagnostics.push(format!("structure: duplicate node instance {node_fqn}"));
            }
        };

    // Phase 46.2 — lower the LaunchDump record's spawn fields into the
    // model's launch-input fields (`NodeInstance::{remaps,ros_args,respawn,
    // respawn_delay,env}`, Phase 46.1b). play_launch's spawn path derives
    // argv/env from these at spawn time (Phase 46.3); nano-ros ignores them.
    fn lower_remaps(remaps: &[(String, String)]) -> Vec<model::Remap> {
        remaps
            .iter()
            .map(|(from, to)| model::Remap {
                from: from.clone(),
                to: to.clone(),
            })
            .collect()
    }
    fn lower_env(env: Option<&[(String, String)]>) -> Vec<model::EnvVar> {
        env.unwrap_or(&[])
            .iter()
            .map(|(name, value)| model::EnvVar {
                name: name.clone(),
                value: value.clone(),
            })
            .collect()
    }

    // `<node machine="…">` → `execution.deploy[fqn].host` (nano-ros #236 /
    // Phase 46.1). Collected alongside `structure.nodes` so the key is the
    // SAME reconciled launch-dump FQN (`fqn(ns, name)`) other consumers
    // (bindings, sched) already use — populated into `execution` below,
    // once that layer exists. `target` is left at its `Default` (`linux`)
    // so a later `--system` config pass can fill/override it without this
    // step guessing platform placement.
    let mut deploy_hosts: BTreeMap<String, String> = BTreeMap::new();

    for n in &dump.node {
        // GAP-1 (46.3a) — a regular <node> with no declared `name` must NOT
        // be dropped from the model. Mirror the `name.or(exec_name)`
        // fallback `execution/context.rs`/`coordinator/builder.rs` already
        // use for the record-driven spawn path (CLAUDE.md 2026-03-12 note)
        // so the model's `structure.nodes` count matches the record path's
        // spawned-node count. "unknown" is the last-resort fallback only
        // when BOTH `name` and `exec_name` are absent (raw executables with
        // no launch-declared name) — same terminal fallback
        // `prepare_node_contexts` uses.
        let name = n
            .name
            .as_deref()
            .or(n.exec_name.as_deref())
            .unwrap_or("unknown");
        let ns = n.namespace.as_deref().unwrap_or("/");
        let node_fqn = fqn(ns, name);
        let decl = find_node_decl(index, dump, n.scope, name);
        if let Some(machine) = &n.machine {
            deploy_hosts.insert(node_fqn.clone(), machine.clone());
        }
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
                lifecycle_autostart: None,
                // GAP-4: global (scope-wide SetParameter) + node-specific,
                // global first so node-specific overrides on key collision.
                params: lower_params_merged(n.global_params.as_deref(), &n.params),
                criticality: decl.and_then(|d| d.criticality.clone()),
                // regular <node>: LaunchDump::NodeRecord carries all four
                // launch spawn fields.
                remaps: lower_remaps(&n.remaps),
                ros_args: n.ros_args.clone().unwrap_or_default(),
                respawn: n.respawn,
                respawn_delay: n.respawn_delay,
                env: lower_env(n.env.as_deref()),
                // GAP-2: non-ROS CLI args (prepended before --ros-args).
                args: n.args.clone().unwrap_or_default(),
                // GAP-3: resolved external param YAML content, verbatim.
                params_files: n.params_files.clone(),
                // GAP-5: <executable> tags (no package) carry their raw
                // command line here; real ROS nodes leave this empty.
                raw_cmd: if n.package.is_none() {
                    n.cmd.clone()
                } else {
                    Vec::new()
                },
                extra_args: Default::default(),
                // 46.3b: the DECLARED name (`None` when the launch had
                // `name=None`), distinct from the `name.or(exec_name)` FQN
                // key above — the spawn path emits `-r __node:=<name>` iff
                // this is `Some`, matching the record path's conditional
                // (`af7c524`); a regular `<node>` is never a container.
                node_name: n.name.clone(),
                is_container: false,
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
                lifecycle_autostart: None,
                // GAP-4: same global+node-specific merge as regular nodes —
                // NodeContainerRecord carries both fields too (a container
                // is itself a process with its own scope-inherited globals).
                // Also fixes a pre-existing bug: this previously discarded
                // `c.params` entirely (`Default::default()`), never lowering
                // a container's own inline params into the model at all.
                params: lower_params_merged(c.global_params.as_deref(), &c.params),
                criticality: None,
                // <node_container>: LaunchDump::NodeContainerRecord carries
                // the same four fields as a regular node (it spawns its own
                // process too).
                remaps: lower_remaps(&c.remaps),
                ros_args: c.ros_args.clone().unwrap_or_default(),
                respawn: c.respawn,
                respawn_delay: c.respawn_delay,
                env: lower_env(c.env.as_deref()),
                args: c.args.clone().unwrap_or_default(),
                params_files: c.params_files.clone(),
                // Containers always have a package/executable — never raw.
                raw_cmd: Vec::new(),
                extra_args: Default::default(),
                // 46.3b: `NodeContainerRecord.name` is non-optional (a
                // container always has a name); THIS is the bit the spawn
                // path keys the --container-mode override off, so a
                // consumer never has to infer container-ness by
                // reverse-resolving composable `container=` refs.
                node_name: Some(c.name.clone()),
                is_container: true,
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
                lifecycle_autostart: None,
                // Composable nodes have no `global_params` field on
                // ComposableNodeRecord (no independent process/scope-merge
                // path of their own) — inline params only, as before.
                params: lower_params(&l.params),
                criticality: decl.and_then(|d| d.criticality.clone()),
                // composable node (`load_node`): LaunchDump::ComposableNodeRecord
                // has `remaps` and `env`, but no `ros_args`/`respawn`/
                // `respawn_delay` — a composable node has no CLI process or
                // independent lifecycle of its own (it lives inside its
                // container's process).
                remaps: lower_remaps(&l.remaps),
                ros_args: Vec::new(),
                respawn: None,
                respawn_delay: None,
                env: lower_env(l.env.as_deref()),
                // No non-ROS args / params_files / raw_cmd for composables —
                // they have no independent process.
                args: Vec::new(),
                params_files: Vec::new(),
                raw_cmd: Vec::new(),
                // GAP-6: LoadNode extra arguments (⊃ use_intra_process_comms).
                extra_args: l
                    .extra_args
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect(),
                // 46.3b: `ComposableNodeRecord.node_name` is non-optional; a
                // composable is not a container (it's distinguished by
                // `plugin.is_some()` anyway).
                node_name: Some(l.node_name.clone()),
                is_container: false,
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
    for (fqn_a, a) in &index.actions {
        structure.actions.insert(
            fqn_a.clone(),
            model::ServiceWiring {
                srv_type: a.srv_type.clone(),
                server: a
                    .servers
                    .iter()
                    .map(|e| resolve_endpoint_ref(&structure.nodes, e))
                    .collect(),
                client: a
                    .clients
                    .iter()
                    .map(|e| resolve_endpoint_ref(&structure.nodes, e))
                    .collect(),
            },
        );
    }

    for (fqn_e, side) in &index.externals {
        use ros_launch_manifest_types::ExternalSide as S;
        let mapped = match side {
            S::Pub => model::ExternalSide::Pub,
            S::Sub => model::ExternalSide::Sub,
            _ => model::ExternalSide::Both,
        };
        contracts.externals.insert(fqn_e.clone(), mapped);
    }

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
            fqn(&scope_key(Some(p.scope_id)), &p.path_name),
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

        // Phase 45.4 — embed the resolved sched structure into
        // `execution.sched` (docs/design/system-model-sched-ssot.md): the
        // SAME single derivation that fed `tiers`/`bindings` above, never
        // re-derived. `chains`/`nodes`/`ranks` are `s.derived`'s own fields
        // (Phase 45.4, `sched_loader::derive_sched_plan`) — carried
        // straight through from the one `MapperInput`/`MapDiagnostics` this
        // function already built.
        //
        // FQN identity: `s.derived.nodes[].name` and `s.derived.chains`'
        // member FQNs are the SAME identities already used for `bindings`
        // above (`t.members`, sourced from `scheduled_records_from_dump`'s
        // `MapperNode::name` / `resolve_chains`'s contract-side resolution)
        // — no fourth FQN builder introduced here.
        let requirements: Vec<model::NodeSchedRequirement> = s
            .derived
            .nodes
            .iter()
            .filter(|n| n.criticality.is_some() || !n.paths.is_empty())
            .map(|n| model::NodeSchedRequirement {
                node_fqn: n.name.clone(),
                criticality: n.criticality,
                paths: n.paths.clone(),
            })
            .collect();
        let exec_sched = model::ExecutionSched {
            chains: s.derived.chains.clone(),
            requirements,
            mapper: Some(s.derived.mapper.clone()),
            ranks: s.derived.ranks.clone(),
            // Phase 45.6 (review) — the overridden-node set the fresh-derive
            // renderer keyed off, embedded verbatim so a model-sourced
            // `--explain` reproduces override(...) classification exactly.
            overrides: s.derived.overrides.clone(),
        };
        if !exec_sched.is_empty() {
            execution.sched = Some(exec_sched);
        }
    }
    // deploy: `<node machine="…">` → per-node host (nano-ros #236 /
    // Phase 46.1). Create the `Deploy` entry with `host` set and `target`
    // left at its `Default` (`linux`) — an explicit `--system` config pass
    // (`resolve.rs`, `SystemConfigToml::apply_to`) owns platform placement
    // and fills/overrides `target` afterwards. Nodes without `machine` get
    // no entry here, so `execution.deploy` stays empty for single-host
    // launches exactly as before (backward-compatible).
    for (node_fqn, host) in deploy_hosts {
        execution.deploy.entry(node_fqn).or_default().host = Some(host);
    }

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

#[cfg(test)]
mod tests {
    use super::*;

    /// Phase 46.2 — a node, a container, and a composable node each carrying
    /// remaps/env, plus respawn/ros_args on the process-having records
    /// (node, container). `resolve`'s golden/integration coverage
    /// (`tests/tests/resolve_multihost.rs`-style) exercises the full CLI
    /// pipeline; this unit test isolates `build_system_model`'s mapping
    /// logic against a hand-built `LaunchDump`, matching the
    /// `sched_loader::tests` convention (`serde_json::json!` avoids
    /// enumerating every field of these large records by hand).
    fn dump_with_launch_fields() -> LaunchDump {
        let json = serde_json::json!({
            "node": [{
                "executable": "detector_node",
                "package": "lidar_centerpoint",
                "name": "detector",
                "namespace": "/perception",
                "params_files": [],
                "cmd": [],
                "remaps": [["/points", "/sensing/lidar/points"]],
                "ros_args": ["--log-level", "detector_node:=debug"],
                "respawn": true,
                "respawn_delay": 2.5,
                "env": [["CUDA_VISIBLE_DEVICES", "0"]]
            }],
            "container": [{
                "executable": "component_container",
                "package": "rclcpp_components",
                "name": "pipeline_container",
                "namespace": "/perception",
                "params_files": [],
                "cmd": [],
                "remaps": [["/tf", "/perception/tf"]],
                "ros_args": ["--log-level", "container:=info"],
                "respawn": false,
                "env": [["OMP_NUM_THREADS", "4"]]
            }],
            "load_node": [{
                "package": "tracker_pkg",
                "plugin": "tracker::TrackerNode",
                "target_container_name": "/perception/pipeline_container",
                "node_name": "tracker",
                "namespace": "/perception",
                "remaps": [["/input", "/perception/objects"]]
            }],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null}
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    #[test]
    fn node_carries_remaps_ros_args_respawn_env() {
        let dump = dump_with_launch_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        let node = &model.structure.nodes["/perception/detector"];
        assert_eq!(
            node.remaps,
            vec![model::Remap {
                from: "/points".to_string(),
                to: "/sensing/lidar/points".to_string(),
            }]
        );
        assert_eq!(
            node.ros_args,
            vec![
                "--log-level".to_string(),
                "detector_node:=debug".to_string()
            ]
        );
        assert_eq!(node.respawn, Some(true));
        assert_eq!(node.respawn_delay, Some(2.5));
        assert_eq!(
            node.env,
            vec![model::EnvVar {
                name: "CUDA_VISIBLE_DEVICES".to_string(),
                value: "0".to_string(),
            }]
        );
    }

    #[test]
    fn container_carries_remaps_ros_args_respawn_env() {
        let dump = dump_with_launch_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        let container = &model.structure.nodes["/perception/pipeline_container"];
        assert_eq!(
            container.remaps,
            vec![model::Remap {
                from: "/tf".to_string(),
                to: "/perception/tf".to_string(),
            }]
        );
        assert_eq!(
            container.ros_args,
            vec!["--log-level".to_string(), "container:=info".to_string()]
        );
        assert_eq!(container.respawn, Some(false));
        assert_eq!(container.respawn_delay, None);
        assert_eq!(
            container.env,
            vec![model::EnvVar {
                name: "OMP_NUM_THREADS".to_string(),
                value: "4".to_string(),
            }]
        );
    }

    /// Composable nodes carry `remaps` (and would carry `env` if the parser
    /// ever populated `ComposableNodeRecord::env` — it currently doesn't)
    /// but never `ros_args`/`respawn`/`respawn_delay`: they have no CLI
    /// process or lifecycle independent of their container.
    #[test]
    fn composable_node_carries_remaps_only() {
        let dump = dump_with_launch_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        let composable = &model.structure.nodes["/perception/tracker"];
        assert_eq!(
            composable.remaps,
            vec![model::Remap {
                from: "/input".to_string(),
                to: "/perception/objects".to_string(),
            }]
        );
        assert!(composable.ros_args.is_empty());
        assert_eq!(composable.respawn, None);
        assert_eq!(composable.respawn_delay, None);
        assert!(composable.env.is_empty());
    }

    /// Phase 46.3a — a `LaunchDump` exercising all six spawn-completeness
    /// gaps found by the W3 analysis (`.superpowers/sdd/p46-w3-analysis.md`):
    /// a `name=None` node (GAP-1), external `params_files` content + scope
    /// `global_params` (GAP-3/GAP-4), non-ROS `args` (GAP-2), a raw
    /// `<executable>` (no `package`, GAP-5), and a composable node with
    /// `extra_args` (GAP-6).
    fn dump_with_gap_fields() -> LaunchDump {
        let json = serde_json::json!({
            "node": [
                {
                    "executable": "detector_node",
                    "package": "lidar_centerpoint",
                    "name": null,
                    "exec_name": "detector_node",
                    "namespace": "/perception",
                    "params": [["max_range", "50.0"]],
                    "params_files": ["/**:\n  ros__parameters:\n    voxel_size: 0.2\n"],
                    "cmd": [],
                    "args": ["--verbose", "--config", "a.yaml"],
                    "global_params": [["use_sim_time", "true"], ["max_range", "10.0"]],
                    "remaps": []
                },
                {
                    "executable": "carla-server",
                    "package": null,
                    "name": null,
                    "namespace": null,
                    "params_files": [],
                    "cmd": ["/usr/bin/carla-server -quality-level=Low", "-nosound"],
                    "remaps": []
                }
            ],
            "container": [],
            "load_node": [{
                "package": "tracker_pkg",
                "plugin": "tracker::TrackerNode",
                "target_container_name": "/perception/pipeline_container",
                "node_name": "tracker",
                "namespace": "/perception",
                "remaps": [],
                "extra_args": {"use_intra_process_comms": "True"}
            }],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null}
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    /// GAP-1 — a regular `<node>` with `name=None` must NOT vanish from
    /// `structure.nodes`; it's keyed by `exec_name` instead (mirroring the
    /// record-path `name.or(exec_name)` fallback, CLAUDE.md 2026-03-12).
    #[test]
    fn name_none_node_falls_back_to_exec_name_key() {
        let dump = dump_with_gap_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        assert!(
            model
                .structure
                .nodes
                .contains_key("/perception/detector_node"),
            "name=None node must be keyed by exec_name, got keys: {:?}",
            model.structure.nodes.keys().collect::<Vec<_>>()
        );
    }

    /// GAP-3 — external `params_files` content (already `$(var …)`-resolved
    /// by the parser) must land in `NodeInstance::params_files` verbatim,
    /// not be dropped.
    #[test]
    fn params_files_content_lands_verbatim() {
        let dump = dump_with_gap_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        let node = &model.structure.nodes["/perception/detector_node"];
        assert_eq!(
            node.params_files,
            vec!["/**:\n  ros__parameters:\n    voxel_size: 0.2\n".to_string()]
        );
    }

    /// GAP-4 — scope-wide `global_params` are merged into `params` with
    /// node-specific values taking precedence on key collision
    /// (`max_range`: global says 10.0, node-specific says 50.0 — 50.0 wins,
    /// matching `node_cmdline.rs`'s `chain!(global, node_specific)` order),
    /// while a global-only key (`use_sim_time`) still comes through.
    #[test]
    fn global_params_merge_with_node_specific_precedence() {
        let dump = dump_with_gap_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        let node = &model.structure.nodes["/perception/detector_node"];
        assert_eq!(
            node.params.get("max_range"),
            Some(&model::ParamValue::Float(50.0)),
            "node-specific param must override global on collision"
        );
        assert_eq!(
            node.params.get("use_sim_time"),
            Some(&model::ParamValue::Bool(true)),
            "global-only param must still be present"
        );
    }

    /// GAP-2 — non-ROS CLI `args` (distinct from `ros_args`) must be
    /// carried through.
    #[test]
    fn non_ros_args_are_carried() {
        let dump = dump_with_gap_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        let node = &model.structure.nodes["/perception/detector_node"];
        assert_eq!(
            node.args,
            vec![
                "--verbose".to_string(),
                "--config".to_string(),
                "a.yaml".to_string(),
            ]
        );
    }

    /// GAP-5 — a raw `<executable>` record (`package: None`) must be
    /// represented in the model via `raw_cmd`, keyed by "unknown" (no
    /// `name`/`exec_name` at all — the terminal fallback), not silently
    /// dropped and not treated as a ROS node.
    #[test]
    fn raw_executable_carries_raw_cmd() {
        let dump = dump_with_gap_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        let node = &model.structure.nodes["/unknown"];
        assert!(node.pkg.is_none());
        assert_eq!(
            node.raw_cmd,
            vec![
                "/usr/bin/carla-server -quality-level=Low".to_string(),
                "-nosound".to_string(),
            ]
        );
    }

    /// GAP-6 — composable-node `extra_args` (⊃ `use_intra_process_comms`)
    /// must be carried through to the model.
    #[test]
    fn composable_extra_args_are_carried() {
        let dump = dump_with_gap_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        let composable = &model.structure.nodes["/perception/tracker"];
        assert_eq!(
            composable.extra_args.get("use_intra_process_comms"),
            Some(&"True".to_string())
        );
    }

    /// 46.3b — `node_name` carries the DECLARED name distinctly, `None`
    /// for a `name=None` node (so the spawn path can withhold `__node`),
    /// `Some` for a declared node, container, and composable.
    #[test]
    fn node_name_carries_declared_name_none_when_unnamed() {
        let dump = dump_with_gap_fields();
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        // name=None regular node → node_name None.
        assert_eq!(
            model.structure.nodes["/perception/detector_node"].node_name,
            None
        );
        // composable → node_name Some (ComposableNodeRecord.node_name is
        // non-optional).
        assert_eq!(
            model.structure.nodes["/perception/tracker"]
                .node_name
                .as_deref(),
            Some("tracker")
        );
    }

    /// 46.3b — the classification bit is populated from the record's actual
    /// node kind, NOT inferred by reverse-resolving composable `container=`
    /// references. A regular node whose name collides with a DANGLING
    /// composable target (no such container exists) must NOT be classified
    /// as a container — the pre-46.3b `resolve_node_ref` bare-name-suffix
    /// heuristic would have misclassified it and then corrupted its
    /// executable via the `--container-mode` override.
    #[test]
    fn regular_node_colliding_dangling_composable_target_is_not_container() {
        let json = serde_json::json!({
            "node": [{
                "executable": "widget_node",
                "package": "widget_pkg",
                "name": "widget",
                "exec_name": "widget_node",
                "namespace": "/tools",
                "params_files": [],
                "cmd": [],
                "remaps": []
            }],
            "container": [{
                "executable": "component_container",
                "package": "rclcpp_components",
                "name": "real_container",
                "namespace": "/perception",
                "params_files": [],
                "cmd": [],
                "remaps": []
            }],
            "load_node": [{
                "package": "some_pkg",
                "plugin": "some::Plugin",
                // Dangling/ambiguous relative target that bare-name-matches
                // the REGULAR node `widget`, not the real container.
                "target_container_name": "widget",
                "node_name": "some_node",
                "namespace": "/perception",
                "remaps": []
            }],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [{"id": 0, "ns": "/", "parent": null}]
        });
        let dump: LaunchDump = serde_json::from_value(json).expect("valid LaunchDump");
        let index = ManifestIndex::default();
        let model = build_system_model(&dump, &index, None, BTreeMap::new(), &BTreeSet::new());

        // The regular node is NOT a container despite the dangling
        // composable target colliding on its bare name.
        assert!(
            !model.structure.nodes["/tools/widget"].is_container,
            "regular node must not be misclassified as a container"
        );
        // The genuine container IS flagged.
        assert!(model.structure.nodes["/perception/real_container"].is_container);
        // The composable is not a container.
        assert!(!model.structure.nodes["/perception/some_node"].is_container);
    }
}
