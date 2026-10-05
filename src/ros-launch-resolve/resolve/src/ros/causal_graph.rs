//! Declared causal graph export (`ros-launch-resolve check --export-graph`).
//!
//! Phase 42.1 (`docs/design/autoware-system-model-study.md`, Q1). This is an
//! **export**, not a validation step: it walks the already-loaded
//! `ManifestIndex` (same data the `check` rules consult) and serializes the
//! declared graph — ROS nodes, topics, pub/sub wiring (tagged causal / state
//! / required), node-level and scope-level `paths:`, and a cycle catalogue
//! computed BEFORE `state:` cuts are applied — so downstream tooling (Phase
//! 42 W2's measured-model join, W4's report) can consume a stable JSON
//! shape without re-deriving it from the manifests.
//!
//! Reuses `manifest_graph::build_global_graph()` (Phase 35) for the
//! node-to-node edge graph (already carries `is_state` per edge, exactly the
//! "before cuts" graph we need for cycle detection) — no changes to the
//! `ros-launch-manifest` check crate are required.
//!
//! JSON schema is documented in `docs/design/causal-graph-export.md`.

use super::{
    manifest_graph::{self, GlobalDataflowGraph},
    manifest_loader::ManifestIndex,
};
use eyre::Result;
use serde::Serialize;
use std::{
    collections::{HashMap, HashSet},
    path::Path,
};

/// Schema version. Bump on breaking JSON shape changes; additive fields
/// (new optional keys) don't require a bump.
///
/// 2 (phase 85 I6): a node path's `input` is its EFFECTIVE trigger's inputs
/// (version 1 copied the legacy `input:` list, so every `trigger: { input:
/// [...] }` path read `input: []`, the same as a timer), and the export
/// carries everything the fault walk reads: `trigger`, `safe_state`,
/// services, externals, `on_violation`, hazards, functions and modes.
const SCHEMA_VERSION: u32 = 2;

/// Top-level export payload.
#[derive(Debug, Serialize)]
pub struct GraphExport {
    pub version: u32,
    pub nodes: Vec<NodeOut>,
    pub topics: Vec<TopicOut>,
    pub pub_edges: Vec<PubEdgeOut>,
    pub sub_edges: Vec<SubEdgeOut>,
    pub node_paths: Vec<NodePathOut>,
    pub scope_paths: Vec<ScopePathOut>,
    pub cycles: Vec<CycleOut>,
    /// Services with their server and client endpoints (version 2).
    pub services: Vec<ServiceOut>,
    /// Hazards: guards, the fault classes claimed, the interval and the
    /// reaction (a scope path or a mode) (version 2).
    pub hazards: Vec<HazardOut>,
    /// Named guard groups (version 2).
    pub functions: Vec<FunctionOut>,
    /// Operational modes: what each requires, its rung's reaction, and the
    /// ladder below it (`fallback`) (version 2).
    pub modes: Vec<ModeOut>,
}

/// One endpoint of a service: the node and its local endpoint name.
#[derive(Debug, Serialize)]
pub struct EndpointOut {
    pub node: String,
    pub endpoint: String,
}

/// A service vertex (version 2): client -> service -> server.
#[derive(Debug, Serialize)]
pub struct ServiceOut {
    pub fqn: String,
    #[serde(rename = "type")]
    pub srv_type: String,
    pub servers: Vec<EndpointOut>,
    pub clients: Vec<EndpointOut>,
    /// `server` | `client` | `both`: the side provided outside the tree.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub external: Option<&'static str>,
}

/// A guard group: a bare list is any-of (losing any member is the fault),
/// `all_of` a redundant set (lost only when every member is).
#[derive(Debug, Serialize)]
pub struct GuardOut {
    pub members: Vec<String>,
    pub all_of: bool,
}

/// A hazard (version 2).
#[derive(Debug, Serialize)]
pub struct HazardOut {
    pub name: String,
    pub scope_id: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub severity: Option<String>,
    /// Resolved guard topic FQNs, one entry per group.
    pub guards: Vec<GuardOut>,
    /// The fault classes claimed (`omission`, `late`, `loss`, `reported`);
    /// empty when the contract names none.
    pub on: Vec<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ftti_ms: Option<f64>,
    /// A scope path name, or a mode whose `fallback` is the ladder.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reaction: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub entry_speed_mps: Option<f64>,
}

/// A named guard group (version 2).
#[derive(Debug, Serialize)]
pub struct FunctionOut {
    pub name: String,
    pub scope_id: usize,
    pub members: Vec<String>,
    pub all_of: bool,
}

/// An operational mode (version 2).
#[derive(Debug, Serialize)]
pub struct ModeOut {
    pub name: String,
    pub scope_id: usize,
    pub requires: Vec<String>,
    /// The ladder: the rungs to fall to, in order.
    pub fallback: Vec<String>,
    /// The scope path that reaches this rung's state.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reaction: Option<String>,
    /// A windowed rung's least duration.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub window_ms: Option<f64>,
    /// `{on: <function>, to: <mode>}`: how a windowed rung is left early.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exit: Option<ExitOut>,
}

#[derive(Debug, Serialize)]
pub struct ExitOut {
    pub on: String,
    pub to: String,
}

/// What fires a node path (version 2): `{"timer": {"rate_hz", "jitter_ms"}}`,
/// `{"input": [...]}`, or `"once"` / `"spontaneous"` / `"unclassified"`.
#[derive(Debug, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum TriggerOut {
    Timer {
        rate_hz: f64,
        #[serde(skip_serializing_if = "Option::is_none")]
        jitter_ms: Option<f64>,
    },
    Input(Vec<String>),
    Once,
    Spontaneous,
    Unclassified,
}

/// A path's safe state (version 2): the endpoint it commands it on.
#[derive(Debug, Serialize)]
pub struct SafeStateOut {
    pub emits: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub settle_ms: Option<f64>,
    /// `true` when the settle is derived from a braking profile.
    pub derived: bool,
}

/// A subscriber's declared reaction (version 2): the detector edge.
#[derive(Debug, Serialize)]
pub struct OnViolationOut {
    pub reaction: String,
    pub on: Vec<&'static str>,
}

/// A ROS node vertex.
#[derive(Debug, Serialize)]
pub struct NodeOut {
    pub fqn: String,
    pub scope_id: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pkg: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub criticality: Option<String>,
    /// The criticality the hazards derive for this node (version 2).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub derived_criticality: Option<DerivedCriticalityOut>,
    /// Declared in a contract (version 2); `false` for a node only a
    /// topic's endpoint list names.
    pub contracted: bool,
}

/// A node's hazard-derived criticality: the severity level, the hazard that
/// set it, and how the node relates to that hazard.
#[derive(Debug, Serialize)]
pub struct DerivedCriticalityOut {
    pub level: String,
    pub hazard: String,
    pub role: &'static str,
}

/// A topic vertex.
#[derive(Debug, Serialize)]
pub struct TopicOut {
    pub fqn: String,
    #[serde(rename = "type")]
    pub msg_type: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_transport_ms: Option<f64>,
    /// The rate derived from the timers that drive the topic.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub derived_rate_hz: Option<f64>,
    /// `pub` | `sub` | `both`: the side provided outside the tree (version 2).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub external: Option<&'static str>,
}

/// A publisher edge: node → topic.
#[derive(Debug, Serialize)]
pub struct PubEdgeOut {
    pub node: String,
    pub topic: String,
    pub endpoint: String,
    /// Topic-level declared rate (the primary "declared rate fact").
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_rate_hz: Option<f64>,
    /// `on_demand: true`: the publisher promises no rate (version 2).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub on_demand: bool,
}

/// A subscriber edge: topic → node, tagged causal / state / required.
#[derive(Debug, Serialize)]
pub struct SubEdgeOut {
    pub topic: String,
    pub node: String,
    pub endpoint: String,
    /// `true` unless the endpoint declares `state: true`. Mutually
    /// exclusive with `state`.
    pub causal: bool,
    /// `state: true` — polled/read-latest, not a causal dependency. Our
    /// existing cycle-cut primitive (see `causal-dag` rule).
    pub state: bool,
    /// `required: true` — must receive at least once before operational.
    /// Orthogonal to `causal`/`state`.
    pub required: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_age_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_transport_ms: Option<f64>,
    /// The reaction this subscriber owes (version 2): a detector.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub on_violation: Option<OnViolationOut>,
}

/// A node-level path: input/output are the node's own endpoint names.
#[derive(Debug, Serialize)]
pub struct NodePathOut {
    pub node: String,
    pub path_name: String,
    /// The endpoints that trigger the path: its effective trigger's inputs
    /// (version 2; version 1 copied the legacy `input:` list).
    pub input: Vec<String>,
    pub output: Vec<String>,
    /// What fires the path (version 2).
    pub trigger: TriggerOut,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safe_state: Option<SafeStateOut>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_latency_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tolerance_ms: Option<f64>,
    pub scope_id: usize,
    /// Always `false` — node paths are intra-node by construction.
    pub cross_node: bool,
}

/// A scope-level path: input/output are resolved topic FQNs, spanning
/// potentially many nodes across the scope's subtree.
#[derive(Debug, Serialize)]
pub struct ScopePathOut {
    pub scope_id: usize,
    pub path_name: String,
    pub input_topics: Vec<String>,
    pub output_topics: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_latency_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tolerance_ms: Option<f64>,
    /// Always `true` — scope paths cross node boundaries by construction.
    pub cross_node: bool,
}

/// One directed edge participating in a cycle (node-to-node, topic as label).
#[derive(Debug, Clone, Serialize)]
pub struct CycleMemberOut {
    pub from: String,
    pub to: String,
    pub topic: String,
    pub state: bool,
}

/// A cycle found in the causal graph BEFORE `state:` cuts are applied
/// (i.e. `state: true` sub edges are included as if causal). Q1 needs both
/// "which cycles exist" and "which cut breaks them".
#[derive(Debug, Serialize)]
pub struct CycleOut {
    /// Full cycle, in traversal order.
    pub members: Vec<CycleMemberOut>,
    /// Subset of `members` that are `state: true` — the edges whose
    /// removal breaks this cycle in the actual (post-cut) causal-dag rule.
    pub cut_by_state: Vec<CycleMemberOut>,
    /// `true` iff at least one member is a `state:` edge — i.e. this cycle
    /// is already broken by the current declarations and the
    /// `causal-dag` rule does not flag it.
    pub broken: bool,
}

/// Build the full graph export from a resolved `ManifestIndex`.
pub fn build_export(index: &ManifestIndex) -> GraphExport {
    let global = manifest_graph::build_global_graph(index);

    // Node metadata (pkg, criticality) isn't tracked on `GlobalNode` — pull
    // it directly from the per-scope manifests, keyed by the same FQN
    // computation `build_global_graph` uses.
    let mut node_meta: HashMap<String, (Option<String>, Option<String>)> = HashMap::new();
    let mut pub_props: HashMap<String, ros_launch_manifest_types::EndpointProps> = HashMap::new();
    let mut sub_props: HashMap<String, ros_launch_manifest_types::EndpointProps> = HashMap::new();
    for resolved in index.manifests.values() {
        for (node_name, node_decl) in &resolved.manifest.nodes {
            // Same launch-dump-reconciled identity `build_global_graph`
            // uses for its node keys (`ManifestIndex::node_identity`) —
            // must agree or `node_meta.get(fqn)` below would silently miss.
            let node_fqn = super::manifest_loader::resolve_node_fqn(
                index,
                resolved.scope_id,
                &resolved.ns,
                node_name,
            );
            node_meta.insert(
                node_fqn.clone(),
                (resolved.pkg.clone(), node_decl.criticality.clone()),
            );
            for (ep, props) in &node_decl.publishers {
                pub_props.insert(format!("{node_fqn}/{ep}"), props.clone());
            }
            for (ep, props) in &node_decl.subscribers {
                sub_props.insert(format!("{node_fqn}/{ep}"), props.clone());
            }
        }
    }

    let mut nodes: Vec<NodeOut> = global
        .nodes
        .keys()
        .map(|fqn| {
            let contracted = node_meta.contains_key(fqn);
            let (pkg, criticality) = node_meta.get(fqn).cloned().unwrap_or_default();
            let scope_id = global.nodes[fqn].scope_id;
            NodeOut {
                fqn: fqn.clone(),
                scope_id,
                pkg,
                criticality,
                derived_criticality: index.derived_criticality.get(fqn).map(|d| {
                    DerivedCriticalityOut {
                        level: d.level.clone(),
                        hazard: d.hazard.clone(),
                        role: d.role,
                    }
                }),
                contracted,
            }
        })
        .collect();
    nodes.sort_by(|a, b| a.fqn.cmp(&b.fqn));

    let topics: Vec<TopicOut> = index
        .topics
        .values()
        .map(|t| TopicOut {
            fqn: t.fqn.clone(),
            msg_type: t.msg_type.clone(),
            rate_hz: t.rate_hz,
            max_transport_ms: t.max_transport_ms,
            derived_rate_hz: t.derived_rate_hz,
            external: index.externals.get(&t.fqn).map(|s| {
                use ros_launch_manifest_types::ExternalSide as S;
                match s {
                    S::Pub => "pub",
                    S::Sub => "sub",
                    S::Both => "both",
                }
            }),
        })
        .collect();

    let mut pub_edges = Vec::new();
    let mut sub_edges = Vec::new();
    for topic in index.topics.values() {
        for pref in &topic.publishers {
            let Some((node, ep)) = split_endpoint_ref(pref) else {
                continue;
            };
            let props = pub_props.get(pref);
            pub_edges.push(PubEdgeOut {
                node,
                topic: topic.fqn.clone(),
                endpoint: ep,
                rate_hz: topic.rate_hz,
                min_rate_hz: props.and_then(|p| p.min_rate_hz),
                max_rate_hz: props.and_then(|p| p.max_rate_hz),
                on_demand: props.is_some_and(|p| p.on_demand == Some(true)),
            });
        }
        for sref in &topic.subscribers {
            let Some((node, ep)) = split_endpoint_ref(sref) else {
                continue;
            };
            let props = sub_props.get(sref);
            let state = props.and_then(|p| p.state).unwrap_or(false);
            let required = props.and_then(|p| p.required).unwrap_or(false);
            sub_edges.push(SubEdgeOut {
                topic: topic.fqn.clone(),
                node,
                endpoint: ep,
                causal: !state,
                state,
                required,
                min_rate_hz: props.and_then(|p| p.min_rate_hz),
                max_age_ms: props.and_then(|p| p.max_age.map(|d| d.as_millis_f64())),
                max_transport_ms: props
                    .and_then(|p| p.max_transport.map(|d| d.as_millis_f64()))
                    .or(topic.max_transport_ms),
                on_violation: props.and_then(|p| p.on_violation.as_ref()).map(|ov| {
                    OnViolationOut {
                        reaction: ov.reaction.clone(),
                        on: ov.on.iter().map(|k| fault_kind(*k)).collect(),
                    }
                }),
            });
        }
    }
    pub_edges
        .sort_by(|a, b| (&a.topic, &a.node, &a.endpoint).cmp(&(&b.topic, &b.node, &b.endpoint)));
    sub_edges
        .sort_by(|a, b| (&a.topic, &a.node, &a.endpoint).cmp(&(&b.topic, &b.node, &b.endpoint)));

    let mut node_paths: Vec<NodePathOut> = index
        .node_paths
        .iter()
        .map(|p| NodePathOut {
            node: p.node_fqn.clone(),
            path_name: p.path_name.clone(),
            input: match p.path.effective_trigger() {
                ros_launch_manifest_types::EffectiveTrigger::Input(eps) => eps,
                _ => Vec::new(),
            },
            output: p.path.output.clone(),
            trigger: trigger_out(&p.path),
            safe_state: p.path.safe_state.as_ref().map(|ss| SafeStateOut {
                emits: ss.emits.clone(),
                settle_ms: ss.settle.map(|d| d.as_millis_f64()),
                derived: ss.settle_profile.is_some(),
            }),
            max_latency_ms: p.path.max_latency.map(|d| d.as_millis_f64()),
            tolerance_ms: p.path.tolerance.map(|d| d.as_millis_f64()),
            scope_id: p.scope_id,
            cross_node: false,
        })
        .collect();
    node_paths.sort_by(|a, b| {
        (a.scope_id, &a.node, &a.path_name).cmp(&(b.scope_id, &b.node, &b.path_name))
    });

    let mut scope_paths: Vec<ScopePathOut> = index
        .scope_paths
        .iter()
        .map(|p| ScopePathOut {
            scope_id: p.scope_id,
            path_name: p.path_name.clone(),
            input_topics: p.input_topics.clone(),
            output_topics: p.output_topics.clone(),
            max_latency_ms: p.path.max_latency.map(|d| d.as_millis_f64()),
            tolerance_ms: p.path.tolerance.map(|d| d.as_millis_f64()),
            cross_node: true,
        })
        .collect();
    scope_paths.sort_by(|a, b| (a.scope_id, &a.path_name).cmp(&(b.scope_id, &b.path_name)));

    let cycles = find_cycles(&global);

    let endpoints = |refs: &[String]| -> Vec<EndpointOut> {
        refs.iter()
            .filter_map(|r| split_endpoint_ref(r))
            .map(|(node, endpoint)| EndpointOut { node, endpoint })
            .collect()
    };
    let services: Vec<ServiceOut> = index
        .services
        .values()
        .map(|s| ServiceOut {
            fqn: s.fqn.clone(),
            srv_type: s.srv_type.clone(),
            servers: endpoints(&s.servers),
            clients: endpoints(&s.clients),
            external: index.endpoint_externals.get(&s.fqn).map(|e| {
                use ros_launch_manifest_types::ExternalEndpointSide as E;
                match e {
                    E::Server => "server",
                    E::Client => "client",
                    E::Both => "both",
                }
            }),
        })
        .collect();
    let hazards: Vec<HazardOut> = index
        .hazards
        .iter()
        .map(|h| HazardOut {
            name: h.name.clone(),
            scope_id: h.scope_id,
            severity: h.decl.severity.clone(),
            guards: h
                .guards
                .iter()
                .map(|g| GuardOut {
                    members: g.members.clone(),
                    all_of: g.all_of,
                })
                .collect(),
            on: h.decl.on.iter().map(|k| fault_kind(*k)).collect(),
            ftti_ms: h.decl.ftti.map(|d| d.as_millis_f64()),
            reaction: h.decl.reaction.clone(),
            entry_speed_mps: h.decl.entry_speed,
        })
        .collect();
    let functions: Vec<FunctionOut> = index
        .functions
        .iter()
        .map(|f| FunctionOut {
            name: f.name.clone(),
            scope_id: f.scope_id,
            members: f.group.members.clone(),
            all_of: f.group.all_of,
        })
        .collect();
    let modes: Vec<ModeOut> = index
        .modes
        .iter()
        .map(|m| ModeOut {
            name: m.name.clone(),
            scope_id: m.scope_id,
            requires: m.decl.requires.clone(),
            fallback: m.decl.fallback.clone(),
            reaction: m.decl.reaction.clone(),
            window_ms: m.decl.window.as_ref().map(|w| w.duration.as_millis_f64()),
            exit: m.decl.exit.as_ref().map(|e| ExitOut {
                on: e.on.clone(),
                to: e.to.clone(),
            }),
        })
        .collect();

    GraphExport {
        version: SCHEMA_VERSION,
        nodes,
        topics,
        pub_edges,
        sub_edges,
        node_paths,
        scope_paths,
        cycles,
        services,
        hazards,
        functions,
        modes,
    }
}

/// A path's trigger as the export writes it (version 2).
fn trigger_out(path: &ros_launch_manifest_types::PathDecl) -> TriggerOut {
    use ros_launch_manifest_types::EffectiveTrigger as T;
    match path.effective_trigger() {
        T::Timer { rate_hz } => TriggerOut::Timer {
            rate_hz,
            jitter_ms: path.timer_jitter().map(|d| d.as_millis_f64()),
        },
        T::Input(eps) => TriggerOut::Input(eps),
        T::Once => TriggerOut::Once,
        T::Spontaneous => TriggerOut::Spontaneous,
        T::Unclassified => TriggerOut::Unclassified,
    }
}

fn fault_kind(kind: ros_launch_manifest_types::FaultKind) -> &'static str {
    use ros_launch_manifest_types::FaultKind as F;
    match kind {
        F::Omission => "omission",
        F::Late => "late",
        F::Loss => "loss",
        F::Reported => "reported",
    }
}

/// Split an endpoint FQN like `/ns/node/endpoint` into `(node_fqn, endpoint_name)`.
fn split_endpoint_ref(ep_ref: &str) -> Option<(String, String)> {
    let pos = ep_ref.rfind('/')?;
    let node = &ep_ref[..pos];
    let ep = &ep_ref[pos + 1..];
    if node.is_empty() || ep.is_empty() {
        return None;
    }
    Some((node.to_string(), ep.to_string()))
}

/// Find cycles in the node-to-node causal graph, counting `state:` edges as
/// causal (i.e. the graph BEFORE cuts are applied — Q1 needs to see cycles
/// that a cut currently breaks, not just the ones that survive).
///
/// Uses a single DFS pass with the classic white/gray/black coloring: each
/// back-edge encountered (an edge into a vertex currently on the DFS stack)
/// yields one simple cycle, reconstructed from the DFS path. This finds
/// every cycle reachable via a back-edge in this traversal order — the
/// standard practical approach for "list the cycles" at study scale. It is
/// not a from-scratch enumeration of all elementary cycles (e.g. Johnson's
/// algorithm) — dense graphs with many alternate simple cycles through the
/// same SCC may see only a subset. Good enough for Autoware-scale manifests
/// (this is an export for human/tooling inspection, not the validation
/// rule).
fn find_cycles(graph: &GlobalDataflowGraph) -> Vec<CycleOut> {
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Color {
        White,
        Gray,
        Black,
    }

    struct State<'g> {
        graph: &'g GlobalDataflowGraph,
        color: HashMap<&'g str, Color>,
        path_edges: Vec<usize>,
        path_nodes: Vec<&'g str>,
        cycles: Vec<Vec<usize>>,
        seen: HashSet<Vec<usize>>,
    }

    fn dfs<'g>(v: &'g str, st: &mut State<'g>) {
        st.color.insert(v, Color::Gray);
        st.path_nodes.push(v);

        if let Some(out_idx) = st.graph.out_edges.get(v) {
            let mut idxs = out_idx.clone();
            idxs.sort_unstable();
            for eidx in idxs {
                let to = st.graph.edges[eidx].to.as_str();
                match st.color.get(to).copied().unwrap_or(Color::White) {
                    Color::White => {
                        st.path_edges.push(eidx);
                        dfs(to, st);
                        st.path_edges.pop();
                    }
                    Color::Gray => {
                        if let Some(start) = st.path_nodes.iter().position(|&n| n == to) {
                            let mut cyc: Vec<usize> = st.path_edges[start..].to_vec();
                            cyc.push(eidx);
                            let mut key = cyc.clone();
                            key.sort_unstable();
                            if st.seen.insert(key) {
                                st.cycles.push(cyc);
                            }
                        }
                    }
                    Color::Black => {}
                }
            }
        }

        st.path_nodes.pop();
        st.color.insert(v, Color::Black);
    }

    let mut order: Vec<&str> = graph.nodes.keys().map(|s| s.as_str()).collect();
    order.sort_unstable();

    let mut st = State {
        graph,
        color: order.iter().map(|&n| (n, Color::White)).collect(),
        path_edges: Vec::new(),
        path_nodes: Vec::new(),
        cycles: Vec::new(),
        seen: HashSet::new(),
    };

    for v in order {
        if st.color.get(v).copied().unwrap_or(Color::White) == Color::White {
            dfs(v, &mut st);
        }
    }

    st.cycles
        .into_iter()
        .map(|edge_idxs| {
            let members: Vec<CycleMemberOut> = edge_idxs
                .iter()
                .map(|&i| {
                    let e = &graph.edges[i];
                    CycleMemberOut {
                        from: e.from.clone(),
                        to: e.to.clone(),
                        topic: e.topic.clone(),
                        state: e.is_state,
                    }
                })
                .collect();
            let cut_by_state: Vec<CycleMemberOut> =
                members.iter().filter(|m| m.state).cloned().collect();
            let broken = !cut_by_state.is_empty();
            CycleOut {
                members,
                cut_by_state,
                broken,
            }
        })
        .collect()
}

/// Escape a string for inclusion in a DOT quoted identifier.
fn dot_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// Render the export as a Graphviz DOT graph: topics as boxes, nodes as
/// ellipses, `state:` sub edges dashed. Kept deliberately plain — this is
/// for human inspection (`dot -Tsvg`), not a styling exercise.
pub fn render_dot(export: &GraphExport) -> String {
    let mut s = String::new();
    s.push_str("digraph causal {\n  rankdir=LR;\n");

    for n in &export.nodes {
        s.push_str(&format!(
            "  \"{}\" [shape=ellipse, label=\"{}\"];\n",
            dot_escape(&n.fqn),
            dot_escape(&n.fqn),
        ));
    }
    for t in &export.topics {
        s.push_str(&format!(
            "  \"{}\" [shape=box, label=\"{}\"];\n",
            dot_escape(&t.fqn),
            dot_escape(&t.fqn),
        ));
    }
    for e in &export.pub_edges {
        s.push_str(&format!(
            "  \"{}\" -> \"{}\";\n",
            dot_escape(&e.node),
            dot_escape(&e.topic),
        ));
    }
    for e in &export.sub_edges {
        let style = if e.state {
            " [style=dashed]"
        } else if e.required {
            " [color=blue]"
        } else {
            ""
        };
        s.push_str(&format!(
            "  \"{}\" -> \"{}\"{style};\n",
            dot_escape(&e.topic),
            dot_escape(&e.node),
        ));
    }

    s.push_str("}\n");
    s
}

/// Write the graph export to `path`, picking JSON or DOT by extension
/// (`.dot` → Graphviz; anything else, including no extension → JSON).
pub fn export_to_file(index: &ManifestIndex, path: &Path) -> Result<()> {
    let export = build_export(index);
    let is_dot = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.eq_ignore_ascii_case("dot"))
        .unwrap_or(false);

    if is_dot {
        std::fs::write(path, render_dot(&export))?;
    } else {
        let json = serde_json::to_string_pretty(&export)?;
        std::fs::write(path, json)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::{
        launch_dump::{LaunchDump, ScopeEntry, ScopeOrigin},
        manifest_loader::ContractSources,
    };
    use std::collections::HashMap;

    /// Load a synthetic system through the real `load_manifests` pipeline
    /// (single file scope, contract delivered via the overlay channel into
    /// a tempdir) so the export sees exactly the same `ManifestIndex` shape
    /// `check` builds:
    ///
    /// - `sensor` publishes `scan` (10 Hz)
    /// - `filter` subscribes `scan` (causal), publishes `filtered`; has a
    ///   node-level path `passthrough` (scan endpoint -> output endpoint)
    /// - `localizer` subscribes `odom` (causal), publishes `pose`
    /// - `planner` subscribes `pose` (`state: true, required: true` — the
    ///   cycle-breaking cut), publishes `cmd`
    /// - `controller` subscribes `cmd` (causal), publishes `odom`
    ///
    /// Causal chain `localizer -> planner -> controller -> localizer`
    /// (via `pose`, `cmd`, `odom`) would cycle if `pose` weren't
    /// `state: true`. A scope-level path `end_to_end` (scan -> filtered)
    /// covers the scope-path export.
    fn synthetic_index() -> ManifestIndex {
        let yaml = r#"
version: 1
nodes:
  sensor:
    pub: [scan]
  filter:
    sub: [scan]
    pub: [output]
    paths:
      passthrough:
        input: [scan]
        output: [output]
        max_latency: 20.0ms
  localizer:
    sub: [odom]
    pub: [pose]
  planner:
    sub:
      pose: { state: true, required: true }
    pub: [cmd]
  controller:
    sub: [cmd]
    pub: [odom]
topics:
  scan:
    type: sensor_msgs/msg/LaserScan
    rate_hz: 10.0
    pub: [sensor/scan]
    sub: [filter/scan]
  filtered:
    type: sensor_msgs/msg/LaserScan
    pub: [filter/output]
    sub: []
  pose:
    type: geometry_msgs/msg/PoseStamped
    pub: [localizer/pose]
    sub: [planner/pose]
  cmd:
    type: geometry_msgs/msg/Twist
    pub: [planner/cmd]
    sub: [controller/cmd]
  odom:
    type: nav_msgs/msg/Odometry
    pub: [controller/odom]
    sub: [localizer/odom]
paths:
  end_to_end:
    input: [scan]
    output: [filtered]
    max_latency: 50.0ms
"#;

        let tmp = tempfile::TempDir::new().unwrap();
        let dest_dir = tmp.path().join("synthetic_pkg").join("launch");
        std::fs::create_dir_all(&dest_dir).unwrap();
        std::fs::write(dest_dir.join("synthetic.contract.yaml"), yaml).unwrap();

        let dump = LaunchDump {
            dropped_actions: Vec::new(),
            node: vec![],
            load_node: vec![],
            container: vec![],
            lifecycle_node: vec![],
            file_data: HashMap::new(),
            variables: HashMap::new(),
            scopes: vec![ScopeEntry {
                id: 0,
                origin: Some(ScopeOrigin {
                    pkg: Some("synthetic_pkg".to_string()),
                    file: "synthetic.launch.xml".to_string(),
                    path: None,
                }),
                ns: "".to_string(),
                args: HashMap::new(),
                parent: None,
            }],
        };

        let sources = ContractSources {
            overlay: Some(tmp.path().to_path_buf()),
            provider: false,
        };
        crate::ros::manifest_loader::load_manifests(&dump, &sources)
            .expect("synthetic manifest loads")
    }

    #[test]
    fn nodes_topics_and_pkg_criticality() {
        let index = synthetic_index();
        let export = build_export(&index);

        assert_eq!(export.nodes.len(), 5, "expected 5 nodes");
        assert_eq!(export.topics.len(), 5, "expected 5 topics");

        let planner = export
            .nodes
            .iter()
            .find(|n| n.fqn == "/planner")
            .expect("planner node present");
        assert_eq!(planner.pkg.as_deref(), Some("synthetic_pkg"));

        let scan = export
            .topics
            .iter()
            .find(|t| t.fqn == "/scan")
            .expect("scan topic present");
        assert_eq!(scan.rate_hz, Some(10.0));
    }

    #[test]
    fn sub_edge_tags_state_and_required() {
        let index = synthetic_index();
        let export = build_export(&index);

        let pose_edge = export
            .sub_edges
            .iter()
            .find(|e| e.topic == "/pose" && e.node == "/planner")
            .expect("pose sub edge present");
        assert!(pose_edge.state, "pose->planner should be state: true");
        assert!(!pose_edge.causal, "state edges are not causal");
        assert!(pose_edge.required, "pose->planner should be required: true");

        let scan_edge = export
            .sub_edges
            .iter()
            .find(|e| e.topic == "/scan" && e.node == "/filter")
            .expect("scan sub edge present");
        assert!(!scan_edge.state);
        assert!(scan_edge.causal);
        assert!(!scan_edge.required);
    }

    #[test]
    fn cycle_found_and_marked_broken_by_state_cut() {
        let index = synthetic_index();
        let export = build_export(&index);

        assert_eq!(
            export.cycles.len(),
            1,
            "expected exactly one cycle: localizer -> planner -> controller -> localizer, got {:?}",
            export.cycles
        );
        let cycle = &export.cycles[0];
        assert!(
            cycle.broken,
            "cycle should be marked broken (pose is state: true)"
        );
        assert_eq!(cycle.cut_by_state.len(), 1);
        assert_eq!(cycle.cut_by_state[0].topic, "/pose");
        assert_eq!(cycle.members.len(), 3, "3-edge cycle");
    }

    #[test]
    fn node_and_scope_paths_exported() {
        let index = synthetic_index();
        let export = build_export(&index);

        assert_eq!(export.node_paths.len(), 1);
        let np = &export.node_paths[0];
        assert_eq!(np.node, "/filter");
        assert_eq!(np.path_name, "passthrough");
        assert_eq!(np.max_latency_ms, Some(20.0));
        assert!(!np.cross_node);

        assert_eq!(export.scope_paths.len(), 1);
        let sp = &export.scope_paths[0];
        assert_eq!(sp.path_name, "end_to_end");
        assert_eq!(sp.input_topics, vec!["/scan".to_string()]);
        assert_eq!(sp.output_topics, vec!["/filtered".to_string()]);
        assert!(sp.cross_node);
    }

    #[test]
    fn dot_render_is_well_formed_and_marks_state_dashed() {
        let index = synthetic_index();
        let export = build_export(&index);
        let dot = render_dot(&export);

        assert!(dot.starts_with("digraph causal {"));
        assert!(dot.trim_end().ends_with('}'));
        assert!(dot.contains("style=dashed"), "state edge should be dashed");
        // Every node/topic fqn should appear as a quoted identifier.
        for n in &export.nodes {
            assert!(dot.contains(&format!("\"{}\"", n.fqn)));
        }
        for t in &export.topics {
            assert!(dot.contains(&format!("\"{}\"", t.fqn)));
        }
    }
}
