//! Phase 43.2 — the narrow, source-agnostic view of contract data the
//! rule engine actually reads. Two constructors: `from_manifest_index`
//! (legacy replay path, behavior-preserving) and `from_model` (replay
//! `--model`, where the checked SystemModel is the single source).
//!
//! Precomputing the `"node FQN/endpoint"` → contract maps here also
//! removes the per-event manifest-tree rescans the rules used to do.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use ros_launch_manifest_model as model;
use ros_launch_manifest_types::OptBound;

use ros_launch_resolve::ros::manifest_loader::ManifestIndex;

/// What the engine needs of one declared topic.
#[derive(Debug, Clone, Default)]
pub struct TopicView {
    pub msg_type: String,
    /// Endpoint refs (`"<node FQN>/<endpoint>"`).
    pub publishers: Vec<String>,
    pub subscribers: Vec<String>,
    /// Declared transport drop budget as a fraction.
    pub max_drop_rate: Option<f64>,
}

/// What the engine needs of one scope path.
#[derive(Debug, Clone, Default)]
pub struct ScopePathView {
    pub name: String,
    pub input_topics: Vec<String>,
    pub output_topics: Vec<String>,
    pub max_latency_ms: Option<f64>,
}

/// The engine's contract input.
#[derive(Debug, Clone, Default)]
pub struct ContractView {
    /// Topic FQN → view.
    pub topics: BTreeMap<String, TopicView>,
    /// Externally-provided topic FQNs (side irrelevant to runtime checks).
    pub externals: BTreeSet<String>,
    /// `"<node FQN>/<endpoint>"` → declared publisher `min_rate_hz`.
    pub pub_min_rate: HashMap<String, f64>,
    /// `"<node FQN>/<endpoint>"` → declared subscriber `max_age_ms`.
    pub sub_max_age: HashMap<String, f64>,
    /// Lifecycle (managed) node FQNs — contract checks gate on Active.
    pub lifecycle_nodes: HashSet<String>,
    pub scope_paths: Vec<ScopePathView>,
    /// Hazards to watch live (phase 73).
    pub hazards: Vec<HazardWatch>,
    /// Modes to derive availability for (phase 75).
    pub modes: Vec<ModeWatch>,
    /// The declared fields the interceptor reads, in the order of
    /// `PLAY_LAUNCH_INTERCEPTION_FIELDS`: a `FieldValue` event's index is a
    /// position here (phase 102 M0b).
    pub fields: Vec<(String, String)>,
    /// Value functions by function key (phase 102 M2).
    pub value_fns: BTreeMap<String, ValueFn>,
    /// Commanded topics whose consumer states what it does on silence (a
    /// subscriber's `max_age` + `on_violation`, or an external side's
    /// `on_silence`): silence there is not `final-element-silent`.
    pub silence_handled: BTreeSet<String>,
    /// Hazard key to the fault classes its `on:` names explicitly (phase
    /// 102 M2). The three communication classes are judged only where a
    /// contract asks for them: a stamp re-sent at rate is right for a state
    /// topic (phase 88's `stale_stamp`), so an omitted `on:` does not.
    pub hazard_classes: BTreeMap<String, Vec<model::FaultKind>>,
    /// rlm v0.3.18 (#6): per hazard key, the levels it runs that state their
    /// own deadline (`modes.<m>.within`), judged on the first publish on the
    /// level's output after the fault (`level-within`).
    pub level_deadlines: BTreeMap<String, Vec<LevelDeadline>>,
    /// Node FQNs some contract describes: every node of a launch scope that
    /// has a contract (`ScopeInfo.manifest`), plus the owner of any endpoint,
    /// path, concurrency or parameter entry a contract states about a node
    /// in another scope. `graph-deviation-runtime` judges
    /// only these nodes' endpoints. A node no contract describes is under
    /// no claim, so an undeclared topic of its is not a deviation (#0080:
    /// 481 warnings on an Autoware run where 4 of 83 scopes had a contract).
    pub contracted_nodes: BTreeSet<String>,
}

/// The node FQN of a `"<node FQN>/<entry>"` contract key.
fn key_node(key: &str) -> Option<&str> {
    key.rsplit_once('/')
        .map(|(n, _)| n)
        .filter(|n| !n.is_empty())
}

/// One required function of a mode, as the live observer needs it.
#[derive(Debug, Clone, Default)]
pub struct RequiredFn {
    /// Member topic FQNs.
    pub members: Vec<String>,
    /// A redundant group: lost only when fewer than `m` members hold.
    pub all_of: bool,
    /// MooN's M (rlm v0.3.13); `None` is 1.
    pub m: Option<u32>,
    /// The function's key in `ContractView::value_fns` when it is also lost
    /// by value (`of:` + `when:`).
    pub value_fn: Option<String>,
}

impl RequiredFn {
    /// A function that is one or more topics, lost when any member is.
    pub fn topics(members: Vec<String>) -> Self {
        Self {
            members,
            ..Self::default()
        }
    }
}

/// One mode, as the live observer needs it (phase 75): the functions it
/// requires, the ladder it falls to, and (rlm v0.3.13) what it commands.
#[derive(Debug, Clone, Default)]
pub struct ModeWatch {
    pub key: String,
    pub requires: Vec<RequiredFn>,
    /// Fallback levels, in order, by mode key.
    pub fallback: Vec<String>,
    /// rlm v0.3.18 (#4): at least one of these must be lost for this mode
    /// to be selected as a level; empty means no such condition.
    pub requires_lost: Vec<RequiredFn>,
    /// rlm v0.3.18 (#5): an exit that re-enters a level above this one.
    pub exit: Option<ExitWatch>,
    /// Commanded topic FQN to the predicates every message on it satisfies
    /// while this level is active.
    pub commands: BTreeMap<String, Vec<model::PredicateContract>>,
    /// When the invariant starts after the level is entered: the largest
    /// route the checker derived for this level over the hazards that reach
    /// it (`LevelRoute.route_ms`). `None` when no hazard's walk reached it,
    /// and then the invariant is not judged.
    pub bound_ms: Option<f64>,
}

/// A level's own deadline, as the observer judges it (rlm v0.3.18): the
/// first publish on one of `outputs` after the fault, plus the settle of the
/// safe state commanded there, against `within_ms`.
#[derive(Debug, Clone, Default)]
pub struct LevelDeadline {
    pub level: String,
    pub outputs: Vec<String>,
    pub within_ms: f64,
    pub settle_ms: Option<f64>,
}

/// An exit that re-enters a level above (rlm v0.3.18, #5).
#[derive(Debug, Clone, Default)]
pub struct ExitWatch {
    /// The function whose holding takes the exit.
    pub holds: RequiredFn,
    /// The mode key re-entered.
    pub to: String,
}

/// A value function (rlm v0.1.46): lost while its predicate holds on a
/// sample of one of its topics.
#[derive(Debug, Clone)]
pub struct ValueFn {
    pub topics: Vec<String>,
    pub when: model::PredicateContract,
}

/// A hazard as the live observer needs it (phase 73): which topics going
/// silent IS the fault, which topics carry the reaction, and the declared
/// numbers to judge the reaction against.
#[derive(Debug, Clone, Default)]
pub struct HazardWatch {
    pub key: String,
    /// Guard topic FQNs, all groups flattened.
    pub guards: Vec<String>,
    /// Topics the reaction commands the safe state on.
    pub sinks: Vec<String>,
    pub ftti_ms: Option<f64>,
    pub settle_ms: Option<f64>,
    /// The fastest declared detector on a guard's reacting subscribers, in
    /// ms: the liveliness lease or `max_age`, whichever is shorter. `None`
    /// means the observer judges silence against the topic's own cadence.
    pub silence_ms: Option<f64>,
    /// A reacting subscriber reports through `/diagnostics`.
    pub via_diagnostics: bool,
    /// Sinks whose publisher the contract declares external (phase 83 F2):
    /// no host process publishes them, so the reaction is observed at the
    /// first host take. A sink the observer never sees published is treated
    /// the same way at runtime, declared or not.
    pub off_host_sinks: Vec<String>,
}

/// The event chains a hazard's reaction runs, in order (phase 75; the
/// meaning rlm v0.3.15 states, play_launch phase 103 D4).
///
/// A reaction naming an event chain IS that chain, one level. One naming a
/// MODE is the mode the hazard takes away, and its reaction is that mode's
/// `fallback:` sequence: each level by its own `reaction:`. The named mode's
/// own `reaction:` is not part of it. The checker charges each of these
/// levels; the observer and `measure` read the same list, so the first
/// level that answers is the reaction. (Before, they read only the LAST
/// level, so a reaction that a higher level gave in time was not seen.)
pub fn reaction_paths(contracts: &model::Contracts, reaction: &str) -> Vec<String> {
    let Some(mode) = contracts.modes.get(reaction) else {
        return vec![reaction.to_string()];
    };
    mode.fallback
        .iter()
        .filter_map(|level| contracts.modes.get(level).and_then(|m| m.reaction.clone()))
        .collect()
}

/// The levels of a hazard's fallback sequence it runs (rlm v0.3.18), as
/// `(mode key, event chain)`: the checker's `degradation_levels` when it
/// wrote them, which leave out a level the fault removes and a level that
/// `requires_lost` a function the fault does not remove; else every level.
pub fn run_levels(
    contracts: &model::Contracts,
    h: &model::HazardContract,
) -> Vec<(String, String)> {
    let Some(reaction) = h.reaction.as_ref() else {
        return Vec::new();
    };
    let Some(mode) = contracts.modes.get(reaction) else {
        return vec![(reaction.clone(), reaction.clone())];
    };
    let bare = |k: &str| k.rsplit_once('/').map_or(k, |(_, n)| n).to_string();
    mode.fallback
        .iter()
        .filter(|level| {
            h.degradation_levels.is_empty()
                || h.degradation_levels.iter().any(|l| l.level == bare(level))
        })
        .filter_map(|level| {
            contracts
                .modes
                .get(level)
                .and_then(|m| m.reaction.clone())
                .map(|p| (level.clone(), p))
        })
        .collect()
}

/// The levels a hazard runs that state their own deadline (rlm v0.3.18,
/// #6), each with its outputs and the settle of the safe state commanded
/// there (the largest, when several paths command one).
pub fn level_deadlines(m: &model::SystemModel, h: &model::HazardContract) -> Vec<LevelDeadline> {
    let settle_on = |outputs: &[String]| {
        m.contracts
            .node_paths
            .values()
            .filter_map(|p| {
                let ss = p.safe_state.as_ref()?;
                outputs
                    .iter()
                    .any(|t| {
                        m.structure
                            .topics
                            .get(t)
                            .is_some_and(|w| w.publishers.contains(&ss.emits))
                    })
                    .then_some(ss.settle_ms)
                    .flatten()
            })
            .fold(None, |a: Option<f64>, v| Some(a.map_or(v, |x| x.max(v))))
    };
    run_levels(&m.contracts, h)
        .into_iter()
        .filter_map(|(level, path)| {
            let within_ms = m.contracts.modes.get(&level)?.within_ms?;
            let outputs = m.contracts.scope_paths.get(&path)?.output.clone();
            Some(LevelDeadline {
                settle_ms: settle_on(&outputs),
                level,
                outputs,
                within_ms,
            })
        })
        .collect()
}

/// The topics a hazard's reaction commands: the outputs of every event
/// chain of the levels it runs ([`run_levels`]), in order, each once.
pub fn hazard_sinks(contracts: &model::Contracts, h: &model::HazardContract) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for (_, path) in run_levels(contracts, h) {
        for t in contracts
            .scope_paths
            .get(&path)
            .map(|p| p.output.clone())
            .unwrap_or_default()
        {
            if !out.contains(&t) {
                out.push(t);
            }
        }
    }
    out
}

/// The topics a hazard's reaction commands: the outputs of every event
/// chain in [`reaction_paths`], in order, each once.
pub fn reaction_sinks(contracts: &model::Contracts, reaction: Option<&String>) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for path in reaction
        .map(|r| reaction_paths(contracts, r))
        .unwrap_or_default()
    {
        for t in contracts
            .scope_paths
            .get(&path)
            .map(|p| p.output.clone())
            .unwrap_or_default()
        {
            if !out.contains(&t) {
                out.push(t);
            }
        }
    }
    out
}

impl ContractView {
    pub fn is_empty(&self) -> bool {
        self.topics.is_empty() && self.externals.is_empty()
    }

    /// Legacy source: the manifest tree loaded at replay time.
    ///
    /// Phase 47.B3: `replay`'s only remaining contract source is
    /// [`Self::from_model`] (the record-only replay path this fed is
    /// retired), so this is unreachable in a plain (non-test) build. Kept,
    /// not deleted: `from_model_matches_from_manifest_index` below (and
    /// several other unit tests in `runtime_enforcement::mod`) use it as
    /// the independent reference to prove `from_model` agrees with it.
    #[allow(dead_code)]
    pub fn from_manifest_index(index: &ManifestIndex) -> Self {
        let mut view = ContractView::default();

        for (fqn, t) in &index.topics {
            view.topics.insert(
                fqn.clone(),
                TopicView {
                    msg_type: t.msg_type.clone(),
                    publishers: t.publishers.clone(),
                    subscribers: t.subscribers.clone(),
                    max_drop_rate: t
                        .drop
                        .as_ref()
                        .and_then(|d| d.max_count.count_per())
                        .map(|c| c.drop_rate()),
                },
            );
        }
        view.externals = index.externals.keys().cloned().collect();

        for resolved in index.manifests.values() {
            for (node_name, node) in &resolved.manifest.nodes {
                let node_fqn = super::qualify(&resolved.ns, node_name);
                if node.lifecycle.unwrap_or(false) {
                    view.lifecycle_nodes.insert(node_fqn.clone());
                }
                for (ep, props) in &node.publishers {
                    if let Some(min) = props.min_rate_hz.hz() {
                        view.pub_min_rate.insert(format!("{node_fqn}/{ep}"), min);
                    }
                }
                for (ep, props) in &node.subscribers {
                    if let Some(max) = props.max_age.ms() {
                        view.sub_max_age.insert(format!("{node_fqn}/{ep}"), max);
                    }
                }
            }
        }

        for sp in &index.scope_paths {
            view.scope_paths.push(ScopePathView {
                name: sp.path_name.clone(),
                input_topics: sp.input_topics.clone(),
                output_topics: sp.output_topics.clone(),
                max_latency_ms: sp.path.max_latency.ms(),
            });
        }
        for h in &index.hazards {
            let guards: Vec<String> = h.guards.iter().flat_map(|g| g.members.clone()).collect();
            let sinks: Vec<String> = h
                .decl
                .reaction
                .as_ref()
                .and_then(|r| {
                    index
                        .scope_paths
                        .iter()
                        .find(|p| p.scope_id == h.scope_id && &p.path_name == r)
                })
                .map(|p| p.output_topics.clone())
                .unwrap_or_default();
            let settle_ms = index
                .node_paths
                .iter()
                .filter_map(|p| {
                    let ss = p.path.safe_state.as_ref()?;
                    let emits = format!("{}/{}", p.node_fqn, ss.emits);
                    sinks
                        .iter()
                        .any(|t| {
                            index
                                .topics
                                .get(t)
                                .is_some_and(|tp| tp.publishers.contains(&emits))
                        })
                        .then_some(ss.settle.ms())
                        .flatten()
                })
                .fold(None, |a: Option<f64>, v| Some(a.map_or(v, |x| x.max(v))));
            let (mut silence_ms, mut via_diagnostics) = (None::<f64>, false);
            for g in &guards {
                let Some(t) = index.topics.get(g) else {
                    continue;
                };
                for sub in &t.subscribers {
                    let Some((node, ep)) = sub.rsplit_once('/') else {
                        continue;
                    };
                    let props = index.manifests.values().find_map(|m| {
                        m.manifest.nodes.iter().find_map(|(name, d)| {
                            (super::qualify(&m.ns, name) == node)
                                .then(|| d.subscribers.get(ep))
                                .flatten()
                        })
                    });
                    let Some(ov) = props.and_then(|p| p.on_violation.as_ref()) else {
                        continue;
                    };
                    let lease = ros_launch_manifest_types::QosDecl::effective(
                        index.topics.get(g).and_then(|t| t.qos.as_ref()),
                        props.and_then(|p| p.qos.as_ref()),
                    )
                    .lease_duration
                    .map(|d| d.as_millis_f64());
                    for a in [props.and_then(|p| p.max_age.ms()), lease]
                        .into_iter()
                        .flatten()
                    {
                        silence_ms = Some(silence_ms.map_or(a, |s: f64| s.min(a)));
                    }
                    via_diagnostics |=
                        ov.mechanism == ros_launch_manifest_types::DetectMechanism::Diagnostics;
                }
            }
            // Phase 83: a derived settle is the checker's own number, per
            // hazard; the literal scan above is the fallback.
            let settle_ms = index
                .budgets
                .iter()
                .find(|b| {
                    b.hazard == h.name
                        && b.role == ros_launch_resolve::ros::manifest_loader::LevelRole::Mrc
                })
                .and_then(|b| b.settle_ms)
                .or(settle_ms);
            let off_host_sinks = sinks
                .iter()
                .filter(|t| {
                    matches!(
                        index.externals.get(*t),
                        Some(
                            ros_launch_manifest_types::ExternalSide::Pub
                                | ros_launch_manifest_types::ExternalSide::Both
                        )
                    )
                })
                .cloned()
                .collect();
            view.hazards.push(HazardWatch {
                key: h.name.clone(),
                guards,
                sinks,
                ftti_ms: h.decl.ftti.ms(),
                settle_ms,
                silence_ms,
                via_diagnostics,
                off_host_sinks,
            });
        }
        for m in &index.modes {
            let to_fn = |r: &String| {
                index
                    .functions
                    .iter()
                    .find(|f| f.scope_id == m.scope_id && &f.name == r)
                    .map(|f| RequiredFn {
                        members: f.group.members.clone(),
                        all_of: f.group.all_of,
                        m: f.m,
                        value_fn: None,
                    })
                    .unwrap_or_else(|| RequiredFn::topics(vec![super::qualify("", r)]))
            };
            let requires_lost: Vec<RequiredFn> = m.decl.requires_lost.iter().map(to_fn).collect();
            let requires = m
                .decl
                .requires
                .iter()
                .map(|r| {
                    index
                        .functions
                        .iter()
                        .find(|f| f.scope_id == m.scope_id && &f.name == r)
                        .map(|f| RequiredFn {
                            members: f.group.members.clone(),
                            all_of: f.group.all_of,
                            m: f.m,
                            value_fn: None,
                        })
                        .unwrap_or_else(|| RequiredFn::topics(vec![super::qualify("", r)]))
                })
                .collect();
            view.modes.push(ModeWatch {
                key: m.name.clone(),
                requires,
                fallback: m.decl.fallback.clone(),
                requires_lost,
                ..ModeWatch::default()
            });
        }
        view
    }

    /// Does the model carry any contract at all? Not the same as a non-empty
    /// view: the view lists every topic of the launch graph, contract or not,
    /// so it is never empty for a real launch. The contracts section is, when
    /// no contract resolved (it serializes to nothing), and that is what
    /// decides whether the default enforcement mode has anything to enforce.
    pub fn model_has_contracts(m: &model::SystemModel) -> bool {
        fn empty(v: &serde_json::Value) -> bool {
            match v {
                serde_json::Value::Null => true,
                serde_json::Value::Array(a) => a.is_empty(),
                serde_json::Value::Object(o) => o.values().all(empty),
                _ => false,
            }
        }
        match serde_json::to_value(&m.contracts) {
            Ok(v) => !empty(&v),
            // Cannot tell: err toward enforcing, the pre-0.15 behaviour.
            Err(_) => true,
        }
    }

    /// Phase 43.2 source: the checked SystemModel (replay `--model`).
    /// Model keys are already launch-side FQNs (reconciled at resolve
    /// time), so no bare-name fallback is needed here.
    pub fn from_model(m: &model::SystemModel) -> Self {
        let mut view = ContractView::default();
        let m_modes = m.contracts.modes.clone();
        let m_functions = m.contracts.functions.clone();

        for (fqn, w) in &m.structure.topics {
            let contract = m.contracts.topics.get(fqn);
            view.topics.insert(
                fqn.clone(),
                TopicView {
                    msg_type: w.msg_type.clone(),
                    publishers: w.publishers.clone(),
                    subscribers: w.subscribers.clone(),
                    max_drop_rate: contract
                        .and_then(|c| c.drop.as_ref())
                        .and_then(|d| d.max_drop_rate),
                },
            );
        }
        view.externals = m.contracts.externals.keys().cloned().collect();
        let c = &m.contracts;
        let endpoint_keys = c
            .pub_endpoints
            .keys()
            .chain(c.sub_endpoints.keys())
            .chain(c.srv_endpoints.keys())
            .chain(c.cli_endpoints.keys())
            .chain(c.action_cli_endpoints.keys())
            .chain(c.node_paths.keys());
        view.contracted_nodes = endpoint_keys
            .filter_map(|k| key_node(k))
            .map(str::to_string)
            .chain(c.node_concurrency.keys().cloned())
            .chain(c.node_params.keys().cloned())
            .chain(
                m.structure
                    .nodes
                    .iter()
                    .filter(|(_, n)| {
                        m.structure
                            .scopes
                            .get(&n.scope)
                            .is_some_and(|s| s.manifest.is_some())
                    })
                    .map(|(fqn, _)| fqn.clone()),
            )
            .collect();

        for (ep_ref, c) in &m.contracts.pub_endpoints {
            if let Some(min) = c.min_rate_hz {
                view.pub_min_rate.insert(ep_ref.clone(), min);
            }
        }
        for (ep_ref, c) in &m.contracts.sub_endpoints {
            if let Some(max) = c.max_age_ms {
                view.sub_max_age.insert(ep_ref.clone(), max);
            }
        }
        for (fqn, n) in &m.structure.nodes {
            if n.lifecycle {
                view.lifecycle_nodes.insert(fqn.clone());
            }
        }
        for (key, p) in &m.contracts.scope_paths {
            view.scope_paths.push(ScopePathView {
                name: key.clone(),
                input_topics: p.input.clone(),
                output_topics: p.output.clone(),
                max_latency_ms: p.max_latency_ms,
            });
        }
        for (key, h) in &m.contracts.hazards {
            view.hazard_classes.insert(key.clone(), h.classes());
            let guards: Vec<String> = h.monitors.iter().flat_map(|g| g.members.clone()).collect();
            let sinks: Vec<String> = hazard_sinks(&m.contracts, h);
            // rlm v0.3.18 (#6): the levels it runs that state a deadline.
            let deadlines = level_deadlines(m, h);
            if !deadlines.is_empty() {
                view.level_deadlines.insert(key.clone(), deadlines);
            }
            let settle_ms = m
                .contracts
                .node_paths
                .values()
                .filter_map(|p| {
                    let ss = p.safe_state.as_ref()?;
                    let on_sink = sinks.iter().any(|t| {
                        m.structure
                            .topics
                            .get(t)
                            .is_some_and(|w| w.publishers.contains(&ss.emits))
                    });
                    if !on_sink {
                        return None;
                    }
                    // Phase 91 E10: a derived settle (a braking profile's
                    // among them) is a plain number on the model, response
                    // included, the largest over the hazards that end
                    // here. It used to be recomputed here per hazard, from
                    // the profile's parameters, without the response.
                    ss.settle_ms
                })
                .fold(None, |a: Option<f64>, v| Some(a.map_or(v, |x| x.max(v))));
            let (mut silence_ms, mut via_diagnostics) = (None::<f64>, false);
            for g in &guards {
                let Some(w) = m.structure.topics.get(g) else {
                    continue;
                };
                for sub in &w.subscribers {
                    let Some(c) = m.contracts.sub_endpoints.get(sub) else {
                        continue;
                    };
                    let Some(ov) = &c.on_violation else {
                        continue;
                    };
                    for a in [
                        c.max_age_ms,
                        c.qos.as_ref().and_then(|q| q.lease_duration_ms),
                    ]
                    .into_iter()
                    .flatten()
                    {
                        silence_ms = Some(silence_ms.map_or(a, |s: f64| s.min(a)));
                    }
                    via_diagnostics |= ov.mechanism == model::DetectMechanism::Diagnostics;
                }
            }
            let off_host_sinks = sinks
                .iter()
                .filter(|t| {
                    matches!(
                        m.contracts.externals.get(*t),
                        Some(model::ExternalSide::Pub | model::ExternalSide::Both)
                    )
                })
                .cloned()
                .collect();
            view.hazards.push(HazardWatch {
                key: key.clone(),
                guards,
                sinks,
                ftti_ms: h.ftti_ms,
                settle_ms,
                silence_ms,
                via_diagnostics,
                off_host_sinks,
            });
        }
        // Phase 102 M2: the values the interceptor reads, the functions lost
        // by them, and what each level commands.
        view.fields = crate::interception::fields::declared_fields(m)
            .into_iter()
            .map(|f| (f.topic, f.path))
            .collect();
        // Phase 103 D1: a constant the resolver could not resolve (its
        // `.msg` was not on the ament index when the model was built) cannot
        // be judged. Say so once, naming it, rather than read it as held.
        for (site, p) in unresolved_constants(m) {
            if let model::PredicateValueContract::Read { read } = &p.value {
                tracing::warn!(
                    "{site}: the predicate on '{}' reads `{read}`, which the model carries \
                     unevaluated (the launch tree gave it no value when the model was built); \
                     it is not judged at run time",
                    p.field
                );
                continue;
            }
            tracing::warn!(
                "{site}: the predicate on '{}' names the constant {}, whose value the model \
                 does not carry (its message definition was not found when the model was \
                 built); it is not judged at run time. Resolve with the workspace that \
                 provides the message sourced",
                p.field,
                match &p.value {
                    model::PredicateValueContract::Constant { constant, .. } => constant.as_str(),
                    _ => "?",
                }
            );
        }
        for (key, f) in &m_functions {
            if let Some(when) = &f.when {
                view.value_fns.insert(
                    key.clone(),
                    ValueFn {
                        topics: f.members.clone(),
                        when: when.clone(),
                    },
                );
            }
        }
        for (key, mc) in &m_modes {
            let requires = mc
                .requires
                .iter()
                .map(|r| {
                    m_functions
                        .get(r)
                        .map(|f| RequiredFn {
                            members: f.members.clone(),
                            all_of: f.all_of,
                            m: f.m,
                            value_fn: f.when.as_ref().map(|_| r.clone()),
                        })
                        .unwrap_or_else(|| RequiredFn::topics(vec![r.clone()]))
                })
                .collect();
            // The level's derived route, over every hazard of its scope that
            // reaches it. A mode key is `<scope>/<name>`; a level route names
            // the bare level.
            let (scope, name) = key.rsplit_once('/').unwrap_or(("", key.as_str()));
            let bound_ms = m
                .contracts
                .hazards
                .iter()
                .filter(|(hk, _)| hk.rsplit_once('/').map(|(s, _)| s) == Some(scope))
                .flat_map(|(_, h)| h.degradation_levels.iter())
                .filter(|l| l.level == name)
                .filter_map(|l| l.route_ms)
                .fold(None, |acc: Option<f64>, v| {
                    Some(acc.map_or(v, |a| a.max(v)))
                });
            for topic in mc.commands.keys() {
                let handled =
                    m.contracts
                        .topics
                        .get(topic)
                        .is_some_and(|t| t.on_silence_after_ms.is_some())
                        || m.structure.topics.get(topic).is_some_and(|w| {
                            w.subscribers.iter().any(|ep| {
                                m.contracts.sub_endpoints.get(ep).is_some_and(|s| {
                                    s.max_age_ms.is_some() && s.on_violation.is_some()
                                })
                            })
                        });
                if handled {
                    view.silence_handled.insert(topic.clone());
                }
            }
            let required = |r: &String| {
                m_functions
                    .get(r)
                    .map(|f| RequiredFn {
                        members: f.members.clone(),
                        all_of: f.all_of,
                        m: f.m,
                        value_fn: f.when.as_ref().map(|_| r.clone()),
                    })
                    .unwrap_or_else(|| RequiredFn::topics(vec![r.clone()]))
            };
            // rlm v0.3.18 (#5): an exit to a level of a fallback sequence is
            // a re-entry the observer follows.
            let exit = mc.exit.as_ref().and_then(|e| {
                m_modes
                    .values()
                    .any(|x| x.fallback.contains(&e.to))
                    .then(|| ExitWatch {
                        holds: required(&e.on),
                        to: e.to.clone(),
                    })
            });
            view.modes.push(ModeWatch {
                key: key.clone(),
                requires,
                fallback: mc.fallback.clone(),
                requires_lost: mc.requires_lost.iter().map(required).collect(),
                exit,
                commands: mc.commands.clone(),
                bound_ms,
            });
        }
        view
    }
}

/// Every predicate whose constant the model carries without a value (phase
/// 103 D1), with where it is.
pub(crate) fn unresolved_constants(
    m: &model::SystemModel,
) -> Vec<(String, &model::PredicateContract)> {
    let unresolved = |p: &model::PredicateContract| {
        matches!(
            p.value,
            model::PredicateValueContract::Constant { value: None, .. }
                | model::PredicateValueContract::Read { .. }
        )
    };
    let mut out = Vec::new();
    for (k, f) in &m.contracts.functions {
        if let Some(p) = f.when.as_ref().filter(|p| unresolved(p)) {
            out.push((format!("function '{k}'"), p));
        }
    }
    for (k, h) in &m.contracts.hazards {
        if let Some(p) = h.when.as_ref().filter(|p| unresolved(p)) {
            out.push((format!("hazard '{k}'"), p));
        }
    }
    for (k, md) in &m.contracts.modes {
        for (t, ps) in &md.commands {
            for p in ps.iter().filter(|p| unresolved(p)) {
                out.push((format!("mode '{k}' commands on '{t}'"), p));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    /// Phase 103 D4: a hazard that enters mode M reacts by M's fallback
    /// sequence, every level in order by its own `reaction:`. M's own
    /// `reaction:` is not part of it. The observer used to read only the
    /// last level.
    #[test]
    fn a_mode_reaction_is_every_level_of_its_fallback() {
        let mut c = model::Contracts::default();
        let mode = |reaction: Option<&str>, fallback: &[&str]| model::ModeContract {
            reaction: reaction.map(str::to_string),
            fallback: fallback.iter().map(|s| s.to_string()).collect(),
            ..Default::default()
        };
        c.modes
            .insert("engaged".into(), mode(Some("own"), &["tor", "stop"]));
        c.modes.insert("tor".into(), mode(Some("tor_chain"), &[]));
        c.modes.insert("stop".into(), mode(Some("stop_chain"), &[]));
        assert_eq!(
            reaction_paths(&c, "engaged"),
            vec!["tor_chain".to_string(), "stop_chain".to_string()]
        );
        assert_eq!(reaction_paths(&c, "a_chain"), vec!["a_chain".to_string()]);
        for (k, out) in [("tor_chain", "/tor"), ("stop_chain", "/brake")] {
            c.scope_paths.insert(
                k.into(),
                model::PathContract {
                    output: vec![out.to_string()],
                    ..Default::default()
                },
            );
        }
        assert_eq!(
            reaction_sinks(&c, Some(&"engaged".to_string())),
            vec!["/tor".to_string(), "/brake".to_string()]
        );
    }

    use super::*;

    /// The two constructors must agree: a model carrying the same facts as
    /// a manifest index yields the same view the engine rules read.
    #[test]
    fn from_model_matches_from_manifest_index() {
        // Manifest-index side.
        let mut index = ManifestIndex::default();
        index.topics.insert(
            "/a/points".to_string(),
            ros_launch_resolve::ros::manifest_loader::ResolvedTopic {
                derived_from_remaps: false,
                fqn: "/a/points".to_string(),
                msg_type: "sensor_msgs/msg/PointCloud2".to_string(),
                qos: None,
                publishers: vec!["/a/sensor/points".to_string()],
                subscribers: vec!["/a/filter/points".to_string()],
                rate_hz: Some(10.0),
                derived_rate_hz: None,
                drop: Some(ros_launch_manifest_types::DropSpec {
                    max_count: Some(ros_launch_manifest_types::spelled(
                        ros_launch_manifest_types::field_table::Context::Drop,
                        "max_count",
                        ros_launch_manifest_types::DropCount { n: 1, w: 20 },
                    )),
                    max_consecutive: Some(ros_launch_manifest_types::spelled(
                        ros_launch_manifest_types::field_table::Context::Drop,
                        "max_consecutive",
                        3u32,
                    )),
                }),
                scope_ids: vec![0],
                on_silence_after_ms: None,
            },
        );
        let mut manifest = ros_launch_manifest_types::Manifest::default();
        let mut node = ros_launch_manifest_types::NodeDecl {
            lifecycle: Some(true),
            ..Default::default()
        };
        node.publishers.insert(
            "points".to_string(),
            ros_launch_manifest_types::EndpointProps {
                min_rate_hz: Some(ros_launch_manifest_types::spelled(
                    ros_launch_manifest_types::field_table::Context::Endpoint,
                    "min_rate",
                    10.0,
                )),
                ..Default::default()
            },
        );
        manifest.nodes.insert("sensor".to_string(), node);
        let mut sub_node = ros_launch_manifest_types::NodeDecl::default();
        sub_node.subscribers.insert(
            "points".to_string(),
            ros_launch_manifest_types::EndpointProps {
                max_age: Some(ros_launch_manifest_types::spelled(
                    ros_launch_manifest_types::field_table::Context::Endpoint,
                    "max_age",
                    ros_launch_manifest_types::duration::Duration::from_millis_f64(50.0),
                )),
                ..Default::default()
            },
        );
        manifest.nodes.insert("filter".to_string(), sub_node);
        index.manifests.insert(
            0,
            ros_launch_resolve::ros::manifest_loader::ResolvedManifest {
                scope_id: 0,
                pkg: None,
                file: "x.yaml".to_string(),
                ns: "/a".to_string(),
                channel: ros_launch_resolve::ros::manifest_loader::ContractChannel::Provider,
                contract_path: std::path::PathBuf::from("x.yaml"),
                manifest,
                source: String::new(),
                diagnostics: vec![],
            },
        );

        // Model side carrying the same facts.
        let mut model = ros_launch_manifest_model::SystemModel::default();
        model.structure.topics.insert(
            "/a/points".to_string(),
            ros_launch_manifest_model::TopicWiring {
                msg_type: "sensor_msgs/msg/PointCloud2".to_string(),
                publishers: vec!["/a/sensor/points".to_string()],
                subscribers: vec!["/a/filter/points".to_string()],
            },
        );
        model.contracts.topics.insert(
            "/a/points".to_string(),
            ros_launch_manifest_model::TopicContract {
                rate_hz: Some(10.0),
                drop: Some(ros_launch_manifest_model::DropContract {
                    max_drop_rate: Some(0.05),
                    max_consecutive: Some(3),
                }),
                ..Default::default()
            },
        );
        model.contracts.pub_endpoints.insert(
            "/a/sensor/points".to_string(),
            ros_launch_manifest_model::PubContract {
                min_rate_hz: Some(10.0),
                ..Default::default()
            },
        );
        model.contracts.sub_endpoints.insert(
            "/a/filter/points".to_string(),
            ros_launch_manifest_model::SubContract {
                max_age_ms: Some(50.0),
                ..Default::default()
            },
        );
        model.structure.nodes.insert(
            "/a/sensor".to_string(),
            ros_launch_manifest_model::NodeInstance {
                scope: "/a".to_string(),
                lifecycle: true,
                ..Default::default()
            },
        );

        let vi = ContractView::from_manifest_index(&index);
        let vm = ContractView::from_model(&model);

        assert_eq!(
            vi.topics["/a/points"].msg_type,
            vm.topics["/a/points"].msg_type
        );
        assert_eq!(
            vi.topics["/a/points"].publishers,
            vm.topics["/a/points"].publishers
        );
        assert_eq!(
            vi.topics["/a/points"].max_drop_rate,
            vm.topics["/a/points"].max_drop_rate
        );
        assert_eq!(
            vi.pub_min_rate.get("/a/sensor/points"),
            vm.pub_min_rate.get("/a/sensor/points")
        );
        assert_eq!(
            vi.sub_max_age.get("/a/filter/points"),
            vm.sub_max_age.get("/a/filter/points")
        );
        assert!(vi.lifecycle_nodes.contains("/a/sensor"));
        assert!(vm.lifecycle_nodes.contains("/a/sensor"));
    }
}
