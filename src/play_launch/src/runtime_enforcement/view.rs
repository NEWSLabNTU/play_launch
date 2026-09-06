//! Phase 43.2 — the narrow, source-agnostic view of contract data the
//! rule engine actually reads. Two constructors: `from_manifest_index`
//! (legacy replay path, behavior-preserving) and `from_model` (replay
//! `--model`, where the checked SystemModel is the single source).
//!
//! Precomputing the `"node FQN/endpoint"` → contract maps here also
//! removes the per-event manifest-tree rescans the rules used to do.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use ros_launch_manifest_model as model;

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
                        .and_then(|d| d.max_count.as_ref())
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
                    if let Some(min) = props.min_rate_hz {
                        view.pub_min_rate.insert(format!("{node_fqn}/{ep}"), min);
                    }
                }
                for (ep, props) in &node.subscribers {
                    if let Some(max) = props.max_age.map(|d| d.as_millis_f64()) {
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
                max_latency_ms: sp.path.max_latency.map(|d| d.as_millis_f64()),
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
                        .then_some(ss.settle.map(|d| d.as_millis_f64()))
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
                    for a in [
                        props.and_then(|p| p.max_age).map(|d| d.as_millis_f64()),
                        lease,
                    ]
                    .into_iter()
                    .flatten()
                    {
                        silence_ms = Some(silence_ms.map_or(a, |s: f64| s.min(a)));
                    }
                    via_diagnostics |=
                        ov.mechanism == ros_launch_manifest_types::DetectMechanism::Diagnostics;
                }
            }
            view.hazards.push(HazardWatch {
                key: h.name.clone(),
                guards,
                sinks,
                ftti_ms: h.decl.ftti.map(|d| d.as_millis_f64()),
                settle_ms,
                silence_ms,
                via_diagnostics,
            });
        }
        view
    }

    /// Phase 43.2 source: the checked SystemModel (replay `--model`).
    /// Model keys are already launch-side FQNs (reconciled at resolve
    /// time), so no bare-name fallback is needed here.
    pub fn from_model(m: &model::SystemModel) -> Self {
        let mut view = ContractView::default();

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
            let guards: Vec<String> = h.guards.iter().flat_map(|g| g.members.clone()).collect();
            let sinks: Vec<String> = h
                .reaction
                .as_ref()
                .and_then(|r| m.contracts.scope_paths.get(r))
                .map(|p| p.output.clone())
                .unwrap_or_default();
            let settle_ms = m
                .contracts
                .node_paths
                .values()
                .filter_map(|p| {
                    let ss = p.safe_state.as_ref()?;
                    sinks
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
            view.hazards.push(HazardWatch {
                key: key.clone(),
                guards,
                sinks,
                ftti_ms: h.ftti_ms,
                settle_ms,
                silence_ms,
                via_diagnostics,
            });
        }
        view
    }
}

#[cfg(test)]
mod tests {
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
                fqn: "/a/points".to_string(),
                msg_type: "sensor_msgs/msg/PointCloud2".to_string(),
                qos: None,
                publishers: vec!["/a/sensor/points".to_string()],
                subscribers: vec!["/a/filter/points".to_string()],
                rate_hz: Some(10.0),
                derived_rate_hz: None,
                max_transport_ms: None,
                drop: Some(ros_launch_manifest_types::DropSpec {
                    max_count: Some(ros_launch_manifest_types::DropCount { n: 1, w: 20 }),
                    max_consecutive: Some(3),
                }),
                scope_ids: vec![0],
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
                min_rate_hz: Some(10.0),
                ..Default::default()
            },
        );
        manifest.nodes.insert("sensor".to_string(), node);
        let mut sub_node = ros_launch_manifest_types::NodeDecl::default();
        sub_node.subscribers.insert(
            "points".to_string(),
            ros_launch_manifest_types::EndpointProps {
                max_age: Some(ros_launch_manifest_types::duration::Duration::from_millis_f64(50.0)),
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
