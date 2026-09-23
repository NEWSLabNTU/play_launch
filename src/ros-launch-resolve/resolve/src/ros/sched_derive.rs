//! Build a `SchedMapper` [`MapperInput`] for a launch tree: resolve its
//! model (structure and contracts, no execution layer) and hand it to
//! `ros_launch_manifest_derive::mapper_input_from_model`, the derivation
//! nano-ros hands its realizer too (design issue #52, phase 78). What the
//! mapper learns about a node, its fastest timer as `rate_hz`, the tightest
//! path budget or `max_response` as its deadline, the effective
//! criticality, its paths and its chains, is the crate's rule applied to
//! this repository's model. This file adds only the facts a platform file
//! carries and the model does not (per-node budgets), and the one spelling
//! the `manual` mapper's `scope =` selector still needs.
//!
//! Until phase 78 W3 this file held a second derivation, over the launch
//! dump and the contract index directly; W2's parity gate held it to the
//! crate on every contract fixture, and W3 deleted it. The `tests` module
//! now pins the crate's plan over every fixture to a committed snapshot.

use std::collections::{BTreeMap, BTreeSet};

use ros_launch_manifest_derive::{DeriveFacts, DeriveReport, mapper_input_from_model};
use ros_launch_manifest_sched::{Criticality, MapperInput, SystemSched};

use crate::ros::{launch_dump::LaunchDump, manifest_loader::ManifestIndex};

/// Build the mapper's input the way both consumers do since phase 78
/// (design issue #52): resolve the model of `dump` plus `index` and hand
/// `ros_launch_manifest_derive::mapper_input_from_model` that model and the
/// platform file's per-node budgets as [`DeriveFacts`]. This is the one
/// entry `derive_sched_plan` calls; the [`DeriveReport`] names what the
/// model could not say.
///
/// `legacy` is set afterwards, as the crate documents: only the `.toml`
/// bridge's `manual` mapper reads it.
pub fn mapper_input_via_model(
    dump: &LaunchDump,
    index: Option<&ManifestIndex>,
    legacy: Option<SystemSched>,
    budgets: &BTreeMap<String, u64>,
) -> (MapperInput, DeriveReport) {
    let model = pre_sched_model(dump, index);
    let (mut input, report) = mapper_input_from_model(&model, &derive_facts_from_budgets(budgets));
    input.legacy = legacy;
    // `MapperNode::scope` carries the node's NAMESPACE, which is what the
    // `manual` mapper's `[[assign]] scope = "/perception"` selector matches
    // (`ros_launch_manifest_sched::resolve::scope_selector_matches`). It used
    // to be the model's FILE-SCOPE key, and this function patched it back on
    // the way out; rlm v0.1.40 (issue 52 R5) derives it in the crate, so the
    // patch is gone and the derivation is read through unchanged.
    (input, report)
}

/// The model the derivation reads: structure and contracts of `dump` under
/// `index`, with no execution layer. `derive_sched_plan` builds it before the
/// plan exists, and the plan is what the execution layer is made of; the
/// caller's full model (provenance, args, execution) is built afterwards by
/// `build_checked_model`, from the same inputs. A run with no contract index
/// (`play_launch run`'s synthetic single-node dump) derives from an empty
/// one, which is the model `resolve` emits for a tree with no contracts.
pub fn pre_sched_model(
    dump: &LaunchDump,
    index: Option<&ManifestIndex>,
) -> ros_launch_manifest_model::SystemModel {
    let empty;
    let index = match index {
        Some(index) => index,
        None => {
            empty = ManifestIndex::default();
            &empty
        }
    };
    super::model_builder::build_system_model(
        dump,
        index,
        None,
        BTreeMap::new(),
        &BTreeSet::new(),
        None,
    )
}

/// The platform file's `budget` overrides as the crate's per-node fact, in
/// milliseconds. `budgets` already carries every spelling a selector may
/// take (`posix_budgets`: the selector, its slashed form and its bare name),
/// and the crate looks a node up by FQN and then by bare name, the fallback
/// `[[assign]].nodes` selectors have always had. No per-path fact exists
/// here: a platform file speaks of nodes.
pub fn derive_facts_from_budgets(budgets: &BTreeMap<String, u64>) -> DeriveFacts {
    DeriveFacts {
        path_exec_ms: BTreeMap::new(),
        node_exec_ms: budgets
            .iter()
            .map(|(selector, us)| (selector.clone(), *us as f64 / 1000.0))
            .collect(),
    }
}

/// Translate the contract's `miss:` into the mapper's mirror of it.
pub(crate) fn convert_miss(
    m: &ros_launch_manifest_types::MissSpec,
) -> ros_launch_manifest_sched::MapperMiss {
    use ros_launch_manifest_sched::{MapperMiss, MapperMissAction};
    use ros_launch_manifest_types::MissAction as A;
    MapperMiss {
        tolerate_n: m.tolerate.as_ref().map(|c| c.n),
        tolerate_w: m.tolerate.as_ref().map(|c| c.w),
        consecutive: m.consecutive,
        action: m.action.map(|a| match a {
            A::Continue => MapperMissAction::Continue,
            A::SkipNext => MapperMissAction::SkipNext,
            A::Abort => MapperMissAction::Abort,
        }),
    }
}

/// The label's bucket, for comparing a declaration against the derivation.
pub(crate) fn parse_criticality_label(raw: &str) -> Option<Criticality> {
    parse_criticality(raw)
}

/// Case-insensitive `high`/`medium`/`low` -> [`Criticality`] parse. The
/// model builder reads it through [`parse_criticality_label`] to lower the
/// advisory label; the crate's derivation then buckets what the model
/// carries.
fn parse_criticality(raw: &str) -> Option<Criticality> {
    match raw.to_ascii_lowercase().as_str() {
        "high" => Some(Criticality::High),
        "medium" => Some(Criticality::Medium),
        "low" => Some(Criticality::Low),
        other => {
            tracing::debug!("sched: unrecognized criticality `{other}` — ignored");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::manifest_loader::{
        ContractChannel, ResolvedManifest, ResolvedNodePath, ResolvedScopePath, ResolvedTopic,
    };
    use ros_launch_manifest_sched::{
        ChainElement, EffectiveTrigger, RankedPlan, ResolvedChain, chain_aware_rank,
    };
    use ros_launch_manifest_types::{Manifest, NodeDecl, PathDecl};
    use std::{
        collections::BTreeMap,
        path::{Path, PathBuf},
    };

    fn dump_with_two_nodes() -> LaunchDump {
        let json = serde_json::json!({
            "node": [
                {
                    "executable": "talker",
                    "name": "talker",
                    "exec_name": "talker",
                    "params_files": [],
                    "cmd": [],
                    "scope": 0
                },
                {
                    "executable": "listener",
                    "name": "listener",
                    "exec_name": "listener",
                    "params_files": [],
                    "cmd": [],
                    "scope": 0
                }
            ],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {
                    "id": 0,
                    "ns": "/",
                    "parent": null,
                    "origin": {"file": "manifest.launch.xml", "path": "/nowhere/manifest.launch.xml"}
                }
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    fn empty_resolved_manifest(scope_id: usize, manifest: Manifest) -> ResolvedManifest {
        ResolvedManifest {
            scope_id,
            pkg: None,
            file: "manifest.launch.xml".to_string(),
            ns: "/".to_string(),
            channel: ContractChannel::Provider,
            contract_path: std::path::PathBuf::new(),
            manifest,
            source: String::new(),
            diagnostics: vec![],
        }
    }

    #[test]
    fn no_index_gives_bare_records() {
        let dump = dump_with_two_nodes();
        let (input, _) = mapper_input_via_model(&dump, None, None, &BTreeMap::new());
        assert_eq!(input.nodes.len(), 2);
        for n in &input.nodes {
            assert_eq!(n.rate_hz, None);
            assert_eq!(n.deadline_us, None);
            assert_eq!(n.criticality, None);
            assert_eq!(n.path_budget_ms, None);
        }
        assert!(input.legacy.is_none());
    }

    /// Phase 78: a node's rate is its fastest timer trigger. The topic's
    /// authored `rate_hz` (100), its derived rate (80) and the publisher's
    /// `min_rate_hz` (30) are promises no mapper reads; before this the max
    /// of the three, 100, was the rate, and a node with no timer at all
    /// ranked on a promise.
    #[test]
    fn rate_hz_is_the_fastest_timer_trigger_and_never_a_promise() {
        let dump = dump_with_two_nodes();

        let tick = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 50.0 }),
            output: vec!["chatter".to_string()],
            ..Default::default()
        };
        let slow = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 5.0 }),
            output: vec!["chatter".to_string()],
            ..Default::default()
        };
        let mut nodes = BTreeMap::new();
        nodes.insert(
            "talker".to_string(),
            NodeDecl {
                publishers: {
                    let mut m = BTreeMap::new();
                    m.insert(
                        "chatter".to_string(),
                        ros_launch_manifest_types::EndpointProps {
                            min_rate_hz: Some(30.0),
                            ..Default::default()
                        },
                    );
                    m
                },
                paths: BTreeMap::from([
                    ("slow".to_string(), slow.clone()),
                    ("tick".to_string(), tick.clone()),
                ]),
                ..Default::default()
            },
        );
        // The listener publishes on a topic that promises a rate, and has no
        // timer: no rate.
        nodes.insert(
            "listener".to_string(),
            NodeDecl {
                publishers: {
                    let mut m = BTreeMap::new();
                    m.insert(
                        "echo".to_string(),
                        ros_launch_manifest_types::EndpointProps {
                            min_rate_hz: Some(30.0),
                            ..Default::default()
                        },
                    );
                    m
                },
                ..Default::default()
            },
        );
        let manifest = Manifest {
            version: 1,
            nodes,
            ..Default::default()
        };

        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, empty_resolved_manifest(0, manifest));
        for (name, path) in [("slow", slow), ("tick", tick)] {
            index.node_paths.push(ResolvedNodePath {
                node_fqn: "/talker".to_string(),
                path_name: name.to_string(),
                path,
                scope_id: 0,
            });
        }
        for (fqn, publisher) in [("/chatter", "/talker/chatter"), ("/echo", "/listener/echo")] {
            index.topics.insert(
                fqn.to_string(),
                ResolvedTopic {
                    derived_from_remaps: false,
                    fqn: fqn.to_string(),
                    msg_type: "std_msgs/msg/String".to_string(),
                    qos: None,
                    publishers: vec![publisher.to_string()],
                    subscribers: vec![],
                    rate_hz: Some(100.0),
                    derived_rate_hz: Some(80.0),
                    max_transport_ms: None,
                    drop: None,
                    scope_ids: vec![0],
                },
            );
        }

        let (input, _) = derived("promised rates", &dump, &index, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(
            talker.rate_hz,
            Some(50.0),
            "the fastest timer, not the promise"
        );
        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.rate_hz, None, "a promise alone is no rate");
    }

    #[test]
    fn deadline_and_path_budget_use_tightest_path() {
        let dump = dump_with_two_nodes();

        let mut index = ManifestIndex::default();
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "loose".to_string(),
            path: PathDecl {
                max_latency: Some(
                    ros_launch_manifest_types::duration::Duration::from_millis_f64(50.0),
                ),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "tight".to_string(),
            path: PathDecl {
                max_latency: Some(
                    ros_launch_manifest_types::duration::Duration::from_millis_f64(10.0),
                ),
                ..Default::default()
            },
            scope_id: 0,
        });

        let (input, _) = mapper_input_via_model(&dump, Some(&index), None, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(talker.path_budget_ms, Some(10.0));
        assert_eq!(talker.deadline_us, Some(10_000));

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.path_budget_ms, None);
        assert_eq!(listener.deadline_us, None);
    }

    #[test]
    fn criticality_is_case_insensitive_and_ignores_unrecognized() {
        let dump = dump_with_two_nodes();

        let mut nodes = BTreeMap::new();
        nodes.insert(
            "talker".to_string(),
            NodeDecl {
                criticality: Some("HIGH".to_string()),
                ..Default::default()
            },
        );
        nodes.insert(
            "listener".to_string(),
            NodeDecl {
                criticality: Some("urgent".to_string()), // not a recognized value
                ..Default::default()
            },
        );
        let manifest = Manifest {
            version: 1,
            nodes,
            ..Default::default()
        };

        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, empty_resolved_manifest(0, manifest));

        let (input, _) = mapper_input_via_model(&dump, Some(&index), None, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(talker.criticality, Some(Criticality::High));

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.criticality, None);
    }

    // ── Phase 44.4: per-path extraction + chain resolution ──

    /// A declared budget reaches `MapperPath::exec_ms` — the slot phase 58's
    /// design pass named as the blocker for proportional deadline
    /// decomposition, and which was hard-coded `None` behind a comment that
    /// went stale when phase 60 made cost authorable.
    ///
    /// And it stays absent where it cannot be attributed. A budget is per NODE
    /// while this field is per PATH, so the two line up only for a single-path
    /// node; `play_launch measure` documents its own emitted budget as the SUM
    /// of a node's per-path maxima, so giving that sum to one of several paths
    /// would overstate it. Absent is a value here, not a gap — the feasibility
    /// diagnostic reports it as "incomplete evidence" rather than as feasible.
    #[test]
    fn a_declared_budget_reaches_a_single_path_and_no_further() {
        let dump = dump_with_two_nodes();
        let budgets = BTreeMap::from([
            ("/talker".to_string(), 3_500u64),
            ("/listener".to_string(), 900u64),
        ]);

        // `/talker` gets ONE path — the budget is unambiguously its cost.
        let mut index = ManifestIndex::default();
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "publish".to_string(),
            path: PathDecl {
                trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 10.0 }),
                output: vec!["out_ep".to_string()],
                ..Default::default()
            },
            scope_id: 0,
        });
        // `/listener` gets TWO — the split is unknown, so neither may claim it.
        for name in ["a", "b"] {
            index.node_paths.push(ResolvedNodePath {
                node_fqn: "/listener".to_string(),
                path_name: name.to_string(),
                path: PathDecl {
                    trigger: Some(ros_launch_manifest_types::Trigger::Input(vec![
                        "out_ep".to_string(),
                    ])),
                    ..Default::default()
                },
                scope_id: 0,
            });
        }

        let (input, _) = mapper_input_via_model(&dump, Some(&index), None, &budgets);

        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(
            talker.paths[0].exec_ms,
            Some(3.5),
            "a single-path node's declared budget IS that path's cost"
        );

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.paths.len(), 2);
        for path in &listener.paths {
            assert_eq!(
                path.exec_ms, None,
                "a multi-path node's budget must not be attributed to any one path"
            );
        }
    }

    #[test]
    fn paths_carry_the_effective_trigger_endpoints_as_qualified_refs() {
        let dump = dump_with_two_nodes();

        let mut index = ManifestIndex::default();
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "publish".to_string(),
            path: PathDecl {
                // Explicit `trigger: { input: [...] }` form — W2's bugfix
                // rule: raw `path.input` is empty here, endpoint names only
                // live inside the effective trigger.
                trigger: Some(ros_launch_manifest_types::Trigger::Input(vec![
                    "in_ep".to_string(),
                ])),
                output: vec!["out_ep".to_string()],
                max_latency: Some(
                    ros_launch_manifest_types::duration::Duration::from_millis_f64(12.5),
                ),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "tick".to_string(),
            path: PathDecl {
                trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 20.0 }),
                ..Default::default()
            },
            scope_id: 0,
        });

        let (input, _) = mapper_input_via_model(&dump, Some(&index), None, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(talker.paths.len(), 2);

        // Endpoints are qualified refs, the model's spelling (phase 78 W2).
        let publish = talker.paths.iter().find(|p| p.name == "publish").unwrap();
        assert_eq!(publish.inputs, vec!["/talker/in_ep".to_string()]);
        assert_eq!(publish.outputs, vec!["/talker/out_ep".to_string()]);
        assert_eq!(publish.max_latency_ms, Some(12.5));
        assert_eq!(
            publish.effective_trigger,
            EffectiveTrigger::Input(vec!["/talker/in_ep".to_string()])
        );

        let tick = talker.paths.iter().find(|p| p.name == "tick").unwrap();
        assert_eq!(
            tick.effective_trigger,
            EffectiveTrigger::Timer { rate_hz: 20.0 }
        );
        assert!(tick.inputs.is_empty());

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert!(listener.paths.is_empty());
    }

    /// Hand-built index: a two-node chain where one path is a Timer
    /// (Boundary) and the other Input (Segment), with distinct node
    /// criticalities. `/talker/tick` declares `max_latency_ms: 2.0` and NO
    /// budget, which is what lets the cost tests below tell a declared
    /// deadline apart from a declared cost.
    fn chain_index_for_cost_tests() -> ManifestIndex {
        // (Boundary) and the other is Input (Segment), with distinct node
        // criticalities — asserts both the Boundary/Segment split and the
        // "criticality = max over member nodes" rule.
        let ms = ros_launch_manifest_types::duration::Duration::from_millis_f64;
        let tick = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 50.0 }),
            output: vec!["chatter".to_string()],
            max_latency: Some(ms(2.0)),
            ..Default::default()
        };
        let react = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Input(vec![
                "chatter".to_string(),
            ])),
            output: vec!["reaction".to_string()],
            max_latency: Some(ms(8.0)),
            ..Default::default()
        };

        let mut nodes = BTreeMap::new();
        nodes.insert(
            "talker".to_string(),
            NodeDecl {
                criticality: Some("low".to_string()),
                // The model reads a node's paths off `index.node_paths` and
                // its label and concurrency off the manifest's `NodeDecl`;
                // the loader fills the first from the second, and this
                // fixture declares both the way the loader would.
                paths: BTreeMap::from([("tick".to_string(), tick.clone())]),
                ..Default::default()
            },
        );
        nodes.insert(
            "listener".to_string(),
            NodeDecl {
                criticality: Some("high".to_string()),
                paths: BTreeMap::from([("react".to_string(), react.clone())]),
                ..Default::default()
            },
        );
        let manifest = Manifest {
            version: 1,
            nodes,
            ..Default::default()
        };

        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, empty_resolved_manifest(0, manifest));
        // The requirement, stated as a scope path: two ends and a budget. The
        // route between them is derived from the trigger/output facts of the
        // two node paths below — which is the whole point of the spelling that
        // replaced `chains:`/`segments:`.
        index.scope_paths.push(ResolvedScopePath {
            scope_id: 0,
            path_name: "mixed_chain".to_string(),
            input_topics: vec!["/chatter".to_string()],
            output_topics: vec!["/reaction".to_string()],
            path: PathDecl {
                max_latency: Some(ms(100.0)),
                ..Default::default()
            },
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "tick".to_string(),
            path: tick,
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/listener".to_string(),
            path_name: "react".to_string(),
            path: react,
            scope_id: 0,
        });
        index.topics.insert(
            "/chatter".to_string(),
            ResolvedTopic {
                derived_from_remaps: false,
                fqn: "/chatter".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/talker/chatter".to_string()],
                subscribers: vec!["/listener/chatter".to_string()],
                rate_hz: None,
                derived_rate_hz: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );
        index.topics.insert(
            "/reaction".to_string(),
            ResolvedTopic {
                derived_from_remaps: false,
                fqn: "/reaction".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/listener/reaction".to_string()],
                subscribers: vec![],
                rate_hz: None,
                derived_rate_hz: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );

        index
    }

    /// The chains the crate resolves over this file's two-node dump under
    /// `index`, as `derive_sched_plan` sees them.
    fn chains_of(index: &ManifestIndex, budgets: &BTreeMap<String, u64>) -> Vec<ResolvedChain> {
        mapper_input_via_model(&dump_with_two_nodes(), Some(index), None, budgets)
            .0
            .chains
    }

    /// A declared budget — and only a declared budget — becomes a cost.
    #[test]
    fn a_declared_budget_becomes_the_boundary_cost() {
        let index = chain_index_for_cost_tests();
        let budgets = BTreeMap::from([("/talker".to_string(), 3_500u64)]);
        let chains = chains_of(&index, &budgets);
        let chain = chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain should resolve");
        match &chain.elements[0] {
            ChainElement::Boundary { exec_ms, .. } => {
                // 3500us declared -> 3.5ms cost. Note the unit change: the
                // platform file speaks microseconds, the chain math
                // milliseconds.
                assert_eq!(*exec_ms, Some(3.5));
            }
            other => panic!("expected a Boundary, got {other:?}"),
        }
    }

    /// Regression guard for the conflation this wave removed: a path's
    /// declared latency must NEVER reappear as its execution cost.
    ///
    /// The two differ by construction here — `max_latency_ms` is 2.0 and no
    /// budget is declared — so any future change that reaches for the deadline
    /// again fails loudly instead of silently making every feasibility verdict
    /// optimistic.
    #[test]
    fn a_declared_deadline_is_never_used_as_a_cost() {
        let index = chain_index_for_cost_tests();
        let chains = chains_of(&index, &BTreeMap::new());
        let chain = chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain should resolve");
        for element in &chain.elements {
            if let ChainElement::Boundary { exec_ms, node, .. } = element {
                assert_eq!(
                    *exec_ms, None,
                    "{node}: cost must be absent without a declared budget, never the deadline"
                );
            }
        }
    }

    #[test]
    fn a_derived_route_classifies_timer_as_boundary_and_criticality_is_max_of_members() {
        let index = chain_index_for_cost_tests();
        let chains = chains_of(&index, &BTreeMap::new());
        let chain = chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain should resolve");
        assert_eq!(chain.elements.len(), 2);
        match &chain.elements[0] {
            ChainElement::Boundary {
                node,
                path,
                period_ms,
                exec_ms,
            } => {
                assert_eq!(node, "/talker");
                assert_eq!(path, "tick");
                assert_eq!(*period_ms, 20.0); // 1000 / 50 Hz
                // No declared budget, so the cost is ABSENT. This assertion
                // used to read `Some(2.0)` — the path's `max_latency_ms`,
                // i.e. its DEADLINE, standing in for its cost. Absent and
                // zero and "the deadline" are three different answers, and
                // only the first is true here.
                assert_eq!(*exec_ms, None);
            }
            other => panic!("expected a Boundary, got {other:?}"),
        }
        match &chain.elements[1] {
            ChainElement::Segment {
                nodes_in_topo_order,
            } => {
                assert_eq!(
                    nodes_in_topo_order,
                    &vec![ros_launch_manifest_sched::SegmentNode {
                        node: "/listener".to_string(),
                        path: "react".to_string(),
                    }]
                );
            }
            other => panic!("expected a Segment, got {other:?}"),
        }
        // max(Low, High) = High.
        assert_eq!(chain.criticality, Criticality::High);
    }

    // -----------------------------------------------------------------------
    // Phase 78 W3: the plan the one derivation makes of every contract
    // fixture, pinned. W2 held this file's own derivation (deleted in W3)
    // against `mapper_input_via_model` on every fixture launch; what stays
    // is the crate's plan over each launch's own contract and platform file,
    // compared byte for byte with the snapshot committed beside this crate
    // (`snapshots/contract_fixtures.ranked_plans.txt`). A red row is a change
    // in what the model carries or in the crate's rule, and re-taking the
    // snapshot (`UPDATE_RANKED_PLAN_SNAPSHOTS=1`) is the review of that
    // change, not a tolerance.
    // -----------------------------------------------------------------------

    /// `<repo>/src/ros-launch-resolve/resolve` is this crate.
    fn repo_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("the repository root")
            .to_path_buf()
    }

    fn launch_stem(launch: &Path) -> String {
        launch
            .file_name()
            .expect("a launch file name")
            .to_string_lossy()
            .trim_end_matches(".launch.xml")
            .to_string()
    }

    /// Every `*.launch.xml` with a contract sidecar next to it under
    /// `tests/fixtures/contract_*/launch/` and `tests/fixtures/rt_workspace/launch/`,
    /// in path order. `contract_merge` contributes two (its include is a
    /// fixture launch of its own).
    fn contract_fixture_launches() -> Vec<PathBuf> {
        let fixtures = repo_root().join("tests/fixtures");
        let mut dirs: Vec<PathBuf> = std::fs::read_dir(&fixtures)
            .unwrap_or_else(|e| panic!("read {}: {e}", fixtures.display()))
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                let name = p.file_name().map(|n| n.to_string_lossy().to_string());
                name.is_some_and(|n| n.starts_with("contract_") || n == "rt_workspace")
            })
            .collect();
        dirs.sort();
        let mut out = Vec::new();
        for dir in dirs {
            let launch_dir = dir.join("launch");
            let mut files: Vec<PathBuf> = std::fs::read_dir(&launch_dir)
                .into_iter()
                .flatten()
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.to_string_lossy().ends_with(".launch.xml"))
                .collect();
            files.sort();
            for launch in files {
                let sidecar = launch_dir.join(format!("{}.contract.yaml", launch_stem(&launch)));
                if sidecar.is_file() {
                    out.push(launch);
                }
            }
        }
        out
    }

    fn fixture_name(launch: &Path) -> String {
        launch
            .strip_prefix(repo_root().join("tests/fixtures"))
            .unwrap_or(launch)
            .display()
            .to_string()
    }

    /// The launch tree, through the Rust parser, as `resolve` reads it.
    fn dump_of(launch: &Path) -> LaunchDump {
        let record = crate::verbs::parse_launch_file(launch, Default::default())
            .unwrap_or_else(|e| panic!("parse {}: {e}", launch.display()));
        let json = serde_json::to_string(&record).expect("record as json");
        serde_json::from_str(&json).expect("a LaunchDump")
    }

    /// The contract index the provider-sidecar channel resolves for the
    /// launch, exactly as `check` and `resolve` do without `--contracts`.
    fn index_of(dump: &LaunchDump, name: &str) -> ManifestIndex {
        let sources = crate::ros::manifest_loader::ContractSources {
            overlay: None,
            provider: true,
        };
        crate::ros::manifest_loader::load_manifests(dump, &sources)
            .unwrap_or_else(|e| panic!("{name}: load manifests: {e}"))
    }

    /// The per-node budgets of the launch's own posix platform file, when it
    /// ships one, the way `derive_sched_plan` reads them.
    fn budgets_of(launch: &Path) -> BTreeMap<String, u64> {
        let platform = launch.with_file_name(format!("{}.system.posix.yaml", launch_stem(launch)));
        if !platform.is_file() {
            return BTreeMap::new();
        }
        let file = ros_launch_manifest_sched::parse_platform_file(&platform)
            .unwrap_or_else(|e| panic!("parse {}: {e}", platform.display()));
        crate::ros::sched_loader::posix_budgets(&file)
    }

    /// The one rendering every consumer compares: `{:#?}` plus a trailing
    /// newline (rlm `derive/tests/parity.rs`).
    fn render(plan: &RankedPlan) -> String {
        format!("{plan:#?}\n")
    }

    /// The one derivation of `dump` under `index` and `budgets`: the input
    /// `derive_sched_plan` hands the mapper, and the rendered plan the
    /// chain-aware ranker makes of it. The report is held to what a model
    /// this resolver built in-process guarantees: a trigger on every path.
    fn derived(
        name: &str,
        dump: &LaunchDump,
        index: &ManifestIndex,
        budgets: &BTreeMap<String, u64>,
    ) -> (MapperInput, String) {
        let (input, report) = mapper_input_via_model(dump, Some(index), None, budgets);
        assert!(
            report.paths_without_trigger.is_empty(),
            "{name}: the model built in-process carries a trigger on every path: {:?}",
            report.paths_without_trigger
        );
        let plan = render(&chain_aware_rank(&input));
        (input, plan)
    }

    /// `<crate>/snapshots/contract_fixtures.ranked_plans.txt`: one section
    /// per fixture launch, a `== <fixture launch>` line and then its plan.
    fn snapshots_path() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("snapshots/contract_fixtures.ranked_plans.txt")
    }

    /// The committed sections, by fixture launch.
    fn committed_plans() -> BTreeMap<String, String> {
        let path = snapshots_path();
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
            panic!(
                "read {}: {e} (UPDATE_RANKED_PLAN_SNAPSHOTS=1 writes it)",
                path.display()
            )
        });
        let mut out = BTreeMap::new();
        let mut current: Option<(String, String)> = None;
        for line in text.split_inclusive('\n') {
            if let Some(name) = line.strip_prefix("== ") {
                if let Some((name, plan)) = current.take() {
                    out.insert(name, plan);
                }
                current = Some((name.trim_end().to_string(), String::new()));
            } else if let Some((_, plan)) = &mut current {
                plan.push_str(line);
            } else {
                panic!(
                    "{}: text before the first section: {line:?}",
                    path.display()
                );
            }
        }
        if let Some((name, plan)) = current {
            out.insert(name, plan);
        }
        out
    }

    /// The 1-based line at which `actual` first differs from `expected`.
    fn first_diff_line(actual: &str, expected: &str) -> usize {
        actual
            .lines()
            .zip(expected.lines())
            .position(|(a, e)| a != e)
            .unwrap_or_else(|| actual.lines().count().min(expected.lines().count()))
            + 1
    }

    /// The permanent form of W2's parity gate (Gates 1 of the phase): the
    /// plan the one derivation makes of every contract fixture launch, from
    /// its own contract and platform file, is the committed one, byte for
    /// byte. `contract_derived_chain`'s section is rlm's own snapshot (the
    /// test below holds the two together); the others are this repository's.
    #[test]
    fn every_contract_fixture_ranks_to_its_committed_plan() {
        let launches = contract_fixture_launches();
        assert!(
            launches
                .iter()
                .any(|l| l.to_string_lossy().contains("contract_derived_chain")),
            "the fixture walk must reach contract_derived_chain: {launches:?}"
        );
        let mut actual: BTreeMap<String, String> = BTreeMap::new();
        let mut summary = Vec::new();
        for launch in &launches {
            let name = fixture_name(launch);
            let dump = dump_of(launch);
            let index = index_of(&dump, &name);
            let budgets = budgets_of(launch);
            let (input, plan) = derived(&name, &dump, &index, &budgets);
            assert!(plan.is_ascii(), "{name}: a plan renders as ASCII");
            summary.push(format!(
                "{name}: {} node(s), {} chain(s), {} budget selector(s), {} ranked item(s)",
                input.nodes.len(),
                input.chains.len(),
                budgets.len(),
                plan.matches("RankItem {").count()
            ));
            actual.insert(name, plan);
        }
        eprintln!(
            "phase 78 plans over {} fixture launch(es):\n  {}",
            launches.len(),
            summary.join("\n  ")
        );

        if std::env::var_os("UPDATE_RANKED_PLAN_SNAPSHOTS").is_some() {
            let mut text = String::new();
            for (name, plan) in &actual {
                text.push_str(&format!("== {name}\n"));
                text.push_str(plan);
            }
            let path = snapshots_path();
            std::fs::write(&path, text).unwrap_or_else(|e| panic!("write {}: {e}", path.display()));
            eprintln!("re-took {}", path.display());
            return;
        }

        let expected = committed_plans();
        let mut diffs = Vec::new();
        for (name, plan) in &actual {
            match expected.get(name) {
                None => diffs.push(format!("{name}: no committed plan")),
                Some(committed) if committed != plan => diffs.push(format!(
                    "{name}: differs from the committed plan at line {}\n\
                     --- committed\n{committed}--- actual\n{plan}",
                    first_diff_line(plan, committed)
                )),
                Some(_) => {}
            }
        }
        for name in expected.keys().filter(|n| !actual.contains_key(*n)) {
            diffs.push(format!("{name}: committed, but no such fixture launch"));
        }
        assert!(
            diffs.is_empty(),
            "the committed plans differ (UPDATE_RANKED_PLAN_SNAPSHOTS=1 re-takes them):\n{}",
            diffs.join("\n")
        );
    }

    /// rlm v0.1.37 `derive/tests/snapshots/contract_derived_chain.ranked_plan.txt`,
    /// verbatim: the Debug text of
    /// `chain_aware_rank(&mapper_input_from_model(fixture, no facts))` on the
    /// resolved `contract_derived_chain` model checked in there. Both
    /// consumers assert their own rank of the same system is this text;
    /// re-taking it is an rlm change (`UPDATE_RANKED_PLAN_SNAPSHOT=1` there)
    /// that both gates then move with.
    const RLM_RANKED_PLAN_SNAPSHOT: &str = r#"RankedPlan {
    items: [
        RankItem {
            node: "/control/control_node",
            path: "control",
            fine_group: 0,
            coarse_group: Some(
                "points_to_cmd",
            ),
            tie_group: None,
            provenance: "derived(chain_aware: points_to_cmd segment drain 1/2)",
        },
        RankItem {
            node: "/perception/filter_component",
            path: "filter",
            fine_group: 0,
            coarse_group: Some(
                "points_to_cmd",
            ),
            tie_group: None,
            provenance: "derived(chain_aware: points_to_cmd segment drain 2/2)",
        },
        RankItem {
            node: "/perception/sensor_node",
            path: "tick",
            fine_group: 1,
            coarse_group: Some(
                "points_to_cmd",
            ),
            tie_group: None,
            provenance: "derived(chain_aware: points_to_cmd boundary RM period=10ms)",
        },
    ],
    warnings: [
        ChainFeasibleWithoutWcet {
            chain: "points_to_cmd",
            boundaries_without_wcet: [
                "/perception/sensor_node/tick",
            ],
        },
    ],
}
"#;

    /// Parity assertion 1 of design issue #52, play_launch's side: the rank
    /// of `contract_derived_chain` from this repository's own launch, contract
    /// and platform file is rlm's golden snapshot, and the committed plan
    /// above is that same text.
    #[test]
    fn contract_derived_chain_ranks_to_rlm_snapshot() {
        assert!(RLM_RANKED_PLAN_SNAPSHOT.is_ascii());
        let launch =
            repo_root().join("tests/fixtures/contract_derived_chain/launch/bringup.launch.xml");
        let name = fixture_name(&launch);
        let dump = dump_of(&launch);
        let index = index_of(&dump, &name);
        let budgets = budgets_of(&launch);
        assert!(
            budgets.is_empty(),
            "the fixture's platform file pins a priority, never a budget, so the \
             snapshot's no-facts rank is the rank here: {budgets:?}"
        );
        let (input, plan) = derived(&name, &dump, &index, &budgets);
        if plan != RLM_RANKED_PLAN_SNAPSHOT {
            let first_diff = first_diff_line(&plan, RLM_RANKED_PLAN_SNAPSHOT);
            panic!(
                "{name}: the rank differs from rlm's snapshot at line {first_diff}\n\
                 --- expected\n{RLM_RANKED_PLAN_SNAPSHOT}\n--- actual\n{plan}"
            );
        }
        assert_eq!(
            committed_plans().get(&name).map(String::as_str),
            Some(RLM_RANKED_PLAN_SNAPSHOT),
            "the committed plan of {name} is rlm's snapshot"
        );
        // The facts rlm's parity test pins in words, so a diff reads as a fact.
        let sensor = input
            .nodes
            .iter()
            .find(|n| n.name == "/perception/sensor_node")
            .expect("the sensor");
        assert_eq!(
            sensor.rate_hz,
            Some(100.0),
            "the timer's rate, from the trigger"
        );
        assert_eq!(sensor.criticality, None);
        let control = input
            .nodes
            .iter()
            .find(|n| n.name == "/control/control_node")
            .expect("the controller");
        assert_eq!(control.rate_hz, None);
        assert_eq!(control.deadline_us, Some(10_000));
        assert_eq!(control.criticality, Some(Criticality::High));
        assert!(input.nodes.iter().all(|n| !n.claims_concurrency));
    }

    /// Seam 1 of W2, resolved in the crate's favour: the one-path budget rule
    /// holds on a chain boundary too. The derivation W3 deleted used to give
    /// a node's budget to the boundary whatever the path count, and to
    /// `MapperPath::exec_ms` only on a one-path node; rlm's
    /// `a_node_budget_is_not_attributed_to_a_boundary_of_a_multi_path_node`
    /// pins the number (a 100 Hz boundary with a 2 ms node budget on a
    /// two-timer node: sampling cost 12 ms the old way, 10 ms and a
    /// `ChainFeasibleWithoutWcet` warning now). Here, the same shape in this
    /// file's own fixture: 50 Hz, 3.5 ms budget, a second timer path added.
    #[test]
    fn a_node_budget_reaches_a_boundary_only_on_a_one_path_node() {
        let ms = ros_launch_manifest_types::duration::Duration::from_millis_f64;
        let mut index = chain_index_for_cost_tests();
        let diag = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 1.0 }),
            output: vec!["diag".to_string()],
            max_latency: Some(ms(1.0)),
            ..Default::default()
        };
        let talker = index
            .manifests
            .get_mut(&0)
            .expect("scope 0")
            .manifest
            .nodes
            .get_mut("talker")
            .expect("talker");
        talker
            .publishers
            .insert("diag".to_string(), Default::default());
        talker.paths.insert("diag".to_string(), diag.clone());
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "diag".to_string(),
            path: diag,
            scope_id: 0,
        });
        index.topics.insert(
            "/diag".to_string(),
            ResolvedTopic {
                derived_from_remaps: false,
                fqn: "/diag".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/talker/diag".to_string()],
                subscribers: vec![],
                rate_hz: None,
                derived_rate_hz: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );
        let budgets = BTreeMap::from([("/talker".to_string(), 3_500u64)]);

        let dump = dump_with_two_nodes();
        let (input, _) = derived("two-timer talker", &dump, &index, &budgets);
        let chain = input
            .chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain resolves");
        let sampling_ms: f64 = chain
            .elements
            .iter()
            .filter_map(|e| match e {
                ChainElement::Boundary {
                    node,
                    path,
                    period_ms,
                    exec_ms,
                } => {
                    assert_eq!((node.as_str(), path.as_str()), ("/talker", "tick"));
                    assert_eq!(*period_ms, 20.0);
                    assert_eq!(
                        *exec_ms, None,
                        "a node budget is not split over two paths, on the boundary either"
                    );
                    Some(period_ms + exec_ms.unwrap_or(0.0))
                }
                ChainElement::Segment { .. } => None,
            })
            .sum();
        assert_eq!(sampling_ms, 20.0, "counted at zero, not at 23.5");
        let talker = input
            .nodes
            .iter()
            .find(|n| n.name == "/talker")
            .expect("talker");
        assert_eq!(talker.paths.len(), 2);
        assert!(talker.paths.iter().all(|p| p.exec_ms.is_none()));
        assert_eq!(talker.rate_hz, Some(50.0), "the fastest timer");
        let plan = chain_aware_rank(&input);
        assert_eq!(
            plan.warnings,
            vec![
                ros_launch_manifest_sched::MapWarning::ChainFeasibleWithoutWcet {
                    chain: "mixed_chain".to_string(),
                    boundaries_without_wcet: vec!["/talker/tick".to_string()],
                }
            ],
            "and the verdict says the boundary was counted at zero"
        );
    }

    /// Seam 2 of W2, resolved in the crate's favour: a node whose derived
    /// criticality buckets to the no-requirement level keeps its advisory
    /// label. The derivation W3 deleted used to emit nothing for it; the
    /// model carries no `node_criticality` entry and the crate falls back to
    /// `NodeInstance::criticality`. A hazard that buckets to a level still
    /// overrides the label.
    #[test]
    fn a_hazard_bucketing_to_no_requirement_leaves_the_label_standing() {
        use crate::ros::manifest_loader::DerivedCriticality;
        let mut index = chain_index_for_cost_tests();
        // listener is labelled `high`; the hazard that reaches it says QM.
        index.derived_criticality.insert(
            "/listener".to_string(),
            DerivedCriticality {
                level: "QM".to_string(),
                rank: 0,
                scale_len: 5,
                hazard: "h1".to_string(),
                role: "reaction",
            },
        );
        // talker is labelled `low`; the hazard that reaches it says ASIL D.
        index.derived_criticality.insert(
            "/talker".to_string(),
            DerivedCriticality {
                level: "D".to_string(),
                rank: 4,
                scale_len: 5,
                hazard: "h1".to_string(),
                role: "detector",
            },
        );
        let dump = dump_with_two_nodes();
        let (input, _) = derived("hazard buckets", &dump, &index, &BTreeMap::new());
        let of = |name: &str| {
            input
                .nodes
                .iter()
                .find(|n| n.name == name)
                .unwrap_or_else(|| panic!("{name}"))
                .criticality
        };
        assert_eq!(of("/listener"), Some(Criticality::High), "the label stands");
        assert_eq!(
            of("/talker"),
            Some(Criticality::High),
            "the hazard overrides the label"
        );
        let chain = input
            .chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain resolves");
        assert_eq!(chain.criticality, Criticality::High);
    }
}
