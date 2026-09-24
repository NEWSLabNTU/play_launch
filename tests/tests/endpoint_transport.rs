//! Issue #0042 — one derivation, one answer, where a contract overrides
//! transport per subscriber.
//!
//! `max_transport` is declarable on a topic AND on a subscriber endpoint, and
//! the endpoint wins where it is declared. Two things computed that
//! precedence: the resolver's `manifest_graph::build_global_graph`, from the
//! `ManifestIndex`, and the manifest crate's `derive`, from the
//! `SystemModel`. Only the first could see the endpoint value, because
//! `model::SubContract` did not carry it — so on any contract that used the
//! field the two produced different route totals with no diagnostic from
//! either.
//!
//! Wave A (rlm `v0.1.42`) put the field on the model and the precedence in
//! `TopicView::transport_ms`; wave B lowers it in `model_builder.rs`. This is
//! the gate: ONE contract, resolved once, read by BOTH consumers, asserted to
//! name the same route.
//!
//! The fixture forks on purpose (see `bringup.contract.yaml` for the worked
//! numbers). A total alone can be right by accident; a total AND the branch
//! that produced it cannot — and both wrong precedences name the other branch.

use play_launch_tests::fixtures;
use ros_launch_manifest_derive::{DeriveFacts, resolve_chains};
use ros_launch_manifest_model::SystemModel;
use ros_launch_manifest_sched::chain::ChainElement;

/// The subscriber that states its own transport, and the one that inherits
/// the topic's.
const OVERRIDING_SUB: &str = "/et/fast/objects";
const INHERITING_SUB: &str = "/et/slow/objects";

/// What both consumers must agree on: `producer → slow → sink`, 40ms.
/// Ignoring the override gives `producer → fast → sink` at 45ms; applying it
/// to every subscriber gives `producer → fast → sink` at 35ms.
const EXPECTED_ROUTE: [&str; 3] = ["/et/producer", "/et/slow", "/et/sink"];
const EXPECTED_TOTAL: &str = "/et/producer → /et/slow → /et/sink = 40.00ms";

fn fixture_launch() -> std::path::PathBuf {
    fixtures::repo_root().join("tests/fixtures/endpoint_transport/launch/bringup.launch.xml")
}

/// `resolve` the fixture and return both the raw model text and the parsed
/// `SystemModel` the shared derivation consumes.
fn resolve_model(out: &std::path::Path) -> SystemModel {
    let env = fixtures::install_env();
    let mut cmd = fixtures::ros_launch_resolve_cmd(&env);
    cmd.args(["resolve", fixture_launch().to_str().unwrap()])
        .args(["-o", out.to_str().unwrap()]);
    let output = cmd.output().expect("run ros-launch-resolve resolve");
    assert!(
        output.status.success(),
        "resolve failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let yaml = std::fs::read_to_string(out).expect("read model");
    serde_yaml_ng::from_str(&yaml).expect("parse SystemModel")
}

/// The lowering itself. `/et/fast/objects` declares `max_transport` and
/// NOTHING else, so it is also the guard test: `sub_contract`'s all-absent
/// early return has to count the new field, or this endpoint is dropped from
/// the model entirely and `derive` sees no override to apply.
#[test]
fn a_subscribers_max_transport_reaches_the_model() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let model = resolve_model(&tmp.path().join("system_model.yaml"));

    let sub = model
        .contracts
        .sub_endpoints
        .get(OVERRIDING_SUB)
        .unwrap_or_else(|| {
            panic!(
                "{OVERRIDING_SUB} declares only `max_transport`; it is missing from \
                 contracts.sub_endpoints, so `sub_contract`'s all-absent guard dropped it. \
                 Keys present: {:?}",
                model.contracts.sub_endpoints.keys().collect::<Vec<_>>()
            )
        });
    assert_eq!(
        sub.max_transport_ms,
        Some(0.0),
        "the endpoint's declared 0ms must reach the model"
    );

    // The other subscriber declares nothing, so it must carry no transport of
    // its own — an absent override is what makes the topic's value apply.
    assert!(
        model
            .contracts
            .sub_endpoints
            .get(INHERITING_SUB)
            .and_then(|s| s.max_transport_ms)
            .is_none(),
        "{INHERITING_SUB} declares no transport; the topic's default is what it pays"
    );
    assert_eq!(
        model
            .contracts
            .topics
            .get("/et/objects")
            .and_then(|t| t.max_transport_ms),
        Some(10.0),
        "the topic default must still be carried"
    );
}

/// The two consumers, on one contract.
///
/// The resolver names its route and total in `scope-budget` (the fixture's
/// budget is 1ms so the rule always fires and prints them). The shared
/// derivation names its route in the chain it resolves for the same scope
/// path. They must be the same route.
#[test]
fn the_resolver_and_the_shared_derivation_take_the_same_route() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let model = resolve_model(&tmp.path().join("system_model.yaml"));

    // --- consumer 1: the resolver's own graph -------------------------
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["check", fixture_launch().to_str().unwrap()]);
    let out = cmd.output().expect("run play_launch check");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        text.contains(EXPECTED_TOTAL),
        "the resolver must charge the endpoint override on its own edge and the \
         topic default on the other: expected `{EXPECTED_TOTAL}` in\n{text}"
    );

    // --- consumer 2: the shared derivation, over the model above ------
    let chains = resolve_chains(&model, &DeriveFacts::default());
    assert_eq!(
        chains.len(),
        1,
        "one scope path, so one chain: {:?}",
        chains.iter().map(|c| &c.name).collect::<Vec<_>>()
    );
    let route: Vec<String> = chains[0]
        .elements
        .iter()
        .flat_map(|e| match e {
            ChainElement::Segment { nodes_in_topo_order } => nodes_in_topo_order
                .iter()
                .map(|n| n.node.clone())
                .collect::<Vec<_>>(),
            ChainElement::Boundary { node, .. } => vec![node.clone()],
        })
        .collect();
    assert_eq!(
        route,
        EXPECTED_ROUTE.map(str::to_string).to_vec(),
        "`derive` must take the SAME branch the resolver named. A route through \
         '/et/fast' means the per-subscriber transport did not reach the model \
         (issue #0042), or reached every subscriber instead of the one that \
         declared it."
    );
}
