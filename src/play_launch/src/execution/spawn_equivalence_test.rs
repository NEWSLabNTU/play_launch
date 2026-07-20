//! Phase 46.3b — the static equivalence gate: prove the model-sourced spawn
//! context (`context::prepare_*_contexts_from_model`) produces the SAME
//! argv/env/params-file-content as the record-sourced spawn context
//! (`context::prepare_*_contexts`) for the same launch, before any process
//! is ever spawned from the model. See `.superpowers/sdd/p46-w3-analysis.md`
//! and the 46.3b report.
//!
//! Uses the `rt_workspace` fixture (`tests/fixtures/rt_workspace/`) — real
//! ROS nodes/container/composable exercising remaps, env, respawn,
//! container-mode override, and composable-container matching. Requires the
//! fixture to be built + dumped + resolved once:
//!
//! ```sh
//! cd tests/fixtures/rt_workspace && just build
//! source /opt/ros/humble/setup.bash && source ../../../install/setup.bash \
//!     && source install/setup.bash
//! ../../../install/play_launch/lib/play_launch/play_launch dump \
//!     --output record.json launch --parser rust rt_demo bringup.launch.xml
//! ../../../install/play_launch/lib/play_launch/play_launch resolve \
//!     --record record.json --sched launch/bringup.system.posix.yaml \
//!     -o system_model.yaml
//! ```
//!
//! `record.json`/`system_model.yaml` are gitignored build products (same
//! convention as `tests/tests/rt_workspace.rs`) — this test SKIPS (not
//! fails) when they're absent, rather than treating a fresh checkout as a
//! failure.

use super::context::{
    ComposableNodeContextSet, prepare_composable_node_contexts,
    prepare_composable_node_contexts_from_model, prepare_container_contexts,
    prepare_container_contexts_from_model, prepare_node_contexts, prepare_node_contexts_from_model,
};
use crate::{
    cli::options::ContainerMode,
    ros::{launch_dump::LaunchDump, manifest_loader::ManifestIndex, model_builder},
};
use ros_launch_manifest_model::SystemModel;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures/rt_workspace")
}

/// Loads `(record.json, system_model.yaml)` from the `rt_workspace`
/// fixture. Returns `None` (having printed a SKIP note) when either file is
/// missing, mirroring `tests/tests/rt_workspace.rs::require_rt_workspace()`.
fn load_fixture() -> Option<(LaunchDump, SystemModel)> {
    let dir = fixture_dir();
    let record_path = dir.join("record.json");
    let model_path = dir.join("system_model.yaml");
    if !record_path.is_file() || !model_path.is_file() {
        eprintln!(
            "SKIP: rt_workspace record.json/system_model.yaml not found under {} — run \
             `cd tests/fixtures/rt_workspace && just build` then dump+resolve \
             (see this file's module doc comment for the exact commands) first",
            dir.display()
        );
        return None;
    }

    let dump = crate::ros::launch_dump::load_launch_dump(&record_path)
        .expect("failed to parse rt_workspace record.json");
    let yaml = std::fs::read_to_string(&model_path).expect("failed to read system_model.yaml");
    let model = SystemModel::from_yaml_str(&yaml).expect("failed to parse system_model.yaml");
    Some((dump, model))
}

/// Regular `<node>` and `<node_container>` instances: build the spawn
/// context via BOTH paths and assert argv (`to_cmdline`) + env are
/// byte-identical per node. Containers are tested at the DEFAULT
/// `--container-mode` (`Isolated`) so the package/executable override is
/// exercised identically on both paths — not just the Stock passthrough.
#[test]
fn rt_workspace_node_and_container_argv_env_match() {
    let Some((dump, model)) = load_fixture() else {
        return;
    };

    let record_tmp = tempfile::TempDir::new().expect("tempdir");
    let model_tmp = tempfile::TempDir::new().expect("tempdir");

    let record_nodes =
        prepare_node_contexts(&dump, record_tmp.path()).expect("record-path node contexts");
    let model_nodes = prepare_node_contexts_from_model(&model, model_tmp.path())
        .expect("model-path node contexts");

    assert_eq!(
        record_nodes.len(),
        model_nodes.len(),
        "regular node count must match between record and model paths"
    );

    for r in &record_nodes {
        let key = (r.record.namespace.clone(), r.record.name.clone());
        let m = model_nodes
            .iter()
            .find(|m| (m.record.namespace.clone(), m.record.name.clone()) == key)
            .unwrap_or_else(|| panic!("model path missing regular node {key:?}"));

        // Compare the structured `NodeCommandLine` (derived `PartialEq`),
        // not the flattened `to_cmdline()` `Vec<String>`: `remaps`/`params`
        // are `HashMap`s built via a fresh `.collect()` on EACH call (once
        // per path), so their iteration order is incidental — even calling
        // `from_node_record` twice on the byte-identical `NodeRecord`
        // produces two argv vectors with the remap flags in a different
        // (but equally correct) order. That's a `HashMap` property, not a
        // record-vs-model divergence; comparing the struct compares
        // content, order-independently for the map/set fields and
        // order-sensitively for `command`/`user_args` (where order is
        // actually load-bearing).
        assert_eq!(
            r.cmdline, m.cmdline,
            "spawn context mismatch for node {key:?}"
        );
    }

    let record_containers =
        prepare_container_contexts(&dump, record_tmp.path(), ContainerMode::Isolated)
            .expect("record-path container contexts");
    let model_containers =
        prepare_container_contexts_from_model(&model, model_tmp.path(), ContainerMode::Isolated)
            .expect("model-path container contexts");

    assert_eq!(
        record_containers.len(),
        model_containers.len(),
        "container count must match between record and model paths"
    );

    for r in &record_containers {
        let key = (
            r.node_context.record.namespace.clone(),
            r.node_context.record.name.clone(),
        );
        let m = model_containers
            .iter()
            .find(|m| {
                (
                    m.node_context.record.namespace.clone(),
                    m.node_context.record.name.clone(),
                ) == key
            })
            .unwrap_or_else(|| panic!("model path missing container {key:?}"));

        // See the regular-node comparison above for why the structured
        // `NodeCommandLine` (not the flattened `to_cmdline()` argv) is the
        // right equality check here.
        assert_eq!(
            r.node_context.cmdline, m.node_context.cmdline,
            "spawn context mismatch for container {key:?} (container-mode override)"
        );
    }
}

/// Same gate, at `--container-mode stock` — proves the equivalence holds
/// independent of the container-mode override branch too (the Isolated
/// test above already exercises the override itself).
#[test]
fn rt_workspace_container_argv_matches_model_stock_mode() {
    let Some((dump, model)) = load_fixture() else {
        return;
    };
    let record_tmp = tempfile::TempDir::new().expect("tempdir");
    let model_tmp = tempfile::TempDir::new().expect("tempdir");

    let record_containers =
        prepare_container_contexts(&dump, record_tmp.path(), ContainerMode::Stock)
            .expect("record-path container contexts (stock)");
    let model_containers =
        prepare_container_contexts_from_model(&model, model_tmp.path(), ContainerMode::Stock)
            .expect("model-path container contexts (stock)");

    assert_eq!(record_containers.len(), model_containers.len());
    for r in &record_containers {
        let key = r.node_context.record.name.clone();
        let m = model_containers
            .iter()
            .find(|m| m.node_context.record.name == key)
            .unwrap_or_else(|| panic!("model path missing container {key:?}"));
        assert_eq!(
            r.node_context.cmdline, m.node_context.cmdline,
            "spawn context mismatch (stock mode) for container {key:?}"
        );
    }
}

/// Composable node LoadNode-request fields: no `NodeCommandLine` involved
/// (composables have no argv of their own) — compare the synthesized
/// `ComposableNodeRecord` fields directly, including GAP-6 `extra_args`.
#[test]
fn rt_workspace_composable_node_request_fields_match() {
    let Some((dump, model)) = load_fixture() else {
        return;
    };

    let record_dir = tempfile::TempDir::new().expect("tempdir");
    let model_dir = tempfile::TempDir::new().expect("tempdir");

    let ComposableNodeContextSet {
        load_node_contexts: record_composables,
    } = prepare_composable_node_contexts(&dump, record_dir.path())
        .expect("record-path composable contexts");
    let ComposableNodeContextSet {
        load_node_contexts: model_composables,
    } = prepare_composable_node_contexts_from_model(&model, model_dir.path())
        .expect("model-path composable contexts");

    assert_eq!(
        record_composables.len(),
        model_composables.len(),
        "composable node count must match between record and model paths"
    );

    for r in &record_composables {
        let key = (r.record.node_name.clone(), r.record.namespace.clone());
        let m = model_composables
            .iter()
            .find(|m| (m.record.node_name.clone(), m.record.namespace.clone()) == key)
            .unwrap_or_else(|| panic!("model path missing composable node {key:?}"));

        assert_eq!(
            r.record.package, m.record.package,
            "package mismatch for {key:?}"
        );
        assert_eq!(
            r.record.plugin, m.record.plugin,
            "plugin mismatch for {key:?}"
        );
        assert_eq!(
            r.record.target_container_name, m.record.target_container_name,
            "target_container_name mismatch for {key:?}"
        );
        assert_eq!(
            r.record.remaps, m.record.remaps,
            "remaps mismatch for {key:?}"
        );
        assert_eq!(
            r.record.params, m.record.params,
            "params mismatch for {key:?}"
        );
        assert_eq!(
            r.record.extra_args, m.record.extra_args,
            "extra_args mismatch for {key:?}"
        );
    }
}

/// `params_files`/`overrides.yaml` content diff: rt_workspace's own nodes
/// carry no params (record.json confirms `params: []`, `params_files: []`
/// on every record), so this asserts BOTH paths agree on that (zero files
/// written, no `--params-file` in argv) — the params-*value* round-trip
/// itself (typed `ParamValue` -> string, including the float-formatting
/// risk) is covered fixture-independently by
/// `node_cmdline::tests::test_node_record_from_instance_field_mapping` /
/// `test_param_value_to_record_string_preserves_float_type`, since
/// rt_workspace has no params to exercise it with.
#[test]
fn rt_workspace_nodes_write_no_params_files_either_path() {
    let Some((dump, model)) = load_fixture() else {
        return;
    };
    let record_tmp = tempfile::TempDir::new().expect("tempdir");
    let model_tmp = tempfile::TempDir::new().expect("tempdir");

    let record_nodes =
        prepare_node_contexts(&dump, record_tmp.path()).expect("record-path node contexts");
    let model_nodes = prepare_node_contexts_from_model(&model, model_tmp.path())
        .expect("model-path node contexts");

    for ctx in record_nodes.iter().chain(model_nodes.iter()) {
        assert!(
            ctx.cmdline.params_files.is_empty(),
            "expected no params_files for {:?}",
            ctx.record.name
        );
        assert!(
            ctx.cmdline.overrides_file.is_none(),
            "expected no overrides.yaml for {:?}",
            ctx.record.name
        );
        assert!(
            !ctx.cmdline
                .to_cmdline(false)
                .iter()
                .any(|a| a.ends_with(".yaml")),
            "expected no --params-file in argv for {:?}",
            ctx.record.name
        );
    }
}

/// Item 3 (46.3b review) — the equivalence gate must exercise materialized
/// params-file CONTENT, not just presence. rt_workspace's own launch has
/// no params, and its shared `bringup.launch.xml` is depended on by ~15
/// other tests (node counts, chain derivation, contract endpoints), so
/// rather than perturb it, this builds a SYNTHETIC `LaunchDump` in-process
/// — a single `rt_demo/sensor_node` (the real, built binary, resolved via
/// ament) carrying BOTH inline `<param>` values (→ `overrides.yaml`) AND an
/// external `params_files` YAML blob (→ `0.yaml`) — resolves it to a
/// `SystemModel` the same way `play_launch resolve` does
/// (`build_system_model`), then builds the spawn context via BOTH paths and
/// asserts the actual bytes written to `overrides.yaml`/`0.yaml` are
/// identical record-vs-model. Skips (not fails) if `rt_demo` isn't on
/// `AMENT_PREFIX_PATH` (fixture unbuilt / env not sourced), mirroring
/// `load_fixture`.
#[test]
fn params_file_content_matches_record_vs_model() {
    // The one real dependency of this test (not the fixture files): the
    // rt_demo package resolvable via ament so `from_node_record` can find
    // the executable. Skip cleanly otherwise.
    if crate::ros::ament_index::find_executable("rt_demo", "sensor_node").is_err() {
        eprintln!(
            "SKIP: rt_demo/sensor_node not resolvable via ament — build the \
             rt_workspace fixture and source its install/setup.bash first"
        );
        return;
    }

    let external_yaml = "/**:\n  ros__parameters:\n    external_gain: 0.25\n    \
                         external_mode: fast\n    external_count: 7\n";
    let dump: LaunchDump = serde_json::from_value(serde_json::json!({
        "node": [{
            "executable": "sensor_node",
            "package": "rt_demo",
            "name": "sensor_node",
            "exec_name": "sensor_node",
            "namespace": "/perception",
            // inline params → overrides.yaml (mix of float/string/int/bool
            // to exercise the type-inference round-trip too)
            "params": [
                ["publish_rate_hz", "100.0"],
                ["label", "primary"],
                ["retries", "3"],
                ["verbose", "true"]
            ],
            // external param file content → 0.yaml, verbatim
            "params_files": [external_yaml],
            "cmd": [],
            "remaps": []
        }],
        "container": [],
        "load_node": [],
        "lifecycle_node": [],
        "file_data": {},
        "scopes": [{"id": 0, "ns": "/", "parent": null}]
    }))
    .expect("valid synthetic LaunchDump");

    let model = model_builder::build_system_model(
        &dump,
        &ManifestIndex::default(),
        None,
        BTreeMap::new(),
        &BTreeSet::new(),
    );

    let record_tmp = tempfile::TempDir::new().expect("tempdir");
    let model_tmp = tempfile::TempDir::new().expect("tempdir");

    let record_nodes =
        prepare_node_contexts(&dump, record_tmp.path()).expect("record-path node contexts");
    let model_nodes = prepare_node_contexts_from_model(&model, model_tmp.path())
        .expect("model-path node contexts");

    assert_eq!(record_nodes.len(), 1);
    assert_eq!(model_nodes.len(), 1);
    let r = &record_nodes[0];
    let m = &model_nodes[0];

    // Path-independent fields equal (the two params-file PATHS embed each
    // run's own temp dir, so the full-struct `PartialEq` can't be used here
    // — the CONTENT is asserted below, which is the point).
    assert_eq!(r.cmdline.command, m.cmdline.command, "command mismatch");
    assert_eq!(
        r.cmdline.user_args, m.cmdline.user_args,
        "user_args mismatch"
    );
    assert_eq!(r.cmdline.remaps, m.cmdline.remaps, "remaps mismatch");
    assert_eq!(
        r.cmdline.params, m.cmdline.params,
        "inline -p params mismatch"
    );
    assert_eq!(r.cmdline.env, m.cmdline.env, "env mismatch");

    // Both paths must have written exactly one external file + one
    // overrides file.
    assert_eq!(
        r.cmdline.params_files.len(),
        1,
        "record path should materialize the one external params file"
    );
    assert!(
        r.cmdline.overrides_file.is_some(),
        "record path overrides.yaml"
    );
    assert_eq!(m.cmdline.params_files.len(), 1);
    assert!(m.cmdline.overrides_file.is_some());

    // The CONTENT diff the review asked for: read the actual bytes each
    // path wrote and assert equality.
    let read = |p: &std::path::Path| std::fs::read_to_string(p).expect("read params file");

    let r_external = read(r.cmdline.params_files.iter().next().unwrap());
    let m_external = read(m.cmdline.params_files.iter().next().unwrap());
    assert_eq!(
        r_external, m_external,
        "external params-file CONTENT diverges record-vs-model"
    );
    // And it's the verbatim source blob (GAP-3: not re-parsed/re-emitted).
    assert_eq!(r_external, external_yaml, "external content not verbatim");

    let r_overrides = read(r.cmdline.overrides_file.as_ref().unwrap());
    let m_overrides = read(m.cmdline.overrides_file.as_ref().unwrap());
    assert_eq!(
        r_overrides, m_overrides,
        "overrides.yaml (inline params) CONTENT diverges record-vs-model:\n\
         record:\n{r_overrides}\nmodel:\n{m_overrides}"
    );
    // Sanity: the inline params actually landed with correct types (float
    // keeps its decimal point, int/bool/string inferred).
    assert!(
        r_overrides.contains("publish_rate_hz: 100.0"),
        "{r_overrides}"
    );
    assert!(r_overrides.contains("retries: 3"), "{r_overrides}");
    assert!(r_overrides.contains("verbose: true"), "{r_overrides}");
    assert!(r_overrides.contains("label: primary"), "{r_overrides}");
}
