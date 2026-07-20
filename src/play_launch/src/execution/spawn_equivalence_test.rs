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
use crate::{cli::options::ContainerMode, ros::launch_dump::LaunchDump};
use ros_launch_manifest_model::SystemModel;
use std::path::PathBuf;

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
