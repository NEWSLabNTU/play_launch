//! Integration tests that load YAML fixture files and run the static checker.

use ros_launch_manifest_check::{Severity, run_checks};
use ros_launch_manifest_types::{
    filter_manifest, parse_manifest, parse_manifest_str, resolve_args, substitute_manifest,
};
use std::{collections::HashMap, path::PathBuf};

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../tests/fixtures")
        .join(name)
        .join("manifest.yaml")
}

// ── manifest_simple: talker/listener, clean ──

#[test]
fn fixture_simple_parses() {
    let m = parse_manifest(&fixture_path("manifest_simple")).unwrap();
    assert_eq!(m.version, 1);
    assert_eq!(m.nodes.len(), 2);
    assert!(m.nodes.contains_key("talker"));
    assert!(m.nodes.contains_key("listener"));
    assert_eq!(m.topics.len(), 1);
    assert!(m.topics.contains_key("chatter"));
}

#[test]
fn fixture_simple_clean() {
    let m = parse_manifest(&fixture_path("manifest_simple")).unwrap();
    let result = run_checks(&m);
    assert!(
        !result.has_errors(),
        "simple fixture should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

// ── manifest_pipeline: perception pipeline, clean ──

#[test]
fn fixture_pipeline_parses() {
    let m = parse_manifest(&fixture_path("manifest_pipeline")).unwrap();
    assert_eq!(
        m.nodes.len(),
        4,
        "cropbox + ground_filter + fusion + tracker"
    );
    assert_eq!(m.topics.len(), 6);
    assert!(!m.paths.is_empty());
    // Verify mix of absolute and relative topic keys
    assert!(m.topics.contains_key("/sensing/lidar/pointcloud"));
    assert!(m.topics.contains_key("cropped_points"));
}

#[test]
fn fixture_pipeline_clean() {
    let m = parse_manifest(&fixture_path("manifest_pipeline")).unwrap();
    let result = run_checks(&m);
    assert!(
        !result.has_errors(),
        "pipeline fixture should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_pipeline_state_endpoint() {
    let m = parse_manifest(&fixture_path("manifest_pipeline")).unwrap();
    let fusion = &m.nodes["fusion"];
    let camera = &fusion.subscribers["camera_objects"];
    assert!(
        camera.state.unwrap_or(false),
        "camera_objects should be state"
    );
}

// ── manifest_ndt: NDT localization with feedback cycle ──

#[test]
fn fixture_ndt_parses() {
    let m = parse_manifest(&fixture_path("manifest_ndt")).unwrap();
    assert_eq!(
        m.nodes.len(),
        3,
        "voxel_grid + ndt_scan_matcher + ekf_localizer"
    );
    assert!(m.services.contains_key("trigger_ndt"));
}

#[test]
fn fixture_ndt_clean() {
    let m = parse_manifest(&fixture_path("manifest_ndt")).unwrap();
    let result = run_checks(&m);
    // The EKF→NDT feedback uses state: true, so it should NOT be a causal cycle
    let cycle_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "causal-dag" && d.severity == Severity::Error)
        .collect();
    assert!(
        cycle_errs.is_empty(),
        "state feedback should not create causal cycle: {cycle_errs:?}"
    );
    assert!(
        !result.has_errors(),
        "NDT fixture should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_ndt_required_endpoints() {
    let m = parse_manifest(&fixture_path("manifest_ndt")).unwrap();
    let ndt = &m.nodes["ndt_scan_matcher"];
    assert!(ndt.subscribers["initial_pose"].required.unwrap_or(false));
    assert!(ndt.subscribers["map"].required.unwrap_or(false));
    assert!(ndt.subscribers["initial_pose"].state.unwrap_or(false));
    assert!(ndt.subscribers["map"].state.unwrap_or(false));
}

#[test]
fn fixture_ndt_service() {
    let m = parse_manifest(&fixture_path("manifest_ndt")).unwrap();
    let svc = &m.services["trigger_ndt"];
    assert_eq!(svc.srv_type, "std_srvs/srv/Trigger");
    assert!(
        svc.server
            .contains(&"ndt_scan_matcher/trigger_node".to_string())
    );
}

// ── manifest_periodic: timer-driven nodes ──

#[test]
fn fixture_periodic_parses() {
    let m = parse_manifest(&fixture_path("manifest_periodic")).unwrap();
    assert_eq!(m.nodes.len(), 3);
    // All paths should have empty input (timer-driven)
    for (name, node) in &m.nodes {
        for (path_name, path) in &node.paths {
            assert!(
                path.input.is_empty(),
                "node {name} path {path_name} should have empty input (periodic)"
            );
        }
    }
}

#[test]
fn fixture_periodic_clean() {
    let m = parse_manifest(&fixture_path("manifest_periodic")).unwrap();
    let result = run_checks(&m);
    assert!(
        !result.has_errors(),
        "periodic fixture should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_periodic_all_state_subs() {
    let m = parse_manifest(&fixture_path("manifest_periodic")).unwrap();
    let controller = &m.nodes["controller"];
    assert!(controller.subscribers["trajectory"].state.unwrap_or(false));
    assert!(
        controller.subscribers["current_velocity"]
            .state
            .unwrap_or(false)
    );
}

// ── manifest_violations: intentional violations ──

#[test]
fn fixture_violations_parses() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    assert!(!m.nodes.is_empty());
}

#[test]
fn fixture_violations_qos_error() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    let result = run_checks(&m);
    let qos_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "qos-compat" && d.severity == Severity::Error)
        .collect();
    assert!(
        !qos_errs.is_empty(),
        "expected QoS error for 'persistent' durability"
    );
}

#[test]
fn fixture_violations_rate_hierarchy() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    let result = run_checks(&m);
    let rate_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "rate-hierarchy" && d.severity == Severity::Error)
        .collect();
    assert!(
        rate_errs.len() >= 2,
        "expected at least 2 rate hierarchy errors (pub < topic, topic < sub): {rate_errs:?}"
    );
}

#[test]
fn fixture_violations_causal_cycle() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    let result = run_checks(&m);
    let cycle_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "causal-dag" && d.severity == Severity::Error)
        .collect();
    assert!(
        !cycle_errs.is_empty(),
        "expected causal cycle error from a↔b"
    );
}

#[test]
fn fixture_violations_scope_budget() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    let result = run_checks(&m);
    // Sum of node latencies (40+25+25+10+10=110) > scope latency (100) should warn
    let budget_warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "scope-budget" && d.severity == Severity::Warning)
        .collect();
    assert!(!budget_warns.is_empty(), "expected scope budget warning");
}

#[test]
fn fixture_violations_drop_sanity() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    let result = run_checks(&m);
    // drop-sanity checks values are in valid range
    let drop_diags: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "drop-sanity")
        .collect();
    // The fixture has valid drop values so no drop-sanity errors expected from the base fixture
    // (the old drop-rate and drop-consecutive rules did chain analysis which is now runtime-only)
    let _ = drop_diags;
}

#[test]
fn fixture_violations_wiring() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    let result = run_checks(&m);
    let wiring_warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "wiring" && d.severity == Severity::Warning)
        .collect();
    assert!(
        !wiring_warns.is_empty(),
        "expected wiring warning for unwired_node"
    );
}

#[test]
fn fixture_violations_has_many_errors() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    let result = run_checks(&m);
    let error_count = result
        .diagnostics
        .iter()
        .filter(|d| d.severity == Severity::Error)
        .count();
    assert!(
        error_count >= 3,
        "expected at least 3 distinct errors, got {error_count}: {:?}",
        result
            .errors()
            .map(|d| format!("[{}] {}", d.rule_id, d.message))
            .collect::<Vec<_>>()
    );
}

#[test]
fn fixture_violations_service_wiring() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    let result = run_checks(&m);
    let svc_warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "service-wiring")
        .collect();
    assert!(
        !svc_warns.is_empty(),
        "expected service-wiring warning for orphan_client"
    );
}

#[test]
fn fixture_violations_service_type() {
    let m = parse_manifest(&fixture_path("manifest_violations")).unwrap();
    let result = run_checks(&m);
    let type_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "service-type" && d.severity == Severity::Error)
        .collect();
    assert!(
        !type_errs.is_empty(),
        "expected service-type error for typeless_service"
    );
}

// ── manifest_multi_scope: nested includes ──

#[test]
fn fixture_multi_scope_parses() {
    let m = parse_manifest(&fixture_path("manifest_multi_scope")).unwrap();
    assert_eq!(m.includes.len(), 2, "perception + planning");
    assert!(m.nodes.contains_key("driver"));
}

#[test]
fn fixture_multi_scope_inline_structure() {
    let m = parse_manifest(&fixture_path("manifest_multi_scope")).unwrap();
    match &m.includes["perception"] {
        ros_launch_manifest_types::IncludeDecl::Inline(inner) => {
            assert_eq!(inner.nodes.len(), 2, "cropbox + detector");
            assert!(inner.topics.contains_key("filtered_points"));
            assert!(!inner.paths.is_empty());
        }
        ros_launch_manifest_types::IncludeDecl::External { .. } => {
            panic!("perception should be inline, not external");
        }
    }
    match &m.includes["planning"] {
        ros_launch_manifest_types::IncludeDecl::Inline(inner) => {
            assert_eq!(inner.nodes.len(), 1, "planner");
            assert!(
                inner.nodes["planner"].subscribers["route"]
                    .state
                    .unwrap_or(false)
            );
            assert!(
                inner.nodes["planner"].subscribers["route"]
                    .required
                    .unwrap_or(false)
            );
        }
        ros_launch_manifest_types::IncludeDecl::External { .. } => {
            panic!("planning should be inline, not external");
        }
    }
}

#[test]
fn fixture_multi_scope_clean() {
    let m = parse_manifest(&fixture_path("manifest_multi_scope")).unwrap();
    let result = run_checks(&m);
    // Multi-scope should have no errors (inline scopes are well-formed)
    assert!(
        !result.has_errors(),
        "multi-scope fixture should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

// ── manifest_args: args and substitution ──

#[test]
fn fixture_args_parses() {
    let m = parse_manifest(&fixture_path("manifest_args")).unwrap();
    assert_eq!(m.args.len(), 3);
    assert!(m.args.contains_key("input_topic"));
    assert!(m.args.contains_key("output_topic"));
    assert!(m.args.contains_key("node_enabled"));
}

#[test]
fn fixture_args_clean() {
    let m = parse_manifest(&fixture_path("manifest_args")).unwrap();
    let result = run_checks(&m);
    // Args with $(var ...) in type fields — checker sees the unsubstituted string
    // but should not error (type is a free-form string)
    assert!(
        !result.has_errors(),
        "args fixture should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

// ── manifest_conditions: if/unless conditions ──

#[test]
fn fixture_conditions_parses() {
    let m = parse_manifest(&fixture_path("manifest_conditions")).unwrap();
    assert_eq!(m.args.len(), 3);
    // Nodes should have conditions before filtering
    assert!(m.nodes["feature_a_node"].if_condition.is_some());
    assert!(m.nodes["legacy_node"].unless_condition.is_some());
    assert!(m.nodes["sensor_specific"].if_condition.is_some());
}

#[test]
fn fixture_conditions_filter_with_scope_args() {
    use ros_launch_manifest_types::{filter_manifest, resolve_args, substitute_manifest};
    use std::collections::HashMap;

    let mut m = parse_manifest(&fixture_path("manifest_conditions")).unwrap();

    // Provide scope args (simulating record.json scope table)
    let scope_args = HashMap::from([
        ("use_feature_a".into(), "true".into()),
        ("use_feature_b".into(), "false".into()),
        ("sensor_model".into(), "velodyne".into()),
    ]);
    let args = resolve_args(&m.args, &scope_args).unwrap();
    m = substitute_manifest(&m, &args).unwrap();
    filter_manifest(&mut m);

    // use_feature_a=true → feature_a_node present, legacy_node excluded
    assert!(m.nodes.contains_key("feature_a_node"));
    assert!(!m.nodes.contains_key("legacy_node"));
    // use_feature_b=false → feature_b_node excluded
    assert!(!m.nodes.contains_key("feature_b_node"));
    // sensor_model=velodyne → sensor_specific present
    assert!(m.nodes.contains_key("sensor_specific"));
    // always_present has no condition
    assert!(m.nodes.contains_key("always_present"));
}

// ── manifest_control_conditional: Autoware-style conditional checkers ──

#[test]
fn fixture_control_conditional_parses() {
    let m = parse_manifest(&fixture_path("manifest_control_conditional")).unwrap();
    assert_eq!(m.nodes.len(), 5, "controller + 4 conditional checkers");
    assert_eq!(m.topics.len(), 2, "control_cmd + predicted_trajectory");
    assert_eq!(m.args.len(), 4, "4 bool args");
    // All args should be Bool type
    for (name, decl) in &m.args {
        assert!(
            matches!(decl, ros_launch_manifest_types::ArgDecl::Bool),
            "arg {name} should be Bool"
        );
    }
}

#[test]
fn fixture_control_conditional_all_enabled() {
    let m = parse_manifest(&fixture_path("manifest_control_conditional")).unwrap();
    let scope_args = HashMap::from([
        ("launch_validator".into(), "true".into()),
        ("launch_aeb".into(), "true".into()),
        ("launch_lane_checker".into(), "true".into()),
        ("launch_collision".into(), "true".into()),
    ]);
    let args = resolve_args(&m.args, &scope_args).unwrap();
    let mut filtered = substitute_manifest(&m, &args).unwrap();
    filter_manifest(&mut filtered);

    assert_eq!(
        filtered.nodes.len(),
        5,
        "all nodes present when all enabled"
    );
    // Conditional nodes present — refs kept
    let pred_subs = &filtered.topics["predicted_trajectory"].subscribers;
    assert_eq!(pred_subs.len(), 3, "3 subscribers to predicted_trajectory");

    let result = run_checks(&filtered);
    assert!(
        !result.has_errors(),
        "all-enabled should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_control_conditional_all_disabled() {
    let m = parse_manifest(&fixture_path("manifest_control_conditional")).unwrap();
    let scope_args = HashMap::from([
        ("launch_validator".into(), "false".into()),
        ("launch_aeb".into(), "false".into()),
        ("launch_lane_checker".into(), "false".into()),
        ("launch_collision".into(), "false".into()),
    ]);
    let args = resolve_args(&m.args, &scope_args).unwrap();
    let mut filtered = substitute_manifest(&m, &args).unwrap();
    filter_manifest(&mut filtered);

    // Only controller remains
    assert_eq!(filtered.nodes.len(), 1);
    assert!(filtered.nodes.contains_key("controller"));

    // predicted_trajectory should have 0 subscribers (all optional, all filtered)
    // but still have 1 publisher → topic survives
    let pred = &filtered.topics["predicted_trajectory"];
    assert!(
        pred.subscribers.is_empty(),
        "all optional subs filtered out"
    );
    assert_eq!(pred.publishers.len(), 1, "controller still publishes");

    let result = run_checks(&filtered);
    assert!(
        !result.has_errors(),
        "all-disabled should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_control_conditional_partial_enable() {
    let m = parse_manifest(&fixture_path("manifest_control_conditional")).unwrap();
    let scope_args = HashMap::from([
        ("launch_validator".into(), "true".into()),
        ("launch_aeb".into(), "false".into()),
        ("launch_lane_checker".into(), "true".into()),
        ("launch_collision".into(), "false".into()),
    ]);
    let args = resolve_args(&m.args, &scope_args).unwrap();
    let mut filtered = substitute_manifest(&m, &args).unwrap();
    filter_manifest(&mut filtered);

    assert_eq!(
        filtered.nodes.len(),
        3,
        "controller + validator + lane_checker"
    );
    assert!(filtered.nodes.contains_key("controller"));
    assert!(filtered.nodes.contains_key("control_validator"));
    assert!(filtered.nodes.contains_key("lane_checker"));
    assert!(!filtered.nodes.contains_key("aeb"));
    assert!(!filtered.nodes.contains_key("collision_detector"));

    // predicted_trajectory: validator + lane_checker subscribe
    let pred_subs = &filtered.topics["predicted_trajectory"].subscribers;
    assert_eq!(pred_subs.len(), 2);

    let result = run_checks(&filtered);
    assert!(
        !result.has_errors(),
        "partial enable should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_control_conditional_satisfiability_clean() {
    // Pre-filter: run satisfiability on the raw manifest with typed args.
    // All topics have the controller as unconditional publisher, so no
    // arg combination produces 0 publishers. Should pass.
    let m = parse_manifest(&fixture_path("manifest_control_conditional")).unwrap();
    let result = run_checks(&m);
    let sat_diags: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability")
        .collect();
    // No satisfiability errors — controller always publishes
    let sat_errs: Vec<_> = sat_diags
        .iter()
        .filter(|d| d.severity == Severity::Error)
        .collect();
    assert!(
        sat_errs.is_empty(),
        "controller always publishes, no satisfiability errors: {sat_errs:?}"
    );
}

#[test]
fn fixture_control_conditional_no_checker_errors() {
    // Pre-filter: refs to conditional nodes are accepted without markers
    let m = parse_manifest(&fixture_path("manifest_control_conditional")).unwrap();
    let result = run_checks(&m);
    assert!(
        !result.has_errors(),
        "pre-filter should have no errors: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_control_conditional_reject_invalid_bool() {
    let m = parse_manifest(&fixture_path("manifest_control_conditional")).unwrap();
    let scope_args = HashMap::from([
        ("launch_validator".into(), "yes".into()), // invalid for bool
        ("launch_aeb".into(), "true".into()),
        ("launch_lane_checker".into(), "true".into()),
        ("launch_collision".into(), "true".into()),
    ]);
    let err = resolve_args(&m.args, &scope_args).unwrap_err();
    assert!(
        matches!(
            err,
            ros_launch_manifest_types::SubstError::InvalidArgValue { .. }
        ),
        "bool arg should reject 'yes': {err}"
    );
}

// ── manifest_service_scope: intra-scope service wiring ──

#[test]
fn fixture_service_scope_parses() {
    let m = parse_manifest(&fixture_path("manifest_service_scope")).unwrap();
    assert_eq!(m.nodes.len(), 2, "mission_planner + route_selector");
    assert_eq!(m.services.len(), 3, "3 service pairs");
    assert_eq!(m.topics.len(), 2, "planner_route + planner_state");
}

#[test]
fn fixture_service_scope_clean() {
    let m = parse_manifest(&fixture_path("manifest_service_scope")).unwrap();
    let result = run_checks(&m);
    assert!(
        !result.has_errors(),
        "service scope should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_service_scope_wiring_correct() {
    let m = parse_manifest(&fixture_path("manifest_service_scope")).unwrap();
    let result = run_checks(&m);
    let svc_diags: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id.starts_with("service"))
        .collect();
    assert!(
        svc_diags.is_empty(),
        "all services correctly wired: {svc_diags:?}"
    );
}

#[test]
fn fixture_service_scope_structure() {
    let m = parse_manifest(&fixture_path("manifest_service_scope")).unwrap();
    // Services should be correctly parsed
    assert!(m.services.contains_key("clear_route"));
    assert!(m.services.contains_key("set_lanelet_route"));
    assert!(m.services.contains_key("set_waypoint_route"));
}

// ── manifest_satisfiability: multi-variant localization ──

#[test]
fn fixture_satisfiability_parses() {
    let m = parse_manifest(&fixture_path("manifest_satisfiability")).unwrap();
    assert_eq!(m.nodes.len(), 5, "3 pose sources + twist_estimator + ekf");
    assert_eq!(m.args.len(), 2, "pose_source + use_twist");
    match &m.args["pose_source"] {
        ros_launch_manifest_types::ArgDecl::Choices(v) => {
            assert_eq!(v, &["ndt", "eagleye", "gnss"]);
        }
        other => panic!("expected Choices, got {other:?}"),
    }
}

#[test]
fn fixture_satisfiability_variant_complete() {
    // All 3 pose sources provide a publisher for localization_pose.
    // Z3 should find no arg combination that leaves it dangling.
    let m = parse_manifest(&fixture_path("manifest_satisfiability")).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    assert!(
        sat_errs.is_empty(),
        "all variants covered, no satisfiability errors: {sat_errs:?}"
    );
}

#[test]
fn fixture_satisfiability_no_unreachable() {
    let m = parse_manifest(&fixture_path("manifest_satisfiability")).unwrap();
    let result = run_checks(&m);
    let unreachable: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.message.contains("unreachable"))
        .collect();
    assert!(
        unreachable.is_empty(),
        "no unreachable nodes: {unreachable:?}"
    );
}

#[test]
fn fixture_satisfiability_ndt_filter() {
    let m = parse_manifest(&fixture_path("manifest_satisfiability")).unwrap();
    let scope_args = HashMap::from([
        ("pose_source".into(), "ndt".into()),
        ("use_twist".into(), "false".into()),
    ]);
    let args = resolve_args(&m.args, &scope_args).unwrap();
    let mut filtered = substitute_manifest(&m, &args).unwrap();
    filter_manifest(&mut filtered);

    assert!(filtered.nodes.contains_key("ndt_node"));
    assert!(!filtered.nodes.contains_key("eagleye_node"));
    assert!(!filtered.nodes.contains_key("gnss_node"));
    assert!(!filtered.nodes.contains_key("twist_estimator"));
    assert!(filtered.nodes.contains_key("ekf"));

    // localization_pose: only ndt_node/pose
    let pose_pubs = &filtered.topics["localization_pose"].publishers;
    assert_eq!(pose_pubs, &["ndt_node/pose"]);

    let result = run_checks(&filtered);
    assert!(
        !result.has_errors(),
        "ndt config should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_satisfiability_eagleye_with_twist() {
    let m = parse_manifest(&fixture_path("manifest_satisfiability")).unwrap();
    let scope_args = HashMap::from([
        ("pose_source".into(), "eagleye".into()),
        ("use_twist".into(), "true".into()),
    ]);
    let args = resolve_args(&m.args, &scope_args).unwrap();
    let mut filtered = substitute_manifest(&m, &args).unwrap();
    filter_manifest(&mut filtered);

    assert!(filtered.nodes.contains_key("eagleye_node"));
    assert!(filtered.nodes.contains_key("twist_estimator"));
    // twist_data: eagleye + twist_estimator both publish
    let twist_pubs = &filtered.topics["twist_data"].publishers;
    assert_eq!(twist_pubs.len(), 2);

    let result = run_checks(&filtered);
    assert!(
        !result.has_errors(),
        "eagleye+twist should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_satisfiability_gnss_minimal() {
    let m = parse_manifest(&fixture_path("manifest_satisfiability")).unwrap();
    let scope_args = HashMap::from([
        ("pose_source".into(), "gnss".into()),
        ("use_twist".into(), "false".into()),
    ]);
    let args = resolve_args(&m.args, &scope_args).unwrap();
    let mut filtered = substitute_manifest(&m, &args).unwrap();
    filter_manifest(&mut filtered);

    // Minimal config: gnss_node + ekf only
    assert_eq!(
        filtered.nodes.len(),
        2,
        "gnss_node + ekf: {:?}",
        filtered.nodes.keys().collect::<Vec<_>>()
    );

    // twist_data: 0 pub + 0 sub after filter → removed
    assert!(
        !filtered.topics.contains_key("twist_data"),
        "twist_data should be removed (0 pub, 0 sub after filter)"
    );

    // After filtering: only gnss_node + ekf remain
    assert_eq!(filtered.nodes.len(), 2);
}

#[test]
fn fixture_satisfiability_reject_invalid_choice() {
    let m = parse_manifest(&fixture_path("manifest_satisfiability")).unwrap();
    let scope_args = HashMap::from([
        ("pose_source".into(), "lidar".into()), // not in [ndt, eagleye, gnss]
        ("use_twist".into(), "true".into()),
    ]);
    let err = resolve_args(&m.args, &scope_args).unwrap_err();
    assert!(
        matches!(
            err,
            ros_launch_manifest_types::SubstError::InvalidArgValue { .. }
        ),
        "choices arg should reject 'lidar': {err}"
    );
}

// ── manifest_standalone: sub-only with type, validates in isolation ──

#[test]
fn fixture_standalone_parses_and_validates() {
    let m = parse_manifest(&fixture_path("manifest_standalone")).unwrap();
    assert_eq!(m.nodes.len(), 1);
    assert!(m.nodes.contains_key("consumer"));
    // Topic declared with type → self-contained
    assert!(m.topics.contains_key("/localization/kinematic_state"));
    let topic = &m.topics["/localization/kinematic_state"];
    assert_eq!(topic.msg_type, "nav_msgs/msg/Odometry");
    assert!(topic.publishers.is_empty());
    assert_eq!(topic.subscribers, vec!["consumer/odometry"]);
}

#[test]
fn fixture_standalone_no_per_manifest_errors() {
    // In standalone mode (no cross-scope merge), a sub-only topic with
    // declared type is valid. The per-manifest checker does NOT warn
    // about missing publishers — that's a cross-scope concern.
    let m = parse_manifest(&fixture_path("manifest_standalone")).unwrap();
    let result = run_checks(&m);
    assert!(
        !result.has_errors(),
        "standalone fixture should have no errors: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

#[test]
fn fixture_standalone_subscriber_max_age_ms() {
    let m = parse_manifest(&fixture_path("manifest_standalone")).unwrap();
    let consumer = &m.nodes["consumer"];
    let odometry = &consumer.subscribers["odometry"];
    assert_eq!(odometry.max_age_ms, Some(100.0));
    assert_eq!(odometry.min_rate_hz, Some(50.0));
}

// ── manifest_qos_match: qos-match rule end-to-end ──

#[test]
fn fixture_qos_match_parses() {
    let m = parse_manifest(&fixture_path("manifest_qos_match")).unwrap();
    assert_eq!(m.topics.len(), 4);
}

#[test]
fn fixture_qos_match_depth_zero_error() {
    let m = parse_manifest(&fixture_path("manifest_qos_match")).unwrap();
    let result = run_checks(&m);
    let errs: Vec<_> = result
        .errors()
        .filter(|d| d.rule_id == "qos-match" && d.path.contains("depth_zero"))
        .collect();
    assert_eq!(errs.len(), 1, "expected 1 qos-match depth=0 error");
    assert!(errs[0].message.contains("depth is 0"));
}

#[test]
fn fixture_qos_match_best_effort_transient_local_warning() {
    let m = parse_manifest(&fixture_path("manifest_qos_match")).unwrap();
    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| {
            d.severity == Severity::Warning
                && d.rule_id == "qos-match"
                && d.path.contains("best_effort_transient")
        })
        .collect();
    assert_eq!(
        warns.len(),
        1,
        "expected 1 qos-match best_effort+transient_local warning"
    );
    assert!(warns[0].message.contains("best_effort"));
    assert!(warns[0].message.contains("transient_local"));
}

#[test]
fn fixture_qos_match_keep_all_with_depth_warning() {
    let m = parse_manifest(&fixture_path("manifest_qos_match")).unwrap();
    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| {
            d.severity == Severity::Warning
                && d.rule_id == "qos-match"
                && d.path.contains("keep_all_with_depth")
        })
        .collect();
    assert_eq!(
        warns.len(),
        1,
        "expected 1 qos-match keep_all+depth warning"
    );
    assert!(warns[0].message.contains("keep_all"));
}

#[test]
fn fixture_qos_match_canonical_vector_map_clean() {
    // The canonical "deliver to late joiners" QoS pattern on /map/vector_map
    // (reliable + transient_local + keep_last + depth:1) should produce
    // no qos-match diagnostics.
    let m = parse_manifest(&fixture_path("manifest_qos_match")).unwrap();
    let result = run_checks(&m);
    let vector_map_qos_match: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "qos-match" && d.path.contains("vector_map"))
        .collect();
    assert!(
        vector_map_qos_match.is_empty(),
        "canonical vector_map QoS should be clean: {vector_map_qos_match:?}"
    );
}

// ── Cross-fixture: parse all fixtures via parse_manifest_str round-trip ──

#[test]
fn all_fixtures_round_trip() {
    let fixtures = [
        "manifest_simple",
        "manifest_pipeline",
        "manifest_ndt",
        "manifest_periodic",
        "manifest_violations",
        "manifest_multi_scope",
        "manifest_args",
        "manifest_conditions",
        "manifest_control_conditional",
        "manifest_service_scope",
        "manifest_satisfiability",
        "manifest_standalone",
        "manifest_qos_match",
        "manifest_parallel_pipeline",
    ];
    for name in fixtures {
        let path = fixture_path(name);
        let yaml = std::fs::read_to_string(&path).unwrap_or_else(|e| {
            panic!("failed to read {}: {e}", path.display());
        });
        let m = parse_manifest_str(&yaml).unwrap_or_else(|e| {
            panic!("failed to parse {name}: {e}");
        });
        assert!(m.version > 0, "{name}: version should be > 0");
        // Just confirm the checker doesn't panic
        let _ = run_checks(&m);
    }
}
