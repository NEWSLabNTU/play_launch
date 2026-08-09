//! Integration tests for the manifest static checker.

use ros_launch_manifest_check::{Severity, run_checks, run_checks_with_spans};
use ros_launch_manifest_types::{parse_manifest_str, parse_manifest_str_with_spans};

fn errors(yaml: &str) -> Vec<String> {
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    result
        .diagnostics
        .iter()
        .filter(|d| d.severity == Severity::Error)
        .map(|d| format!("[{}] {}", d.rule_id, d.message))
        .collect()
}

fn warnings(yaml: &str) -> Vec<String> {
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    result
        .diagnostics
        .iter()
        .filter(|d| d.severity == Severity::Warning)
        .map(|d| format!("[{}] {}", d.rule_id, d.message))
        .collect()
}

fn all_diags(yaml: &str) -> Vec<String> {
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    result
        .diagnostics
        .iter()
        .map(|d| format!("[{}] {}", d.rule_id, d.message))
        .collect()
}

// ── Clean manifests (no violations) ──

#[test]
fn test_clean_simple() {
    let yaml = r#"
version: 1
nodes:
  talker:
    pub: [chatter]
  listener:
    sub: [chatter]
topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    assert!(
        !result.has_errors(),
        "expected no errors: {:?}",
        errors(yaml)
    );
}

#[test]
fn test_clean_pipeline() {
    let yaml = r#"
version: 1
nodes:
  cropbox:
    pub: [output]
    sub: [input]
    paths:
      main: { input: input, output: [output], max_latency_ms: 5 }
  ground:
    pub: [output]
    sub: [input]
    paths:
      main: { input: input, output: [output], max_latency_ms: 15 }
  centerpoint:
    pub: [objects]
    sub: [pointcloud]
    paths:
      main: { input: pointcloud, output: [objects], max_latency_ms: 30 }
topics:
  cropped:
    type: PointCloud2
    pub: [cropbox/output]
    sub: [ground/input]
  no_ground:
    type: PointCloud2
    pub: [ground/output]
    sub: [centerpoint/pointcloud]
sub:
  raw_data: [cropbox/input]
pub:
  detections: [centerpoint/objects]
paths:
  main:
    input: raw_data
    output: [detections]
    max_latency_ms: 60
"#;
    let _diags = all_diags(yaml);
    // Should have no errors (scope budget 60 >= 5+15+30=50)
    let errs = errors(yaml);
    assert!(errs.is_empty(), "unexpected errors: {errs:?}");
}

// ── Endpoint uniqueness ──

#[test]
fn test_endpoint_duplicate() {
    // Construct a manifest programmatically with duplicate endpoints
    // (parser catches this from YAML, but checker also validates)
    use ros_launch_manifest_types::*;
    let mut node = NodeDecl::default();
    node.publishers
        .insert("data".into(), EndpointProps::default());
    node.cli.insert("data".into(), EndpointProps::default());

    let mut m = Manifest::default();
    m.nodes.insert("relay".into(), node);

    let result = run_checks(&m);
    let errs: Vec<_> = result.errors().map(|d| d.message.clone()).collect();
    assert!(
        errs.iter().any(|e| e.contains("duplicate endpoint")),
        "expected duplicate error: {errs:?}"
    );
}

// ── QoS compatibility ──

#[test]
fn test_qos_invalid_reliability() {
    let yaml = r#"
version: 1
topics:
  pointcloud:
    type: PointCloud2
    qos:
      reliability: maybe
"#;
    let errs = errors(yaml);
    assert!(
        errs.iter().any(|e| e.contains("invalid reliability")),
        "expected QoS error: {errs:?}"
    );
}

#[test]
fn test_qos_valid() {
    let yaml = r#"
version: 1
topics:
  pointcloud:
    type: PointCloud2
    qos:
      reliability: best_effort
      durability: volatile
"#;
    let errs = errors(yaml);
    assert!(errs.is_empty(), "unexpected errors: {errs:?}");
}

// ── qos-match: structural QoS validity ──

#[test]
fn test_qos_match_depth_zero_errors() {
    let yaml = r#"
version: 1
topics:
  pointcloud:
    type: PointCloud2
    qos:
      reliability: reliable
      depth: 0
"#;
    let errs = errors(yaml);
    assert!(
        errs.iter()
            .any(|e| e.contains("[qos-match]") && e.contains("depth is 0")),
        "expected qos-match depth=0 error: {errs:?}"
    );
}

#[test]
fn test_qos_match_keep_all_with_depth_warns() {
    let yaml = r#"
version: 1
topics:
  pointcloud:
    type: PointCloud2
    qos:
      reliability: reliable
      history: keep_all
      depth: 10
"#;
    let warns = warnings(yaml);
    assert!(
        warns
            .iter()
            .any(|w| w.contains("[qos-match]") && w.contains("keep_all") && w.contains("depth")),
        "expected qos-match keep_all+depth warning: {warns:?}"
    );
}

#[test]
fn test_qos_match_best_effort_transient_local_warns() {
    let yaml = r#"
version: 1
topics:
  pointcloud:
    type: PointCloud2
    qos:
      reliability: best_effort
      durability: transient_local
"#;
    let warns = warnings(yaml);
    assert!(
        warns.iter().any(|w| {
            w.contains("[qos-match]") && w.contains("best_effort") && w.contains("transient_local")
        }),
        "expected qos-match best_effort+transient_local warning: {warns:?}"
    );
}

#[test]
fn test_qos_match_clean_profile() {
    // Reliable + transient_local + keep_last + depth=10 is the standard
    // "deliver to late joiners" pattern — should produce no diagnostics.
    let yaml = r#"
version: 1
topics:
  map:
    type: OccupancyGrid
    qos:
      reliability: reliable
      durability: transient_local
      history: keep_last
      depth: 1
"#;
    let errs = errors(yaml);
    let warns = warnings(yaml);
    let qos_match: Vec<_> = warns.iter().filter(|w| w.contains("[qos-match]")).collect();
    assert!(errs.is_empty(), "unexpected errors: {errs:?}");
    assert!(
        qos_match.is_empty(),
        "unexpected qos-match warnings: {qos_match:?}"
    );
}

#[test]
fn test_qos_match_no_qos_no_diagnostics() {
    // Topic without QoS section should not trigger qos-match.
    let yaml = r#"
version: 1
topics:
  data:
    type: String
"#;
    let errs = errors(yaml);
    let warns = warnings(yaml);
    assert!(errs.is_empty(), "unexpected errors: {errs:?}");
    let qos_match: Vec<_> = warns.iter().filter(|w| w.contains("[qos-match]")).collect();
    assert!(
        qos_match.is_empty(),
        "unexpected qos-match warnings: {qos_match:?}"
    );
}

// ── Rate hierarchy ──

#[test]
fn test_rate_pub_below_topic() {
    let yaml = r#"
version: 1
nodes:
  driver:
    pub:
      pointcloud:
        min_rate_hz: 5
topics:
  pointcloud:
    type: PointCloud2
    pub: [driver/pointcloud]
    rate_hz: 10
"#;
    let errs = errors(yaml);
    assert!(
        errs.iter()
            .any(|e| e.contains("min_rate_hz (5) < topic rate_hz (10)")),
        "expected rate hierarchy error: {errs:?}"
    );
}

#[test]
fn test_rate_topic_below_sub() {
    let yaml = r#"
version: 1
nodes:
  consumer:
    sub:
      input:
        min_rate_hz: 15
topics:
  data:
    type: String
    sub: [consumer/input]
    rate_hz: 10
"#;
    let errs = errors(yaml);
    assert!(
        errs.iter()
            .any(|e| e.contains("topic rate_hz (10) < subscriber")),
        "expected rate hierarchy error: {errs:?}"
    );
}

#[test]
fn test_rate_hierarchy_ok() {
    let yaml = r#"
version: 1
nodes:
  driver:
    pub:
      pointcloud:
        min_rate_hz: 15
  consumer:
    sub:
      input:
        min_rate_hz: 5
topics:
  pointcloud:
    type: PointCloud2
    pub: [driver/pointcloud]
    sub: [consumer/input]
    rate_hz: 10
"#;
    let errs = errors(yaml);
    assert!(errs.is_empty(), "unexpected errors: {errs:?}");
}

// ── Scope budget ──

#[test]
fn test_scope_budget_tight() {
    let yaml = r#"
version: 1
nodes:
  a:
    sub: [input]
    pub: [output]
    paths:
      main: { input: input, output: [output], max_latency_ms: 30 }
  b:
    sub: [input]
    pub: [output]
    paths:
      main: { input: input, output: [output], max_latency_ms: 40 }
topics:
  mid:
    type: String
    pub: [a/output]
    sub: [b/input]
paths:
  main:
    input: raw
    output: [out]
    max_latency_ms: 50
"#;
    let warns = warnings(yaml);
    assert!(
        warns.iter().any(|w| w.contains("may be less than sum")),
        "expected budget warning: {warns:?}"
    );
}

// ── Causal DAG ──

#[test]
fn test_causal_dag_ok() {
    let yaml = r#"
version: 1
nodes:
  a:
    pub: [output]
    sub: [input]
  b:
    pub: [output]
    sub: [input]
topics:
  mid:
    type: String
    pub: [a/output]
    sub: [b/input]
"#;
    let errs = errors(yaml);
    let dag_errs: Vec<_> = errs.iter().filter(|e| e.contains("causal-dag")).collect();
    assert!(dag_errs.is_empty(), "unexpected DAG error: {dag_errs:?}");
}

#[test]
fn test_causal_dag_cycle() {
    let yaml = r#"
version: 1
nodes:
  a:
    pub: [output]
    sub: [input]
  b:
    pub: [output]
    sub: [input]
topics:
  a_to_b:
    type: String
    pub: [a/output]
    sub: [b/input]
  b_to_a:
    type: String
    pub: [b/output]
    sub: [a/input]
"#;
    let errs = errors(yaml);
    assert!(
        errs.iter().any(|e| e.contains("cycle")),
        "expected cycle error: {errs:?}"
    );
}

// ── Drop sanity ──

#[test]
fn test_drop_sanity_max_consecutive_zero() {
    let yaml = r#"
version: 1
nodes:
  a:
    paths:
      main:
        input: x
        output: [y]
        drop:
          max_count: 5 / 100
          max_consecutive: 0
"#;
    let errs = errors(yaml);
    assert!(
        errs.iter().any(|e| e.contains("max_consecutive is 0")),
        "expected drop-sanity error for max_consecutive 0: {errs:?}"
    );
}

#[test]
fn test_drop_sanity_valid_values() {
    let yaml = r#"
version: 1
nodes:
  a:
    paths:
      main:
        input: x
        output: [y]
        drop:
          max_count: 2 / 100
          max_consecutive: 3
paths:
  main:
    input: raw
    output: [out]
    drop:
      max_count: 3 / 100
      max_consecutive: 5
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sanity_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "drop-sanity" && d.severity == Severity::Error)
        .collect();
    assert!(sanity_errs.is_empty(), "unexpected errors: {sanity_errs:?}");
}

#[test]
fn test_drop_sanity_effective_rate_insufficient() {
    let yaml = r#"
version: 1
nodes:
  consumer:
    sub:
      data:
        min_rate_hz: 10
topics:
  sensor:
    type: PointCloud2
    pub: []
    sub: [consumer/data]
    rate_hz: 10
    drop:
      max_count: 50 / 100
"#;
    let errs = errors(yaml);
    assert!(
        errs.iter().any(|e| e.contains("effective delivery rate")),
        "expected drop-sanity effective rate error: {errs:?}"
    );
}

// ── Wiring ──

#[test]
fn test_unwired_path_endpoint() {
    let yaml = r#"
version: 1
nodes:
  processor:
    sub: [input]
    pub: [output]
    paths:
      main:
        input: input
        output: [output]
        max_latency_ms: 10
"#;
    let warns = warnings(yaml);
    assert!(
        warns.iter().any(|w| w.contains("not wired")),
        "expected wiring warning: {warns:?}"
    );
}

// ── Full pipeline (Autoware-like) ──

#[test]
fn test_full_perception_pipeline() {
    let yaml = r#"
version: 1
nodes:
  cropbox:
    sub:
      input:
        min_rate_hz: 10
    pub: [output]
    paths:
      main: { input: input, output: [output], max_latency_ms: 5 }
  ground:
    sub: [input]
    pub: [output]
    paths:
      main: { input: input, output: [output], max_latency_ms: 15 }
  centerpoint:
    sub: [pointcloud]
    pub:
      objects:
        min_rate_hz: 10
    paths:
      main:
        input: pointcloud
        output: [objects]
        max_latency_ms: 30
        drop: 5 / 100
topics:
  cropped:
    type: PointCloud2
    pub: [cropbox/output]
    sub: [ground/input]
    rate_hz: 10
  no_ground:
    type: PointCloud2
    pub: [ground/output]
    sub: [centerpoint/pointcloud]
    rate_hz: 10
paths:
  main:
    input: raw_data
    output: [detections]
    max_latency_ms: 60
    drop: 6 / 100
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let errs: Vec<_> = result.errors().collect();
    assert!(
        errs.is_empty(),
        "expected no errors on clean pipeline: {:?}",
        errs.iter().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

// ── Source span tests ──

#[test]
fn test_span_qos_error_has_byte_range() {
    let yaml = r#"version: 1
topics:
  pointcloud:
    type: PointCloud2
    qos:
      reliability: maybe
"#;
    let parsed = parse_manifest_str_with_spans(yaml).unwrap();
    let result = run_checks_with_spans(&parsed.manifest, parsed.spans);

    let qos_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "qos-compat" && d.severity == Severity::Error)
        .collect();
    assert!(!qos_errs.is_empty());

    // At least one diagnostic should have a span
    let with_span: Vec<_> = qos_errs.iter().filter(|d| d.span.is_some()).collect();
    assert!(
        !with_span.is_empty(),
        "expected at least one QoS diagnostic with span, got: {:?}",
        qos_errs
            .iter()
            .map(|d| (&d.path, &d.span))
            .collect::<Vec<_>>()
    );

    // The span should point at actual source text
    let span = with_span[0].span.as_ref().unwrap();
    let source_slice = &yaml[span.clone()];
    assert!(
        !source_slice.is_empty(),
        "span should point at non-empty source text"
    );
}

#[test]
fn test_span_rate_hierarchy_error_has_byte_range() {
    let yaml = r#"version: 1
nodes:
  driver:
    pub:
      pointcloud:
        min_rate_hz: 5
topics:
  pointcloud:
    type: PointCloud2
    pub: [driver/pointcloud]
    rate_hz: 10
"#;
    let parsed = parse_manifest_str_with_spans(yaml).unwrap();
    let result = run_checks_with_spans(&parsed.manifest, parsed.spans);

    let rate_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "rate-hierarchy")
        .collect();
    assert!(!rate_errs.is_empty());

    let with_span: Vec<_> = rate_errs.iter().filter(|d| d.span.is_some()).collect();
    assert!(
        !with_span.is_empty(),
        "expected rate-hierarchy diagnostic with span"
    );
}

#[test]
fn test_span_scope_budget_warning_has_byte_range() {
    let yaml = r#"version: 1
nodes:
  a:
    paths:
      main:
        input: []
        output: [out]
        max_latency_ms: 200
paths:
  main:
    input: raw
    output: [out]
    max_latency_ms: 50
"#;
    let parsed = parse_manifest_str_with_spans(yaml).unwrap();
    let result = run_checks_with_spans(&parsed.manifest, parsed.spans);

    let budget_warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "scope-budget" && d.severity == Severity::Warning)
        .collect();
    assert!(!budget_warns.is_empty());

    let with_span: Vec<_> = budget_warns.iter().filter(|d| d.span.is_some()).collect();
    assert!(
        !with_span.is_empty(),
        "expected scope-budget diagnostic with span"
    );
}

#[test]
fn test_no_spans_without_index() {
    // run_checks (no spans) should produce diagnostics with span=None
    let yaml = r#"version: 1
topics:
  pc:
    type: PointCloud2
    qos:
      reliability: maybe
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);

    assert!(result.has_errors());
    for diag in &result.diagnostics {
        assert!(diag.span.is_none(), "expected no spans without index");
    }
}

// ── Service wiring and type checks ──

#[test]
fn test_service_wiring_clean() {
    let yaml = r#"
version: 1
nodes:
  server_node:
    srv:
      my_service: {}
  client_node:
    cli:
      my_service: {}
services:
  my_service:
    type: std_srvs/srv/Trigger
    server: [server_node/my_service]
    client: [client_node/my_service]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let svc_diags: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id.starts_with("service"))
        .collect();
    assert!(
        svc_diags.is_empty(),
        "expected no service issues: {svc_diags:?}"
    );
}

#[test]
fn test_service_wiring_missing_server() {
    let yaml = r#"
version: 1
nodes:
  client_node:
    cli:
      orphan_service: {}
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "service-wiring")
        .collect();
    assert!(
        !warns.is_empty(),
        "expected service-wiring warning for orphan client"
    );
}

#[test]
fn test_service_type_missing() {
    let yaml = r#"
version: 1
services:
  bad_service:
    server: [node/srv]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "service-type" && d.severity == Severity::Error)
        .collect();
    assert!(
        !errs.is_empty(),
        "expected service-type error for missing type"
    );
}

#[test]
fn test_service_type_server_not_on_node() {
    let yaml = r#"
version: 1
nodes:
  my_node:
    pub: [output]
services:
  my_service:
    type: std_srvs/srv/Trigger
    server: [my_node/nonexistent_srv]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "service-type" && d.severity == Severity::Warning)
        .collect();
    assert!(
        !warns.is_empty(),
        "expected warning for server not found on node"
    );
}

// ── Arg type validation ──

#[test]
fn test_arg_types_bool_valid_values() {
    use ros_launch_manifest_types::{ArgDecl, resolve_args};
    use std::collections::{BTreeMap, HashMap};

    let manifest_args = BTreeMap::from([("flag".into(), ArgDecl::Bool)]);

    // "true" and "false" are valid
    for val in &["true", "false"] {
        let caller = HashMap::from([("flag".into(), val.to_string())]);
        assert!(
            resolve_args(&manifest_args, &caller).is_ok(),
            "{val} should be valid for Bool"
        );
    }

    // "yes", "1", "True", "FALSE" are all invalid
    for val in &["yes", "no", "1", "0", "True", "FALSE", ""] {
        let caller = HashMap::from([("flag".into(), val.to_string())]);
        assert!(
            resolve_args(&manifest_args, &caller).is_err(),
            "'{val}' should be invalid for Bool"
        );
    }
}

#[test]
fn test_arg_types_choices_validation() {
    use ros_launch_manifest_types::{ArgDecl, resolve_args};
    use std::collections::{BTreeMap, HashMap};

    let manifest_args = BTreeMap::from([(
        "mode".into(),
        ArgDecl::Choices(vec!["ndt".into(), "eagleye".into(), "gnss".into()]),
    )]);

    for val in &["ndt", "eagleye", "gnss"] {
        let caller = HashMap::from([("mode".into(), val.to_string())]);
        assert!(resolve_args(&manifest_args, &caller).is_ok());
    }

    let caller = HashMap::from([("mode".into(), "lidar".into())]);
    let err = resolve_args(&manifest_args, &caller).unwrap_err();
    assert!(matches!(
        err,
        ros_launch_manifest_types::SubstError::InvalidArgValue { .. }
    ));
}

#[test]
fn test_arg_types_mixed_free_and_typed() {
    use ros_launch_manifest_types::{ArgDecl, resolve_args};
    use std::collections::{BTreeMap, HashMap};

    let manifest_args = BTreeMap::from([
        ("free_arg".into(), ArgDecl::String),
        ("bool_arg".into(), ArgDecl::Bool),
        (
            "choice_arg".into(),
            ArgDecl::Choices(vec!["a".into(), "b".into()]),
        ),
    ]);

    // All valid
    let caller = HashMap::from([
        ("free_arg".into(), "anything goes here".into()),
        ("bool_arg".into(), "true".into()),
        ("choice_arg".into(), "a".into()),
    ]);
    let resolved = resolve_args(&manifest_args, &caller).unwrap();
    assert_eq!(resolved["free_arg"], "anything goes here");
    assert_eq!(resolved["bool_arg"], "true");
    assert_eq!(resolved["choice_arg"], "a");

    // Bool invalid but others fine → error on bool
    let caller2 = HashMap::from([
        ("free_arg".into(), "ok".into()),
        ("bool_arg".into(), "maybe".into()),
        ("choice_arg".into(), "a".into()),
    ]);
    assert!(resolve_args(&manifest_args, &caller2).is_err());
}

#[test]
fn test_arg_types_parse_all_forms() {
    // List form: all String
    let yaml = "args: [x, y, z]\nversion: 1\n";
    let m = parse_manifest_str(yaml).unwrap();
    assert_eq!(m.args.len(), 3);
    assert!(
        m.args
            .values()
            .all(|d| matches!(d, ros_launch_manifest_types::ArgDecl::String))
    );

    // Map with null: String
    let yaml2 = "args:\n  x:\n  y:\nversion: 1\n";
    let m2 = parse_manifest_str(yaml2).unwrap();
    assert_eq!(m2.args.len(), 2);

    // Map with types
    let yaml3 = r#"
args:
  flag:
    type: bool
  mode:
    choices: [a, b, c]
  name:
version: 1
"#;
    let m3 = parse_manifest_str(yaml3).unwrap();
    assert!(matches!(
        m3.args["flag"],
        ros_launch_manifest_types::ArgDecl::Bool
    ));
    match &m3.args["mode"] {
        ros_launch_manifest_types::ArgDecl::Choices(v) => assert_eq!(v, &["a", "b", "c"]),
        _ => panic!("expected Choices"),
    }
    assert!(matches!(
        m3.args["name"],
        ros_launch_manifest_types::ArgDecl::String
    ));
}

// ── Condition edge cases ──

#[test]
fn test_condition_compound_and_or() {
    use ros_launch_manifest_types::{filter_manifest, resolve_args, substitute_manifest};
    use std::collections::HashMap;

    let yaml = r#"
args:
  x:
  y:
version: 1
nodes:
  both_true:
    if: $(var x) == 'a' and $(var y) == 'b'
    pub: [output]
  either_true:
    if: $(var x) == 'a' or $(var y) == 'b'
    pub: [output]
"#;
    let m = parse_manifest_str(yaml).unwrap();

    // x=a, y=b → both nodes present
    let args = HashMap::from([("x".into(), "a".into()), ("y".into(), "b".into())]);
    let resolved = resolve_args(&m.args, &args).unwrap();
    let mut m1 = substitute_manifest(&m, &resolved).unwrap();
    filter_manifest(&mut m1);
    assert!(m1.nodes.contains_key("both_true"));
    assert!(m1.nodes.contains_key("either_true"));

    // x=a, y=c → only either_true (x == 'a' matches)
    let args2 = HashMap::from([("x".into(), "a".into()), ("y".into(), "c".into())]);
    let resolved2 = resolve_args(&m.args, &args2).unwrap();
    let mut m2 = substitute_manifest(&m, &resolved2).unwrap();
    filter_manifest(&mut m2);
    assert!(!m2.nodes.contains_key("both_true"));
    assert!(m2.nodes.contains_key("either_true"));

    // x=z, y=z → neither
    let args3 = HashMap::from([("x".into(), "z".into()), ("y".into(), "z".into())]);
    let resolved3 = resolve_args(&m.args, &args3).unwrap();
    let mut m3 = substitute_manifest(&m, &resolved3).unwrap();
    filter_manifest(&mut m3);
    assert!(!m3.nodes.contains_key("both_true"));
    assert!(!m3.nodes.contains_key("either_true"));
}

#[test]
fn test_condition_unless_with_expression() {
    use ros_launch_manifest_types::{filter_manifest, resolve_args, substitute_manifest};
    use std::collections::HashMap;

    let yaml = r#"
args:
  mode:
version: 1
nodes:
  legacy:
    unless: $(var mode) == 'new'
    pub: [output]
  modern:
    if: $(var mode) == 'new'
    pub: [output]
"#;
    let m = parse_manifest_str(yaml).unwrap();

    let args = HashMap::from([("mode".into(), "new".into())]);
    let resolved = resolve_args(&m.args, &args).unwrap();
    let mut filtered = substitute_manifest(&m, &resolved).unwrap();
    filter_manifest(&mut filtered);
    assert!(
        !filtered.nodes.contains_key("legacy"),
        "unless should exclude when expr is true"
    );
    assert!(filtered.nodes.contains_key("modern"));

    let args2 = HashMap::from([("mode".into(), "old".into())]);
    let resolved2 = resolve_args(&m.args, &args2).unwrap();
    let mut filtered2 = substitute_manifest(&m, &resolved2).unwrap();
    filter_manifest(&mut filtered2);
    assert!(
        filtered2.nodes.contains_key("legacy"),
        "unless should include when expr is false"
    );
    assert!(!filtered2.nodes.contains_key("modern"));
}

#[test]
fn test_condition_on_service_and_action() {
    use ros_launch_manifest_types::filter_manifest;

    let yaml = r#"
version: 1
nodes:
  server:
    srv:
      my_srv: {}
services:
  conditional_svc:
    if: "false"
    type: std_srvs/srv/Trigger
    server: [server/my_srv]
actions:
  conditional_act:
    unless: "true"
    type: nav2_msgs/action/Navigate
    server: []
"#;
    let mut m = parse_manifest_str(yaml).unwrap();
    filter_manifest(&mut m);
    assert!(
        !m.services.contains_key("conditional_svc"),
        "service if=false → filtered"
    );
    assert!(
        !m.actions.contains_key("conditional_act"),
        "action unless=true → filtered"
    );
}

#[test]
fn test_condition_parenthesized() {
    use ros_launch_manifest_types::{filter_manifest, resolve_args, substitute_manifest};
    use std::collections::HashMap;

    let yaml = r#"
args:
  a:
  b:
  c:
version: 1
nodes:
  complex:
    if: ($(var a) == 'x' or $(var b) == 'y') and $(var c) == 'z'
    pub: [output]
"#;
    let m = parse_manifest_str(yaml).unwrap();

    // a=x, b=n, c=z → (true or false) and true → true
    let args = HashMap::from([
        ("a".into(), "x".into()),
        ("b".into(), "n".into()),
        ("c".into(), "z".into()),
    ]);
    let resolved = resolve_args(&m.args, &args).unwrap();
    let mut m1 = substitute_manifest(&m, &resolved).unwrap();
    filter_manifest(&mut m1);
    assert!(m1.nodes.contains_key("complex"));

    // a=n, b=n, c=z → (false or false) and true → false
    let args2 = HashMap::from([
        ("a".into(), "n".into()),
        ("b".into(), "n".into()),
        ("c".into(), "z".into()),
    ]);
    let resolved2 = resolve_args(&m.args, &args2).unwrap();
    let mut m2 = substitute_manifest(&m, &resolved2).unwrap();
    filter_manifest(&mut m2);
    assert!(!m2.nodes.contains_key("complex"));
}

#[test]
fn test_condition_on_scope_path() {
    use ros_launch_manifest_types::filter_manifest;

    let yaml = r#"
version: 1
nodes:
  a:
    sub: [input]
    pub: [output]
paths:
  debug_path:
    if: "false"
    input: raw
    output: [out]
    max_latency_ms: 100
  active_path:
    if: "true"
    input: raw
    output: [out]
    max_latency_ms: 50
"#;
    let mut m = parse_manifest_str(yaml).unwrap();
    filter_manifest(&mut m);
    assert!(!m.paths.contains_key("debug_path"));
    assert!(m.paths.contains_key("active_path"));
    // Condition cleared on surviving path
    assert!(m.paths["active_path"].if_condition.is_none());
}

// Scope interface tests removed — scope_pub, scope_sub, scope_srv, scope_cli,
// action_server, action_client fields no longer exist on Manifest.

// ── Dangling entity edge cases ──

#[test]
fn test_dangling_action_no_server() {
    let yaml = r#"
version: 1
nodes:
  client:
    cli:
      navigate: {}
actions:
  navigate:
    type: nav2_msgs/action/NavigateToPose
    server: []
    client: [client/navigate]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "dangling-entity" && d.severity == Severity::Error)
        .collect();
    assert!(!errs.is_empty(), "action with no server should be error");
    assert!(
        errs[0].message.contains("no server"),
        "message should mention no server: {}",
        errs[0].message
    );
}

#[test]
fn test_dangling_cascading_from_condition_filter() {
    // After filtering a conditional node, a topic loses its only publisher.
    use ros_launch_manifest_types::filter_manifest;

    let yaml = r#"
version: 1
nodes:
  optional_pub:
    if: "false"
    pub: [data]
  consumer:
    sub: [input]
topics:
  sensor:
    type: sensor_msgs/msg/PointCloud2
    pub: [optional_pub/data]
    sub: [consumer/input]
"#;
    let mut m = parse_manifest_str(yaml).unwrap();
    filter_manifest(&mut m);

    // optional_pub filtered → topic has 0 pub, 1 sub
    assert!(
        m.topics.contains_key("sensor"),
        "topic survives (has subscriber)"
    );
    assert!(m.topics["sensor"].publishers.is_empty());

    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "dangling-entity" && d.message.contains("no publishers"))
        .collect();
    assert!(
        !warns.is_empty(),
        "should warn about 0 publishers after filter"
    );
}

#[test]
fn test_dangling_service_cascading() {
    // Filtered node removes server from service → dangling service
    use ros_launch_manifest_types::filter_manifest;

    let yaml = r#"
version: 1
nodes:
  server_node:
    if: "false"
    srv:
      trigger: {}
  client_node:
    cli:
      trigger: {}
services:
  trigger_svc:
    type: std_srvs/srv/Trigger
    server: [server_node/trigger]
    client: [client_node/trigger]
"#;
    let mut m = parse_manifest_str(yaml).unwrap();
    filter_manifest(&mut m);

    // Service still exists (client present) but server gone
    assert!(m.services.contains_key("trigger_svc"));
    assert!(m.services["trigger_svc"].server.is_empty());

    let result = run_checks(&m);
    let errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "dangling-entity" && d.severity == Severity::Error)
        .collect();
    assert!(!errs.is_empty(), "service with 0 servers should be error");
}

#[test]
fn test_dangling_empty_service_removed() {
    use ros_launch_manifest_types::filter_manifest;

    let yaml = r#"
version: 1
nodes:
  opt_server:
    if: "false"
    srv:
      svc: {}
  opt_client:
    if: "false"
    cli:
      svc: {}
services:
  gone_svc:
    type: std_srvs/srv/Trigger
    server: [opt_server/svc]
    client: [opt_client/svc]
"#;
    let mut m = parse_manifest_str(yaml).unwrap();
    filter_manifest(&mut m);

    assert!(
        !m.services.contains_key("gone_svc"),
        "service with 0 server + 0 client should be removed"
    );
}

#[test]
fn test_dangling_empty_action_removed() {
    use ros_launch_manifest_types::filter_manifest;

    let yaml = r#"
version: 1
nodes:
  opt_server:
    if: "false"
    srv:
      act: {}
  opt_client:
    if: "false"
    cli:
      act: {}
actions:
  gone_act:
    type: nav2_msgs/action/Navigate
    server: [opt_server/act]
    client: [opt_client/act]
"#;
    let mut m = parse_manifest_str(yaml).unwrap();
    filter_manifest(&mut m);

    assert!(
        !m.actions.contains_key("gone_act"),
        "action with 0 server + 0 client should be removed"
    );
}

// ── Satisfiability edge cases ──

#[test]
fn test_satisfiability_no_finite_args_skipped() {
    // Only free (String) args — satisfiability rule should skip silently
    let yaml = r#"
version: 1
args:
  topic_name:
nodes:
  a:
    if: $(var topic_name) == 'foo'
    pub: [out]
  b:
    sub: [in_data]
topics:
  data:
    type: std_msgs/msg/String
    pub: [a/out]
    sub: [b/in_data]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability")
        .collect();
    assert!(
        sat.is_empty(),
        "no finite args → satisfiability rule should produce no diagnostics"
    );
}

#[test]
fn test_satisfiability_service_all_optional_servers() {
    // Service where all servers are on conditional nodes.
    // If no config activates a server, that's an error.
    let yaml = r#"
version: 1
args:
  mode:
    choices: [a, b]
nodes:
  server_a:
    if: $(var mode) == 'a'
    srv:
      trigger: {}
  client:
    cli:
      trigger: {}
services:
  trigger_svc:
    type: std_srvs/srv/Trigger
    server: [server_a/trigger]
    client: [client/trigger]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    // mode=b → server_a filtered → 0 servers
    assert!(
        !sat_errs.is_empty(),
        "mode=b leaves service with 0 servers: {sat_errs:?}"
    );
    assert!(
        sat_errs[0].message.contains("mode=b"),
        "should mention mode=b: {}",
        sat_errs[0].message
    );
}

#[test]
fn test_satisfiability_action_all_optional_servers() {
    let yaml = r#"
version: 1
args:
  use_nav:
    type: bool
nodes:
  nav_server:
    if: $(var use_nav)
    srv:
      navigate: {}
  client:
    cli:
      navigate: {}
actions:
  navigate:
    type: nav2_msgs/action/NavigateToPose
    server: [nav_server/navigate]
    client: [client/navigate]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    // use_nav=false → nav_server filtered → 0 servers
    assert!(
        !sat_errs.is_empty(),
        "use_nav=false leaves action with 0 servers"
    );
    assert!(
        sat_errs[0].message.contains("use_nav=false"),
        "should mention use_nav=false: {}",
        sat_errs[0].message
    );
}

#[test]
fn test_satisfiability_unconditional_publisher_prevents_error() {
    // One unconditional + optional publishers → always at least 1 pub → no error
    let yaml = r#"
version: 1
args:
  extra:
    type: bool
nodes:
  always_pub:
    pub: [data]
  extra_pub:
    if: $(var extra)
    pub: [data]
  consumer:
    sub: [input]
topics:
  stream:
    type: std_msgs/msg/String
    pub:
      - always_pub/data
      - extra_pub/data
    sub: [consumer/input]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    assert!(
        sat_errs.is_empty(),
        "unconditional publisher prevents dangling: {sat_errs:?}"
    );
}

#[test]
fn test_satisfiability_unreachable_unless_always_true() {
    // unless condition that's always true → node unreachable
    let yaml = r#"
version: 1
args:
  flag:
    type: bool
nodes:
  always_there:
    pub: [out]
  never_there:
    unless: $(var flag) == 'true' or $(var flag) == 'false'
    pub: [out]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.message.contains("unreachable"))
        .collect();
    assert!(
        !warns.is_empty(),
        "unless (always true) → unreachable: {:?}",
        result
            .diagnostics
            .iter()
            .filter(|d| d.rule_id == "satisfiability")
            .map(|d| &d.message)
            .collect::<Vec<_>>()
    );
}

#[test]
fn test_satisfiability_multiple_choices_cross_product() {
    // Two choice args create a cross product. Only one combination is bad.
    let yaml = r#"
version: 1
args:
  sensor:
    choices: [lidar, camera]
  mode:
    choices: [fast, accurate]
nodes:
  lidar_fast:
    if: $(var sensor) == 'lidar' and $(var mode) == 'fast'
    pub: [result]
  lidar_accurate:
    if: $(var sensor) == 'lidar' and $(var mode) == 'accurate'
    pub: [result]
  camera_fast:
    if: $(var sensor) == 'camera' and $(var mode) == 'fast'
    pub: [result]
  # Missing: camera + accurate
  consumer:
    sub: [input]
topics:
  detection:
    type: std_msgs/msg/String
    pub:
      - lidar_fast/result
      - lidar_accurate/result
      - camera_fast/result
    sub: [consumer/input]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    assert!(!sat_errs.is_empty(), "camera+accurate has 0 publishers");
    // Error message should mention the specific failing combination
    let msg = &sat_errs[0].message;
    assert!(
        msg.contains("sensor=camera") && msg.contains("mode=accurate"),
        "should report camera+accurate: {msg}"
    );
}

#[test]
fn test_satisfiability_bool_and_choices_mixed() {
    // Bool + choices args. The bool controls an extra node.
    let yaml = r#"
version: 1
args:
  backend:
    choices: [gpu, cpu]
  use_fallback:
    type: bool
nodes:
  gpu_detector:
    if: $(var backend) == 'gpu'
    pub: [objects]
  cpu_detector:
    if: $(var backend) == 'cpu'
    pub: [objects]
  fallback:
    if: $(var use_fallback)
    pub: [objects]
  tracker:
    sub: [input]
topics:
  detections:
    type: std_msgs/msg/String
    pub:
      - gpu_detector/objects
      - cpu_detector/objects
      - fallback/objects
    sub: [tracker/input]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    // Every combination has at least 1 pub:
    // gpu+false=gpu_detector, gpu+true=gpu_detector+fallback,
    // cpu+false=cpu_detector, cpu+true=cpu_detector+fallback
    assert!(
        sat_errs.is_empty(),
        "all combinations have a publisher: {sat_errs:?}"
    );
}

#[test]
fn test_satisfiability_unreachable_impossible_value() {
    // Condition references a value not in the enum domain
    let yaml = r#"
version: 1
args:
  mode:
    choices: [a, b]
nodes:
  normal:
    if: $(var mode) == 'a'
    pub: [out]
  impossible:
    if: $(var mode) == 'c'
    pub: [out]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.message.contains("unreachable"))
        .collect();
    assert!(
        warns.iter().any(|w| w.message.contains("impossible")),
        "node with mode=='c' should be unreachable when choices are [a,b]: {warns:?}"
    );
    // normal node should NOT be unreachable
    assert!(
        !warns.iter().any(|w| w.message.contains("normal")),
        "normal node should be reachable"
    );
}

#[test]
fn test_satisfiability_not_equal_condition() {
    // != condition — satisfiability rule should handle it
    let yaml = r#"
version: 1
args:
  mode:
    choices: [ndt, eagleye]
nodes:
  not_ndt:
    if: $(var mode) != 'ndt'
    pub: [pose]
  consumer:
    sub: [input]
topics:
  pose:
    type: geometry_msgs/msg/PoseStamped
    pub: [not_ndt/pose]
    sub: [consumer/input]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    // mode=ndt → not_ndt filtered → 0 publishers
    assert!(
        !sat_errs.is_empty(),
        "mode=ndt leaves topic with 0 publishers"
    );
    assert!(
        sat_errs[0].message.contains("mode=ndt"),
        "should mention mode=ndt: {}",
        sat_errs[0].message
    );
}

// ── State-only subscriber skip in satisfiability ──

#[test]
fn test_satisfiability_skips_state_only_subscribers() {
    // All subscribers are state: true (not required) — 0 publishers is fine
    let yaml = r#"
version: 1
args:
  use_twist:
    type: bool
nodes:
  twist_estimator:
    if: $(var use_twist)
    pub: [twist]
  ekf:
    sub:
      twist_input:
        state: true
topics:
  twist_data:
    type: geometry_msgs/msg/TwistStamped
    pub: [twist_estimator/twist]
    sub: [ekf/twist_input]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    // use_twist=false → 0 publishers, but ekf/twist_input is state-only → skip
    assert!(
        sat_errs.is_empty(),
        "state-only subscriber should not trigger satisfiability error: {sat_errs:?}"
    );
}

#[test]
fn test_satisfiability_state_required_still_errors() {
    // Subscriber is state: true BUT required: true — needs at least one message
    let yaml = r#"
version: 1
args:
  use_map:
    type: bool
nodes:
  map_loader:
    if: $(var use_map)
    pub: [map]
  ndt:
    sub:
      map:
        state: true
        required: true
topics:
  pointcloud_map:
    type: sensor_msgs/msg/PointCloud2
    pub: [map_loader/map]
    sub: [ndt/map]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    // use_map=false → 0 publishers, ndt/map is required → error
    assert!(
        !sat_errs.is_empty(),
        "state+required subscriber should still trigger error"
    );
}

#[test]
fn test_satisfiability_mixed_state_and_causal_subs() {
    // One state-only sub + one causal sub → not all state-only → still checked
    let yaml = r#"
version: 1
args:
  use_extra:
    type: bool
nodes:
  extra_pub:
    if: $(var use_extra)
    pub: [data]
  causal_consumer:
    sub:
      input: {}
  state_consumer:
    sub:
      input:
        state: true
topics:
  stream:
    type: std_msgs/msg/String
    pub: [extra_pub/data]
    sub:
      - causal_consumer/input
      - state_consumer/input
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    // causal_consumer/input is not state → topic is still checked
    assert!(
        !sat_errs.is_empty(),
        "mixed state+causal subs should still be checked"
    );
}

// ── Conditional ref inference for services ──

#[test]
fn test_conditional_service_server_inferred() {
    // Conditional server ref — no ? needed, optionality inferred from node condition
    use ros_launch_manifest_types::filter_manifest;

    let yaml = r#"
version: 1
nodes:
  cond_server:
    if: "false"
    srv:
      trigger: {}
  client:
    cli:
      trigger: {}
services:
  svc:
    type: std_srvs/srv/Trigger
    server: [cond_server/trigger]
    client: [client/trigger]
"#;
    let mut m = parse_manifest_str(yaml).unwrap();
    filter_manifest(&mut m);

    // cond_server filtered → server ref silently dropped
    assert!(m.services["svc"].server.is_empty());
    // client ref kept (unconditional node)
    assert_eq!(m.services["svc"].client.len(), 1);
}

// ── Substitution in scope interface ──

// test_substitution_in_scope_interface removed — scope interface fields no longer exist.

// ── Full pipeline: substitute → filter → check ──

#[test]
fn test_full_pipeline_substitute_filter_check() {
    use ros_launch_manifest_types::{filter_manifest, resolve_args, substitute_manifest};
    use std::collections::HashMap;

    let yaml = r#"
args:
  mode:
    choices: [ndt, eagleye]
  use_twist:
    type: bool

version: 1
nodes:
  ndt_node:
    if: $(var mode) == 'ndt'
    pub: [pose]
    sub: [pointcloud]
    paths:
      main: { input: pointcloud, output: [pose], max_latency_ms: 30 }
  eagleye_node:
    if: $(var mode) == 'eagleye'
    pub: [pose]
    sub: [gnss]
    paths:
      main: { input: gnss, output: [pose], max_latency_ms: 15 }
  twist_node:
    if: $(var use_twist)
    pub: [twist]
    sub: [imu]
  ekf:
    sub:
      pose_in: {}
      twist_in:
        state: true
    pub: [output]
    paths:
      main: { input: pose_in, output: [output], max_latency_ms: 5 }

topics:
  pose:
    type: geometry_msgs/msg/PoseStamped
    pub:
      - ndt_node/pose
      - eagleye_node/pose
    sub: [ekf/pose_in]
    rate_hz: 10
  twist:
    type: geometry_msgs/msg/TwistStamped
    pub: [twist_node/twist]
    sub: []

paths:
  main:
    input: pointcloud
    output: [localization]
    max_latency_ms: 40
"#;

    let m = parse_manifest_str(yaml).unwrap();

    // Test ndt+twist
    let args1 = HashMap::from([
        ("mode".into(), "ndt".into()),
        ("use_twist".into(), "true".into()),
    ]);
    let resolved1 = resolve_args(&m.args, &args1).unwrap();
    let mut m1 = substitute_manifest(&m, &resolved1).unwrap();
    filter_manifest(&mut m1);

    assert_eq!(m1.nodes.len(), 3, "ndt + twist + ekf");
    let result1 = run_checks(&m1);
    assert!(
        !result1.has_errors(),
        "ndt+twist should be clean: {:?}",
        result1.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );

    // Test eagleye without twist
    let args2 = HashMap::from([
        ("mode".into(), "eagleye".into()),
        ("use_twist".into(), "false".into()),
    ]);
    let resolved2 = resolve_args(&m.args, &args2).unwrap();
    let mut m2 = substitute_manifest(&m, &resolved2).unwrap();
    filter_manifest(&mut m2);

    assert_eq!(m2.nodes.len(), 2, "eagleye + ekf");
    // twist topic: 0 pub + 0 sub after filter → removed
    assert!(
        !m2.topics.contains_key("twist"),
        "twist topic removed (0 pub, 0 sub)"
    );

    let result2 = run_checks(&m2);
    assert!(
        !result2.has_errors(),
        "eagleye without twist should be clean: {:?}",
        result2.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );

    // Pre-filter satisfiability should pass — pose topic is variant-complete,
    // twist topic has 0 subscribers so satisfiability doesn't check it
    let result_raw = run_checks(&m);
    let sat_errs: Vec<_> = result_raw
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    assert!(sat_errs.is_empty(), "variant-complete: {sat_errs:?}");
}

// ── Combined args + conditions ──

#[test]
fn test_args_conditions_combined() {
    use ros_launch_manifest_types::{filter_manifest, resolve_args, substitute_manifest};
    use std::collections::HashMap;

    let yaml = r#"
args:
  use_sensor:
  sensor_topic:

version: 1
nodes:
  sensor_driver:
    if: $(var use_sensor)
    pub:
      pointcloud:
        min_rate_hz: 10
  processor:
    sub: [input]
    pub: [output]
topics:
  sensor_data:
    if: $(var use_sensor)
    type: $(var sensor_topic)
    pub: [sensor_driver/pointcloud]
    sub: [processor/input]
    rate_hz: 10
"#;
    let m = parse_manifest_str(yaml).unwrap();

    // Case 1: use_sensor=true — sensor_driver and sensor_data present
    let scope_args = HashMap::from([
        ("use_sensor".into(), "true".into()),
        ("sensor_topic".into(), "sensor_msgs/msg/PointCloud2".into()),
    ]);
    let args = resolve_args(&m.args, &scope_args).unwrap();
    let mut m1 = substitute_manifest(&m, &args).unwrap();
    filter_manifest(&mut m1);

    assert!(m1.nodes.contains_key("sensor_driver"));
    assert!(m1.nodes.contains_key("processor"));
    assert!(m1.topics.contains_key("sensor_data"));
    assert_eq!(
        m1.topics["sensor_data"].msg_type,
        "sensor_msgs/msg/PointCloud2"
    );

    let result = run_checks(&m1);
    assert!(
        !result.has_errors(),
        "case 1 (sensor enabled) should be clean: {:?}",
        result.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );

    // Case 2: use_sensor=false — sensor_driver and sensor_data filtered out
    let scope_args2 = HashMap::from([
        ("use_sensor".into(), "false".into()),
        ("sensor_topic".into(), "sensor_msgs/msg/PointCloud2".into()),
    ]);
    let args2 = resolve_args(&m.args, &scope_args2).unwrap();
    let mut m2 = substitute_manifest(&m, &args2).unwrap();
    filter_manifest(&mut m2);

    assert!(!m2.nodes.contains_key("sensor_driver"));
    assert!(m2.nodes.contains_key("processor"));
    assert!(!m2.topics.contains_key("sensor_data"));

    let result2 = run_checks(&m2);
    assert!(
        !result2.has_errors(),
        "case 2 (sensor disabled) should be clean: {:?}",
        result2.errors().map(|d| d.to_string()).collect::<Vec<_>>()
    );
}

// ── Dangling entity checks ──

#[test]
fn test_dangling_topic_no_pub() {
    let yaml = r#"
version: 1
nodes:
  consumer:
    sub: [input]
topics:
  data:
    type: std_msgs/msg/String
    pub: []
    sub: [consumer/input]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "dangling-entity" && d.message.contains("no publishers"))
        .collect();
    assert!(!warns.is_empty(), "expected warning for topic with no pub");
}

#[test]
fn test_dangling_topic_no_sub() {
    let yaml = r#"
version: 1
nodes:
  producer:
    pub: [output]
topics:
  data:
    type: std_msgs/msg/String
    pub: [producer/output]
    sub: []
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "dangling-entity" && d.message.contains("no subscribers"))
        .collect();
    assert!(!warns.is_empty(), "expected warning for topic with no sub");
}

#[test]
fn test_dangling_service_no_server() {
    let yaml = r#"
version: 1
nodes:
  caller:
    cli:
      my_service: {}
services:
  my_service:
    type: std_srvs/srv/Trigger
    server: []
    client: [caller/my_service]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "dangling-entity" && d.severity == Severity::Error)
        .collect();
    assert!(
        !errs.is_empty(),
        "expected error for service with no server"
    );
}

#[test]
fn test_dangling_topic_both_empty_removed() {
    // After filtering, if both pub and sub are empty, the topic should be removed
    // by cleanup. The dangling rule shouldn't see it.
    use ros_launch_manifest_types::filter_manifest;

    let yaml = r#"
version: 1
nodes:
  optional_pub:
    if: "false"
    pub: [output]
  optional_sub:
    if: "false"
    sub: [input]
topics:
  data:
    type: std_msgs/msg/String
    pub: [optional_pub/output]
    sub: [optional_sub/input]
"#;
    let mut m = parse_manifest_str(yaml).unwrap();
    filter_manifest(&mut m);

    // Topic should have been removed (both sides empty after filtering)
    assert!(
        !m.topics.contains_key("data"),
        "empty topic should be removed by cleanup"
    );
}

// ── Satisfiability checks ──

#[test]
fn test_satisfiability_variant_complete() {
    // Two variants of pose_source — each provides a publisher. All configs are safe.
    let yaml = r#"
version: 1
args:
  pose_source:
    choices: [ndt, eagleye]
nodes:
  ndt_node:
    if: $(var pose_source) == 'ndt'
    pub: [pose]
  eagleye_node:
    if: $(var pose_source) == 'eagleye'
    pub: [pose]
  consumer:
    sub: [pose_input]
topics:
  localization_pose:
    type: geometry_msgs/msg/PoseStamped
    pub:
      - ndt_node/pose
      - eagleye_node/pose
    sub: [consumer/pose_input]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    assert!(
        sat_errs.is_empty(),
        "variant-complete manifest should have no satisfiability errors: {sat_errs:?}"
    );
}

#[test]
fn test_satisfiability_variant_incomplete() {
    // Three choices but only two nodes — gnss has no publisher.
    let yaml = r#"
version: 1
args:
  pose_source:
    choices: [ndt, eagleye, gnss]
nodes:
  ndt_node:
    if: $(var pose_source) == 'ndt'
    pub: [pose]
  eagleye_node:
    if: $(var pose_source) == 'eagleye'
    pub: [pose]
  consumer:
    sub: [pose_input]
topics:
  localization_pose:
    type: geometry_msgs/msg/PoseStamped
    pub:
      - ndt_node/pose
      - eagleye_node/pose
    sub: [consumer/pose_input]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    assert!(
        !sat_errs.is_empty(),
        "variant-incomplete should report satisfiability error"
    );
    // Should mention gnss
    assert!(
        sat_errs[0].message.contains("pose_source=gnss"),
        "error should mention gnss: {}",
        sat_errs[0].message
    );
}

#[test]
fn test_satisfiability_unreachable_node() {
    // Bool arg but condition compares to invalid value — always false.
    let yaml = r#"
version: 1
args:
  flag:
    type: bool
nodes:
  normal_node:
    pub: [output]
  unreachable_node:
    if: $(var flag) == 'wtf'
    pub: [output]
topics:
  data:
    type: std_msgs/msg/String
    pub: [normal_node/output]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let warns: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.message.contains("unreachable"))
        .collect();
    assert!(
        !warns.is_empty(),
        "should detect unreachable node: {:?}",
        result
            .diagnostics
            .iter()
            .filter(|d| d.rule_id == "satisfiability")
            .map(|d| d.message.as_str())
            .collect::<Vec<_>>()
    );
}

#[test]
fn test_satisfiability_bool_args() {
    // Two bool flags controlling two nodes — both present in at least one config.
    let yaml = r#"
version: 1
args:
  use_a:
    type: bool
  use_b:
    type: bool
nodes:
  node_a:
    if: $(var use_a)
    pub: [out_a]
  node_b:
    if: $(var use_b)
    pub: [out_b]
  always:
    sub: [in]
topics:
  data_a:
    type: std_msgs/msg/String
    pub: [node_a/out_a]
    sub: [always/in]
"#;
    let m = parse_manifest_str(yaml).unwrap();
    let result = run_checks(&m);
    let sat_errs: Vec<_> = result
        .diagnostics
        .iter()
        .filter(|d| d.rule_id == "satisfiability" && d.severity == Severity::Error)
        .collect();
    // When use_a=false, topic data_a has 0 pub but 1 sub — error
    assert!(
        !sat_errs.is_empty(),
        "should detect dangling when use_a=false"
    );
}
