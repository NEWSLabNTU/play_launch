//! nano-ros #276 — `params_files` YAML projects into concrete parameters.

use ros_launch_manifest_model::{NodeInstance, ParamSource, ParamValue};

fn node_with(files: Vec<String>, inline: &[(&str, ParamValue)]) -> NodeInstance {
    let mut n = NodeInstance {
        params_files: files,
        ..Default::default()
    };
    for (k, v) in inline {
        n.params.insert((*k).to_string(), v.clone());
    }
    n
}

#[test]
fn wildcard_and_fqn_sections_project_with_inline_winning() {
    let yaml = r#"
/**:
  ros__parameters:
    shared_rate: 10
    qos:
      depth: 5
/ctrl/planner:
  ros__parameters:
    target_acceleration: 1.5
    mode: "sport"
    gains: [1.0, 2.0]
    enabled: true
"#;
    let n = node_with(
        vec![yaml.to_string()],
        &[("mode", ParamValue::Str("inline-wins".into()))],
    );
    let p = n.resolved_params("/ctrl/planner");

    // wildcard section applies
    assert_eq!(p.get("shared_rate"), Some(&ParamValue::Int(10)));
    // nested maps flatten with dots
    assert_eq!(p.get("qos.depth"), Some(&ParamValue::Int(5)));
    // node-specific section applies, with types preserved
    assert_eq!(p.get("target_acceleration"), Some(&ParamValue::Float(1.5)));
    assert_eq!(p.get("enabled"), Some(&ParamValue::Bool(true)));
    assert_eq!(
        p.get("gains"),
        Some(&ParamValue::StrList(vec!["1.0".into(), "2.0".into()]))
    );
    // inline `<param>` beats the file
    assert_eq!(p.get("mode"), Some(&ParamValue::Str("inline-wins".into())));
}

#[test]
fn later_file_overrides_earlier_and_foreign_sections_are_ignored() {
    let a = "/**:\n  ros__parameters:\n    rate: 1\n";
    let b = "planner:\n  ros__parameters:\n    rate: 2\n";
    let other = "/other/node:\n  ros__parameters:\n    rate: 99\n";
    let n = node_with(vec![a.to_string(), b.to_string(), other.to_string()], &[]);
    let p = n.resolved_params("/ctrl/planner");
    // bare-name section matches; later file wins; the foreign node is skipped
    assert_eq!(p.get("rate"), Some(&ParamValue::Int(2)));
}

#[test]
fn unparsable_file_is_skipped_not_fatal() {
    let n = node_with(vec!["{{{ not yaml".to_string()], &[]);
    assert!(n.resolved_params("/x").is_empty());
}

#[test]
fn rcl_style_wildcard_sections_match() {
    // Partial wildcards are what real Autoware configs use; rcl's
    // `rcl_yaml_param_parser` accepts `**` (any depth) and `*` (one segment).
    let yaml = r#"
/sensing/**:
  ros__parameters:
    from_subtree: 1
/*/planner:
  ros__parameters:
    from_single_seg: 2
/other/**:
  ros__parameters:
    must_not_apply: 3
"#;
    let n = node_with(vec![yaml.to_string()], &[]);

    let deep = n.resolved_params("/sensing/lidar/driver");
    assert_eq!(deep.get("from_subtree"), Some(&ParamValue::Int(1)));
    assert!(!deep.contains_key("must_not_apply"), "{deep:?}");

    let one = n.resolved_params("/ctrl/planner");
    assert_eq!(one.get("from_single_seg"), Some(&ParamValue::Int(2)));
    assert!(!one.contains_key("from_subtree"), "{one:?}");

    // `**` also matches zero segments: /sensing itself.
    let zero = n.resolved_params("/sensing");
    assert_eq!(zero.get("from_subtree"), Some(&ParamValue::Int(1)));

    // A single-segment wildcard must NOT span two segments.
    let too_deep = n.resolved_params("/a/b/planner");
    assert!(!too_deep.contains_key("from_single_seg"), "{too_deep:?}");
}

// ---------------------------------------------------------------------------
// phase-54 / play_launch issue 0007 — the ORDERED source list.
// ---------------------------------------------------------------------------

const A_IS_2: &str = r#"
/**:
  ros__parameters:
    a: 2
"#;

fn ordered(sources: Vec<ParamSource>) -> NodeInstance {
    NodeInstance {
        param_sources: sources,
        ..Default::default()
    }
}

fn inline_src(name: &str, value: ParamValue) -> ParamSource {
    ParamSource::Inline {
        name: name.to_string(),
        value,
    }
}

#[test]
fn inline_then_file_lets_the_file_win() {
    // <param name="a" value="1"/> then <param from="a_is_2.yaml"/> — ROS emits
    // both as --params-file in order, so the FILE wins.
    let n = ordered(vec![
        inline_src("a", ParamValue::Int(1)),
        ParamSource::File {
            content: A_IS_2.to_string(),
        },
    ]);
    let out = n.resolved_params("/ctrl/planner");
    assert_eq!(out.get("a"), Some(&ParamValue::Int(2)), "{out:?}");
}

#[test]
fn file_then_inline_lets_the_inline_win() {
    // The mirror case — proves the fix is a reordering, not an inversion.
    let n = ordered(vec![
        ParamSource::File {
            content: A_IS_2.to_string(),
        },
        inline_src("a", ParamValue::Int(1)),
    ]);
    let out = n.resolved_params("/ctrl/planner");
    assert_eq!(out.get("a"), Some(&ParamValue::Int(1)), "{out:?}");
}

#[test]
fn ordered_list_shadows_the_legacy_split_views() {
    // When param_sources is present it is authoritative: the legacy fields must
    // not re-apply on top (that would restore "inline always wins").
    let mut n = ordered(vec![
        inline_src("a", ParamValue::Int(1)),
        ParamSource::File {
            content: A_IS_2.to_string(),
        },
    ]);
    n.params.insert("a".to_string(), ParamValue::Int(1));
    n.params_files.push(A_IS_2.to_string());
    let out = n.resolved_params("/ctrl/planner");
    assert_eq!(out.get("a"), Some(&ParamValue::Int(2)), "{out:?}");
}

#[test]
fn empty_ordered_list_falls_back_to_the_legacy_split() {
    // Records written before phase-54 carry only the split view.
    let n = node_with(vec![A_IS_2.to_string()], &[("a", ParamValue::Int(1))]);
    let out = n.resolved_params("/ctrl/planner");
    assert_eq!(out.get("a"), Some(&ParamValue::Int(1)), "{out:?}");
}

#[test]
fn file_sources_fold_left_to_right() {
    let later = r#"
/**:
  ros__parameters:
    a: 3
"#;
    let n = ordered(vec![
        ParamSource::File {
            content: A_IS_2.to_string(),
        },
        ParamSource::File {
            content: later.to_string(),
        },
    ]);
    let out = n.resolved_params("/ctrl/planner");
    assert_eq!(out.get("a"), Some(&ParamValue::Int(3)), "{out:?}");
}

// ---------------------------------------------------------------------------
// Within-file section precedence is by SPECIFICITY, not textual order.
// ---------------------------------------------------------------------------

#[test]
fn a_specific_section_beats_a_wildcard_written_after_it() {
    // rcl buckets a file's sections per node; a node-specific block overrides
    // `/**` however the two are ordered. nano-ros's (now retired) duplicate
    // matcher had this right and the model did not — regression guard.
    let yaml = r#"
/ctrl/planner:
  ros__parameters:
    rate: 25
/**:
  ros__parameters:
    rate: 10
    shared: 1
"#;
    let n = node_with(vec![yaml.to_string()], &[]);
    let out = n.resolved_params("/ctrl/planner");
    assert_eq!(out.get("rate"), Some(&ParamValue::Int(25)), "{out:?}");
    assert_eq!(out.get("shared"), Some(&ParamValue::Int(1)), "{out:?}");
}

#[test]
fn a_partial_wildcard_sits_between_the_global_one_and_a_literal() {
    let yaml = r#"
/ctrl/planner:
  ros__parameters:
    rank: 3
/**:
  ros__parameters:
    rank: 1
/ctrl/**:
  ros__parameters:
    rank: 2
"#;
    let n = node_with(vec![yaml.to_string()], &[]);
    let out = n.resolved_params("/ctrl/planner");
    assert_eq!(out.get("rank"), Some(&ParamValue::Int(3)), "{out:?}");

    // Drop the literal section: the partial wildcard must still beat `/**`.
    let no_literal = yaml.replace("/ctrl/planner:\n  ros__parameters:\n    rank: 3\n", "");
    let n = node_with(vec![no_literal], &[]);
    let out = n.resolved_params("/ctrl/planner");
    assert_eq!(out.get("rank"), Some(&ParamValue::Int(2)), "{out:?}");
}

#[test]
fn equally_specific_sections_keep_file_order() {
    // Two spellings of the same node — no specificity difference, so the later
    // one wins, as a plain last-writer merge would.
    let yaml = r#"
planner:
  ros__parameters:
    rate: 1
/ctrl/planner:
  ros__parameters:
    rate: 2
"#;
    let n = node_with(vec![yaml.to_string()], &[]);
    let out = n.resolved_params("/ctrl/planner");
    assert_eq!(out.get("rate"), Some(&ParamValue::Int(2)), "{out:?}");
}

#[test]
fn bake_strings_keep_a_float_a_float() {
    // `1.0f64.to_string()` is "1", which a runtime that re-types by inference
    // reads back as an INTEGER.
    assert_eq!(ParamValue::Float(1.0).to_bake_string(), "1.0");
    assert_eq!(ParamValue::Int(1).to_bake_string(), "1");
    assert_eq!(ParamValue::Bool(true).to_bake_string(), "true");
    assert_eq!(ParamValue::Str("a".into()).to_bake_string(), "a");
    assert_eq!(
        ParamValue::StrList(vec!["a".into(), "b".into()]).to_bake_string(),
        "a,b"
    );
}
