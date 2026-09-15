//! `<timer period="N">` — ROS 2's `TimerAction` on the XML and YAML
//! frontends.
//!
//! The regression these guard is not "the delay was wrong". It is that
//! `timer` fell through to the parser's unsupported-action arm, so the timer
//! AND EVERY NODE UNDER IT was dropped: a launch file with 17 nodes resolved
//! to 7, and `check` exited 0 on the result. So the first assertion in almost
//! every test here is simply that the delayed node EXISTS.

use play_launch_parser::parse_launch_file;
use std::{collections::HashMap, io::Write};
use tempfile::NamedTempFile;

/// Parse a launch file given as a string, with the extension the frontend
/// dispatches on.
fn parse(suffix: &str, body: &str) -> play_launch_parser::record::RecordJson {
    let mut f = NamedTempFile::with_suffix(suffix).expect("temp file");
    f.write_all(body.as_bytes()).expect("write");
    f.flush().expect("flush");
    parse_launch_file(f.path(), HashMap::new()).expect("parse should succeed")
}

fn parse_with_args(
    suffix: &str,
    body: &str,
    args: HashMap<String, String>,
) -> play_launch_parser::record::RecordJson {
    let mut f = NamedTempFile::with_suffix(suffix).expect("temp file");
    f.write_all(body.as_bytes()).expect("write");
    f.flush().expect("flush");
    parse_launch_file(f.path(), args).expect("parse should succeed")
}

fn node<'a>(
    record: &'a play_launch_parser::record::RecordJson,
    name: &str,
) -> &'a play_launch_parser::record::NodeRecord {
    record
        .node
        .iter()
        .find(|n| n.name.as_deref() == Some(name))
        .unwrap_or_else(|| {
            panic!(
                "node `{name}` missing from the record; got {:?}",
                record
                    .node
                    .iter()
                    .map(|n| n.name.clone())
                    .collect::<Vec<_>>()
            )
        })
}

/// The reported bug, minimised: the timed node vanished entirely and only
/// `/plain_node` survived.
#[test]
fn a_timers_children_are_parsed_and_carry_its_delay() {
    let record = parse(
        ".launch.xml",
        r#"<launch>
             <node pkg="demo_nodes_cpp" exec="talker" name="plain_node"/>
             <timer period="3.0">
               <node pkg="demo_nodes_cpp" exec="listener" name="timed_node"/>
             </timer>
           </launch>"#,
    );

    assert_eq!(record.node.len(), 2, "both nodes must be modelled");
    assert_eq!(node(&record, "plain_node").start_delay_secs, None);
    assert_eq!(node(&record, "timed_node").start_delay_secs, Some(3.0));
}

/// A timer is not an action the parser drops any more, so `check` has
/// nothing to refuse on.
#[test]
fn a_timer_is_no_longer_a_dropped_action() {
    let record = parse(
        ".launch.xml",
        r#"<launch>
             <timer period="1.0">
               <node pkg="demo_nodes_cpp" exec="listener" name="timed_node"/>
             </timer>
           </launch>"#,
    );
    assert!(
        record.dropped_actions.is_empty(),
        "got {:?}",
        record.dropped_actions
    );
}

/// ROS 2 starts the inner timer when the outer one fires, so the delays ADD.
#[test]
fn nested_timers_add_their_periods() {
    let record = parse(
        ".launch.xml",
        r#"<launch>
             <timer period="3.0">
               <node pkg="demo_nodes_cpp" exec="listener" name="outer"/>
               <timer period="2.0">
                 <node pkg="demo_nodes_cpp" exec="listener" name="inner"/>
               </timer>
             </timer>
           </launch>"#,
    );

    assert_eq!(node(&record, "outer").start_delay_secs, Some(3.0));
    assert_eq!(node(&record, "inner").start_delay_secs, Some(5.0));
}

/// `period` is a substitution-bearing attribute in ROS 2.
#[test]
fn a_period_may_be_a_substitution() {
    let mut args = HashMap::new();
    args.insert("startup_delay".to_string(), "7.5".to_string());
    let record = parse_with_args(
        ".launch.xml",
        r#"<launch>
             <arg name="startup_delay" default="1.0"/>
             <timer period="$(var startup_delay)">
               <node pkg="demo_nodes_cpp" exec="listener" name="timed_node"/>
             </timer>
           </launch>"#,
        args,
    );
    assert_eq!(node(&record, "timed_node").start_delay_secs, Some(7.5));
}

/// A timer delays whatever it contains, not only plain `<node>`s: a
/// container and its composables ride along.
#[test]
fn a_container_under_a_timer_is_delayed_with_its_composables() {
    let record = parse(
        ".launch.xml",
        r#"<launch>
             <timer period="4.0">
               <node_container pkg="rclcpp_components" exec="component_container"
                               name="my_container" namespace="">
                 <composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker"
                                  name="talker"/>
               </node_container>
             </timer>
           </launch>"#,
    );

    assert_eq!(record.container.len(), 1);
    assert_eq!(record.container[0].start_delay_secs, Some(4.0));
    assert_eq!(record.load_node.len(), 1);
    assert_eq!(record.load_node[0].start_delay_secs, Some(4.0));
}

/// A timer's `if`/`unless` is evaluated like any other action's: a timer
/// that is skipped must not contribute its children.
#[test]
fn a_conditional_timer_is_skipped_whole() {
    let record = parse(
        ".launch.xml",
        r#"<launch>
             <timer period="3.0" if="false">
               <node pkg="demo_nodes_cpp" exec="listener" name="never"/>
             </timer>
             <timer period="3.0" unless="false">
               <node pkg="demo_nodes_cpp" exec="listener" name="always"/>
             </timer>
           </launch>"#,
    );

    assert_eq!(record.node.len(), 1);
    assert_eq!(node(&record, "always").start_delay_secs, Some(3.0));
}

/// A timer does not scope: ROS 2's `TimerAction` is not a `GroupAction`, so
/// a namespace pushed inside one is still pushed afterwards.
#[test]
fn a_timer_does_not_scope_its_body() {
    let record = parse(
        ".launch.xml",
        r#"<launch>
             <timer period="1.0">
               <push-ros-namespace namespace="sensing"/>
               <node pkg="demo_nodes_cpp" exec="listener" name="inside"/>
             </timer>
             <node pkg="demo_nodes_cpp" exec="listener" name="after"/>
           </launch>"#,
    );

    assert_eq!(
        node(&record, "inside").namespace.as_deref(),
        Some("/sensing")
    );
    assert_eq!(
        node(&record, "after").namespace.as_deref(),
        Some("/sensing")
    );
    assert_eq!(node(&record, "after").start_delay_secs, None);
}

/// ROS 2 requires `period`; so do we, rather than silently defaulting to
/// zero (which would read as "not delayed" — the very failure mode this
/// whole change is about).
#[test]
fn a_timer_without_a_period_is_a_parse_error() {
    let mut f = NamedTempFile::with_suffix(".launch.xml").unwrap();
    f.write_all(
        br#"<launch>
              <timer>
                <node pkg="demo_nodes_cpp" exec="listener" name="x"/>
              </timer>
            </launch>"#,
    )
    .unwrap();
    f.flush().unwrap();
    assert!(parse_launch_file(f.path(), HashMap::new()).is_err());
}

/// An unknown attribute is rejected the way it is on every other element —
/// the timer spec is a real entry in the attribute table, not a hole in it.
#[test]
fn an_unknown_timer_attribute_is_rejected() {
    let mut f = NamedTempFile::with_suffix(".launch.xml").unwrap();
    f.write_all(br#"<launch><timer period="1.0" zzz="x"/></launch>"#)
        .unwrap();
    f.flush().unwrap();
    assert!(parse_launch_file(f.path(), HashMap::new()).is_err());
}

// ── YAML frontend ──────────────────────────────────────────────────────

#[test]
fn the_yaml_frontend_supports_timer_too() {
    let record = parse(
        ".launch.yaml",
        r#"
launch:
- node: {pkg: demo_nodes_cpp, exec: talker, name: plain_node}
- timer:
    period: "2.5"
    children:
    - node: {pkg: demo_nodes_cpp, exec: listener, name: timed_node}
"#,
    );

    assert_eq!(record.node.len(), 2);
    assert_eq!(node(&record, "plain_node").start_delay_secs, None);
    assert_eq!(node(&record, "timed_node").start_delay_secs, Some(2.5));
    assert!(record.dropped_actions.is_empty());
}

#[test]
fn nested_yaml_timers_add_their_periods() {
    let record = parse(
        ".launch.yaml",
        r#"
launch:
- timer:
    period: "3.0"
    children:
    - node: {pkg: demo_nodes_cpp, exec: listener, name: outer}
    - timer:
        period: "2.0"
        children:
        - node: {pkg: demo_nodes_cpp, exec: listener, name: inner}
"#,
    );

    assert_eq!(node(&record, "outer").start_delay_secs, Some(3.0));
    assert_eq!(node(&record, "inner").start_delay_secs, Some(5.0));
}

// ── dropped actions ────────────────────────────────────────────────────

/// The other half of the bug: an action the parser does not implement took
/// its subtree with it and left nothing a caller could act on. Now it is a
/// value on the record, which is what lets `check` exit non-zero.
#[test]
fn an_unsupported_action_is_recorded_not_just_warned_about() {
    let record = parse(
        ".launch.xml",
        r#"<launch>
             <node pkg="demo_nodes_cpp" exec="talker" name="plain_node"/>
             <log message="hello"/>
           </launch>"#,
    );

    assert_eq!(record.node.len(), 1);
    assert_eq!(record.dropped_actions.len(), 1);
    assert_eq!(record.dropped_actions[0].action, "log");
    assert!(record.dropped_actions[0].file.is_some());
}

/// One finding per (action, file) — a launch file with thirty `<log>` lines
/// is one problem, not thirty.
#[test]
fn repeated_drops_of_the_same_action_are_reported_once() {
    let record = parse(
        ".launch.xml",
        r#"<launch>
             <log message="a"/>
             <log message="b"/>
             <log message="c"/>
           </launch>"#,
    );
    assert_eq!(record.dropped_actions.len(), 1);
}

#[test]
fn an_unsupported_yaml_action_is_recorded_too() {
    let record = parse(
        ".launch.yaml",
        r#"
launch:
- node: {pkg: demo_nodes_cpp, exec: talker, name: plain_node}
- log: {message: hello}
"#,
    );
    assert_eq!(record.dropped_actions.len(), 1);
    assert_eq!(record.dropped_actions[0].action, "log");
}
