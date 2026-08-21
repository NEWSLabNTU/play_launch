//! Per-element attribute strictness — the Rust parser must accept exactly
//! what ROS 2's `launch_xml` accepts, warn on ROS 2 attributes it does not
//! implement, and reject everything else.

use play_launch_parser::xml::attr_spec::{spec_for, validate_named};

#[test]
fn node_spec_lists_the_ros2_supported_attributes() {
    let spec = spec_for("node").expect("node has a spec");
    for attr in [
        "pkg",
        "exec",
        "name",
        "namespace",
        "args",
        // Issue #9 — implemented, so it moved out of `known_unsupported`.
        // Its own `--ros-args` block; the container case is what Autoware
        // uses to silence an INFO-spammy API container.
        "ros_args",
        "output",
        "respawn",
        "respawn_delay",
        "if",
        "unless",
    ] {
        assert!(
            spec.supported.contains(&attr),
            "`{attr}` must be supported on <node>; supported = {:?}",
            spec.supported
        );
    }
}

#[test]
fn node_spec_marks_unimplemented_ros2_attributes_as_known() {
    let spec = spec_for("node").expect("node has a spec");
    for attr in ["exec_name", "launch-prefix", "cwd", "emulate_tty", "shell"] {
        assert!(
            spec.known_unsupported.contains(&attr),
            "`{attr}` is valid ROS 2 and must warn, not error; \
             known_unsupported = {:?}",
            spec.known_unsupported
        );
    }
}

#[test]
fn machine_is_rejected_on_node() {
    let err = validate_named("node", &["pkg", "exec", "machine"])
        .expect_err("machine= is ROS 1 syntax and must be rejected");
    let msg = err.to_string();
    assert!(
        msg.contains("Unexpected attribute(s) found in `node`"),
        "message must mirror launch_xml: {msg}"
    );
    assert!(
        msg.contains("'machine'"),
        "message must name the attribute: {msg}"
    );
}

#[test]
fn unknown_attributes_are_rejected_and_all_named_at_once() {
    let err = validate_named("node", &["pkg", "exec", "zzz", "aaa"])
        .expect_err("unknown attributes must be rejected");
    let msg = err.to_string();
    assert!(msg.contains("'aaa'"), "{msg}");
    assert!(msg.contains("'zzz'"), "{msg}");
    // Sorted, so the message is stable across attribute ordering.
    assert!(
        msg.find("'aaa'").unwrap() < msg.find("'zzz'").unwrap(),
        "attributes must be sorted: {msg}"
    );
}

#[test]
fn known_unsupported_attributes_are_accepted() {
    validate_named("node", &["pkg", "exec", "launch-prefix"])
        .expect("launch-prefix is valid ROS 2 — warn, do not error");
}

#[test]
fn group_namespace_is_known_unsupported_not_supported() {
    // ROS 2 rejects `<group namespace=>` outright (measured); this parser
    // reads it. Downgrading to a warning tells authors their launch file is
    // not portable without breaking them today.
    let spec = spec_for("group").expect("group has a spec");
    assert!(spec.known_unsupported.contains(&"namespace"), "{spec:?}");
    assert!(spec.known_unsupported.contains(&"ns"), "{spec:?}");
    assert!(!spec.supported.contains(&"namespace"), "{spec:?}");
}

#[test]
fn push_ros_namespace_ns_is_known_unsupported_not_supported() {
    // Same precedent as `<group namespace=/ns=>` above: ROS 2 rejects `ns`
    // on `<push-ros-namespace>` outright (measured, even alongside a valid
    // `namespace=`), but this parser reads it as a backwards-compat alias.
    // Warn, don't break launch files that already depend on the alias.
    let spec = spec_for("push-ros-namespace").expect("push-ros-namespace has a spec");
    assert!(spec.known_unsupported.contains(&"ns"), "{spec:?}");
    assert!(!spec.supported.contains(&"ns"), "{spec:?}");
}

#[test]
fn child_elements_do_not_accept_conditions() {
    // `<param>`, `<remap>`, `<env>` are not actions — ROS 2 rejects if/unless
    // on them (measured).
    for element in ["param", "remap", "env"] {
        let spec = spec_for(element).unwrap_or_else(|| panic!("{element} has a spec"));
        assert!(
            !spec.supported.contains(&"if"),
            "<{element}> must not accept if=: {spec:?}"
        );
    }
}

#[test]
fn hyphenated_element_aliases_resolve_to_the_same_spec() {
    // The traverser dispatches `set-env` and `set_env` to the same action.
    assert_eq!(
        spec_for("set-env").map(|s| s.supported),
        spec_for("set_env").map(|s| s.supported),
    );
    assert_eq!(
        spec_for("node-container").map(|s| s.supported),
        spec_for("node_container").map(|s| s.supported),
    );
}

#[test]
fn the_launch_root_is_not_strict() {
    // Measured: `<launch zzz="1">` parses fine in ROS 2.
    assert!(
        spec_for("launch").is_none(),
        "<launch> must have no spec so validation skips it"
    );
}

use std::io::Write;

/// Parse `source` from a scratch file carrying `suffix`.
///
/// The suffix is load-bearing: `parse_launch_file` dispatches XML vs YAML on
/// the file extension.
///
/// The name must be unique per call. `cargo test` runs every test in a binary
/// in ONE process across several threads, so a `std::process::id()`-derived
/// name is identical for all of them and tests clobber each other's fixture
/// between write and parse — intermittent failures that look like flakiness
/// (issue 0009). `NamedTempFile` allocates a unique path and removes it on
/// drop; `tempfile_in` keeps it under the repo's own `tmp/` rather than
/// `/tmp`, per the project convention.
fn parse_scratch(source: &str, suffix: &str) -> play_launch_parser::error::Result<()> {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tmp");
    std::fs::create_dir_all(&dir).expect("create tmp dir");
    let mut fh = tempfile::Builder::new()
        .prefix("attr_strict_")
        .suffix(suffix)
        .tempfile_in(&dir)
        .expect("create fixture");
    fh.write_all(source.as_bytes()).expect("write fixture");
    fh.flush().expect("flush fixture");
    play_launch_parser::parse_launch_file(fh.path(), Default::default()).map(|_| ())
}

fn parse_source(xml: &str) -> play_launch_parser::error::Result<()> {
    parse_scratch(xml, ".launch.xml")
}

#[test]
fn parsing_rejects_machine_on_node() {
    let err = parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" machine="robot1"/>
</launch>
"#,
    )
    .expect_err("machine= must fail the parse");
    let msg = err.to_string();
    assert!(
        msg.contains("Unexpected attribute(s) found in `node`") && msg.contains("'machine'"),
        "{msg}"
    );
}

#[test]
fn parsing_rejects_an_unknown_attribute() {
    let err = parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" zzz_bogus="x"/>
</launch>
"#,
    )
    .expect_err("unknown attributes must fail the parse");
    assert!(err.to_string().contains("'zzz_bogus'"), "{err}");
}

#[test]
fn parsing_accepts_a_known_unsupported_attribute() {
    parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" launch-prefix="nice -n 5"/>
</launch>
"#,
    )
    .expect("launch-prefix is valid ROS 2 — must parse with a warning");
}

#[test]
fn parsing_accepts_push_ros_namespace_ns_as_a_known_unsupported_alias() {
    // `ns=` on `<push-ros-namespace>` is not real ROS 2 (measured: ROS 2
    // rejects it even alongside a valid `namespace=`), but this parser reads
    // it as a backwards-compat alias for `namespace=` (see the
    // `push-ros-namespace` arm in traverser/entity.rs). Same precedent as
    // `<group namespace=/ns=>`: warn, don't break launch files that already
    // depend on the alias. Pins the precedent so a future change can't
    // silently flip this back to a hard error.
    parse_source(
        r#"<launch>
  <push-ros-namespace ns="/n"/>
  <node pkg="demo_nodes_cpp" exec="talker" name="t"/>
</launch>
"#,
    )
    .expect("push-ros-namespace ns= is a known-unsupported alias — must parse with a warning");
}

#[test]
fn parsing_accepts_an_unknown_attribute_on_the_launch_root() {
    parse_source(
        r#"<launch zzz="1">
  <node pkg="demo_nodes_cpp" exec="talker" name="t"/>
</launch>
"#,
    )
    .expect("<launch> is not strict in ROS 2");
}

#[test]
fn parsing_rejects_an_unknown_attribute_on_a_child_element() {
    // `<param>`, `<remap>`, `<env>` never reach the traverser dispatch —
    // the action's own children loop consumes them, so they need their own
    // validation hook.
    let err = parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t">
    <remap from="a" to="b" zzz_bogus="x"/>
  </node>
</launch>
"#,
    )
    .expect_err("unknown attributes on <remap> must fail the parse");
    let msg = err.to_string();
    assert!(
        msg.contains("Unexpected attribute(s) found in `remap`") && msg.contains("'zzz_bogus'"),
        "{msg}"
    );
}

#[test]
fn parsing_validates_even_when_a_condition_excludes_the_element() {
    // ROS 2 parses (and validates) regardless of if/unless — conditions are
    // evaluated at launch time. This parser evaluates them at parse time, so
    // validation must run before the condition check.
    let err = parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" machine="robot1" unless="true"/>
</launch>
"#,
    )
    .expect_err("a skipped element must still be validated");
    assert!(err.to_string().contains("'machine'"), "{err}");
}

fn parse_yaml_source(yaml: &str) -> play_launch_parser::error::Result<()> {
    parse_scratch(yaml, ".launch.yaml")
}

#[test]
fn yaml_parsing_rejects_an_unknown_key() {
    let err = parse_yaml_source(
        "launch:\n  - node:\n      pkg: demo_nodes_cpp\n      exec: talker\n      zzz_bogus: x\n",
    )
    .expect_err("unknown YAML node keys must fail the parse");
    let msg = err.to_string();
    assert!(
        msg.contains("Unexpected attribute(s) found in `node`") && msg.contains("'zzz_bogus'"),
        "{msg}"
    );
}

#[test]
fn yaml_parsing_rejects_machine() {
    let err = parse_yaml_source(
        "launch:\n  - node:\n      pkg: demo_nodes_cpp\n      exec: talker\n      machine: robot1\n",
    )
    .expect_err("machine= is ROS 1 syntax and must be rejected in YAML too");
    assert!(err.to_string().contains("'machine'"), "{err}");
}

#[test]
fn yaml_parsing_accepts_supported_keys() {
    parse_yaml_source(
        "launch:\n  - node:\n      pkg: demo_nodes_cpp\n      exec: talker\n      name: t\n      namespace: /n\n",
    )
    .expect("supported keys must parse");
}

// ── Issue 0011: elements the traverser dispatches that previously had no
// spec, and so accepted any attribute silently. ─────────────────────────────

#[test]
fn unset_env_rejects_an_unknown_attribute() {
    // Measured: ROS 2 accepts `name` + conditions here and rejects the rest
    // (`Unexpected attribute(s) found in `unset_env`: {'zzz_bogus'}`).
    let err = parse_source(r#"<launch><unset_env name="A" zzz_bogus="x"/></launch>"#)
        .expect_err("unknown attributes on <unset_env> must be rejected");
    assert!(err.to_string().contains("'zzz_bogus'"), "{err}");
}

#[test]
fn unset_env_accepts_its_supported_attributes() {
    parse_source(r#"<launch><unset_env name="A" if="true"/></launch>"#)
        .expect("name= and if= are valid on <unset_env>");
}

#[test]
fn pop_ros_namespace_rejects_an_unknown_attribute() {
    // A PARSER EXTENSION — ROS 2 rejects the element itself
    // (`Unrecognized entity of the type: pop-ros-namespace`), so the oracle
    // cannot probe its attributes and only this test covers the spec.
    let err = parse_source(
        r#"<launch><push-ros-namespace namespace="/n"/><pop-ros-namespace zzz_bogus="x"/></launch>"#,
    )
    .expect_err("unknown attributes on <pop-ros-namespace> must be rejected");
    assert!(err.to_string().contains("'zzz_bogus'"), "{err}");
}

#[test]
fn pop_ros_namespace_accepts_conditions() {
    parse_source(r#"<launch><push-ros-namespace namespace="/n"/><pop-ros-namespace/></launch>"#)
        .expect("<pop-ros-namespace> takes no attributes of its own");
}

#[test]
fn declare_argument_rejects_an_unknown_attribute() {
    // Also a parser extension (`Unrecognized entity of the type:
    // declare_argument` upstream); same reasoning as above.
    let err = parse_source(r#"<launch><declare_argument name="a" zzz_bogus="x"/></launch>"#)
        .expect_err("unknown attributes on <declare_argument> must be rejected");
    assert!(err.to_string().contains("'zzz_bogus'"), "{err}");
}

#[test]
fn declare_argument_accepts_its_supported_attributes() {
    parse_source(
        r#"<launch><declare_argument name="a" default="d" description="doc" choices="x,y"/></launch>"#,
    )
    .expect("name/default/description/choices are all read by DeclareArgumentAction");
}

#[test]
fn hyphenated_aliases_of_the_new_specs_resolve() {
    assert_eq!(
        spec_for("unset-env").map(|s| s.element),
        Some("unset_env"),
        "`unset-env` is dispatched alongside `unset_env` and must share its spec"
    );
    assert_eq!(
        spec_for("pop_ros_namespace").map(|s| s.element),
        Some("pop-ros-namespace"),
        "`pop_ros_namespace` is dispatched alongside `pop-ros-namespace`"
    );
}

// ── Issue 0010: YAML validated only the top-level action mapping, so nested
// child sequences were unchecked and a non-mapping action body was a silent
// no-op. ────────────────────────────────────────────────────────────────────

#[test]
fn yaml_rejects_an_unknown_key_on_a_nested_param() {
    let err = parse_yaml_source(
        "launch:\n  - node:\n      pkg: demo_nodes_cpp\n      exec: talker\n\
         \n      param:\n        - name: p\n          value: 1\n          zzz_bogus: x\n",
    )
    .expect_err("unknown keys on a nested param must be rejected");
    assert!(err.to_string().contains("'zzz_bogus'"), "{err}");
}

#[test]
fn yaml_rejects_an_unknown_key_on_a_nested_remap() {
    let err = parse_yaml_source(
        "launch:\n  - node:\n      pkg: demo_nodes_cpp\n      exec: talker\n\
         \n      remap:\n        - from: a\n          to: b\n          zzz_bogus: x\n",
    )
    .expect_err("unknown keys on a nested remap must be rejected");
    assert!(err.to_string().contains("'zzz_bogus'"), "{err}");
}

#[test]
fn yaml_rejects_an_unknown_key_on_a_nested_composable_node() {
    // `composable_node`'s spec had NO enforcement path at all on the YAML
    // frontend before this — it is reachable only as a nested sequence.
    let err = parse_yaml_source(
        "launch:\n  - load_composable_node:\n      target: /c\n\
         \n      composable_node:\n        - pkg: p\n          plugin: P\n\
         \n          name: n\n          zzz_bogus: x\n",
    )
    .expect_err("unknown keys on a nested composable_node must be rejected");
    assert!(err.to_string().contains("'zzz_bogus'"), "{err}");
}

#[test]
fn yaml_accepts_valid_nested_children() {
    parse_yaml_source(
        "launch:\n  - node:\n      pkg: demo_nodes_cpp\n      exec: talker\n\
         \n      param:\n        - name: p\n          value: 1\n\
         \n      remap:\n        - from: a\n          to: b\n",
    )
    .expect("valid nested param/remap keys must still parse");
}

#[test]
fn yaml_rejects_a_non_mapping_action_body() {
    // `- node:` with a null body previously reached no handler and was a
    // silent no-op: neither a validation error nor a parse error.
    let err = parse_yaml_source("launch:\n  - node:\n")
        .expect_err("a null action body must be refused, not silently ignored");
    let msg = err.to_string();
    assert!(msg.contains("node"), "{msg}");
    assert!(
        msg.contains("mapping"),
        "should say what was expected: {msg}"
    );
}

#[test]
fn yaml_rejects_a_scalar_action_body() {
    let err = parse_yaml_source("launch:\n  - node: \"bogus\"\n")
        .expect_err("a scalar action body must be refused");
    assert!(err.to_string().contains("a string"), "{err}");
}
