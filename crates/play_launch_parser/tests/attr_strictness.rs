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
    for attr in [
        "exec_name",
        "ros_args",
        "launch-prefix",
        "cwd",
        "emulate_tty",
        "shell",
    ] {
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
