//! Differential attribute test: this parser must agree with real ROS 2 on
//! which attributes each element accepts.
//!
//! ROS 2 is the oracle (CLAUDE.md: "When Rust and Python parser behaviors
//! differ, Python's behavior is always correct"). Skips when no ROS 2
//! environment is available.

use play_launch_parser::xml::attr_spec::{spec_for, validate_named};
use std::{io::Write, process::Command};

/// Minimal well-formed body per element, `{ATTR}` marking the injection
/// point. Every one of these must parse in ROS 2 with `{ATTR}` empty.
const FIXTURES: &[(&str, &str)] = &[
    (
        "node",
        r#"<node pkg="demo_nodes_cpp" exec="talker" name="t"{ATTR}/>"#,
    ),
    ("executable", r#"<executable cmd="echo hi"{ATTR}/>"#),
    ("arg", r#"<arg name="a" default="d"{ATTR}/>"#),
    ("let", r#"<let name="a" value="v"{ATTR}/>"#),
    ("group", r#"<group{ATTR}><let name="x" value="1"/></group>"#),
    ("set_env", r#"<set_env name="A" value="1"{ATTR}/>"#),
    (
        "set_parameter",
        r#"<set_parameter name="p" value="1"{ATTR}/>"#,
    ),
    ("set_remap", r#"<set_remap from="a" to="b"{ATTR}/>"#),
    (
        "push-ros-namespace",
        r#"<push-ros-namespace namespace="/n"{ATTR}/>"#,
    ),
    (
        "node_container",
        r#"<node_container pkg="rclcpp_components" exec="component_container" name="c" namespace=""{ATTR}/>"#,
    ),
    (
        "load_composable_node",
        r#"<load_composable_node target="/c"{ATTR}><composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker" name="t"/></load_composable_node>"#,
    ),
    (
        "composable_node",
        r#"<load_composable_node target="/c"><composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker" name="t"{ATTR}/></load_composable_node>"#,
    ),
    (
        "param",
        r#"<node pkg="demo_nodes_cpp" exec="talker" name="t"><param name="p" value="1"{ATTR}/></node>"#,
    ),
    (
        "remap",
        r#"<node pkg="demo_nodes_cpp" exec="talker" name="t"><remap from="a" to="b"{ATTR}/></node>"#,
    ),
    (
        "env",
        r#"<node pkg="demo_nodes_cpp" exec="talker" name="t"><env name="A" value="1"{ATTR}/></node>"#,
    ),
];

/// Every attribute worth probing, across all elements.
const CANDIDATES: &[&str] = &[
    "if",
    "unless",
    "pkg",
    "exec",
    "plugin",
    "name",
    "namespace",
    "ns",
    "args",
    "ros_args",
    "exec_name",
    "output",
    "respawn",
    "respawn_delay",
    "launch-prefix",
    "cwd",
    "emulate_tty",
    "shell",
    "cmd",
    "scoped",
    "forwarding",
    "default",
    "description",
    "value",
    "file",
    "target",
    "from",
    "to",
    "type",
    "machine",
    "zzz_bogus",
];

/// Deliberate divergences: this parser warns where ROS 2 rejects, because
/// rejecting would break launch files that already rely on the behavior.
/// Ruled on in Task 2 (`<group namespace=/ns=>`) and extended during Task 2's
/// fix round (`<push-ros-namespace ns=>`, matching the `<group>` precedent).
const ALLOWED_DIVERGENCES: &[(&str, &str)] = &[
    ("group", "namespace"),
    ("group", "ns"),
    ("push-ros-namespace", "ns"),
];

fn tmp_dir() -> std::path::PathBuf {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tmp/attr_diff");
    std::fs::create_dir_all(&dir).expect("create tmp dir");
    dir
}

fn write_fixture(body: &str, attr: Option<&str>, tag: &str) -> std::path::PathBuf {
    let injected = match attr {
        Some(a) => format!(" {a}=\"x\""),
        None => String::new(),
    };
    let xml = format!(
        "<launch>\n  {}\n</launch>\n",
        body.replace("{ATTR}", &injected)
    );
    let path = tmp_dir().join(format!("{tag}.launch.xml"));
    let mut fh = std::fs::File::create(&path).expect("write fixture");
    fh.write_all(xml.as_bytes()).expect("write fixture");
    path
}

/// Raw outcome of feeding a fixture to real ROS 2's XML frontend.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum Ros2Outcome {
    /// `parse_description` completed with no exception.
    Success,
    /// Rejected specifically because of an unrecognized attribute — the
    /// signal this whole test is built to compare against.
    Reject,
    /// Some other exception (missing required attribute, bad type
    /// coercion, etc.) — not an attribute-name verdict either way.
    OtherError,
}

/// Run the fixture through real ROS 2's XML frontend. `None` means no ROS 2
/// environment is available (sentinel exit code 42).
fn ros2_outcome(path: &std::path::Path) -> Option<Ros2Outcome> {
    let script = format!(
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 || exit 42; \
         python3 -c \"
from launch.frontend import Parser
import sys
try:
    root, parser = Parser.load(sys.argv[1])
    parser.parse_description(root)
    print('SUCCESS')
except Exception as exc:
    print('REJECT' if 'Unexpected attribute' in str(exc) else 'OTHER_ERROR')
\" {}",
        path.display()
    );
    let out = Command::new("bash").arg("-c").arg(&script).output().ok()?;
    if out.status.code() == Some(42) {
        return None;
    }
    let stdout = String::from_utf8_lossy(&out.stdout);
    if stdout.contains("REJECT") {
        Some(Ros2Outcome::Reject)
    } else if stdout.contains("SUCCESS") {
        Some(Ros2Outcome::Success)
    } else if stdout.contains("OTHER_ERROR") {
        Some(Ros2Outcome::OtherError)
    } else {
        None
    }
}

/// Does real ROS 2 accept this attribute *name*? Used for the candidate
/// matrix, where the question is narrowly "does ROS 2 recognize this
/// attribute on this element" — an unrelated failure (bad value, missing
/// unrelated attribute) is not a verdict on the attribute under test, so it
/// counts as "not rejected". `None` means no ROS 2 available.
///
/// This is deliberately loose. `ros2_parses_cleanly` below is the strict
/// counterpart used for the no-injection baseline, where looseness would
/// let an unrelated failure masquerade as a pass (see
/// `every_fixture_has_a_passing_baseline`).
fn ros2_accepts(path: &std::path::Path) -> Option<bool> {
    match ros2_outcome(path)? {
        Ros2Outcome::Reject => Some(false),
        Ros2Outcome::Success | Ros2Outcome::OtherError => Some(true),
    }
}

/// Does this fixture parse *cleanly* in real ROS 2 — no exception at all?
/// Strict on purpose: the baseline fixtures must be unimpeachably valid
/// ROS 2 launch XML with zero injected attributes, or every candidate
/// judged against them is meaningless. `None` means no ROS 2 available.
fn ros2_parses_cleanly(path: &std::path::Path) -> Option<bool> {
    Some(ros2_outcome(path)? == Ros2Outcome::Success)
}

/// Does this parser accept it? `known_unsupported` counts as accepted —
/// it warns and continues, which is what ROS 2 does modulo behavior.
/// (`validate_named` already implements this: it logs a warning and returns
/// `Ok` for `known_unsupported` attributes, so no special-casing is needed
/// here beyond delegating to it.)
fn rust_accepts(element: &str, attr: Option<&str>) -> bool {
    let mut attrs: Vec<&str> = Vec::new();
    if let Some(a) = attr {
        attrs.push(a);
    }
    validate_named(element, &attrs).is_ok()
}

#[test]
fn every_fixture_has_a_passing_baseline() {
    let Some(_) = ros2_parses_cleanly(&write_fixture(FIXTURES[0].1, None, "baseline_probe")) else {
        eprintln!("skip: no ROS 2 environment");
        return;
    };
    for (element, body) in FIXTURES {
        let path = write_fixture(body, None, &format!("base_{element}"));
        assert_eq!(
            ros2_parses_cleanly(&path),
            Some(true),
            "baseline fixture for <{element}> must parse CLEANLY in ROS 2 \
             with no injected attribute (no exception at all, not merely \
             one lacking 'Unexpected attribute'); otherwise every candidate \
             looks accepted and this whole test is vacuous"
        );
    }
}

#[test]
fn rust_and_ros2_agree_on_every_candidate_attribute() {
    let Some(_) = ros2_accepts(&write_fixture(FIXTURES[0].1, None, "agree_probe")) else {
        eprintln!("skip: no ROS 2 environment");
        return;
    };

    let mut disagreements: Vec<String> = Vec::new();

    for (element, body) in FIXTURES {
        assert!(
            spec_for(element).is_some(),
            "<{element}> is in the differential matrix but has no AttrSpec"
        );
        for attr in CANDIDATES {
            if ALLOWED_DIVERGENCES.contains(&(*element, *attr)) {
                continue;
            }
            let path = write_fixture(body, Some(attr), &format!("{element}_{attr}"));
            let Some(ros2) = ros2_accepts(&path) else {
                continue;
            };
            let rust = rust_accepts(element, Some(attr));
            if ros2 != rust {
                disagreements.push(format!(
                    "<{element} {attr}=>: ROS 2 {}, Rust {}",
                    if ros2 { "accepts" } else { "rejects" },
                    if rust { "accepts" } else { "rejects" },
                ));
            }
        }
    }

    assert!(
        disagreements.is_empty(),
        "attribute allowlists have drifted from ROS 2:\n  {}",
        disagreements.join("\n  ")
    );
}
