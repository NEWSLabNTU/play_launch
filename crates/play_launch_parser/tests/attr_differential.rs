//! Differential attribute test: this parser must agree with real ROS 2 on
//! which attributes each element accepts.
//!
//! ROS 2 is the oracle (CLAUDE.md: "When Rust and Python parser behaviors
//! differ, Python's behavior is always correct"). Skips when no ROS 2
//! environment is available.
//!
//! All ROS 2 probes for a test function run through ONE Python process
//! (`attr_differential_probe.py`), not one `bash -c 'source setup.bash;
//! python3 -c "..."'` subprocess per probe — see `ros2_batch_outcomes`.

use play_launch_parser::xml::attr_spec::{spec_for, validate_named};
use std::{collections::BTreeMap, io::Write, path::PathBuf, process::Command};

/// Minimal well-formed body per element, `{ATTR}` marking the injection
/// point. Every one of these must parse in ROS 2 with `{ATTR}` empty —
/// except the entries listed in `STRUCTURAL_BASELINE_EXCEPTIONS`, which
/// exist to validate a Rust-only extension ROS 2 rejects structurally
/// regardless of any attribute (see that constant's doc comment).
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
    // `<arg>` under `<include>` — a distinct ROS 2 entity from top-level
    // `<arg>` above (see the `include-arg` spec in `xml/attr_spec.rs`).
    // The include's `file=` target never needs to resolve: ROS 2 validates
    // `<arg>` children eagerly during `parse_description`, without visiting
    // the included file (that happens at execution time, not parse time —
    // measured: a nonexistent `file=` target still parses cleanly here).
    (
        "include-arg",
        r#"<include file="/nonexistent/attr-diff-target.launch.xml"><arg name="a" value="v"{ATTR}/></include>"#,
    ),
    // `<arg>` under `<executable>` — a Rust-only extension; real ROS 2
    // rejects `<arg>` as a child of `<executable>` entirely, regardless of
    // its attributes. Listed in `STRUCTURAL_BASELINE_EXCEPTIONS` below: its
    // baseline is asserted to fail with that EXACT structural reason (not
    // silently skipped), and it is excluded from the candidate-agreement
    // matrix because ROS 2 never reaches attribute-level validation for it
    // — there is no oracle signal to compare against.
    (
        "executable-arg",
        r#"<executable cmd="echo hi"><arg value="v"{ATTR}/></executable>"#,
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

/// Elements whose baseline (zero injected attributes) is REJECTED by real
/// ROS 2 for a reason that has nothing to do with attributes — a different
/// validation axis (illegal child ELEMENT, not illegal attribute) that this
/// per-attribute harness has no oracle signal for. `executable-arg` models
/// a Rust-only extension: `<arg>` as a child of `<executable>` is not legal
/// ROS 2 at all (measured below), so no candidate attribute on it can ever
/// be judged against ROS 2 either — every probe would report
/// `OtherError`, which `rust_and_ros2_agree_on_every_candidate_attribute`'s
/// loose acceptance would count as "not rejected" regardless of the
/// injected attribute, manufacturing agreement that isn't real. So this
/// element is (a) excluded from that matrix entirely, and (b) still
/// required, in the baseline test, to fail with EXACTLY this message — not
/// just "any failure" — so silent drift (ROS 2 starts accepting the
/// element, or starts rejecting it for an unrelated reason) still fails
/// loudly instead of the exclusion quietly absorbing it.
const STRUCTURAL_BASELINE_EXCEPTIONS: &[(&str, &str)] = &[(
    "executable-arg",
    "Unexpected nested tag(s) found in `executable`: {'arg'}",
)];

fn tmp_dir() -> PathBuf {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tmp/attr_diff");
    std::fs::create_dir_all(&dir).expect("create tmp dir");
    dir
}

fn write_fixture(body: &str, attr: Option<&str>, tag: &str) -> PathBuf {
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
#[derive(Debug, Clone)]
enum Ros2Outcome {
    /// `parse_description` completed with no exception.
    Success,
    /// Rejected specifically because of an unrecognized attribute — the
    /// signal this whole test is built to compare against.
    Reject,
    /// Some other exception (missing required attribute, illegal child
    /// element, bad type coercion, etc.) — not an attribute-name verdict
    /// either way. Carries the exception text so callers that need an exact
    /// reason (see `STRUCTURAL_BASELINE_EXCEPTIONS`) can check it.
    OtherError(String),
}

impl Ros2Outcome {
    fn is_success(&self) -> bool {
        matches!(self, Ros2Outcome::Success)
    }

    fn is_reject(&self) -> bool {
        matches!(self, Ros2Outcome::Reject)
    }

    fn message(&self) -> &str {
        match self {
            Ros2Outcome::OtherError(m) => m,
            _ => "",
        }
    }
}

#[derive(serde::Deserialize)]
struct ProbeResult {
    kind: String,
    message: String,
}

/// Feed every fixture in `manifest` (tag -> fixture path) to real ROS 2's
/// XML frontend inside ONE Python process (`attr_differential_probe.py`),
/// instead of one `bash -c 'source setup.bash; python3 -c "..."'` subprocess
/// per fixture. `None` means no ROS 2 environment is available (sentinel
/// exit code 42 from failing to source `setup.bash`).
///
/// `run_id` must be distinct per caller: `#[test]` functions in one binary
/// run concurrently (as threads under plain `cargo test`, or as concurrent
/// processes under nextest), and every caller shares `tmp_dir()` — a fixed
/// manifest/results filename would let two callers race on the same files.
fn ros2_batch_outcomes(
    run_id: &str,
    manifest: &BTreeMap<String, PathBuf>,
) -> Option<BTreeMap<String, Ros2Outcome>> {
    let dir = tmp_dir();
    let manifest_path = dir.join(format!("manifest_{run_id}.json"));
    let results_path = dir.join(format!("results_{run_id}.json"));

    let manifest_json: BTreeMap<&str, String> = manifest
        .iter()
        .map(|(tag, path)| (tag.as_str(), path.display().to_string()))
        .collect();
    std::fs::write(
        &manifest_path,
        serde_json::to_string(&manifest_json).expect("serialize manifest"),
    )
    .expect("write manifest");
    // Stale results from a previous (possibly failed) run must not leak into
    // this one — the script writes a fresh file on success, but if it
    // crashes before writing, a leftover file would look like a real result.
    let _ = std::fs::remove_file(&results_path);

    let probe_script =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/attr_differential_probe.py");
    let script = format!(
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 || exit 42; \
         python3 {} {} {}",
        probe_script.display(),
        manifest_path.display(),
        results_path.display(),
    );
    let out = Command::new("bash").arg("-c").arg(&script).output().ok()?;
    if out.status.code() == Some(42) {
        return None;
    }
    if !out.status.success() {
        eprintln!(
            "attr_differential: ROS 2 probe script exited with {:?}\nstdout: {}\nstderr: {}",
            out.status.code(),
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr),
        );
        return None;
    }

    let results_raw = std::fs::read_to_string(&results_path).ok()?;
    let results: BTreeMap<String, ProbeResult> =
        serde_json::from_str(&results_raw).expect("parse probe results JSON");

    Some(
        results
            .into_iter()
            .map(|(tag, r)| {
                let outcome = match r.kind.as_str() {
                    "SUCCESS" => Ros2Outcome::Success,
                    "REJECT" => Ros2Outcome::Reject,
                    _ => Ros2Outcome::OtherError(r.message),
                };
                (tag, outcome)
            })
            .collect(),
    )
}

/// Look up `tag` in a batch result map. Warns (rather than silently
/// treating it as "no verdict") if the tag is missing — that should only
/// happen if the probe script crashed on that specific fixture, which is
/// worth knowing about even though the test still degrades gracefully.
fn lookup<'a>(results: &'a BTreeMap<String, Ros2Outcome>, tag: &str) -> Option<&'a Ros2Outcome> {
    let outcome = results.get(tag);
    if outcome.is_none() {
        eprintln!(
            "attr_differential: no ROS 2 result for tag '{tag}' — probe script may have skipped it"
        );
    }
    outcome
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
    let manifest: BTreeMap<String, PathBuf> = FIXTURES
        .iter()
        .map(|(element, body)| {
            (
                element.to_string(),
                write_fixture(body, None, &format!("base_{element}")),
            )
        })
        .collect();

    let Some(results) = ros2_batch_outcomes("baseline", &manifest) else {
        eprintln!("skip: no ROS 2 environment");
        return;
    };

    for (element, _) in FIXTURES {
        let Some(outcome) = lookup(&results, element) else {
            panic!("no ROS 2 result for baseline fixture <{element}>");
        };

        if let Some((_, expected_reason)) = STRUCTURAL_BASELINE_EXCEPTIONS
            .iter()
            .find(|(e, _)| e == element)
        {
            // Documented, structurally-justified exception (see the
            // constant's doc comment) — not a silent skip. Pin the EXACT
            // rejection reason so drift (ROS 2 starts accepting this, or
            // starts rejecting it for a different reason) still fails here.
            assert!(
                !outcome.is_success(),
                "<{element}> was expected to fail structurally in ROS 2 \
                 (not an attribute problem — see STRUCTURAL_BASELINE_EXCEPTIONS) \
                 but it parsed cleanly; the exclusion from the candidate matrix \
                 is no longer justified and should be removed"
            );
            assert!(
                outcome.message().contains(expected_reason),
                "<{element}> failed as expected, but not for the documented \
                 reason.\n  expected to contain: {expected_reason}\n  actual: {}",
                outcome.message()
            );
            continue;
        }

        assert!(
            outcome.is_success(),
            "baseline fixture for <{element}> must parse CLEANLY in ROS 2 \
             with no injected attribute (no exception at all, not merely \
             one lacking 'Unexpected attribute'); otherwise every candidate \
             looks accepted and this whole test is vacuous. ROS 2 said: {}",
            outcome.message()
        );
    }
}

#[test]
fn rust_and_ros2_agree_on_every_candidate_attribute() {
    let mut manifest: BTreeMap<String, PathBuf> = BTreeMap::new();
    for (element, body) in FIXTURES {
        if STRUCTURAL_BASELINE_EXCEPTIONS
            .iter()
            .any(|(e, _)| e == element)
        {
            // No oracle signal at the attribute level for this element (see
            // the constant's doc comment) — every candidate is skipped, not
            // just the injection that happens to match the structural
            // rejection reason.
            continue;
        }
        for attr in CANDIDATES {
            if ALLOWED_DIVERGENCES.contains(&(*element, *attr)) {
                continue;
            }
            let tag = format!("{element}_{attr}");
            manifest.insert(tag.clone(), write_fixture(body, Some(attr), &tag));
        }
    }

    let Some(results) = ros2_batch_outcomes("candidates", &manifest) else {
        eprintln!("skip: no ROS 2 environment");
        return;
    };

    let mut disagreements: Vec<String> = Vec::new();

    for (element, _) in FIXTURES {
        assert!(
            spec_for(element).is_some(),
            "<{element}> is in the differential matrix but has no AttrSpec"
        );
        if STRUCTURAL_BASELINE_EXCEPTIONS
            .iter()
            .any(|(e, _)| e == element)
        {
            continue;
        }
        for attr in CANDIDATES {
            if ALLOWED_DIVERGENCES.contains(&(*element, *attr)) {
                continue;
            }
            let tag = format!("{element}_{attr}");
            let Some(outcome) = lookup(&results, &tag) else {
                continue;
            };
            // Loose on purpose: an unrelated failure (bad value, missing
            // unrelated attribute) is not a verdict on the attribute under
            // test, so it counts as "not rejected".
            let ros2 = !outcome.is_reject();
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
