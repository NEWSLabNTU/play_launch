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

use play_launch_parser::xml::attr_spec::{all_specs, spec_for, validate_named};
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
    ("unset_env", r#"<unset_env name="A"{ATTR}/>"#),
    // Two PARSER EXTENSIONS ROS 2 does not know as elements at all
    // (`Unrecognized entity of the type: …`), so — like `executable-arg` —
    // the oracle never reaches attribute validation for them. Their
    // baselines are pinned to that exact structural message below rather
    // than skipped, and they sit out the candidate-agreement matrix.
    ("pop-ros-namespace", r#"<pop-ros-namespace{ATTR}/>"#),
    (
        "declare_argument",
        r#"<declare_argument name="a" default="d"{ATTR}/>"#,
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
    "choices",
    "value",
    "file",
    "target",
    "from",
    "to",
    "type",
    "machine",
    "zzz_bogus",
    "on_exit",
    "node-name",
    "allow_substs",
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
const STRUCTURAL_BASELINE_EXCEPTIONS: &[(&str, &str)] = &[
    (
        "executable-arg",
        "Unexpected nested tag(s) found in `executable`: {'arg'}",
    ),
    (
        "pop-ros-namespace",
        "Unrecognized entity of the type: pop-ros-namespace",
    ),
    (
        "declare_argument",
        "Unrecognized entity of the type: declare_argument",
    ),
];

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

/// Result of attempting to run the batched ROS 2 oracle. Deliberately NOT
/// `Option<...>`: collapsing "confirmed no ROS 2" and "the probe script
/// broke for some unrelated reason" into the same `None`/skip is exactly
/// the vacuous-test failure mode this whole file exists to prevent (see
/// `every_fixture_has_a_passing_baseline`'s doc comment) — it would just be
/// reintroduced one layer down, at the subprocess boundary, by batching.
/// `NoRos2` is the only variant a caller may treat as a skip; `Failed` must
/// panic loud.
enum Ros2Batch {
    /// Sourcing `setup.bash` failed (sentinel exit code 42) — confirmed, no
    /// ROS 2 environment here. The only variant that should skip.
    NoRos2,
    /// Something else went wrong: the subprocess failed to spawn, exited
    /// non-zero for a reason other than "no ROS 2", or produced a
    /// missing/unparsable results file. NOT "no ROS 2 available" — a caller
    /// that treats this as a skip would make every downstream assertion
    /// vacuously "pass" instead of failing loud on a broken harness.
    Failed(String),
    /// The batch ran to completion: per-tag verdicts.
    Outcomes(BTreeMap<String, Ros2Outcome>),
}

/// Feed every fixture in `manifest` (tag -> fixture path) to real ROS 2's
/// XML frontend inside ONE Python process (`attr_differential_probe.py`),
/// instead of one `bash -c 'source setup.bash; python3 -c "..."'` subprocess
/// per fixture.
///
/// `run_id` must be distinct per caller: `#[test]` functions in one binary
/// run concurrently (as threads under plain `cargo test`, or as concurrent
/// processes under nextest), and every caller shares `tmp_dir()` — a fixed
/// manifest/results filename would let two callers race on the same files.
fn ros2_batch_outcomes(run_id: &str, manifest: &BTreeMap<String, PathBuf>) -> Ros2Batch {
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
    let out = match Command::new("bash").arg("-c").arg(&script).output() {
        Ok(out) => out,
        Err(e) => return Ros2Batch::Failed(format!("failed to spawn probe subprocess: {e}")),
    };
    if out.status.code() == Some(42) {
        return Ros2Batch::NoRos2;
    }
    if !out.status.success() {
        return Ros2Batch::Failed(format!(
            "ROS 2 probe script exited with {:?} (this is NOT \"no ROS 2\" — that's sentinel \
             exit code 42; investigate rather than treating this as a skip)\nstdout: {}\nstderr: {}",
            out.status.code(),
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr),
        ));
    }

    let results_raw = match std::fs::read_to_string(&results_path) {
        Ok(raw) => raw,
        Err(e) => {
            return Ros2Batch::Failed(format!(
                "probe script exited 0 but results file {} is unreadable: {e}",
                results_path.display()
            ));
        }
    };
    let results: BTreeMap<String, ProbeResult> = match serde_json::from_str(&results_raw) {
        Ok(r) => r,
        Err(e) => {
            return Ros2Batch::Failed(format!(
                "probe script exited 0 but results file {} is not valid JSON: {e}\nraw: {results_raw}",
                results_path.display()
            ));
        }
    };

    Ros2Batch::Outcomes(
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

/// Look up `tag` in a batch result map. Panics rather than degrading —
/// once `ros2_batch_outcomes` has returned `Outcomes`, every tag in the
/// manifest that produced it should have a corresponding entry (the probe
/// script iterates the manifest exactly); a missing entry means the script
/// silently dropped a fixture, which is a bug worth failing loud on, not a
/// per-candidate skip that could mask a real disagreement.
fn lookup<'a>(results: &'a BTreeMap<String, Ros2Outcome>, tag: &str) -> &'a Ros2Outcome {
    results.get(tag).unwrap_or_else(|| {
        panic!(
            "no ROS 2 result for tag '{tag}' even though the probe batch reported success — \
             the probe script silently dropped a fixture it should have covered"
        )
    })
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

/// Hardening for the root cause behind the `on_exit`/`node-name` misses:
/// `CANDIDATES` is a closed, hand-written list, so an attribute ROS 2
/// accepts that nobody thought to name is never probed by
/// `rust_and_ros2_agree_on_every_candidate_attribute`. This assertion
/// closes one half of that gap — every attribute a spec already *claims*
/// (via `supported` or `known_unsupported`) must be in `CANDIDATES`, so a
/// table edit that isn't accompanied by a `CANDIDATES` entry fails a test
/// instead of silently going unprobed (exactly what happened to
/// `allow_substs` when it was added to `param`'s `known_unsupported` in
/// 687992e without a matching `CANDIDATES` entry).
///
/// What this does NOT buy: it cannot discover an attribute that is absent
/// from BOTH a spec and `CANDIDATES` — e.g. the original `on_exit`/
/// `node-name` misses, before anyone added them anywhere. That gap needs a
/// human (or a tool) reading ROS 2's `parse()` source, not this assertion.
/// This only guarantees internal consistency between the tables and the
/// probe list, not completeness against ROS 2 itself.
#[test]
fn every_specced_attribute_is_a_candidate() {
    let mut missing: Vec<String> = Vec::new();
    for spec in all_specs() {
        for attr in spec.supported.iter().chain(spec.known_unsupported.iter()) {
            if !CANDIDATES.contains(attr) {
                missing.push(format!("<{} {}=…>", spec.element, attr));
            }
        }
    }
    assert!(
        missing.is_empty(),
        "attribute(s) named in an AttrSpec but never probed by the \
         differential test (add to CANDIDATES):\n  {}",
        missing.join("\n  ")
    );
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

    let results = match ros2_batch_outcomes("baseline", &manifest) {
        Ros2Batch::NoRos2 => {
            eprintln!("skip: no ROS 2 environment");
            return;
        }
        Ros2Batch::Failed(detail) => panic!(
            "ROS 2 probe batch failed for a reason unrelated to \"no ROS 2\" — failing loud \
             instead of skipping, since a skip here would silently pass every baseline \
             assertion this test exists to make:\n{detail}"
        ),
        Ros2Batch::Outcomes(map) => map,
    };

    for (element, _) in FIXTURES {
        let outcome = lookup(&results, element);

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

    let results = match ros2_batch_outcomes("candidates", &manifest) {
        Ros2Batch::NoRos2 => {
            eprintln!("skip: no ROS 2 environment");
            return;
        }
        Ros2Batch::Failed(detail) => panic!(
            "ROS 2 probe batch failed for a reason unrelated to \"no ROS 2\" — failing loud \
             instead of skipping, since a skip here would silently pass every candidate \
             comparison this test exists to make:\n{detail}"
        ),
        Ros2Batch::Outcomes(map) => map,
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
            let outcome = lookup(&results, &tag);
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
