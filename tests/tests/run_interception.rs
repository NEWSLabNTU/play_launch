//! Issue #0053 — `--interception on` is wired on the `run` verb.
//!
//! `run` read only the scheduling half of `load_runtime_config`, so the flag
//! was accepted and ignored: no ring, no `LD_PRELOAD`, no consumer task and no
//! `interception/` directory in the bundle. The sibling defect (#0045) was
//! REFUSED because a contract is keyed by launch file and `run` names a
//! package and an executable; interception has no such key, so this one is a
//! wiring job.
//!
//! The gate is not "the directory exists" — that proves the plumbing ran, not
//! that what it wrote is usable. It is that `play_launch measure` reads the
//! bundle and reports the events it found, and that
//! `scripts/capture_manifest.py` joins the recorded endpoints onto a model.

use play_launch_tests::{fixtures, process::ManagedProcess};
use std::{
    path::{Path, PathBuf},
    process::Stdio,
    time::Duration,
};

/// Path to the interception .so (built by `just build-interception`, which is
/// NOT part of the colcon workspace — so it can be absent from `install/`).
fn interception_so_path() -> PathBuf {
    for rel in [
        "src/play_launch_interception/target/release/libplay_launch_interception.so",
        "src/play_launch_interception/target/debug/libplay_launch_interception.so",
    ] {
        let p = fixtures::repo_root().join(rel);
        if p.is_file() {
            return p;
        }
    }
    panic!(
        "libplay_launch_interception.so not found under {}. Run `just build-interception` first.",
        fixtures::repo_root().display()
    );
}

/// A SystemModel naming the one node `run` spawned, with one declared path.
///
/// `run` has no launch file and therefore no model of its own; every consumer
/// of a bundle (`measure`, `capture_manifest.py`) needs one for the node KEYS.
/// That the keys line up at all is the point: the member name the interceptor
/// stamps on every record is the node's FQN, not the bare executable.
const MODEL: &str = r#"
meta:
  version: 1
structure:
  scopes:
    "0":
      path: /
  nodes:
    /talker:
      scope: "0"
      pkg: demo_nodes_cpp
      exec: talker
  topics:
    /chatter:
      type: std_msgs/msg/String
      pub: [/talker/chatter]
contracts:
  node_paths:
    /talker/publish:
      output: [/talker/chatter]
      max_latency_ms: 10.0
"#;

struct Bundle {
    _tmp: tempfile::TempDir,
    dir: PathBuf,
    model: PathBuf,
    stderr: String,
}

impl Bundle {
    fn interception(&self, name: &str) -> PathBuf {
        self.dir.join("interception").join(name)
    }
}

/// `play_launch run demo_nodes_cpp talker`, left publishing for a few seconds
/// and then stopped with SIGTERM so the summaries are written.
///
/// `--disable-all` covers monitoring, diagnostics and the web UI only — never
/// interception — so it keeps this test off port 8080 without touching what is
/// under test. `--interception` is passed explicitly, which is the whole
/// claim: nothing else on `run` turns interception on, because `run` builds no
/// rule engine to imply it.
fn run_talker(interception: Option<&str>, seconds: u64) -> Bundle {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir = tmp.path().to_path_buf();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.arg("run").arg("--disable-all");
    if let Some(switch) = interception {
        cmd.args(["--interception", switch]);
    }
    cmd.args(["demo_nodes_cpp", "talker"]);
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", interception_so_path());
    cmd.env("RUST_LOG", "play_launch=debug");
    cmd.stdout(Stdio::from(
        std::fs::File::create(work_dir.join("stdout.log")).unwrap(),
    ));
    cmd.stderr(Stdio::from(
        std::fs::File::create(work_dir.join("stderr.log")).unwrap(),
    ));

    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch run");

    // Let the talker publish. ManagedProcess::drop sends SIGTERM, waits 2s,
    // then SIGKILL — the SIGTERM is what makes `run` signal its own shutdown,
    // which is what flushes `events.jsonl` and writes the summaries.
    std::thread::sleep(Duration::from_secs(seconds));
    drop(proc);
    std::thread::sleep(Duration::from_millis(500));

    let model = work_dir.join("system_model.yaml");
    std::fs::write(&model, MODEL).unwrap();
    let stderr = std::fs::read_to_string(work_dir.join("stderr.log")).unwrap_or_default();

    Bundle {
        dir: work_dir.join("play_log/latest"),
        model,
        stderr,
        _tmp: tmp,
    }
}

/// THE GATE: a `run` bundle `play_launch measure` can actually read.
///
/// Asserted through the verb rather than by reading the file ourselves,
/// because "the artifact exists" and "the artifact is usable" are different
/// claims and only the second one is worth anything. `measure`'s header counts
/// the events it parsed, so a zero there fails the test as surely as a missing
/// file does.
#[test]
fn test_run_bundle_is_readable_by_measure() {
    let bundle = run_talker(Some("on"), 8);

    let events = bundle.interception("events.jsonl");
    assert!(
        events.is_file(),
        "no {} — `--interception on` recorded nothing on `run`.\nstderr:\n{}",
        events.display(),
        bundle.stderr
    );

    // The per-message record has to name a real topic, not just be non-empty:
    // the topic-name records are what let a reader resolve a hash at all.
    let body = std::fs::read_to_string(&events).unwrap();
    assert!(
        body.contains(r#""n":"/chatter""#),
        "events.jsonl carries no name record for /chatter:\n{body}"
    );
    assert!(
        body.contains(r#""n":"/talker","d":"pub""#),
        "events.jsonl carries no publish by /talker — the member name must be the node's \
         FQN, since that is the key every consumer joins on:\n{body}"
    );

    let env = fixtures::install_env();
    let out = fixtures::play_launch_cmd(&env)
        .args([
            "measure",
            bundle.dir.to_str().unwrap(),
            "--model",
            bundle.model.to_str().unwrap(),
        ])
        .output()
        .expect("run play_launch measure");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "measure failed on a run bundle: {stderr}"
    );
    assert!(
        stdout.contains("# play_launch measure"),
        "measure printed no report:\n{stdout}\n{stderr}"
    );
    // `N message events over X s`, from the file it just read. Zero here means
    // measure found the file and it said nothing.
    let events_line = stdout
        .lines()
        .find(|l| l.contains("message events over"))
        .unwrap_or_else(|| panic!("measure printed no event count:\n{stdout}"));
    let count: u64 = events_line
        .split_whitespace()
        .nth(1)
        .and_then(|n| n.parse().ok())
        .unwrap_or_else(|| panic!("cannot read the event count from {events_line:?}"));
    assert!(
        count > 0,
        "measure read the bundle and found no events: {events_line:?}"
    );
}

/// The cheaper second reader: `scripts/capture_manifest.py` walks
/// `endpoints.tsv` + `node_identity.tsv` + `stats_summary.json` and emits a
/// contract. It refuses outright when no observed endpoint joins to a model
/// node, so exit 0 here is a statement about all three files at once.
#[test]
fn test_run_bundle_is_readable_by_capture_manifest() {
    let bundle = run_talker(Some("on"), 8);

    for name in ["endpoints.tsv", "node_identity.tsv", "stats_summary.json"] {
        assert!(
            bundle.interception(name).is_file(),
            "missing {} from the run bundle.\nstderr:\n{}",
            bundle.interception(name).display(),
            bundle.stderr
        );
    }

    let script = fixtures::repo_root().join("scripts/capture_manifest.py");
    let out = std::process::Command::new("python3")
        .arg(&script)
        .arg(&bundle.dir)
        .args(["--model", bundle.model.to_str().unwrap()])
        .output()
        .expect("run capture_manifest.py");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "capture_manifest.py refused the run bundle:\n{stderr}\n{stdout}"
    );
    assert!(
        stdout.contains("/talker"),
        "the captured contract does not mention the node that ran:\n{stdout}"
    );
}

/// The negative control: nothing but the flag turns interception on.
///
/// Without it the bundle must look exactly as it did before #0053 was fixed —
/// which is also what makes the test above a statement about the flag rather
/// than about something else in the bundle.
#[test]
fn test_run_without_the_flag_records_nothing() {
    let bundle = run_talker(None, 3);
    assert!(
        bundle.dir.join("run_info.json").is_file(),
        "the run did not produce a bundle at all: {}",
        bundle.dir.display()
    );
    assert!(
        !bundle.dir.join("interception").exists(),
        "a plain `play_launch run` wrote {} — `run` enforces no contract, so nothing \
         implies interception there",
        bundle.dir.join("interception").display()
    );
}

/// Issue #0045 still holds, with interception wired.
///
/// Half of #0045's reasoning was that `run` observes nothing; this change
/// removes that half. The refusal stands on the other half — a contract is
/// located by launch file — so `--enforce-rules strict` must still be refused
/// before anything is created, even alongside `--interception on`.
#[test]
fn test_enforcement_is_still_refused_with_interception_on() {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().unwrap();
    let out = fixtures::play_launch_cmd(&env)
        .current_dir(tmp.path())
        .args([
            "run",
            "--disable-all",
            "--interception",
            "on",
            "--enforce-rules",
            "strict",
            "demo_nodes_cpp",
            "talker",
        ])
        .output()
        .expect("run play_launch run");
    assert!(
        !out.status.success(),
        "--enforce-rules strict was accepted on `run`"
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("--enforce-rules strict is not available on `run`"),
        "the refusal did not name itself:\n{stderr}"
    );
    // Refused before anything is created (issue #0023): no half-written
    // bundle, no moved `latest`.
    assert!(
        !Path::new(&tmp.path().join("play_log")).exists(),
        "a refused run left a bundle behind"
    );
}
