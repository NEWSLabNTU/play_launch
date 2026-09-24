//! Issue #0045 — `--enforce-rules` on `run`.
//!
//! The flag lives on `CommonOptions`, so every verb PARSES it; `run` acted
//! on it nowhere. `run --enforce-rules strict` therefore started its node
//! and exited 0 having built no rule engine and observed no event, which is
//! exactly the shape phase 79 spent three waves removing from `launch`/`up`
//! — a gate that is green because nothing was measured.
//!
//! The ruling is that `run` cannot enforce: a contract is LOCATED by launch
//! file (`<pkg>/launch/<stem>.contract.yaml`, overlay or provider sidecar)
//! and `run` names a package and an executable, so no channel has a key to
//! look one up with. `run --check` already says this about its own contract
//! step. So an explicit non-`off` mode is refused BEFORE the spawn, and
//! these tests pin each half of that: the refusal fires and starts
//! nothing, the default and `off` still run, and the refusal is about the
//! flag rather than about the word `strict` appearing in argv.
//!
//! Every test gets its own `TempDir` cwd, for the reason `run_check.rs`
//! documents: `play_log/<timestamp>` is second-granular with no uniquifier,
//! and nextest runs these in parallel.

use play_launch_tests::{fixtures, process::ManagedProcess};
use std::process::{Output, Stdio};
use std::time::Duration;
use tempfile::TempDir;

/// `play_launch run <args...> demo_nodes_cpp talker` in a private cwd.
///
/// Flags go before the positionals, the way the issue's own reproduction
/// wrote them. (`RunArgs::args` is `trailing_var_arg`, but that only takes
/// effect after a `--` or a first node argument — see the last test.)
fn run_talker(flags: &[&str]) -> (Output, TempDir) {
    let env = fixtures::install_env();
    let work = TempDir::new().expect("failed to create tempdir");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work.path());
    cmd.arg("run");
    cmd.args(flags);
    // --check stops before spawning: these tests are about the gate in
    // front of the spawn, and the ones that must NOT refuse would otherwise
    // run a talker forever.
    cmd.args(["--disable-all", "--check", "demo_nodes_cpp", "talker"]);
    let out = cmd.output().expect("failed to run play_launch");
    (out, work)
}

fn text(out: &Output) -> String {
    format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    )
}

/// The gate: every explicit mode that promises enforcement is refused, and
/// the refusal happens before anything is created.
#[test]
fn explicit_enforcement_modes_are_refused_before_the_spawn() {
    for mode in ["strict", "warn", "record-only"] {
        let (out, work) = run_talker(&["--enforce-rules", mode]);
        let all = text(&out);

        assert!(
            !out.status.success(),
            "`run --enforce-rules {mode}` exited {:?} — a mode that enforces \
             nothing must not report a pass.\n{all}",
            out.status
        );
        assert!(
            all.contains("not available on `run`"),
            "the refusal must say the flag does not apply here.\n{all}"
        );
        assert!(
            all.contains("launch file"),
            "the refusal must give the reason (contracts are keyed by launch \
             file).\n{all}"
        );
        assert!(
            all.contains("--enforce-rules off"),
            "the refusal must name the way to run unenforced.\n{all}"
        );
        // Refused before `create_log_dir`, so no half-written bundle and no
        // moved `latest` (issue #0023).
        assert!(
            !work.path().join("play_log").exists(),
            "a refusal left a run bundle behind: {}",
            work.path().join("play_log").display()
        );
    }
}

/// `--enforce-rules strict` is refused by the equals spelling too — the
/// check asks clap, not the raw argv, so both forms must land the same.
#[test]
fn the_equals_spelling_is_refused_too() {
    let (out, _work) = run_talker(&["--enforce-rules=strict"]);
    let all = text(&out);
    assert!(
        !out.status.success() && all.contains("not available on `run`"),
        "`--enforce-rules=strict` must be refused like the spaced form.\n{all}"
    );
}

/// The default mode is `warn` and nobody typed it, so a plain `run` must
/// keep working. This is the assertion that makes the refusal usable at all:
/// refusing the VALUE rather than the flag would reject every invocation.
#[test]
fn a_plain_run_is_unaffected() {
    let (out, _work) = run_talker(&[]);
    let all = text(&out);
    assert!(
        out.status.success(),
        "a plain `run` must not be refused for carrying the default \
         --enforce-rules warn.\n{all}"
    );
    assert!(
        !all.contains("not available on `run`"),
        "nothing was asked for, so nothing should be refused.\n{all}"
    );
}

/// And `off` is the documented way to say "unenforced", so typing it is not
/// an error.
#[test]
fn explicit_off_is_accepted() {
    let (out, _work) = run_talker(&["--enforce-rules", "off"]);
    let all = text(&out);
    assert!(
        out.status.success(),
        "`--enforce-rules off` is the way out the refusal names; it must \
         work.\n{all}"
    );
}

/// `run`'s node arguments are `trailing_var_arg`, so after a `--` the tokens
/// `--enforce-rules strict` are the NODE's and play_launch was never
/// addressed. A textual scan of argv would refuse this run; asking clap for
/// the value's SOURCE does not.
///
/// (Without the `--` clap takes the flag for itself even when it follows the
/// executable, which this file's other tests cover — measured while writing
/// them, and the reason the check is not a string search.)
#[test]
fn the_flag_passed_through_to_the_node_is_not_a_refusal() {
    let env = fixtures::install_env();
    let work = TempDir::new().expect("failed to create tempdir");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work.path());
    cmd.args([
        "run",
        "--disable-all",
        "--check",
        "demo_nodes_cpp",
        "talker",
        "--",
        "--enforce-rules",
        "strict",
    ]);
    let out = cmd.output().expect("failed to run play_launch");
    let all = text(&out);
    assert!(
        out.status.success() && !all.contains("not available on `run`"),
        "those tokens are the node's arguments, not play_launch's.\n{all}"
    );
}

/// The issue's own reproduction, without `--check`: a REAL strict run, which
/// used to start the talker and exit 0. It must now end non-zero having
/// spawned nothing — and it must end at all, which is why this one is under
/// a timeout: a regression here is a talker running forever, not an
/// assertion failure.
#[test]
fn the_reported_invocation_starts_nothing() {
    let env = fixtures::install_env();
    let work = TempDir::new().expect("failed to create tempdir");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work.path());
    cmd.args([
        "run",
        "--enforce-rules",
        "strict",
        "--interception",
        "off",
        "demo_nodes_cpp",
        "talker",
    ]);
    cmd.stdout(Stdio::from(
        std::fs::File::create(work.path().join("stdout.log")).unwrap(),
    ));
    cmd.stderr(Stdio::from(
        std::fs::File::create(work.path().join("stderr.log")).unwrap(),
    ));
    let mut proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let status = proc.wait_with_timeout(Duration::from_secs(30));

    let all = format!(
        "{}{}",
        std::fs::read_to_string(work.path().join("stdout.log")).unwrap_or_default(),
        std::fs::read_to_string(work.path().join("stderr.log")).unwrap_or_default()
    );
    assert!(
        !status.success(),
        "`run --enforce-rules strict --interception off` exited {status:?}; \
         a strict run that measures nothing must not pass.\n{all}"
    );
    assert!(
        all.contains("not available on `run`"),
        "the refusal must name the flag and the verb.\n{all}"
    );
    assert!(
        !work.path().join("play_log").exists(),
        "the run was refused but still created a bundle, so something ran \
         before the gate"
    );
}
