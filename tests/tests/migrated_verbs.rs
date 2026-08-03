//! The verb surface `pip install play_launch` gives a user.
//!
//! Two things are pinned here.
//!
//! 1. A RENAMED verb must name its replacement. Asserting only that the
//!    command "fails" is what nano-ros issue 0285 did: it failed, with
//!    `unrecognized subcommand 'resolve'`, from inside a cmake configure.
//!    These tests assert the error is USEFUL.
//!
//! 2. The five launch-tree verbs are back and actually WORK. `check` and
//!    `resolve` used to be tested here as removed verbs that printed
//!    migration advice; that advice pointed at `ros-launch-resolve`, a
//!    developer binary the wheel does not ship, and with `resolve`/`dump`
//!    both gone no play_launch verb could write the `system_model.yaml`
//!    that `up` requires. Those two tests are deleted, replaced by the
//!    round trip below.

use play_launch_tests::{fixtures, process::ManagedProcess};
use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

/// Repo-root-relative path, for fixtures and outputs.
fn repo(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("tests/ has a parent")
        .join(rel)
}

#[test]
fn replay_names_up_as_its_replacement() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["replay", "system_model.yaml"]);
    let out = cmd.output().expect("failed to run play_launch");

    assert!(!out.status.success(), "`replay` must exit non-zero");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        err.contains("renamed to `up`"),
        "must name the rename: {err}"
    );
    assert!(
        err.contains("play_launch up system_model.yaml"),
        "must echo the user's own argument in the new form: {err}"
    );
    // An eyre `Location:` footer reads as an internal crash in the exact
    // scenario this handler exists for (surfacing from a cmake configure).
    assert!(
        !err.contains("Location:"),
        "migration guidance must not carry a source-location footer: {err}"
    );
}

/// `resolve` must WRITE a model. This is the verb whose removal left `up`
/// with nothing to spawn from — a pip-only user had the consumer and no
/// producer.
#[test]
fn resolve_writes_a_system_model() {
    let env = fixtures::install_env();
    let out_path = repo("tmp/migrated_verbs_resolve.yaml");
    let _ = std::fs::remove_file(&out_path);
    std::fs::create_dir_all(out_path.parent().unwrap()).expect("mkdir tmp/");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args([
        "resolve",
        repo("tests/fixtures/simple_test/launch/pure_nodes.launch.xml")
            .to_str()
            .unwrap(),
        "-o",
        out_path.to_str().unwrap(),
    ]);
    let out = cmd.output().expect("failed to run play_launch");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "`resolve` must succeed on a clean launch file: {err}"
    );
    let model = std::fs::read_to_string(&out_path).unwrap_or_else(|e| {
        panic!(
            "`resolve` wrote no model at {}: {e}\n{err}",
            out_path.display()
        )
    });
    assert!(
        model.contains("structure:"),
        "the emitted file must be a SystemModel:\n{model}"
    );

    // `meta.resolver` must name the BINARY the user ran, at the version they
    // installed. It read this library crate's own `CARGO_PKG_*`, so every
    // model `play_launch resolve` wrote — the ONE user-facing artifact, which
    // people commit to git — was stamped `tool: ros-launch-resolve` /
    // `version: 0.1.0`: a developer binary that is not in the wheel, at a
    // version unrelated to the 0.9.0 that produced it.
    let parsed: serde_json::Value = serde_yaml_ng::from_str(&model).expect("parse model yaml");
    let resolver = &parsed["meta"]["resolver"];
    assert_eq!(
        resolver["tool"].as_str(),
        Some("play_launch"),
        "meta.resolver.tool must name the binary the user ran:\n{resolver:?}"
    );
    // Cross-checked against what the binary reports for `--version`, so this
    // needs no hard-coded number and cannot rot at the next release bump.
    let version_out = fixtures::play_launch_cmd(&env)
        .arg("--version")
        .output()
        .expect("failed to run play_launch --version");
    let reported = String::from_utf8_lossy(&version_out.stdout);
    let reported = reported
        .split_whitespace()
        .next_back()
        .expect("`--version` printed nothing");
    assert_eq!(
        resolver["version"].as_str(),
        Some(reported),
        "meta.resolver.version must be the version the binary reports \
         ({reported}), not the library crate's:\n{resolver:?}"
    );

    // ...and `up` must ACCEPT what `resolve` just wrote — the half of the
    // round trip that did not exist before this change (with `resolve` and
    // `dump` both gone from play_launch, a pip-only user had `up` and no
    // producer for its input).
    //
    // `up` supervises until signalled, so it is spawned under
    // `ManagedProcess` (setsid + PR_SET_PDEATHSIG + PGID kill on Drop) and
    // read back from its log, not run to completion. `--disable-all` keeps
    // it to the spawn path: no monitoring, diagnostics or web UI.
    let up_log = repo("tmp/migrated_verbs_up.log");
    let log_file = std::fs::File::create(&up_log).expect("create up log");
    let mut up = fixtures::play_launch_cmd(&env);
    up.args(["up", out_path.to_str().unwrap(), "--disable-all"])
        .stdout(std::process::Stdio::from(
            log_file.try_clone().expect("dup log fd"),
        ))
        .stderr(std::process::Stdio::from(log_file));
    let _guard = ManagedProcess::spawn(&mut up).expect("failed to spawn play_launch up");

    let deadline = Instant::now() + Duration::from_secs(30);
    let mut log = String::new();
    while Instant::now() < deadline {
        log = std::fs::read_to_string(&up_log).unwrap_or_default();
        if log.contains("Spawn source: SystemModel") {
            break;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    assert!(
        log.contains("Spawn source: SystemModel"),
        "`up` must load and spawn from the model `resolve` just wrote:\n{log}"
    );
    assert!(
        !log.contains("Failed to load SystemModel") && !log.contains("unexpected argument"),
        "`up` must not reject the model as the wrong artifact:\n{log}"
    );
}

/// `check` must run the checker and PASS on a clean launch file.
#[test]
fn check_runs_the_checker() {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args([
        "check",
        repo("tests/fixtures/simple_test/launch/pure_nodes.launch.xml")
            .to_str()
            .unwrap(),
    ]);
    let out = cmd.output().expect("failed to run play_launch");
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "`check` must exit 0 on a launch file with no contract errors: {err}"
    );
    assert!(
        !err.contains("was removed in 0.9.0"),
        "`check` is a live verb again, not a migration stub: {err}"
    );
}

/// THE trap this task exists to avoid: `verbs::check::run` returns
/// `Result<i32>` and deliberately does NOT call `std::process::exit`, so a
/// handler written as `verbs::check::run(inputs)?; Ok(())` compiles, prints
/// every diagnostic, and exits 0 — a contract checker that cannot fail.
/// Issues 0008, 0012 and 0014 are three earlier instances of that shape in
/// this repo.
///
/// The fixture's contract is deliberately inconsistent (publishers promise
/// 100 Hz on a 500 Hz channel → `rate-hierarchy` Errors), so this run MUST
/// exit non-zero.
#[test]
fn check_exits_nonzero_on_a_contract_error() {
    let env = fixtures::install_env();
    let launch = repo("tests/fixtures/contract_error/launch/bringup.launch.xml");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["check", launch.to_str().unwrap()]);
    let out = cmd.output().expect("failed to run play_launch");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);

    // The diagnostics must actually be produced -- otherwise a fixture that
    // silently stopped tripping the rule would make this test pass for the
    // wrong reason (no errors found, exit 0... which would then fail the
    // status assertion, but with a misleading message).
    assert!(
        stdout.contains("rate-hierarchy") || stderr.contains("rate-hierarchy"),
        "the fixture must still trip the rate-hierarchy rule:\n{stdout}\n{stderr}"
    );
    assert_eq!(
        out.status.code(),
        Some(1),
        "`check` must EXIT 1 when an Error-severity diagnostic survives \
         filtering -- a checker that prints errors and exits 0 is a checker \
         CI cannot use:\n{stdout}\n{stderr}"
    );
}

/// `--rule` narrows the exit code exactly as it narrows the printed output:
/// filtering to a rule that only yields warnings turns the same failing run
/// into a passing one. This is the library's documented contract (Task 8
/// report §3) and is the half most easily lost by a wrapper that computes
/// its own status.
#[test]
fn check_rule_filter_narrows_the_exit_code() {
    let env = fixtures::install_env();
    let launch = repo("tests/fixtures/contract_error/launch/bringup.launch.xml");

    let mut matching = fixtures::play_launch_cmd(&env);
    matching.args([
        "check",
        launch.to_str().unwrap(),
        "--rule",
        "rate-hierarchy",
    ]);
    assert_eq!(
        matching.output().expect("run").status.code(),
        Some(1),
        "filtering TO the failing rule must stay exit 1"
    );

    let mut excluding = fixtures::play_launch_cmd(&env);
    excluding.args([
        "check",
        launch.to_str().unwrap(),
        "--rule",
        "dangling-entity",
    ]);
    assert_eq!(
        excluding.output().expect("run").status.code(),
        Some(0),
        "filtering to a warning-only rule must exit 0"
    );
}

/// For these five verbs stdout is a DATA channel, not a log channel:
/// `resolve -o -` writes the SystemModel and `check --format json` writes a
/// report, both to be piped into another program. play_launch's tracing
/// subscriber writes to stdout for every other verb, so when these arrived
/// from `ros-launch-resolve` (whose subscriber writes to stderr) every
/// library log line landed in the middle of the artifact — ANSI escapes and
/// WARN lines inside the YAML/JSON. `main::logs_to_stderr` is the fix; this
/// is what stops it silently regressing.
#[test]
fn machine_readable_stdout_is_not_polluted_by_logs() {
    let env = fixtures::install_env();
    // A fixture that DOES produce log lines (it loads two provider
    // contracts) — against a silent one this test would pass vacuously.
    let launch = repo("tests/fixtures/contract_merge/launch/bringup.launch.xml");

    let mut resolve = fixtures::play_launch_cmd(&env);
    resolve.args(["resolve", launch.to_str().unwrap(), "-o", "-"]);
    let out = resolve.output().expect("run");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("Loaded 2 manifest(s)"),
        "the fixture must still emit log lines, or this test proves nothing: {stderr}"
    );
    assert!(
        !stdout.contains('\u{1b}') && !stdout.contains("manifest(s)"),
        "`resolve -o -` must write ONLY the model to stdout:\n{stdout}"
    );
    assert!(
        stdout.trim_start().starts_with("meta:"),
        "stdout must begin with the model's first key:\n{stdout}"
    );

    let err_launch = repo("tests/fixtures/contract_error/launch/bringup.launch.xml");
    let mut check = fixtures::play_launch_cmd(&env);
    check.args(["check", err_launch.to_str().unwrap(), "--format", "json"]);
    let out = check.output().expect("run");
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.trim_start().starts_with('[') || stdout.trim_start().starts_with('{'),
        "`check --format json` must write parseable JSON to stdout:\n{stdout}"
    );
    assert!(
        !stdout.contains('\u{1b}'),
        "no ANSI-coloured log line may reach the JSON report:\n{stdout}"
    );
}

/// Names of the subcommands clap lists under `Commands:` in a help page.
///
/// Entries are rendered at exactly two spaces of indent; wrapped description
/// lines are indented further, so the two-space test alone separates them.
fn subcommands_listed_in(help: &str) -> Vec<String> {
    let mut names = Vec::new();
    let mut in_commands = false;
    for line in help.lines() {
        if line.starts_with("Commands:") {
            in_commands = true;
            continue;
        }
        if in_commands {
            // A blank line or a new unindented section header ends the list.
            if line.trim().is_empty() || !line.starts_with(' ') {
                if !line.trim().is_empty() {
                    break;
                }
                continue;
            }
            let rest = match line.strip_prefix("  ") {
                Some(r) => r,
                None => continue,
            };
            if rest.starts_with(' ') {
                continue; // wrapped description line
            }
            let name = rest.split_whitespace().next().unwrap_or_default();
            if !name.is_empty() && name != "help" {
                names.push(name.to_string());
            }
        }
    }
    names
}

/// The wheel ships `play_launch` and nothing else, so help must never send a
/// user to a binary they do not have.
///
/// This walks EVERY subcommand's long help, not just the top level. The
/// original version inspected `play_launch --help` alone, which a leak in any
/// subcommand's `after_help` — exactly where the removed-verb examples that
/// caused this rule lived — would have walked straight past. Nested
/// subcommands (`contract eject`, `dump launch`) are reached by recursing on
/// each page's own `Commands:` list; `replay` is seeded explicitly because it
/// is `hide = true` and so appears in no listing.
#[test]
fn help_never_names_the_developer_binary() {
    let env = fixtures::install_env();

    let render = |path: &[String]| -> String {
        let mut cmd = fixtures::play_launch_cmd(&env);
        cmd.args(path.iter().map(String::as_str));
        cmd.arg("--help");
        let out = cmd.output().expect("failed to run play_launch");
        format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        )
    };

    let root = render(&[]);
    for verb in [
        "launch", "run", "up", "resolve", "dump", "check", "plot", "contract", "context", "setcap",
        "verify",
    ] {
        assert!(
            root.contains(&format!("  {verb}")),
            "--help must advertise `{verb}`:\n{root}"
        );
    }

    // Breadth-first over the whole command tree. `replay` is hidden but still
    // reachable, so its page is checked too.
    let mut queue: Vec<Vec<String>> = vec![vec![], vec!["replay".to_string()]];
    let mut checked = 0usize;
    while let Some(path) = queue.pop() {
        let help = if path.is_empty() {
            root.clone()
        } else {
            render(&path)
        };
        let label = if path.is_empty() {
            "play_launch --help".to_string()
        } else {
            format!("play_launch {} --help", path.join(" "))
        };
        assert!(
            !help.contains("ros-launch-resolve"),
            "`{label}` must not name the developer-only binary:\n{help}"
        );
        checked += 1;
        for sub in subcommands_listed_in(&help) {
            let mut next = path.clone();
            next.push(sub);
            queue.push(next);
        }
    }

    // A parser regression that returned no subcommands would make the walk
    // vacuous: 11 verbs + the root + hidden `replay` + nested `contract eject`
    // and `dump launch` is 15 pages, so anything near 1 means the walk broke.
    assert!(
        checked >= 15,
        "the help walk must reach every subcommand; it visited only {checked} pages"
    );
}

/// The same rule, on the surface `--help` cannot see: a TRIGGERED ERROR.
///
/// `help_never_names_the_developer_binary` above walks static help text. But
/// the five reshaped verbs live in `src/ros-launch-resolve/resolve/`, and
/// eyre's default handler appends the source location it captured to every
/// error a `fn main() -> Result<()>` reports:
///
/// ```text
/// Error: Launch file not found: /nonexistent/foo.launch.xml
///
/// Location:
///     src/ros-launch-resolve/resolve/src/verbs/mod.rs:93:24
/// ```
///
/// So every failing `resolve`/`dump`/`check`/`plot`/`contract` run printed the
/// developer-only binary's name to a user who only ever installed
/// `play_launch` — invisibly to any help audit. `util::cli_errors` replaces
/// that report; this pins it.
///
/// Each case must FAIL (a passing command proves nothing about error output),
/// and the assertion covers stdout and stderr together.
#[test]
fn a_triggered_error_never_names_the_developer_binary() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }

    // Every reshaped verb, each driven into a failure it owns.
    let cases: Vec<Vec<&str>> = vec![
        vec!["resolve", "/nonexistent/foo.launch.xml", "-o", "/dev/null"],
        vec!["check", "/nonexistent/foo.launch.xml"],
        vec!["dump", "launch", "/nonexistent/foo.launch.xml"],
        vec!["contract", "eject", "/nonexistent/foo.launch.xml"],
        vec!["plot", "--log-dir", "/nonexistent/logs"],
        // A package name that resolves to nothing takes the other arm of
        // `resolve_launch_file`, whose message this wave also rewrote.
        vec!["check", "no_such_package_anywhere", "no_such.launch.xml"],
    ];

    for args in cases {
        let mut cmd = fixtures::play_launch_cmd(&env);
        cmd.args(&args);
        let out = cmd.output().expect("failed to run play_launch");
        let combined = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        let label = format!("play_launch {}", args.join(" "));

        assert!(
            !out.status.success(),
            "`{label}` was supposed to FAIL — this case no longer tests error output:\n{combined}"
        );
        assert!(
            !combined.contains("ros-launch-resolve"),
            "`{label}` named the developer-only binary in its error output:\n{combined}"
        );
        // The location footer is how the name leaked; reject it directly too,
        // so a future handler change that reintroduces file paths is caught
        // even if the path happens not to contain the binary's name.
        assert!(
            !combined.contains("Location:"),
            "`{label}` printed an eyre `Location:` footer — an internal source \
             path is not a user-facing error report:\n{combined}"
        );
    }
}
