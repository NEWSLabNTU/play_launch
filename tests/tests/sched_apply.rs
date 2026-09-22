//! Integration smoke test for the `--sched` apply-layer (Phase 38.8).
//!
//! Unit tests for `sched_apply` / `sched_plan` / `sched_loader` already cover
//! the syscall layer and FQN resolution in isolation. This test is the
//! end-to-end smoke: run `play_launch launch` against the lightest CI-runnable
//! fixture (`simple_test/pure_nodes.launch.xml`, same as
//! `simple_workspace::test_launch_pure_nodes`) with a `--sched` spec that
//! assigns every node (via `scope = "/"`, so it's robust to the fixture's
//! exact node names) to a real-time tier, and assert the apply-layer engages.
//!
//! "Engages" is asserted tolerant of host privilege:
//! - On a host with `CAP_SYS_NICE`/root, `play_launch` actually applies the
//!   tier and logs `applied tier 'rt'` (debug-level — hence `RUST_LOG=play_launch=debug`
//!   in the test env).
//! - On an unprivileged host (the common CI case), the preflight hint
//!   containing `cap_sys_nice` is logged instead (warn-level, always visible).
//!
//! Either substring proves the apply-layer ran, not just the parser/resolver.

use std::process::Stdio;
use std::time::Duration;

use play_launch_tests::fixtures;
use play_launch_tests::process::ManagedProcess;

/// Write a scheduling spec TOML assigning every node in the launch tree to a
/// single `rt` real-time tier via a root scope selector. Using `scope = "/"`
/// instead of explicit node names keeps this test robust to the fixture's
/// exact FQNs.
fn write_sched_toml(dir: &std::path::Path) -> std::path::PathBuf {
    let path = dir.join("system.toml");
    std::fs::write(
        &path,
        r#"
[tiers.rt]
class = "real_time"

[tiers.rt.posix]
priority = 20
sched_class = "SCHED_FIFO"

[[assign]]
tier = "rt"
scope = "/"
"#,
    )
    .expect("failed to write system.toml");
    path
}

/// Local re-implementation of `play_launch`'s `has_sched_privilege()`
/// preflight (root or `CAP_SYS_NICE` in the effective capability set). The
/// tests crate cannot link against the `play_launch` binary crate directly,
/// so this mirrors `src/play_launch/src/execution/sched_apply.rs` closely
/// enough to gate the strict-abort assertion below.
fn host_has_sched_privilege() -> bool {
    // Delegates to the shared probe, which also inspects the
    // installed rt_helper's FILE capability. The earlier local
    // version read only `/proc/self/status`, so on a machine where
    // `just setcap` had been run it reported "unprivileged", and the
    // strict-abort test then waited 30s for an abort that could never
    // happen.
    fixtures::host_can_apply_rt_sched()
}

/// Spawn `play_launch launch` for the given launch file with `--sched
/// <sched_path> --sched-apply <mode>`, redirecting stdout/stderr to files
/// under `work_dir` (a fresh temp directory, so `play_log/latest` doesn't
/// collide with other tests running concurrently against the same fixture).
/// Returns the spawned guard plus the stdout/stderr paths.
fn spawn_sched_launch_for(
    env: &std::collections::HashMap<String, String>,
    work_dir: &std::path::Path,
    sched_path: &std::path::Path,
    launch: &std::path::Path,
    mode: &str,
) -> (ManagedProcess, std::path::PathBuf, std::path::PathBuf) {
    spawn_sched_launch_in_mode(env, work_dir, sched_path, launch, mode, None)
}

/// [`spawn_sched_launch_for`] with an explicit `--container-mode` (phase 80):
/// the flag decides whether a composable is a process of its own, and
/// therefore whether the tier the model binds to it can be applied at all.
fn spawn_sched_launch_in_mode(
    env: &std::collections::HashMap<String, String>,
    work_dir: &std::path::Path,
    sched_path: &std::path::Path,
    launch: &std::path::Path,
    mode: &str,
    container_mode: Option<&str>,
) -> (ManagedProcess, std::path::PathBuf, std::path::PathBuf) {
    let stdout_path = work_dir.join("stdout.log");
    let stderr_path = work_dir.join("stderr.log");
    let stdout_file = std::fs::File::create(&stdout_path).expect("failed to create stdout file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(env);
    cmd.current_dir(work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--sched",
        sched_path.to_str().unwrap(),
        "--sched-apply",
        mode,
    ]);
    if let Some(cm) = container_mode {
        cmd.args(["--container-mode", cm]);
    }
    cmd.arg(launch.to_str().unwrap());
    cmd.stdout(Stdio::from(stdout_file));
    cmd.stderr(Stdio::from(stderr_file));
    // The success-path evidence ("applied tier '<name>'") is logged at
    // debug level — raise RUST_LOG so it's visible. Never rely on info!
    // being promoted for test convenience.
    cmd.env("RUST_LOG", "play_launch=debug");

    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");
    (proc, stdout_path, stderr_path)
}

/// Convenience wrapper of [`spawn_sched_launch_for`] for the `simple_test`
/// pure_nodes fixture (used by the pre-existing warn/strict tests below).
fn spawn_sched_launch(
    env: &std::collections::HashMap<String, String>,
    work_dir: &std::path::Path,
    sched_path: &std::path::Path,
    mode: &str,
) -> (ManagedProcess, std::path::PathBuf, std::path::PathBuf) {
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");
    spawn_sched_launch_for(env, work_dir, sched_path, &launch, mode)
}

/// Wait until `output_path` contains at least `count` lines matching any of
/// `patterns`, or until `timeout` elapses. Returns the number of matching
/// lines found. Mirrors `wait_for_pattern` in `container_events.rs`.
fn wait_for_pattern(
    output_path: &std::path::Path,
    patterns: &[&str],
    count: usize,
    timeout: Duration,
) -> usize {
    let start = std::time::Instant::now();
    let mut found = 0;
    while start.elapsed() < timeout {
        std::thread::sleep(Duration::from_secs(1));
        let content = std::fs::read_to_string(output_path).unwrap_or_default();
        found = content
            .lines()
            .filter(|l| patterns.iter().any(|p| l.contains(p)))
            .count();
        if found >= count {
            return found;
        }
    }
    found
}

/// `--sched-apply warn`: the run must reach steady state (same process count
/// as a plain `simple_workspace` launch — the apply-layer never blocks or
/// crashes node startup), and stderr/stdout together must contain evidence
/// the apply-layer engaged: either the resolved tier name (`applied tier
/// 'rt'`, privileged host) or the `cap_sys_nice` preflight hint (unprivileged
/// host).
#[test]
fn sched_apply_warn_engages_and_launch_succeeds() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");

    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let sched_path = write_sched_toml(tmp.path());

    // First resolve to get the expected process count (mirrors
    // simple_workspace::test_launch_pure_nodes). Phase 47.B6: model, not
    // record.json (`dump`'s only artifact now).
    let (model, _model_tmp) = fixtures::resolve_model(&env, launch.to_str().unwrap(), None, "rust");
    let expected = fixtures::count_expected_processes_from_model(&model);
    assert!(expected > 0, "expected at least 1 process");

    // Fresh temp work dir: avoids play_log/latest colliding with
    // simple_workspace's own use of the fixture directory as cwd.
    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let (_proc, stdout_path, stderr_path) =
        spawn_sched_launch(&env, work_tmp.path(), &sched_path, "warn");

    let play_log = work_tmp.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, expected, Duration::from_secs(15));

    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(
        actual, expected,
        "process count mismatch: actual={actual}, expected={expected} \
         (apply-layer should never block or crash node startup)"
    );

    let stdout = std::fs::read_to_string(&stdout_path).unwrap_or_default();
    let stderr = std::fs::read_to_string(&stderr_path).unwrap_or_default();
    let combined = format!("{stdout}\n{stderr}").to_lowercase();

    let engaged = (combined.contains("applied tier") && combined.contains("'rt'"))
        || combined.contains("cap_sys_nice");
    assert!(
        engaged,
        "expected evidence the sched apply-layer engaged (either \
         \"applied tier 'rt'\" on a privileged host, or a cap_sys_nice \
         preflight hint on an unprivileged host) in combined output:\n{combined}"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group.
}

/// `--sched-apply strict` on an unprivileged host must abort BEFORE any node
/// comes up (non-zero exit + `cap_sys_nice` in stderr). If this test host
/// actually has `CAP_SYS_NICE`/root, the apply succeeds and there is nothing
/// to abort on — skip the assertion rather than flaking on privileged CI
/// runners.
#[test]
fn sched_apply_strict_aborts_before_spawn_when_unprivileged() {
    if host_has_sched_privilege() {
        eprintln!(
            "SKIP: sched_apply_strict_aborts_before_spawn_when_unprivileged: host has \
             CAP_SYS_NICE/root, so --sched-apply strict would succeed rather than abort"
        );
        return;
    }

    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let sched_path = write_sched_toml(tmp.path());

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let (mut proc, _stdout_path, stderr_path) =
        spawn_sched_launch(&env, work_tmp.path(), &sched_path, "strict");

    // The preflight abort happens before any node is spawned, so this should
    // return quickly; 30s is generous headroom for parser + colcon startup.
    let status = proc.wait_with_timeout(Duration::from_secs(30));

    let stderr = std::fs::read_to_string(&stderr_path).unwrap_or_default();

    assert!(
        !status.success(),
        "expected --sched-apply strict to abort on an unprivileged host \
         (exit status: {status:?})\nstderr:\n{stderr}"
    );
    assert!(
        stderr.to_lowercase().contains("cap_sys_nice"),
        "expected a 'cap_sys_nice' hint in strict-abort stderr:\n{stderr}"
    );
}

/// Phase 38.9: composable-node processes (not just top-level nodes/containers)
/// must go through the RT apply path. `container_events` (`--container-mode`
/// defaults to `isolated`) launches 1 container + 2 composable nodes
/// (Talker, Listener); each composable is its own fork+exec'd process with
/// its own pid, delivered via `ComponentEvent::LOADED`. On LOADED, the
/// container actor (`container_actor/component_events.rs`) looks up the
/// composable's resolved tier and calls `apply_tier(event.pid, tier)`:
/// - unprivileged host (EPERM): `warn!("{name}: sched apply failed for
///   composable '{composable}' (pid {pid}): {err}")`
/// - privileged host: `debug!("{name}: applied tier '{tier}' to composable
///   '{composable}' (pid {pid})")` — hence `RUST_LOG=play_launch=debug` in
///   `spawn_sched_launch_for`.
///
/// Asserting on either line (tolerant of host privilege, same pattern as
/// `sched_apply_warn_engages_and_launch_succeeds` above) proves a
/// per-composable pid was delivered, a tier was resolved for it via
/// `scope = "/"`, and `apply_tier` was actually invoked for that composable
/// — the thing that is new in 38.9, as opposed to the pre-38.9 behavior of
/// only scheduling the container process itself.
#[test]
fn composable_scheduling_engages_on_isolated_container() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("container_events")
        .join("launch/container_events.launch.xml");

    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let sched_path = write_sched_toml(tmp.path());

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let (_proc, stdout_path, stderr_path) =
        spawn_sched_launch_for(&env, work_tmp.path(), &sched_path, &launch, "warn");

    // Wait for both composables to report LOADED before inspecting the
    // apply-layer evidence — the ComponentEvent (and thus the apply_tier
    // call) fires as part of handling that event.
    let loaded = wait_for_pattern(
        &stdout_path,
        &["ComponentEvent LOADED", "LoadSucceeded"],
        2,
        Duration::from_secs(30),
    );

    let stdout = std::fs::read_to_string(&stdout_path).unwrap_or_default();
    let stderr = std::fs::read_to_string(&stderr_path).unwrap_or_default();
    let combined = format!("{stdout}\n{stderr}");

    assert!(
        loaded >= 2,
        "expected 2 LOADED events for the container_events fixture (talker, \
         listener), found {loaded}\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
    );

    // Find the specific composable apply/warn line (verbatim), rather than a
    // loose substring match, so the assertion can only pass if a
    // per-composable apply attempt was actually logged.
    let composable_line = combined.lines().find(|l| {
        let ll = l.to_lowercase();
        (ll.contains("applied tier") && ll.contains("to composable"))
            || ll.contains("sched apply failed for composable")
    });

    assert!(
        composable_line.is_some(),
        "expected a per-composable sched apply line (either \"applied tier \
         '...' to composable '...'\" on a privileged host, or \"sched apply \
         failed for composable '...'\" on an unprivileged host) in combined \
         output:\n{combined}"
    );
    eprintln!(
        "matched composable apply line: {}",
        composable_line.unwrap()
    );

    // _proc dropped here — ManagedProcess::drop kills the process group.
}

// ── Phase 80 / issue #0035: a tier the container mode cannot apply ──
//
// `resolve --sched` derives a tier for every node carrying a timing fact,
// composables included, and `check --explain` prints it. Under
// `--container-mode observable` or `stock` a composable is a thread pool
// inside the container's process, so the LOADED handler is handed pid 0 and
// correctly declines to apply the tier — and used to decline in complete
// silence, leaving the model's `execution.bindings` promising a priority
// that `ps -eLo tid,cls,rtprio` would never show.
//
// The fixture is `container_events` (1 container, 2 composables: talker,
// listener) with the same root-scope `system.toml` the tests above use, so
// every node — composables included — is bound to the `rt` tier.

/// The composable FQNs the `container_events` fixture resolves to. Both
/// declare `namespace=""`, so they sit at the root.
const FIXTURE_COMPOSABLES: [&str; 2] = ["/talker", "/listener"];

/// Under `--container-mode stock` every tiered composable must be named,
/// once, before anything is spawned.
#[test]
fn composable_tier_unapplied_under_stock_is_named() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("container_events")
        .join("launch/container_events.launch.xml");

    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let sched_path = write_sched_toml(tmp.path());

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let (_proc, stdout_path, stderr_path) = spawn_sched_launch_in_mode(
        &env,
        work_tmp.path(),
        &sched_path,
        &launch,
        "warn",
        Some("stock"),
    );

    // The decision is made before spawning, so this lands early; the wait is
    // headroom for parser + model build.
    // `launch` writes its tracing output to stdout (`main.rs`'s
    // `logs_to_stderr` is false for it); the refusal in the strict test below
    // is an eyre error and goes to stderr instead.
    let named = wait_for_pattern(
        &stdout_path,
        &["scheduling: composable"],
        FIXTURE_COMPOSABLES.len(),
        Duration::from_secs(30),
    );
    assert_eq!(
        named,
        FIXTURE_COMPOSABLES.len(),
        "expected one warning per tiered composable, found {named}"
    );

    let stdout = std::fs::read_to_string(&stdout_path).unwrap_or_default();
    let stderr = std::fs::read_to_string(&stderr_path).unwrap_or_default();
    let combined = format!("{stdout}\n{stderr}");

    for fqn in FIXTURE_COMPOSABLES {
        let named = combined.lines().find(|l| {
            l.contains("scheduling: composable") && l.contains(&format!("composable '{fqn}'"))
        });
        assert!(
            named.is_some(),
            "expected a pre-spawn warning naming composable '{fqn}' under \
             --container-mode stock, in:\n{combined}"
        );
        let line = named.unwrap();
        // The message has to carry the three things a user needs: the tier it
        // was given, the mode that dropped it, and the mode that would not.
        assert!(line.contains("'rt'"), "tier not named: {line}");
        assert!(
            line.contains("--container-mode stock"),
            "mode not named: {line}"
        );
        assert!(
            line.contains("--container-mode isolated"),
            "remedy not named: {line}"
        );
    }

    // And it is recorded, not just printed: `measure` reads run_info.json.
    let run_info_path = work_tmp.path().join("play_log/latest/run_info.json");
    let run_info: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(&run_info_path)
            .unwrap_or_else(|e| panic!("read {}: {e}", run_info_path.display())),
    )
    .expect("run_info.json is valid JSON");
    let rows = run_info["sched_unapplied"]
        .as_array()
        .unwrap_or_else(|| panic!("run_info.json carries no sched_unapplied: {run_info}"));
    assert_eq!(rows.len(), 2, "{run_info}");
    let nodes: Vec<&str> = rows.iter().filter_map(|r| r["node"].as_str()).collect();
    for fqn in FIXTURE_COMPOSABLES {
        assert!(nodes.contains(&fqn), "{run_info}");
    }

    // _proc dropped here — ManagedProcess::drop kills the process group.
}

/// `--sched-apply strict` under a mode that cannot apply a composable's tier
/// refuses at the start boundary — non-zero exit, and no node directory ever
/// written. Unlike the CAP_SYS_NICE strict test above, this one does not
/// depend on host privilege: the refusal is decided from the container mode
/// and the plan alone, before the privilege check is reached.
#[test]
fn composable_tier_strict_refuses_before_spawn_under_stock() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("container_events")
        .join("launch/container_events.launch.xml");

    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let sched_path = write_sched_toml(tmp.path());

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let (mut proc, _stdout_path, stderr_path) = spawn_sched_launch_in_mode(
        &env,
        work_tmp.path(),
        &sched_path,
        &launch,
        "strict",
        Some("stock"),
    );

    let status = proc.wait_with_timeout(Duration::from_secs(30));
    let stderr = std::fs::read_to_string(&stderr_path).unwrap_or_default();

    assert!(
        !status.success(),
        "expected --sched-apply strict to refuse a run whose composable tiers \
         cannot be applied (exit status: {status:?})\nstderr:\n{stderr}"
    );
    assert!(
        stderr.contains("--sched-apply strict") && stderr.contains("composable"),
        "expected the refusal to name the composable tiers:\n{stderr}"
    );
    for fqn in FIXTURE_COMPOSABLES {
        assert!(
            stderr.contains(&format!("composable '{fqn}'")),
            "expected '{fqn}' in the refusal:\n{stderr}"
        );
    }

    // Before spawn means before spawn: no node directory carries a cmdline.
    let play_log = work_tmp.path().join("play_log/latest");
    let spawned = fixtures::count_cmdline_files(&play_log);
    assert_eq!(
        spawned,
        0,
        "strict must refuse BEFORE any node is spawned, found {spawned} \
         cmdline files under {}",
        play_log.display()
    );
}

/// The assertion that stops the silent regression coming back from the other
/// side: under `--container-mode isolated` — the default, and the path
/// almost every user takes — the same model must produce NO such warning,
/// because every composable is fork+exec'd and its tier IS applied.
/// `composable_scheduling_engages_on_isolated_container` above asserts the
/// apply itself; this asserts the absence of the report, and that
/// `run_info.json` says "asked, none" rather than staying silent.
#[test]
fn composable_tier_silent_under_isolated() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("container_events")
        .join("launch/container_events.launch.xml");

    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let sched_path = write_sched_toml(tmp.path());

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let (_proc, stdout_path, stderr_path) = spawn_sched_launch_in_mode(
        &env,
        work_tmp.path(),
        &sched_path,
        &launch,
        "warn",
        Some("isolated"),
    );

    // Wait for the composables to load, so the run has demonstrably passed
    // the point at which a stock run would have complained.
    let loaded = wait_for_pattern(
        &stdout_path,
        &[
            "ComponentEvent LOADED",
            "LoadSucceeded",
            "control channel LOADED",
        ],
        2,
        Duration::from_secs(30),
    );

    let stdout = std::fs::read_to_string(&stdout_path).unwrap_or_default();
    let stderr = std::fs::read_to_string(&stderr_path).unwrap_or_default();
    let combined = format!("{stdout}\n{stderr}");

    assert!(
        loaded >= 2,
        "expected 2 composables to load under isolated, found {loaded}\n{combined}"
    );
    assert!(
        !combined.contains("scheduling: composable"),
        "isolated applies every composable tier — no unapplied-tier warning \
         may be printed:\n{combined}"
    );

    // Asked, and the answer was none. Absent would mean never asked.
    let run_info_path = work_tmp.path().join("play_log/latest/run_info.json");
    let run_info: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(&run_info_path)
            .unwrap_or_else(|e| panic!("read {}: {e}", run_info_path.display())),
    )
    .expect("run_info.json is valid JSON");
    assert_eq!(
        run_info["sched_unapplied"].as_array().map(Vec::len),
        Some(0),
        "isolated must record an EMPTY unapplied set, not no set: {run_info}"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group.
}
