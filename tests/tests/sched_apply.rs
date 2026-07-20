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
    // SAFETY: geteuid() takes no arguments and cannot fail.
    if unsafe { libc::geteuid() } == 0 {
        return true;
    }

    const CAP_SYS_NICE: u64 = 23;

    let Ok(status) = std::fs::read_to_string("/proc/self/status") else {
        return false;
    };

    for line in status.lines() {
        if let Some(hex) = line.strip_prefix("CapEff:") {
            let hex = hex.trim();
            if let Ok(mask) = u64::from_str_radix(hex, 16) {
                return (mask & (1 << CAP_SYS_NICE)) != 0;
            }
            return false;
        }
    }

    false
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
        launch.to_str().unwrap(),
    ]);
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

    // First dump to get the expected process count (mirrors
    // simple_workspace::test_launch_pure_nodes). Phase 46.5: `--format
    // record` keeps the legacy record.json shape `count_expected_processes`
    // reads (dump's default is the SystemModel).
    let record_path = tmp.path().join("record.json");
    let mut dump_proc = ManagedProcess::spawn(fixtures::play_launch_cmd(&env).args([
        "dump",
        "--output",
        record_path.to_str().unwrap(),
        "--format",
        "record",
        "launch",
        "--parser",
        "rust",
        launch.to_str().unwrap(),
    ]))
    .expect("failed to spawn dump");
    let status = dump_proc.wait_with_timeout(Duration::from_secs(60));
    assert!(status.success());

    let expected = fixtures::count_expected_processes(&record_path);
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
            "host has CAP_SYS_NICE/root — --sched-apply strict would succeed, \
             not abort; skipping strict-abort assertion"
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
