//! Integration tests for the `rt_workspace` fixture (Phase 39.3).
//!
//! `tests/fixtures/rt_workspace/` is a small, real, buildable ROS 2 workspace
//! (`rt_demo` package: `sensor_node`, `control_node`, `filter_component`)
//! doubling as both the runnable example behind `docs/guide/rt-scheduling.md`
//! and an integration fixture that exercises the RT apply-layer against our
//! own nodes rather than borrowed system packages. See
//! `docs/superpowers/specs/2026-07-15-rt-workspace-fixture-design.md` and
//! `.superpowers/sdd/p39-w1-report.md` (build gotchas, exact commands).
//!
//! Follows the `io_stress` pattern: excluded from `just test` (fast suite),
//! included in `just test-all`. All tests skip cleanly (not fail) if the
//! fixture hasn't been built (`cd tests/fixtures/rt_workspace && just build`).

use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use std::process::Stdio;
use std::time::Duration;

use play_launch_tests::fixtures;
use play_launch_tests::fixtures::array_len;
use play_launch_tests::process::ManagedProcess;

fn fixture_dir() -> PathBuf {
    fixtures::test_workspace_path("rt_workspace")
}

/// Returns `true` (and prints nothing) if the fixture is built, or prints a
/// clear skip message and returns `false` otherwise. Mirrors
/// `require_autoware()` (`tests/tests/autoware.rs`) in spirit but — since
/// this fixture is a colcon build product that may legitimately be absent on
/// a fresh checkout (like `io_stress`) — skips rather than panics.
fn require_rt_workspace() -> bool {
    let installed_pkg = fixture_dir().join("install/rt_demo");
    if !installed_pkg.is_dir() {
        eprintln!(
            "SKIP: rt_workspace fixture not built ({} missing) — run \
             `cd tests/fixtures/rt_workspace && just build` first",
            installed_pkg.display()
        );
        return false;
    }
    true
}

/// Local re-implementation of `play_launch`'s `has_sched_privilege()`
/// preflight (root or `CAP_SYS_NICE` in the effective capability set).
/// Duplicated from `tests/tests/sched_apply.rs` — the tests crate cannot
/// link against the `play_launch` binary crate directly.
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

/// Dump `bringup.launch.xml` (package mode: `rt_demo bringup.launch.xml`)
/// with the given parser, returning the parsed record and the temp dir
/// backing `record.json` (kept alive for the caller, e.g. for
/// `compare_records`).
fn dump_bringup(
    env: &HashMap<String, String>,
    parser: &str,
) -> (serde_json::Value, tempfile::TempDir) {
    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let output_path = tmp.path().join("record.json");

    let mut proc = ManagedProcess::spawn(fixtures::play_launch_cmd(env).args([
        "dump",
        "--output",
        output_path.to_str().unwrap(),
        "launch",
        "--parser",
        parser,
        "rt_demo",
        "bringup.launch.xml",
    ]))
    .expect("failed to spawn play_launch dump");

    let status = proc.wait_with_timeout(Duration::from_secs(60));
    assert!(
        status.success(),
        "play_launch dump (parser={parser}) failed"
    );

    let data = std::fs::read_to_string(&output_path).expect("failed to read record.json");
    let record = serde_json::from_str(&data).expect("failed to parse record.json");
    (record, tmp)
}

// ---- Dump parity ----

/// Rust vs Python parse of `bringup.launch.xml` must agree on entity counts
/// (2 nodes, 1 container, 1 load_node — `sensor_node`, `control_node`,
/// `perception_container`, `filter_component`) and be functionally
/// equivalent per `scripts/compare_records.py` (same check as the fixture's
/// own `just compare-dumps`).
#[test]
fn dump_parity_bringup() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let (rust_record, rust_tmp) = dump_bringup(&env, "rust");
    let (python_record, python_tmp) = dump_bringup(&env, "python");

    assert_eq!(array_len(&rust_record, "node"), 2, "rust: expected 2 nodes");
    assert_eq!(
        array_len(&rust_record, "container"),
        1,
        "rust: expected 1 container"
    );
    assert_eq!(
        array_len(&rust_record, "load_node"),
        1,
        "rust: expected 1 load_node"
    );

    assert_eq!(
        array_len(&python_record, "node"),
        array_len(&rust_record, "node"),
        "node count mismatch: rust={}, python={}",
        array_len(&rust_record, "node"),
        array_len(&python_record, "node")
    );
    assert_eq!(
        array_len(&python_record, "container"),
        array_len(&rust_record, "container"),
    );
    assert_eq!(
        array_len(&python_record, "load_node"),
        array_len(&rust_record, "load_node"),
    );

    let (ok, output) = fixtures::compare_records(
        &rust_tmp.path().join("record.json"),
        &python_tmp.path().join("record.json"),
    );
    assert!(
        ok,
        "compare_records.py reported a mismatch between rust and python dumps:\n{output}"
    );
}

// ---- `check` (contracts + --sched) ----

/// `play_launch check --sched system.toml rt_demo bringup.launch.xml`
/// (provider-sidecar-only channel) must exit 0 and report the provider
/// sidecar was loaded (`[0 overlay, 1 provider]`).
#[test]
fn check_provider_sidecar_passes() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let sched = fixture_dir().join("system.toml");
    assert!(
        sched.is_file(),
        "system.toml not found: {}",
        sched.display()
    );

    let output = fixtures::play_launch_cmd(&env)
        .args([
            "check",
            "rt_demo",
            "bringup.launch.xml",
            "--sched",
            sched.to_str().unwrap(),
        ])
        .output()
        .expect("failed to run play_launch check");

    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(
        output.status.success(),
        "check (provider sidecar only) exited nonzero:\n{combined}"
    );
    assert!(
        combined.contains("[0 overlay, 1 provider]"),
        "expected the provider-sidecar resolution line in check output:\n{combined}"
    );
}

/// `play_launch check --contracts contracts --sched system.toml rt_demo
/// bringup.launch.xml` must exit 0 and report the overlay overrode the
/// provider sidecar (`[1 overlay, 0 provider]`) — the actual proof the
/// overlay resolution engaged (the `max_age_ms` diff itself isn't visible in
/// `check`'s terminal output, only this resolved-channel line is; see
/// `.superpowers/sdd/p39-w1-report.md`).
#[test]
fn check_overlay_overrides_provider_sidecar() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let sched = fixture_dir().join("system.toml");
    let contracts = fixture_dir().join("contracts");
    assert!(
        contracts.is_dir(),
        "contracts overlay dir not found: {}",
        contracts.display()
    );

    let output = fixtures::play_launch_cmd(&env)
        .args([
            "check",
            "rt_demo",
            "bringup.launch.xml",
            "--contracts",
            contracts.to_str().unwrap(),
            "--sched",
            sched.to_str().unwrap(),
        ])
        .output()
        .expect("failed to run play_launch check");

    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(
        output.status.success(),
        "check (overlay + provider) exited nonzero:\n{combined}"
    );
    assert!(
        combined.contains("[1 overlay, 0 provider]"),
        "expected the overlay-overrides-provider resolution line in check output:\n{combined}"
    );
}

// ---- sched-apply smoke ----

/// Spawn `play_launch launch rt_demo bringup.launch.xml --sched system.toml
/// --sched-apply warn` and assert the apply path engages for the standalone
/// nodes, the container, and the composable — tolerant of host privilege
/// (reused pattern from `sched_apply.rs`): either the resolved-tier debug
/// line (privileged host) or the `sched apply failed` / `cap_sys_nice`
/// warning (unprivileged host, the expected case on this environment).
#[test]
fn sched_apply_warn_smoke_launch() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let sched = fixture_dir().join("system.toml");

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let stdout_path = work_tmp.path().join("stdout.log");
    let stderr_path = work_tmp.path().join("stderr.log");
    let stdout_file = fs::File::create(&stdout_path).expect("failed to create stdout file");
    let stderr_file = fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_tmp.path());
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--sched",
        sched.to_str().unwrap(),
        "--sched-apply",
        "warn",
        "rt_demo",
        "bringup.launch.xml",
    ]);
    cmd.stdout(Stdio::from(stdout_file));
    cmd.stderr(Stdio::from(stderr_file));
    // "applied tier '<name>' ..." success-path evidence is logged at debug
    // level — raise RUST_LOG so it's visible if this host is privileged.
    // Never rely on info! being promoted for test convenience.
    cmd.env("RUST_LOG", "play_launch=debug");

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_tmp.path().join("play_log/latest");
    // 2 standalone nodes (sensor_node, control_node) + 1 container
    // (perception_container); composables don't get a separate
    // `node/<name>/cmdline` directory (metadata lives in the parent
    // container's metadata.json).
    fixtures::wait_for_processes(&play_log, 3, Duration::from_secs(30));

    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(
        actual, 3,
        "process count mismatch: actual={actual}, expected=3 (2 nodes + 1 container)"
    );

    let stdout = fs::read_to_string(&stdout_path).unwrap_or_default();
    let stderr = fs::read_to_string(&stderr_path).unwrap_or_default();
    let combined = format!("{stdout}\n{stderr}").to_lowercase();

    // Standalone node (sensor_node or control_node): "[<name>] applied tier
    // '...' (pid ...)" (privileged) or "[<name>] sched apply failed (pid
    // ...): ..." (unprivileged) — excluding the composable-specific variants
    // of the same phrases.
    let standalone_engaged = combined.lines().any(|l| {
        (l.contains("applied tier") || l.contains("sched apply failed"))
            && !l.contains("composable")
    });
    assert!(
        standalone_engaged,
        "expected a standalone-node sched apply line (applied tier / sched \
         apply failed, not composable-scoped) in combined output:\n{combined}"
    );

    // Composable (filter_component, loaded into perception_container):
    // "applied tier '...' to composable '...' (pid ...)" (privileged) or
    // "sched apply failed for composable '...' (pid ...): ..." (unprivileged).
    let composable_engaged = combined.lines().any(|l| {
        (l.contains("applied tier") && l.contains("composable"))
            || l.contains("sched apply failed for composable")
    });
    assert!(
        composable_engaged,
        "expected a per-composable sched apply line (applied tier '...' to \
         composable '...', or sched apply failed for composable '...') in \
         combined output:\n{combined}"
    );

    // Also confirm the fallback/preflight evidence is present when
    // unprivileged, so a silent misconfiguration (e.g. --sched not even
    // parsed) can't slip through by accidentally matching the loose
    // substrings above.
    if !host_has_sched_privilege() {
        assert!(
            combined.contains("cap_sys_nice") || combined.contains("permission denied"),
            "expected unprivileged evidence (cap_sys_nice hint or permission \
             denied) in combined output on an unprivileged host:\n{combined}"
        );
    }

    // _proc dropped here — ManagedProcess::drop kills the process group.
}

// ---- Per-TID assertion (privileged hosts only) ----

/// On a host with `CAP_SYS_NICE`/root, every thread of `control_node` must
/// show `SCHED_FIFO` policy, priority 20, and cpu affinity `{0}` — the exact
/// tier `system.toml` assigns it. Auto-skips on unprivileged hosts (the
/// expected case here — no root/sudo steps are executed by this suite; see
/// `justfile`'s `verify-sched-rt` recipe for the manual privileged-host
/// version of this same check against `setcap`-granted `play_launch_rt_helper`).
#[test]
fn per_tid_sched_fifo_launch_privileged_only() {
    if !require_rt_workspace() {
        return;
    }
    if !host_has_sched_privilege() {
        eprintln!(
            "SKIP: host lacks CAP_SYS_NICE/root — per-TID SCHED_FIFO/affinity \
             assertion requires `play_launch setcap` (or root); skipping"
        );
        return;
    }

    let env = fixtures::rt_workspace_env();
    let sched = fixture_dir().join("system.toml");

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_tmp.path());
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--sched",
        sched.to_str().unwrap(),
        "--sched-apply",
        "strict",
        "rt_demo",
        "bringup.launch.xml",
    ]);
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_tmp.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 3, Duration::from_secs(30));

    let pid_path = play_log.join("node/control_node/pid");
    assert!(
        pid_path.is_file(),
        "control_node pid file not found: {}",
        pid_path.display()
    );
    let pid: i32 = fs::read_to_string(&pid_path)
        .expect("failed to read control_node pid file")
        .trim()
        .parse()
        .expect("control_node pid file did not contain an integer");

    let task_dir = format!("/proc/{pid}/task");
    let entries = fs::read_dir(&task_dir).unwrap_or_else(|e| {
        panic!("failed to read {task_dir}: {e} (did control_node exit early?)")
    });

    let mut threads = 0;
    let mut fifo_prio_20 = 0;
    for entry in entries.flatten() {
        let stat_path = entry.path().join("stat");
        let Ok(stat) = fs::read_to_string(&stat_path) else {
            continue; // thread exited mid-scan
        };
        threads += 1;

        // comm (field 2) is parenthesized and may itself contain ')' or
        // whitespace — split on the LAST ") " before splitting the
        // remaining whitespace-separated fields (mirrors justfile's
        // verify-sched-rt awk-free parsing).
        let rest = stat.rsplit_once(") ").map(|(_, r)| r).unwrap_or(&stat);
        let fields: Vec<&str> = rest.split_whitespace().collect();
        // fields[] is 0-indexed starting at stat field 3 (state), so field
        // 40 (rt_priority) is fields[37] and field 41 (policy) is fields[38].
        let rt_priority = fields.get(37).copied().unwrap_or("?");
        let policy = fields.get(38).copied().unwrap_or("?");

        // SCHED_FIFO == 1 (linux/sched.h)
        assert_eq!(
            policy,
            "1",
            "control_node thread {:?} expected SCHED_FIFO (policy=1), got policy={policy}",
            entry.path()
        );
        assert_eq!(
            rt_priority,
            "20",
            "control_node thread {:?} expected priority 20, got {rt_priority}",
            entry.path()
        );
        fifo_prio_20 += 1;
    }

    assert!(threads > 0, "control_node had no threads to inspect");
    assert_eq!(
        fifo_prio_20, threads,
        "expected every thread of control_node to be SCHED_FIFO prio 20, got {fifo_prio_20}/{threads}"
    );

    let status = fs::read_to_string(format!("/proc/{pid}/status"))
        .expect("failed to read control_node /proc/<pid>/status");
    let cpus_allowed_list = status
        .lines()
        .find_map(|l| l.strip_prefix("Cpus_allowed_list:"))
        .expect("Cpus_allowed_list not found in /proc/<pid>/status")
        .trim();
    assert_eq!(
        cpus_allowed_list, "0",
        "expected control_node pinned to cpu 0 (system.toml core=0), got Cpus_allowed_list={cpus_allowed_list}"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group.
}
