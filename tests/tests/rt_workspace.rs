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

use std::{fs, path::PathBuf, process::Stdio, time::Duration};

use play_launch_tests::{fixtures, fixtures::array_len, process::ManagedProcess};

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

// ---- Dump parity (Phase 47.B1 — moved onto the SystemModel) ----

/// Rust vs Python parse of `bringup.launch.xml` must agree on entity counts
/// (2 nodes, 1 container, 1 composable — `sensor_node`, `control_node`,
/// `perception_container`, `filter_component`) and be functionally
/// equivalent per `scripts/compare_models.py` (same check as the fixture's
/// own `just compare-dumps`).
///
/// Phase 47.B1 — the parser-parity gate moved off `record.json` onto the
/// unified SystemModel (`play_launch resolve`), the prerequisite for
/// hard-removing `record.json` in 47.B2+. This fixture also exercises the
/// contracts/scheduling layers (`bringup.contract.yaml` +
/// `bringup.system.posix.yaml`, auto-discovered by channel resolution),
/// which `compare_models.py` compares whenever either side is non-empty.
#[test]
fn dump_parity_bringup() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let (rust_model, rust_tmp) =
        fixtures::resolve_model(&env, "rt_demo", Some("bringup.launch.xml"), "rust");
    let (python_model, python_tmp) =
        fixtures::resolve_model(&env, "rt_demo", Some("bringup.launch.xml"), "python");

    let (rust_nodes, rust_containers, rust_composables) =
        fixtures::model_entity_counts(&rust_model);
    assert_eq!(rust_nodes, 2, "rust: expected 2 nodes");
    assert_eq!(rust_containers, 1, "rust: expected 1 container");
    assert_eq!(rust_composables, 1, "rust: expected 1 composable");

    let (python_nodes, python_containers, python_composables) =
        fixtures::model_entity_counts(&python_model);
    assert_eq!(
        python_nodes, rust_nodes,
        "node count mismatch: rust={rust_nodes}, python={python_nodes}"
    );
    assert_eq!(python_containers, rust_containers);
    assert_eq!(python_composables, rust_composables);

    let (ok, output) = fixtures::compare_models(
        &rust_tmp.path().join("system_model.yaml"),
        &python_tmp.path().join("system_model.yaml"),
    );
    assert!(
        ok,
        "compare_models.py reported a mismatch between rust and python models:\n{output}"
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

// ---- v2: platform-file channel resolution + --explain + per-target coexistence ----

/// `play_launch check rt_demo bringup.launch.xml` with **no `--sched` at
/// all** must resolve `launch/bringup.system.posix.yaml` through the
/// provider-sidecar channel (Phase 41.3) and print its provenance line, and
/// derive with the v2 `chain_aware` mapper (Phase 44.5: switched from
/// `rate_monotonic` as the showcase for the fixture's `points_to_cmd` chain)
/// — not `manual`, proving the v2 file, not `system.toml`, was picked. Also
/// asserts 0 manifest errors (deliverable 4a: the chain fixture must check
/// clean).
#[test]
fn check_channel_resolution_picks_provider_platform_file_with_no_sched_flag() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let output = fixtures::play_launch_cmd(&env)
        .args(["check", "rt_demo", "bringup.launch.xml"])
        .output()
        .expect("failed to run play_launch check");

    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(
        output.status.success(),
        "check (no --sched, channel resolution) exited nonzero:\n{combined}"
    );
    assert!(
        combined.contains("Scheduling platform file [provider]:")
            && combined.contains("bringup.system.posix.yaml"),
        "expected the provider platform-file provenance line in check output:\n{combined}"
    );
    assert!(
        combined.contains("mapper=chain_aware"),
        "expected the v2 chain_aware mapper (not the legacy manual bridge) \
         to have been selected:\n{combined}"
    );
    assert!(
        combined.contains("0 errors,"),
        "expected the chain fixture to check with 0 manifest errors:\n{combined}"
    );
}

/// A user overlay platform file (`--contracts contracts`, still no
/// `--sched`) must win over the provider sidecar for the same `(stem,
/// target)` — `contracts/rt_demo/launch/bringup.system.posix.yaml` widens
/// the band and re-pins `control_node` to priority 22 (vs the provider's
/// 20).
#[test]
fn check_channel_resolution_overlay_platform_file_beats_provider() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let contracts = fixture_dir().join("contracts");
    assert!(
        contracts.is_dir(),
        "contracts overlay dir not found: {}",
        contracts.display()
    );

    let output = fixtures::play_launch_cmd(&env)
        .args(["check", "--contracts"])
        .arg(&contracts)
        .args(["rt_demo", "bringup.launch.xml"])
        .output()
        .expect("failed to run play_launch check");

    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(
        output.status.success(),
        "check (overlay platform file, no --sched) exited nonzero:\n{combined}"
    );
    assert!(
        combined.contains("Scheduling platform file [overlay]:")
            && combined.contains("bringup.system.posix.yaml"),
        "expected the overlay platform-file provenance line in check output:\n{combined}"
    );
    assert!(
        combined.contains("prio=22") && combined.contains("control_node"),
        "expected the overlay's control_node pin (priority 22, not the \
         provider's 20) in the tier table:\n{combined}"
    );
}

/// `--explain` against the fixture's explicit provider platform file must
/// show all provenance kinds design doc §7 documents, now under the
/// `chain_aware` mapper (Phase 44.5): `derived(chain_aware: points_to_cmd
/// segment drain ...)` for `filter_component` (the chain's event-segment
/// sink relative to `sensor_node`), `derived(chain_aware: points_to_cmd
/// boundary RM period=10ms)` for `sensor_node` (the chain's timer
/// boundary), `override(control_node)` for the pinned node (overrides
/// still beat the chain-derived rank), and `default (no timing facts)` for
/// the fact-less container — plus the `system file:`/`contract[...]:`
/// footer lines.
///
/// Priorities are asserted **exactly**, computed by hand from the
/// chain_aware mapper spec against this fixture's `points_to_cmd` chain
/// (band 10-40, see `.superpowers/sdd/p44-w5-report.md` for the full
/// derivation): the chain resolves to one `Boundary` (`sensor_node.tick`,
/// 100 Hz) followed by one `Segment` (`filter_component.filter` ->
/// `control_node.control`, in source-to-sink declaration order). The
/// mapper walks chain elements sink-to-source and, within a `Segment`,
/// drain-toward-sink (downstream ranks above upstream): `control_node`
/// (segment sink) > `filter_component` (segment source) > `sensor_node`
/// (boundary, ranked below every segment item). Dense-rank-to-band
/// (`band.max - k`, k=0,1,2 for a 3-item, no-collapse-needed sequence)
/// gives `control_node`=40, `filter_component`=39, `sensor_node`=38 — but
/// `control_node` is separately pinned by the platform file's `overrides:`
/// to 20 (overrides always win), so only `filter_component`=39 and
/// `sensor_node`=38 surface as chain_aware-derived priorities here.
#[test]
fn check_explain_shows_derived_override_and_default_provenance() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let sched = fixture_dir().join("launch/bringup.system.posix.yaml");
    assert!(
        sched.is_file(),
        "provider platform file not found: {}",
        sched.display()
    );

    let output = fixtures::play_launch_cmd(&env)
        .args([
            "check",
            "--sched",
            sched.to_str().unwrap(),
            "--explain",
            "rt_demo",
            "bringup.launch.xml",
        ])
        .output()
        .expect("failed to run play_launch check --explain");

    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(
        output.status.success(),
        "check --explain exited nonzero:\n{combined}"
    );
    assert!(
        combined.contains("points_to_cmd"),
        "expected the fixture's chain name in --explain provenance:\n{combined}"
    );
    assert!(
        combined.contains("derived(chain_aware: points_to_cmd segment drain")
            && combined.contains("-> prio 39"),
        "expected filter_component's segment-drain provenance at the \
         by-hand-derived priority 39:\n{combined}"
    );
    assert!(
        combined.contains("derived(chain_aware: points_to_cmd boundary RM period=10ms)")
            && combined.contains("-> prio 38"),
        "expected sensor_node's boundary-RM provenance at the by-hand-derived \
         priority 38:\n{combined}"
    );
    assert!(
        combined.contains("override(control_node)"),
        "expected an override(control_node) provenance entry (overrides beat \
         the chain-derived rank 40):\n{combined}"
    );
    assert!(
        combined.contains("default (no timing facts)"),
        "expected a default (no timing facts) provenance entry for the \
         fact-less container:\n{combined}"
    );
    assert!(
        combined.contains("system file:") && combined.contains("contract["),
        "expected the system file / contract provenance footer lines:\n{combined}"
    );
    // The fixture's override pins control_node (the chain sink) to priority
    // 20, BELOW filter_component's derived 39 — the loader must call out
    // that the pin defeats drain-toward-sink ordering (44.5 review).
    assert!(
        combined.contains("below its chain-derived rank"),
        "expected the override-inversion warning for control_node's pin:\n{combined}"
    );
}

/// Deliverable 4(c): an overlay that tightens the fixture's chain budget
/// below the chain's own `sampling_cost` (the `sensor_node.tick` boundary's
/// period alone, 10ms at 100 Hz) must trigger the `chain-sampling-
/// feasibility` warning (structural infeasibility — no scheduling
/// assignment can fix it) — `check` must still exit 0 (warnings never
/// fail `check`).
#[test]
fn check_chain_budget_tightened_below_sampling_cost_warns_infeasible() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let overlay_root = tempfile::TempDir::new().expect("failed to create overlay root");
    let overlay_launch_dir = overlay_root.path().join("rt_demo/launch");
    fs::create_dir_all(&overlay_launch_dir).expect("failed to create overlay launch dir");
    fs::write(
        overlay_launch_dir.join("bringup.contract.yaml"),
        "\
version: 1

nodes:
  sensor_node:
    pub:
      points_raw:
        min_rate_hz: 100
    paths:
      tick:
        trigger: { timer: { rate_hz: 100 } }
        output: [points_raw]

  filter_component:
    sub:
      points_raw:
        min_rate_hz: 100
    pub:
      points_filtered:
        min_rate_hz: 100
    paths:
      filter:
        trigger: { input: [points_raw] }
        output: [points_filtered]
        max_latency_ms: 5

  control_node:
    criticality: high
    sub:
      points_filtered:
        min_rate_hz: 100
        max_age_ms: 50
    pub:
      cmd:
        min_rate_hz: 100
    paths:
      control:
        trigger: { input: [points_filtered] }
        output: [cmd]
        max_latency_ms: 10

chains:
  points_to_cmd:
    semantics: reaction
    max_latency_ms: 5
    segments:
      - { scope: /, path: tick }
      - { via: /perception/points_raw }
      - { scope: /, path: filter }
      - { via: /perception/points_filtered }
      - { scope: /, path: control }

topics:
  /perception/points_raw:
    type: std_msgs/msg/String
    pub: [sensor_node/points_raw]
    sub: [filter_component/points_raw]
    rate_hz: 100

  /perception/points_filtered:
    type: std_msgs/msg/String
    pub: [filter_component/points_filtered]
    sub: [control_node/points_filtered]
    rate_hz: 100

  /control/cmd:
    type: std_msgs/msg/String
    pub: [control_node/cmd]
    rate_hz: 100
",
    )
    .expect("failed to write tightened-budget overlay contract");

    let output = fixtures::play_launch_cmd(&env)
        .args(["check", "--contracts"])
        .arg(overlay_root.path())
        .args(["rt_demo", "bringup.launch.xml"])
        .output()
        .expect("failed to run play_launch check");

    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(
        output.status.success(),
        "tightened chain budget must warn, not fail, check:\n{combined}"
    );
    assert!(
        combined.contains("chain-sampling-feasibility"),
        "expected the chain-sampling-feasibility warning:\n{combined}"
    );
    assert!(
        combined.contains("points_to_cmd"),
        "expected the warning to name the infeasible chain:\n{combined}"
    );
    assert!(
        combined.contains("sampling_cost"),
        "expected the sampling_cost breakdown in the warning:\n{combined}"
    );
}

/// Per design doc §2.2/§3.1, a non-`posix` platform file resolves through
/// the identical provider-sidecar channel under `--target zephyr` and
/// **parses** (valid v2 schema) — proven here by asserting the failure is a
/// mapper-derive error (`requires \`resources.rt_priority_band\``), not a
/// YAML/schema parse error. None of play_launch's built-in mappers derive a
/// plan for non-`posix` facts today (RTOS derivation is nano-ros's job), so
/// `check --target zephyr` is expected to error at that later stage — this
/// test locks in that the failure mode is exactly this, not a schema
/// regression, and that no apply/syscall is ever attempted (`check` never
/// applies, for any target).
#[test]
fn check_target_zephyr_platform_file_parses_but_derive_is_posix_only() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let output = fixtures::play_launch_cmd(&env)
        .args([
            "check",
            "--target",
            "zephyr",
            "rt_demo",
            "bringup.launch.xml",
        ])
        .output()
        .expect("failed to run play_launch check --target zephyr");

    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(
        !output.status.success(),
        "expected check --target zephyr to fail at the derive step (no \
         posix facts for the deadline_monotonic mapper):\n{combined}"
    );
    assert!(
        combined.contains("Scheduling platform file [provider]:")
            && combined.contains("bringup.system.zephyr.yaml"),
        "expected the zephyr file to resolve via the provider channel \
         (proving it was found and parsed) before the derive error:\n{combined}"
    );
    assert!(
        combined.contains("requires `resources.rt_priority_band`"),
        "expected the mapper-derive error (not a parse/schema error), \
         proving the zephyr file parsed successfully:\n{combined}"
    );
    assert!(
        !combined.to_lowercase().contains("invalid platform file")
            && !combined.to_lowercase().contains("parse error")
            && !combined.to_lowercase().contains("deserializ"),
        "expected no parse/schema-level error text:\n{combined}"
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

// ---- `resolve` (SystemModel emission, RFC-0050) ----

/// `play_launch resolve rt_demo bringup.launch.xml --sched system.toml`
/// must emit a SystemModel whose three layers join on launch-side node
/// FQNs: the contract endpoint keys and the sched bindings must reference
/// nodes that exist in `structure.nodes`.
#[test]
fn resolve_emits_joined_system_model() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let sched = fixture_dir().join("system.toml");
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let out = tmp.path().join("system_model.yaml");

    let mut proc = ManagedProcess::spawn(fixtures::play_launch_cmd(&env).args([
        "resolve",
        "rt_demo",
        "bringup.launch.xml",
        "--sched",
        sched.to_str().unwrap(),
        "-o",
        out.to_str().unwrap(),
    ]))
    .expect("failed to spawn play_launch resolve");
    let status = proc.wait_with_timeout(Duration::from_secs(60));
    assert!(status.success(), "play_launch resolve failed");

    let yaml = std::fs::read_to_string(&out).expect("read system_model.yaml");
    let model: serde_json::Value = serde_yaml_value(&yaml);

    // meta: schema version + provenance hashes present
    assert_eq!(model["meta"]["version"], 1);
    assert!(
        model["meta"]["inputs"]
            .as_array()
            .map(Vec::len)
            .unwrap_or(0)
            >= 3,
        "launch + contract + platform file must be hashed"
    );

    // structure: the four rt_demo nodes
    let nodes = model["structure"]["nodes"]
        .as_object()
        .expect("structure.nodes");
    assert!(nodes.contains_key("/control/control_node"));
    assert!(nodes.contains_key("/perception/filter_component"));
    assert_eq!(nodes["/control/control_node"]["criticality"], "high");

    // contracts join on structure FQNs (manifest scope-ns refs reconciled)
    let subs = model["contracts"]["sub_endpoints"]
        .as_object()
        .expect("contracts.sub_endpoints");
    assert!(
        // Phase 44.5: control_node now subscribes points_filtered (the
        // chain's second hop), not points_raw directly — see
        // bringup.contract.yaml's `points_to_cmd` chain.
        subs.contains_key("/control/control_node/points_filtered"),
        "sub contract key must use the launch-side node FQN, got: {:?}",
        subs.keys().collect::<Vec<_>>()
    );

    // execution: every binding target exists in structure.nodes
    let bindings = model["execution"]["bindings"]
        .as_object()
        .expect("execution.bindings");
    assert!(!bindings.is_empty());
    for node in bindings.keys() {
        assert!(
            nodes.contains_key(node),
            "sched binding references unknown node {node}"
        );
    }
}

/// Minimal YAML → JSON value bridge for assertions (tests avoid a direct
/// serde_yaml dependency elsewhere; keep it local).
fn serde_yaml_value(yaml: &str) -> serde_json::Value {
    serde_yaml_ng::from_str(yaml).expect("parse system_model.yaml")
}

/// Phase 45.10 — `resolve` with the `chain_aware` platform file
/// (`bringup.system.posix.yaml`, the `points_to_cmd` chain showcase) does NOT
/// embed the resolved sched plan (`execution.sched` was removed — the model is
/// INPUT only). The mapper's OUTPUT is still applied via `execution.tiers` +
/// `execution.bindings` (the Linux realization), and the full chain-aware
/// provenance is available on demand from a fresh derive — `check --sched
/// --explain` must show every `points_to_cmd` chain member with `chain_aware`
/// (or `override`) provenance, since it re-derives via the same
/// `derive_sched_plan`.
#[test]
fn resolve_omits_sched_but_check_explain_shows_chain_aware() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let sched = fixture_dir().join("launch/bringup.system.posix.yaml");
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let out = tmp.path().join("system_model.yaml");

    let mut proc = ManagedProcess::spawn(fixtures::play_launch_cmd(&env).args([
        "resolve",
        "rt_demo",
        "bringup.launch.xml",
        "--sched",
        sched.to_str().unwrap(),
        "-o",
        out.to_str().unwrap(),
    ]))
    .expect("failed to spawn play_launch resolve");
    let status = proc.wait_with_timeout(Duration::from_secs(60));
    assert!(status.success(), "play_launch resolve failed");

    let yaml = std::fs::read_to_string(&out).expect("read system_model.yaml");
    let model: serde_json::Value = serde_yaml_value(&yaml);

    // The resolved sched plan is NOT embedded (Phase 45.10).
    assert!(
        model["execution"]["sched"].is_null(),
        "execution.sched must not be embedded (model is input only): {model}"
    );
    // But the applied Linux realization (tiers + bindings) IS carried — that's
    // what the RT apply layer reads.
    assert!(
        model["execution"]["tiers"].is_object() && model["execution"]["bindings"].is_object(),
        "tiers + bindings carry the applied schedule: {model}"
    );
    let bindings = model["execution"]["bindings"]
        .as_object()
        .expect("execution.bindings");
    // Every rt_demo node is bound to a tier (the mapper's projected outcome).
    assert!(!bindings.is_empty(), "bindings must be populated: {model}");

    // The known points_to_cmd chain members (from the fixture's chains: decl).
    let chain_member_fqns = [
        "/perception/sensor_node",
        "/perception/filter_component",
        "/control/control_node",
    ];

    // `check --sched --explain` re-derives via the same `derive_sched_plan`,
    // so every chain member must show `chain_aware` (or an explicit
    // `override`, e.g. control_node's pinned priority) provenance — the
    // chain-aware plan is fully available on demand even though it is no
    // longer embedded in the model.
    let explain_output = fixtures::play_launch_cmd(&env)
        .args([
            "check",
            "rt_demo",
            "bringup.launch.xml",
            "--sched",
            sched.to_str().unwrap(),
            "--explain",
        ])
        .output()
        .expect("failed to run check --sched --explain");
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&explain_output.stdout),
        String::from_utf8_lossy(&explain_output.stderr),
    );
    assert!(
        explain_output.status.success(),
        "check --sched --explain failed:\n{combined}"
    );
    for fqn in &chain_member_fqns {
        // `combined` has more than one line mentioning `fqn` (the plain
        // tier-table dump prints it too, with no provenance) — scan every
        // matching line for the `--explain` FQN-table row.
        let matched = combined
            .lines()
            .filter(|l| l.contains(fqn))
            .any(|l| l.contains("chain_aware") || l.contains("override"));
        assert!(
            matched,
            "chain member {fqn} must be chain_aware- or override-provenanced in --explain:\n{combined}"
        );
    }
}

/// Backward-compat (45.4): when NO platform file resolves, `resolve` must NOT
/// emit `execution.sched` — it stays absent, so old consumers and round-trips
/// are unaffected. The rt_workspace fixture ships a provider platform sidecar
/// (so a plain `resolve` DOES auto-resolve it — that's the auto-discovery,
/// tested elsewhere); `--no-provider-contracts` suppresses that sidecar,
/// exercising the genuine no-platform-file path end-to-end (complements the
/// model-crate unit test).
#[test]
fn resolve_without_platform_file_omits_execution_sched() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let out = tmp.path().join("system_model.yaml");

    let mut proc = ManagedProcess::spawn(fixtures::play_launch_cmd(&env).args([
        "resolve",
        "--no-provider-contracts",
        "rt_demo",
        "bringup.launch.xml",
        "-o",
        out.to_str().unwrap(),
    ]))
    .expect("failed to spawn play_launch resolve");
    assert!(
        proc.wait_with_timeout(Duration::from_secs(60)).success(),
        "play_launch resolve (--no-provider-contracts) failed"
    );

    let yaml = std::fs::read_to_string(&out).expect("read system_model.yaml");
    // Match the `sched:` map key at execution-nesting depth, not the
    // `sched_class:` inside a tier platform spec.
    assert!(
        !yaml
            .lines()
            .any(|l| l.trim_end() == "sched:" || l.trim() == "sched:"),
        "resolve with no platform file must not emit execution.sched:\n{yaml}"
    );
}

/// Phase 46.4/47.B3: the Phase 43.1 model↔record binding gate is removed,
/// and the legacy `--input-file record.json` compat path is hard-cut —
/// `replay --model` spawns straight from the model's `structure.nodes`, with
/// no record companion required OR possible. Resolves `bringup.launch.xml`
/// into a model in one scratch dir, then replays it from a *different*,
/// empty working directory (no `record.json` anywhere near it) and asserts
/// the same 3-process outcome (2 standalone nodes + 1 container; composables
/// don't get a separate `node/<name>/cmdline`) that `sched_apply_warn_smoke_launch`
/// proves for the in-memory `launch` path.
#[test]
fn replay_model_spawns_without_record_companion() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let resolve_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let model_path = resolve_tmp.path().join("system_model.yaml");

    let mut proc = ManagedProcess::spawn(fixtures::play_launch_cmd(&env).args([
        "resolve",
        "rt_demo",
        "bringup.launch.xml",
        "-o",
        model_path.to_str().unwrap(),
    ]))
    .expect("spawn resolve");
    assert!(
        proc.wait_with_timeout(Duration::from_secs(60)).success(),
        "play_launch resolve failed"
    );

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let stdout_path = work_tmp.path().join("stdout.log");
    let stderr_path = work_tmp.path().join("stderr.log");
    let stdout_file = fs::File::create(&stdout_path).expect("failed to create stdout file");
    let stderr_file = fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_tmp.path());
    cmd.args([
        "replay",
        "--model",
        model_path.to_str().unwrap(),
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
    ]);
    cmd.stdout(Stdio::from(stdout_file));
    cmd.stderr(Stdio::from(stderr_file));

    // No record.json exists anywhere under `work_tmp` — proves the model
    // alone drives the spawn.
    assert!(!work_tmp.path().join("record.json").exists());

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch replay");

    let play_log = work_tmp.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 3, Duration::from_secs(30));

    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(
        actual, 3,
        "process count mismatch: actual={actual}, expected=3 (2 nodes + 1 \
         container) — replay --model must spawn cleanly with no record \
         companion present"
    );

    let stderr = fs::read_to_string(&stderr_path).unwrap_or_default();
    assert!(
        !stderr.contains("does not match the record bound")
            && !stderr.contains("carries no bound record"),
        "the record-binding gate must be gone from the --model path:\n{stderr}"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group.
}

/// Phase 46.4 acceptance: `resolve --parser python` produces a spawnable
/// SystemModel — the Python parser must keep working as a first-class model
/// producer, not just a record.json producer, ahead of record.json's
/// eventual retirement (46.5). Resolves `bringup.launch.xml` with
/// `--parser python`, asserts the model carries the same 4 nodes
/// (2 standalone + 1 container + 1 composable) the Rust path produces, then
/// replays it — with no record companion — and asserts the same 3-process
/// spawn outcome as the Rust-path test above.
///
/// Asserts only parser-inherent, install-independent facts (node count +
/// spawnability): the contract/sched layers apply on the shared scope table
/// regardless of parser (Phase 40.1) and would be full parity with Rust
/// here (rt_workspace ships both a contract sidecar and a platform file) —
/// but only when the embedded Python imports a current play_launch package
/// (the one that emits `ScopeOrigin.path`). A stale pip-installed package
/// (pre-40.1) silently disables the provider-sidecar channels, so a
/// contract/sched assertion would be install-flaky; that full-parity check
/// is demonstrated by the manual acceptance run in the W4 report instead.
#[test]
fn resolve_parser_python_produces_model_that_replays_cleanly() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let resolve_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let model_path = resolve_tmp.path().join("system_model.yaml");

    let mut proc = ManagedProcess::spawn(fixtures::play_launch_cmd(&env).args([
        "resolve",
        "--parser",
        "python",
        "rt_demo",
        "bringup.launch.xml",
        "-o",
        model_path.to_str().unwrap(),
    ]))
    .expect("spawn resolve --parser python");
    assert!(
        proc.wait_with_timeout(Duration::from_secs(120)).success(),
        "play_launch resolve --parser python failed"
    );

    let model: serde_json::Value =
        serde_yaml_value(&std::fs::read_to_string(&model_path).expect("read system_model.yaml"));
    let node_count = model["structure"]["nodes"]
        .as_object()
        .map(|m| m.len())
        .unwrap_or(0);
    assert_eq!(
        node_count, 4,
        "expected 4 nodes from the Python parser (matching the Rust-path \
         model): {model}"
    );

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_tmp.path());
    cmd.args([
        "replay",
        "--model",
        model_path.to_str().unwrap(),
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
    ]);
    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch replay");

    let play_log = work_tmp.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 3, Duration::from_secs(30));

    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(
        actual, 3,
        "process count mismatch: actual={actual}, expected=3 (2 nodes + 1 \
         container) — a Python-parser-produced model must spawn cleanly"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group.
}

// ---- `check --export-graph` (Phase 42.1) ----

/// `play_launch check --export-graph <path>.json rt_demo bringup.launch.xml`
/// (provider-sidecar contract, no `--sched`) must exit 0 and write a JSON
/// export with the 3 nodes / 3 topics / 3 node-level paths declared in
/// `bringup.contract.yaml` (Phase 44.5: sensor_node's `tick` + filter_component's
/// `filter` + control_node's `control` — no scope-level path, no cycles — a
/// straight sensor -> filter -> control pipeline).
#[test]
fn check_export_graph_json_matches_contract() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let export_path = tmp.path().join("graph.json");

    let output = fixtures::play_launch_cmd(&env)
        .args([
            "check",
            "rt_demo",
            "bringup.launch.xml",
            "--export-graph",
            export_path.to_str().unwrap(),
        ])
        .output()
        .expect("failed to run play_launch check --export-graph");

    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(
        output.status.success(),
        "check --export-graph exited nonzero:\n{combined}"
    );
    assert!(
        combined.contains("Exported causal graph to"),
        "expected export confirmation line in check output:\n{combined}"
    );

    let data = fs::read_to_string(&export_path).expect("failed to read export JSON");
    let graph: serde_json::Value = serde_json::from_str(&data).expect("export JSON should parse");

    assert_eq!(graph["version"], 1);
    assert_eq!(
        array_len(&graph, "nodes"),
        3,
        "expected 3 nodes (sensor_node, filter_component, control_node): {graph}"
    );
    assert_eq!(
        array_len(&graph, "topics"),
        3,
        "expected 3 topics (points_raw, points_filtered, cmd): {graph}"
    );
    assert_eq!(
        array_len(&graph, "node_paths"),
        3,
        "expected 3 node-level paths (Phase 44.5: sensor_node.tick, \
         filter_component.filter, control_node.control): {graph}"
    );
    assert_eq!(
        array_len(&graph, "scope_paths"),
        0,
        "bringup.contract.yaml declares no scope-level paths: {graph}"
    );
    assert_eq!(
        array_len(&graph, "cycles"),
        0,
        "sensor -> filter/control is a straight pipeline, no cycles: {graph}"
    );

    // Phase 44.6: node FQNs in the export now reconcile against the real
    // launch-dump identity (`control_node` has its own `namespace="/control"`
    // attribute, which wins over the contract's root scope) instead of the
    // old naive `scope_ns + bare_name` ("/control_node") — see
    // `ManifestIndex::node_identity`.
    let control_node = graph["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["fqn"] == "/control/control_node")
        .expect("control_node present in export");
    assert_eq!(control_node["criticality"], "high");
}

/// Extension dispatch: `.dot` writes a Graphviz digraph, not JSON.
#[test]
fn check_export_graph_dot_extension_dispatch() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let export_path = tmp.path().join("graph.dot");

    let output = fixtures::play_launch_cmd(&env)
        .args([
            "check",
            "rt_demo",
            "bringup.launch.xml",
            "--export-graph",
            export_path.to_str().unwrap(),
        ])
        .output()
        .expect("failed to run play_launch check --export-graph (.dot)");

    assert!(
        output.status.success(),
        "check --export-graph (.dot) exited nonzero:\n{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );

    let dot = fs::read_to_string(&export_path).expect("failed to read export DOT");
    assert!(dot.starts_with("digraph causal {"));
    assert!(dot.contains("/control_node"));
}

// ---- Phase 46.5 — retirement: dump emits the model, no record companion,
// deprecated record.json replay ----

/// `dump <launch> -o <path>` with no `--format` (the new default) must
/// produce the SystemModel — the exact same artifact `resolve` produces —
/// not a record.json, and must NOT leave a `<path>.record.json` companion
/// on disk. Then `replay --model <path>` must spawn cleanly from it (same
/// 3-process outcome `replay_model_spawns_without_record_companion`
/// proves for `resolve`), confirming `dump` and `resolve` fully converged.
#[test]
fn dump_default_emits_model_and_replays_cleanly() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let dump_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let model_path = dump_tmp.path().join("m.yaml");

    let mut proc = ManagedProcess::spawn(fixtures::play_launch_cmd(&env).args([
        "dump",
        "-o",
        model_path.to_str().unwrap(),
        "launch",
        "rt_demo",
        "bringup.launch.xml",
    ]))
    .expect("spawn dump");
    assert!(
        proc.wait_with_timeout(Duration::from_secs(60)).success(),
        "play_launch dump (default format) failed"
    );

    // No record.json companion anywhere near the model.
    let companion = dump_tmp.path().join("m.record.json");
    assert!(
        !companion.exists(),
        "dump must not write a record.json companion next to the model (Phase 46.5): {}",
        companion.display()
    );

    // The output IS the SystemModel (structure.nodes present, no top-level
    // record.json `node` array — the two artifacts have disjoint shapes).
    let yaml = fs::read_to_string(&model_path).expect("failed to read dump output");
    let model = serde_yaml_value(&yaml);
    assert!(
        model.get("structure").is_some(),
        "dump's default output must be the SystemModel (structure.* present):\n{yaml}"
    );
    assert!(
        model.get("node").is_none(),
        "dump's default output must NOT be the legacy record.json shape (top-level `node`):\n{yaml}"
    );
    let nodes = model["structure"]["nodes"]
        .as_object()
        .expect("structure.nodes");
    assert_eq!(
        nodes.len(),
        4,
        "rt_demo bringup: 2 nodes + 1 container + 1 composable"
    );

    // And it replays cleanly, from a separate empty dir, with no record.json
    // anywhere — same acceptance `replay_model_spawns_without_record_companion`
    // runs for `resolve`.
    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_tmp.path());
    cmd.args([
        "replay",
        "--model",
        model_path.to_str().unwrap(),
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
    ]);
    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch replay");

    let play_log = work_tmp.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 3, Duration::from_secs(30));
    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(
        actual, 3,
        "process count mismatch: actual={actual}, expected=3 (2 nodes + 1 container) — \
         replay --model must spawn cleanly from a `dump`-produced model"
    );

    // _proc dropped here — ManagedProcess::drop kills the process group.
}

// ---- Phase 46.5 — retirement: resolve drops the record companion ----

/// `resolve -o <path>` must not write a `<path>.record.json` companion next
/// to the model (Phase 46.5 drops the Phase 43.1 companion write).
#[test]
fn resolve_no_longer_writes_record_companion() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let model_path = tmp.path().join("system_model.yaml");

    let mut proc = ManagedProcess::spawn(fixtures::play_launch_cmd(&env).args([
        "resolve",
        "rt_demo",
        "bringup.launch.xml",
        "-o",
        model_path.to_str().unwrap(),
    ]))
    .expect("spawn resolve");
    assert!(
        proc.wait_with_timeout(Duration::from_secs(60)).success(),
        "play_launch resolve failed"
    );

    let companion = model_path.with_extension("record.json");
    assert!(
        !companion.exists(),
        "resolve must not write a record.json companion next to the model (Phase 46.5): {}",
        companion.display()
    );
}

/// Run `play_launch <args>` capturing stdout+stderr, returning
/// `(ExitStatus, combined_output)`. For the CLI-rejection tests below —
/// they error before spawning any node, so no process-group cleanup dance
/// is needed beyond `ManagedProcess`'s own Drop.
fn run_capture(
    env: &std::collections::HashMap<String, String>,
    args: &[&str],
) -> (std::process::ExitStatus, String) {
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let out_path = tmp.path().join("stdout.log");
    let err_path = tmp.path().join("stderr.log");
    let out_file = fs::File::create(&out_path).expect("create stdout");
    let err_file = fs::File::create(&err_path).expect("create stderr");

    let mut cmd = fixtures::play_launch_cmd(env);
    cmd.args(args);
    cmd.stdout(Stdio::from(out_file));
    cmd.stderr(Stdio::from(err_file));
    let mut proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");
    let status = proc.wait_with_timeout(Duration::from_secs(15));

    let combined = format!(
        "{}\n{}",
        fs::read_to_string(&out_path).unwrap_or_default(),
        fs::read_to_string(&err_path).unwrap_or_default()
    );
    (status, combined)
}

/// `replay --input-file <path>` and a `record.json`-shaped positional are a
/// HARD CUT (Phase 47.B3): the record.json replay surface is removed
/// entirely. Both must fail with a CLEAN error (a deliberate non-zero exit
/// code, NOT a signal-kill or a Rust panic/exit-101) carrying the helpful
/// migration message pointing at `resolve`/`replay --model` — not clap's
/// bare "unexpected argument" and not an opaque YAML-parse failure.
#[test]
fn replay_input_file_flag_is_removed_and_errors_clearly() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    for args in [
        &["replay", "--input-file", "record.json"][..],
        &["replay", "old_dump.record.json"][..],
    ] {
        let (status, output) = run_capture(&env, args);

        // Clean error: exited deliberately (has an exit code — not killed by
        // a signal), non-zero, and not a Rust panic (exit 101).
        let code = status.code();
        assert!(
            code.is_some(),
            "replay {args:?} must exit with a code (clean error), not be killed by a signal:\n{output}"
        );
        assert!(
            !status.success() && code != Some(101),
            "replay {args:?} must fail cleanly (deliberate non-zero exit, not a panic/exit-101): code={code:?}\n{output}"
        );
        // Helpful migration message, not a bare clap/YAML error.
        let lower = output.to_lowercase();
        assert!(
            lower.contains("record.json replay was removed") && lower.contains("replay --model"),
            "replay {args:?} must print the Phase 47 record.json migration message pointing at `replay --model`:\n{output}"
        );
    }
}

/// `replay` with no model at all (neither positional nor `--model`) must
/// fail with a CLEAN error (deliberate non-zero exit, not a crash) carrying
/// the "requires a SystemModel" message — not panic, not silently no-op.
#[test]
fn replay_without_model_errors_clearly() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();

    let (status, output) = run_capture(&env, &["replay", "--disable-web-ui"]);

    let code = status.code();
    assert!(
        code.is_some(),
        "replay with no model must exit with a code (clean error), not be killed by a signal:\n{output}"
    );
    assert!(
        !status.success() && code != Some(101),
        "replay with no model must fail cleanly (deliberate non-zero exit, not a panic): code={code:?}\n{output}"
    );
    assert!(
        output.to_lowercase().contains("requires a systemmodel"),
        "replay with no model must print the 'requires a SystemModel' guidance:\n{output}"
    );
}
