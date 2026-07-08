//! Resolve the shared scheduling spec against a launch dump and invert the
//! result into a per-FQN lookup the actor system consumes.
//!
//! `SchedPlan::build` is the bridge between the crate's tier-centric
//! `ResolvedTierTable` (tier -> members) and the actor system's node-centric
//! view (FQN -> knobs). It also surfaces two kinds of "not applied in v1"
//! warnings: `time_triggered` tiers (no `SCHED_DEADLINE` support yet) and
//! composable (load_node) members (no per-process scheduling target yet).
//!
//! Public items here are not yet called outside this module (actor wiring
//! is a later phase-38 task), hence the blanket `dead_code` allow — mirrors
//! `sched_apply`.
#![allow(dead_code)]

use std::collections::HashMap;
use std::path::Path;

use eyre::{Result, WrapErr};
use ros_launch_manifest_sched::{DEFAULT_TIER, parse_system_sched, resolve};

use crate::execution::sched_apply::{AppliedTier, SchedApplyMode, SchedPolicy};
use crate::ros::launch_dump::LaunchDump;
use crate::ros::sched_loader::{composable_fqns, sched_nodes_from_dump};

/// The `posix` target key — Linux RT placement sub-table.
const TARGET: &str = "posix";

/// Per-FQN scheduling plan, resolved once at startup and consulted by actors
/// as processes come up.
#[derive(Debug)]
pub struct SchedPlan {
    by_fqn: HashMap<String, AppliedTier>,
    pub mode: SchedApplyMode,
    /// User-facing warnings emitted at build (also logged). Kept for testability.
    pub warnings: Vec<String>,
}

impl SchedPlan {
    /// Read + parse the scheduling TOML at `sched_path`, resolve it for
    /// `posix` against `dump`'s nodes, and invert tier membership into a
    /// per-FQN lookup.
    pub fn build(dump: &LaunchDump, sched_path: &Path, mode: SchedApplyMode) -> Result<SchedPlan> {
        let text = std::fs::read_to_string(sched_path)
            .wrap_err_with(|| format!("failed to read {}", sched_path.display()))?;
        let sched =
            parse_system_sched(&text).map_err(|e| eyre::eyre!("scheduling spec error: {e}"))?;

        let nodes = sched_nodes_from_dump(dump);
        let table = resolve(&sched.tiers, &sched.assign, &nodes, TARGET)
            .map_err(|e| eyre::eyre!("scheduling resolve error: {e}"))?;

        let composables = composable_fqns(dump);

        // Number of online CPUs, computed once. Used to bound-check `core`
        // below. Falls back to 1 if unavailable, which conservatively
        // rejects any non-zero core on a system that can't report its CPU
        // count.
        let ncpu = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);

        let mut by_fqn = HashMap::new();
        let mut warnings = Vec::new();

        for tier in &table.tiers {
            // The default tier's members have no meaningful scheduling assignment.
            if tier.name == DEFAULT_TIER {
                continue;
            }

            if tier.class.as_deref() == Some("time_triggered") {
                let msg = format!(
                    "tier `{}`: SCHED_DEADLINE not applied in v1 (time_triggered)",
                    tier.name
                );
                tracing::warn!("{msg}");
                warnings.push(msg);
            }

            let policy = SchedPolicy::from_sched_class(tier.sched_class.as_deref());
            let priority = tier.priority as i32;

            // Deterministic config validation, done here (build time) rather
            // than at apply time (actor hook, after spawn): a malformed spec
            // is a spec bug, not a runtime/permission condition, so it must
            // hard-error BEFORE any process is spawned, uniformly in ALL
            // apply modes (including `Off`/`Warn` — this is not a
            // strict-only check).
            if matches!(policy, SchedPolicy::Fifo | SchedPolicy::Rr) && !(1..=99).contains(&priority)
            {
                eyre::bail!(
                    "tier '{}': RT priority {priority} out of range 1..=99",
                    tier.name
                );
            }

            if let Some(c) = tier.core
                && c as usize >= ncpu
            {
                eyre::bail!(
                    "tier '{}': core {c} >= available CPUs ({ncpu})",
                    tier.name
                );
            }

            let applied = AppliedTier {
                policy,
                priority,
                core: tier.core,
                tier_name: tier.name.clone(),
            };

            for fqn in &tier.members {
                if composables.contains(fqn) {
                    let msg = format!(
                        "composable `{fqn}`: Linux scheduling not applied in v1 (tracked for phase 38.9)"
                    );
                    tracing::warn!("{msg}");
                    warnings.push(msg);
                }
                by_fqn.insert(fqn.clone(), applied.clone());
            }
        }

        Ok(SchedPlan {
            by_fqn,
            mode,
            warnings,
        })
    }

    /// Look up the resolved scheduling knobs for a fully-qualified node name.
    pub fn for_fqn(&self, fqn: &str) -> Option<&AppliedTier> {
        self.by_fqn.get(fqn)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dump_with_node_and_composable() -> LaunchDump {
        let json = serde_json::json!({
            "node": [{
                "executable": "ndt_localizer",
                "name": "ndt_localizer",
                "exec_name": "ndt_localizer",
                "namespace": "/control",
                "params_files": [],
                "cmd": [],
                "scope": 0
            }],
            "load_node": [{
                "package": "my_pkg",
                "plugin": "my_pkg::MyNode",
                "target_container_name": "my_container",
                "node_name": "my_node",
                "namespace": "/control",
                "scope": 0
            }],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null}
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    /// RAII guard that removes the wrapped path on drop, including on panic
    /// (e.g. an `assert!`/`expect` failure partway through a test body) —
    /// unlike a manual `remove_file` call after `f(&path)`, which is skipped
    /// entirely if `f` unwinds.
    struct TempTomlGuard(std::path::PathBuf);

    impl Drop for TempTomlGuard {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    /// Write `contents` to a unique temp file, run `f` with its path, then remove it
    /// (even if `f` panics).
    fn with_temp_toml<F: FnOnce(&Path)>(contents: &str, f: F) {
        let path = std::env::temp_dir().join(format!(
            "play_launch_sched_plan_test_{}_{}.toml",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::write(&path, contents).expect("write temp toml");
        let guard = TempTomlGuard(path);
        f(&guard.0);
    }

    #[test]
    fn resolves_regular_node_into_applied_tier() {
        let dump = dump_with_node_and_composable();
        let toml = r#"
[tiers.control]
class = "real_time"

[tiers.control.posix]
priority = 80
sched_class = "SCHED_FIFO"

[[assign]]
tier = "control"
nodes = ["/control/ndt_localizer", "/control/my_node"]
"#;
        with_temp_toml(toml, |path| {
            let plan = SchedPlan::build(&dump, path, SchedApplyMode::Warn).expect("build plan");

            let applied = plan
                .for_fqn("/control/ndt_localizer")
                .expect("regular node should be in plan");
            assert_eq!(applied.policy, SchedPolicy::Fifo);
            assert_eq!(applied.priority, 80);
            assert_eq!(applied.tier_name, "control");

            assert!(
                plan.warnings
                    .iter()
                    .any(|w| w.contains("/control/my_node")),
                "expected a warning mentioning the composable FQN, got: {:?}",
                plan.warnings
            );
        });
    }

    #[test]
    fn unassigned_node_in_default_tier_has_no_applied_entry() {
        let dump = dump_with_node_and_composable();
        // No [[assign]] rules at all: everything lands in the synthesized default tier.
        let toml = "";
        with_temp_toml(toml, |path| {
            let plan = SchedPlan::build(&dump, path, SchedApplyMode::Off).expect("build plan");
            assert!(plan.for_fqn("/control/ndt_localizer").is_none());
            assert!(plan.for_fqn("/control/my_node").is_none());
        });
    }

    #[test]
    fn rejects_rt_priority_zero_at_build_time() {
        let dump = dump_with_node_and_composable();
        let toml = r#"
[tiers.control]
class = "real_time"

[tiers.control.posix]
priority = 0
sched_class = "SCHED_FIFO"

[[assign]]
tier = "control"
nodes = ["/control/ndt_localizer"]
"#;
        with_temp_toml(toml, |path| {
            // Strict/Warn/Off all validate the same: a malformed spec is a
            // spec bug, not a runtime condition, so `build` errors uniformly
            // regardless of mode. Using `Warn` here is representative.
            let err = SchedPlan::build(&dump, path, SchedApplyMode::Warn)
                .expect_err("priority 0 should be rejected at build time");
            let msg = err.to_string();
            assert!(
                msg.contains("control") && msg.contains("priority") && msg.contains("0"),
                "expected a message naming the tier and the bad priority, got: {msg}"
            );
        });
    }

    #[test]
    fn rejects_rt_priority_above_range_at_build_time() {
        let dump = dump_with_node_and_composable();
        let toml = r#"
[tiers.control]
class = "real_time"

[tiers.control.posix]
priority = 100
sched_class = "SCHED_FIFO"

[[assign]]
tier = "control"
nodes = ["/control/ndt_localizer"]
"#;
        with_temp_toml(toml, |path| {
            let err = SchedPlan::build(&dump, path, SchedApplyMode::Strict)
                .expect_err("priority 100 should be rejected at build time");
            let msg = err.to_string();
            assert!(
                msg.contains("control") && msg.contains("100"),
                "expected a message naming the tier and the bad priority, got: {msg}"
            );
        });
    }

    #[test]
    fn rejects_core_beyond_available_cpus_at_build_time() {
        let dump = dump_with_node_and_composable();
        // 999 is far beyond any real machine's CPU count.
        let toml = r#"
[tiers.control]
class = "real_time"

[tiers.control.posix]
priority = 20
core = 999

[[assign]]
tier = "control"
nodes = ["/control/ndt_localizer"]
"#;
        with_temp_toml(toml, |path| {
            // Off mode also errors — validation is unconditional on mode,
            // since a bad core is a spec bug regardless of whether apply is
            // even attempted.
            let err = SchedPlan::build(&dump, path, SchedApplyMode::Off)
                .expect_err("core 999 should be rejected at build time even in Off mode");
            let msg = err.to_string();
            assert!(
                msg.contains("control") && msg.contains("999"),
                "expected a message naming the tier and the bad core, got: {msg}"
            );
        });
    }

    #[test]
    fn valid_tier_builds_successfully() {
        let dump = dump_with_node_and_composable();
        let toml = r#"
[tiers.control]
class = "real_time"

[tiers.control.posix]
priority = 20
sched_class = "SCHED_FIFO"
core = 0

[[assign]]
tier = "control"
nodes = ["/control/ndt_localizer"]
"#;
        with_temp_toml(toml, |path| {
            let plan = SchedPlan::build(&dump, path, SchedApplyMode::Strict)
                .expect("valid tier (prio 20, core 0) should build fine");
            let applied = plan
                .for_fqn("/control/ndt_localizer")
                .expect("node should be in plan");
            assert_eq!(applied.priority, 20);
            assert_eq!(applied.core, Some(0));
        });
    }

    #[test]
    fn time_triggered_tier_warns() {
        let dump = dump_with_node_and_composable();
        let toml = r#"
[tiers.periodic]
class = "time_triggered"
period_us = 10000

[tiers.periodic.posix]
priority = 50

[[assign]]
tier = "periodic"
nodes = ["/control/ndt_localizer"]
"#;
        with_temp_toml(toml, |path| {
            let plan = SchedPlan::build(&dump, path, SchedApplyMode::Warn).expect("build plan");
            assert!(
                plan.warnings
                    .iter()
                    .any(|w| w.contains("time_triggered") && w.contains("periodic")),
                "expected a time_triggered warning, got: {:?}",
                plan.warnings
            );
        });
    }
}
