//! Resolve the shared scheduling spec against a launch dump and invert the
//! result into a per-FQN lookup the actor system consumes.
//!
//! `SchedPlan::build` is the bridge between the crate's tier-centric
//! `ResolvedTierTable` (tier -> members) and the actor system's node-centric
//! view (FQN -> knobs). It also surfaces a "not applied in v1" warning for
//! `time_triggered` tiers (no `SCHED_DEADLINE` support yet). Composable
//! (load_node) members ARE scheduled (phase 38.9) — they're resolved into
//! `by_fqn` like any other member, with no warning.
//!
//! Public items here are not yet called outside this module (actor wiring
//! is a later phase-38 task), hence the blanket `dead_code` allow — mirrors
//! `sched_apply`.
#![allow(dead_code)]

use std::{collections::HashMap, path::Path};

use eyre::Result;
use ros_launch_manifest_sched::DEFAULT_TIER;

use crate::{
    execution::sched_apply::{AppliedTier, SchedApplyMode, SchedPolicy},
    ros::{
        launch_dump::LaunchDump, manifest_loader::ManifestIndex, sched_loader::derive_sched_plan,
    },
};

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
    /// Parse the scheduling platform file at `sched_path` (v2 `.yaml` or
    /// legacy `.toml`, dispatched by extension), run the full
    /// derive→override→validate pipeline
    /// (`ros::sched_loader::derive_sched_plan`) for `target`, and invert the
    /// resulting tier membership into a per-FQN lookup.
    ///
    /// `index` is the resolved contract index (if any manifests were
    /// loaded) — passed straight through to the derive stage, which uses it
    /// to extract per-node timing facts (rate/deadline/criticality) for
    /// contract-aware mappers. `None`/legacy-TOML callers get bare
    /// `MapperNode`s, reproducing today's `manual`-only behavior exactly.
    pub fn build(
        dump: &LaunchDump,
        index: Option<&ManifestIndex>,
        sched_path: &Path,
        target: &str,
        mode: SchedApplyMode,
    ) -> Result<SchedPlan> {
        let derived = derive_sched_plan(dump, index, sched_path, target, mode)?;
        let table = derived.plan;
        let mut warnings = derived.warnings;

        // Number of online CPUs, computed once. Used to bound-check `core`
        // below. Falls back to 1 if unavailable, which conservatively
        // rejects any non-zero core on a system that can't report its CPU
        // count.
        let ncpu = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);

        let mut by_fqn = HashMap::new();

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

            // Deterministic config validation, done here (build time) rather
            // than at apply time (actor hook, after spawn): a malformed spec
            // is a spec bug, not a runtime/permission condition, so it must
            // hard-error BEFORE any process is spawned, uniformly in ALL
            // apply modes (including `Off`/`Warn` — this is not a
            // strict-only check). Validate the raw `i64` priority (which is
            // deliberately wide to admit RTOS negative coop priorities) BEFORE
            // narrowing to `i32`, so a pathological value can't slip through
            // via truncation.
            if matches!(policy, SchedPolicy::Fifo | SchedPolicy::Rr)
                && !(1..=99).contains(&tier.priority)
            {
                eyre::bail!(
                    "tier '{}': RT priority {} out of range 1..=99",
                    tier.name,
                    tier.priority
                );
            }

            // Safe: validated to 1..=99 for RT policies; Other ignores priority.
            let priority = tier.priority as i32;

            if let Some(c) = tier.core
                && c as usize >= ncpu
            {
                eyre::bail!("tier '{}': core {c} >= available CPUs ({ncpu})", tier.name);
            }

            let applied = AppliedTier {
                policy,
                priority,
                core: tier.core,
                tier_name: tier.name.clone(),
            };

            for fqn in &tier.members {
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
    fn resolves_regular_node_and_composable_into_applied_tier_with_no_warning() {
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
            let plan = SchedPlan::build(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect("build plan");

            let applied = plan
                .for_fqn("/control/ndt_localizer")
                .expect("regular node should be in plan");
            assert_eq!(applied.policy, SchedPolicy::Fifo);
            assert_eq!(applied.priority, 80);
            assert_eq!(applied.tier_name, "control");

            // Composables are now scheduled like any other member (phase
            // 38.9): present in the plan, no "not applied" warning.
            let composable_applied = plan
                .for_fqn("/control/my_node")
                .expect("composable should be in plan");
            assert_eq!(composable_applied.policy, SchedPolicy::Fifo);
            assert_eq!(composable_applied.priority, 80);

            assert!(
                plan.warnings.is_empty(),
                "expected no warnings, got: {:?}",
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
            let plan = SchedPlan::build(&dump, None, path, "posix", SchedApplyMode::Off)
                .expect("build plan");
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
            let err = SchedPlan::build(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect_err("priority 0 should be rejected at build time");
            let msg = err.to_string();
            assert!(
                msg.contains("control") && msg.contains("priority") && msg.contains("0"),
                "expected a message naming the tier and the bad priority, got: {msg}"
            );
        });
    }

    #[test]
    fn rejects_rt_priority_that_only_looks_valid_after_i32_truncation() {
        // 2^32 + 20 truncates to 20 (in range) as i32, but the raw i64 is
        // nonsensical and must be rejected — guards against validating the
        // narrowed value instead of the authored one.
        let dump = dump_with_node_and_composable();
        let toml = r#"
[tiers.control]
class = "real_time"

[tiers.control.posix]
priority = 4294967316
sched_class = "SCHED_FIFO"

[[assign]]
tier = "control"
nodes = ["/control/ndt_localizer"]
"#;
        with_temp_toml(toml, |path| {
            let _ = SchedPlan::build(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect_err("i64 priority 2^32+20 must be rejected, not truncated to 20");
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
            let err = SchedPlan::build(&dump, None, path, "posix", SchedApplyMode::Strict)
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
            let err = SchedPlan::build(&dump, None, path, "posix", SchedApplyMode::Off)
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
            let plan = SchedPlan::build(&dump, None, path, "posix", SchedApplyMode::Strict)
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
            let plan = SchedPlan::build(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect("build plan");
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
