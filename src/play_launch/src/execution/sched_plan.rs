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

            let applied = AppliedTier {
                policy: SchedPolicy::from_sched_class(tier.sched_class.as_deref()),
                priority: tier.priority as i32,
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

    /// Write `contents` to a unique temp file, run `f` with its path, then remove it.
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
        f(&path);
        let _ = std::fs::remove_file(&path);
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
