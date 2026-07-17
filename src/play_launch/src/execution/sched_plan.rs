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

use std::{
    collections::{BTreeSet, HashMap},
    path::Path,
};

use eyre::Result;
use ros_launch_manifest_sched::DEFAULT_TIER;

use crate::{
    cli::options::ContainerMode,
    execution::sched_apply::{AppliedTier, SchedApplyMode, SchedPolicy},
    ros::{
        launch_dump::LaunchDump,
        manifest_loader::ManifestIndex,
        sched_loader::{derive_sched_plan, fqn_for},
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
    /// Every node FQN that participates in a resolved `chains:` declaration
    /// (Phase 44.4) — threaded through from
    /// [`crate::ros::sched_loader::DerivedSchedPlan::chain_member_nodes`],
    /// consumed by [`chain_container_colocation_warnings`]. Empty for
    /// [`SchedPlan::from_model`] (the Phase 43.3 `SystemModel` execution
    /// layer carries bindings, not chain structure — a known gap, not this
    /// wave's scope; see that method's doc comment).
    pub chain_member_nodes: BTreeSet<String>,
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
        let chain_member_nodes = derived.chain_member_nodes;

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
            chain_member_nodes,
        })
    }

    /// Look up the resolved scheduling knobs for a fully-qualified node name.
    pub fn for_fqn(&self, fqn: &str) -> Option<&AppliedTier> {
        self.by_fqn.get(fqn)
    }

    /// Phase 43.3 — build the per-FQN plan from a checked SystemModel's
    /// execution layer instead of re-deriving from the platform file:
    /// `bindings` (FQN → tier name) + `tiers[name].<target>` placement.
    /// Mapper derivation, overrides, and band validation already happened
    /// at resolve time; this only re-checks platform applicability and the
    /// deterministic spec invariants (RT priority range, core bounds) —
    /// those depend on THIS host, not on the resolver's.
    pub fn from_model(
        model: &ros_launch_manifest_model::SystemModel,
        target: &str,
        mode: SchedApplyMode,
    ) -> Result<SchedPlan> {
        let mut warnings = Vec::new();
        let ncpu = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);
        let mut by_fqn = HashMap::new();

        for (fqn, tier_name) in &model.execution.bindings {
            let Some(tier) = model.execution.tiers.get(tier_name) else {
                eyre::bail!(
                    "SystemModel: binding '{fqn}' references undeclared tier '{tier_name}'"
                );
            };
            let Some(spec) = tier.platform(target) else {
                eyre::bail!(
                    "SystemModel: tier '{tier_name}' has no `{target}` placement sub-table — \
                     the model was resolved for a different target; re-run \
                     `play_launch resolve --target {target}`"
                );
            };
            if tier.class.as_deref() == Some("time_triggered") {
                let msg = format!(
                    "tier `{tier_name}`: SCHED_DEADLINE not applied in v1 (time_triggered)"
                );
                tracing::warn!("{msg}");
                warnings.push(msg);
            }
            let policy = SchedPolicy::from_sched_class(spec.sched_class.as_deref());
            if matches!(policy, SchedPolicy::Fifo | SchedPolicy::Rr)
                && !(1..=99).contains(&spec.priority)
            {
                eyre::bail!(
                    "tier '{tier_name}': RT priority {} out of range 1..=99",
                    spec.priority
                );
            }
            if let Some(c) = spec.core
                && c as usize >= ncpu
            {
                eyre::bail!("tier '{tier_name}': core {c} >= available CPUs ({ncpu})");
            }
            by_fqn.insert(
                fqn.clone(),
                AppliedTier {
                    policy,
                    priority: spec.priority as i32,
                    core: spec.core,
                    tier_name: tier_name.clone(),
                },
            );
        }

        Ok(SchedPlan {
            by_fqn,
            mode,
            warnings,
            // Phase 43.3's `SystemModel` execution layer is bindings-only
            // (fqn -> tier name); it doesn't carry chain structure (that's
            // resolved once at `play_launch resolve` time and not re-emitted
            // into the model). The container co-location warning is simply
            // unavailable on the `--model` path until a future wave threads
            // chain membership through the model too — documented gap, not
            // a silent one (44.4 W4 report).
            chain_member_nodes: BTreeSet::new(),
        })
    }
}

/// Chain members co-located in a non-isolated container share one OS
/// process and cannot receive distinct scheduling priorities (chain-aware-
/// mapper design step 8, Phase 44.4 §4) — a best-effort warning naming the
/// container and its chain-member composable nodes.
///
/// **Placement decision**: computed here, at launch/replay time
/// (`commands::replay`, after `container_contexts`/`SchedPlan` are both
/// built), rather than at `check` time
/// (`ros::sched_loader::derive_sched_plan`/`check_sched`). `--container-mode`
/// is a *runtime replay flag* with no equivalent at `check` time (`check`
/// only parses+validates a launch file and its contracts — it never decides
/// how containers will actually be spawned), so the mode this warning keys
/// off simply isn't knowable at `check` time. `derive_sched_plan` only
/// exposes the data half of this warning (`chain_member_nodes`, mapper-
/// independent) — the runtime half (container topology + mode) lives here.
///
/// Fires only under `ContainerMode::Observable`/`Stock` — under `Isolated`
/// every composable node is fork+exec'd into its own process (Phase 19.10),
/// so no two nodes ever share a process and co-location is structurally
/// impossible. (The design doc's own wording, "stock (non-isolated)
/// container", predates the `Observable`/`Isolated`/`Stock` three-way split
/// this codebase later grew — `Observable` shares the design doc's failure
/// mode exactly as much as `Stock` does: neither forks per node.)
///
/// Container membership resolution mirrors `builder.rs`'s composable-node
/// target matching (documented in the top-level `CLAUDE.md`, "2026-03-11"
/// changelog entry): exact FQN match first, then a suffix match
/// (`container_fqn.ends_with("/{target_name}")`) for relative targets.
pub fn chain_container_colocation_warnings(
    dump: &LaunchDump,
    container_mode: ContainerMode,
    chain_member_nodes: &BTreeSet<String>,
) -> Vec<String> {
    if container_mode == ContainerMode::Isolated || chain_member_nodes.is_empty() {
        return Vec::new();
    }

    let containers: Vec<String> = dump
        .container
        .iter()
        .filter(|c| !c.name.is_empty())
        .map(|c| fqn_for(dump, Some(c.namespace.as_str()), &c.name, c.scope))
        .collect();

    let mut by_container: std::collections::BTreeMap<String, Vec<String>> =
        std::collections::BTreeMap::new();
    for lc in &dump.load_node {
        if lc.node_name.is_empty() {
            continue;
        }
        let member_fqn = fqn_for(dump, Some(lc.namespace.as_str()), &lc.node_name, lc.scope);
        if !chain_member_nodes.contains(&member_fqn) {
            continue;
        }
        let target = lc.target_container_name.trim();
        let suffix = format!("/{}", target.trim_start_matches('/'));
        let container_fqn = containers
            .iter()
            .find(|fqn| fqn.as_str() == target)
            .or_else(|| containers.iter().find(|fqn| fqn.ends_with(&suffix)))
            .cloned();
        if let Some(container_fqn) = container_fqn {
            by_container
                .entry(container_fqn)
                .or_default()
                .push(member_fqn);
        }
    }

    by_container
        .into_iter()
        .filter(|(_, members)| members.len() >= 2)
        .map(|(container_fqn, mut members)| {
            members.sort();
            format!(
                "scheduling: chain-member composable nodes {members:?} are co-located in \
                 non-isolated container '{container_fqn}' (--container-mode {container_mode:?}) \
                 — they share one process and cannot receive distinct scheduling priorities; \
                 use --container-mode isolated to schedule them independently"
            )
        })
        .collect()
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

#[cfg(test)]
mod model_tests {
    use super::*;

    fn model_with_binding(target_table: bool) -> ros_launch_manifest_model::SystemModel {
        let mut m = ros_launch_manifest_model::SystemModel::default();
        let mut tier = ros_launch_manifest_sched::TierDef {
            class: Some("real_time".to_string()),
            ..Default::default()
        };
        if target_table {
            tier.posix = Some(ros_launch_manifest_sched::TierPlatformSpec {
                priority: 40,
                stack_bytes: None,
                core: Some(0),
                sched_class: Some("SCHED_FIFO".to_string()),
                preempt_threshold: None,
                deadline_us: None,
            });
        }
        m.execution.tiers.insert("ctrl".to_string(), tier);
        m.execution
            .bindings
            .insert("/a/control_node".to_string(), "ctrl".to_string());
        m
    }

    #[test]
    fn from_model_binds_applied_tier() {
        let plan = SchedPlan::from_model(&model_with_binding(true), "posix", SchedApplyMode::Warn)
            .expect("plan");
        let t = plan.for_fqn("/a/control_node").expect("bound");
        assert_eq!(t.priority, 40);
        assert_eq!(t.core, Some(0));
        assert_eq!(t.tier_name, "ctrl");
        assert!(matches!(t.policy, SchedPolicy::Fifo));
    }

    #[test]
    fn from_model_missing_target_table_fails_loud() {
        let err = SchedPlan::from_model(&model_with_binding(false), "posix", SchedApplyMode::Warn)
            .unwrap_err();
        assert!(
            err.to_string().contains("no `posix` placement"),
            "got: {err}"
        );
    }
}
