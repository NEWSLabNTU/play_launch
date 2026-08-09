//! Legacy `.toml` bridge: the current (v1) `system.toml` schema keeps
//! parsing and maps into the v2 [`PlatformFile`] shape with
//! `mapper: "manual"`, `target: "posix"`, and the legacy tiers+assign spec
//! carried in [`PlatformFile::legacy`] so the `manual` [`SchedMapper`]
//! ([`crate::mapper::ManualMapper`]) can reproduce today's [`crate::resolve`]
//! output exactly.
//!
//! Extension selects the parser ([`crate::platform::parse_platform_file`]);
//! this module only implements the `.toml` half of that dispatch.

use std::collections::BTreeMap;

use crate::{
    parse::parse_system_sched,
    platform::{PlatformFile, PlatformResources, PosixResources},
    resolve::SchedError,
};

/// Parse a legacy `system.toml` document into the v2 [`PlatformFile`] shape.
///
/// - `target` is always `"posix"` (the legacy schema predates multi-target
///   platform files; `nano-ros`'s RTOS targets use per-platform sub-tables
///   inside the same tier, resolved directly via [`crate::resolve::resolve`]
///   rather than through this bridge).
/// - `mapper` is always `"manual"`.
/// - `resources` is empty (`PosixResources::default()`) — the legacy schema
///   has no `rt_priority_band`/`isolated_cpus` concept; those are new in v2.
/// - `overrides` is always empty — legacy `[[assign]]` bindings are carried
///   whole in `legacy`, not translated into per-node overrides.
pub fn parse_legacy_toml(input: &str) -> Result<PlatformFile, SchedError> {
    let sched = parse_system_sched(input)?;
    Ok(PlatformFile {
        target: "posix".to_string(),
        mapper: "manual".to_string(),
        resources: PlatformResources::Posix(PosixResources::default()),
        overrides: BTreeMap::new(),
        legacy: Some(sched),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        mapper::{MapperInput, MapperNode, MapperRegistry, PlatformFacts},
        resolve::{SchedNode, resolve},
    };

    const FIXTURE_TOML: &str = r#"
[tiers.control]
class = "real_time"
deadline_us = 50000

[tiers.control.posix]
priority = 80
sched_class = "SCHED_FIFO"
core = 1

[tiers.perception]
class = "real_time"

[tiers.perception.posix]
priority = 60

[[assign]]
tier = "control"
nodes = ["/control/ndt_localizer"]

[[assign]]
tier = "perception"
scope = "/perception/lidar"
"#;

    fn fixture_nodes() -> Vec<SchedNode> {
        vec![
            SchedNode {
                name: "/control/ndt_localizer".to_string(),
                scope: "/control".to_string(),
            },
            SchedNode {
                name: "/perception/lidar/a".to_string(),
                scope: "/perception/lidar".to_string(),
            },
            SchedNode {
                name: "/other/b".to_string(),
                scope: "/other".to_string(),
            },
        ]
    }

    #[test]
    fn bridge_produces_manual_platform_file() {
        let file = parse_legacy_toml(FIXTURE_TOML).expect("bridge parses");
        assert_eq!(file.target, "posix");
        assert_eq!(file.mapper, "manual");
        assert!(file.overrides.is_empty());
        let legacy = file.legacy.expect("legacy spec carried");
        assert_eq!(legacy.tiers.len(), 2);
        assert_eq!(legacy.assign.len(), 2);
    }

    /// Equivalence test (required by the brief): fixture TOML → bridge →
    /// `manual` mapper output must equal direct `resolve()` output.
    #[test]
    fn bridge_then_manual_mapper_matches_direct_resolve() {
        let nodes = fixture_nodes();

        let sched = parse_system_sched(FIXTURE_TOML).unwrap();
        let direct = resolve(&sched.tiers, &sched.assign, &nodes, "posix").expect("direct resolve");

        let file = parse_legacy_toml(FIXTURE_TOML).expect("bridge parses");
        let registry = MapperRegistry::with_builtins();
        let manual = registry.get("manual").expect("manual mapper registered");

        let input = MapperInput {
            nodes: nodes
                .iter()
                .map(|n| MapperNode {
                    name: n.name.clone(),
                    scope: n.scope.clone(),
                    rate_hz: None,
                    deadline_us: None,
                    criticality: None,
                    path_budget_ms: None,
                    paths: Vec::new(),
                })
                .collect(),
            legacy: file.legacy.clone(),
            chains: Vec::new(),
        };
        let facts = match &file.resources {
            PlatformResources::Posix(p) => PlatformFacts::Posix(p.clone()),
            PlatformResources::Raw(v) => PlatformFacts::Raw(v.clone()),
        };

        let via_bridge = manual.map(&input, &facts).expect("manual mapper maps");

        assert_eq!(via_bridge, direct);
    }
}
