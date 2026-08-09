//! R1-P1 — the integrator `system.toml` reader (canonical-path decision:
//! `play_launch resolve` ingests the system config and fills the model's
//! execution layer; consumers never parse `system.toml` themselves).
//!
//! This is a LENIENT SUBSET reader: it understands the execution-relevant
//! sections (`[system]` defaults, `[deploy.<name>]`, `[[transport]]`,
//! `[[bridge]]`) and ignores everything else (components, tiers — tiers
//! ride the sched pipeline). Unknown fields are tolerated so richer
//! consumer schemas (nano-ros's full `SystemToml`) stay authoritative for
//! their own sections.
//!
//! Placement: `[deploy.<name>].nodes = ["/fqn", …]` lists the node FQNs
//! deployed to that target. A single deploy block with no `nodes` list
//! means "every node" (the common single-image case); multiple blocks
//! require explicit lists (ambiguity is an error — fail-loud).

use std::collections::BTreeMap;

use serde::Deserialize;

use crate::{Autostart, Bridge, Deploy, Execution, ExtraValue, Target, Transport};

#[derive(Debug, Default, Deserialize)]
pub struct SystemConfigToml {
    #[serde(default)]
    pub system: SystemDefaults,
    #[serde(default)]
    pub deploy: BTreeMap<String, DeployBlock>,
    #[serde(default, rename = "transport")]
    pub transports: Vec<TransportBlock>,
    #[serde(default, rename = "bridge")]
    pub bridges: Vec<BridgeBlock>,
    /// nano-ros `[tiers.<name>]` scheduling tiers (RFC-0015). The block is
    /// the shared-schema `sched::TierDef` verbatim — `spin_period_us` +
    /// per-RTOS sub-tables — so it deserializes straight into
    /// `execution.tiers`. (play_launch's own systems supply tiers via
    /// `--sched`; nano-ros authors them inline in `system.toml`, and the
    /// canonical-path decision is that `resolve` consumes `system.toml`.)
    #[serde(default)]
    pub tiers: BTreeMap<String, ros_launch_manifest_sched::TierDef>,
    /// nano-ros `[lifecycle] autostart = "none|configure|active"`.
    #[serde(default)]
    pub lifecycle: Option<LifecycleBlock>,
    /// nano-ros `[[component]]` rows — carry `group_tiers` (the
    /// group→tier bindings) keyed by the component/node `name`.
    #[serde(default, rename = "component")]
    pub components: Vec<ComponentBlock>,
}

#[derive(Debug, Default, Deserialize)]
pub struct LifecycleBlock {
    #[serde(default)]
    pub autostart: Option<String>,
}

#[derive(Debug, Default, Deserialize)]
pub struct ComponentBlock {
    #[serde(default)]
    pub name: Option<String>,
    #[serde(default)]
    pub class: Option<String>,
    #[serde(default)]
    pub pkg: Option<String>,
    /// `group_tiers = { <group> = <tier> }` — RFC-0047 group→tier binding.
    #[serde(default)]
    pub group_tiers: BTreeMap<String, String>,
}

#[derive(Debug, Default, Deserialize)]
pub struct SystemDefaults {
    #[serde(default)]
    pub rmw: Option<String>,
    #[serde(default)]
    pub domain_id: Option<u32>,
    #[serde(default)]
    pub locator: Option<String>,
    #[serde(default)]
    pub features: Vec<String>,
}

#[derive(Debug, Default, Deserialize)]
pub struct DeployBlock {
    #[serde(default)]
    pub kind: Option<String>,
    #[serde(default)]
    pub target: Option<String>,
    #[serde(default)]
    pub board: Option<String>,
    #[serde(default)]
    pub framework: Option<String>,
    #[serde(default)]
    pub rmw: Option<String>,
    #[serde(default)]
    pub domain_id: Option<u32>,
    #[serde(default)]
    pub locator: Option<String>,
    #[serde(default)]
    pub profile: Option<String>,
    #[serde(default)]
    pub optimize: Option<String>,
    #[serde(default)]
    pub features: Vec<String>,
    /// R1-P1 placement — node FQNs deployed to this target.
    #[serde(default)]
    pub nodes: Vec<String>,
    /// Launch file this block applies to, relative to the bringup pkg
    /// (`multihost.launch.xml`). `None` means "every launch file".
    ///
    /// nano-ros has always written this key (its own `DeployTarget` mirror
    /// carries a `launch` field), but this struct did not declare it and
    /// serde silently dropped it. Placement then counted launch-scoped blocks
    /// against launch files they were never meant to govern — see
    /// [`Self::applies_to_launch`] and nano-ros issue 0291.
    #[serde(default)]
    pub launch: Option<String>,
}

impl DeployBlock {
    /// Whether this block governs `launch_file`.
    ///
    /// An unscoped block (`launch` absent) governs every launch file. A scoped
    /// one governs only its own — compared on the file NAME, since the key is
    /// written relative to the bringup pkg while callers hold a full path.
    /// An unknown caller launch (`None`) keeps every block, preserving the
    /// pre-0291 behaviour for callers that cannot say which file they resolve.
    pub fn applies_to_launch(&self, launch_file: Option<&str>) -> bool {
        let (Some(scope), Some(current)) = (self.launch.as_deref(), launch_file) else {
            return true;
        };
        let base = |p: &str| p.rsplit(['/', '\\']).next().unwrap_or(p).to_string();
        base(scope) == base(current)
    }
}

#[derive(Debug, Default, Deserialize)]
pub struct TransportBlock {
    #[serde(default)]
    pub kind: Option<String>,
    #[serde(default)]
    pub id: Option<String>,
    #[serde(default)]
    pub ip: Option<String>,
    #[serde(default)]
    pub mac: Option<String>,
    #[serde(default)]
    pub gateway: Option<String>,
    #[serde(default)]
    pub interfaces: Vec<String>,
    #[serde(default)]
    pub ssid: Option<String>,
    #[serde(default)]
    pub password: Option<String>,
    #[serde(default)]
    pub device: Option<String>,
    #[serde(default)]
    pub baudrate: Option<u32>,
    #[serde(default)]
    pub rmw: Option<String>,
    #[serde(default)]
    pub locator: Option<String>,
    #[serde(default)]
    pub domain: Option<u32>,
}

#[derive(Debug, Default, Deserialize)]
pub struct BridgeBlock {
    pub name: String,
    pub from: String,
    pub to: String,
    #[serde(default)]
    pub topics: Vec<String>,
    #[serde(default)]
    pub bidirectional: bool,
}

/// Parse the integrator `system.toml` (lenient subset).
pub fn parse_system_config(input: &str) -> Result<SystemConfigToml, String> {
    toml::from_str(input).map_err(|e| e.to_string())
}

fn clamp_domain(d: Option<u32>, diags: &mut Vec<String>, ctx: &str) -> Option<u8> {
    match d {
        None => None,
        Some(v) if v <= 232 => Some(v as u8),
        Some(v) => {
            diags.push(format!(
                "system config: {ctx}: domain_id {v} out of ROS range 0..=232 — ignored"
            ));
            None
        }
    }
}

impl SystemConfigToml {
    /// Fill an [`Execution`] layer from this config for the given node
    /// FQNs. Returns human-readable diagnostics (unknown placement nodes,
    /// clamped domains); ambiguity (multiple deploy blocks, none listing a
    /// node) is an `Err` — fail-loud, never silent partial placement.
    pub fn apply_to(
        &self,
        execution: &mut Execution,
        node_fqns: &[&str],
    ) -> Result<Vec<String>, String> {
        self.apply_to_launch(execution, node_fqns, None)
    }

    /// [`Self::apply_to`], told WHICH launch file is being resolved.
    ///
    /// Deploy blocks scoped with `launch = "…"` are filtered out when they
    /// name a different file. Without this, a bringup with one unscoped block
    /// plus two scoped ones looked like "three deploy blocks" to every launch
    /// file, and the multi-block rule below demanded an explicit
    /// `nodes = [..]` for nodes the scoped blocks never governed — a hard
    /// error on a config that is correct (nano-ros issue 0291).
    pub fn apply_to_launch(
        &self,
        execution: &mut Execution,
        node_fqns: &[&str],
        launch_file: Option<&str>,
    ) -> Result<Vec<String>, String> {
        let mut diags = Vec::new();

        execution.features = self.system.features.clone();

        // nano-ros `[tiers.*]` → the execution tier table (verbatim schema).
        execution.tiers = self.tiers.clone();

        // `[[component]].group_tiers` → `execution.bindings`
        // (`<node FQN>/<group>` → tier). Map the component `name` to its
        // resolved node FQN (bare-name match against the launch nodes); an
        // unmatched component is a diagnostic, not fatal (conditional nodes
        // may be absent in this variant).
        for c in &self.components {
            let Some(name) = &c.name else { continue };
            if c.group_tiers.is_empty() {
                continue;
            }
            let fqn = node_fqns
                .iter()
                .find(|f| f.rsplit('/').next().unwrap_or(f) == name.as_str());
            match fqn {
                Some(fqn) => {
                    for (group, tier) in &c.group_tiers {
                        execution
                            .bindings
                            .insert(format!("{fqn}/{group}"), tier.clone());
                    }
                }
                None => diags.push(format!(
                    "system config: [[component]] '{name}' has group_tiers but no \
                     matching launch node (absent in this variant?)"
                )),
            }
        }

        for t in &self.transports {
            execution.transports.push(Transport {
                kind: t.kind.clone().unwrap_or_else(|| "ethernet".to_string()),
                id: t.id.clone(),
                ip: t.ip.clone(),
                mac: t.mac.clone(),
                gateway: t.gateway.clone(),
                interfaces: t.interfaces.clone(),
                ssid: t.ssid.clone(),
                password: t.password.clone(),
                device: t.device.clone(),
                baudrate: t.baudrate,
                rmw: t.rmw.clone(),
                locator: t.locator.clone(),
                domain: clamp_domain(t.domain, &mut diags, "transport"),
            });
        }
        for b in &self.bridges {
            execution.bridges.push(Bridge {
                name: b.name.clone(),
                from: b.from.clone(),
                to: b.to.clone(),
                topics: b.topics.clone(),
                bidirectional: b.bidirectional,
            });
        }

        if self.deploy.is_empty() {
            return Ok(diags);
        }

        // Placement resolution — over the blocks that GOVERN this launch file.
        let in_scope: Vec<(&String, &DeployBlock)> = self
            .deploy
            .iter()
            .filter(|(_, b)| b.applies_to_launch(launch_file))
            .collect();
        if in_scope.is_empty() {
            return Ok(diags);
        }
        let single = (in_scope.len() == 1).then(|| in_scope[0].0);
        for fqn in node_fqns {
            let (dname, block) = if let Some(k) = single {
                let b = &self.deploy[k];
                if !b.nodes.is_empty() && !b.nodes.iter().any(|n| n == fqn) {
                    continue; // explicitly not placed
                }
                (k.as_str(), b)
            } else {
                // Placement sources, in order:
                //   1. an explicit `nodes = [..]` entry;
                //   2. the node's own `<node machine="…">`, which `model_builder`
                //      has already recorded as `execution.deploy[fqn].host`.
                //
                // (2) matters because `machine=` IS a placement — the multi-host
                // example says `machine="robot1"` and expects `[deploy.robot1]`.
                // Demanding a duplicate `nodes = [..]` for something the launch
                // file already states is redundant, and made that example
                // unresolvable (nano-ros issue 0291).
                let by_machine = execution
                    .deploy
                    .get(*fqn)
                    .and_then(|d| d.host.as_deref())
                    .and_then(|h| in_scope.iter().find(|(k, _)| k.as_str() == h))
                    .map(|(k, b)| (k.as_str(), *b));
                match in_scope
                    .iter()
                    .find(|(_, b)| b.nodes.iter().any(|n| n == fqn))
                    .map(|(k, b)| (k.as_str(), *b))
                    .or(by_machine)
                {
                    Some((k, b)) => (k, b),
                    None => {
                        return Err(format!(
                            "system config: node '{fqn}' is not placed — with multiple \
                             [deploy.*] blocks every node needs a `nodes = [..]` entry"
                        ));
                    }
                }
            };
            let target = if let Some(board) = &block.board {
                Target::Mcu {
                    board: board.clone(),
                }
            } else {
                Target::Linux
            };
            let mut extra = BTreeMap::new();
            if let Some(v) = &block.kind {
                extra.insert("kind".to_string(), ExtraValue::Str(v.clone()));
            }
            if let Some(v) = &block.target {
                extra.insert("target".to_string(), ExtraValue::Str(v.clone()));
            }
            if let Some(v) = &block.framework {
                extra.insert("framework".to_string(), ExtraValue::Str(v.clone()));
            }
            if let Some(v) = &block.profile {
                extra.insert("profile".to_string(), ExtraValue::Str(v.clone()));
            }
            if let Some(v) = &block.optimize {
                extra.insert("optimize".to_string(), ExtraValue::Str(v.clone()));
            }
            if !block.features.is_empty() {
                extra.insert(
                    "features".to_string(),
                    ExtraValue::StrList(block.features.clone()),
                );
            }
            extra.insert(
                "deploy_name".to_string(),
                ExtraValue::Str(dname.to_string()),
            );
            // Preserve a launch-derived host (`<node machine="…">`) — this
            // insert replaces the whole entry, and blanking it here dropped
            // the very placement that selected this block (issue 0291).
            let existing_host = execution.deploy.get(*fqn).and_then(|d| d.host.clone());
            execution.deploy.insert(
                (*fqn).to_string(),
                Deploy {
                    target: Some(target),
                    host: existing_host,
                    // RFC-0004 ladder: deploy override > system default.
                    domain: clamp_domain(block.domain_id, &mut diags, dname)
                        .or_else(|| clamp_domain(self.system.domain_id, &mut diags, "[system]")),
                    locator: block
                        .locator
                        .clone()
                        .or_else(|| self.system.locator.clone()),
                    rmw: block.rmw.clone().or_else(|| self.system.rmw.clone()),
                    extra,
                },
            );
        }

        // Placement lists naming unknown nodes: diagnostic, not fatal
        // (conditional nodes may be absent in this variant).
        for (dname, block) in &self.deploy {
            for n in &block.nodes {
                if !node_fqns.contains(&n.as_str()) {
                    diags.push(format!(
                        "system config: [deploy.{dname}] places unknown node '{n}' \
                         (absent in this variant?)"
                    ));
                }
            }
        }
        Ok(diags)
    }

    /// The `[lifecycle] autostart` level, if declared. The consumer applies
    /// it to each lifecycle node's `structure.nodes[].lifecycle_autostart`
    /// (it lives in the structure layer, which `apply_to` — execution-only —
    /// cannot reach). Unknown strings return `None`.
    pub fn lifecycle_autostart(&self) -> Option<Autostart> {
        match self.lifecycle.as_ref()?.autostart.as_deref()? {
            "active" => Some(Autostart::Active),
            "configure" => Some(Autostart::Configure),
            "none" => Some(Autostart::None),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOML: &str = r#"
[system]
rmw = "zenoh"
domain_id = 7
features = ["safety"]

[deploy.native]
nodes = ["/ctrl/control_node"]
profile = "release"

[deploy.imu]
board = "stm32f4"
rmw = "cyclonedds"
domain_id = 9
nodes = ["/sensing/imu_node"]
optimize = "size"

[[transport]]
kind = "ethernet"
id = "eth0"
ip = "10.0.2.50/24"
mac = "02:00:00:00:00:01"
domain = 7

[[bridge]]
name = "uplink"
from = "eth0"
to = "can0"
topics = ["/perception/objects"]
"#;

    #[test]
    fn reads_tiers_lifecycle_and_group_tier_bindings() {
        // nano-ros system.toml shape: inline [tiers.*], [[component]]
        // group_tiers, [lifecycle].
        let toml = r#"
[system]
rmw = "zenoh"
[lifecycle]
autostart = "active"
[[component]]
pkg = "ctrl_pkg"
class = "ctrl_pkg::Ctrl"
name = "control_node"
group_tiers = { ctrl = "high" }
[[component]]
pkg = "telem_pkg"
class = "telem_pkg::Telem"
name = "telem_node"
group_tiers = { telem = "low" }
[tiers.high]
spin_period_us = 1000
[tiers.high.posix]
priority = 80
[tiers.high.zephyr]
priority = 5
[tiers.low]
spin_period_us = 10000
[tiers.low.posix]
priority = 10
"#;
        let cfg = parse_system_config(toml).expect("parses");
        let mut e = Execution::default();
        let diags = cfg
            .apply_to(&mut e, &["/control_node", "/telem_node"])
            .expect("applies");
        assert!(diags.is_empty(), "{diags:?}");
        // tiers rode in verbatim.
        assert_eq!(e.tiers.len(), 2);
        assert_eq!(e.tiers["high"].spin_period_us, Some(1000));
        assert_eq!(e.tiers["high"].posix.as_ref().unwrap().priority, 80);
        assert_eq!(e.tiers["high"].zephyr.as_ref().unwrap().priority, 5);
        // group_tiers → bindings keyed by FQN.
        assert_eq!(e.bindings["/control_node/ctrl"], "high");
        assert_eq!(e.bindings["/telem_node/telem"], "low");
        // lifecycle autostart accessor.
        assert_eq!(cfg.lifecycle_autostart(), Some(Autostart::Active));
    }

    /// nano-ros issue 0291 — a bringup with ONE unscoped deploy block plus
    /// launch-scoped siblings must not trip the multi-block placement rule.
    ///
    /// This is `examples/workspaces/cpp/src/demo_bringup` verbatim: three
    /// `[deploy.*]` blocks, no `nodes = [..]` anywhere, robot1/robot2 scoped
    /// to `multihost.launch.xml`. Resolving `system.launch.xml` used to fail
    /// with "node '/listener' is not placed" because `launch` was not a field
    /// on `DeployBlock` at all — serde dropped it and all three blocks counted.
    #[test]
    fn launch_scoped_deploy_blocks_do_not_govern_other_launch_files() {
        let toml = r#"
[system]
name = "demo"

[deploy.native]
kind = "self"
target = "x86_64-unknown-linux-gnu"

[deploy.robot1]
kind = "self"
target = "x86_64-unknown-linux-gnu"
launch = "multihost.launch.xml"

[deploy.robot2]
kind = "self"
target = "x86_64-unknown-linux-gnu"
launch = "multihost.launch.xml"
"#;
        let cfg = parse_system_config(toml).expect("parses");

        // The key must round-trip now, not be silently dropped.
        assert_eq!(
            cfg.deploy["robot1"].launch.as_deref(),
            Some("multihost.launch.xml"),
            "`launch` must be a declared field, not swallowed by serde"
        );

        // system.launch.xml: only `native` is in scope -> implicit placement.
        let mut e = Execution::default();
        cfg.apply_to_launch(&mut e, &["/talker", "/listener"], Some("system.launch.xml"))
            .expect("one in-scope block places implicitly");
        assert!(e.deploy.contains_key("/talker"));
        assert!(e.deploy.contains_key("/listener"));

        // A full path resolves the same — comparison is on the file name.
        let mut e2 = Execution::default();
        cfg.apply_to_launch(
            &mut e2,
            &["/talker"],
            Some("/abs/src/demo_bringup/launch/system.launch.xml"),
        )
        .expect("path and bare name agree");
        assert!(e2.deploy.contains_key("/talker"));

        // multihost.launch.xml: native + both robots are in scope -> genuinely
        // ambiguous without `nodes = [..]`, so the fail-loud rule still fires.
        let mut e3 = Execution::default();
        let err = cfg
            .apply_to_launch(&mut e3, &["/talker"], Some("multihost.launch.xml"))
            .expect_err("three in-scope blocks with no placement is still an error");
        assert!(err.contains("is not placed"), "{err}");

        // Callers that cannot name the launch file keep the old behaviour.
        let mut e4 = Execution::default();
        assert!(
            cfg.apply_to(&mut e4, &["/talker"]).is_err(),
            "an unknown launch file must not silently narrow the scope"
        );
    }

    /// nano-ros issue 0291 — `<node machine="robot1">` IS a placement.
    ///
    /// `model_builder` records it as `execution.deploy[fqn].host` before this
    /// runs, so demanding a duplicate `nodes = [..]` for the same fact made
    /// the multi-host example unresolvable. A host naming an in-scope deploy
    /// block places the node.
    #[test]
    fn node_machine_attribute_places_without_a_nodes_list() {
        let toml = r#"
[system]
name = "demo"

[deploy.native]
kind = "self"

[deploy.robot1]
kind = "self"
launch = "multihost.launch.xml"

[deploy.robot2]
kind = "self"
launch = "multihost.launch.xml"
"#;
        let cfg = parse_system_config(toml).expect("parses");

        // What `model_builder` leaves behind for `<node machine="…">`.
        let mut e = Execution::default();
        e.deploy.entry("/talker".to_string()).or_default().host = Some("robot1".into());
        e.deploy.entry("/listener".to_string()).or_default().host = Some("robot2".into());

        cfg.apply_to_launch(
            &mut e,
            &["/talker", "/listener"],
            Some("multihost.launch.xml"),
        )
        .expect("machine= places both nodes");

        assert_eq!(e.deploy["/talker"].host.as_deref(), Some("robot1"));
        assert_eq!(e.deploy["/listener"].host.as_deref(), Some("robot2"));

        // A host naming no deploy block is still unplaced -> fail loud.
        let mut e2 = Execution::default();
        e2.deploy.entry("/ghost".to_string()).or_default().host = Some("nowhere".into());
        let err = cfg
            .apply_to_launch(&mut e2, &["/ghost"], Some("multihost.launch.xml"))
            .expect_err("an unknown host is not a placement");
        assert!(err.contains("is not placed"), "{err}");
    }

    #[test]
    fn fills_execution_with_ladder_and_placement() {
        let cfg = parse_system_config(TOML).expect("parses");
        let mut e = Execution::default();
        let diags = cfg
            .apply_to(&mut e, &["/ctrl/control_node", "/sensing/imu_node"])
            .expect("applies");
        assert!(diags.is_empty(), "{diags:?}");
        assert_eq!(e.features, vec!["safety"]);
        let ctrl = &e.deploy["/ctrl/control_node"];
        assert_eq!(ctrl.target, Some(Target::Linux));
        assert_eq!(ctrl.domain, Some(7), "system default rides the ladder");
        assert_eq!(ctrl.rmw.as_deref(), Some("zenoh"));
        assert_eq!(ctrl.extra["profile"], ExtraValue::Str("release".into()));
        let imu = &e.deploy["/sensing/imu_node"];
        assert_eq!(
            imu.target.clone().unwrap(),
            Target::Mcu {
                board: "stm32f4".into()
            }
        );
        assert_eq!(imu.domain, Some(9), "deploy override wins");
        assert_eq!(imu.rmw.as_deref(), Some("cyclonedds"));
        assert_eq!(e.transports[0].mac.as_deref(), Some("02:00:00:00:00:01"));
        assert_eq!(e.bridges[0].name, "uplink");
    }

    #[test]
    fn unplaced_node_with_multiple_blocks_fails_loud() {
        let cfg = parse_system_config(TOML).expect("parses");
        let mut e = Execution::default();
        let err = cfg
            .apply_to(&mut e, &["/ctrl/control_node", "/ghost"])
            .unwrap_err();
        assert!(err.contains("'/ghost' is not placed"), "{err}");
    }

    #[test]
    fn single_block_defaults_to_all_nodes() {
        let cfg = parse_system_config(
            "[system]\nrmw = \"zenoh\"\n[deploy.native]\nprofile = \"debug\"\n",
        )
        .expect("parses");
        let mut e = Execution::default();
        cfg.apply_to(&mut e, &["/a", "/b"]).expect("applies");
        assert_eq!(e.deploy.len(), 2);
        assert_eq!(e.deploy["/a"].target, Some(Target::Linux));
    }
}
