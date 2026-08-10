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

use indexmap::IndexMap;
use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::{
    Autostart, Bridge, Deploy, Execution, ExtraValue, NodeInstance, ParamSource, ParamValue,
    Target, Transport,
};

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
    /// nano-ros `[param_services]` — presence enables the runtime parameter
    /// services capability. The historical nano-ros spelling is a bare table
    /// (`[param_services]`), equivalent to `[system] features =
    /// ["param_services"]`; before this field existed the section was
    /// SILENTLY ignored (no top-level deny), so a resolver-produced model
    /// lost the capability while the hand-migrated ones kept it — nano-ros
    /// issue 0387's params arm.
    #[serde(default)]
    pub param_services: Option<ParamServicesBlock>,
}

/// The `[param_services]` capability block. Empty today; a table (not a
/// bool) so future knobs (e.g. persistence) can land without a schema
/// break.
#[derive(Debug, Default, Deserialize)]
pub struct ParamServicesBlock {}

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
    /// `params = { <name> = <value> }` — deployment-time parameter values for
    /// this component's node, equivalent to an inline `<param>` in the launch
    /// file.
    ///
    /// Some parameters are a property of the DEPLOYMENT rather than of the
    /// launch description — a `qos_overrides.…` entry chosen per system is the
    /// motivating case (nano-ros `ws-qos-rust`). Before this existed the only
    /// way to express one was to type it into the resolved model by hand, so
    /// re-resolving silently dropped it: the model was both the artifact and
    /// the only record of the intent.
    #[serde(default)]
    pub params: BTreeMap<String, toml::Value>,
    /// `params_files = ["<yaml content>", …]` — parameter FILE contents for
    /// this component's node, same shape as `NodeInstance::params_files` and
    /// the `<param from=…>` launch form. Applied in order, before `params`.
    #[serde(default)]
    pub params_files: Vec<String>,
}

#[derive(Debug, Default, Deserialize)]
pub struct SystemDefaults {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rmw: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_id: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub locator: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<String>,
}

/// One `[deploy.<name>]` block.
///
/// **This is the single definition of the `system.toml` deploy schema**
/// (nano-ros issue 0293). It used to be mirrored by nano-ros's own
/// `DeployTarget`, and the two drifted: this struct lacked `launch`, so serde
/// silently dropped it and launch-scoped blocks were counted against every
/// launch file. nano-ros re-exports this type rather than redeclaring it.
///
/// `deny_unknown_fields` makes the next divergence a loud parse error instead
/// of a silently ignored key. Safe to add: nano-ros's mirror already denied,
/// so any `system.toml` in use already satisfies this field set.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeployBlock {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kind: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub board: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub framework: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rmw: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain_id: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub locator: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profile: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub optimize: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<String>,
    /// R1-P1 placement — node FQNs deployed to this target.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub nodes: Vec<String>,
    /// Launch file this block applies to, relative to the bringup pkg
    /// (`multihost.launch.xml`). `None` means "every launch file".
    ///
    /// nano-ros has always written this key (its own `DeployTarget` mirror
    /// carries a `launch` field), but this struct did not declare it and
    /// serde silently dropped it. Placement then counted launch-scoped blocks
    /// against launch files they were never meant to govern — see
    /// [`Self::applies_to_launch`] and nano-ros issue 0291.
    #[serde(default, skip_serializing_if = "Option::is_none")]
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rmw: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
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
    /// Project `[[component]] params` / `params_files` onto the STRUCTURE
    /// layer, the way `[lifecycle] autostart` is projected by the resolver.
    ///
    /// `apply_to` only reaches `Execution`; parameters are a per-node property
    /// and live in `structure.nodes`, so this is a separate entry point rather
    /// than a widening of that signature.
    ///
    /// Components match nodes by BARE NAME, the same rule `group_tiers` uses,
    /// and an unmatched component is a diagnostic rather than an error — a
    /// conditional node may legitimately be absent from this variant.
    ///
    /// Precedence follows ROS and the launch path: `params_files` apply in
    /// order, then inline `params` win. Values already present on the node
    /// (from the launch file) are NOT overwritten — the launch description is
    /// the more specific statement.
    pub fn apply_params_to_nodes(&self, nodes: &mut IndexMap<String, NodeInstance>) -> Vec<String> {
        let mut diags = Vec::new();
        for c in &self.components {
            let Some(name) = &c.name else { continue };
            if c.params.is_empty() && c.params_files.is_empty() {
                continue;
            }
            let fqn = nodes
                .keys()
                .find(|f| f.rsplit('/').next().unwrap_or(f) == name.as_str())
                .cloned();
            // Fall back to the component's PACKAGE when the instance name does
            // not match a launch node name. A consolidated workspace gives
            // `[[component]] name` a workspace-unique spelling
            // (`rust_params_param_talker`) while the launch file keeps the plain
            // node name (`param_talker`), so bare-name matching silently finds
            // nothing — every per-node projection then no-ops without saying so.
            // Only an UNAMBIGUOUS pkg counts: two instances of one package are
            // exactly the case the instance name exists to tell apart.
            let fqn = fqn.or_else(|| {
                let pkg = c.pkg.as_deref()?;
                let mut it = nodes
                    .iter()
                    .filter(|(_, inst)| inst.pkg.as_deref() == Some(pkg))
                    .map(|(f, _)| f.clone());
                let first = it.next()?;
                it.next().is_none().then_some(first)
            });
            let Some(fqn) = fqn else {
                diags.push(format!(
                    "system config: [[component]] '{name}' declares params but has no \
                     matching launch node (absent in this variant?)"
                ));
                continue;
            };
            let Some(inst) = nodes.get_mut(&fqn) else {
                continue;
            };
            for content in &c.params_files {
                if !inst.params_files.iter().any(|f| f == content) {
                    inst.params_files.push(content.clone());
                    inst.param_sources.push(ParamSource::File {
                        content: content.clone(),
                    });
                }
            }
            for (k, v) in &c.params {
                let Some(pv) = toml_to_param_value(v) else {
                    diags.push(format!(
                        "system config: [[component]] '{name}' param '{k}' has an \
                         unsupported type; expected bool, integer, float, string or \
                         string list"
                    ));
                    continue;
                };
                // The launch file is more specific — do not overwrite it.
                if inst.params.contains_key(k) {
                    continue;
                }
                inst.params.insert(k.clone(), pv.clone());
                inst.param_sources.push(ParamSource::Inline {
                    name: k.clone(),
                    value: pv,
                });
            }
        }
        diags
    }

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
        // `[param_services]` is sugar for `features = ["param_services"]`.
        if self.param_services.is_some()
            && !execution.features.iter().any(|f| f == "param_services")
        {
            execution.features.push("param_services".to_string());
        }

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

        // Placement resolution — over the blocks that GOVERN this launch file
        // AND that actually PARTITION nodes.
        //
        // `[deploy.*]` carries two different meanings, and only one of them is a
        // placement:
        //
        //   * `kind = "self"`  — a machine. Multiple of these PARTITION the
        //     nodes between them via their `nodes = [..]` lists.
        //   * `kind = "embedded"` — a BOARD BUILD of the whole system. Every
        //     such block runs every node; they are alternatives, not shares.
        //
        // Counting embedded blocks as placement domains asks "which of these
        // six machines runs /talker" about six builds of one image, and the
        // answer is "all of them". `ws-realtime-c` states this plainly: native
        // + zephyr + nuttx, two nodes, no `nodes = [..]` anywhere — and it
        // could not re-resolve at all, failing `node '/ctrl_node' is not
        // placed`. It only looked healthy because its committed model predates
        // this resolver and was never regenerated (nano-ros issue 0320).
        //
        // Same class as issue 0291 immediately above: placement counting blocks
        // it was never meant to govern. That one was a dropped `launch` field;
        // this one is a conflated axis.
        let partitioning: Vec<(&String, &DeployBlock)> = self
            .deploy
            .iter()
            .filter(|(_, b)| b.applies_to_launch(launch_file))
            .filter(|(_, b)| b.kind.as_deref() != Some("embedded"))
            .collect();
        // An embedded-only bringup still has a target worth naming — but only
        // when there is exactly ONE of them.
        //
        // With a single embedded block, falling back gives every node that
        // target, which is unambiguous and useful. With SEVERAL, the fallback
        // asks which of N whole-system board builds runs a given node, and the
        // answer is "all of them" — a node->target map cannot say that. The
        // first version of this fell back unconditionally and so turned two
        // embedded blocks into `node '/ctrl_node' is not placed`, which is the
        // very error this filter exists to prevent (nano-ros ws-realtime-c-mps2
        // and ws-realtime-cpp-mps2 both declare freertos + mps2-an385-freertos).
        //
        // So: emit NO placement instead. Consumers already treat a board the
        // map never mentions as unconstrained, which is exactly right here.
        let governing: Vec<(&String, &DeployBlock)> = self
            .deploy
            .iter()
            .filter(|(_, b)| b.applies_to_launch(launch_file))
            .collect();
        let in_scope: Vec<(&String, &DeployBlock)> = if !partitioning.is_empty() {
            partitioning
        } else if governing.len() == 1 {
            governing
        } else {
            return Ok(diags);
        };
        if in_scope.is_empty() {
            return Ok(diags);
        }
        // nano-ros issue 0356 — a system that ALSO declares `kind = "embedded"`
        // board builds is multi-board: the same nodes run on the self machine(s)
        // AND on every embedded board. The model deploy is single-target per
        // node, so pinning a placed node to its self-block's concrete target
        // (e.g. `linux`) makes `nros codegen entry --board <embedded>` drop it
        // (`keep()` only keeps a `linux` node for the `native`/`posix` boards) —
        // the model placed talker/listener on native, and the freertos entry
        // found nothing. Leave such nodes board-AGNOSTIC (`target = None`): the
        // codegen `keep()` includes a `None` node on EVERY board, and each
        // entry's own `--board` supplies the concrete target. Single-target
        // (no embedded) workspaces keep their exact placement.
        let multi_board = self
            .deploy
            .values()
            .any(|b| b.kind.as_deref() == Some("embedded"));
        let single = (in_scope.len() == 1).then(|| in_scope[0].0);
        for fqn in node_fqns {
            let (dname, block) = if let Some(k) = single {
                let b = &self.deploy[k];
                if !b.nodes.is_empty() && !b.nodes.iter().any(|n| n == fqn) {
                    continue; // explicitly not placed
                }
                (k.as_str(), b)
            } else {
                // The one placement source: an explicit `nodes = [..]` entry.
                //
                // There used to be a second — the node's `<node machine="…">`,
                // recorded by `model_builder` as `execution.deploy[fqn].host`
                // (nano-ros issue 0291) — but `machine=` is ROS 1 roslaunch
                // syntax that ROS 2's frontend rejects, so the capture and the
                // `Deploy.host` field are gone (nano-ros issue 0364). Per-host
                // slicing now happens at RESOLVE time via an ordinary launch
                // argument + `if=` conditions; each per-host model then
                // contains only that host's nodes.
                match in_scope
                    .iter()
                    .find(|(_, b)| b.nodes.iter().any(|n| n == fqn))
                    .map(|(k, b)| (k.as_str(), *b))
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
            let target = if multi_board {
                // Board-agnostic (issue 0356): the entry's `--board` decides.
                None
            } else if let Some(board) = &block.board {
                Some(Target::Mcu {
                    board: board.clone(),
                })
            } else {
                Some(Target::Linux)
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
            execution.deploy.insert(
                (*fqn).to_string(),
                Deploy {
                    target,
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
    /// nano-ros issue 0293 (SSoT half) — this struct is the ONE definition of
    /// the deploy schema, and `deny_unknown_fields` makes the next divergence
    /// loud. A key no field claims is now a parse error, not a silent drop —
    /// silently dropping `launch` is exactly how the original bug happened.
    #[test]
    fn deploy_block_rejects_unknown_keys_and_round_trips() {
        let toml = r#"
[system]
name = "demo"

[deploy.robot1]
kind = "self"
target = "x86_64-unknown-linux-gnu"
launch = "multihost.launch.xml"
nodes = ["/talker"]
"#;
        let cfg = parse_system_config(toml).expect("parses");
        let b = &cfg.deploy["robot1"];
        assert_eq!(b.launch.as_deref(), Some("multihost.launch.xml"));
        assert_eq!(b.nodes, vec!["/talker".to_string()]);

        // Serialize is required because nano-ros WRITES system.toml through
        // this same type; absent options must not appear.
        let out = toml::to_string(b).expect("serializes");
        assert!(out.contains("launch = "), "{out}");
        assert!(
            !out.contains("board"),
            "absent options must be skipped:\n{out}"
        );

        // A key no field claims fails loudly.
        let bad = toml.replace("nodes", "nodez");
        let err = parse_system_config(&bad).expect_err("unknown key must be rejected");
        assert!(err.contains("nodez"), "error should name the key: {err}");
    }

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

    /// nano-ros issue 0356 — a system that declares BOTH self machines and
    /// `kind = "embedded"` board builds is multi-board: the same nodes run on
    /// native AND on every embedded board. Because the model deploy is
    /// single-target per node, such placements must be board-AGNOSTIC
    /// (`target = None`), so `nros codegen entry --board <b>` (whose `keep()`
    /// admits a `None` node on every board) includes them for native and each
    /// embedded board alike. Pinning `native`'s `linux` target here made the
    /// freertos/nuttx/threadx entries codegen-fail "no nodes on board".
    #[test]
    fn embedded_blocks_make_placement_board_agnostic() {
        let toml = r#"
[system]
name = "demo"

[deploy.native]
kind = "self"
target = "x86_64-unknown-linux-gnu"

[deploy.freertos]
kind = "embedded"
board = "mps2-an385-freertos"

[deploy.nuttx]
kind = "embedded"
board = "nuttx-qemu-arm"
"#;
        let cfg = parse_system_config(toml).expect("parses");
        let mut e = Execution::default();
        cfg.apply_to_launch(&mut e, &["/talker", "/listener"], Some("system.launch.xml"))
            .expect("embedded blocks do not partition; native places implicitly");
        // Placed, but board-agnostic — the entry's `--board` decides.
        for n in ["/talker", "/listener"] {
            let d = e.deploy.get(n).unwrap_or_else(|| panic!("{n} placed"));
            assert!(
                d.target.is_none(),
                "{n} must be board-agnostic (target=None) in a multi-board system, got {:?}",
                d.target
            );
        }

        // Control: WITHOUT any embedded block, the same native placement keeps
        // its concrete Linux target (single-board behaviour is unchanged).
        let single = toml
            .split("[deploy.freertos]")
            .next()
            .expect("prefix before embedded blocks");
        let cfg1 = parse_system_config(single).expect("parses");
        let mut e1 = Execution::default();
        cfg1.apply_to_launch(&mut e1, &["/talker"], Some("system.launch.xml"))
            .expect("single self block places");
        assert!(
            matches!(e1.deploy["/talker"].target, Some(Target::Linux)),
            "single-board native placement must keep its Linux target, got {:?}",
            e1.deploy["/talker"].target
        );
    }

    /// nano-ros issue 0364 — with `machine=` gone (ROS 1 roslaunch syntax;
    /// its `Deploy.host` fallback of issue 0291 went with it), `nodes = [..]`
    /// is the ONLY placement source when multiple self blocks are in scope.
    /// A node in no list fails loud instead of riding a launch-derived host.
    #[test]
    fn nodes_lists_are_the_only_placement_with_multiple_self_blocks() {
        let toml = r#"
[system]
name = "demo"

[deploy.native]
kind = "self"

[deploy.robot1]
kind = "self"
launch = "multihost.launch.xml"
nodes = ["/talker"]

[deploy.robot2]
kind = "self"
launch = "multihost.launch.xml"
nodes = ["/listener"]
"#;
        let cfg = parse_system_config(toml).expect("parses");

        let mut e = Execution::default();
        cfg.apply_to_launch(
            &mut e,
            &["/talker", "/listener"],
            Some("multihost.launch.xml"),
        )
        .expect("nodes lists place both nodes");

        assert_eq!(
            e.deploy["/talker"].extra.get("deploy_name"),
            Some(&ExtraValue::Str("robot1".into()))
        );
        assert_eq!(
            e.deploy["/listener"].extra.get("deploy_name"),
            Some(&ExtraValue::Str("robot2".into()))
        );

        // A node in no list is unplaced -> fail loud (pre-0291 behaviour,
        // restored because the machine=-derived fact no longer exists).
        let err = cfg
            .apply_to_launch(&mut e, &["/ghost"], Some("multihost.launch.xml"))
            .expect_err("a node in no nodes list is not placed");
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
    fn component_params_and_files_project_onto_the_node() {
        let cfg = parse_system_config(
            "[system]\nrmw = \"zenoh\"\n\
             [[component]]\nname = \"talker\"\n\
             params = { qos = \"best_effort\", rate = 10 }\n\
             params_files = [\"talker:\\n  ros__parameters:\\n    x: 1\\n\"]\n\
             [deploy.native]\nkind = \"self\"\n",
        )
        .expect("parses");
        let mut nodes = IndexMap::new();
        nodes.insert("/talker".to_string(), NodeInstance::default());
        let diags = cfg.apply_params_to_nodes(&mut nodes);
        assert!(diags.is_empty(), "unexpected diagnostics: {diags:?}");
        let n = &nodes["/talker"];
        assert_eq!(
            n.params.get("qos"),
            Some(&ParamValue::Str("best_effort".into()))
        );
        assert_eq!(n.params.get("rate"), Some(&ParamValue::Int(10)));
        assert_eq!(n.params_files.len(), 1);
        // one File source + two Inline sources
        assert_eq!(n.param_sources.len(), 3);
    }

    #[test]
    fn component_matches_by_pkg_when_the_instance_name_differs() {
        // A consolidated workspace renames `[[component]] name` for uniqueness
        // while the launch file keeps the plain node name. Bare-name matching
        // finds nothing; the pkg is what still ties them together.
        let cfg = parse_system_config(
            "[system]\nrmw = \"zenoh\"\n\
             [[component]]\npkg = \"rust_param_talker_pkg\"\n\
             name = \"rust_params_param_talker\"\nparams = { rate = 7 }\n\
             [deploy.native]\nkind = \"self\"\n",
        )
        .expect("parses");
        let mut inst = NodeInstance::default();
        inst.pkg = Some("rust_param_talker_pkg".to_string());
        let mut nodes = IndexMap::new();
        nodes.insert("/param_talker".to_string(), inst);
        let diags = cfg.apply_params_to_nodes(&mut nodes);
        assert!(diags.is_empty(), "unexpected diagnostics: {diags:?}");
        assert_eq!(
            nodes["/param_talker"].params.get("rate"),
            Some(&ParamValue::Int(7))
        );
    }

    #[test]
    fn an_ambiguous_pkg_does_not_match() {
        // Two instances of one package are exactly what the instance name is
        // for — guessing between them would be worse than the diagnostic.
        let cfg = parse_system_config(
            "[system]\nrmw = \"zenoh\"\n\
             [[component]]\npkg = \"talker_pkg\"\nname = \"ghost\"\nparams = { a = 1 }\n\
             [deploy.native]\nkind = \"self\"\n",
        )
        .expect("parses");
        let mut nodes = IndexMap::new();
        for fqn in ["/talker_one", "/talker_two"] {
            let mut i = NodeInstance::default();
            i.pkg = Some("talker_pkg".to_string());
            nodes.insert(fqn.to_string(), i);
        }
        let diags = cfg.apply_params_to_nodes(&mut nodes);
        assert_eq!(
            diags.len(),
            1,
            "expected the unmatched diagnostic: {diags:?}"
        );
        assert!(nodes["/talker_one"].params.is_empty());
        assert!(nodes["/talker_two"].params.is_empty());
    }

    #[test]
    fn launch_params_win_over_component_params() {
        let cfg = parse_system_config(
            "[system]\nrmw = \"zenoh\"\n\
             [[component]]\nname = \"talker\"\nparams = { rate = 10 }\n\
             [deploy.native]\nkind = \"self\"\n",
        )
        .expect("parses");
        let mut inst = NodeInstance::default();
        inst.params.insert("rate".into(), ParamValue::Int(99));
        let mut nodes = IndexMap::new();
        nodes.insert("/talker".to_string(), inst);
        cfg.apply_params_to_nodes(&mut nodes);
        // The launch description is the more specific statement.
        assert_eq!(
            nodes["/talker"].params.get("rate"),
            Some(&ParamValue::Int(99))
        );
    }

    #[test]
    fn unmatched_component_with_params_is_a_diagnostic_not_a_panic() {
        let cfg = parse_system_config(
            "[system]\nrmw = \"zenoh\"\n\
             [[component]]\nname = \"ghost\"\nparams = { a = 1 }\n\
             [deploy.native]\nkind = \"self\"\n",
        )
        .expect("parses");
        let mut nodes = IndexMap::new();
        nodes.insert("/talker".to_string(), NodeInstance::default());
        let diags = cfg.apply_params_to_nodes(&mut nodes);
        assert_eq!(diags.len(), 1);
        assert!(diags[0].contains("ghost"), "{diags:?}");
    }

    #[test]
    fn bare_param_services_section_projects_into_features() {
        let cfg = parse_system_config(
            "[system]\nrmw = \"zenoh\"\n[param_services]\n[deploy.native]\nkind = \"self\"\n",
        )
        .expect("parses");
        let mut e = Execution::default();
        cfg.apply_to(&mut e, &["/a"]).expect("applies");
        assert_eq!(e.features, vec!["param_services"]);
    }

    #[test]
    fn param_services_section_does_not_duplicate_explicit_feature() {
        let cfg = parse_system_config(
            "[system]\nrmw = \"zenoh\"\nfeatures = [\"param_services\"]\n[param_services]\n[deploy.native]\nkind = \"self\"\n",
        )
        .expect("parses");
        let mut e = Execution::default();
        cfg.apply_to(&mut e, &["/a"]).expect("applies");
        assert_eq!(e.features, vec!["param_services"]);
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

/// TOML scalar → [`ParamValue`]. Returns `None` for shapes the model has no
/// representation for (tables, mixed arrays), which the caller reports as a
/// diagnostic rather than dropping silently.
fn toml_to_param_value(v: &toml::Value) -> Option<ParamValue> {
    Some(match v {
        toml::Value::Boolean(b) => ParamValue::Bool(*b),
        toml::Value::Integer(i) => ParamValue::Int(*i),
        toml::Value::Float(f) => ParamValue::Float(*f),
        toml::Value::String(s) => ParamValue::Str(s.clone()),
        toml::Value::Array(a) => {
            let mut out = Vec::with_capacity(a.len());
            for e in a {
                out.push(e.as_str()?.to_string());
            }
            ParamValue::StrList(out)
        }
        _ => return None,
    })
}
