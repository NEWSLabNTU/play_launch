//! `<autoware_node>` — Autoware 2.1.0's launch action (#0095).
//!
//! Registered by `autoware_agnocast_wrapper`
//! (`autoware_agnocast_wrapper/actions/autoware_node.py`,
//! `@expose_action("autoware_node")`). It accepts everything `<node>`
//! accepts plus `mode`, `target` and `<extra_arg>` children, and when it
//! EXECUTES it becomes one of two ordinary actions:
//!
//! - with `target` (and not running on Agnocast): a `LoadComposableNodes`
//!   into that container, the component class read from the ament resource
//!   `autoware_node_plugins/<package>__<executable>`;
//! - otherwise: a `Node` process, whose environment carries the transport
//!   the action settled on (`ENABLE_AGNOCAST`, and `LD_PRELOAD` with or
//!   without the heaphook), over whatever the launch file's `<env>` said.
//!
//! This module decides which, as `AutowareNode.execute` does, and hands the
//! result to the paths `<load_composable_node>` and `<node>` already take —
//! there is no second implementation of either.

use crate::{
    actions::{
        ComposableNodeAction, LoadComposableNodeAction, NodeAction, Parameter,
        container::composable_param_file, node::ParamSourceSpec,
    },
    error::{ParseError, Result},
    substitution::{LaunchContext, Substitution, parse_substitutions, resolve_substitutions},
    xml::{Entity, XmlEntity},
};
use std::collections::HashMap;

/// The ament resource type the plugin and the Agnocast build flag come from.
pub const PLUGIN_RESOURCE: &str = "autoware_node_plugins";

const MODES: [&str; 2] = ["auto", "rclcpp"];

/// An `<autoware_node>`, parsed and not yet executed.
#[derive(Debug, Clone)]
pub struct AutowareNodeAction {
    /// Everything `<node>` accepts — `AutowareNode.parse` hands what is left
    /// after its own attributes to `Node.parse`.
    pub node: NodeAction,
    pub mode: Option<Vec<Substitution>>,
    pub target: Option<Vec<Substitution>>,
    /// `<extra_arg name= value=/>`, both substitutions, in document order.
    pub extra_args: Vec<(Vec<Substitution>, Vec<Substitution>)>,
}

/// What an `<autoware_node>` becomes when it executes.
#[derive(Debug)]
pub enum AutowareNodeForm {
    /// A component loaded into `target`.
    Component(LoadComposableNodeAction),
    /// A process of its own.
    Process(Box<NodeAction>),
}

impl AutowareNodeAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        let attr = |name: &str| -> Result<Option<Vec<Substitution>>> {
            entity
                .optional_attr_str(name)?
                .map(|s| parse_substitutions(&s))
                .transpose()
        };
        let mode = attr("mode")?;
        let target = attr("target")?;
        let mut extra_args = Vec::new();
        for child in entity.children() {
            if child.type_name() != "extra_arg" {
                continue;
            }
            crate::xml::attr_spec::validate_attrs(&child)?;
            let get = |a: &str| -> Result<Vec<Substitution>> {
                let v =
                    child
                        .required_attr_str(a)?
                        .ok_or_else(|| ParseError::MissingAttribute {
                            element: "extra_arg".to_string(),
                            attribute: a.to_string(),
                        })?;
                parse_substitutions(&v)
            };
            extra_args.push((get("name")?, get("value")?));
        }
        Ok(Self {
            node: NodeAction::from_entity(entity)?,
            mode,
            target,
            extra_args,
        })
    }

    /// Decide the form, as `AutowareNode.execute` does, against the context
    /// this action executes in.
    pub fn resolve(&self, context: &LaunchContext) -> Result<AutowareNodeForm> {
        let perform = |subs: &[Substitution]| -> Result<String> {
            resolve_substitutions(subs, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))
        };
        let package = perform(&self.node.package)?;
        let executable = perform(&self.node.executable)?;
        let name = self.node.name.as_deref().map(perform).transpose()?;
        let target = self.target.as_deref().map(perform).transpose()?;
        let label = name
            .clone()
            .filter(|n| !n.is_empty())
            .unwrap_or_else(|| executable.clone());
        let (plugin, built_with_agnocast) = registration(context, &package, &executable);

        // Read unconditionally, so an unknown mode is rejected in every
        // build and run, as upstream does.
        let mode = self
            .mode
            .as_deref()
            .map(perform)
            .transpose()?
            .filter(|m| !m.is_empty())
            .unwrap_or_else(|| "auto".to_string());
        if !MODES.contains(&mode.as_str()) {
            return Err(failed(format!(
                "'{label}' has an unknown mode '{mode}', expected one of {}",
                MODES.join(", ")
            )));
        }

        // `use_agnocast` overrides `ENABLE_AGNOCAST`. Upstream reads
        // `os.environ`, which on Humble is what `<set_env>`/`<unset_env>`
        // write — the view `resolve_environment_variable` gives.
        let launch_enabled = context
            .resolve_environment_variable("ENABLE_AGNOCAST")
            .as_deref()
            == Some("1");
        let enabled = match context.get_configuration("use_agnocast") {
            Some(v) => v == "1",
            None => launch_enabled,
        };
        let use_agnocast = built_with_agnocast && enabled && mode != "rclcpp";

        let target = target.filter(|t| !t.is_empty());
        let target = if use_agnocast && target.is_some() {
            log::info!(
                "'{label}' runs as a process of its own rather than in '{}', since it runs on \
                 Agnocast",
                target.as_deref().unwrap_or_default()
            );
            None
        } else {
            target
        };

        if let Some(target) = target {
            let Some(plugin) = plugin else {
                return Err(failed(format!(
                    "'{executable}' of {package} registers no {PLUGIN_RESOURCE} resource, so \
                     the component to load as '{label}' is unknown"
                )));
            };
            if built_with_agnocast && launch_enabled {
                log::warn!(
                    "loading '{label}' into '{target}' while ENABLE_AGNOCAST=1: the container \
                     has to be a plain rclcpp one started with ENABLE_AGNOCAST=0"
                );
            }
            return Ok(AutowareNodeForm::Component(
                self.component(context, target, plugin, &label)?,
            ));
        }

        Ok(AutowareNodeForm::Process(Box::new(self.process(
            context,
            use_agnocast,
            enabled && plugin.is_none(),
            &package,
            &executable,
            &label,
        )?)))
    }

    /// `LoadComposableNodes(target, [ComposableNode(package, plugin, name,
    /// namespace, parameters, remappings, extra_arguments)])`. Every process
    /// attribute sits this form out, as upstream.
    fn component(
        &self,
        context: &LaunchContext,
        target: String,
        plugin: String,
        label: &str,
    ) -> Result<LoadComposableNodeAction> {
        let err =
            |e: crate::error::SubstitutionError| ParseError::InvalidSubstitution(e.to_string());
        let mut parameters = Vec::new();
        for source in &self.node.param_sources {
            match source {
                ParamSourceSpec::Inline(p) => parameters.push(p.evaluate(context).map_err(err)?),
                ParamSourceSpec::File { path, .. } => {
                    let path = resolve_substitutions(path, context).map_err(err)?;
                    parameters.extend(composable_param_file(&path, context, || label.to_string())?);
                }
            }
        }
        let remappings = self
            .node
            .remappings
            .iter()
            .map(|r| {
                Ok((
                    resolve_substitutions(&r.from, context).map_err(err)?,
                    resolve_substitutions(&r.to, context).map_err(err)?,
                ))
            })
            .collect::<Result<Vec<_>>>()?;
        // Extra arguments are parameters to `launch_ros`, typed the same way.
        let mut extra_args = HashMap::new();
        for (name, value) in &self.extra_args {
            let name = resolve_substitutions(name, context).map_err(err)?;
            let (k, v) = Parameter::new(name, value.clone())
                .evaluate(context)
                .map_err(err)?;
            extra_args.insert(k, v);
        }
        Ok(LoadComposableNodeAction {
            target: vec![Substitution::Text(target)],
            composable_nodes: vec![ComposableNodeAction {
                package: self.node.package.clone(),
                plugin: vec![Substitution::Text(plugin)],
                name: self.node.name.clone().unwrap_or_default(),
                namespace: self.node.namespace.clone(),
                parameters,
                remappings,
                extra_args,
            }],
        })
    }

    /// The `Node` upstream returns: the launch file's `<env>` first, then
    /// the transport the action settled on, which wins.
    fn process(
        &self,
        context: &LaunchContext,
        use_agnocast: bool,
        warn_unregistered: bool,
        package: &str,
        executable: &str,
        label: &str,
    ) -> Result<NodeAction> {
        let mut node = self.node.clone();

        let given = match node.environment.iter().position(|(k, _)| k == "LD_PRELOAD") {
            Some(i) => {
                let (_, value) = node.environment.remove(i);
                Some(
                    resolve_substitutions(&parse_substitutions(&value)?, context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?,
                )
            }
            None => None,
        };
        let base = given.clone().unwrap_or_else(|| {
            context
                .resolve_environment_variable("LD_PRELOAD")
                .unwrap_or_default()
        });
        let configured = configured_heaphook(context);

        let mut transport: Vec<(String, String)> = Vec::new();
        if use_agnocast {
            let heaphook = heaphook_path(&configured, given.as_deref(), label)?;
            transport.push(("ENABLE_AGNOCAST".into(), "1".into()));
            transport.push(("LD_PRELOAD".into(), ld_preload(&heaphook, &base, label)));
        } else {
            if warn_unregistered {
                log::warn!(
                    "'{executable}' of {package} registers no {PLUGIN_RESOURCE} resource, so \
                     '{label}' runs on rclcpp although Agnocast is enabled"
                );
            }
            transport.push(("ENABLE_AGNOCAST".into(), "0".into()));
            let kept = without_heaphook(&configured, &base, given.is_some(), label);
            if given.is_some() || kept != base {
                transport.push(("LD_PRELOAD".into(), kept));
            }
        }
        // `env.update(transport_env)`: a key the launch file set keeps its
        // place and takes the action's value.
        for (k, v) in transport {
            match node.environment.iter_mut().find(|(key, _)| *key == k) {
                Some(slot) => slot.1 = v,
                None => node.environment.push((k, v)),
            }
        }
        Ok(node)
    }
}

/// `get_resource("autoware_node_plugins", "<package>__<executable>")`: the
/// first prefix on `AMENT_PREFIX_PATH` holding it, its content
/// `<component class>;<0|1>`. `(None, false)` when no prefix registers it.
fn registration(
    context: &LaunchContext,
    package: &str,
    executable: &str,
) -> (Option<String>, bool) {
    let prefixes = context
        .resolve_environment_variable("AMENT_PREFIX_PATH")
        .unwrap_or_default();
    let resource = format!("{package}__{executable}");
    let content = prefixes
        .split(':')
        .filter(|p| !p.is_empty())
        .find_map(|prefix| {
            std::fs::read_to_string(
                std::path::Path::new(prefix)
                    .join("share/ament_index/resource_index")
                    .join(PLUGIN_RESOURCE)
                    .join(&resource),
            )
            .ok()
        });
    match content {
        Some(content) => {
            let content = content.trim();
            let (plugin, flag) = content.split_once(';').unwrap_or((content, ""));
            (Some(plugin.to_string()), flag == "1")
        }
        None => (None, false),
    }
}

fn failed(message: String) -> ParseError {
    ParseError::ActionFailed {
        action: "<autoware_node>".to_string(),
        message,
    }
}

fn basename(path: &str) -> &str {
    path.rsplit('/').next().unwrap_or(path)
}

/// The `agnocast_heaphook_path` configuration, else the distro's default.
fn configured_heaphook(context: &LaunchContext) -> String {
    context
        .get_configuration("agnocast_heaphook_path")
        .unwrap_or_else(|| {
            let distro = context
                .resolve_environment_variable("ROS_DISTRO")
                .unwrap_or_else(|| "humble".to_string());
            format!("/opt/ros/{distro}/lib/libagnocast_heaphook.so")
        })
}

/// The heaphook to preload: one in the `LD_PRELOAD` the launch file gave
/// wins, else the configured one. A path (not a bare file name) must exist.
fn heaphook_path(configured: &str, given: Option<&str>, label: &str) -> Result<String> {
    let heaphook = given
        .and_then(|g| {
            g.split(':')
                .find(|e| basename(e) == basename(configured))
                .map(str::to_string)
        })
        .unwrap_or_else(|| configured.to_string());
    if heaphook.contains('/') && !std::path::Path::new(&heaphook).exists() {
        return Err(failed(format!(
            "the Agnocast heaphook '{heaphook}' for '{label}' does not exist; point <env> \
             LD_PRELOAD or the 'agnocast_heaphook_path' launch configuration at the one built \
             against this workspace's agnocastlib"
        )));
    }
    Ok(heaphook)
}

/// Prepend the heaphook to `base`, keeping exactly one of it.
fn ld_preload(heaphook: &str, base: &str, label: &str) -> String {
    let mut kept = Vec::new();
    let mut replaced = Vec::new();
    for entry in base.split(':').filter(|e| !e.is_empty()) {
        if basename(entry) != basename(heaphook) {
            kept.push(entry);
        } else if entry != heaphook {
            replaced.push(entry);
        }
    }
    if !replaced.is_empty() {
        log::warn!(
            "the LD_PRELOAD for '{label}' already carries {}; preloading '{heaphook}' instead",
            replaced.join(", ")
        );
    }
    std::iter::once(heaphook)
        .chain(kept)
        .collect::<Vec<_>>()
        .join(":")
}

/// Drop the heaphook from `base`.
fn without_heaphook(heaphook: &str, base: &str, given: bool, label: &str) -> String {
    let entries: Vec<&str> = base.split(':').filter(|e| !e.is_empty()).collect();
    let kept: Vec<&str> = entries
        .iter()
        .copied()
        .filter(|e| basename(e) != basename(heaphook))
        .collect();
    if given && kept.len() != entries.len() {
        log::warn!(
            "'{label}' does not run on Agnocast, so the heaphook in its <env> LD_PRELOAD is left \
             out"
        );
    }
    kept.join(":")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ld_preload_keeps_one_heaphook_first() {
        let h = "/opt/ros/humble/lib/libagnocast_heaphook.so";
        assert_eq!(ld_preload(h, "", "n"), h);
        assert_eq!(
            ld_preload(h, "/a.so:/x/libagnocast_heaphook.so:/b.so", "n"),
            format!("{h}:/a.so:/b.so")
        );
        assert_eq!(
            without_heaphook(h, "/a.so:/y/libagnocast_heaphook.so", true, "n"),
            "/a.so"
        );
    }

    #[test]
    fn a_bare_heaphook_name_is_the_linkers_to_resolve() {
        assert_eq!(
            heaphook_path("libagnocast_heaphook.so", None, "n").unwrap(),
            "libagnocast_heaphook.so"
        );
        assert!(heaphook_path("/nonexistent/libagnocast_heaphook.so", None, "n").is_err());
    }
}
