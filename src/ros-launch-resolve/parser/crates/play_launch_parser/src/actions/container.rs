//! Container action for composable nodes

use crate::{
    error::{ParseError, Result},
    params::extract_params_from_yaml,
    record::{ComposableNodeContainerRecord, LoadNodeRecord},
    substitution::{LaunchContext, Substitution, parse_substitutions, resolve_substitutions},
    xml::{Entity, XmlEntity},
};
use std::{collections::HashMap, path::Path};

/// Default ROS 2 package for component containers.
pub const DEFAULT_CONTAINER_PACKAGE: &str = "rclcpp_components";
/// Default ROS 2 executable for component containers.
pub const DEFAULT_CONTAINER_EXECUTABLE: &str = "component_container";

/// Container action representing a composable node container
#[derive(Debug, Clone)]
pub struct ContainerAction {
    pub name: Vec<Substitution>,
    pub namespace: Option<Vec<Substitution>>,
    pub package: Vec<Substitution>,
    pub executable: Vec<Substitution>,
    pub args: Option<Vec<Substitution>>,
    /// `<node_container ros_args=…>` — issue #9. Its own `--ros-args` block,
    /// which is how Autoware suppresses an INFO-spammy container
    /// (`--log-level <container>:=warn`). Survives a `--container-mode`
    /// override untouched: that rewrites `args`, never this.
    pub ros_args: Option<Vec<Substitution>>,
    /// `<node_container respawn=… respawn_delay=…>` — a container is a node,
    /// and ROS 2 honours both. These used to be accepted by the attribute
    /// spec and then dropped, so a respawning container did not respawn
    /// under the Rust parser while the Python one carried it. Composables
    /// have no respawn attribute of their own; play_launch's
    /// `composable_respawn: inherit` reads the container's.
    pub respawn: Option<Vec<Substitution>>,
    pub respawn_delay: Option<Vec<Substitution>>,
    pub composable_nodes: Vec<ComposableNodeAction>,
    /// A container is a `Node`: its own `<param>`, `<remap>` and `<env>`
    /// reach its process.
    pub parameters: Vec<crate::actions::node::Parameter>,
    pub param_files: Vec<Vec<Substitution>>,
    pub remappings: Vec<crate::actions::node::Remapping>,
    pub environment: Vec<(String, String)>,
}

/// The container's `respawn`/`respawn_delay`, resolved the way the Python
/// dump does (`composable_node_container.py`): `respawn` is `Some(true)` only
/// when it is true — false and unset are both `None` — and a delay is kept
/// as given.
fn resolve_respawn(
    respawn: Option<&Vec<Substitution>>,
    respawn_delay: Option<&Vec<Substitution>>,
    context: &LaunchContext,
) -> Result<(Option<bool>, Option<f64>)> {
    let respawn = respawn
        .map(|subs| {
            resolve_substitutions(subs, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))
        })
        .transpose()?
        .and_then(|v| {
            matches!(v.trim().to_ascii_lowercase().as_str(), "true" | "1" | "yes").then_some(true)
        });
    let respawn_delay = respawn_delay
        .map(|subs| {
            let v = resolve_substitutions(subs, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            v.trim().parse::<f64>().map_err(|_| {
                ParseError::InvalidSubstitution(format!(
                    "Failed to parse node_container respawn_delay value '{v}' as number"
                ))
            })
        })
        .transpose()?;
    Ok((respawn, respawn_delay))
}

/// Composable node action
#[derive(Debug, Clone)]
pub struct ComposableNodeAction {
    pub package: Vec<Substitution>,
    pub plugin: Vec<Substitution>,
    pub name: Vec<Substitution>,
    pub namespace: Option<Vec<Substitution>>,
    pub parameters: Vec<(String, String)>,
    pub remappings: Vec<(String, String)>,
    pub extra_args: HashMap<String, String>,
}

/// Resolve a namespace value against context, handling absolute/relative logic.
fn resolve_namespace(
    ns_subs: Option<&Vec<Substitution>>,
    context: &LaunchContext,
) -> Result<String> {
    if let Some(ns) = ns_subs {
        let ns_resolved = resolve_substitutions(ns, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        if ns_resolved.starts_with('/') {
            Ok(ns_resolved)
        } else if ns_resolved.is_empty() {
            Ok(context.current_namespace())
        } else {
            let current_ns = context.current_namespace();
            if current_ns == "/" {
                Ok(format!("/{}", ns_resolved))
            } else {
                Ok(format!("{}/{}", current_ns, ns_resolved))
            }
        }
    } else {
        Ok(context.current_namespace())
    }
}

impl ContainerAction {
    pub fn from_entity(entity: &XmlEntity, context: &LaunchContext) -> Result<Self> {
        // Get container name — parse only, don't resolve
        let name_str =
            entity
                .required_attr_str("name")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "node_container".to_string(),
                    attribute: "name".to_string(),
                })?;
        let name = parse_substitutions(&name_str)?;

        // Get package (defaults to DEFAULT_CONTAINER_PACKAGE if not specified)
        let package = if let Some(pkg_str) = entity.optional_attr_str("pkg")? {
            parse_substitutions(&pkg_str)?
        } else {
            vec![Substitution::Text(DEFAULT_CONTAINER_PACKAGE.to_string())]
        };

        // Get executable (defaults to DEFAULT_CONTAINER_EXECUTABLE if not specified)
        let executable = if let Some(exec_str) = entity.optional_attr_str("exec")? {
            parse_substitutions(&exec_str)?
        } else {
            vec![Substitution::Text(DEFAULT_CONTAINER_EXECUTABLE.to_string())]
        };

        // Get namespace — parse only, resolve at use site
        let namespace = entity
            .optional_attr_str("namespace")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        // Parse args attribute (command-line arguments before --ros-args)
        let args_raw = entity.optional_attr_str("args")?;
        log::debug!(
            "Container args_raw={:?}, all attrs={:?}",
            args_raw,
            entity.attributes()
        );
        let args = args_raw.map(|s| parse_substitutions(&s)).transpose()?;

        // Parse ros_args attribute (arguments in their own --ros-args block)
        // — issue #9.
        let ros_args = entity
            .optional_attr_str("ros_args")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        let respawn = entity
            .optional_attr_str("respawn")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;
        let respawn_delay = entity
            .optional_attr_str("respawn_delay")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        // Parse composable_node children
        let mut composable_nodes = Vec::new();
        let mut parameters = Vec::new();
        let mut param_files = Vec::new();
        let mut remappings = Vec::new();
        let mut environment = Vec::new();
        for child in entity.children() {
            // Child elements never reach `traverse_entity` — validate here.
            crate::xml::attr_spec::validate_attrs(&child)?;
            match child.type_name() {
                "composable_node" | "composable-node" => {
                    // Issue #7: `if=`/`unless=` were accepted by the attribute
                    // spec and then never evaluated, so a composable the
                    // launch file excluded was loaded anyway — silently, and
                    // differently from `ros2 launch`. Conditions are resolved
                    // at parse time here exactly as they are for `<node>`,
                    // because the selected path is what this parser records.
                    if !crate::condition::should_process_entity(&child, context)? {
                        log::debug!(
                            "Skipping composable_node '{}' — excluded by if/unless",
                            child.optional_attr_str("name")?.unwrap_or_default()
                        );
                        continue;
                    }
                    composable_nodes.push(ComposableNodeAction::from_entity(&child, context)?);
                }
                "param" => {
                    if let Some(from) = child.optional_attr_str("from")? {
                        param_files.push(parse_substitutions(&from)?);
                    } else {
                        parameters.extend(crate::actions::node::Parameter::from_entity(&child)?);
                    }
                }
                "remap" => remappings.push(crate::actions::node::Remapping::from_entity(&child)?),
                "env" => environment.push(crate::actions::node::parse_env(&child)?),
                other => {
                    log::warn!("Unexpected element '{}' in node_container", other);
                }
            }
        }

        Ok(Self {
            name,
            namespace,
            package,
            executable,
            args,
            ros_args,
            respawn,
            respawn_delay,
            composable_nodes,
            parameters,
            param_files,
            remappings,
            environment,
        })
    }

    /// What the container process gets as a `Node`: its parameters (inline
    /// and files), remappings — the global ones (`<set_remap>`) first — and
    /// environment (`<set_env>` plus its own `<env>`).
    #[allow(clippy::type_complexity)]
    fn node_inputs(
        &self,
        context: &LaunchContext,
    ) -> Result<(
        Vec<(String, String)>,
        Vec<String>,
        Vec<(String, String)>,
        Option<Vec<(String, String)>>,
    )> {
        let sub_err =
            |e: crate::error::SubstitutionError| ParseError::InvalidSubstitution(e.to_string());
        let params = self
            .parameters
            .iter()
            .map(|p| p.evaluate(context).map_err(sub_err))
            .collect::<Result<Vec<_>>>()?;
        let files = self
            .param_files
            .iter()
            .map(|f| resolve_substitutions(f, context).map_err(sub_err))
            .collect::<Result<Vec<_>>>()?;
        let mut remaps = context.remappings();
        for r in &self.remappings {
            remaps.push((
                resolve_substitutions(&r.from, context).map_err(sub_err)?,
                resolve_substitutions(&r.to, context).map_err(sub_err)?,
            ));
        }
        let mut env = context.environment();
        for (k, v) in &self.environment {
            let subs = parse_substitutions(v)?;
            env.insert(
                k.clone(),
                resolve_substitutions(&subs, context).map_err(sub_err)?,
            );
        }
        let env = (!env.is_empty()).then(|| env.into_iter().collect());
        Ok((params, files, remaps, env))
    }

    pub fn to_container_record(
        &self,
        context: &LaunchContext,
    ) -> Result<ComposableNodeContainerRecord> {
        use crate::record::generator::{build_ros_command, resolve_exec_path};

        // Resolve deferred fields
        let name = resolve_substitutions(&self.name, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let package = resolve_substitutions(&self.package, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let executable = resolve_substitutions(&self.executable, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let namespace = resolve_namespace(self.namespace.as_ref(), context)?;

        let exec_path = resolve_exec_path(&package, &executable);
        let gp: Vec<(String, String)> = context.global_parameters().into_iter().collect();
        let ns_ref = if namespace.is_empty() || namespace == "/" {
            None
        } else {
            Some(namespace.as_str())
        };

        // Resolve args (before --ros-args) and ros_args (its own block,
        // issue #9).
        let resolve_args = |subs: &Option<Vec<Substitution>>| -> Result<Option<Vec<String>>> {
            subs.as_deref()
                .map(|s| crate::record::generator::resolve_arg_list(s, context))
                .transpose()
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))
                .map(|o| o.flatten())
        };
        let arguments = resolve_args(&self.args)?;
        let ros_arguments = resolve_args(&self.ros_args)?;
        let (respawn, respawn_delay) =
            resolve_respawn(self.respawn.as_ref(), self.respawn_delay.as_ref(), context)?;

        let empty_args = Vec::new();
        let arg_list = arguments.as_deref().unwrap_or(&empty_args);
        let ros_arg_list = ros_arguments.as_deref().unwrap_or(&empty_args);

        let (params, files, remaps, env) = self.node_inputs(context)?;
        let cmd = build_ros_command(
            &exec_path,
            Some(name.as_str()),
            ns_ref,
            &gp,
            &params,
            &files,
            &remaps,
            arg_list,
            ros_arg_list,
        );

        let global_params = if gp.is_empty() { None } else { Some(gp) };

        Ok(ComposableNodeContainerRecord {
            start_delay_secs: None,
            args: arguments,
            cmd,
            env,
            exec_name: Some(name.clone()),
            executable,
            global_params,
            name: name.clone(),
            namespace,
            package,
            params,
            params_files: files
                .iter()
                .map(|f| {
                    crate::params::load_and_resolve_param_file(std::path::Path::new(f), context)
                        .unwrap_or_else(|_| f.clone())
                })
                .collect(),
            param_sources: Vec::new(),
            remaps,
            respawn,
            respawn_delay,
            ros_args: ros_arguments,
            scope: None,
        })
    }

    pub fn to_load_node_records(&self, context: &LaunchContext) -> Result<Vec<LoadNodeRecord>> {
        // Resolve container name and namespace for passing to composable nodes
        let name = resolve_substitutions(&self.name, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let namespace = resolve_namespace(self.namespace.as_ref(), context)?;

        Ok(self
            .composable_nodes
            .iter()
            .map(|node| node.to_load_node_record(&name, &namespace, context))
            .collect())
    }

    pub fn to_node_record(&self, context: &LaunchContext) -> Result<crate::record::NodeRecord> {
        use crate::record::{
            NodeRecord,
            generator::{build_ros_command, resolve_exec_path},
        };

        // Resolve deferred fields
        let name = resolve_substitutions(&self.name, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let package = resolve_substitutions(&self.package, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let executable = resolve_substitutions(&self.executable, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let namespace = resolve_namespace(self.namespace.as_ref(), context)?;

        let exec_path = resolve_exec_path(&package, &executable);
        let gp: Vec<(String, String)> = context.global_parameters().into_iter().collect();
        let ns_ref = if namespace.is_empty() || namespace == "/" {
            None
        } else {
            Some(namespace.as_str())
        };

        // Resolve args (before --ros-args) and ros_args (its own block,
        // issue #9).
        let resolve_args = |subs: &Option<Vec<Substitution>>| -> Result<Option<Vec<String>>> {
            subs.as_deref()
                .map(|s| crate::record::generator::resolve_arg_list(s, context))
                .transpose()
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))
                .map(|o| o.flatten())
        };
        let arguments = resolve_args(&self.args)?;
        let ros_arguments = resolve_args(&self.ros_args)?;
        let (respawn, respawn_delay) =
            resolve_respawn(self.respawn.as_ref(), self.respawn_delay.as_ref(), context)?;

        let empty_args = Vec::new();
        let arg_list = arguments.as_deref().unwrap_or(&empty_args);
        let ros_arg_list = ros_arguments.as_deref().unwrap_or(&empty_args);

        let cmd = build_ros_command(
            &exec_path,
            Some(name.as_str()),
            ns_ref,
            &gp,
            &[],
            &[],
            &[],
            arg_list,
            ros_arg_list,
        );

        Ok(NodeRecord {
            start_delay_secs: None,
            // The Rust parser does not model on_exit handlers; only the Python
            // dump path carries them (see NodeRecord::on_exit_shutdown).
            on_exit_shutdown: None,
            args: arguments,
            cmd,
            env: None,
            exec_name: None,
            executable,
            global_params: None,
            name: Some(name),
            namespace: Some(namespace),
            package: Some(package),
            params: Vec::new(),
            params_files: Vec::new(),
            param_sources: Vec::new(),
            remaps: Vec::new(),
            respawn,
            respawn_delay,
            ros_args: ros_arguments,
            scope: None,
        })
    }
}

impl ComposableNodeAction {
    pub fn from_entity(entity: &XmlEntity, context: &LaunchContext) -> Result<Self> {
        // Get required attributes — parse only, don't resolve
        let package_str =
            entity
                .required_attr_str("pkg")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "composable_node".to_string(),
                    attribute: "pkg".to_string(),
                })?;
        let package = parse_substitutions(&package_str)?;

        let plugin_str =
            entity
                .required_attr_str("plugin")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "composable_node".to_string(),
                    attribute: "plugin".to_string(),
                })?;
        let plugin = parse_substitutions(&plugin_str)?;

        // Get name (required for composable nodes) — parse only
        let name_str =
            entity
                .required_attr_str("name")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "composable_node".to_string(),
                    attribute: "name".to_string(),
                })?;
        let name = parse_substitutions(&name_str)?;

        // Get optional namespace — parse only
        let namespace = entity
            .optional_attr_str("namespace")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        // Parse children for params, remaps and extra args (still resolved
        // eagerly — these are runtime values)
        let mut parameters = Vec::new();
        let mut remappings = Vec::new();
        let mut extra_args = HashMap::new();

        for child in entity.children() {
            // Child elements never reach `traverse_entity` — validate here.
            crate::xml::attr_spec::validate_attrs(&child)?;
            match child.type_name() {
                "param" => {
                    // Check if this is a parameter file reference
                    if let Some(from_attr) = child.optional_attr_str("from")? {
                        // This is a parameter file - resolve path and load YAML
                        let from_parsed = parse_substitutions(&from_attr)?;
                        let from_resolved = resolve_substitutions(&from_parsed, context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

                        // Load YAML parameter file and extract parameters
                        log::debug!("Found param file='{}', checking if YAML", from_resolved);
                        if from_resolved.ends_with(".yaml") || from_resolved.ends_with(".yml") {
                            match extract_params_from_yaml(Path::new(&from_resolved), context) {
                                Ok(yaml_params) => {
                                    log::debug!(
                                        "Loaded {} parameters from {}",
                                        yaml_params.len(),
                                        from_resolved
                                    );
                                    parameters.extend(yaml_params);
                                }
                                Err(e) => {
                                    log::warn!(
                                        "Failed to load YAML parameter file {}: {}",
                                        from_resolved,
                                        e
                                    );
                                    // Fallback: store as __param_file
                                    parameters.push(("__param_file".to_string(), from_resolved));
                                }
                            }
                        } else {
                            // Non-YAML file - store as reference
                            parameters.push(("__param_file".to_string(), from_resolved));
                        }
                    } else {
                        // An inline parameter, or nested ones, evaluated the
                        // way `launch_ros` evaluates a node's.
                        for p in crate::actions::node::Parameter::from_entity(&child)? {
                            parameters.push(
                                p.evaluate(context)
                                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?,
                            );
                        }
                    }
                }
                "remap" => {
                    let from = child.required_attr_str("from")?.ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "remap".to_string(),
                            attribute: "from".to_string(),
                        }
                    })?;
                    let to = child.required_attr_str("to")?.ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "remap".to_string(),
                            attribute: "to".to_string(),
                        }
                    })?;
                    let from_parsed = parse_substitutions(&from)?;
                    let to_parsed = parse_substitutions(&to)?;
                    let from_resolved = resolve_substitutions(&from_parsed, context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    let to_resolved = resolve_substitutions(&to_parsed, context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    remappings.push((from_resolved, to_resolved));
                }
                "extra_arg" => {
                    // Issue #0022. `extra_arg` was in this element's allowed
                    // children — so a launch file using it validated cleanly —
                    // and then fell through to the `other` arm below, which
                    // logs at debug and drops it. Every layer downstream
                    // already carried the field faithfully (action -> record ->
                    // `NodeInstance.extra_args` -> LoadNode `extra_arguments`,
                    // typed by YAML inference in `parameter_conversion.rs`), so
                    // this was the only hop where the value went missing, and
                    // the author got no signal at all.
                    let name = child.required_attr_str("name")?.ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "extra_arg".to_string(),
                            attribute: "name".to_string(),
                        }
                    })?;
                    let value = child.required_attr_str("value")?.ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "extra_arg".to_string(),
                            attribute: "value".to_string(),
                        }
                    })?;
                    // Extra arguments are parameters to `launch_ros`, typed
                    // the same way.
                    let (name_resolved, value_resolved) =
                        crate::actions::node::Parameter::new(name, parse_substitutions(&value)?)
                            .evaluate(context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    extra_args.insert(name_resolved, value_resolved);
                }
                other => {
                    log::debug!("Skipping '{}' in composable_node", other);
                }
            }
        }

        Ok(Self {
            package,
            plugin,
            name,
            namespace,
            parameters,
            remappings,
            extra_args,
        })
    }

    pub fn to_load_node_record(
        &self,
        container_name: &str,
        container_namespace: &str,
        context: &LaunchContext,
    ) -> LoadNodeRecord {
        // Resolve deferred fields
        let package = resolve_substitutions(&self.package, context).unwrap_or_default();
        let plugin = resolve_substitutions(&self.plugin, context).unwrap_or_default();
        let name = resolve_substitutions(&self.name, context).unwrap_or_default();
        let namespace = if let Some(ref ns) = self.namespace {
            let ns_resolved = resolve_substitutions(ns, context).unwrap_or_default();
            if ns_resolved.starts_with('/') {
                ns_resolved
            } else if ns_resolved.is_empty() {
                context.current_namespace()
            } else {
                let current_ns = context.current_namespace();
                if current_ns == "/" {
                    format!("/{}", ns_resolved)
                } else {
                    format!("{}/{}", current_ns, ns_resolved)
                }
            }
        } else {
            // Issue #0021 — the LAUNCH CONTEXT namespace, NOT the container's.
            //
            // A container's `namespace=` attribute names the container
            // PROCESS; it is not pushed onto the context, so it never reaches
            // the composable nodes loaded into it. Verified against stock
            // `ros2 launch` with a container at `namespace="probe"` inside
            // `/outer/probe`:
            //
            //     /outer/probe/child_one              <- composable node
            //     /outer/probe/probe/probe_container  <- the container itself
            //
            // Reading `container_namespace` here doubled the segment for
            // every composable in such a container, and did so silently: the
            // container's own name stayed right, so entity counts matched and
            // only an FQN comparison showed it. Where a container declares no
            // namespace of its own the two values are equal, which is why
            // nearly every launch file hid this.
            //
            // `container_namespace` is still correct for `target_container_name`
            // below — that one IS the container's own name.
            context.current_namespace()
        };

        log::debug!(
            "to_load_node_record: node='{}', self.parameters.len()={}",
            name,
            self.parameters.len()
        );

        // Build full target container name: namespace + name
        // This matches the Python implementation's behavior
        let target_container_name = if container_namespace == "/" {
            format!("/{}", container_name)
        } else if container_namespace.is_empty() {
            container_name.to_string()
        } else {
            format!("{}/{}", container_namespace, container_name)
        };

        // Merge global parameters from context with node-specific parameters
        let gp: Vec<(String, String)> = context.global_parameters().into_iter().collect();
        let merged_params =
            crate::record::generator::merge_params_with_global(&gp, &self.parameters);

        log::debug!(
            "to_load_node_record: node='{}', final merged_params.len()={}",
            name,
            merged_params.len()
        );

        LoadNodeRecord {
            start_delay_secs: None,
            package,
            plugin,
            target_container_name,
            node_name: name,
            namespace,
            log_level: None,
            // `get_composable_node_load_request` puts the global remappings
            // (`<set_remap>`) ahead of the node's own.
            remaps: {
                let mut remaps = context.remappings();
                remaps.extend(self.remappings.iter().cloned());
                remaps
            },
            params: merged_params,
            extra_args: self.extra_args.clone(),
            env: None,
            scope: None,
        }
    }
}
