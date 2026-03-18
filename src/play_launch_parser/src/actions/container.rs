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
    pub composable_nodes: Vec<ComposableNodeAction>,
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

        // Parse composable_node children
        let mut composable_nodes = Vec::new();
        for child in entity.children() {
            match child.type_name() {
                "composable_node" | "composable-node" => {
                    composable_nodes.push(ComposableNodeAction::from_entity(&child, context)?);
                }
                "param" | "remap" | "env" => {
                    // Skip these for now - containers don't typically have their own params
                    log::debug!("Skipping {} in node_container", child.type_name());
                }
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
            composable_nodes,
        })
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

        // Resolve args attribute (command-line arguments before --ros-args)
        let arguments: Option<Vec<String>> = if let Some(ref args_subs) = self.args {
            let resolved = resolve_substitutions(args_subs, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            let arg_list: Vec<String> =
                resolved.split_whitespace().map(|s| s.to_string()).collect();
            if arg_list.is_empty() {
                None
            } else {
                Some(arg_list)
            }
        } else {
            None
        };

        let empty_args = Vec::new();
        let arg_list = arguments.as_deref().unwrap_or(&empty_args);

        let cmd = build_ros_command(
            &exec_path,
            Some(name.as_str()),
            ns_ref,
            &gp,
            &[],
            &[],
            &[],
            arg_list,
        );

        let global_params = if gp.is_empty() { None } else { Some(gp) };

        Ok(ComposableNodeContainerRecord {
            args: arguments,
            cmd,
            env: None,
            exec_name: Some(name.clone()),
            executable,
            global_params,
            name: name.clone(),
            namespace,
            package,
            params: Vec::new(),
            params_files: Vec::new(),
            remaps: Vec::new(),
            respawn: None,
            respawn_delay: None,
            ros_args: None,
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

        // Resolve args attribute (command-line arguments before --ros-args)
        let arguments: Option<Vec<String>> = if let Some(ref args_subs) = self.args {
            let resolved = resolve_substitutions(args_subs, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            let arg_list: Vec<String> =
                resolved.split_whitespace().map(|s| s.to_string()).collect();
            if arg_list.is_empty() {
                None
            } else {
                Some(arg_list)
            }
        } else {
            None
        };

        let empty_args = Vec::new();
        let arg_list = arguments.as_deref().unwrap_or(&empty_args);

        let cmd = build_ros_command(
            &exec_path,
            Some(name.as_str()),
            ns_ref,
            &gp,
            &[],
            &[],
            &[],
            arg_list,
        );

        Ok(NodeRecord {
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
            remaps: Vec::new(),
            respawn: None,
            respawn_delay: None,
            ros_args: None,
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

        // Parse children for params and remaps (still resolved eagerly — these are runtime values)
        let mut parameters = Vec::new();
        let mut remappings = Vec::new();
        let extra_args = HashMap::new();

        for child in entity.children() {
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
                        // This is an inline parameter
                        let name = child.required_attr_str("name")?.ok_or_else(|| {
                            ParseError::MissingAttribute {
                                element: "param".to_string(),
                                attribute: "name".to_string(),
                            }
                        })?;
                        let value = child.required_attr_str("value")?.ok_or_else(|| {
                            ParseError::MissingAttribute {
                                element: "param".to_string(),
                                attribute: "value".to_string(),
                            }
                        })?;
                        let name_parsed = parse_substitutions(&name)?;
                        let value_parsed = parse_substitutions(&value)?;
                        let name_resolved = resolve_substitutions(&name_parsed, context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                        let value_resolved = resolve_substitutions(&value_parsed, context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                        parameters.push((name_resolved, value_resolved));
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
            container_namespace.to_string()
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
            package,
            plugin,
            target_container_name,
            node_name: name,
            namespace,
            log_level: None,
            remaps: self.remappings.clone(),
            params: merged_params,
            extra_args: self.extra_args.clone(),
            env: None,
            scope: None,
        }
    }
}
