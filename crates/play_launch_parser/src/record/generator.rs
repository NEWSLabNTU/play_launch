//! Command-line and record generation

use crate::{
    actions::{ExecutableAction, NodeAction},
    error::GenerationError,
    params::load_and_resolve_param_file,
    record::types::NodeRecord,
    substitution::{LaunchContext, resolve_substitutions},
};
use std::path::Path;

/// Normalize boolean values to Python convention (True/False instead of true/false).
/// This matches the Python parser's output format which uses Python's str(bool) convention.
pub fn normalize_param_value(value: &str) -> String {
    match value {
        "true" => "True".to_string(),
        "false" => "False".to_string(),
        _ => value.to_string(),
    }
}

/// Resolve executable path by trying package resolution, falling back to hardcoded path.
pub fn resolve_exec_path(package: &str, executable: &str) -> String {
    crate::python::bridge::find_package_executable(package, executable)
        .unwrap_or_else(|| format!("/opt/ros/humble/lib/{}/{}", package, executable))
}

/// Build a ROS 2 command line from already-resolved values.
///
/// Canonical parameter order (matches Python parser):
/// 1. exec_path
/// 2. args (before --ros-args)
/// 3. --ros-args
/// 4. -r __node:=name (if present)
/// 5. -r __ns:=ns (if non-empty and not "/")
/// 6. -p key:=value global params (normalized)
/// 7. --params-file path
/// 8. -p key:=value node params (normalized)
/// 9. -r from:=to remappings
#[allow(clippy::too_many_arguments)]
pub fn build_ros_command(
    exec_path: &str,
    name: Option<&str>,
    namespace: Option<&str>,
    global_params: &[(String, String)],
    params: &[(String, String)],
    params_files: &[String],
    remaps: &[(String, String)],
    args: &[String],
) -> Vec<String> {
    let mut cmd = vec![exec_path.to_string()];

    // 2. Custom arguments (before --ros-args)
    for arg in args {
        cmd.push(arg.clone());
    }

    // 3. ROS args delimiter
    cmd.push("--ros-args".to_string());

    // 4. Node name
    if let Some(node_name) = name {
        cmd.push("-r".to_string());
        cmd.push(format!("__node:={}", node_name));
    }

    // 5. Namespace (only if non-root)
    if let Some(ns) = namespace
        && !ns.is_empty()
        && ns != "/"
    {
        cmd.push("-r".to_string());
        cmd.push(format!("__ns:={}", ns));
    }

    // 6. Global parameters (normalized)
    for (key, value) in global_params {
        cmd.push("-p".to_string());
        cmd.push(format!("{}:={}", key, normalize_param_value(value)));
    }

    // 7. Parameter files
    for params_file in params_files {
        cmd.push("--params-file".to_string());
        cmd.push(params_file.clone());
    }

    // 8. Node-specific parameters (normalized)
    for (key, value) in params {
        cmd.push("-p".to_string());
        cmd.push(format!("{}:={}", key, normalize_param_value(value)));
    }

    // 9. Remappings
    for (from, to) in remaps {
        cmd.push("-r".to_string());
        cmd.push(format!("{}:={}", from, to));
    }

    cmd
}

/// Merge global parameters with node-specific parameters.
/// Global parameters come first, node-specific parameters can override by key.
pub fn merge_params_with_global(
    global_params: &[(String, String)],
    node_params: &[(String, String)],
) -> Vec<(String, String)> {
    let mut merged: Vec<(String, String)> = global_params.to_vec();

    for (key, value) in node_params {
        if let Some(existing) = merged.iter_mut().find(|(k, _)| k == key) {
            existing.1 = value.clone();
        } else {
            merged.push((key.clone(), value.clone()));
        }
    }

    merged
}

pub struct CommandGenerator;

impl CommandGenerator {
    pub fn generate_node_record(
        node: &NodeAction,
        context: &LaunchContext,
    ) -> Result<NodeRecord, GenerationError> {
        let package = resolve_substitutions(&node.package, context)?;
        let executable = resolve_substitutions(&node.executable, context)?;

        let name = if let Some(name_subs) = &node.name {
            Some(resolve_substitutions(name_subs, context)?)
        } else {
            None
        };

        let namespace = if let Some(ns_subs) = &node.namespace {
            // Node has explicit namespace - resolve and normalize it
            let ns_resolved = resolve_substitutions(ns_subs, context)?;

            // Ensure namespace is absolute (starts with '/') or combine with current namespace
            let normalized_ns = if ns_resolved.starts_with('/') {
                // Already absolute
                ns_resolved
            } else if ns_resolved.is_empty() {
                // Empty namespace means use current namespace from context
                context.current_namespace()
            } else {
                // Relative namespace: combine with current namespace from context
                let current_ns = context.current_namespace();
                if current_ns == "/" {
                    format!("/{}", ns_resolved)
                } else {
                    format!("{}/{}", current_ns, ns_resolved)
                }
            };
            // If normalized namespace is "/" (root), output None to match Python behavior
            if normalized_ns == "/" {
                None
            } else {
                Some(normalized_ns)
            }
        } else {
            // Use namespace from context (group scoping)
            // If current namespace is "/" (root/unspecified), output None to match Python behavior
            let current_ns = context.current_namespace();
            if current_ns == "/" {
                None
            } else {
                Some(current_ns)
            }
        };

        // Process inline parameters (normalize booleans to Python convention)
        let mut params: Vec<(String, String)> = node
            .parameters
            .iter()
            .map(|p| {
                let resolved_value = resolve_substitutions(&p.value, context)?;
                Ok((p.name.clone(), normalize_param_value(&resolved_value)))
            })
            .collect::<Result<Vec<_>, GenerationError>>()?;

        // Process parameter files
        // Python distinguishes between temp files and regular files:
        // - Temp files (/tmp/launch_params_*): Expand into inline params
        // - Regular files: Store resolved YAML content
        let mut params_files = Vec::new();
        let mut params_file_paths = Vec::new();
        for param_file_subs in &node.param_files {
            let param_file_path = resolve_substitutions(param_file_subs, context)?;

            // Check if this is a temp parameter file (created by launch system)
            if param_file_path.contains("/tmp/launch_params_") {
                // Temp file: extract parameters as inline params
                let extracted_params =
                    crate::params::extract_params_from_yaml(Path::new(&param_file_path), context)
                        .map_err(|e| {
                        GenerationError::IoError(format!(
                            "Failed to extract parameters from temp file '{}': {}",
                            param_file_path, e
                        ))
                    })?;
                params.extend(extracted_params);
            } else {
                // Regular file: load and resolve substitutions in YAML content
                let resolved_contents =
                    load_and_resolve_param_file(Path::new(&param_file_path), context).map_err(
                        |e| {
                            GenerationError::IoError(format!(
                                "Failed to load and resolve parameter file '{}': {}",
                                param_file_path, e
                            ))
                        },
                    )?;
                params_files.push(resolved_contents);
                params_file_paths.push(param_file_path);
            }
        }

        // Collect node-specific remappings
        let node_remaps: Result<Vec<_>, GenerationError> = node
            .remappings
            .iter()
            .map(|r| {
                let from = resolve_substitutions(&r.from, context)?;
                let to = resolve_substitutions(&r.to, context)?;
                Ok((from, to))
            })
            .collect();
        let mut remaps = node_remaps?;

        // Prepend global remappings from context (set_remap actions)
        // Global remappings are added first so node-specific remappings can override them
        let global_remaps = context.remappings(); // Already owned Vec, no clone needed
        remaps.splice(0..0, global_remaps);

        // Merge context environment with node-specific environment
        // Node-specific environment takes precedence
        let mut merged_env = context.environment(); // Already owned HashMap, no clone needed
        for (key, value) in &node.environment {
            // Resolve any substitutions in the environment value
            let substitutions = crate::substitution::parse_substitutions(value).map_err(|e| {
                GenerationError::Substitution(crate::error::SubstitutionError::InvalidSubstitution(
                    e.to_string(),
                ))
            })?;
            let resolved_value = resolve_substitutions(&substitutions, context)?;
            merged_env.insert(key.clone(), resolved_value); // key.clone() needed since we're iterating with &
        }

        let env = if merged_env.is_empty() {
            None
        } else {
            Some(merged_env.into_iter().collect::<Vec<_>>())
        };

        // Get global parameters from context (already filtered to SetParameter values)
        // Normalize booleans to Python convention (True/False)
        let global_params = if context.global_parameters().is_empty() {
            None
        } else {
            Some(
                context
                    .global_parameters()
                    .into_iter()
                    .map(|(k, v)| (k, normalize_param_value(&v)))
                    .collect::<Vec<_>>(),
            )
        };

        // Resolve args attribute (command-line arguments before --ros-args)
        let args: Option<Vec<String>> = if let Some(ref args_subs) = node.args {
            let resolved = resolve_substitutions(args_subs, context)?;
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

        // Build command using already-resolved values (matching Python parser behavior)
        let cmd = Self::build_node_command(
            &package,
            &executable,
            &name,
            &namespace,
            &remaps,
            &params,
            &global_params,
            &params_file_paths,
            &args,
        )?;

        Ok(NodeRecord {
            args,
            cmd,
            env,
            exec_name: Some(executable.clone()),
            executable,
            global_params,
            name,
            namespace,
            package: Some(package),
            params,
            params_files,
            remaps,
            respawn: node
                .respawn
                .as_ref()
                .map(|subs| {
                    let resolved = resolve_substitutions(subs, context)?;
                    resolved.parse::<bool>().map_err(|_| {
                        GenerationError::Substitution(
                            crate::error::SubstitutionError::InvalidSubstitution(format!(
                                "Failed to parse respawn value '{}' as boolean",
                                resolved
                            )),
                        )
                    })
                })
                .transpose()?,
            respawn_delay: node
                .respawn_delay
                .as_ref()
                .map(|subs| {
                    let resolved = resolve_substitutions(subs, context)?;
                    resolved.parse::<f64>().map_err(|_| {
                        GenerationError::Substitution(
                            crate::error::SubstitutionError::InvalidSubstitution(format!(
                                "Failed to parse respawn_delay value '{}' as number",
                                resolved
                            )),
                        )
                    })
                })
                .transpose()?,
            ros_args: None,
            scope: None,
        })
    }

    pub fn generate_node_command(
        node: &NodeAction,
        context: &LaunchContext,
    ) -> Result<Vec<String>, GenerationError> {
        let record = Self::generate_node_record(node, context)?;
        Ok(record.cmd)
    }

    /// Build command line from already-resolved values.
    /// Delegates to the shared `build_ros_command()` function.
    #[allow(clippy::too_many_arguments)]
    fn build_node_command(
        package: &str,
        executable: &str,
        name: &Option<String>,
        namespace: &Option<String>,
        remaps: &[(String, String)],
        params: &[(String, String)],
        global_params: &Option<Vec<(String, String)>>,
        params_file_paths: &[String],
        args: &Option<Vec<String>>,
    ) -> Result<Vec<String>, GenerationError> {
        let exec_path = resolve_exec_path(package, executable);
        let empty_gp = Vec::new();
        let gp = global_params.as_deref().unwrap_or(&empty_gp);
        let empty_args = Vec::new();
        let arg_list = args.as_deref().unwrap_or(&empty_args);

        Ok(build_ros_command(
            &exec_path,
            name.as_deref(),
            namespace.as_deref(),
            gp,
            params,
            params_file_paths,
            remaps,
            arg_list,
        ))
    }

    pub fn generate_executable_record(
        exec: &ExecutableAction,
        context: &LaunchContext,
    ) -> Result<NodeRecord, GenerationError> {
        let cmd_str = resolve_substitutions(&exec.cmd, context)?;

        // Build command vector - clone cmd_str since we need it for executable field
        let mut cmd = vec![cmd_str.clone()];

        // Add arguments
        for arg in &exec.arguments {
            let arg_str = resolve_substitutions(arg, context)?;
            cmd.push(arg_str);
        }

        // Calculate name - if custom name provided, use it; otherwise use cmd[0] reference
        // This avoids cloning cmd_str twice
        let name = if let Some(name_subs) = &exec.name {
            Some(resolve_substitutions(name_subs, context)?)
        } else {
            // Reuse cmd[0] instead of cloning cmd_str again
            cmd.first().cloned()
        };

        // Merge context environment with executable-specific environment
        // Executable-specific environment takes precedence
        let mut merged_env = context.environment(); // Already owned HashMap, no clone needed
        for (key, value) in &exec.environment {
            // Resolve any substitutions in the environment value
            let substitutions = crate::substitution::parse_substitutions(value).map_err(|e| {
                GenerationError::Substitution(crate::error::SubstitutionError::InvalidSubstitution(
                    e.to_string(),
                ))
            })?;
            let resolved_value = resolve_substitutions(&substitutions, context)?;
            merged_env.insert(key.clone(), resolved_value); // key.clone() needed since we're iterating with &
        }

        let env = if merged_env.is_empty() {
            None
        } else {
            Some(merged_env.into_iter().collect::<Vec<_>>())
        };

        // Get global parameters from context (already filtered to SetParameter values)
        // Normalize booleans to Python convention (True/False)
        let global_params = if context.global_parameters().is_empty() {
            None
        } else {
            Some(
                context
                    .global_parameters()
                    .into_iter()
                    .map(|(k, v)| (k, normalize_param_value(&v)))
                    .collect::<Vec<_>>(),
            )
        };

        Ok(NodeRecord {
            args: if exec.arguments.is_empty() {
                None
            } else {
                Some(
                    exec.arguments
                        .iter()
                        .map(|a| resolve_substitutions(a, context))
                        .collect::<Result<Vec<_>, _>>()?,
                )
            },
            cmd,
            env,
            exec_name: name.clone(),
            executable: cmd_str,
            global_params,
            name,
            namespace: None, // Executables don't have namespaces
            package: None,   // Executables don't have packages
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        actions::{Parameter, Remapping},
        substitution::{LaunchContext, Substitution},
    };

    #[test]
    fn test_generate_simple_command() {
        let node = NodeAction {
            package: vec![Substitution::Text("demo_nodes_cpp".to_string())],
            executable: vec![Substitution::Text("talker".to_string())],
            name: None,
            namespace: None,
            parameters: vec![],
            param_files: vec![],
            remappings: vec![],
            environment: vec![],
            args: None,
            output: None,
            respawn: None,
            respawn_delay: None,
        };

        let context = LaunchContext::new();
        let cmd = CommandGenerator::generate_node_command(&node, &context).unwrap();

        // Executable path resolved via find_package_executable or fallback
        assert!(cmd[0].ends_with("/demo_nodes_cpp/talker"));
        assert_eq!(cmd[1], "--ros-args");
        // No __node:= when name is None (matches Python parser - ROS 2 defaults to executable)
        assert!(!cmd.iter().any(|s| s.starts_with("__node:=")));
        // No __ns when namespace is root (matches Python parser)
        assert!(!cmd.contains(&"__ns:=/".to_string()));
    }

    #[test]
    fn test_generate_command_with_params() {
        let node = NodeAction {
            package: vec![Substitution::Text("demo".to_string())],
            executable: vec![Substitution::Text("node".to_string())],
            name: None,
            namespace: None,
            parameters: vec![Parameter {
                name: "rate".to_string(),
                value: vec![Substitution::Text("10.0".to_string())],
            }],
            param_files: vec![],
            remappings: vec![],
            environment: vec![],
            args: None,
            output: None,
            respawn: None,
            respawn_delay: None,
        };

        let context = LaunchContext::new();
        let cmd = CommandGenerator::generate_node_command(&node, &context).unwrap();

        assert!(cmd.contains(&"-p".to_string()));
        assert!(cmd.contains(&"rate:=10.0".to_string()));
    }

    #[test]
    fn test_generate_command_with_remaps() {
        let node = NodeAction {
            package: vec![Substitution::Text("demo".to_string())],
            executable: vec![Substitution::Text("node".to_string())],
            name: None,
            namespace: None,
            parameters: vec![],
            param_files: vec![],
            remappings: vec![Remapping {
                from: vec![Substitution::Text("chatter".to_string())],
                to: vec![Substitution::Text("/chat".to_string())],
            }],
            environment: vec![],
            args: None,
            output: None,
            respawn: None,
            respawn_delay: None,
        };

        let context = LaunchContext::new();
        let cmd = CommandGenerator::generate_node_command(&node, &context).unwrap();

        assert!(cmd.contains(&"-r".to_string()));
        assert!(cmd.contains(&"chatter:=/chat".to_string()));
    }
}
