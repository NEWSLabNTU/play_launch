use super::super::LaunchTraverser;
use crate::{
    actions::{
        ComposableNodeAction, ContainerAction, ExecutableAction, IncludeAction,
        LoadComposableNodeAction, NodeAction, Parameter, Remapping,
        container::{DEFAULT_CONTAINER_EXECUTABLE, DEFAULT_CONTAINER_PACKAGE},
    },
    condition::is_truthy,
    error::{ParseError, Result},
    file_cache::read_file_cached,
    record::CommandGenerator,
    substitution::{
        ArgumentMetadata, LaunchContext, Substitution, parse_substitutions, resolve_substitutions,
    },
};
use serde_yaml_ng::{Mapping, Value};
use std::{collections::HashMap, path::Path};

impl LaunchTraverser {
    /// Process a YAML launch file (handles all action types)
    pub(crate) fn process_yaml_launch_file(&mut self, path: &Path) -> Result<()> {
        let content = read_file_cached(path)?;
        let yaml: Value = serde_yaml_ng::from_str(&content)
            .map_err(|e| ParseError::InvalidSubstitution(format!("Invalid YAML: {}", e)))?;

        // Save and set current file for correct $(dirname) resolution
        let prev_file = self.context.current_file().cloned();
        self.context.set_current_file(path.to_path_buf());

        let result = if let Some(launch_list) = yaml.get("launch").and_then(|v| v.as_sequence()) {
            self.process_yaml_actions(launch_list, path)
        } else {
            Ok(())
        };

        // Restore previous current file
        if let Some(prev) = prev_file {
            self.context.set_current_file(prev);
        }

        result
    }

    /// Process a list of YAML launch actions
    fn process_yaml_actions(&mut self, items: &[Value], path: &Path) -> Result<()> {
        for item in items {
            let item_map = match item.as_mapping() {
                Some(m) => m,
                None => continue,
            };

            // Each item is a single-key mapping: { action_type: { ...attrs } }
            for (key, value) in item_map {
                let action_type = match key.as_str() {
                    Some(s) => s,
                    None => continue,
                };

                let action_map = value.as_mapping();

                // Same allowlists the XML frontend uses. YAML nests child
                // elements as keys of this mapping (`node: { pkg: …, param:
                // [...] }`) where XML makes them separate elements, so this
                // uses the `_yaml_keys` variant, which also accepts
                // `spec.children`.
                if let Some(map) = action_map {
                    let keys: Vec<&str> = map.keys().filter_map(|k| k.as_str()).collect();
                    crate::xml::attr_spec::validate_yaml_keys(action_type, &keys)?;
                }

                // Check if/unless conditions on the action
                if let Some(map) = action_map
                    && !self.yaml_check_condition(map)?
                {
                    log::debug!("Skipping YAML {} due to condition", action_type);
                    continue;
                }

                match action_type {
                    "arg" => {
                        if let Some(map) = action_map {
                            self.process_yaml_arg(map)?;
                        }
                    }
                    "include" => {
                        if let Some(map) = action_map {
                            self.process_yaml_include(map)?;
                        }
                    }
                    "group" => {
                        if let Some(map) = action_map {
                            self.process_yaml_group(map, path)?;
                        }
                    }
                    "node" => {
                        if let Some(map) = action_map {
                            self.process_yaml_node(map)?;
                        }
                    }
                    "let" => {
                        if let Some(map) = action_map {
                            self.process_yaml_let(map)?;
                        }
                    }
                    "set_env" | "set-env" => {
                        if let Some(map) = action_map {
                            self.process_yaml_set_env(map)?;
                        }
                    }
                    "unset_env" | "unset-env" => {
                        if let Some(map) = action_map {
                            self.process_yaml_unset_env(map)?;
                        }
                    }
                    "push-ros-namespace" | "push_ros_namespace" => {
                        if let Some(map) = action_map {
                            self.process_yaml_push_namespace(map)?;
                        }
                    }
                    "set_parameter" | "set-parameter" => {
                        if let Some(map) = action_map {
                            self.process_yaml_set_parameter(map)?;
                        }
                    }
                    "set_remap" | "set-remap" => {
                        if let Some(map) = action_map {
                            self.process_yaml_set_remap(map)?;
                        }
                    }
                    "executable" => {
                        if let Some(map) = action_map {
                            self.process_yaml_executable(map)?;
                        }
                    }
                    "node_container" | "node-container" => {
                        if let Some(map) = action_map {
                            self.process_yaml_node_container(map)?;
                        }
                    }
                    "load_composable_node" | "load-composable-node" => {
                        if let Some(map) = action_map {
                            self.process_yaml_load_composable_node(map)?;
                        }
                    }
                    other => {
                        log::warn!("Unsupported YAML action type: {}", other);
                    }
                }
            }
        }
        Ok(())
    }

    /// Check if/unless conditions on a YAML action mapping
    fn yaml_check_condition(&self, map: &Mapping) -> Result<bool> {
        if let Some(if_val) = yaml_str(map, "if") {
            let subs = parse_substitutions(if_val)?;
            let resolved = resolve_substitutions(&subs, &self.context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            if !is_truthy(&resolved) {
                return Ok(false);
            }
        }

        if let Some(unless_val) = yaml_str(map, "unless") {
            let subs = parse_substitutions(unless_val)?;
            let resolved = resolve_substitutions(&subs, &self.context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            if is_truthy(&resolved) {
                return Ok(false);
            }
        }

        Ok(true)
    }

    /// Process a YAML arg declaration
    fn process_yaml_arg(&mut self, map: &Mapping) -> Result<()> {
        let name = yaml_str(map, "name").ok_or_else(|| ParseError::MissingAttribute {
            element: "arg".to_string(),
            attribute: "name".to_string(),
        })?;
        let default_str = yaml_value_string(map, "default");
        let description = yaml_str(map, "description");

        log::debug!("[RUST] YAML declares arg: {} = {:?}", name, default_str);

        // Resolve default value substitutions if present
        let resolved_default = if let Some(ref default) = default_str {
            if let Ok(subs) = parse_substitutions(default) {
                resolve_substitutions(&subs, &self.context).ok()
            } else {
                Some(default.clone())
            }
        } else {
            None
        };

        self.context.declare_argument(ArgumentMetadata {
            name: name.to_string(),
            default: resolved_default.clone(),
            description: description.map(|s| s.to_string()),
            choices: None,
        });

        if let Some(resolved) = resolved_default
            && self.context.get_configuration(name).is_none()
        {
            log::debug!(
                "[RUST] YAML setting default value for {}: {}",
                name,
                resolved
            );
            self.context.set_configuration(name.to_string(), resolved);
        }

        Ok(())
    }

    /// Process a YAML include action
    fn process_yaml_include(&mut self, map: &Mapping) -> Result<()> {
        let file_str = yaml_str(map, "file").ok_or_else(|| ParseError::MissingAttribute {
            element: "include".to_string(),
            attribute: "file".to_string(),
        })?;
        let file = parse_substitutions(file_str)?;

        // Parse args list
        let mut args = Vec::new();
        if let Some(arg_list) = map
            .get(Value::String("arg".to_string()))
            .and_then(|v| v.as_sequence())
        {
            for arg_item in arg_list {
                if let Some(arg_map) = arg_item.as_mapping() {
                    let name =
                        yaml_str(arg_map, "name").ok_or_else(|| ParseError::MissingAttribute {
                            element: "include/arg".to_string(),
                            attribute: "name".to_string(),
                        })?;
                    let value_str = yaml_value_string(arg_map, "value").ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "include/arg".to_string(),
                            attribute: "value".to_string(),
                        }
                    })?;
                    let value = parse_substitutions(&value_str)?;
                    args.push((name.to_string(), value));
                }
            }
        }

        let include = IncludeAction { file, args };
        self.process_include(&include)
    }

    /// Process a YAML group action
    fn process_yaml_group(&mut self, map: &Mapping, path: &Path) -> Result<()> {
        let scope = self.context.save_scope();

        // Push namespace if specified
        if let Some(ns_str) = yaml_str(map, "ns") {
            let ns_subs = parse_substitutions(ns_str)?;
            let namespace = resolve_substitutions(&ns_subs, &self.context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            self.context.push_namespace(namespace);
        }

        // Process children
        let result = if let Some(children) = map
            .get(Value::String("children".to_string()))
            .and_then(|v| v.as_sequence())
        {
            self.process_yaml_actions(children, path)
        } else {
            Ok(())
        };

        self.context.restore_scope(scope);
        result
    }

    /// Process a YAML node action
    fn process_yaml_node(&mut self, map: &Mapping) -> Result<()> {
        let pkg_str = yaml_str(map, "pkg").ok_or_else(|| ParseError::MissingAttribute {
            element: "node".to_string(),
            attribute: "pkg".to_string(),
        })?;
        let exec_str = yaml_str(map, "exec").ok_or_else(|| ParseError::MissingAttribute {
            element: "node".to_string(),
            attribute: "exec".to_string(),
        })?;

        let package = parse_substitutions(pkg_str)?;
        let executable = parse_substitutions(exec_str)?;
        let name = yaml_str(map, "name").map(parse_substitutions).transpose()?;
        let namespace = yaml_str(map, "namespace")
            .map(parse_substitutions)
            .transpose()?;

        // Parse params
        let mut parameters = Vec::new();
        let mut param_files = Vec::new();
        if let Some(param_list) = map
            .get(Value::String("param".to_string()))
            .and_then(|v| v.as_sequence())
        {
            for param_item in param_list {
                if let Some(param_map) = param_item.as_mapping() {
                    if let Some(from_str) = yaml_str(param_map, "from") {
                        param_files.push(parse_substitutions(from_str)?);
                    } else if let (Some(pname), Some(pvalue)) = (
                        yaml_str(param_map, "name"),
                        yaml_value_string(param_map, "value"),
                    ) {
                        parameters.push(Parameter {
                            name: pname.to_string(),
                            value: parse_substitutions(&pvalue)?,
                        });
                    }
                }
            }
        }

        // Parse remappings
        let mut remappings = Vec::new();
        if let Some(remap_list) = map
            .get(Value::String("remap".to_string()))
            .and_then(|v| v.as_sequence())
        {
            for remap_item in remap_list {
                if let Some(remap_map) = remap_item.as_mapping()
                    && let (Some(from_str), Some(to_str)) =
                        (yaml_str(remap_map, "from"), yaml_str(remap_map, "to"))
                {
                    remappings.push(Remapping {
                        from: parse_substitutions(from_str)?,
                        to: parse_substitutions(to_str)?,
                    });
                }
            }
        }

        let args = yaml_str(map, "args").map(parse_substitutions).transpose()?;
        let output = yaml_str(map, "output").map(|s| s.to_string());
        let respawn = yaml_str(map, "respawn")
            .map(parse_substitutions)
            .transpose()?;
        let respawn_delay = yaml_str(map, "respawn_delay")
            .map(parse_substitutions)
            .transpose()?;

        let node = NodeAction {
            package,
            executable,
            name,
            namespace,
            parameters,
            param_files,
            param_sources: Vec::new(),
            remappings,
            environment: Vec::new(),
            args,
            output,
            respawn,
            respawn_delay,
        };

        let record = CommandGenerator::generate_node_record(&node, &self.context)
            .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
        self.records.push(record);
        Ok(())
    }

    /// Process a YAML let action
    fn process_yaml_let(&mut self, map: &Mapping) -> Result<()> {
        let name = yaml_str(map, "name").ok_or_else(|| ParseError::MissingAttribute {
            element: "let".to_string(),
            attribute: "name".to_string(),
        })?;
        let value =
            yaml_value_string(map, "value").ok_or_else(|| ParseError::MissingAttribute {
                element: "let".to_string(),
                attribute: "value".to_string(),
            })?;

        let resolved_value = if let Ok(subs) = parse_substitutions(&value) {
            resolve_substitutions(&subs, &self.context).unwrap_or_else(|e| {
                log::debug!(
                    "Could not resolve YAML let value for {}: {}, using raw value",
                    name,
                    e
                );
                value.clone()
            })
        } else {
            value
        };

        log::debug!("YAML setting {} = {} in context", name, resolved_value);
        self.context
            .set_configuration(name.to_string(), resolved_value);
        Ok(())
    }

    /// Process a YAML set_env action
    fn process_yaml_set_env(&mut self, map: &Mapping) -> Result<()> {
        let name = yaml_str(map, "name").ok_or_else(|| ParseError::MissingAttribute {
            element: "set_env".to_string(),
            attribute: "name".to_string(),
        })?;
        let value_str =
            yaml_value_string(map, "value").ok_or_else(|| ParseError::MissingAttribute {
                element: "set_env".to_string(),
                attribute: "value".to_string(),
            })?;

        let subs = parse_substitutions(&value_str)?;
        let value = resolve_substitutions(&subs, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        self.context
            .set_environment_variable(name.to_string(), value);
        Ok(())
    }

    /// Process a YAML unset_env action
    fn process_yaml_unset_env(&mut self, map: &Mapping) -> Result<()> {
        let name = yaml_str(map, "name").ok_or_else(|| ParseError::MissingAttribute {
            element: "unset_env".to_string(),
            attribute: "name".to_string(),
        })?;
        self.context.unset_environment_variable(name);
        Ok(())
    }

    /// Process a YAML push-ros-namespace action
    fn process_yaml_push_namespace(&mut self, map: &Mapping) -> Result<()> {
        let ns_str = yaml_str(map, "namespace")
            .or_else(|| yaml_str(map, "ns"))
            .ok_or_else(|| ParseError::MissingAttribute {
                element: "push-ros-namespace".to_string(),
                attribute: "namespace or ns".to_string(),
            })?;

        let ns_subs = parse_substitutions(ns_str)?;
        let namespace = resolve_substitutions(&ns_subs, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        self.context.push_namespace(namespace);
        Ok(())
    }

    /// Process a YAML set_parameter action
    fn process_yaml_set_parameter(&mut self, map: &Mapping) -> Result<()> {
        let name = yaml_str(map, "name").ok_or_else(|| ParseError::MissingAttribute {
            element: "set_parameter".to_string(),
            attribute: "name".to_string(),
        })?;
        let value_str =
            yaml_value_string(map, "value").ok_or_else(|| ParseError::MissingAttribute {
                element: "set_parameter".to_string(),
                attribute: "value".to_string(),
            })?;

        let subs = parse_substitutions(&value_str)?;
        let value = resolve_substitutions(&subs, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        self.context.set_global_parameter(name.to_string(), value);
        Ok(())
    }

    /// Process a YAML set_remap action
    fn process_yaml_set_remap(&mut self, map: &Mapping) -> Result<()> {
        let from_str = yaml_str(map, "from").ok_or_else(|| ParseError::MissingAttribute {
            element: "set_remap".to_string(),
            attribute: "from".to_string(),
        })?;
        let to_str = yaml_str(map, "to").ok_or_else(|| ParseError::MissingAttribute {
            element: "set_remap".to_string(),
            attribute: "to".to_string(),
        })?;

        let from_subs = parse_substitutions(from_str)?;
        let from = resolve_substitutions(&from_subs, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let to_subs = parse_substitutions(to_str)?;
        let to = resolve_substitutions(&to_subs, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        self.context.add_remapping(from, to);
        Ok(())
    }

    /// Process a YAML executable action
    fn process_yaml_executable(&mut self, map: &Mapping) -> Result<()> {
        let cmd_str = yaml_str(map, "cmd").ok_or_else(|| ParseError::MissingAttribute {
            element: "executable".to_string(),
            attribute: "cmd".to_string(),
        })?;
        let cmd = parse_substitutions(cmd_str)?;
        let cwd = yaml_str(map, "cwd").map(parse_substitutions).transpose()?;
        let name = yaml_str(map, "name").map(parse_substitutions).transpose()?;
        let shell = yaml_str(map, "shell").is_some_and(|s| s == "true");
        let output = yaml_str(map, "output").map(|s| s.to_string());

        // Parse env children
        let mut environment = Vec::new();
        if let Some(env_list) = map
            .get(Value::String("env".to_string()))
            .and_then(|v| v.as_sequence())
        {
            for env_item in env_list {
                if let Some(env_map) = env_item.as_mapping()
                    && let (Some(n), Some(v)) =
                        (yaml_str(env_map, "name"), yaml_str(env_map, "value"))
                {
                    environment.push((n.to_string(), v.to_string()));
                }
            }
        }

        // Parse arg children
        let mut arguments = Vec::new();
        if let Some(arg_list) = map
            .get(Value::String("arg".to_string()))
            .and_then(|v| v.as_sequence())
        {
            for arg_item in arg_list {
                if let Some(arg_map) = arg_item.as_mapping()
                    && let Some(val) = yaml_str(arg_map, "value")
                {
                    arguments.push(parse_substitutions(val)?);
                }
            }
        }

        let exec = ExecutableAction {
            cmd,
            cwd,
            name,
            shell,
            output,
            environment,
            arguments,
        };

        let record = CommandGenerator::generate_executable_record(&exec, &self.context)
            .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
        self.records.push(record);
        Ok(())
    }

    /// Process a YAML node_container action
    fn process_yaml_node_container(&mut self, map: &Mapping) -> Result<()> {
        let name_str = yaml_str(map, "name").ok_or_else(|| ParseError::MissingAttribute {
            element: "node_container".to_string(),
            attribute: "name".to_string(),
        })?;
        let name = parse_substitutions(name_str)?;

        let package = if let Some(pkg_str) = yaml_str(map, "pkg") {
            parse_substitutions(pkg_str)?
        } else {
            vec![Substitution::Text(DEFAULT_CONTAINER_PACKAGE.to_string())]
        };

        let executable = if let Some(exec_str) = yaml_str(map, "exec") {
            parse_substitutions(exec_str)?
        } else {
            vec![Substitution::Text(DEFAULT_CONTAINER_EXECUTABLE.to_string())]
        };

        let namespace = yaml_str(map, "namespace")
            .map(parse_substitutions)
            .transpose()?;
        let args = yaml_str(map, "args").map(parse_substitutions).transpose()?;

        // Parse composable_node children
        let composable_nodes = parse_yaml_composable_nodes(map, "composable_node", &self.context)?;

        let container = ContainerAction {
            name,
            namespace,
            package,
            executable,
            args,
            composable_nodes,
        };

        self.containers
            .push(container.to_container_record(&self.context)?);
        let load_node_records = container.to_load_node_records(&self.context)?;
        self.load_nodes.extend(load_node_records);
        Ok(())
    }

    /// Process a YAML load_composable_node action
    fn process_yaml_load_composable_node(&mut self, map: &Mapping) -> Result<()> {
        let target_str = yaml_str(map, "target").ok_or_else(|| ParseError::MissingAttribute {
            element: "load_composable_node".to_string(),
            attribute: "target".to_string(),
        })?;
        let target = parse_substitutions(target_str)?;

        let composable_nodes = parse_yaml_composable_nodes(map, "composable_node", &self.context)?;

        let action = LoadComposableNodeAction {
            target,
            composable_nodes,
        };

        for capture in action.to_captures(&self.context)? {
            self.context.capture_load_node(capture);
        }
        Ok(())
    }
}

/// Parse composable_node children from a YAML mapping
fn parse_yaml_composable_nodes(
    map: &Mapping,
    key: &str,
    context: &LaunchContext,
) -> Result<Vec<ComposableNodeAction>> {
    let mut nodes = Vec::new();
    if let Some(node_list) = map
        .get(Value::String(key.to_string()))
        .and_then(|v| v.as_sequence())
    {
        for node_item in node_list {
            if let Some(node_map) = node_item.as_mapping() {
                nodes.push(parse_yaml_composable_node(node_map, context)?);
            }
        }
    }
    Ok(nodes)
}

/// Parse a single composable_node from a YAML mapping
fn parse_yaml_composable_node(
    map: &Mapping,
    context: &LaunchContext,
) -> Result<ComposableNodeAction> {
    let pkg_str = yaml_str(map, "pkg").ok_or_else(|| ParseError::MissingAttribute {
        element: "composable_node".to_string(),
        attribute: "pkg".to_string(),
    })?;
    let plugin_str = yaml_str(map, "plugin").ok_or_else(|| ParseError::MissingAttribute {
        element: "composable_node".to_string(),
        attribute: "plugin".to_string(),
    })?;
    let name_str = yaml_str(map, "name").ok_or_else(|| ParseError::MissingAttribute {
        element: "composable_node".to_string(),
        attribute: "name".to_string(),
    })?;

    let package = parse_substitutions(pkg_str)?;
    let plugin = parse_substitutions(plugin_str)?;
    let name = parse_substitutions(name_str)?;
    let namespace = yaml_str(map, "namespace")
        .map(parse_substitutions)
        .transpose()?;

    // Parse params (resolved eagerly like XML)
    let mut parameters = Vec::new();
    if let Some(param_list) = map
        .get(Value::String("param".to_string()))
        .and_then(|v| v.as_sequence())
    {
        for param_item in param_list {
            if let Some(param_map) = param_item.as_mapping()
                && let (Some(pname), Some(pvalue)) = (
                    yaml_str(param_map, "name"),
                    yaml_value_string(param_map, "value"),
                )
            {
                let name_subs = parse_substitutions(pname)?;
                let value_subs = parse_substitutions(&pvalue)?;
                let name_resolved = resolve_substitutions(&name_subs, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                let value_resolved = resolve_substitutions(&value_subs, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                parameters.push((name_resolved, value_resolved));
            }
        }
    }

    // Parse remappings (resolved eagerly like XML)
    let mut remappings = Vec::new();
    if let Some(remap_list) = map
        .get(Value::String("remap".to_string()))
        .and_then(|v| v.as_sequence())
    {
        for remap_item in remap_list {
            if let Some(remap_map) = remap_item.as_mapping()
                && let (Some(from_str), Some(to_str)) =
                    (yaml_str(remap_map, "from"), yaml_str(remap_map, "to"))
            {
                let from_subs = parse_substitutions(from_str)?;
                let to_subs = parse_substitutions(to_str)?;
                let from_resolved = resolve_substitutions(&from_subs, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                let to_resolved = resolve_substitutions(&to_subs, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                remappings.push((from_resolved, to_resolved));
            }
        }
    }

    Ok(ComposableNodeAction {
        package,
        plugin,
        name,
        namespace,
        parameters,
        remappings,
        extra_args: HashMap::new(),
    })
}

/// Extract a string value from a YAML mapping by key
fn yaml_str<'a>(map: &'a Mapping, key: &str) -> Option<&'a str> {
    map.get(Value::String(key.to_string()))
        .and_then(|v| v.as_str())
}

/// Extract a value from a YAML mapping and convert to String (handles bool, number, string)
fn yaml_value_string(map: &Mapping, key: &str) -> Option<String> {
    map.get(Value::String(key.to_string()))
        .and_then(value_to_string)
}

/// Convert a YAML value to its string representation
fn value_to_string(value: &Value) -> Option<String> {
    match value {
        Value::String(s) => Some(s.clone()),
        Value::Bool(b) => Some(b.to_string()),
        Value::Number(n) => Some(n.to_string()),
        Value::Null => None,
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_yaml_str_extracts_string() {
        let mut map = Mapping::new();
        map.insert(
            Value::String("name".to_string()),
            Value::String("my_arg".to_string()),
        );
        assert_eq!(yaml_str(&map, "name"), Some("my_arg"));
        assert_eq!(yaml_str(&map, "missing"), None);
    }

    #[test]
    fn test_yaml_value_string_handles_types() {
        let mut map = Mapping::new();
        map.insert(
            Value::String("str_val".to_string()),
            Value::String("hello".to_string()),
        );
        map.insert(Value::String("bool_val".to_string()), Value::Bool(true));
        map.insert(
            Value::String("int_val".to_string()),
            Value::Number(serde_yaml_ng::Number::from(42)),
        );

        assert_eq!(
            yaml_value_string(&map, "str_val"),
            Some("hello".to_string())
        );
        assert_eq!(
            yaml_value_string(&map, "bool_val"),
            Some("true".to_string())
        );
        assert_eq!(yaml_value_string(&map, "int_val"), Some("42".to_string()));
        assert_eq!(yaml_value_string(&map, "missing"), None);
    }

    #[test]
    fn test_value_to_string_null() {
        assert_eq!(value_to_string(&Value::Null), None);
    }

    #[test]
    fn test_value_to_string_sequence() {
        let seq = Value::Sequence(vec![Value::String("a".to_string())]);
        assert_eq!(value_to_string(&seq), None);
    }
}
