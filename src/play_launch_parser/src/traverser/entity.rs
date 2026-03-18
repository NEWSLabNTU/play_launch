use super::super::LaunchTraverser;
use crate::{
    actions::{
        ArgAction, ContainerAction, DeclareArgumentAction, ExecutableAction, GroupAction,
        IncludeAction, LetAction, LoadComposableNodeAction, NodeAction, SetEnvAction,
        SetParameterAction, SetRemapAction, UnsetEnvAction,
    },
    condition::should_process_entity,
    error::{ParseError, Result},
    record::CommandGenerator,
    substitution::{ArgumentMetadata, parse_substitutions, resolve_substitutions},
    xml::{Entity, XmlEntity},
};
use std::collections::HashMap;

impl LaunchTraverser {
    pub(crate) fn traverse_entity(&mut self, entity: &XmlEntity) -> Result<()> {
        // Check if entity should be processed based on if/unless conditions
        if !should_process_entity(entity, &self.context)? {
            log::debug!("Skipping {} due to condition", entity.type_name());
            return Ok(());
        }

        match entity.type_name() {
            "launch" => {
                for child in entity.children() {
                    self.traverse_entity(&child)?;
                }
            }
            "arg" => {
                let arg = ArgAction::from_entity(entity)?;
                arg.apply(&mut self.context, &HashMap::new());
            }
            "declare_argument" => {
                let declare_arg = DeclareArgumentAction::from_entity(entity)?;

                // Resolve default value if present
                let default = if let Some(default_subs) = &declare_arg.default {
                    Some(
                        resolve_substitutions(default_subs, &self.context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?,
                    )
                } else {
                    None
                };

                log::debug!(
                    "[RUST] Declaring argument: {} = {:?}",
                    declare_arg.name,
                    default
                );

                // Store metadata
                let metadata = ArgumentMetadata {
                    name: declare_arg.name.clone(),
                    default,
                    description: declare_arg.description.clone(),
                    choices: declare_arg.choices.clone(),
                };
                self.context.declare_argument(metadata);

                // If a default value is provided and the argument is not yet set, apply it
                if let Some(default_val) = &declare_arg.default
                    && self.context.get_configuration(&declare_arg.name).is_none()
                {
                    let resolved_default = resolve_substitutions(default_val, &self.context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    log::debug!(
                        "[RUST] Setting default value for {}: {}",
                        declare_arg.name,
                        resolved_default
                    );
                    self.context
                        .set_configuration(declare_arg.name, resolved_default);
                }
            }
            "node" => {
                // XML nodes use CommandGenerator for complete parameter file loading
                let node = NodeAction::from_entity(entity)?;
                let mut record = CommandGenerator::generate_node_record(&node, &self.context)
                    .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
                record.scope = Some(self.current_scope_id);
                self.records.push(record);
            }
            "executable" => {
                // Executables don't use NodeCapture - they generate NodeRecord directly
                let exec = ExecutableAction::from_entity(entity)?;
                let mut record = CommandGenerator::generate_executable_record(&exec, &self.context)
                    .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
                record.scope = Some(self.current_scope_id);
                self.records.push(record);
            }
            "include" => {
                let include = IncludeAction::from_entity(entity)?;
                self.process_include(&include)?;
            }
            "group" => {
                let group = GroupAction::from_entity(entity)?;

                // Save scope state — restored after group regardless of success/failure
                let scope = self.context.save_scope();

                // Push namespace if specified
                if let Some(ns_subs) = &group.namespace {
                    let namespace = resolve_substitutions(ns_subs, &self.context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    self.context.push_namespace(namespace);
                }

                // Traverse children, then always restore scope
                let result = entity
                    .children()
                    .try_for_each(|child| self.traverse_entity(&child));
                self.context.restore_scope(scope);
                result?;
            }
            "let" => {
                let let_action = LetAction::from_entity(entity)?;
                // Parse and resolve substitutions in the value (e.g., $(eval ...), $(var ...))
                // Fall back to raw value if resolution fails (e.g., missing packages)
                let resolved_value = if let Ok(value_subs) = parse_substitutions(&let_action.value)
                {
                    resolve_substitutions(&value_subs, &self.context).unwrap_or_else(|e| {
                        log::debug!(
                            "Could not resolve <let> value for {}: {}, using raw value",
                            let_action.name,
                            e
                        );
                        let_action.value.clone()
                    })
                } else {
                    let_action.value.clone()
                };
                log::debug!(
                    "Setting {} = {} in context",
                    let_action.name,
                    resolved_value
                );
                // Set resolved variable in context (acts like arg)
                self.context
                    .set_configuration(let_action.name, resolved_value);
            }
            "set_env" | "set-env" => {
                let set_env = SetEnvAction::from_entity(entity)?;
                let value = resolve_substitutions(&set_env.value, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.set_environment_variable(set_env.name, value);
            }
            "unset_env" | "unset-env" => {
                let unset_env = UnsetEnvAction::from_entity(entity)?;
                self.context.unset_environment_variable(&unset_env.name);
            }
            "set_parameter" => {
                let set_param = SetParameterAction::from_entity(entity)?;
                let value = resolve_substitutions(&set_param.value, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.set_global_parameter(set_param.name, value);
            }
            "set_remap" | "set-remap" => {
                let set_remap = SetRemapAction::from_entity(entity)?;
                let from = resolve_substitutions(&set_remap.from, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                let to = resolve_substitutions(&set_remap.to, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.add_remapping(from, to);
            }
            "push-ros-namespace" => {
                // Try "namespace" attribute first (Autoware/ROS 2 standard),
                // then fall back to "ns" for backwards compatibility
                let ns_str = entity
                    .required_attr_str("namespace")
                    .ok()
                    .flatten()
                    .or_else(|| entity.required_attr_str("ns").ok().flatten())
                    .ok_or_else(|| ParseError::MissingAttribute {
                        element: "push-ros-namespace".to_string(),
                        attribute: "namespace or ns".to_string(),
                    })?;

                let ns_subs = parse_substitutions(&ns_str)?;
                let namespace = resolve_substitutions(&ns_subs, &self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

                self.context.push_namespace(namespace);
            }
            "pop-ros-namespace" => {
                self.context.pop_namespace();
            }
            "node_container" | "node-container" => {
                // Parse container and its composable nodes
                let container_action = ContainerAction::from_entity(entity, &self.context)?;

                // Add container record
                let mut container_rec = container_action.to_container_record(&self.context)?;
                container_rec.scope = Some(self.current_scope_id);
                self.containers.push(container_rec);

                // Add load_node records for each composable node
                let mut load_node_records = container_action.to_load_node_records(&self.context)?;
                for rec in &mut load_node_records {
                    rec.scope = Some(self.current_scope_id);
                }
                self.load_nodes.extend(load_node_records);

                log::debug!(
                    "Parsed container with {} composable nodes",
                    container_action.composable_nodes.len()
                );
            }
            "composable_node" | "composable-node" => {
                // Composable nodes are loaded into containers
                // Standalone composable nodes should be inside a node_container
                log::debug!("Skipping standalone composable_node (should be in node_container)");
            }
            "load_composable_node" | "load-composable-node" => {
                // Parse load_composable_node action and convert to captures
                let action = LoadComposableNodeAction::from_entity(entity, &self.context)?;
                let captures = action.to_captures(&self.context)?;

                log::debug!(
                    "Loaded {} composable node(s) into container via load_composable_node",
                    captures.len()
                );

                // Add to context captures
                for capture in captures {
                    self.context.capture_load_node(capture);
                }
            }
            other => {
                log::warn!("Unsupported action type: {}", other);
                // For MVP: skip unknown actions
            }
        }
        Ok(())
    }
}
