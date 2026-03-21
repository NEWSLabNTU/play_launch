//! IR builder — constructs a `LaunchProgram` tree from XML without evaluating conditions.

use super::super::LaunchTraverser;
use crate::{
    actions::{
        ContainerAction, DeclareArgumentAction, ExecutableAction, GroupAction, IncludeAction,
        LetAction, LoadComposableNodeAction, NodeAction, SetEnvAction, SetParameterAction,
        SetRemapAction, UnsetEnvAction,
    },
    error::{ParseError, Result},
    file_cache::read_file_cached,
    ir::{Action, ActionKind, Condition, Expr, LaunchProgram, Span},
    substitution::{ArgumentMetadata, parse_substitutions, resolve_substitutions},
    xml::{Entity, XmlEntity},
};
use std::path::Path;

impl LaunchTraverser {
    /// Build an IR `LaunchProgram` from a file, dispatching by extension.
    pub fn build_ir_file(&mut self, path: &Path) -> Result<LaunchProgram> {
        let source = path.to_path_buf();

        // Check file extension
        if let Some(ext) = path.extension().and_then(|s| s.to_str()) {
            match ext {
                "py" => {
                    // Python files produce an OpaqueFunction placeholder
                    let action = Action {
                        kind: ActionKind::OpaqueFunction {
                            description: format!("Python launch file: {}", path.display()),
                        },
                        condition: None,
                        span: Some(Span {
                            file: source.clone(),
                            line: 1,
                        }),
                    };
                    return Ok(LaunchProgram {
                        source,
                        body: vec![action],
                    });
                }
                "yaml" | "yml" => {
                    return self.build_ir_yaml(path);
                }
                _ => {}
            }
        }

        // XML: parse and build IR
        self.context.set_current_file(path.to_path_buf());
        let content = read_file_cached(path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = crate::xml::XmlEntity::new(doc.root_element());
        let body = self.build_ir_entity(&root, &source)?;

        Ok(LaunchProgram { source, body })
    }

    /// Build IR from a YAML launch file.
    ///
    /// YAML launch files in ROS 2 are primarily used as preset files that declare
    /// arguments with default values. Unlike XML includes which use isolated scope,
    /// YAML includes modify the parent scope — variables they declare become visible
    /// to subsequent includes in the same file.
    fn build_ir_yaml(&mut self, path: &Path) -> Result<LaunchProgram> {
        use serde_yml::Value;

        let source = path.to_path_buf();
        let content = read_file_cached(path)?;
        let yaml: Value = serde_yml::from_str(&content)
            .map_err(|e| ParseError::InvalidSubstitution(format!("Invalid YAML: {}", e)))?;

        let mut body = Vec::new();

        if let Some(launch_list) = yaml.get("launch").and_then(|v| v.as_sequence()) {
            for item in launch_list {
                if let Some(arg_map) = item.get("arg").and_then(|v| v.as_mapping())
                    && let Some(name) = arg_map
                        .get(Value::String("name".to_string()))
                        .and_then(|v| v.as_str())
                {
                    let default_value = arg_map
                        .get(Value::String("default".to_string()))
                        .and_then(|v| v.as_str());
                    let description = arg_map
                        .get(Value::String("description".to_string()))
                        .and_then(|v| v.as_str())
                        .map(|s| s.to_string());

                    let default_expr = default_value
                        .map(|d| parse_substitutions(d).map(Expr::new))
                        .transpose()?;

                    body.push(Action {
                        kind: ActionKind::DeclareArgument {
                            name: name.to_string(),
                            default: default_expr,
                            description,
                            choices: None,
                        },
                        condition: None,
                        span: Some(Span {
                            file: source.clone(),
                            line: 1,
                        }),
                    });

                    // Apply to context — YAML presets modify parent scope so
                    // later substitutions (e.g. in include file paths) can resolve.
                    self.context.declare_argument(ArgumentMetadata {
                        name: name.to_string(),
                        default: default_value.map(|s| s.to_string()),
                        description: None,
                        choices: None,
                    });
                    if let Some(default) = default_value
                        && self.context.get_configuration(name).is_none()
                    {
                        self.context
                            .set_configuration(name.to_string(), default.to_string());
                    }
                }
            }
        }

        Ok(LaunchProgram { source, body })
    }

    /// Build IR actions from a single XML entity and its children.
    ///
    /// Unlike `traverse_entity()`, this method:
    /// - Stores `if`/`unless` as `Condition` instead of evaluating
    /// - Processes ALL conditional branches (never skips)
    /// - Returns `Vec<Action>` rather than mutating `self.records`
    fn build_ir_entity(&mut self, entity: &XmlEntity, current_file: &Path) -> Result<Vec<Action>> {
        let mut actions = Vec::new();

        match entity.type_name() {
            "launch" => {
                for child in entity.children() {
                    actions.extend(self.build_ir_entity(&child, current_file)?);
                }
            }

            "arg" => {
                // <arg> sets a variable — still applied to context so later
                // substitutions (e.g. in include file paths) can resolve.
                let arg = crate::actions::ArgAction::from_entity(entity)?;
                arg.apply(&mut self.context, &std::collections::HashMap::new());

                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);

                // <arg> with a default is functionally a DeclareArgument
                actions.push(Action {
                    kind: ActionKind::DeclareArgument {
                        name: arg.name.clone(),
                        default: arg
                            .default
                            .as_ref()
                            .map(|d| parse_substitutions(d))
                            .transpose()?
                            .map(Expr::new),
                        description: arg.description.clone(),
                        choices: None,
                    },
                    condition,
                    span,
                });
            }

            "declare_argument" => {
                let declare_arg = DeclareArgumentAction::from_entity(entity)?;

                // Apply to context so include paths can resolve
                if let Some(ref default_subs) = declare_arg.default
                    && self.context.get_configuration(&declare_arg.name).is_none()
                    && let Ok(resolved) = resolve_substitutions(default_subs, &self.context)
                {
                    self.context
                        .set_configuration(declare_arg.name.clone(), resolved);
                }
                // Also record metadata
                let metadata = ArgumentMetadata {
                    name: declare_arg.name.clone(),
                    default: declare_arg
                        .default
                        .as_ref()
                        .and_then(|d| resolve_substitutions(d, &self.context).ok()),
                    description: declare_arg.description.clone(),
                    choices: declare_arg.choices.clone(),
                };
                self.context.declare_argument(metadata);

                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: ActionKind::DeclareArgument {
                        name: declare_arg.name,
                        default: declare_arg.default.map(Expr::new),
                        description: declare_arg.description,
                        choices: declare_arg.choices,
                    },
                    condition,
                    span,
                });
            }

            "let" => {
                let let_action = LetAction::from_entity(entity)?;

                // Apply to context so include paths can resolve
                if let Ok(value_subs) = parse_substitutions(&let_action.value)
                    && let Ok(resolved) = resolve_substitutions(&value_subs, &self.context)
                {
                    self.context
                        .set_configuration(let_action.name.clone(), resolved);
                }

                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                let value_subs = parse_substitutions(&let_action.value)?;
                actions.push(Action {
                    kind: ActionKind::SetVariable {
                        name: let_action.name,
                        value: Expr::new(value_subs),
                    },
                    condition,
                    span,
                });
            }

            "node" => {
                let node = NodeAction::from_entity(entity)?;
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: node.into(),
                    condition,
                    span,
                });
            }

            "executable" => {
                let exec = ExecutableAction::from_entity(entity)?;
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: exec.into(),
                    condition,
                    span,
                });
            }

            "include" => {
                let include = IncludeAction::from_entity(entity)?;
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                let body = self.build_ir_include(&include, current_file);
                actions.push(Action {
                    kind: ActionKind::Include {
                        file: Expr::new(include.file),
                        args: include
                            .args
                            .into_iter()
                            .map(|(name, value)| crate::ir::IncludeArg {
                                name,
                                value: Expr::new(value),
                            })
                            .collect(),
                        body,
                    },
                    condition,
                    span,
                });
            }

            "group" => {
                let group = GroupAction::from_entity(entity)?;
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);

                // Save scope, push namespace, build children, restore scope
                let scope = self.context.save_scope();
                if let Some(ref ns_subs) = group.namespace
                    && let Ok(namespace) = resolve_substitutions(ns_subs, &self.context)
                {
                    self.context.push_namespace(namespace);
                }
                let mut body = Vec::new();
                for child in entity.children() {
                    body.extend(self.build_ir_entity(&child, current_file)?);
                }
                self.context.restore_scope(scope);

                actions.push(Action {
                    kind: ActionKind::Group {
                        namespace: group.namespace.map(Expr::new),
                        body,
                    },
                    condition,
                    span,
                });
            }

            "set_env" | "set-env" => {
                let set_env = SetEnvAction::from_entity(entity)?;
                // Apply to context
                if let Ok(value) = resolve_substitutions(&set_env.value, &self.context) {
                    self.context
                        .set_environment_variable(set_env.name.clone(), value);
                }
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: set_env.into(),
                    condition,
                    span,
                });
            }

            "unset_env" | "unset-env" => {
                let unset_env = UnsetEnvAction::from_entity(entity)?;
                self.context.unset_environment_variable(&unset_env.name);
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: unset_env.into(),
                    condition,
                    span,
                });
            }

            "set_parameter" => {
                let set_param = SetParameterAction::from_entity(entity)?;
                if let Ok(value) = resolve_substitutions(&set_param.value, &self.context) {
                    self.context
                        .set_global_parameter(set_param.name.clone(), value);
                }
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: set_param.into(),
                    condition,
                    span,
                });
            }

            "set_remap" | "set-remap" => {
                let set_remap = SetRemapAction::from_entity(entity)?;
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: set_remap.into(),
                    condition,
                    span,
                });
            }

            "push-ros-namespace" => {
                let ns_str = entity
                    .optional_attr_str("namespace")?
                    .or_else(|| entity.optional_attr_str("ns").ok().flatten())
                    .ok_or_else(|| ParseError::MissingAttribute {
                        element: "push-ros-namespace".to_string(),
                        attribute: "namespace or ns".to_string(),
                    })?;
                let ns_subs = parse_substitutions(&ns_str)?;

                // Apply to context
                if let Ok(namespace) = resolve_substitutions(&ns_subs, &self.context) {
                    self.context.push_namespace(namespace);
                }

                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: ActionKind::PushNamespace {
                        namespace: Expr::new(ns_subs),
                    },
                    condition,
                    span,
                });
            }

            "pop-ros-namespace" => {
                self.context.pop_namespace();
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: ActionKind::PopNamespace,
                    condition,
                    span,
                });
            }

            "node_container" | "node-container" => {
                let container = ContainerAction::from_entity(entity, &self.context)?;
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: container.into(),
                    condition,
                    span,
                });
            }

            "composable_node" | "composable-node" => {
                log::debug!(
                    "Skipping standalone composable_node in IR (should be in node_container)"
                );
            }

            "load_composable_node" | "load-composable-node" => {
                let action = LoadComposableNodeAction::from_entity(entity, &self.context)?;
                let condition = extract_condition(entity)?;
                let span = make_span(entity, current_file);
                actions.push(Action {
                    kind: action.into(),
                    condition,
                    span,
                });
            }

            other => {
                log::debug!("Skipping unsupported action type in IR builder: {}", other);
            }
        }

        Ok(actions)
    }

    /// Build IR for an included file, returning `Some(Box<LaunchProgram>)` on success,
    /// or `None` if the file cannot be resolved/parsed.
    fn build_ir_include(
        &mut self,
        include: &IncludeAction,
        current_file: &Path,
    ) -> Option<Box<LaunchProgram>> {
        // Try to resolve the file path
        let file_path_str = match resolve_substitutions(&include.file, &self.context) {
            Ok(s) => s,
            Err(e) => {
                log::debug!("IR: cannot resolve include path: {}", e);
                return None;
            }
        };
        let file_path = Path::new(&file_path_str);

        // Resolve relative paths
        let resolved_path = if file_path.is_relative() {
            if let Some(parent) = current_file.parent() {
                parent.join(file_path)
            } else {
                file_path.to_path_buf()
            }
        } else {
            file_path.to_path_buf()
        };

        // Canonicalize for circular detection
        let canonical_path = resolved_path
            .canonicalize()
            .unwrap_or_else(|_| resolved_path.clone());

        if self.include_chain.contains(&canonical_path) {
            log::debug!(
                "IR: circular include detected: {}",
                canonical_path.display()
            );
            return None;
        }

        // YAML includes modify parent scope — process directly on self
        let is_yaml = resolved_path
            .extension()
            .and_then(|e| e.to_str())
            .map(|e| matches!(e, "yaml" | "yml"))
            .unwrap_or(false);

        if is_yaml {
            match self.build_ir_yaml(&resolved_path) {
                Ok(program) => return Some(Box::new(program)),
                Err(e) => {
                    log::debug!(
                        "IR: failed to build YAML IR for {}: {}",
                        resolved_path.display(),
                        e
                    );
                    return None;
                }
            }
        }

        // Apply include args to a child context (XML includes use isolated scope)
        let mut child_context = self.context.child();
        child_context.set_current_file(resolved_path.clone());
        for (key, value_subs) in &include.args {
            if let Ok(resolved_value) = resolve_substitutions(value_subs, &child_context) {
                child_context.set_configuration(key.clone(), resolved_value);
            }
        }

        // Build IR for the included file
        let mut child_chain = self.include_chain.clone();
        child_chain.push(canonical_path);

        let mut child_traverser = LaunchTraverser {
            context: child_context,
            include_chain: child_chain,
            max_include_depth: self.max_include_depth,
            records: Vec::new(),
            containers: Vec::new(),
            load_nodes: Vec::new(),
            scope_table: crate::record::ScopeTable::new(),
            current_scope_id: 0,
        };

        match child_traverser.build_ir_file(&resolved_path) {
            Ok(program) => Some(Box::new(program)),
            Err(e) => {
                log::debug!(
                    "IR: failed to build IR for included file {}: {}",
                    resolved_path.display(),
                    e
                );
                None
            }
        }
    }
}

/// Extract `if`/`unless` condition from an XML entity without evaluating it.
fn extract_condition(entity: &XmlEntity) -> Result<Option<Condition>> {
    if let Some(if_str) = entity.optional_attr_str("if")? {
        let subs = parse_substitutions(&if_str)?;
        return Ok(Some(Condition::If(Expr::new(subs))));
    }
    if let Some(unless_str) = entity.optional_attr_str("unless")? {
        let subs = parse_substitutions(&unless_str)?;
        return Ok(Some(Condition::Unless(Expr::new(subs))));
    }
    Ok(None)
}

/// Build a `Span` from an XML entity's position in the source file.
fn make_span(entity: &XmlEntity, current_file: &Path) -> Option<Span> {
    Some(Span {
        file: current_file.to_path_buf(),
        line: entity.line_number(),
    })
}
