//! IR evaluator — walks a `LaunchProgram` tree, resolves expressions, and produces records.

use super::super::LaunchTraverser;
use crate::{
    actions::{
        ComposableNodeAction, ContainerAction, ExecutableAction, LoadComposableNodeAction,
        NodeAction,
    },
    condition::is_truthy,
    error::{ParseError, Result},
    ir::{
        Action, ActionKind, ComposableNodeDecl, Condition, EnvDecl, Expr, LaunchProgram, RemapDecl,
    },
    record::CommandGenerator,
};

impl LaunchTraverser {
    /// Evaluate an IR `LaunchProgram`, resolving expressions and populating records.
    pub fn evaluate_ir(&mut self, program: &LaunchProgram) -> Result<()> {
        for action in &program.body {
            self.evaluate_action(action)?;
        }
        Ok(())
    }

    fn evaluate_action(&mut self, action: &Action) -> Result<()> {
        // Check condition
        if let Some(ref condition) = action.condition
            && !self.evaluate_condition_expr(condition)?
        {
            return Ok(());
        }

        match &action.kind {
            ActionKind::DeclareArgument {
                name,
                default,
                description,
                choices,
            } => {
                // Resolve default value if present
                let resolved_default = if let Some(default_expr) = default {
                    Some(
                        default_expr
                            .resolve(&self.context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?,
                    )
                } else {
                    None
                };

                // Store metadata
                let metadata = crate::substitution::ArgumentMetadata {
                    name: name.clone(),
                    default: resolved_default,
                    description: description.clone(),
                    choices: choices.clone(),
                };
                self.context.declare_argument(metadata);

                // If default present and arg not yet set, apply it
                if let Some(default_expr) = default
                    && self.context.get_configuration(name).is_none()
                {
                    let resolved = default_expr
                        .resolve(&self.context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    self.context.set_configuration(name.clone(), resolved);
                }
            }

            ActionKind::SetVariable { name, value } => {
                let resolved = value
                    .resolve(&self.context)
                    .unwrap_or_else(|_| String::new());
                self.context.set_configuration(name.clone(), resolved);
            }

            ActionKind::SetEnv { name, value } => {
                let resolved = value
                    .resolve(&self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context
                    .set_environment_variable(name.clone(), resolved);
            }

            ActionKind::UnsetEnv { name } => {
                self.context.unset_environment_variable(name);
            }

            ActionKind::PushNamespace { namespace } => {
                let resolved = namespace
                    .resolve(&self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.push_namespace(resolved);
            }

            ActionKind::PopNamespace => {
                self.context.pop_namespace();
            }

            ActionKind::SetParameter { name, value } => {
                let resolved = value
                    .resolve(&self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.set_global_parameter(name.clone(), resolved);
            }

            ActionKind::SetRemap { from, to } => {
                let from_resolved = from
                    .resolve(&self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                let to_resolved = to
                    .resolve(&self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.add_remapping(from_resolved, to_resolved);
            }

            ActionKind::Group { namespace, body } => {
                let scope = self.context.save_scope();

                if let Some(ns_expr) = namespace {
                    let ns = ns_expr
                        .resolve(&self.context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    self.context.push_namespace(ns);
                }

                let result = body.iter().try_for_each(|a| self.evaluate_action(a));
                self.context.restore_scope(scope);
                result?;
            }

            ActionKind::Include { file, args, body } => {
                self.evaluate_include(file, args, body.as_deref())?;
            }

            ActionKind::SpawnNode { .. } => {
                let node_action = ir_to_node_action(&action.kind);
                let record = CommandGenerator::generate_node_record(&node_action, &self.context)
                    .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
                self.records.push(record);
            }

            ActionKind::SpawnExecutable { .. } => {
                let exec_action = ir_to_executable_action(&action.kind);
                let record =
                    CommandGenerator::generate_executable_record(&exec_action, &self.context)
                        .map_err(|e| ParseError::IoError(std::io::Error::other(e.to_string())))?;
                self.records.push(record);
            }

            ActionKind::SpawnContainer { .. } => {
                let container_action = ir_to_container_action(&action.kind, &self.context);
                self.containers
                    .push(container_action.to_container_record(&self.context)?);
                let load_records = container_action.to_load_node_records(&self.context)?;
                self.load_nodes.extend(load_records);
            }

            ActionKind::LoadComposableNode { .. } => {
                let load_action = ir_to_load_composable_node_action(&action.kind, &self.context);
                let records = load_action.to_load_node_records(&self.context)?;
                self.load_nodes.extend(records);
            }

            ActionKind::OpaqueFunction { description } => {
                log::debug!("Skipping OpaqueFunction during evaluation: {}", description);
            }
        }

        Ok(())
    }

    fn evaluate_condition_expr(&self, condition: &Condition) -> Result<bool> {
        match condition {
            Condition::If(expr) => {
                let resolved = expr
                    .resolve(&self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                Ok(is_truthy(&resolved))
            }
            Condition::Unless(expr) => {
                let resolved = expr
                    .resolve(&self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                Ok(!is_truthy(&resolved))
            }
        }
    }

    fn evaluate_include(
        &mut self,
        file: &Expr,
        args: &[crate::ir::IncludeArg],
        body: Option<&LaunchProgram>,
    ) -> Result<()> {
        // If body is None (Python include or unresolved), skip
        let body = match body {
            Some(b) => b,
            None => {
                let file_str = file
                    .resolve(&self.context)
                    .unwrap_or_else(|_| "<unresolved>".to_string());
                log::debug!(
                    "Skipping include with no IR body (Python/unresolved): {}",
                    file_str
                );
                return Ok(());
            }
        };

        // YAML includes modify parent scope directly (no child context).
        // This matches ROS 2 behavior where YAML preset files set variables
        // visible to subsequent includes in the same file.
        let is_yaml = body
            .source
            .extension()
            .and_then(|e| e.to_str())
            .map(|e| matches!(e, "yaml" | "yml"))
            .unwrap_or(false);

        if is_yaml {
            for arg in args {
                let resolved = arg
                    .value
                    .resolve(&self.context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                self.context.set_configuration(arg.name.clone(), resolved);
            }
            for action in &body.body {
                self.evaluate_action(action)?;
            }
            return Ok(());
        }

        // Create child context (XML includes use isolated scope)
        let mut include_context = self.context.child();
        include_context.set_current_file(body.source.clone());

        // Apply include args (resolve in child context for forward references)
        for arg in args {
            let resolved = arg
                .value
                .resolve(&include_context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            include_context.set_configuration(arg.name.clone(), resolved);
        }

        // Create child traverser
        let mut child_traverser = LaunchTraverser {
            context: include_context,
            include_chain: self.include_chain.clone(),
            max_include_depth: self.max_include_depth,
            strict_includes: self.strict_includes,
            records: Vec::new(),
            containers: Vec::new(),
            load_nodes: Vec::new(),
            scope_table: std::mem::take(&mut self.scope_table),
            current_scope_id: self.current_scope_id,
        };

        child_traverser.evaluate_ir(body)?;

        // Take back the scope table
        self.scope_table = std::mem::take(&mut child_traverser.scope_table);

        // Merge records
        self.records.extend(child_traverser.records);
        self.containers.extend(child_traverser.containers);
        self.load_nodes.extend(child_traverser.load_nodes);

        // Merge captures
        for node in child_traverser.context.captured_nodes() {
            self.context.capture_node(node.clone());
        }
        for container in child_traverser.context.captured_containers() {
            self.context.capture_container(container.clone());
        }
        for load_node in child_traverser.context.captured_load_nodes() {
            self.context.capture_load_node(load_node.clone());
        }

        // Propagate global parameters back
        for (key, value) in child_traverser.context.global_parameters() {
            self.context.set_global_parameter(key, value);
        }

        Ok(())
    }
}

// --- IR → Action conversion helpers ---

fn ir_to_node_action(kind: &ActionKind) -> NodeAction {
    match kind {
        ActionKind::SpawnNode {
            package,
            executable,
            name,
            namespace,
            params,
            param_files,
            remaps,
            env,
            args,
            respawn,
            respawn_delay,
        } => NodeAction {
            package: package.parts.clone(),
            executable: executable.parts.clone(),
            name: name.as_ref().map(|n| n.parts.clone()),
            namespace: namespace.as_ref().map(|n| n.parts.clone()),
            parameters: params
                .iter()
                .map(|p| crate::actions::node::Parameter {
                    name: p.name.clone(),
                    value: p.value.parts.clone(),
                })
                .collect(),
            param_files: param_files.iter().map(|pf| pf.parts.clone()).collect(),
            param_sources: Vec::new(),
            remappings: remaps_to_action(remaps),
            environment: env_decls_to_pairs(env),
            args: args.as_ref().map(|a| a.parts.clone()),
            output: None,
            respawn: respawn.as_ref().map(|r| r.parts.clone()),
            respawn_delay: respawn_delay.as_ref().map(|r| r.parts.clone()),
        },
        _ => unreachable!("ir_to_node_action called with non-SpawnNode"),
    }
}

fn ir_to_executable_action(kind: &ActionKind) -> ExecutableAction {
    match kind {
        ActionKind::SpawnExecutable {
            cmd,
            name,
            args,
            env,
        } => ExecutableAction {
            cmd: cmd.parts.clone(),
            cwd: None,
            name: name.as_ref().map(|n| n.parts.clone()),
            shell: false,
            output: None,
            environment: env_decls_to_pairs(env),
            arguments: args.iter().map(|a| a.parts.clone()).collect(),
        },
        _ => unreachable!("ir_to_executable_action called with non-SpawnExecutable"),
    }
}

fn ir_to_container_action(
    kind: &ActionKind,
    context: &crate::substitution::LaunchContext,
) -> ContainerAction {
    match kind {
        ActionKind::SpawnContainer {
            package,
            executable,
            name,
            namespace,
            args,
            nodes,
        } => ContainerAction {
            package: package.parts.clone(),
            executable: executable.parts.clone(),
            name: name.parts.clone(),
            namespace: namespace.as_ref().map(|n| n.parts.clone()),
            args: args.as_ref().map(|a| a.parts.clone()),
            composable_nodes: nodes
                .iter()
                .map(|n| composable_decl_to_action(n, context))
                .collect(),
        },
        _ => unreachable!("ir_to_container_action called with non-SpawnContainer"),
    }
}

fn ir_to_load_composable_node_action(
    kind: &ActionKind,
    context: &crate::substitution::LaunchContext,
) -> LoadComposableNodeAction {
    match kind {
        ActionKind::LoadComposableNode { target, nodes } => LoadComposableNodeAction {
            target: target.parts.clone(),
            composable_nodes: nodes
                .iter()
                .map(|n| composable_decl_to_action(n, context))
                .collect(),
        },
        _ => unreachable!("ir_to_load_composable_node_action called with non-LoadComposableNode"),
    }
}

fn composable_decl_to_action(
    decl: &ComposableNodeDecl,
    context: &crate::substitution::LaunchContext,
) -> ComposableNodeAction {
    // ComposableNodeAction params/remaps are (String, String) — resolve Expr fields
    let parameters: Vec<(String, String)> = decl
        .params
        .iter()
        .map(|p| {
            let value = p
                .value
                .resolve(context)
                .unwrap_or_else(|_| p.value.as_literal().unwrap_or("").to_string());
            (p.name.clone(), value)
        })
        .collect();

    let remappings: Vec<(String, String)> = decl
        .remaps
        .iter()
        .map(|r| {
            let from = r
                .from
                .resolve(context)
                .unwrap_or_else(|_| r.from.as_literal().unwrap_or("").to_string());
            let to =
                r.to.resolve(context)
                    .unwrap_or_else(|_| r.to.as_literal().unwrap_or("").to_string());
            (from, to)
        })
        .collect();

    ComposableNodeAction {
        package: decl.package.parts.clone(),
        plugin: decl.plugin.parts.clone(),
        name: decl.name.parts.clone(),
        namespace: decl.namespace.as_ref().map(|n| n.parts.clone()),
        parameters,
        remappings,
        extra_args: decl.extra_args.iter().cloned().collect(),
    }
}

fn remaps_to_action(remaps: &[RemapDecl]) -> Vec<crate::actions::node::Remapping> {
    remaps
        .iter()
        .map(|r| crate::actions::node::Remapping {
            from: r.from.parts.clone(),
            to: r.to.parts.clone(),
        })
        .collect()
}

fn env_decls_to_pairs(env: &[EnvDecl]) -> Vec<(String, String)> {
    env.iter()
        .map(|e| {
            let name = e.name.as_literal().unwrap_or("").to_string();
            let value = e.value.as_literal().unwrap_or("").to_string();
            (name, value)
        })
        .collect()
}
