//! Launch Tree Intermediate Representation (IR)
//!
//! An unevaluated representation of a launch file's structure.
//! Preserves conditional branches, substitution expressions, and include hierarchy
//! for static analysis, evaluation, and annotation.

use crate::substitution::{LaunchContext, Substitution, resolve_substitutions};
use std::path::PathBuf;

/// A lazy string expression (unevaluated substitution chain).
/// Evaluate with a `LaunchContext` to resolve to a concrete string.
#[derive(Debug, Clone, PartialEq)]
pub struct Expr {
    /// The substitution parts that make up this expression.
    pub parts: Vec<Substitution>,
}

impl Expr {
    /// Create an `Expr` from substitution parts.
    pub fn new(parts: Vec<Substitution>) -> Self {
        Self { parts }
    }

    /// Create an `Expr` from a literal string (no substitutions).
    pub fn literal(s: impl Into<String>) -> Self {
        Self {
            parts: vec![Substitution::Text(s.into())],
        }
    }

    /// Returns `true` if this expression is a single literal text with no substitutions.
    pub fn is_literal(&self) -> bool {
        self.parts.len() == 1 && matches!(&self.parts[0], Substitution::Text(_))
    }

    /// If this expression is a literal, return its value.
    pub fn as_literal(&self) -> Option<&str> {
        if self.parts.len() == 1
            && let Substitution::Text(s) = &self.parts[0]
        {
            return Some(s.as_str());
        }
        None
    }

    /// Resolve this expression against a `LaunchContext`.
    pub fn resolve(
        &self,
        context: &LaunchContext,
    ) -> Result<String, crate::error::SubstitutionError> {
        resolve_substitutions(&self.parts, context)
    }
}

impl From<Vec<Substitution>> for Expr {
    fn from(parts: Vec<Substitution>) -> Self {
        Self { parts }
    }
}

/// Source location for an IR node.
#[derive(Debug, Clone, PartialEq)]
pub struct Span {
    pub file: PathBuf,
    pub line: usize,
}

/// Condition gating an action's execution.
#[derive(Debug, Clone, PartialEq)]
pub enum Condition {
    /// Execute when the expression evaluates to truthy.
    If(Expr),
    /// Execute when the expression evaluates to falsy.
    Unless(Expr),
}

/// A single action with optional condition and source provenance.
#[derive(Debug, Clone)]
pub struct Action {
    pub kind: ActionKind,
    pub condition: Option<Condition>,
    pub span: Option<Span>,
}

/// All launch action types.
#[derive(Debug, Clone)]
pub enum ActionKind {
    // --- Arguments & Variables ---
    /// `<arg>` or `<declare_argument>` — declares a launch argument with optional default.
    DeclareArgument {
        name: String,
        default: Option<Expr>,
        description: Option<String>,
        choices: Option<Vec<String>>,
    },

    /// `<let>` — sets a scoped variable unconditionally.
    SetVariable { name: String, value: Expr },

    // --- Environment ---
    /// `<set_env>` — sets an environment variable in scope.
    SetEnv { name: String, value: Expr },

    /// `<unset_env>` — removes an environment variable from scope.
    UnsetEnv { name: String },

    // --- Namespace ---
    /// `<push-ros-namespace>` — pushes a namespace onto the namespace stack.
    PushNamespace { namespace: Expr },

    /// `<pop-ros-namespace>` — pops the most recent namespace from the stack.
    PopNamespace,

    // --- Global Parameters & Remappings ---
    /// `<set_parameter>` — sets a global parameter in scope.
    SetParameter { name: String, value: Expr },

    /// `<set_remap>` — sets a global topic remapping in scope.
    SetRemap { from: Expr, to: Expr },

    // --- Grouping ---
    /// `<group>` — scoped block with optional namespace.
    Group {
        namespace: Option<Expr>,
        body: Vec<Action>,
    },

    // --- Include ---
    /// `<include>` — includes another launch file, with its parsed body attached.
    Include {
        file: Expr,
        args: Vec<IncludeArg>,
        body: Option<Box<LaunchProgram>>,
    },

    // --- Nodes ---
    /// `<node>` — spawns a ROS 2 node.
    SpawnNode {
        package: Expr,
        executable: Expr,
        name: Option<Expr>,
        namespace: Option<Expr>,
        params: Vec<ParamDecl>,
        param_files: Vec<Expr>,
        remaps: Vec<RemapDecl>,
        env: Vec<EnvDecl>,
        args: Option<Expr>,
        respawn: Option<Expr>,
        respawn_delay: Option<Expr>,
        /// `<node machine="…">` — ROS 2 multi-host target host.
        machine: Option<Expr>,
    },

    /// `<executable>` — spawns a non-ROS executable.
    SpawnExecutable {
        cmd: Expr,
        name: Option<Expr>,
        args: Vec<Expr>,
        env: Vec<EnvDecl>,
    },

    /// `<node_container>` — spawns a composable node container with inline nodes.
    SpawnContainer {
        package: Expr,
        executable: Expr,
        name: Expr,
        namespace: Option<Expr>,
        args: Option<Expr>,
        nodes: Vec<ComposableNodeDecl>,
    },

    /// `<load_composable_node>` — loads composable nodes into an existing container.
    LoadComposableNode {
        target: Expr,
        nodes: Vec<ComposableNodeDecl>,
    },

    /// Python `OpaqueFunction` or other non-analyzable action.
    OpaqueFunction { description: String },
}

/// A complete launch program (one file's worth of actions).
#[derive(Debug, Clone)]
pub struct LaunchProgram {
    pub source: PathBuf,
    pub body: Vec<Action>,
}

impl LaunchProgram {
    /// Collect all declared argument names (recursive into groups and includes).
    pub fn arguments(&self) -> Vec<&str> {
        let mut result = Vec::new();
        collect_arguments(&self.body, &mut result);
        result
    }

    /// Collect all node-spawning actions (SpawnNode, SpawnExecutable, SpawnContainer).
    pub fn all_nodes(&self) -> Vec<&Action> {
        let mut result = Vec::new();
        collect_nodes(&self.body, &mut result);
        result
    }
}

fn collect_arguments<'a>(actions: &'a [Action], out: &mut Vec<&'a str>) {
    for action in actions {
        match &action.kind {
            ActionKind::DeclareArgument { name, .. } => {
                out.push(name.as_str());
            }
            ActionKind::Group { body, .. } => {
                collect_arguments(body, out);
            }
            ActionKind::Include {
                body: Some(program),
                ..
            } => {
                collect_arguments(&program.body, out);
            }
            _ => {}
        }
    }
}

fn collect_nodes<'a>(actions: &'a [Action], out: &mut Vec<&'a Action>) {
    for action in actions {
        match &action.kind {
            ActionKind::SpawnNode { .. }
            | ActionKind::SpawnExecutable { .. }
            | ActionKind::SpawnContainer { .. } => {
                out.push(action);
            }
            ActionKind::Group { body, .. } => {
                collect_nodes(body, out);
            }
            ActionKind::Include {
                body: Some(program),
                ..
            } => {
                collect_nodes(&program.body, out);
            }
            _ => {}
        }
    }
}

// --- Supporting types ---

/// An argument passed to an `<include>`.
#[derive(Debug, Clone, PartialEq)]
pub struct IncludeArg {
    pub name: String,
    pub value: Expr,
}

/// An inline parameter declaration.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamDecl {
    pub name: String,
    pub value: Expr,
}

/// A topic remapping declaration.
#[derive(Debug, Clone, PartialEq)]
pub struct RemapDecl {
    pub from: Expr,
    pub to: Expr,
}

/// An environment variable declaration.
#[derive(Debug, Clone, PartialEq)]
pub struct EnvDecl {
    pub name: Expr,
    pub value: Expr,
}

/// A composable node to be loaded into a container.
#[derive(Debug, Clone)]
pub struct ComposableNodeDecl {
    pub package: Expr,
    pub plugin: Expr,
    pub name: Expr,
    pub namespace: Option<Expr>,
    pub params: Vec<ParamDecl>,
    pub remaps: Vec<RemapDecl>,
    pub extra_args: Vec<(String, String)>,
    pub condition: Option<Condition>,
    pub span: Option<Span>,
}
