//! Action module

pub mod arg;
pub mod container;
pub mod declare_argument;
pub mod executable;
pub mod group;
pub mod include;
pub mod let_action;
pub mod load_composable_node;
pub mod node;
pub mod set_env;
pub mod set_parameter;
pub mod set_remap;

pub use arg::ArgAction;
pub use container::{ComposableNodeAction, ContainerAction};
pub use declare_argument::DeclareArgumentAction;
pub use executable::ExecutableAction;
pub use group::GroupAction;
pub use include::IncludeAction;
pub use let_action::LetAction;
pub use load_composable_node::LoadComposableNodeAction;
pub use node::{NodeAction, Parameter, Remapping};
pub use set_env::{SetEnvAction, UnsetEnvAction};
pub use set_parameter::SetParameterAction;
pub use set_remap::SetRemapAction;

// --- From impls for converting action types to IR ActionKind ---

#[cfg(feature = "ir")]
use crate::ir::{ActionKind, ComposableNodeDecl, EnvDecl, Expr, IncludeArg, ParamDecl, RemapDecl};

#[cfg(feature = "ir")]
impl From<NodeAction> for ActionKind {
    fn from(node: NodeAction) -> Self {
        ActionKind::SpawnNode {
            package: Expr::new(node.package),
            executable: Expr::new(node.executable),
            name: node.name.map(Expr::new),
            namespace: node.namespace.map(Expr::new),
            params: node
                .parameters
                .into_iter()
                .map(|p| ParamDecl {
                    name: p.name,
                    value: Expr::new(p.value),
                })
                .collect(),
            param_files: node.param_files.into_iter().map(Expr::new).collect(),
            remaps: node
                .remappings
                .into_iter()
                .map(|r| RemapDecl {
                    from: Expr::new(r.from),
                    to: Expr::new(r.to),
                })
                .collect(),
            env: node
                .environment
                .into_iter()
                .map(|(k, v)| EnvDecl {
                    name: Expr::literal(k),
                    value: Expr::literal(v),
                })
                .collect(),
            args: node.args.map(Expr::new),
            ros_args: node.ros_args.map(Expr::new),
            respawn: node.respawn.map(Expr::new),
            respawn_delay: node.respawn_delay.map(Expr::new),
        }
    }
}

#[cfg(feature = "ir")]
impl From<ExecutableAction> for ActionKind {
    fn from(exec: ExecutableAction) -> Self {
        ActionKind::SpawnExecutable {
            cmd: Expr::new(exec.cmd),
            name: exec.name.map(Expr::new),
            args: exec.arguments.into_iter().map(Expr::new).collect(),
            env: exec
                .environment
                .into_iter()
                .map(|(k, v)| EnvDecl {
                    name: Expr::literal(k),
                    value: Expr::literal(v),
                })
                .collect(),
        }
    }
}

#[cfg(feature = "ir")]
impl From<ContainerAction> for ActionKind {
    fn from(c: ContainerAction) -> Self {
        ActionKind::SpawnContainer {
            package: Expr::new(c.package),
            executable: Expr::new(c.executable),
            name: Expr::new(c.name),
            namespace: c.namespace.map(Expr::new),
            args: c.args.map(Expr::new),
            ros_args: c.ros_args.map(Expr::new),
            nodes: c.composable_nodes.into_iter().map(Into::into).collect(),
        }
    }
}

#[cfg(feature = "ir")]
impl From<ComposableNodeAction> for ComposableNodeDecl {
    fn from(n: ComposableNodeAction) -> Self {
        ComposableNodeDecl {
            package: Expr::new(n.package),
            plugin: Expr::new(n.plugin),
            name: Expr::new(n.name),
            namespace: n.namespace.map(Expr::new),
            params: n
                .parameters
                .into_iter()
                .map(|(k, v)| ParamDecl {
                    name: k,
                    value: Expr::literal(v),
                })
                .collect(),
            remaps: n
                .remappings
                .into_iter()
                .map(|(f, t)| RemapDecl {
                    from: Expr::literal(f),
                    to: Expr::literal(t),
                })
                .collect(),
            extra_args: n.extra_args.into_iter().collect(),
            condition: None,
            span: None,
        }
    }
}

#[cfg(feature = "ir")]
impl From<LoadComposableNodeAction> for ActionKind {
    fn from(l: LoadComposableNodeAction) -> Self {
        ActionKind::LoadComposableNode {
            target: Expr::new(l.target),
            nodes: l.composable_nodes.into_iter().map(Into::into).collect(),
        }
    }
}

#[cfg(feature = "ir")]
impl From<IncludeAction> for ActionKind {
    fn from(inc: IncludeAction) -> Self {
        ActionKind::Include {
            file: Expr::new(inc.file),
            args: inc
                .args
                .into_iter()
                .map(|(name, value)| IncludeArg {
                    name,
                    value: Expr::new(value),
                })
                .collect(),
            body: None,
        }
    }
}

#[cfg(feature = "ir")]
impl From<SetEnvAction> for ActionKind {
    fn from(a: SetEnvAction) -> Self {
        ActionKind::SetEnv {
            name: a.name,
            value: Expr::new(a.value),
        }
    }
}

#[cfg(feature = "ir")]
impl From<UnsetEnvAction> for ActionKind {
    fn from(a: UnsetEnvAction) -> Self {
        ActionKind::UnsetEnv { name: a.name }
    }
}

#[cfg(feature = "ir")]
impl From<SetParameterAction> for ActionKind {
    fn from(a: SetParameterAction) -> Self {
        ActionKind::SetParameter {
            name: a.name,
            value: Expr::new(a.value),
        }
    }
}

#[cfg(feature = "ir")]
impl From<SetRemapAction> for ActionKind {
    fn from(a: SetRemapAction) -> Self {
        ActionKind::SetRemap {
            from: Expr::new(a.from),
            to: Expr::new(a.to),
        }
    }
}

#[cfg(feature = "ir")]
impl From<GroupAction> for ActionKind {
    fn from(g: GroupAction) -> Self {
        ActionKind::Group {
            namespace: g.namespace.map(Expr::new),
            body: Vec::new(),
        }
    }
}
