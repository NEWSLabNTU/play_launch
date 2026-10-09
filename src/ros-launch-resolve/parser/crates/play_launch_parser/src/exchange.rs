//! What crosses between the traverser and the Python half while a
//! `.launch.py` runs (C ABI 7).
//!
//! `launch` runs a Python launch description's entities in the SAME context as
//! the file that included it, in order, and an `IncludeLaunchDescription`
//! among them runs its target right there, before the next entity. So the
//! state has to travel both ways, and at every include, not only at the ends:
//!
//! * into the file: the includer's configurations, namespace, global
//!   parameters and remappings, and environment ([`ContextState`]);
//! * out at each include: what the file has produced so far and the state it
//!   has built up ([`IncludeRequest`]) — the traverser runs the target in that
//!   state and hands its own state back, so the entities after the include see
//!   what the included file declared, `<let>`, pushed or set;
//! * out at the end: the rest of what it produced and its final state
//!   ([`ExecResult`]), which becomes the includer's, since an include does not
//!   scope anything.
//!
//! Before ABI 7 the file ran to completion first and its includes were replayed
//! afterwards from a list, with the configurations it had set left behind in
//! the Python half: an XML file it included never saw them, and neither did
//! the file that included it.

use crate::captures::{ContainerCapture, DeclaredArgumentCapture, LoadNodeCapture, NodeCapture};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// One of the two global lists `launch_ros` keeps by reference
/// (`ros_remaps`, `global_params`), as it crosses the boundary.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SharedListState {
    pub items: Vec<(String, String)>,
    /// Whether the sender's list is the SAME list the receiver had current at
    /// the last exchange — appended to, not replaced. The receiver then writes
    /// the items through its own list rather than starting a new one, so a
    /// group that saved that list sees the appends after it pops, exactly as
    /// launch's shallow `PushLaunchConfigurations` copy does.
    pub same: bool,
}

/// The launch-context state a `.launch.py` reads and writes.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct ContextState {
    /// Every launch configuration, resolved.
    pub configurations: BTreeMap<String, String>,
    /// The ROS namespace stack; its top is `ros_namespace`.
    pub namespace_stack: Vec<String>,
    /// `ros_remaps`; `None` when unset.
    pub remaps: Option<SharedListState>,
    /// `global_params`; `None` when unset.
    pub global_parameters: Option<SharedListState>,
    /// Environment variables set by `<set_env>` / `SetEnvironmentVariable`.
    pub environment: BTreeMap<String, String>,
    /// The file being run, for `ThisLaunchFileDir()` / `$(dirname)`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub current_file: Option<String>,
}

/// Which list was current at the last exchange, on THIS side — what
/// [`SharedListState::same`] is measured against. Kept by each call frame,
/// so nested executions do not disturb an outer one's reference point.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ListSync {
    pub(crate) remap: Option<usize>,
    pub(crate) param: Option<usize>,
}

/// Everything a `.launch.py` produced between two hand-overs, in order.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct Produced {
    pub nodes: Vec<NodeCapture>,
    pub containers: Vec<ContainerCapture>,
    pub load_nodes: Vec<LoadNodeCapture>,
    /// Every `DeclareLaunchArgument` executed (issue 0030).
    pub declared_arguments: Vec<DeclaredArgumentCapture>,
    /// What the Python half recognised but could not model.
    pub unsupported: Vec<(String, Option<String>)>,
}

impl Produced {
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
            && self.containers.is_empty()
            && self.load_nodes.is_empty()
            && self.declared_arguments.is_empty()
            && self.unsupported.is_empty()
    }
}

/// An `IncludeLaunchDescription` the Python half reached: run `file_path`
/// now, in `state`, after taking in `produced`.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct IncludeRequest {
    pub produced: Produced,
    pub state: ContextState,
    /// Resolved path of the launch file to include.
    pub file_path: String,
    /// The include's own `launch_arguments`, resolved, in order. They are
    /// already set in `state` (launch sets them in the includer's context);
    /// they travel separately because the required-argument check looks at
    /// what the include passed, never at what was in scope.
    pub args: Vec<(String, String)>,
    /// The start delay of the timers the include sits under, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub delay_secs: Option<f64>,
}

/// How a `.launch.py` finished.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ExecResult {
    pub produced: Produced,
    pub state: ContextState,
}

/// The traverser's side of an execution: runs the includes a `.launch.py`
/// reaches, as it reaches them.
pub trait IncludeHost {
    /// Take in what the file produced so far and its state, run the include,
    /// and return the state the file continues with.
    fn include(&mut self, request: IncludeRequest) -> Result<ContextState, String>;
}
