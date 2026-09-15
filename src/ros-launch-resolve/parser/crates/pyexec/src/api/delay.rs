//! Where a Python `TimerAction`'s delay is recorded.
//!
//! The XML/YAML frontends get this for free: the traverser walks the timer
//! body itself, so everything appended to the member lists while that body
//! was being walked is, by construction, inside the timer (see
//! `play_launch_parser::traverser::delay`). The Python frontend cannot use
//! that argument. Python evaluates `actions=[Node(...)]` BEFORE
//! `TimerAction.__new__` runs, so by the time the timer exists its children
//! are already in the capture lists, mixed in with every other node the file
//! built — and "the last N captures" is not a sound reading of which ones
//! they are, because a node built earlier and passed in by name breaks it.
//!
//! What IS sound is object identity. `TimerAction.__new__` is handed the
//! actual mock objects, and each capturing mock (`Node`, `LifecycleNode`,
//! `ComposableNodeContainer`, `LoadComposableNodes`) records the SPAN of
//! capture-list indices its own constructor appended — a mark taken around
//! its own body, which is the same argument the traverser makes, scoped to
//! one constructor call. The timer then walks its `actions` by identity,
//! unions those spans, and stamps the delay on exactly them. Where the node
//! was built, what it was named, and what order the file built things in
//! stop mattering.
//!
//! Three shapes remain unattributable, and each is REPORTED rather than
//! guessed at:
//!
//! * a `period` that is not a number at parse time (a `LaunchConfiguration`
//!   the file never sets, say) — there is no delay to attribute;
//! * the same action object in two unrelated timers — `ros2 launch` starts
//!   it twice, this model holds it once, so only one delay can survive;
//! * a child this parser cannot see into (`IncludeLaunchDescription`, a class
//!   it does not know) — its nodes, if any, are reached by another route.
//!
//! An `OpaqueFunction` child is neither: its nodes do not exist yet when the
//! timer is constructed, so the executor's walk applies the delay by mark —
//! and a timer holding one that the walk never reaches is reported at the end
//! of the run.

use std::{
    cell::RefCell,
    collections::{HashMap, HashSet},
};

use play_launch_parser::bridge::{
    note_unsupported_action, update_captured_containers, update_captured_load_nodes,
    update_captured_nodes, with_launch_context,
};
use pyo3::{
    prelude::*,
    types::{PyList, PyTuple},
};

/// Which capture list an entry lives in.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub(crate) enum CaptureKind {
    Node,
    Container,
    LoadNode,
}

/// One capture, addressed by its list and its index there. Indices are
/// stable: the capture lists are only ever appended to and mutated in place
/// during a file's execution (`process_launch_arguments` re-resolves
/// substitutions in situ; nothing removes or reorders).
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub(crate) struct CaptureRef {
    kind: CaptureKind,
    index: usize,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Lens {
    nodes: usize,
    containers: usize,
    load_nodes: usize,
}

fn lens() -> Lens {
    with_launch_context(|ctx| Lens {
        nodes: ctx.captured_nodes().len(),
        containers: ctx.captured_containers().len(),
        load_nodes: ctx.captured_load_nodes().len(),
    })
}

/// What one mock action's constructor appended to the capture lists.
///
/// The default (all-zero) span is empty, which is what a mock whose
/// `condition` evaluated false should carry: it captured nothing, so a timer
/// around it has nothing to delay.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct CaptureSpan {
    before: Lens,
    after: Lens,
}

/// A mark taken before a constructor captures anything.
pub(crate) struct OpenSpan(Lens);

/// Open a span. Call at the top of a capturing constructor, close at the end.
pub(crate) fn open_span() -> OpenSpan {
    OpenSpan(lens())
}

impl OpenSpan {
    pub(crate) fn close(self) -> CaptureSpan {
        CaptureSpan {
            before: self.0,
            after: lens(),
        }
    }
}

impl CaptureSpan {
    pub(crate) fn refs(&self) -> Vec<CaptureRef> {
        let mut out = Vec::new();
        for index in self.before.nodes..self.after.nodes {
            out.push(CaptureRef {
                kind: CaptureKind::Node,
                index,
            });
        }
        for index in self.before.containers..self.after.containers {
            out.push(CaptureRef {
                kind: CaptureKind::Container,
                index,
            });
        }
        for index in self.before.load_nodes..self.after.load_nodes {
            out.push(CaptureRef {
                kind: CaptureKind::LoadNode,
                index,
            });
        }
        out
    }
}

// ========== per-execution registry ==========

#[derive(Default)]
struct Registry {
    next_seq: u64,
    /// Capture → the timer that owns its delay. A second, unrelated timer
    /// claiming the same capture is the one genuinely ambiguous shape.
    owner: HashMap<CaptureRef, u64>,
    /// Timers whose `actions` held an `OpaqueFunction`, with what to report
    /// if the executor's walk never reaches them.
    deferred: HashMap<u64, String>,
    visited: HashSet<u64>,
}

thread_local! {
    static REGISTRY: RefCell<Registry> = RefCell::new(Registry::default());
}

/// Clear the registry. Called once per launch-file execution: capture
/// indices are meaningful only within the run that produced them.
pub(crate) fn reset() {
    REGISTRY.with(|r| *r.borrow_mut() = Registry::default());
}

fn next_seq() -> u64 {
    REGISTRY.with(|r| {
        let mut r = r.borrow_mut();
        r.next_seq += 1;
        r.next_seq
    })
}

pub(crate) fn mark_visited(seq: u64) {
    REGISTRY.with(|r| {
        r.borrow_mut().visited.insert(seq);
    });
}

fn owner_of(entry: CaptureRef) -> Option<u64> {
    REGISTRY.with(|r| r.borrow().owner.get(&entry).copied())
}

/// Claim `refs` for timer `seq`. A capture already owned by a timer inside
/// `family` (this timer, or one nested in it) is legitimately re-claimed —
/// that is how nested periods add. One owned by anything else is the shared
/// case, and comes back separately rather than being stamped.
fn claim(
    refs: &[CaptureRef],
    seq: u64,
    family: &HashSet<u64>,
) -> (Vec<CaptureRef>, Vec<CaptureRef>) {
    REGISTRY.with(|r| {
        let mut r = r.borrow_mut();
        let mut mine = Vec::new();
        let mut shared = Vec::new();
        for entry in refs {
            match r.owner.get(entry) {
                Some(other) if !family.contains(other) => shared.push(*entry),
                _ => {
                    r.owner.insert(*entry, seq);
                    mine.push(*entry);
                }
            }
        }
        (mine, shared)
    })
}

/// Add `secs` to each capture's start delay. Nested timers ADD: `ros2 launch`
/// starts the inner timer when the outer one fires, so 2 s inside 3 s fires
/// at 5 s — the same rule the XML traverser applies.
pub(crate) fn accumulate(refs: &[CaptureRef], secs: f64) {
    fn add(slot: &mut Option<f64>, secs: f64) {
        *slot = Some(slot.unwrap_or(0.0) + secs);
    }
    if refs.iter().any(|r| r.kind == CaptureKind::Node) {
        update_captured_nodes(|v| {
            for r in refs.iter().filter(|r| r.kind == CaptureKind::Node) {
                if let Some(n) = v.get_mut(r.index) {
                    add(&mut n.start_delay_secs, secs);
                }
            }
        });
    }
    if refs.iter().any(|r| r.kind == CaptureKind::Container) {
        update_captured_containers(|v| {
            for r in refs.iter().filter(|r| r.kind == CaptureKind::Container) {
                if let Some(c) = v.get_mut(r.index) {
                    add(&mut c.start_delay_secs, secs);
                }
            }
        });
    }
    if refs.iter().any(|r| r.kind == CaptureKind::LoadNode) {
        update_captured_load_nodes(|v| {
            for r in refs.iter().filter(|r| r.kind == CaptureKind::LoadNode) {
                if let Some(l) = v.get_mut(r.index) {
                    add(&mut l.start_delay_secs, secs);
                }
            }
        });
    }
}

/// Name one capture the way a reader of the launch file would recognise it.
/// A diagnostic that says "a delay was lost" without saying WHICH node lost
/// it is the thing this whole path was faulted for.
fn describe(entry: CaptureRef) -> String {
    with_launch_context(|ctx| match entry.kind {
        CaptureKind::Node => ctx
            .captured_nodes()
            .get(entry.index)
            .map(|n| match &n.name {
                Some(name) => format!("{}/{} (name={name})", n.package, n.executable),
                None => format!("{}/{}", n.package, n.executable),
            }),
        CaptureKind::Container => ctx
            .captured_containers()
            .get(entry.index)
            .map(|c| format!("container {}{}", c.namespace, c.name)),
        CaptureKind::LoadNode => ctx.captured_load_nodes().get(entry.index).map(|l| {
            format!(
                "composable {}/{} (name={})",
                l.package, l.plugin, l.node_name
            )
        }),
    })
    .unwrap_or_else(|| "<capture missing>".to_string())
}

fn describe_all(refs: &[CaptureRef]) -> String {
    if refs.is_empty() {
        return "nothing this parser models".to_string();
    }
    refs.iter()
        .map(|r| describe(*r))
        .collect::<Vec<_>>()
        .join(", ")
}

// ========== walking a timer's `actions` by identity ==========

/// What a walk of one timer's `actions` found.
#[derive(Default)]
pub(crate) struct TimerChildren {
    refs: Vec<CaptureRef>,
    seen: HashSet<CaptureRef>,
    /// Timers nested inside this one, so a capture they already claimed is
    /// recognised as this timer's own rather than as a sharing conflict.
    family: HashSet<u64>,
    /// Children this parser cannot see into, described for the diagnostic.
    unattributable: Vec<String>,
    /// `OpaqueFunction` children — handled by the executor's walk.
    deferred: Vec<String>,
}

impl TimerChildren {
    fn add_span(&mut self, span: CaptureSpan) {
        for entry in span.refs() {
            self.add_ref(entry);
        }
    }

    /// Deduplicated: `actions=[n, n]` is one node, delayed once, not twice.
    fn add_ref(&mut self, entry: CaptureRef) {
        if self.seen.insert(entry) {
            self.refs.push(entry);
        }
    }
}

/// Actions that reach the record through no capture of their own, so a timer
/// around them has nothing to attribute and nothing to report. `ExecuteProcess`
/// and `ExecuteLocal` are here because this parser does not model a bare
/// process at all — that loss is its own gap, and naming it "timer" would
/// misreport it.
const INERT: &[&str] = &[
    "AppendEnvironmentVariable",
    "DeclareLaunchArgument",
    "EmitEvent",
    "ExecuteLocal",
    "ExecuteProcess",
    "LifecycleTransition",
    "LogInfo",
    "OpaqueCoroutine",
    "PopEnvironment",
    "PopLaunchConfigurations",
    "PopRosNamespace",
    "PushEnvironment",
    "PushLaunchConfigurations",
    "PushRosNamespace",
    "RegisterEventHandler",
    "ResetEnvironment",
    "ResetLaunchConfigurations",
    "SetEnvironmentVariable",
    "SetLaunchConfiguration",
    "SetParameter",
    "SetParametersFromFile",
    "SetROSLogDir",
    "SetRemap",
    "SetUseSimTime",
    "Shutdown",
    "UnsetEnvironmentVariable",
    "UnsetLaunchConfiguration",
];

/// Walk one entry of a timer's `actions` list, by object identity.
pub(crate) fn collect(py: Python, obj: &Py<PyAny>, out: &mut TimerChildren) {
    use crate::api::{
        actions::{GroupAction, OpaqueFunction, TimerAction},
        launch::LaunchDescription,
        launch_ros::{ComposableNodeContainer, LifecycleNode, LoadComposableNodes, Node, RosTimer},
    };

    let bound = obj.bind(py);

    if bound.is_none() {
        return;
    }

    // A nested list/tuple of actions. Checked by type rather than by trying
    // to extract a sequence: a `str` would extract as a list of characters.
    if bound.is_instance_of::<PyList>() || bound.is_instance_of::<PyTuple>() {
        if let Ok(items) = bound.extract::<Vec<Py<PyAny>>>() {
            for item in &items {
                collect(py, item, out);
            }
        }
        return;
    }

    // The capturing mocks: each knows exactly what it appended.
    if let Ok(x) = bound.cast::<Node>() {
        let span = x.borrow().capture_span();
        out.add_span(span);
        return;
    }
    if let Ok(x) = bound.cast::<LifecycleNode>() {
        let span = x.borrow().capture_span();
        out.add_span(span);
        return;
    }
    if let Ok(x) = bound.cast::<ComposableNodeContainer>() {
        let span = x.borrow().capture_span();
        out.add_span(span);
        return;
    }
    if let Ok(x) = bound.cast::<LoadComposableNodes>() {
        let span = x.borrow().capture_span();
        out.add_span(span);
        return;
    }

    // Containers of actions: recurse. The borrow is dropped before recursing
    // so a structure that reaches itself cannot double-borrow.
    if let Ok(x) = bound.cast::<GroupAction>() {
        let actions = x.borrow().actions.clone();
        for a in &actions {
            collect(py, a, out);
        }
        return;
    }
    if let Ok(x) = bound.cast::<LaunchDescription>() {
        let actions = x.borrow().actions.clone();
        for a in &actions {
            collect(py, a, out);
        }
        return;
    }

    // A nested timer has already attributed its own children; take its result
    // whole, so the outer period adds onto the inner one.
    if let Ok(x) = bound.cast::<TimerAction>() {
        let inner = x.borrow();
        let (refs, family, deferred) = (
            inner.owned.clone(),
            inner.family.clone(),
            inner.has_deferred,
        );
        drop(inner);
        out.family.extend(family);
        for entry in refs {
            out.add_ref(entry);
        }
        if deferred {
            out.deferred
                .push("a nested TimerAction's OpaqueFunction".to_string());
        }
        return;
    }
    if let Ok(x) = bound.cast::<RosTimer>() {
        let inner = x.borrow();
        let (refs, family, deferred) = (
            inner.owned.clone(),
            inner.family.clone(),
            inner.has_deferred,
        );
        drop(inner);
        out.family.extend(family);
        for entry in refs {
            out.add_ref(entry);
        }
        if deferred {
            out.deferred
                .push("a nested RosTimer's OpaqueFunction".to_string());
        }
        return;
    }

    // Its nodes do not exist yet — the executor's walk delays them instead.
    if bound.cast::<OpaqueFunction>().is_ok() {
        out.deferred.push("OpaqueFunction(...)".to_string());
        return;
    }

    let class = bound
        .get_type()
        .name()
        .and_then(|n| n.extract::<String>())
        .unwrap_or_else(|_| "<unknown>".to_string());
    if INERT.contains(&class.as_str()) {
        return;
    }
    out.unattributable.push(format!("{class}(...)"));
}

/// Read a timer's `period` the way `<timer period=…>` is read: eagerly, and
/// through the substitution machinery, so `LaunchConfiguration('delay')`
/// resolves against what the file has set.
///
/// `None` means "not a number here", which is reported rather than guessed.
/// Note that ROS 2 resolves `period` when the timer FIRES, not when the
/// description is built, so a period that only resolves later is a divergence
/// in its own right (see `docs/guide/parser-features.md`).
pub(crate) fn resolve_period(py: Python, period: &Py<PyAny>) -> Option<f64> {
    if let Ok(v) = period.extract::<f64>(py) {
        return Some(v);
    }
    let text = crate::api::utils::pyobject_to_string(py, period).ok()?;
    text.trim().parse::<f64>().ok()
}

/// How a timer's `period` should read back in a diagnostic.
pub(crate) fn period_repr(py: Python, period: &Py<PyAny>) -> String {
    period
        .bind(py)
        .repr()
        .and_then(|r| r.extract::<String>())
        .unwrap_or_else(|_| "?".to_string())
}

// ========== applying one timer ==========

/// What a timer stores after attributing itself, so an enclosing timer can
/// build on it.
pub(crate) struct AppliedTimer {
    pub(crate) seq: u64,
    pub(crate) owned: Vec<CaptureRef>,
    pub(crate) family: HashSet<u64>,
    pub(crate) has_deferred: bool,
}

/// Attribute one timer's delay to its children, and report whatever could
/// not be attributed. Shared by `TimerAction` and `launch_ros`'s `RosTimer`.
pub(crate) fn apply_timer(
    py: Python,
    class: &str,
    period: &Py<PyAny>,
    actions: &[Py<PyAny>],
) -> AppliedTimer {
    let seq = next_seq();
    let mut children = TimerChildren::default();
    for a in actions {
        collect(py, a, &mut children);
    }
    let TimerChildren {
        refs,
        mut family,
        unattributable,
        deferred,
        ..
    } = children;
    family.insert(seq);

    let repr = period_repr(py, period);
    let Some(secs) = resolve_period(py, period) else {
        if !refs.is_empty() || !deferred.is_empty() {
            note_unsupported_action(
                "timer",
                Some(format!(
                    "{class}(period={repr}) in a Python launch file: the period is not a number \
                     this parser can resolve while reading the file, so the delay could not be \
                     attributed and these start immediately in the model: {}. Give the timer a \
                     literal period, or express the delay in XML/YAML `<timer period=…>`.",
                    describe_all(&refs)
                )),
            );
        }
        return AppliedTimer {
            seq,
            owned: refs,
            family,
            has_deferred: !deferred.is_empty(),
        };
    };

    let (mine, shared) = claim(&refs, seq, &family);
    if !shared.is_empty() {
        note_unsupported_action(
            "timer",
            Some(format!(
                "{class}(period={secs}) in a Python launch file shares an action object with \
                 another timer: {}. `ros2 launch` starts such an action once per timer; this \
                 model holds it once, so only the first timer's delay is kept and this {secs}s \
                 one is discarded. Build a separate Node(...) for each timer.",
                describe_all(&shared)
            )),
        );
    }
    accumulate(&mine, secs);

    if !unattributable.is_empty() {
        note_unsupported_action(
            "timer",
            Some(format!(
                "{class}(period={secs}) in a Python launch file holds {} — this parser cannot see \
                 which members they contribute, so any node they carry starts immediately in the \
                 model instead of after {secs}s. Put the nodes directly in the timer's `actions`, \
                 or express the delay in XML/YAML `<timer period=…>`.",
                unattributable.join(", ")
            )),
        );
    }

    if !deferred.is_empty() {
        let detail = format!(
            "{class}(period={secs}) in a Python launch file holds {} whose actions this parser \
             never evaluated, so nothing carries its {secs}s delay. Return the nodes from \
             `generate_launch_description()` directly, or express the delay in XML/YAML \
             `<timer period=…>`.",
            deferred.join(", ")
        );
        REGISTRY.with(|r| {
            r.borrow_mut().deferred.insert(seq, detail);
        });
    }

    AppliedTimer {
        seq,
        owned: refs,
        family,
        has_deferred: !deferred.is_empty(),
    }
}

/// A node reached by the executor's walk OUTSIDE any timer, whose capture a
/// timer has already claimed, starts twice under `ros2 launch` — once at t=0
/// and once when the timer fires. The model holds it once, so say so.
pub(crate) fn note_immediate_start(py: Python, entity: &Py<PyAny>) {
    use crate::api::launch_ros::{
        ComposableNodeContainer, LifecycleNode, LoadComposableNodes, Node,
    };

    let bound = entity.bind(py);
    let span = if let Ok(x) = bound.cast::<Node>() {
        x.borrow().capture_span()
    } else if let Ok(x) = bound.cast::<LifecycleNode>() {
        x.borrow().capture_span()
    } else if let Ok(x) = bound.cast::<ComposableNodeContainer>() {
        x.borrow().capture_span()
    } else if let Ok(x) = bound.cast::<LoadComposableNodes>() {
        x.borrow().capture_span()
    } else {
        return;
    };

    let delayed: Vec<CaptureRef> = span
        .refs()
        .into_iter()
        .filter(|r| owner_of(*r).is_some())
        .collect();
    if delayed.is_empty() {
        return;
    }
    note_unsupported_action(
        "timer",
        Some(format!(
            "an action object in a Python launch file is both inside a timer and started \
             directly: {}. `ros2 launch` starts it twice — once immediately and once when the \
             timer fires — and this model holds it once, keeping only the delayed start. Build a \
             separate Node(...) for each start.",
            describe_all(&delayed)
        )),
    );
}

/// Report every timer whose `OpaqueFunction` child the executor's walk never
/// reached. Called once, after the walk.
pub(crate) fn report_unvisited_deferred() {
    // Sorted by timer seq, so a file with two of these reports them in the
    // order the file declared them rather than in hash order.
    let pending: Vec<String> = REGISTRY.with(|r| {
        let r = r.borrow();
        let mut pending: Vec<(u64, String)> = r
            .deferred
            .iter()
            .filter(|(seq, _)| !r.visited.contains(*seq))
            .map(|(seq, detail)| (*seq, detail.clone()))
            .collect();
        pending.sort_by_key(|(seq, _)| *seq);
        pending.into_iter().map(|(_, detail)| detail).collect()
    });
    for detail in pending {
        note_unsupported_action("timer", Some(detail));
    }
}
