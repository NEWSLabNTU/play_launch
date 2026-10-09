//! Executing a Python launch description the way `launch` does.
//!
//! `launch` executes a description's entities LAZILY and IN ORDER: an action
//! does its work when it is visited, not when it is constructed, against the
//! launch context as the actions before it left it. A `condition=` is
//! evaluated at that point and, if false, the action and everything under it
//! is skipped; a scoped `GroupAction` pushes the configurations and pops them
//! after its body; an `IncludeLaunchDescription` runs its target right there,
//! so the actions after it see what the included file declared.
//!
//! The mocks used to do their work in their CONSTRUCTORS. Python evaluates
//! `GroupAction([Node(...)], condition=IfCondition('false'))` inside-out, so
//! the node was captured before the group — or its condition — existed: a
//! false condition on a group, an include or a `SetLaunchConfiguration` was
//! ignored, a scoped group scoped nothing, and every include ran after the
//! whole file had. Each mock now only records its arguments, and this module
//! walks the returned description and executes each action as `launch`
//! would.
//!
//! An include is handed to the traverser through the [`IncludeHost`] the
//! execution was started with, synchronously, with what has been produced so
//! far and the state reached; the walk continues in the state it hands back.

use play_launch_parser::{
    bridge::{drain_produced, with_launch_context},
    exchange::{IncludeHost, IncludeRequest, ListSync},
    substitution::context::ConfigurationSnapshot,
};
use pyo3::{
    exceptions::PyRuntimeError,
    prelude::*,
    types::{PyList, PyTuple},
};
use std::{cell::RefCell, collections::HashMap};

/// One `.launch.py` execution. Executions nest — an include of a `.launch.py`
/// runs while the including file is mid-walk — so these form a stack.
struct Frame {
    /// The traverser. Valid for the frame's lifetime, which is the
    /// `exec_file` call that pushed it (see [`with_frame`]).
    host: *mut (dyn IncludeHost + 'static),
    /// This side's reference point for the shared global lists.
    sync: ListSync,
    /// The delays of the timers the walk is inside, innermost last.
    delays: Vec<f64>,
    /// Capturing actions already executed, by object identity, with the
    /// delay they ran under. Executing one twice is reported, not repeated.
    executed: HashMap<usize, (f64, String)>,
    /// What standalone `PushLaunchConfigurations` / `PushEnvironment` saved.
    config_stack: Vec<ConfigurationSnapshot>,
    env_stack: Vec<HashMap<String, String>>,
}

thread_local! {
    static FRAMES: RefCell<Vec<Frame>> = const { RefCell::new(Vec::new()) };
    /// How many entities with a `condition=` the walk is inside. A
    /// declaration under one is "conditionally included" in `launch`'s
    /// include-time check, which therefore does not demand it.
    static CONDITIONAL_DEPTH: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// Whether the walk is inside an entity that carries a `condition=`.
pub(crate) fn under_condition() -> bool {
    CONDITIONAL_DEPTH.with(|d| d.get() > 0)
}

/// Run `f` as one execution talking to `host`. Returns `f`'s result and the
/// reference point for the shared lists at the end, for the final export.
pub(crate) fn with_frame<R>(
    host: &mut dyn IncludeHost,
    sync: ListSync,
    f: impl FnOnce() -> R,
) -> (R, ListSync) {
    // SAFETY: the pointer is only dereferenced while this frame is on the
    // stack, i.e. strictly inside this call, during which `host` is borrowed.
    let host: *mut (dyn IncludeHost + '_) = host;
    let host: *mut (dyn IncludeHost + 'static) = unsafe { std::mem::transmute(host) };
    FRAMES.with(|f| {
        f.borrow_mut().push(Frame {
            host,
            sync,
            delays: Vec::new(),
            executed: HashMap::new(),
            config_stack: Vec::new(),
            env_stack: Vec::new(),
        })
    });
    struct Pop;
    impl Drop for Pop {
        fn drop(&mut self) {
            FRAMES.with(|f| {
                f.borrow_mut().pop();
            });
        }
    }
    let pop = Pop;
    let result = f();
    let sync = FRAMES.with(|f| f.borrow().last().map(|fr| fr.sync).unwrap_or_default());
    drop(pop);
    (result, sync)
}

/// The start delay the walk is under: the sum of the enclosing timers.
pub(crate) fn current_delay() -> f64 {
    FRAMES.with(|f| {
        f.borrow()
            .last()
            .map(|fr| fr.delays.iter().sum())
            .unwrap_or(0.0)
    })
}

fn push_delay(secs: f64) {
    FRAMES.with(|f| {
        if let Some(fr) = f.borrow_mut().last_mut() {
            fr.delays.push(secs);
        }
    });
}

fn pop_delay() {
    FRAMES.with(|f| {
        if let Some(fr) = f.borrow_mut().last_mut() {
            fr.delays.pop();
        }
    });
}

/// Record that the capturing action `obj` is executing now. Returns `false`
/// — and reports it — when it already has: `launch` refuses to execute an
/// action twice ("executed more than once"), and the model holds it once.
pub(crate) fn claim_execution(obj: &Bound<'_, PyAny>, what: &str) -> bool {
    let id = obj.as_ptr() as usize;
    let now = current_delay();
    let earlier = FRAMES.with(|f| {
        let mut f = f.borrow_mut();
        let fr = f.last_mut()?;
        if let Some(prev) = fr.executed.get(&id) {
            return Some(prev.clone());
        }
        fr.executed.insert(id, (now, what.to_string()));
        None
    });
    let Some((then, _)) = earlier else {
        return true;
    };
    let detail = if then > 0.0 && now > 0.0 {
        format!(
            "a timer in a Python launch file shares an action object with another timer: \
             {what}. `ros2 launch` refuses to execute an action twice; this model keeps the \
             first start (after {then}s) and discards this {now}s one. Build a separate \
             Node(...) for each timer."
        )
    } else if then > 0.0 || now > 0.0 {
        let delayed = if then > 0.0 { then } else { now };
        format!(
            "an action object in a Python launch file is both inside a timer and started \
             directly: {what}. `ros2 launch` refuses to execute an action twice; this model \
             keeps the first start and discards the other (the delayed one is {delayed}s). \
             Build a separate Node(...) for each start."
        )
    } else {
        format!(
            "an action object in a Python launch file is executed more than once: {what}. \
             `ros2 launch` refuses this; the model holds it once."
        )
    };
    play_launch_parser::bridge::note_unsupported_action("timer", Some(detail));
    false
}

/// A standalone `PushLaunchConfigurations`.
pub(crate) fn push_configurations() {
    let snap = with_launch_context(|ctx| ctx.push_launch_configurations());
    FRAMES.with(|f| {
        if let Some(fr) = f.borrow_mut().last_mut() {
            fr.config_stack.push(snap);
        }
    });
}

/// A standalone `PopLaunchConfigurations`. Popping more than was pushed is
/// an error in `launch` too.
pub(crate) fn pop_configurations() -> PyResult<()> {
    let snap = FRAMES.with(|f| {
        f.borrow_mut()
            .last_mut()
            .and_then(|fr| fr.config_stack.pop())
    });
    let snap = snap.ok_or_else(|| {
        PyRuntimeError::new_err("PopLaunchConfigurations without a matching push")
    })?;
    with_launch_context(|ctx| ctx.pop_launch_configurations(snap));
    Ok(())
}

/// A standalone `PushEnvironment`.
pub(crate) fn push_environment() {
    let snap = with_launch_context(|ctx| ctx.push_environment());
    FRAMES.with(|f| {
        if let Some(fr) = f.borrow_mut().last_mut() {
            fr.env_stack.push(snap);
        }
    });
}

/// A standalone `PopEnvironment`.
pub(crate) fn pop_environment() -> PyResult<()> {
    let snap = FRAMES.with(|f| f.borrow_mut().last_mut().and_then(|fr| fr.env_stack.pop()));
    let snap =
        snap.ok_or_else(|| PyRuntimeError::new_err("PopEnvironment without a matching push"))?;
    with_launch_context(|ctx| ctx.pop_environment(snap));
    Ok(())
}

/// Apply the current timer delay to every capture appended since `mark`.
pub(crate) fn stamp_delay(mark: Lens) {
    let secs = current_delay();
    if secs <= 0.0 {
        return;
    }
    with_launch_context(|ctx| {
        for n in &mut ctx.captured_nodes_mut()[mark.nodes..] {
            n.start_delay_secs = Some(n.start_delay_secs.unwrap_or(0.0) + secs);
        }
        for c in &mut ctx.captured_containers_mut()[mark.containers..] {
            c.start_delay_secs = Some(c.start_delay_secs.unwrap_or(0.0) + secs);
        }
        for l in &mut ctx.captured_load_nodes_mut()[mark.load_nodes..] {
            l.start_delay_secs = Some(l.start_delay_secs.unwrap_or(0.0) + secs);
        }
    });
}

/// How many captures exist right now — a mark for [`stamp_delay`].
#[derive(Clone, Copy, Default)]
pub(crate) struct Lens {
    nodes: usize,
    containers: usize,
    load_nodes: usize,
}

pub(crate) fn lens() -> Lens {
    with_launch_context(|ctx| Lens {
        nodes: ctx.captured_nodes().len(),
        containers: ctx.captured_containers().len(),
        load_nodes: ctx.captured_load_nodes().len(),
    })
}

/// Names of what was captured since `mark`, for a diagnostic.
pub(crate) fn describe_since(mark: Lens) -> Vec<String> {
    with_launch_context(|ctx| {
        let mut out = Vec::new();
        for n in &ctx.captured_nodes()[mark.nodes..] {
            out.push(match &n.name {
                Some(name) => format!("{}/{} (name={name})", n.package, n.executable),
                None => format!("{}/{}", n.package, n.executable),
            });
        }
        for c in &ctx.captured_containers()[mark.containers..] {
            out.push(format!("container {}{}", c.namespace, c.name));
        }
        for l in &ctx.captured_load_nodes()[mark.load_nodes..] {
            out.push(format!(
                "composable {}/{} (name={})",
                l.package, l.plugin, l.node_name
            ));
        }
        out
    })
}

/// Visit the actions under a timer of `secs` seconds (if known).
pub(crate) fn with_delay<R>(secs: Option<f64>, f: impl FnOnce() -> R) -> R {
    match secs {
        Some(s) => {
            push_delay(s);
            let r = f();
            pop_delay();
            r
        }
        None => f(),
    }
}

/// Run an include NOW: hand what was produced so far and the state reached to
/// the traverser, which runs `file_path` with `args`, and continue in the
/// state it returns.
pub(crate) fn include(file_path: String, args: Vec<(String, String)>) -> PyResult<()> {
    let delay = current_delay();
    let (host, sync) = FRAMES.with(|f| {
        let f = f.borrow();
        let fr = f.last().expect("an include outside any execution");
        (fr.host, fr.sync)
    });
    let request = with_launch_context(|ctx| IncludeRequest {
        produced: drain_produced(ctx),
        state: ctx.export_state(&sync),
        file_path,
        args,
        delay_secs: (delay > 0.0).then_some(delay),
    });
    let sent = with_launch_context(|ctx| ctx.list_sync());
    // SAFETY: see `with_frame` — the frame, and so the borrow `host` came
    // from, is live for the whole walk this call is part of.
    let answer = unsafe { (*host).include(request) };
    let state = answer.map_err(PyRuntimeError::new_err)?;
    let new_sync = with_launch_context(|ctx| ctx.import_state(&state, &sent));
    FRAMES.with(|f| {
        if let Some(fr) = f.borrow_mut().last_mut() {
            fr.sync = new_sync;
        }
    });
    Ok(())
}

/// The entities to visit in `obj`: an action, a list or tuple of them, or a
/// `LaunchDescription`.
pub(crate) fn visit_any(py: Python, obj: &Bound<'_, PyAny>) -> PyResult<()> {
    if obj.is_none() {
        return Ok(());
    }
    if obj.is_instance_of::<PyList>() || obj.is_instance_of::<PyTuple>() {
        for item in obj.try_iter()? {
            visit_any(py, &item?)?;
        }
        return Ok(());
    }
    visit_entity(py, obj)
}

/// Whether `entity` carries a `condition=`, and whether it lets it execute.
fn condition_of(py: Python, entity: &Bound<'_, PyAny>) -> PyResult<Option<bool>> {
    let Ok(cond) = entity.getattr("condition") else {
        return Ok(None);
    };
    if cond.is_none() {
        return Ok(None);
    }
    crate::api::conditions::evaluate(py, &cond).map(Some)
}

/// Execute one entity, as `launch` visits it.
pub(crate) fn visit_entity(py: Python, entity: &Bound<'_, PyAny>) -> PyResult<()> {
    use crate::api::launch::LaunchDescription;

    if let Ok(ld) = entity.cast::<LaunchDescription>() {
        let actions = ld.borrow().actions.clone();
        for action in &actions {
            visit_any(py, action.bind(py))?;
        }
        return Ok(());
    }

    let conditional = match condition_of(py, entity)? {
        Some(false) => {
            log::debug!(
                "Skipping {} — its condition is false",
                entity
                    .get_type()
                    .name()
                    .map(|n| n.to_string())
                    .unwrap_or_default()
            );
            return Ok(());
        }
        Some(true) => true,
        None => false,
    };
    if conditional {
        CONDITIONAL_DEPTH.with(|d| d.set(d.get() + 1));
    }
    let result = execute_entity(py, entity);
    if conditional {
        CONDITIONAL_DEPTH.with(|d| d.set(d.get() - 1));
    }
    result
}

/// Do what `entity` does when executed.
fn execute_entity(py: Python, entity: &Bound<'_, PyAny>) -> PyResult<()> {
    use crate::api::{actions as a, launch_ros as r};

    macro_rules! exec {
        ($ty:ty) => {
            if let Ok(x) = entity.cast::<$ty>() {
                return <$ty>::execute(&x, py);
            }
        };
    }
    exec!(a::DeclareLaunchArgument);
    exec!(a::SetLaunchConfiguration);
    exec!(a::UnsetLaunchConfiguration);
    exec!(a::PushLaunchConfigurations);
    exec!(a::PopLaunchConfigurations);
    exec!(a::ResetLaunchConfigurations);
    exec!(a::SetEnvironmentVariable);
    exec!(a::UnsetEnvironmentVariable);
    exec!(a::AppendEnvironmentVariable);
    exec!(a::PushEnvironment);
    exec!(a::PopEnvironment);
    exec!(a::ResetEnvironment);
    exec!(a::GroupAction);
    exec!(a::IncludeLaunchDescription);
    exec!(a::OpaqueFunction);
    exec!(a::TimerAction);
    exec!(a::RegisterEventHandler);
    exec!(a::ExecuteProcess);
    exec!(a::LogInfo);
    exec!(r::RosTimer);
    exec!(r::PushRosNamespace);
    exec!(r::PopRosNamespace);
    exec!(r::SetParameter);
    exec!(r::SetParametersFromFile);
    exec!(r::SetRemap);
    exec!(r::SetUseSimTime);
    exec!(r::Node);
    exec!(r::LifecycleNode);
    exec!(r::ComposableNodeContainer);
    exec!(r::LoadComposableNodes);

    let class = entity
        .get_type()
        .name()
        .map(|n| n.to_string())
        .unwrap_or_else(|_| "<unknown>".to_string());
    log::trace!("Skipping entity this frontend does not execute: {class}");
    Ok(())
}
