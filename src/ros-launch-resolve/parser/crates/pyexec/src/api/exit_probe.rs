//! An `on_exit` that is a FUNCTION of the exit event (#0090).
//!
//! `launch` calls `on_exit(event, context)` when the process exits and runs
//! what it returns, so a handler can shut the launch down on one exit code
//! and let the process respawn on another. play_launch's runtime has no
//! Python, so the function cannot be called at the exit. It is called HERE
//! instead, once for every code a process can exit with (a status 0 to 255,
//! or `-N` for a death by signal N), and the codes it returned a shutdown
//! for are recorded; the runtime ends the launch on exactly those.
//!
//! That is faithful for a function of the exit code, which is what such
//! handlers are. It is refused, with a warning, for one that is not: a
//! function that raises, or that answers differently when asked twice
//! (it keeps state, such as a count of exits). Whatever else a function
//! returns (`LogInfo` aside) is not run, and is reported.

use play_launch_parser::record::{ExitCodes, all_exit_codes};
use pyo3::{
    prelude::*,
    types::{PyDict, PyList, PyTuple},
};
use std::collections::BTreeSet;

/// What evaluating an `on_exit` function found.
pub(crate) enum Probed {
    /// No exit ends the launch.
    Never,
    /// Every exit does: the same as `on_exit=Shutdown()`.
    Always,
    /// These exits do.
    Codes(ExitCodes),
    /// The function cannot be evaluated here; the reason.
    Refused(String),
}

/// One call's answer: whether it shut down, and the classes of the other
/// actions it returned.
struct Answer {
    shuts_down: bool,
    others: Vec<String>,
}

fn class_name(obj: &Bound<'_, PyAny>) -> String {
    obj.get_type()
        .name()
        .map(|n| n.to_string())
        .unwrap_or_else(|_| "<unknown>".to_string())
}

fn answer(result: &Bound<'_, PyAny>) -> Answer {
    let mut answer = Answer {
        shuts_down: false,
        others: Vec::new(),
    };
    if result.is_none() {
        return answer;
    }
    let items: Vec<Bound<'_, PyAny>> =
        if result.is_instance_of::<PyList>() || result.is_instance_of::<PyTuple>() {
            match result.try_iter() {
                Ok(it) => it.flatten().collect(),
                Err(_) => vec![result.clone()],
            }
        } else {
            vec![result.clone()]
        };
    for item in items {
        if crate::api::launch_ros::on_exit_shuts_down(&item) {
            answer.shuts_down = true;
        } else if !item.is_instance_of::<crate::api::actions::LogInfo>() && !item.is_none() {
            answer.others.push(class_name(&item));
        }
    }
    answer
}

/// Call `f(event, context)` for every exit code, with the event `launch`
/// would pass (`returncode`, `action`, `pid`, `cmd`, ...) and the walk's
/// context. `action` is the process the handler is for, `name` its name.
fn call_all(
    py: Python,
    f: &Bound<'_, PyAny>,
    action: Option<&Bound<'_, PyAny>>,
    name: &str,
) -> PyResult<Vec<Answer>> {
    let context = crate::api::utils::create_launch_context(py)?;
    let namespace = py.import("types")?.getattr("SimpleNamespace")?;
    let mut answers = Vec::new();
    for code in all_exit_codes() {
        let kwargs = PyDict::new(py);
        let action = action
            .map(|a| a.clone().into_any())
            .unwrap_or_else(|| py.None().into_bound(py));
        kwargs.set_item("action", &action)?;
        kwargs.set_item("execute_process_action", &action)?;
        kwargs.set_item("name", name)?;
        kwargs.set_item("process_name", name)?;
        kwargs.set_item("cmd", PyList::empty(py))?;
        kwargs.set_item("cwd", py.None())?;
        kwargs.set_item("env", py.None())?;
        kwargs.set_item("pid", 0)?;
        kwargs.set_item("returncode", code)?;
        let event = namespace.call((), Some(&kwargs))?;
        let result = f.call1((event, context.bind(py)))?;
        answers.push(answer(&result));
    }
    Ok(answers)
}

/// Evaluate `f` for every exit code. `name` is how a warning names the
/// process.
pub(crate) fn probe(
    py: Python,
    f: &Bound<'_, PyAny>,
    action: Option<&Bound<'_, PyAny>>,
    name: &str,
) -> Probed {
    // What the function prints is not the resolver's output (which may be
    // the model, on stdout); 640 calls of a `print` would be noise.
    let sys = match py.import("sys") {
        Ok(s) => s,
        Err(e) => return Probed::Refused(e.to_string()),
    };
    let saved = (sys.getattr("stdout").ok(), sys.getattr("stderr").ok());
    let sink = py
        .import("io")
        .and_then(|io| io.getattr("StringIO")?.call0());
    if let Ok(sink) = &sink {
        let _ = sys.setattr("stdout", sink);
        let _ = sys.setattr("stderr", sink);
    }
    // Twice: a function that keeps state answers differently the second
    // time, and its first answers say nothing about a real exit.
    let first = call_all(py, f, action, name);
    let second = first.as_ref().ok().map(|_| call_all(py, f, action, name));
    if let (Some(out), Some(err)) = saved {
        let _ = sys.setattr("stdout", out);
        let _ = sys.setattr("stderr", err);
    }
    let first = match first {
        Ok(a) => a,
        Err(e) => return Probed::Refused(format!("it raised {e}")),
    };
    let second = match second {
        Some(Ok(a)) => a,
        Some(Err(e)) => return Probed::Refused(format!("it raised {e} when called again")),
        None => unreachable!("the second pass runs whenever the first succeeded"),
    };
    if first
        .iter()
        .zip(&second)
        .any(|(a, b)| a.shuts_down != b.shuts_down)
    {
        return Probed::Refused(
            "it answered differently when called twice with the same exit code, so it \
             depends on more than the exit"
                .to_string(),
        );
    }

    let mut others: BTreeSet<String> = BTreeSet::new();
    let mut codes: BTreeSet<i32> = BTreeSet::new();
    for (code, a) in all_exit_codes().zip(&first) {
        if a.shuts_down {
            codes.insert(code);
        }
        others.extend(a.others.iter().cloned());
    }
    if !others.is_empty() {
        crate::api::utils::warn_user(&format!(
            "The on_exit function of {name} returns {} on some exits, which play_launch \
             does not run; only a shutdown it returns is honoured.",
            others.into_iter().collect::<Vec<_>>().join(", ")
        ));
    }
    match ExitCodes::from_codes(&codes) {
        None => Probed::Never,
        Some(c) if c.is_every_code() => Probed::Always,
        Some(c) => Probed::Codes(c),
    }
}
