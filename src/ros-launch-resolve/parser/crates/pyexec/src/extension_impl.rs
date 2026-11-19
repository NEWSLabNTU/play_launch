//! `$(name …)` for a substitution only a launch extension defines (#0099).
//!
//! `launch` loads every module the `launch.frontend.launch_extension` entry
//! points name before it parses a frontend file, and each registers what it
//! exposes with `@expose_substitution(name)`. `$(robot-value r k)` in an XML
//! file is then `RobotValue.parse([[r], [k]])`, the object that builds,
//! performed in the launch context. This does the same with the extension's
//! own code, running on this crate's stand-ins for `launch`
//! (`crate::api::launch_extensions`).

use play_launch_parser::{
    bridge::LaunchContextGuard,
    exchange::{ContextState, ListSync},
    substitution::context::LaunchContext,
};
use pyo3::prelude::*;

/// Perform `$(name args…)` in `state`. `args` are the arguments, performed.
/// `Err` is `Unknown substitution type: <name>` when no extension exposes
/// `name`, followed by the extensions that could not be loaded, if any — one
/// of them may be the one that would have.
pub(crate) fn perform_substitution_pyo3(
    name: &str,
    args: &[String],
    state: &ContextState,
) -> Result<String, String> {
    // A context of this half's own, seeded with the caller's state, as
    // `exec_file` does: a `perform(context)` that reads
    // `context.launch_configurations` reads the caller's.
    let mut ctx = LaunchContext::new();
    let _ = ctx.import_state(state, &ListSync::default());
    let _guard = LaunchContextGuard::new(&mut ctx);
    Python::attach(|py| {
        let performed = perform(py, name, args).map_err(|e| format!("$({name}): {e}"))?;
        match performed {
            Some(value) => Ok(value),
            None => Err(unknown(py, name)),
        }
    })
}

fn perform(py: Python<'_>, name: &str, args: &[String]) -> PyResult<Option<String>> {
    crate::executor::ensure_mocks(py)?;
    let extensions = crate::api::launch_extensions(py)?;
    let context = Py::new(py, crate::api::utils::LaunchContextView)?;
    let value = extensions.call_method1("perform_substitution", (name, args.to_vec(), context))?;
    if value.is_none() {
        return Ok(None);
    }
    Ok(Some(value.extract::<String>()?))
}

/// No extension exposes `name`: `launch`'s answer, and the extensions that
/// failed to load, which `launch` only warns about.
fn unknown(py: Python<'_>, name: &str) -> String {
    let failures = || -> PyResult<Vec<String>> {
        crate::api::launch_extensions(py)?
            .getattr("failures")?
            .extract::<Vec<String>>()
    };
    let failures = failures().unwrap_or_default();
    let mut message = format!("Unknown substitution type: {name}");
    if !failures.is_empty() {
        message.push_str("\n\nThese launch extensions could not be loaded:");
        for failure in failures {
            message.push_str("\n  ");
            message.push_str(failure.trim_end());
        }
    }
    message
}
