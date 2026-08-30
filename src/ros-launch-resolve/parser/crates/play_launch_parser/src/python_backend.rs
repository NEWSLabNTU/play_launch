//! The seam between "parse launch files" and "execute Python".
//!
//! ROS 2 defines two things in terms of CPython, and both are load-bearing for
//! compatibility rather than conveniences we could drop:
//!
//!   * a `.launch.py` file IS a Python program, and
//!   * `$(eval …)` IS Python evaluation, per the launch specification — which
//!     means an **XML** launch file can need an interpreter too.
//!
//! So the parser cannot stop calling Python. What it can stop doing is *linking*
//! it unconditionally. Today `pyo3` is a hard dependency of this crate, so
//! `libpython` is a `DT_NEEDED` on every consumer — including `play_launch`'s
//! runtime, which then cannot start on a host without a matching interpreter
//! even to replay a recorded XML launch.
//!
//! This trait is the boundary that makes the dependency optional. With no
//! backend registered the parser still scans, still resolves XML and YAML, and
//! fails only where Python is genuinely required — naming what it could not do.
//!
//! # Why a process-global registry
//!
//! `$(eval …)` is evaluated deep inside the substitution engine, in a free
//! function with no access to a context object. Threading a backend parameter
//! through the whole substitution API to reach it would be a large, invasive
//! change to code that has nothing to do with Python. A global set once at
//! startup mirrors what the crate already does for the launch context
//! (`bridge`'s thread-local), and keeps this change confined to the seam.
//!
//! The cost of a global is a hidden dependency, and the mitigation is that
//! being unset is not a panic or a silent wrong answer — it is
//! [`PythonUnavailable`], which says which of the two capabilities was wanted.

use std::sync::OnceLock;

/// Executes the Python that ROS 2's launch format requires.
///
/// Implemented by the `pyo3`-linking half. Two methods rather than one because
/// the two call sites are genuinely different: one runs a file for its side
/// effects (captures land in the thread-local launch context), the other
/// evaluates an expression to a string.
pub trait PythonBackend: Send + Sync {
    /// Run a `.launch.py` file. Captures are written through `bridge`'s
    /// thread-local context, which the caller has already published.
    fn exec_file(&self, path: &str) -> Result<(), String>;

    /// Evaluate a `$(eval …)` expression to its string result.
    fn eval_expr(&self, expr: &str) -> Result<String, String>;
}

/// What was wanted, so the message can say so.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PythonNeed {
    /// A `.launch.py` file.
    LaunchFile,
    /// A `$(eval …)` substitution — reachable from XML and YAML too.
    EvalSubstitution,
}

impl PythonNeed {
    fn what(self) -> &'static str {
        match self {
            Self::LaunchFile => "a Python launch file",
            Self::EvalSubstitution => "a `$(eval …)` substitution",
        }
    }
}

/// No backend is registered, and something needed one.
#[derive(Debug, Clone)]
pub struct PythonUnavailable {
    pub need: PythonNeed,
    /// The launch file or expression that required it.
    pub subject: String,
}

impl std::fmt::Display for PythonUnavailable {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "no Python backend is loaded, and {} needs one: {}\n\
             \n\
             ROS 2 defines `.launch.py` files and `$(eval …)` in terms of CPython, so \
             these cannot be evaluated without an interpreter. XML and YAML launch files \
             that use neither are unaffected and were parsed normally.",
            self.need.what(),
            self.subject,
        )?;
        // WHY there is none, when the loader knows. Without this the message
        // reads as a deliberate build choice — "this build has no Python
        // backend" — on a machine that has Python installed and working, which
        // is the most common case and the least actionable phrasing for it.
        // The loader's own errors are written to be acted on (they name
        // $NROS_PYTHON, the missing half, the ABI mismatch); they were going to
        // `log::warn!` and being dropped, so the one place a user reads
        // reported a symptom with no cause.
        if let Some(reason) = unavailable_reason() {
            write!(f, "\n\nWhy no interpreter is loaded:\n{reason}")?;
        }
        Ok(())
    }
}

impl std::error::Error for PythonUnavailable {}

static BACKEND: OnceLock<Box<dyn PythonBackend>> = OnceLock::new();

/// Why no backend was registered, when whoever tried knows.
///
/// Set by the driver that attempted the load; read only when something
/// actually needed Python, so a launch tree that never touches it stays
/// silent. `OnceLock` for the same reason as `BACKEND`: the first attempt is
/// the informative one, and a later one cannot have more context.
static UNAVAILABLE_REASON: OnceLock<String> = OnceLock::new();

/// Record why the backend could not be installed. First call wins.
pub fn set_unavailable_reason(reason: impl Into<String>) {
    let _ = UNAVAILABLE_REASON.set(reason.into());
}

/// The recorded reason, if one was recorded.
#[must_use]
pub fn unavailable_reason() -> Option<&'static str> {
    UNAVAILABLE_REASON.get().map(String::as_str)
}

/// Register the backend. Idempotent: the first registration wins and later
/// ones are ignored. Returns whether THIS call installed it, so a caller that
/// cares about being first can tell — nothing in the parser does, because
/// `require` calls this on every use while `python/` is still in-crate.
pub fn set_backend(backend: Box<dyn PythonBackend>) -> bool {
    BACKEND.set(backend).is_ok()
}

/// The registered backend, or `None` in a build that has no Python half.
pub fn backend() -> Option<&'static dyn PythonBackend> {
    BACKEND.get().map(|b| b.as_ref())
}

/// The backend, or a [`PythonUnavailable`] naming what wanted it.
///
/// Registration is the CALLER's job (W2b).
///
/// While `python/` was a module of this crate it registered itself here
/// on first use, so no consumer had to do anything. Now it is a separate
/// crateate that this one cannot see even name, so the choice of WHETN a
/// build has a Python half is the caller's s — which is the whole point:
/// a build that never links `libpython` is exactly what the "no
/// interpreter" path needs, and it is now REPRESENTABLE.
pub fn require(
    need: PythonNeed,
    subject: &str,
) -> Result<&'static dyn PythonBackend, PythonUnavailable> {
    backend().ok_or_else(|| PythonUnavailable {
        need,
        subject: subject.to_string(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_names_the_capability_and_the_subject() {
        let e = PythonUnavailable {
            need: PythonNeed::EvalSubstitution,
            subject: "$(eval 1 + 1)".into(),
        };
        let s = e.to_string();
        assert!(s.contains("$(eval …)"), "names the capability: {s}");
        assert!(s.contains("$(eval 1 + 1)"), "names the subject: {s}");
        // The reassurance matters as much as the error: a user whose launch
        // tree is pure XML should not think the whole parse failed.
        assert!(s.contains("XML and YAML"), "says what still worked: {s}");
    }

    #[test]
    fn launch_file_need_reads_differently() {
        let e = PythonUnavailable {
            need: PythonNeed::LaunchFile,
            subject: "bringup.launch.py".into(),
        };
        assert!(e.to_string().contains("a Python launch file"));
    }
}
