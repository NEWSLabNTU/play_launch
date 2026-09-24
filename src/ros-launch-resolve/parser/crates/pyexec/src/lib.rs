//! Executes the Python that ROS 2's launch format defines.
//!
//! Split from `play_launch_parser` (nano-ros issue 0897 W2b) so that
//! `pyo3` — and therefore `libpython` — is a dependency of THIS crate
//! and not of the parser. A build with no Python half links no
//! interpreter, and a launch tree that is pure XML or YAML resolves
//! with no interpreter present.
//!
//! Reimplements `launch` / `launch_ros` / `launch_xml` in Rust and
//! injects them into `sys.modules` behind a `sys.meta_path` blocker, so
//! the real ROS packages can never load — which is how a `.launch.py`
//! runs with no ROS installation at all.

//! Python launch file support via pyo3
//!
//! This module provides support for executing Python launch files by embedding
//! a Python interpreter and mocking the ROS 2 `launch` and `launch_ros` APIs.
//!
//! ## Architecture
//!
//! - **Executor**: Manages Python interpreter and executes .py files
//! - **Bridge**: Converts between Python and Rust types
//! - **API**: Mock Python classes that capture node definitions
//!
//! ## Strategy
//!
//! Instead of parsing Python syntax, we execute Python files in an embedded
//! interpreter with our own mock API. When Python code creates a `Node`, our
//! mock class captures it immediately (capture-on-construction pattern).

pub mod api;
// The extern "C" surface a driver dlopens (0897 W3).
pub mod c_abi;
pub mod eval_impl;
pub mod executor;

pub use executor::PythonLaunchExecutor;

/// The pyo3 half, as a [`play_launch_parser::python_backend::PythonBackend`].
///
/// While `python/` still lives in this crate (issue 0897 W1) this is registered
/// automatically, so behaviour is unchanged for every existing consumer. When
/// the module moves to its own crate (W2) the registration becomes the loader's
/// job and this impl travels with it — the trait, not the module path, is what
/// the parser depends on.
pub struct Pyo3Backend;

impl play_launch_parser::python_backend::PythonBackend for Pyo3Backend {
    fn exec_file(&self, path: &str) -> Result<(), String> {
        executor::PythonLaunchExecutor::new()
            .execute(path)
            .map_err(|e| e.to_string())
    }

    fn eval_expr(&self, expr: &str) -> Result<String, String> {
        eval_impl::eval_expr_pyo3(expr).map_err(|e| e.to_string())
    }
}

/// Register the pyo3 backend if nothing has claimed the slot yet.
///
/// Called from the parser's entry points rather than a `ctor`: this repo builds
/// for wasm and for a Python extension module too, and a constructor that runs
/// in every one of those is the kind of hidden initialisation that is hard to
/// opt out of.
pub fn register_default_backend() {
    play_launch_parser::python_backend::set_backend(Box::new(Pyo3Backend));
}

/// Install this crate as the parser's Python backend.
///
/// The parser calls Python for the two things ROS 2 defines that way —
/// `.launch.py` files and `$(eval …)` — and since 0897 W2b it does so
/// through a trait rather than a direct dependency. This fills it in.
///
/// Idempotent. A consumer that never calls it gets a parser that
/// resolves XML and YAML and reports `PythonUnavailable` for those
/// two, which is the point of the split: the dependency is the
/// CALLER's s to take on, and `libpython` is no longer a `DT_NEEDED`
/// of the parser.
pub fn register() {
    play_launch_parser::python_backend::set_backend(Box::new(Pyo3Backend));
}

/// Serialise a test that drives the embedded interpreter, and make sure a
/// backend is registered before it runs.
///
/// The interpreter and the parser's `LaunchContext` bridge are PROCESS state,
/// and the default test harness runs a binary's tests as threads of one
/// process — so two tests executing a `.launch.py` at once interleave inside
/// one interpreter. What that looks like is not a crash: `c_abi`'s ABI-4 test
/// came back `KeyError: 'rear_overhang'` about one run in three, which is
/// issue #0028's exact symptom, so the flake read as a regression of a
/// shipped fix (issue #0050).
///
/// Defined HERE, in the crate that owns the interpreter, rather than copied
/// into each test module: `play_launch_parser`'s Python suites already depend
/// on this crate (they call [`register`]), so one definition covers every
/// test binary that can start an interpreter. Two independently-written locks
/// over one interpreter is not a fix.
///
/// Recovers from a poisoned mutex — a test that panicked while holding it has
/// already failed, and poisoning every test after it only hides which one.
#[doc(hidden)]
pub fn python_test_guard() -> std::sync::MutexGuard<'static, ()> {
    static PYTHON_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    let guard = PYTHON_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // Under the lock: `set_backend` writes a global, and registration is the
    // caller's job since 0897 W2b. Doing it in the guard every Python test
    // already takes means there is one place to say it.
    register();
    guard
}
