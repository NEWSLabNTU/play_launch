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
pub mod eval_impl;
pub mod executor;

pub use executor::PythonLaunchExecutor;

/// The pyo3 half, as a [`crate::python_backend::PythonBackend`].
///
/// While `python/` still lives in this crate (issue 0897 W1) this is registered
/// automatically, so behaviour is unchanged for every existing consumer. When
/// the module moves to its own crate (W2) the registration becomes the loader's
/// job and this impl travels with it — the trait, not the module path, is what
/// the parser depends on.
pub struct Pyo3Backend;

impl crate::python_backend::PythonBackend for Pyo3Backend {
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
    crate::python_backend::set_backend(Box::new(Pyo3Backend));
}
