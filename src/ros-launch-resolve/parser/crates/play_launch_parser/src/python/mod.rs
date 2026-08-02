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
pub mod bridge;
pub mod executor;

pub use executor::PythonLaunchExecutor;
