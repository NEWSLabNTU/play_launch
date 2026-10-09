//! Mock `launch.launch_description_sources` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

/// Mock PythonLaunchDescriptionSource class
///
/// Python equivalent:
/// ```python
/// from launch.launch_description_sources import PythonLaunchDescriptionSource
/// from launch.substitutions import PathJoinSubstitution
///
/// source = PythonLaunchDescriptionSource([
///     PathJoinSubstitution([
///         FindPackageShare('my_package'),
///         'launch',
///         'my_launch.launch.py'
///     ])
/// ])
/// ```
///
/// Represents a Python launch file source for IncludeLaunchDescription
#[pyclass(module = "launch.launch_description_sources", from_py_object)]
#[derive(Clone)]
pub struct PythonLaunchDescriptionSource {
    launch_file_path: Py<PyAny>,
}

#[pymethods]
impl PythonLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: Py<PyAny>) -> Self {
        Self { launch_file_path }
    }

    /// Get the launch file path
    ///
    /// In real ROS 2, this resolves substitutions and returns the path
    pub fn get_launch_file_path(&self, py: Python) -> PyResult<String> {
        resolve_path(py, &self.launch_file_path)
    }
}

impl PythonLaunchDescriptionSource {
    fn __repr__(&self) -> String {
        "PythonLaunchDescriptionSource(...)".to_string()
    }
}

/// Helper to resolve a path from a Py<PyAny> (handles substitutions)
/// Shared by all LaunchDescriptionSource types
fn resolve_path(py: Python, path_obj: &Py<PyAny>) -> PyResult<String> {
    // Performed against the context the walk is in when the include
    // executes, as `launch` performs a source's location.
    crate::api::utils::pyobject_to_string(py, path_obj)
        .map(|s| crate::api::utils::resolve_tokens(&s))
}

#[pyclass(module = "launch.launch_description_sources", from_py_object)]
#[derive(Clone)]
pub struct XMLLaunchDescriptionSource {
    launch_file_path: Py<PyAny>,
}

#[pymethods]
impl XMLLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: Py<PyAny>) -> Self {
        Self { launch_file_path }
    }

    /// Get the launch file path
    pub fn get_launch_file_path(&self, py: Python) -> PyResult<String> {
        resolve_path(py, &self.launch_file_path)
    }

    fn __repr__(&self) -> String {
        "XMLLaunchDescriptionSource(...)".to_string()
    }
}

/// Mock YAMLLaunchDescriptionSource class
///
/// Python equivalent:
/// ```python
/// from launch.launch_description_sources import YAMLLaunchDescriptionSource
///
/// source = YAMLLaunchDescriptionSource('/path/to/file.launch.yaml')
/// ```
///
/// Represents a YAML launch file source for IncludeLaunchDescription
#[pyclass(module = "launch.launch_description_sources", from_py_object)]
#[derive(Clone)]
pub struct YAMLLaunchDescriptionSource {
    launch_file_path: Py<PyAny>,
}

#[pymethods]
impl YAMLLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: Py<PyAny>) -> Self {
        Self { launch_file_path }
    }

    /// Get the launch file path
    pub fn get_launch_file_path(&self, py: Python) -> PyResult<String> {
        resolve_path(py, &self.launch_file_path)
    }

    fn __repr__(&self) -> String {
        "YAMLLaunchDescriptionSource(...)".to_string()
    }
}

/// Mock FrontendLaunchDescriptionSource class
///
/// Python equivalent:
/// ```python
/// from launch.launch_description_sources import FrontendLaunchDescriptionSource
///
/// source = FrontendLaunchDescriptionSource('/path/to/file.launch.xml')
/// ```
///
/// Automatically detects the launch file type based on extension
#[pyclass(module = "launch.launch_description_sources", from_py_object)]
#[derive(Clone)]
pub struct FrontendLaunchDescriptionSource {
    launch_file_path: Py<PyAny>,
}

#[pymethods]
impl FrontendLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: Py<PyAny>) -> Self {
        Self { launch_file_path }
    }

    /// Get the launch file path
    pub fn get_launch_file_path(&self, py: Python) -> PyResult<String> {
        resolve_path(py, &self.launch_file_path)
    }

    fn __repr__(&self) -> String {
        "FrontendLaunchDescriptionSource(...)".to_string()
    }
}

/// Mock AnyLaunchDescriptionSource class
///
/// Python equivalent:
/// ```python
/// from launch.launch_description_sources import AnyLaunchDescriptionSource
///
/// source = AnyLaunchDescriptionSource('/path/to/file.launch')
/// ```
///
/// Automatically detects the launch file type
#[pyclass(module = "launch.launch_description_sources", from_py_object)]
#[derive(Clone)]
pub struct AnyLaunchDescriptionSource {
    launch_file_path: Py<PyAny>,
}

#[pymethods]
impl AnyLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: Py<PyAny>) -> Self {
        Self { launch_file_path }
    }

    /// Get the launch file path
    pub fn get_launch_file_path(&self, py: Python) -> PyResult<String> {
        resolve_path(py, &self.launch_file_path)
    }

    fn __repr__(&self) -> String {
        "AnyLaunchDescriptionSource(...)".to_string()
    }
}
