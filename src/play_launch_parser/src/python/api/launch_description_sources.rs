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
#[pyclass(module = "launch.launch_description_sources")]
#[derive(Clone)]
pub struct PythonLaunchDescriptionSource {
    launch_file_path: PyObject,
}

#[pymethods]
impl PythonLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: PyObject) -> Self {
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

/// Helper to resolve a path from a PyObject (handles substitutions)
/// Shared by all LaunchDescriptionSource types
fn resolve_path(py: Python, path_obj: &PyObject) -> PyResult<String> {
    // Try to extract as string
    if let Ok(s) = path_obj.extract::<String>(py) {
        return Ok(s);
    }

    // Get context from __main__ namespace if available (created by OpaqueFunction)
    let context = py
        .import("__main__")
        .ok()
        .and_then(|m| m.dict().get_item("context").ok().flatten());

    let context_obj = context
        .map(|c| c.to_object(py))
        .unwrap_or_else(|| py.None());

    // Try to extract as list of substitutions
    if let Ok(list) = path_obj.bind(py).downcast::<pyo3::types::PyList>() {
        let mut parts = Vec::new();
        for item in list.iter() {
            // Try perform() first for substitutions like FindPackageShare
            if let Ok(result) = item.call_method1("perform", (context_obj.bind(py),)) {
                parts.push(result.extract::<String>()?);
            } else if let Ok(s) = item.extract::<String>() {
                parts.push(s);
            } else if let Ok(str_result) = item.call_method0("__str__") {
                parts.push(str_result.extract::<String>()?);
            }
        }
        return Ok(parts.join(""));
    }

    // Try calling perform() for substitutions (PathJoinSubstitution, FindPackageShare, etc.)
    if let Ok(result) = path_obj.call_method1(py, "perform", (context_obj.bind(py),))
        && let Ok(s) = result.extract::<String>(py)
    {
        return Ok(s);
    }

    // Fallback to __str__ method
    if let Ok(str_result) = path_obj.call_method0(py, "__str__")
        && let Ok(s) = str_result.extract::<String>(py)
    {
        return Ok(s);
    }

    // Last resort
    Ok(path_obj.to_string())
}

/// Mock XMLLaunchDescriptionSource class
///
/// Python equivalent:
/// ```python
/// from launch.launch_description_sources import XMLLaunchDescriptionSource
///
/// source = XMLLaunchDescriptionSource('/path/to/file.launch.xml')
/// ```
///
/// Represents an XML launch file source for IncludeLaunchDescription
#[pyclass(module = "launch.launch_description_sources")]
#[derive(Clone)]
pub struct XMLLaunchDescriptionSource {
    launch_file_path: PyObject,
}

#[pymethods]
impl XMLLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: PyObject) -> Self {
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
#[pyclass(module = "launch.launch_description_sources")]
#[derive(Clone)]
pub struct YAMLLaunchDescriptionSource {
    launch_file_path: PyObject,
}

#[pymethods]
impl YAMLLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: PyObject) -> Self {
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
#[pyclass(module = "launch.launch_description_sources")]
#[derive(Clone)]
pub struct FrontendLaunchDescriptionSource {
    launch_file_path: PyObject,
}

#[pymethods]
impl FrontendLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: PyObject) -> Self {
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
#[pyclass(module = "launch.launch_description_sources")]
#[derive(Clone)]
pub struct AnyLaunchDescriptionSource {
    launch_file_path: PyObject,
}

#[pymethods]
impl AnyLaunchDescriptionSource {
    #[new]
    fn new(launch_file_path: PyObject) -> Self {
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
