//! Path/package substitution types
//!
//! TextSubstitution, PathJoinSubstitution, FindPackageShare, ThisLaunchFileDir,
//! FileContent, AnonName, ExecutableInPackage, FindPackage, LaunchLogDir, ThisLaunchFile

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

use crate::python::api::utils as sub_utils;

/// Mock TextSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import TextSubstitution
/// text = TextSubstitution(text='literal text')
/// ```
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct TextSubstitution {
    text: String,
}

#[pymethods]
impl TextSubstitution {
    #[new]
    #[pyo3(signature = (*, text))]
    fn new(text: String) -> Self {
        Self { text }
    }

    fn __str__(&self) -> String {
        self.text.clone()
    }

    fn __repr__(&self) -> String {
        format!("TextSubstitution(text='{}')", self.text)
    }
}

/// Mock PathJoinSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import PathJoinSubstitution
/// path = PathJoinSubstitution([part1, part2, part3])
/// ```
///
/// Joins path components using '/' separator
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct PathJoinSubstitution {
    substitutions: Vec<Py<PyAny>>,
}

#[pymethods]
impl PathJoinSubstitution {
    #[new]
    fn new(substitutions: Vec<Py<PyAny>>) -> Self {
        Self { substitutions }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        // Convert each substitution to string and join with '/'
        let mut parts = Vec::new();
        for obj in &self.substitutions {
            let part = if let Ok(s) = obj.extract::<String>(py) {
                s
            } else {
                // Try calling __str__ on the object
                obj.call_method0(py, "__str__")?.extract::<String>(py)?
            };
            parts.push(part);
        }
        Ok(parts.join("/"))
    }

    fn __repr__(&self) -> String {
        format!("PathJoinSubstitution({} parts)", self.substitutions.len())
    }

    /// Perform the substitution - resolve all parts and join with '/'
    ///
    /// This method calls perform() on each substitution if available, or falls back to __str__
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let mut parts = Vec::new();
        for obj in &self.substitutions {
            // Try to call perform() if available (for FindPackageShare, LaunchConfiguration, etc.)
            let part = if let Ok(result) = obj.call_method1(py, "perform", (context,)) {
                result.extract::<String>(py)?
            } else if let Ok(s) = obj.extract::<String>(py) {
                s
            } else {
                // Fallback to __str__
                obj.call_method0(py, "__str__")?.extract::<String>(py)?
            };
            parts.push(part);
        }
        Ok(parts.join("/"))
    }
}

/// Mock FindPackageShare substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import FindPackageShare
/// pkg_path = FindPackageShare('package_name')
/// # or
/// pkg_path = FindPackageShare(LaunchConfiguration('pkg_name'))
/// ```
///
/// Returns substitution format: `$(find-pkg-share package_name)`
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct FindPackageShare {
    package_name: Py<PyAny>,
}

#[pymethods]
impl FindPackageShare {
    #[new]
    fn new(package_name: Py<PyAny>) -> Self {
        Self { package_name }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        use pyo3::types::PyList;

        // Extract package name (could be string, substitution, or list)
        let pkg = if let Ok(s) = self.package_name.extract::<String>(py) {
            s
        } else if let Ok(list) = self.package_name.bind(py).cast::<PyList>() {
            // Handle list case: concatenate all elements using __str__()
            let mut result = String::new();
            for item in list.iter() {
                // Try to extract as string
                if let Ok(s) = item.extract::<String>() {
                    result.push_str(&s);
                }
                // Try calling __str__
                else if let Ok(str_result) = item.call_method0("__str__")
                    && let Ok(s) = str_result.extract::<String>()
                {
                    result.push_str(&s);
                }
            }
            result
        } else {
            // Try calling __str__ on the object (for LaunchConfiguration, etc.)
            self.package_name
                .call_method0(py, "__str__")?
                .extract::<String>(py)?
        };
        Ok(format!("$(find-pkg-share {})", pkg))
    }

    fn __repr__(&self) -> String {
        "FindPackageShare(...)".to_string()
    }

    /// Perform the substitution - resolve to actual package path
    ///
    /// In ROS 2, this method is called with a launch context to resolve the package path.
    /// We implement the same package finding logic as the Rust substitution system.
    fn perform(&self, py: Python, _context: &Bound<'_, PyAny>) -> PyResult<String> {
        use pyo3::types::PyList;

        // Extract package name (could be string, LaunchConfiguration, or list)
        let pkg_name = if let Ok(s) = self.package_name.extract::<String>(py) {
            s
        } else if let Ok(list) = self.package_name.bind(py).cast::<PyList>() {
            // Handle list case: concatenate all elements after performing them
            let mut result = String::new();
            for item in list.iter() {
                // Try to extract as string
                if let Ok(s) = item.extract::<String>() {
                    result.push_str(&s);
                }
                // Try calling perform() if it exists (for LaunchConfiguration, etc.)
                else if item.hasattr("perform")? {
                    if let Ok(performed) = item.call_method1("perform", (_context,))
                        && let Ok(s) = performed.extract::<String>()
                    {
                        result.push_str(&s);
                        continue;
                    }
                    // Fallback to __str__ if perform fails
                    if let Ok(str_result) = item.call_method0("__str__")
                        && let Ok(s) = str_result.extract::<String>()
                    {
                        result.push_str(&s);
                    }
                }
                // Try calling __str__
                else if let Ok(str_result) = item.call_method0("__str__")
                    && let Ok(s) = str_result.extract::<String>()
                {
                    result.push_str(&s);
                }
            }
            result
        } else {
            // Try calling perform() on the object if it has it (for LaunchConfiguration)
            let obj_ref = self.package_name.bind(py);
            if obj_ref.hasattr("perform")? {
                if let Ok(result) = obj_ref.call_method1("perform", (_context,)) {
                    result.extract::<String>()?
                } else {
                    // Fallback to __str__
                    obj_ref.call_method0("__str__")?.extract::<String>()?
                }
            } else {
                // Fallback to __str__
                obj_ref.call_method0("__str__")?.extract::<String>()?
            }
        };

        // Resolve package path using ROS package finding logic
        Self::find_package_share(&pkg_name).ok_or_else(|| {
            pyo3::exceptions::PyFileNotFoundError::new_err(format!(
                "Package '{}' not found. Ensure the package is installed and sourced.",
                pkg_name
            ))
        })
    }
}

impl FindPackageShare {
    /// Find ROS 2 package share directory (delegates to shared implementation with caching)
    fn find_package_share(package_name: &str) -> Option<String> {
        crate::substitution::types::find_package_share(package_name)
    }
}

/// Mock ThisLaunchFileDir substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import ThisLaunchFileDir
/// launch_dir = ThisLaunchFileDir()
/// ```
///
/// Returns substitution format: `$(dirname)`
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct ThisLaunchFileDir {}

#[pymethods]
impl ThisLaunchFileDir {
    #[new]
    fn new() -> Self {
        Self {}
    }

    fn __str__(&self) -> String {
        "$(dirname)".to_string()
    }

    fn __repr__(&self) -> String {
        "ThisLaunchFileDir()".to_string()
    }
}

/// Mock FileContent substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import FileContent
/// content = FileContent(PathJoinSubstitution([...]))
/// ```
///
/// Reads file contents and returns as string
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct FileContent {
    path: Py<PyAny>,
}

#[pymethods]
impl FileContent {
    #[new]
    fn new(path: Py<PyAny>) -> Self {
        Self { path }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        // Try to get the path as string
        let path_str = if let Ok(s) = self.path.extract::<String>(py) {
            s
        } else if let Ok(str_result) = self.path.call_method0(py, "__str__") {
            str_result.extract::<String>(py)?
        } else {
            return Ok(String::new());
        };

        // Read file contents
        match std::fs::read_to_string(&path_str) {
            Ok(content) => Ok(content),
            Err(e) => {
                log::warn!("FileContent: Failed to read file '{}': {}", path_str, e);
                // Return empty string on error (graceful degradation)
                Ok(String::new())
            }
        }
    }

    fn __repr__(&self) -> String {
        "FileContent(...)".to_string()
    }

    /// Perform the substitution - resolve path and read file contents
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let path_obj_ref = self.path.bind(py);

        // Try to call perform() on the path if it has it (for substitutions)
        let path_str = if path_obj_ref.hasattr("perform")? {
            if let Ok(result) = path_obj_ref.call_method1("perform", (context,)) {
                result.extract::<String>()?
            } else {
                // Fallback to __str__
                path_obj_ref.call_method0("__str__")?.extract::<String>()?
            }
        } else if let Ok(s) = self.path.extract::<String>(py) {
            s
        } else {
            // Fallback to __str__
            path_obj_ref.call_method0("__str__")?.extract::<String>()?
        };

        log::debug!("FileContent: Reading file '{}'", path_str);

        // Read file contents
        match std::fs::read_to_string(&path_str) {
            Ok(content) => {
                log::debug!(
                    "FileContent: Successfully read {} bytes from '{}'",
                    content.len(),
                    path_str
                );
                Ok(content)
            }
            Err(e) => {
                log::warn!("FileContent: Failed to read file '{}': {}", path_str, e);
                // Return empty string on error for graceful degradation
                // This matches ROS 2 behavior where missing files don't crash the parser
                Ok(String::new())
            }
        }
    }
}

/// Mock AnonName substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import AnonName
/// anon = AnonName('my_node')
/// ```
///
/// Generates an anonymous name with a random suffix
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct AnonName {
    name: String,
}

#[pymethods]
impl AnonName {
    #[new]
    fn new(name: String) -> Self {
        Self { name }
    }

    fn __str__(&self) -> String {
        format!("$(anon {})", self.name)
    }

    fn __repr__(&self) -> String {
        format!("AnonName('{}')", self.name)
    }
}

/// Mock ExecutableInPackage substitution
///
/// Python equivalent:
/// ```python
/// from launch_ros.substitutions import ExecutableInPackage
/// exec_path = ExecutableInPackage(package='my_pkg', executable='my_node')
/// ```
///
/// Finds the full path to an executable within a ROS package
#[pyclass(module = "launch_ros.substitutions", from_py_object)]
#[derive(Clone)]
pub struct ExecutableInPackage {
    package: Py<PyAny>,
    executable: Py<PyAny>,
}

#[pymethods]
impl ExecutableInPackage {
    #[new]
    #[pyo3(signature = (package, executable, **_kwargs))]
    fn new(
        package: Py<PyAny>,
        executable: Py<PyAny>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        Self {
            package,
            executable,
        }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let pkg_str = sub_utils::pyobject_to_string(py, &self.package)?;
        let exec_str = sub_utils::pyobject_to_string(py, &self.executable)?;

        // Return a placeholder path that represents the executable location
        // In static analysis, we can't actually find the executable
        Ok(format!("$(find-exec {} {})", pkg_str, exec_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let pkg_str = sub_utils::pyobject_to_string(py, &self.package)
            .unwrap_or_else(|_| "<package>".to_string());
        let exec_str = sub_utils::pyobject_to_string(py, &self.executable)
            .unwrap_or_else(|_| "<executable>".to_string());
        format!("ExecutableInPackage('{}', '{}')", pkg_str, exec_str)
    }

    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let pkg_str = sub_utils::perform_or_to_string(&self.package, py, context)?;
        let exec_str = sub_utils::perform_or_to_string(&self.executable, py, context)?;

        // Return a placeholder path
        Ok(format!("$(find-exec {} {})", pkg_str, exec_str))
    }
}

/// Mock FindPackage substitution
///
/// Python equivalent:
/// ```python
/// from launch_ros.substitutions import FindPackage
/// pkg_prefix = FindPackage('my_pkg')
/// ```
///
/// Finds the install prefix path of a ROS package (different from FindPackageShare)
#[pyclass(module = "launch_ros.substitutions", from_py_object)]
#[derive(Clone)]
pub struct FindPackage {
    package: Py<PyAny>,
}

#[pymethods]
impl FindPackage {
    #[new]
    fn new(package: Py<PyAny>) -> Self {
        Self { package }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let pkg_str = sub_utils::pyobject_to_string(py, &self.package)?;
        Ok(format!("$(find-pkg-prefix {})", pkg_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let pkg_str = sub_utils::pyobject_to_string(py, &self.package)
            .unwrap_or_else(|_| "<package>".to_string());
        format!("FindPackage('{}')", pkg_str)
    }

    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let pkg_str = sub_utils::perform_or_to_string(&self.package, py, context)?;
        Ok(format!("$(find-pkg-prefix {})", pkg_str))
    }
}

/// Mock LaunchLogDir substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import LaunchLogDir
/// log_dir = LaunchLogDir()
/// ```
///
/// Returns the launch log directory path
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct LaunchLogDir {}

#[pymethods]
impl LaunchLogDir {
    #[new]
    fn new() -> Self {
        Self {}
    }

    fn __str__(&self) -> String {
        // Return placeholder - can't determine actual log dir in static analysis
        "$(launch-log-dir)".to_string()
    }

    fn __repr__(&self) -> String {
        "LaunchLogDir()".to_string()
    }

    fn perform(&self, _py: Python, _context: &Bound<'_, PyAny>) -> PyResult<String> {
        Ok("$(launch-log-dir)".to_string())
    }
}

/// Mock ThisLaunchFile substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import ThisLaunchFile
/// this_file = ThisLaunchFile()
/// ```
///
/// Returns the full path to the current launch file
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct ThisLaunchFile {}

#[pymethods]
impl ThisLaunchFile {
    #[new]
    fn new() -> Self {
        Self {}
    }

    fn __str__(&self) -> String {
        // Return placeholder - actual path would be set during parsing
        "$(this-launch-file)".to_string()
    }

    fn __repr__(&self) -> String {
        "ThisLaunchFile()".to_string()
    }

    fn perform(&self, _py: Python, _context: &Bound<'_, PyAny>) -> PyResult<String> {
        Ok("$(this-launch-file)".to_string())
    }
}
