//! Path/package substitution types
//!
//! TextSubstitution, PathJoinSubstitution, FindPackageShare, ThisLaunchFileDir,
//! FileContent, AnonName, ExecutableInPackage, FindPackage, LaunchLogDir, ThisLaunchFile

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

use crate::api::utils as sub_utils;

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

    #[getter]
    fn text(&self) -> String {
        self.text.clone()
    }

    /// `launch`'s: the text itself.
    fn perform(&self, _context: &Bound<'_, PyAny>) -> String {
        self.text.clone()
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

    /// `launch`'s: each component (itself a `SomeSubstitutionsType`)
    /// performed, then `str(pathlib.Path(*components))` — an absolute
    /// component restarts the path, as it does there.
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let mut parts = Vec::new();
        for obj in &self.substitutions {
            parts.push(sub_utils::perform_or_to_string(obj, py, context)?);
        }
        let path = py
            .import("pathlib")?
            .getattr("Path")?
            .call1(pyo3::types::PyTuple::new(py, parts)?)?;
        Ok(path.str()?.to_string())
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

    /// `launch_ros`'s: the package's share directory, the way `ament_index`
    /// finds it; a package it does not find is an error.
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let pkg_name = sub_utils::perform_or_to_string(&self.package_name, py, context)?;
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
        play_launch_parser::substitution::types::find_package_share(package_name)
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

    /// `launch`'s: the directory of the file being run (absolute). The file
    /// crosses the C ABI in `ContextState.current_file`; where none is set,
    /// the `$(dirname)` token the host rewrites once the file returns.
    fn perform(&self, _context: &Bound<'_, PyAny>) -> String {
        use play_launch_parser::substitution::types::Substitution;
        sub_utils::resolve_with_engine(Substitution::Dirname)
            .unwrap_or_else(|_| "$(dirname)".to_string())
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
pub struct AnonName {
    name: Py<PyAny>,
}

impl Clone for AnonName {
    fn clone(&self) -> Self {
        Python::attach(|py| Self {
            name: self.name.clone_ref(py),
        })
    }
}

#[pymethods]
impl AnonName {
    #[new]
    fn new(name: Py<PyAny>) -> Self {
        Self { name }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        Ok(format!(
            "$(anon {})",
            sub_utils::pyobject_to_string(py, &self.name)?
        ))
    }

    fn __repr__(&self, py: Python) -> String {
        let name =
            sub_utils::pyobject_to_string(py, &self.name).unwrap_or_else(|_| "<name>".to_string());
        format!("AnonName('{name}')")
    }

    /// `launch`'s: `<name>_<host>_<pid>_<random>`, computed once per name
    /// for the launch — the engine's `$(anon)`, so the Python and XML
    /// spellings of one name are the same name.
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        use play_launch_parser::substitution::types::Substitution;
        let name = sub_utils::perform_or_to_string(&self.name, py, context)?;
        sub_utils::resolve_with_engine(Substitution::Anon(vec![Substitution::Text(name)]))
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

        Ok(format!("$(exec-in-pkg {} {})", exec_str, pkg_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let pkg_str = sub_utils::pyobject_to_string(py, &self.package)
            .unwrap_or_else(|_| "<package>".to_string());
        let exec_str = sub_utils::pyobject_to_string(py, &self.executable)
            .unwrap_or_else(|_| "<executable>".to_string());
        format!("ExecutableInPackage('{}', '{}')", pkg_str, exec_str)
    }

    /// `launch_ros`'s: `<prefix>/lib/<package>/<executable>`, an error when
    /// it is not there (the engine's `$(exec-in-pkg)`).
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let pkg_str = sub_utils::perform_or_to_string(&self.package, py, context)?;
        let exec_str = sub_utils::perform_or_to_string(&self.executable, py, context)?;
        sub_utils::resolve_call("exec-in-pkg", vec![exec_str, pkg_str])
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

    /// `launch_ros`'s `FindPackagePrefix`: the package's install prefix,
    /// an error when `ament_index` does not find it.
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let pkg_str = sub_utils::perform_or_to_string(&self.package, py, context)?;
        sub_utils::resolve_call("find-pkg-prefix", vec![pkg_str])
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
        "$(launch_log_dir)".to_string()
    }

    fn __repr__(&self) -> String {
        "LaunchLogDir()".to_string()
    }

    /// The engine's `$(launch_log_dir)`: `ROS_LOG_DIR`, else `~/.ros/log`.
    fn perform(&self, _context: &Bound<'_, PyAny>) -> PyResult<String> {
        sub_utils::resolve_call("launch_log_dir", vec![])
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
        // The host's token for this launch file's absolute path. It is
        // `$(filename)` and not a token of its own because ROS 2 exposes
        // `ThisLaunchFile` to the frontends under exactly that name
        // (`@expose_substitution('filename')`), and its `perform` returns the
        // full `current_launch_file_path`. The host rewrites it in
        // `traverser/python_exec.rs` once execution of this file returns;
        // `perform` resolves it directly when the current file is known.
        // It used to return `$(this-launch-file)`, which nothing knew how to
        // read, so the literal reached the record and the command line
        // (issue 0041).
        "$(filename)".to_string()
    }

    fn __repr__(&self) -> String {
        "ThisLaunchFile()".to_string()
    }

    /// `launch`'s: the absolute path of the file being run; where none is
    /// set, the `$(filename)` token the host rewrites.
    fn perform(&self, _context: &Bound<'_, PyAny>) -> String {
        use play_launch_parser::substitution::types::Substitution;
        sub_utils::resolve_with_engine(Substitution::Filename)
            .unwrap_or_else(|_| "$(filename)".to_string())
    }
}
