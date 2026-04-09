use pyo3::exceptions::{PyFileNotFoundError, PyRuntimeError};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::collections::HashMap;
use std::path::PathBuf;

// Alias to avoid name collision with the #[pymodule] function.
use ::play_launch_parser as parser;

/// Find a launch file inside a ROS package by searching AMENT_PREFIX_PATH.
fn find_launch_file(package: &str, file: &str) -> Result<PathBuf, String> {
    let ament_paths = std::env::var("AMENT_PREFIX_PATH").map_err(|_| {
        format!(
            "AMENT_PREFIX_PATH is not set — source your ROS workspace first \
             (e.g. `source /opt/ros/humble/setup.bash`)"
        )
    })?;

    for prefix in ament_paths.split(':') {
        let path = PathBuf::from(prefix)
            .join("share")
            .join(package)
            .join("launch")
            .join(file);
        if path.exists() {
            return Ok(path);
        }
    }

    Err(format!(
        "Launch file '{file}' not found in package '{package}'. \
         Searched AMENT_PREFIX_PATH: {ament_paths}"
    ))
}

/// Convert a RecordJson to a Python dict.
fn record_to_dict(py: Python<'_>, record: &parser::record::RecordJson) -> PyResult<Py<PyDict>> {
    let value = serde_json::to_value(record)
        .map_err(|e| PyRuntimeError::new_err(format!("Serialization error: {e}")))?;
    let obj: Bound<'_, PyDict> = pythonize::pythonize(py, &value)
        .map_err(|e| PyRuntimeError::new_err(format!("Conversion error: {e}")))?
        .downcast_into()
        .map_err(|e| PyRuntimeError::new_err(format!("Expected dict, got: {e}")))?;
    Ok(obj.unbind())
}

/// Parse a ROS 2 launch file by path.
///
/// Args:
///     path: Path to the launch file (.launch.xml, .launch.py, .launch.yaml)
///     args: Optional dict of launch arguments (e.g. {"vehicle_model": "sample"})
///
/// Returns:
///     dict with keys: node, container, load_node, scopes, variables, file_data, lifecycle_node
#[pyfunction]
#[pyo3(signature = (path, args=None))]
fn parse_file(
    py: Python<'_>,
    path: &str,
    args: Option<HashMap<String, String>>,
) -> PyResult<Py<PyDict>> {
    let path = PathBuf::from(path);
    if !path.exists() {
        return Err(PyFileNotFoundError::new_err(format!(
            "Launch file not found: {}",
            path.display()
        )));
    }

    let cli_args = args.unwrap_or_default();
    let record = parser::parse_launch_file(&path, cli_args)
        .map_err(|e| PyRuntimeError::new_err(format!("Parse error: {e}")))?;

    record_to_dict(py, &record)
}

/// Parse a ROS 2 launch file by package name.
///
/// Searches AMENT_PREFIX_PATH for the launch file.
///
/// Args:
///     package: ROS package name (e.g. "autoware_launch")
///     file: Launch file name (e.g. "planning_simulator.launch.xml")
///     args: Optional dict of launch arguments
///
/// Returns:
///     dict with keys: node, container, load_node, scopes, variables, file_data, lifecycle_node
#[pyfunction]
#[pyo3(signature = (package, file, args=None))]
fn parse_package(
    py: Python<'_>,
    package: &str,
    file: &str,
    args: Option<HashMap<String, String>>,
) -> PyResult<Py<PyDict>> {
    let path =
        find_launch_file(package, file).map_err(|e| PyFileNotFoundError::new_err(e))?;

    let cli_args = args.unwrap_or_default();
    let record = parser::parse_launch_file(&path, cli_args)
        .map_err(|e| PyRuntimeError::new_err(format!("Parse error: {e}")))?;

    record_to_dict(py, &record)
}

/// High-performance ROS 2 launch file parser.
///
/// Functions:
///     parse_file(path, args=None) — parse a launch file by path
///     parse_package(package, file, args=None) — parse by ROS package name
#[pymodule]
fn play_launch_parser(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add_function(wrap_pyfunction!(parse_file, m)?)?;
    m.add_function(wrap_pyfunction!(parse_package, m)?)?;
    Ok(())
}
