use pyo3::exceptions::{PyFileNotFoundError, PyRuntimeError, PySystemExit};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::collections::HashMap;
use std::io::Write;
use std::path::PathBuf;

// Alias to avoid name collision with the #[pymodule] function.
use ::play_launch_parser as parser;
use parser::record::RecordJson;

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
fn record_to_dict(py: Python<'_>, record: &RecordJson) -> PyResult<Py<PyDict>> {
    let value = serde_json::to_value(record)
        .map_err(|e| PyRuntimeError::new_err(format!("Serialization error: {e}")))?;
    let obj: Bound<'_, PyDict> = pythonize::pythonize(py, &value)
        .map_err(|e| PyRuntimeError::new_err(format!("Conversion error: {e}")))?
        .cast_into()
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

// ── CLI implementation (exposed as Python entry point) ──

fn node_fqn(namespace: Option<&str>, name: Option<&str>) -> String {
    let ns = namespace.unwrap_or("/");
    let n = name.unwrap_or("?");
    if ns == "/" {
        format!("/{n}")
    } else {
        format!("{ns}/{n}")
    }
}

fn format_summary(record: &RecordJson, launch_label: &str) -> String {
    let mut out = String::new();
    out.push_str(&format!("Launch: {launch_label}\n\n"));
    out.push_str(&format!("  Nodes:        {}\n", record.node.len()));
    out.push_str(&format!("  Containers:   {}\n", record.container.len()));
    out.push_str(&format!("  Composable:   {}\n", record.load_node.len()));
    out.push_str(&format!("  Scopes:       {}\n", record.scopes.len()));
    out.push_str(&format!("  Variables:    {}\n", record.variables.len()));

    if !record.node.is_empty() {
        out.push_str("\nNodes:\n");
        for node in &record.node {
            let fqn = node_fqn(node.namespace.as_deref(), node.name.as_deref());
            let pkg = node.package.as_deref().unwrap_or("-");
            out.push_str(&format!("  {fqn:<60} {pkg}\n"));
        }
    }
    if !record.container.is_empty() {
        out.push_str("\nContainers:\n");
        for c in &record.container {
            let fqn = node_fqn(Some(&c.namespace), Some(&c.name));
            out.push_str(&format!("  {fqn:<60} {}\n", c.package));
        }
    }
    if !record.load_node.is_empty() {
        out.push_str("\nComposable Nodes:\n");
        for ln in &record.load_node {
            let fqn = node_fqn(Some(&ln.namespace), Some(&ln.node_name));
            out.push_str(&format!(
                "  {fqn:<60} {} -> {}\n",
                ln.plugin, ln.target_container_name
            ));
        }
    }
    out
}

fn format_names(record: &RecordJson) -> String {
    let mut out = String::new();
    for node in &record.node {
        out.push_str(&node_fqn(node.namespace.as_deref(), node.name.as_deref()));
        out.push('\n');
    }
    for c in &record.container {
        out.push_str(&node_fqn(Some(&c.namespace), Some(&c.name)));
        out.push('\n');
    }
    for ln in &record.load_node {
        out.push_str(&node_fqn(Some(&ln.namespace), Some(&ln.node_name)));
        out.push('\n');
    }
    out
}

/// CLI entry point. Parses sys.argv-style arguments and writes to stdout/file.
///
/// Raises SystemExit with the appropriate exit code.
#[pyfunction]
#[pyo3(signature = (args))]
fn _cli_main(args: Vec<String>) -> PyResult<()> {
    use clap::{Parser, Subcommand, ValueEnum};

    #[derive(Parser)]
    #[command(name = "play-launch-parser")]
    #[command(about = "High-performance ROS 2 launch file parser")]
    #[command(version = env!("CARGO_PKG_VERSION"))]
    struct Cli {
        #[command(subcommand)]
        command: Commands,

        #[arg(short, long, global = true)]
        verbose: bool,

        #[arg(short, long, global = true)]
        quiet: bool,

        #[arg(short, long, value_enum, default_value_t = Format::Json, global = true)]
        format: Format,

        #[arg(short, long, global = true)]
        output: Option<PathBuf>,
    }

    #[derive(Clone, ValueEnum)]
    enum Format {
        Json,
        Summary,
        Names,
    }

    #[derive(Subcommand)]
    enum Commands {
        Launch {
            package: String,
            file: String,
            #[arg(value_parser = parse_kv_arg)]
            args: Vec<(String, String)>,
        },
        File {
            path: PathBuf,
            #[arg(value_parser = parse_kv_arg)]
            args: Vec<(String, String)>,
        },
    }

    fn parse_kv_arg(s: &str) -> Result<(String, String), String> {
        let parts: Vec<&str> = s.split(":=").collect();
        if parts.len() != 2 {
            return Err(format!("Invalid argument format (expected key:=value): {s}"));
        }
        Ok((parts[0].to_string(), parts[1].to_string()))
    }

    let cli = Cli::try_parse_from(&args).map_err(|e| {
        // clap prints help/version/error to stderr, then we exit
        let _ = e.print();
        PySystemExit::new_err(2)
    })?;

    // Resolve launch file
    let (launch_path, launch_label) = match &cli.command {
        Commands::Launch { package, file, .. } => {
            match find_launch_file(package, file) {
                Ok(path) => (path, format!("{file} ({package})")),
                Err(e) => {
                    eprintln!("Error: {e}");
                    return Err(PySystemExit::new_err(2));
                }
            }
        }
        Commands::File { path, .. } => {
            if !path.exists() {
                eprintln!("Error: file not found: {}", path.display());
                return Err(PySystemExit::new_err(2));
            }
            let label = path
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("unknown")
                .to_string();
            (path.clone(), label)
        }
    };

    let cli_args: HashMap<String, String> = match &cli.command {
        Commands::Launch { args, .. } | Commands::File { args, .. } => {
            args.iter().cloned().collect()
        }
    };

    let record = match parser::parse_launch_file(&launch_path, cli_args) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Error: {e}");
            return Err(PySystemExit::new_err(1));
        }
    };

    let output_text = match cli.format {
        Format::Json => record
            .to_json()
            .map_err(|e| PyRuntimeError::new_err(format!("JSON error: {e}")))?,
        Format::Summary => format_summary(&record, &launch_label),
        Format::Names => format_names(&record),
    };

    if let Some(output_path) = &cli.output {
        std::fs::write(output_path, &output_text).map_err(|e| {
            PyRuntimeError::new_err(format!("Write error: {e}"))
        })?;
    } else {
        let stdout = std::io::stdout();
        let mut handle = stdout.lock();
        handle.write_all(output_text.as_bytes()).ok();
        handle.flush().ok();
    }

    Ok(())
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
    m.add_function(wrap_pyfunction!(_cli_main, m)?)?;
    Ok(())
}
