//! Python launch file executor
//!
//! Executes Python launch files with PyO3 mocks to capture node definitions.

use play_launch_parser::error::Result;
use pyo3::prelude::*;
use std::ffi::CString;

/// Executes Python launch files with mock API
#[derive(Default)]

pub struct PythonLaunchExecutor;

impl PythonLaunchExecutor {
    /// Create new executor
    pub fn new() -> Self {
        Self
    }

    /// Execute a Python launch file and capture entities
    pub fn execute(&self, launch_file_path: &str) -> Result<()> {
        Python::attach(|py| {
            log::debug!("Executing Python launch file: {}", launch_file_path);

            // Register PyO3 mock modules in sys.modules
            crate::api::register_modules(py).map_err(py_err)?;

            // CRITICAL: Aggressively isolate Python environment to prevent real ROS packages from loading
            let isolation_code = r#"
import sys
import types

# STEP 0: Disable bytecode caching to prevent stale imports
sys.dont_write_bytecode = True

# STEP 1: Install import hook to block only launch.* filesystem searches
# This allows other ROS packages (ament_index_python, etc.) to load normally
import sys
import importlib.util
import importlib.machinery

class LaunchModuleBlocker:
    """Blocks filesystem imports of launch.* modules while allowing sys.modules"""
    def find_spec(self, fullname, path, target=None):
        # Only intercept launch.* module imports
        if fullname.startswith(('launch', 'launch_ros', 'launch_xml')):
            # If it's already in sys.modules (our mock), let default machinery handle it
            if fullname in sys.modules:
                return None  # Continue with default import machinery
            # Block filesystem search by returning a failing spec
            # This prevents real ROS launch modules from being found on sys.path
            raise ImportError(f"Blocked filesystem import of {fullname} (use mocks only)")
        # Allow all other imports to proceed normally
        return None

# Install at beginning of sys.meta_path to intercept before filesystem finders
if not any(isinstance(f, LaunchModuleBlocker) for f in sys.meta_path):
    sys.meta_path.insert(0, LaunchModuleBlocker())

# STEP 2: Remove existing launch* submodules from sys.modules that aren't our mocks
# This clears any cached imports from real ROS packages
# Keep our registered top-level and submodule mocks
our_mocks = {
    'launch', 'launch.actions', 'launch.substitutions', 'launch.conditions',
    'launch.event_handlers', 'launch.events', 'launch.events.process',
    'launch.launch_context', 'launch.launch_description_sources',
    'launch.frontend', 'launch.frontend.type_utils', 'launch.utilities',
    'launch.utilities.type_utils', 'launch.some_substitutions_type',
    'launch_ros', 'launch_ros.actions', 'launch_ros.descriptions',
    'launch_ros.substitutions', 'launch_ros.parameter_descriptions', 'launch_ros.utilities',
    'launch_xml', 'launch_xml.launch_description_sources',
}

to_delete = [k for k in list(sys.modules.keys())
             if k.startswith(('launch.', 'launch_ros.', 'launch_xml.'))
             and k not in our_mocks]

for key in to_delete:
    del sys.modules[key]

# STEP 4: Clear import caches to force fresh lookups
import importlib
importlib.invalidate_caches()

# STEP 5: Verify our mocks are properly registered and accessible
def _verify_mocks():
    """Verify that our mocks are accessible and have correct attributes"""
    try:
        from launch.actions import GroupAction
        # Test that actions attribute exists
        ga = GroupAction([])
        if hasattr(ga, 'actions'):
            return True, f"OK: GroupAction.actions exists, module={GroupAction.__module__}"
        else:
            return False, f"ERROR: GroupAction missing .actions, module={GroupAction.__module__}"
    except Exception as e:
        return False, f"ERROR: {e}"

_ok, _msg = _verify_mocks()
if not _ok:
    raise RuntimeError(f"Mock verification failed: {_msg}")
"#;

            // IMPORTANT: Run isolation in the GLOBAL context first so sys.modules is clean
            let isolation_cstr = CString::new(isolation_code).expect("isolation code contains NUL");
            py.run(&isolation_cstr, None, None).map_err(py_err)?;
            log::debug!("Installed aggressive Python environment isolation for launch* mocks");

            // Launch configurations are already in the thread-local LaunchContext
            // (set by execute_python_file() before calling this executor)

            // Execute the Python file directly with exec() instead of runpy
            // This gives us complete control over the execution environment
            use std::fs;
            let code = fs::read_to_string(launch_file_path).map_err(|e| {
                play_launch_parser::error::ParseError::PythonError(format!(
                    "Failed to read Python file: {}",
                    e
                ))
            })?;

            // CRITICAL: Create a FRESH namespace for each Python file execution
            // This prevents imports from one file polluting another file's namespace
            // We use a new empty dict but set __builtins__ to maintain access to built-in functions
            let globals = pyo3::types::PyDict::new(py);
            let builtins = py.import("builtins").map_err(py_err)?;
            globals.set_item("__builtins__", builtins).map_err(py_err)?;

            // Set __file__ and __name__
            globals
                .set_item("__file__", launch_file_path)
                .map_err(py_err)?;
            globals.set_item("__name__", "__main__").map_err(py_err)?;

            // CRITICAL: Run isolation AGAIN right before executing the file
            // This ensures any modifications to sys.modules/sys.path by previous files are reset
            py.run(&isolation_cstr, None, None).map_err(py_err)?;
            log::debug!("Re-applied isolation right before file execution");

            // Add debug logging to catch Python execution errors
            log::debug!("Executing Python code from: {}", launch_file_path);
            log::trace!("Python code length: {} bytes", code.len());

            // Execute the file
            let code_cstr = CString::new(code).map_err(|e| {
                play_launch_parser::error::ParseError::PythonError(format!(
                    "Python code contains NUL byte: {}",
                    e
                ))
            })?;
            if let Err(e) = py.run(&code_cstr, Some(&globals), None) {
                log::error!("Python execution failed: {}", e);
                // Log the Python traceback if available
                if let Some(traceback) = e.traceback(py)
                    && let Ok(tb_str) = traceback.format()
                {
                    log::error!("Python traceback:\n{}", tb_str);
                }
                return Err(py_err(e));
            }

            // Get and call generate_launch_description()
            let gen_fn = globals
                .get_item("generate_launch_description")
                .map_err(py_err)?
                .ok_or_else(|| {
                    play_launch_parser::error::ParseError::PythonError(
                        "No generate_launch_description() function found".to_string(),
                    )
                })?;

            let launch_desc: Py<PyAny> = gen_fn.call0().map_err(py_err)?.into();

            // Execute what it returned, as `launch` does: in order, each
            // action when it is reached (`api::visit`).
            crate::api::visit::visit_any(py, launch_desc.bind(py)).map_err(py_err)?;

            log::debug!("Python launch file execution complete");
            Ok(())
        })
    }
}

/// pyo3's error, as the parser's.
///
/// Was `impl From<pyo3::PyErr> for ParseError` in core's `error.rs`, which made
/// core depend on pyo3 for the sake of `?`. Core keeps the VARIANT — reporting a
/// Python failure is its business — and the pyo3 type stays on this side.
///
/// It has to be a function rather than a `From` impl once `python/` becomes its
/// own crate: both `PyErr` and `ParseError` would then be foreign there, and the
/// orphan rule forbids the impl. Explicit `.map_err(py_err)` is the price, and
/// it is visible at each call site, which is not the worst outcome.
fn py_err(e: pyo3::PyErr) -> play_launch_parser::error::ParseError {
    play_launch_parser::error::ParseError::PythonError(e.to_string())
}
