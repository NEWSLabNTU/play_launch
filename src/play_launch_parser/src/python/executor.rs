//! Python launch file executor
//!
//! Executes Python launch files with PyO3 mocks to capture node definitions.

use crate::error::Result;
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
        Python::with_gil(|py| {
            log::debug!("Executing Python launch file: {}", launch_file_path);

            // Register PyO3 mock modules in sys.modules
            crate::python::api::register_modules(py)?;

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
    'launch.event_handlers', 'launch.launch_description_sources', 'launch.frontend',
    'launch.frontend.type_utils', 'launch.utilities', 'launch.utilities.type_utils',
    'launch.some_substitutions_type',
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
            py.run(&isolation_cstr, None, None)?;
            log::debug!("Installed aggressive Python environment isolation for launch* mocks");

            // Launch configurations are already in the thread-local LaunchContext
            // (set by execute_python_file() before calling this executor)

            // Execute the Python file directly with exec() instead of runpy
            // This gives us complete control over the execution environment
            use std::fs;
            let code = fs::read_to_string(launch_file_path).map_err(|e| {
                crate::error::ParseError::PythonError(format!("Failed to read Python file: {}", e))
            })?;

            // CRITICAL: Create a FRESH namespace for each Python file execution
            // This prevents imports from one file polluting another file's namespace
            // We use a new empty dict but set __builtins__ to maintain access to built-in functions
            let globals = pyo3::types::PyDict::new(py);
            let builtins = py.import("builtins")?;
            globals.set_item("__builtins__", builtins)?;

            // Set __file__ and __name__
            globals.set_item("__file__", launch_file_path)?;
            globals.set_item("__name__", "__main__")?;

            // CRITICAL: Run isolation AGAIN right before executing the file
            // This ensures any modifications to sys.modules/sys.path by previous files are reset
            py.run(&isolation_cstr, None, None)?;
            log::debug!("Re-applied isolation right before file execution");

            // Add debug logging to catch Python execution errors
            log::debug!("Executing Python code from: {}", launch_file_path);
            log::trace!("Python code length: {} bytes", code.len());

            // Execute the file
            let code_cstr = CString::new(code).map_err(|e| {
                crate::error::ParseError::PythonError(format!(
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
                return Err(e.into());
            }

            // Get and call generate_launch_description()
            let gen_fn = globals
                .get_item("generate_launch_description")?
                .ok_or_else(|| {
                    crate::error::ParseError::PythonError(
                        "No generate_launch_description() function found".to_string(),
                    )
                })?;

            let launch_desc: PyObject = gen_fn.call0()?.into();

            // Visit all entities in the launch description
            visit_launch_description(py, &launch_desc)?;

            // Re-resolve any containers/load_nodes with unresolved substitutions
            // This handles cases where containers are created before DeclareLaunchArgument
            process_launch_arguments(py, &launch_desc)?;

            log::debug!("Python launch file execution complete");
            Ok(())
        })
    }
}

/// Process launch arguments and re-resolve any containers with unresolved names
fn process_launch_arguments(_py: Python, _launch_desc: &PyObject) -> PyResult<()> {
    use crate::{
        python::bridge::{
            update_captured_containers, update_captured_load_nodes, update_captured_nodes,
            with_launch_context,
        },
        substitution::{parser::parse_substitutions, types::resolve_substitutions},
    };

    // Snapshot configurations from the thread-local LaunchContext
    // (includes CLI args, include args, and DeclareLaunchArgument defaults)
    // We snapshot because update_captured_* also borrows the context via thread-local
    let configs = with_launch_context(|ctx| ctx.configurations());

    // Build a temporary context for substitution resolution
    let mut ctx = crate::substitution::context::LaunchContext::new();
    for (key, value) in &configs {
        ctx.set_configuration(key.clone(), value.clone());
    }

    // Re-resolve container names that contain unresolved substitutions
    update_captured_containers(|containers| {
        for container in containers.iter_mut() {
            if container.name.contains("$(") {
                log::debug!(
                    "Re-resolving container name '{}' with updated context",
                    container.name
                );
                if let Ok(subs) = parse_substitutions(&container.name)
                    && let Ok(resolved) = resolve_substitutions(&subs, &ctx)
                {
                    log::debug!("Container name '{}' -> '{}'", container.name, resolved);
                    container.name = resolved;
                }
            }
        }
    });

    // Re-resolve target_container_name, node names, remappings, and parameters in load_nodes
    update_captured_load_nodes(|load_nodes| {
        for load_node in load_nodes.iter_mut() {
            // Re-resolve node name
            if load_node.node_name.contains("$(") {
                log::debug!(
                    "Re-resolving composable node name '{}' with updated context",
                    load_node.node_name
                );
                if let Ok(subs) = parse_substitutions(&load_node.node_name)
                    && let Ok(resolved) = resolve_substitutions(&subs, &ctx)
                {
                    log::debug!(
                        "Composable node name '{}' -> '{}'",
                        load_node.node_name,
                        resolved
                    );
                    load_node.node_name = resolved;
                }
            }

            // Re-resolve target_container_name
            if load_node.target_container_name.contains("$(") {
                log::debug!(
                    "Re-resolving target_container_name '{}' for node '{}'",
                    load_node.target_container_name,
                    load_node.node_name
                );
                if let Ok(subs) = parse_substitutions(&load_node.target_container_name)
                    && let Ok(resolved) = resolve_substitutions(&subs, &ctx)
                {
                    log::debug!(
                        "Target container '{}' -> '{}' for node '{}'",
                        load_node.target_container_name,
                        resolved,
                        load_node.node_name
                    );
                    load_node.target_container_name = resolved;
                }
            }

            // Re-resolve remappings
            for remap in load_node.remappings.iter_mut() {
                if remap.1.contains("$(")
                    && let Ok(subs) = parse_substitutions(&remap.1)
                    && let Ok(resolved) = resolve_substitutions(&subs, &ctx)
                {
                    log::debug!(
                        "Remap '{}' -> '{}' for node '{}'",
                        remap.1,
                        resolved,
                        load_node.node_name
                    );
                    remap.1 = resolved;
                }
            }

            // Re-resolve parameters and expand __param_file references
            let mut expanded_params = Vec::new();
            for param in load_node.parameters.iter() {
                let key = param.0.clone();
                let mut value = param.1.clone();

                // Resolve substitutions in values
                if value.contains("$(")
                    && let Ok(subs) = parse_substitutions(&value)
                    && let Ok(resolved) = resolve_substitutions(&subs, &ctx)
                {
                    log::debug!(
                        "Param '{}' -> '{}' for node '{}'",
                        value,
                        resolved,
                        load_node.node_name
                    );
                    value = resolved;
                }

                // Expand __param_file entries: load YAML and inline parameters
                if key == "__param_file" && (value.ends_with(".yaml") || value.ends_with(".yml")) {
                    match crate::params::load_param_file(std::path::Path::new(&value)) {
                        Ok(yaml_params) => {
                            log::debug!(
                                "Expanded __param_file '{}' -> {} parameters for node '{}'",
                                value,
                                yaml_params.len(),
                                load_node.node_name
                            );
                            expanded_params.extend(yaml_params);
                            continue;
                        }
                        Err(e) => {
                            log::warn!(
                                "Failed to load __param_file '{}' for node '{}': {}",
                                value,
                                load_node.node_name,
                                e
                            );
                            // Keep as-is on failure
                        }
                    }
                }

                expanded_params.push((key, value));
            }
            load_node.parameters = expanded_params;
        }
    });

    // Re-resolve remappings and parameters in regular nodes
    update_captured_nodes(|nodes| {
        for node in nodes.iter_mut() {
            // Re-resolve node name
            if let Some(ref name) = node.name
                && name.contains("$(")
            {
                log::debug!("Re-resolving node name '{}' with updated context", name);
                if let Ok(subs) = parse_substitutions(name)
                    && let Ok(resolved) = resolve_substitutions(&subs, &ctx)
                {
                    log::debug!("Node name '{}' -> '{}'", name, resolved);
                    node.name = Some(resolved);
                }
            }

            // Re-resolve remappings
            for remap in node.remappings.iter_mut() {
                if remap.1.contains("$(")
                    && let Ok(subs) = parse_substitutions(&remap.1)
                    && let Ok(resolved) = resolve_substitutions(&subs, &ctx)
                {
                    log::debug!("Node remap '{}' -> '{}'", remap.1, resolved);
                    remap.1 = resolved;
                }
            }

            // Re-resolve parameters
            for param in node.parameters.iter_mut() {
                if param.1.contains("$(")
                    && let Ok(subs) = parse_substitutions(&param.1)
                    && let Ok(resolved) = resolve_substitutions(&subs, &ctx)
                {
                    log::debug!("Node param '{}' -> '{}'", param.1, resolved);
                    param.1 = resolved;
                }
            }
        }
    });

    Ok(())
}

/// Visit all entities in a launch description
fn visit_launch_description(py: Python, launch_desc: &PyObject) -> PyResult<()> {
    // Get entities from launch description (it's a property, not a method)
    let entities = launch_desc.getattr(py, "actions")?;
    let entities_list: Vec<PyObject> = entities.extract(py)?;

    log::debug!(
        "Visiting {} entities in launch description",
        entities_list.len()
    );

    // Visit each entity
    for entity in entities_list.iter() {
        visit_entity(py, entity)?;
    }

    Ok(())
}

/// Visit a single entity
fn visit_entity(py: Python, entity: &PyObject) -> PyResult<()> {
    // Get entity type
    let entity_class = entity.getattr(py, "__class__")?;
    let entity_type = entity_class
        .getattr(py, "__name__")?
        .extract::<String>(py)?;

    log::debug!("Visiting entity: {}", entity_type);

    match entity_type.as_str() {
        "Node" | "LifecycleNode" | "ComposableNodeContainer" | "LoadComposableNodes" => {
            // Already captured during construction
            log::debug!("Entity {} already captured", entity_type);
        }

        "OpaqueFunction" => {
            log::debug!("Executing OpaqueFunction");
            // Call execute() to run the function and get returned entities
            let result = entity.call_method0(py, "execute")?;

            // Result should be a list of entities or None
            if !result.is_none(py)
                && let Ok(entities) = result.extract::<Vec<PyObject>>(py)
            {
                for entity in entities {
                    visit_entity(py, &entity)?;
                }
            }
        }

        "IncludeLaunchDescription" => {
            log::debug!("Processing IncludeLaunchDescription");
            // Includes are captured during construction
        }

        "GroupAction" => {
            log::debug!("Processing GroupAction");
            // Get nested actions and visit them
            let sub_entities = entity.getattr(py, "actions")?;
            let sub_list: Vec<PyObject> = sub_entities.extract(py)?;
            log::debug!("GroupAction contains {} sub-actions", sub_list.len());
            for sub in sub_list {
                visit_entity(py, &sub)?;
            }
        }

        "DeclareLaunchArgument"
        | "SetLaunchConfiguration"
        | "SetEnvironmentVariable"
        | "PushRosNamespace"
        | "PopRosNamespace" => {
            log::debug!("Skipping action: {}", entity_type);
            // These are handled during static parsing
        }

        _ => {
            log::trace!("Skipping unknown entity type: {}", entity_type);
        }
    }

    Ok(())
}
