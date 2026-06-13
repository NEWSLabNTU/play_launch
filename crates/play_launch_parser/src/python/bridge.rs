//! Bridge between Python and Rust types

use crate::{
    actions::container::{DEFAULT_CONTAINER_EXECUTABLE, DEFAULT_CONTAINER_PACKAGE},
    captures::{ContainerCapture, IncludeCapture, LoadNodeCapture, NodeCapture},
    error::Result,
    record::{ComposableNodeContainerRecord, LoadNodeRecord, NodeRecord},
    substitution::LaunchContext,
};
use std::{cell::RefCell, collections::HashMap};

impl NodeCapture {
    /// Convert to NodeRecord and generate command line
    ///
    /// # Arguments
    /// * `global_params` - Global ROS parameters from SetParameter actions (passed from context)
    pub fn to_record(&self, global_params: &Option<Vec<(String, String)>>) -> Result<NodeRecord> {
        // Generate ROS command line (now includes global params)
        let cmd = self.generate_command(global_params);

        // Extract parameter files from command (--params-file arguments)
        let params_files = self.extract_params_files_from_cmd(&cmd);

        Ok(NodeRecord {
            args: if self.arguments.is_empty() {
                None
            } else {
                Some(self.arguments.clone())
            },
            cmd,
            env: if self.env_vars.is_empty() {
                None
            } else {
                Some(self.env_vars.clone())
            },
            exec_name: Some(self.executable.clone()), // exec_name = executable name
            executable: self.executable.clone(),
            global_params: global_params.clone(),
            machine: self.machine.clone(),
            name: self.name.clone(),
            namespace: self.namespace.clone(),
            package: Some(self.package.clone()),
            params: self
                .parameters
                .iter()
                .map(|(k, v)| {
                    (
                        k.clone(),
                        crate::record::generator::normalize_param_value(v),
                    )
                })
                .collect(),
            params_files,
            remaps: self.remappings.clone(),
            respawn: None,
            respawn_delay: None,
            ros_args: None, // Python parser doesn't populate ros_args
            scope: None,
        })
    }

    /// Extract parameter file paths from command line
    fn extract_params_files_from_cmd(&self, cmd: &[String]) -> Vec<String> {
        let mut params_files = Vec::new();
        let mut i = 0;
        while i < cmd.len() {
            if cmd[i] == "--params-file" && i + 1 < cmd.len() {
                params_files.push(cmd[i + 1].clone());
                i += 2;
            } else {
                i += 1;
            }
        }
        params_files
    }

    /// Generate ROS 2 command line using the shared `build_ros_command()`.
    fn generate_command(&self, global_params: &Option<Vec<(String, String)>>) -> Vec<String> {
        use crate::record::generator::build_ros_command;

        // Resolve executable path, with ros2 run fallback
        let exec_path = if let Some(path) = find_package_executable(&self.package, &self.executable)
        {
            path
        } else {
            log::warn!(
                "Could not resolve executable path for {}/{}, using 'ros2 run'",
                self.package,
                self.executable
            );
            // ros2 run fallback: return early with custom command format
            let mut cmd = vec![
                "ros2".to_string(),
                "run".to_string(),
                self.package.clone(),
                self.executable.clone(),
            ];
            cmd.extend(self.arguments.clone());
            cmd.push("--ros-args".to_string());
            if let Some(ref name) = self.name {
                cmd.push("-r".to_string());
                cmd.push(format!("__node:={}", name));
            }
            if let Some(ref ns) = self.namespace
                && !ns.is_empty()
                && ns != "/"
            {
                cmd.push("-r".to_string());
                cmd.push(format!("__ns:={}", ns));
            }
            return cmd;
        };

        let empty_gp = Vec::new();
        let gp = global_params.as_deref().unwrap_or(&empty_gp);

        build_ros_command(
            &exec_path,
            self.name.as_deref(),
            self.namespace.as_deref(),
            gp,
            &self.parameters,
            &self.params_files,
            &self.remappings,
            &self.arguments,
        )
    }
}

impl ContainerCapture {
    /// Convert to ComposableNodeContainerRecord
    ///
    /// # Arguments
    /// * `global_params` - Global ROS parameters from SetParameter actions (passed from context)
    pub fn to_record(
        &self,
        global_params: &Option<Vec<(String, String)>>,
    ) -> Result<ComposableNodeContainerRecord> {
        let package = self
            .package
            .clone()
            .unwrap_or_else(|| DEFAULT_CONTAINER_PACKAGE.to_string());
        let executable = self
            .executable
            .clone()
            .unwrap_or_else(|| DEFAULT_CONTAINER_EXECUTABLE.to_string());

        // Generate command if not provided
        let cmd = if self.cmd.is_empty() {
            use crate::record::generator::{build_ros_command, resolve_exec_path};

            let exec_path = resolve_exec_path(&package, &executable);
            let empty_gp = Vec::new();
            let gp = global_params.as_deref().unwrap_or(&empty_gp);
            let ns_ref = if self.namespace.is_empty() || self.namespace == "/" {
                None
            } else {
                Some(self.namespace.as_str())
            };

            build_ros_command(
                &exec_path,
                Some(self.name.as_str()),
                ns_ref,
                gp,
                &[],
                &[],
                &[],
                &[],
            )
        } else {
            self.cmd.clone()
        };

        Ok(ComposableNodeContainerRecord {
            args: None,
            cmd,
            env: None,
            exec_name: Some(self.name.clone()),
            executable,
            global_params: global_params.clone(),
            name: self.name.clone(),
            namespace: self.namespace.clone(),
            package,
            params: Vec::new(),
            params_files: Vec::new(),
            remaps: Vec::new(),
            respawn: Some(false),
            respawn_delay: None,
            ros_args: None,
            scope: None,
        })
    }
}

impl LoadNodeCapture {
    /// Convert to LoadNodeRecord
    ///
    /// # Arguments
    /// * `global_params` - Global ROS parameters from SetParameter actions (passed from context)
    pub fn to_record(
        &self,
        global_params: &Option<Vec<(String, String)>>,
    ) -> Result<LoadNodeRecord> {
        log::debug!(
            "LoadNodeCapture::to_record: node='{}', self.parameters.len()={}",
            self.node_name,
            self.parameters.len()
        );

        // Merge global parameters with node-specific parameters
        let empty_gp = Vec::new();
        let gp = global_params.as_deref().unwrap_or(&empty_gp);
        log::debug!(
            "LoadNodeCapture::to_record: {} global params for node '{}'",
            gp.len(),
            self.node_name
        );

        let merged_params =
            crate::record::generator::merge_params_with_global(gp, &self.parameters);

        log::debug!(
            "LoadNodeCapture::to_record: Final merged_params count={} for node '{}'",
            merged_params.len(),
            self.node_name
        );

        Ok(LoadNodeRecord {
            package: self.package.clone(),
            plugin: self.plugin.clone(),
            target_container_name: self.target_container_name.clone(),
            node_name: self.node_name.clone(),
            namespace: self.namespace.clone(),
            log_level: None,
            remaps: self.remappings.clone(),
            params: merged_params,
            extra_args: HashMap::new(),
            env: None,
            scope: None,
        })
    }
}

/// Push a namespace onto the ROS namespace stack
/// Uses LaunchContext via thread-local storage
///
/// IMPORTANT: Do NOT add "/" prefix here. LaunchContext.push_namespace() handles
/// relative vs absolute namespace semantics correctly:
/// - Relative "ns1" → appended to current (e.g., "/" → "/ns1")
/// - Absolute "/ns1" → used as-is
pub fn push_ros_namespace(namespace: String) -> bool {
    with_launch_context(|ctx| {
        let depth_before = ctx.namespace_depth();
        log::debug!("Pushing ROS namespace to LaunchContext: '{}'", namespace);
        ctx.push_namespace(namespace);
        let did_push = ctx.namespace_depth() > depth_before;
        if !did_push {
            log::debug!("Namespace push was a no-op (empty or root namespace)");
        }
        did_push
    })
}

/// Pop a namespace from the ROS namespace stack
/// Uses LaunchContext via thread-local storage
pub fn pop_ros_namespace() {
    with_launch_context(|ctx| {
        if ctx.namespace_depth() > 1 {
            ctx.pop_namespace();
            log::debug!("Popped ROS namespace from LaunchContext");
        } else {
            log::warn!("Attempted to pop from root namespace in LaunchContext");
        }
    });
}

/// Get the current ROS namespace
/// Uses LaunchContext via thread-local storage
pub fn get_current_ros_namespace() -> String {
    with_launch_context(|ctx| {
        let result = ctx.current_namespace();
        log::debug!("get_current_ros_namespace from LaunchContext: '{}'", result);
        result
    })
}

// ========== Thread-Local LaunchContext (unified context) ==========

thread_local! {
    /// Thread-local storage for the current LaunchContext
    /// This allows Python API classes to access the context without global state
    static CURRENT_LAUNCH_CONTEXT: RefCell<Option<*mut LaunchContext>> = const { RefCell::new(None) };
}

/// Set the current LaunchContext for this thread
/// SAFETY: The caller must ensure the LaunchContext lives for the duration of Python execution
fn set_current_launch_context(ctx: &mut LaunchContext) {
    CURRENT_LAUNCH_CONTEXT.with(|cell| {
        *cell.borrow_mut() = Some(ctx as *mut LaunchContext);
    });
}

/// Get the current LaunchContext for this thread.
/// Returns None if no context is set.
///
/// Private — external code should use `with_launch_context()` or
/// `try_with_launch_context()` which safely scope the borrow.
fn get_current_launch_context() -> Option<*mut LaunchContext> {
    CURRENT_LAUNCH_CONTEXT.with(|cell| *cell.borrow())
}

/// Clear the current LaunchContext for this thread
fn clear_current_launch_context() {
    CURRENT_LAUNCH_CONTEXT.with(|cell| {
        *cell.borrow_mut() = None;
    });
}

/// RAII guard that sets the thread-local LaunchContext on creation and clears it on drop.
/// This ensures the context is always cleared even if the code between set and clear panics
/// or returns early.
pub struct LaunchContextGuard;

impl LaunchContextGuard {
    /// Set the thread-local LaunchContext for the duration of this guard's lifetime.
    ///
    /// SAFETY: The caller must ensure the LaunchContext outlives this guard.
    pub fn new(ctx: &mut LaunchContext) -> Self {
        set_current_launch_context(ctx);
        Self
    }
}

impl Drop for LaunchContextGuard {
    fn drop(&mut self) {
        clear_current_launch_context();
    }
}

/// Execute a closure with access to the current LaunchContext
/// Panics if no context is set
pub fn with_launch_context<F, R>(f: F) -> R
where
    F: FnOnce(&mut LaunchContext) -> R,
{
    let ctx_ptr = get_current_launch_context()
        .expect("No LaunchContext set - Python execution must be called through LaunchTraverser");

    // SAFETY: The pointer is valid for the duration of Python execution
    // as guaranteed by set_current_launch_context
    unsafe { f(&mut *ctx_ptr) }
}

/// Execute a closure with access to the current LaunchContext, if one is set.
///
/// Returns `None` if no context is available (unlike `with_launch_context` which panics).
pub fn try_with_launch_context<F, R>(f: F) -> Option<R>
where
    F: FnOnce(&mut LaunchContext) -> R,
{
    let ctx_ptr = get_current_launch_context()?;
    // SAFETY: The pointer is valid for the duration of Python execution
    // as guaranteed by set_current_launch_context
    Some(unsafe { f(&mut *ctx_ptr) })
}

/// Get a clone of all captured nodes from LaunchContext
/// Panics if no context is set
pub fn get_captured_nodes() -> Vec<NodeCapture> {
    with_launch_context(|ctx| ctx.captured_nodes().to_vec())
}

/// Get a clone of all captured containers from LaunchContext
/// Panics if no context is set
pub fn get_captured_containers() -> Vec<ContainerCapture> {
    with_launch_context(|ctx| ctx.captured_containers().to_vec())
}

/// Get a clone of all captured load_nodes from LaunchContext
/// Panics if no context is set
pub fn get_captured_load_nodes() -> Vec<LoadNodeCapture> {
    with_launch_context(|ctx| ctx.captured_load_nodes().to_vec())
}

/// Get a clone of all captured includes from LaunchContext
/// Panics if no context is set
pub fn get_captured_includes() -> Vec<IncludeCapture> {
    with_launch_context(|ctx| ctx.captured_includes().to_vec())
}

/// Update captured nodes in LaunchContext by applying a mutation function
/// Panics if no context is set
pub fn update_captured_nodes<F>(f: F)
where
    F: FnOnce(&mut Vec<NodeCapture>),
{
    with_launch_context(|ctx| {
        let nodes = ctx.captured_nodes_mut();
        f(nodes);
    })
}

/// Update captured containers in LaunchContext by applying a mutation function
/// Panics if no context is set
pub fn update_captured_containers<F>(f: F)
where
    F: FnOnce(&mut Vec<ContainerCapture>),
{
    with_launch_context(|ctx| {
        let containers = ctx.captured_containers_mut();
        f(containers);
    })
}

/// Update captured load_nodes in LaunchContext by applying a mutation function
/// Panics if no context is set
pub fn update_captured_load_nodes<F>(f: F)
where
    F: FnOnce(&mut Vec<LoadNodeCapture>),
{
    with_launch_context(|ctx| {
        let load_nodes = ctx.captured_load_nodes_mut();
        f(load_nodes);
    })
}

// ========== Helper Functions ==========

/// Find ROS 2 package executable path
///
/// Searches for the executable in the ROS 2 package lib directory.
/// Returns the full path to the executable if found.
///
/// Search order:
/// 1. `{AMENT_PREFIX_PATH}/{package}/lib/{package}/{executable}`
/// 2. `/opt/ros/{ROS_DISTRO}/lib/{package}/{executable}`
/// 3. Common ROS 2 distributions (jazzy, iron, humble, galactic, foxy)
pub fn find_package_executable(package_name: &str, executable: &str) -> Option<String> {
    // Try AMENT_PREFIX_PATH first (for local installs)
    if let Ok(prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
        for prefix in prefix_path.split(':') {
            let exec_path = format!("{}/lib/{}/{}", prefix, package_name, executable);
            if std::path::Path::new(&exec_path).exists() {
                log::debug!("Resolved executable: {} -> {}", executable, exec_path);
                return Some(exec_path);
            }
        }
    }

    // Try ROS_DISTRO environment variable
    if let Ok(distro) = std::env::var("ROS_DISTRO") {
        let exec_path = format!("/opt/ros/{}/lib/{}/{}", distro, package_name, executable);
        if std::path::Path::new(&exec_path).exists() {
            log::debug!("Resolved executable: {} -> {}", executable, exec_path);
            return Some(exec_path);
        }
    }

    // Fallback: Try common ROS 2 distributions
    for distro in crate::substitution::types::KNOWN_ROS_DISTROS {
        let exec_path = format!("/opt/ros/{}/lib/{}/{}", distro, package_name, executable);
        if std::path::Path::new(&exec_path).exists() {
            log::debug!("Resolved executable: {} -> {}", executable, exec_path);
            return Some(exec_path);
        }
    }

    log::debug!(
        "Could not resolve executable path for {}/{}",
        package_name,
        executable
    );
    None
}

// ========== Capture Functions (use LaunchContext via thread-local) ==========

/// Capture a node to LaunchContext
pub fn capture_node(node: NodeCapture) {
    with_launch_context(|ctx| ctx.capture_node(node));
}

/// Capture a container to LaunchContext
pub fn capture_container(container: ContainerCapture) {
    with_launch_context(|ctx| ctx.capture_container(container));
}

/// Capture a load_node to LaunchContext
pub fn capture_load_node(load_node: LoadNodeCapture) {
    with_launch_context(|ctx| ctx.capture_load_node(load_node));
}

/// Capture an include to LaunchContext
pub fn capture_include(include: IncludeCapture) {
    with_launch_context(|ctx| ctx.capture_include(include));
}
