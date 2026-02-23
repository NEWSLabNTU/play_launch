//! Shared ABI definitions for the play_launch WASM compilation and execution pipeline.
//!
//! This crate defines the contract between compiled WASM modules (produced by
//! `play_launch_wasm_codegen`) and the host runtime (`play_launch_wasm_runtime`).
//! Both crates import these definitions, ensuring they stay synchronized by construction.
//!
//! # Architecture
//!
//! ```text
//!           play_launch_wasm_common   (this crate)
//!                    |
//!           +--------+--------+
//!           v                 v
//!     wasm_codegen      wasm_runtime
//!      IR -> .wasm      .wasm -> JSON
//! ```
//!
//! # ABI Versioning
//!
//! Compiled WASM modules embed [`ABI_VERSION`] as a global. The runtime checks this
//! on instantiation and rejects modules with mismatched versions. Bump [`ABI_VERSION`]
//! on any breaking change to host imports.

/// ABI version embedded in compiled WASM modules.
///
/// The runtime checks `(global $abi_version i32)` against this value on instantiation.
/// Bump on any breaking change to host import signatures or semantics.
pub const ABI_VERSION: u32 = 1;

/// WASM import module name. All host functions are imported under this namespace.
///
/// ```wat
/// (import "launch" "begin_node" (func ...))
/// ```
pub const HOST_MODULE: &str = "launch";

/// Name of the WASM global that stores the ABI version in compiled modules.
pub const ABI_VERSION_GLOBAL: &str = "abi_version";

/// Name of the exported entry point function in compiled modules.
///
/// Signature: `(func (export "plan") (param $args_ptr i32) (param $args_len i32))`
///
/// The host writes CLI argument key-value pairs into guest memory and passes a pointer
/// to the serialized args. The function executes the compiled launch program, calling
/// host imports to produce records.
pub const ENTRY_POINT: &str = "plan";

/// Name of the exported memory in compiled modules.
pub const MEMORY_EXPORT: &str = "memory";

/// Host import function names.
///
/// Constants shared between codegen (which emits `(import "launch" "<name>" ...)`)
/// and runtime (which registers `linker.func_wrap("launch", "<name>", ...)`).
pub mod imports {
    // --- Context management ---

    /// Declare a launch argument with optional default value.
    ///
    /// Signature: `(name_ptr, name_len, default_ptr, default_len) -> void`
    ///
    /// If `default_len == 0` and `default_ptr == 0`, no default is provided.
    pub const DECLARE_ARG: &str = "declare_arg";

    /// Set a scoped variable (`<let name="..." value="..."/>`).
    ///
    /// Signature: `(name_ptr, name_len, value_ptr, value_len) -> void`
    pub const SET_VAR: &str = "set_var";

    /// Resolve a variable (LaunchConfiguration). Returns the current value.
    ///
    /// Signature: `(name_ptr, name_len) -> (result_ptr, result_len)`
    pub const RESOLVE_VAR: &str = "resolve_var";

    /// Set an environment variable in the current scope.
    ///
    /// Signature: `(name_ptr, name_len, value_ptr, value_len) -> void`
    pub const SET_ENV: &str = "set_env";

    /// Remove an environment variable from the current scope.
    ///
    /// Signature: `(name_ptr, name_len) -> void`
    pub const UNSET_ENV: &str = "unset_env";

    /// Push a ROS namespace onto the namespace stack.
    ///
    /// Signature: `(ns_ptr, ns_len) -> void`
    pub const PUSH_NAMESPACE: &str = "push_namespace";

    /// Pop the most recent namespace from the stack.
    ///
    /// Signature: `() -> void`
    pub const POP_NAMESPACE: &str = "pop_namespace";

    /// Set a global parameter in the current scope.
    ///
    /// Signature: `(name_ptr, name_len, value_ptr, value_len) -> void`
    pub const SET_GLOBAL_PARAM: &str = "set_global_param";

    /// Set a global topic remapping in the current scope.
    ///
    /// Signature: `(from_ptr, from_len, to_ptr, to_len) -> void`
    pub const SET_REMAP: &str = "set_remap";

    /// Save the current scope state. Returns a scope ID for later restoration.
    ///
    /// Signature: `() -> scope_id: i32`
    pub const SAVE_SCOPE: &str = "save_scope";

    /// Restore a previously saved scope state.
    ///
    /// Signature: `(scope_id: i32) -> void`
    pub const RESTORE_SCOPE: &str = "restore_scope";

    // --- Package resolution ---

    /// Find the share directory for a ROS package (`$(find-pkg-share pkg)`).
    ///
    /// Signature: `(pkg_ptr, pkg_len) -> (result_ptr, result_len)`
    pub const FIND_PACKAGE_SHARE: &str = "find_package_share";

    /// Resolve the full path to a ROS executable.
    ///
    /// Signature: `(pkg_ptr, pkg_len, exec_ptr, exec_len) -> (result_ptr, result_len)`
    pub const RESOLVE_EXEC_PATH: &str = "resolve_exec_path";

    // --- Substitution evaluation ---

    /// Evaluate `$(env VAR)` or `$(env VAR default)`.
    ///
    /// Signature: `(name_ptr, name_len, default_ptr, default_len) -> (result_ptr, result_len)`
    ///
    /// If `default_len == 0` and `default_ptr == 0`, no default is provided.
    pub const EVAL_ENV_VAR: &str = "eval_env_var";

    /// Evaluate `$(command ...)`. Subject to host policy (allowlist/passthrough/strict).
    ///
    /// Signature: `(cmd_ptr, cmd_len) -> (result_ptr, result_len)`
    pub const EVAL_COMMAND: &str = "eval_command";

    /// Evaluate `$(eval ...)` (Python expression).
    ///
    /// Signature: `(expr_ptr, expr_len) -> (result_ptr, result_len)`
    pub const EVAL_PYTHON_EXPR: &str = "eval_python_expr";

    /// Test whether a string value is truthy (ROS 2 truthiness rules).
    ///
    /// Signature: `(value_ptr, value_len) -> i32` (1 = truthy, 0 = falsy)
    pub const IS_TRUTHY: &str = "is_truthy";

    // --- Node builder ---

    /// Begin building a node record. Subsequent `set_node_*` / `add_node_*` calls
    /// configure it. Finalize with [`END_NODE`].
    ///
    /// Signature: `() -> void`
    pub const BEGIN_NODE: &str = "begin_node";

    /// Set the node's ROS package.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_NODE_PKG: &str = "set_node_pkg";

    /// Set the node's executable name.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_NODE_EXEC: &str = "set_node_exec";

    /// Set the node's name.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_NODE_NAME: &str = "set_node_name";

    /// Set the node's namespace.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_NODE_NAMESPACE: &str = "set_node_namespace";

    /// Add a parameter to the node being built.
    ///
    /// Signature: `(name_ptr, name_len, value_ptr, value_len) -> void`
    pub const ADD_NODE_PARAM: &str = "add_node_param";

    /// Add a parameter file path to the node being built.
    ///
    /// Signature: `(path_ptr, path_len) -> void`
    pub const ADD_NODE_PARAM_FILE: &str = "add_node_param_file";

    /// Add a topic remapping to the node being built.
    ///
    /// Signature: `(from_ptr, from_len, to_ptr, to_len) -> void`
    pub const ADD_NODE_REMAP: &str = "add_node_remap";

    /// Add an environment variable to the node being built.
    ///
    /// Signature: `(name_ptr, name_len, value_ptr, value_len) -> void`
    pub const ADD_NODE_ENV: &str = "add_node_env";

    /// Set the node's additional arguments string.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_NODE_ARGS: &str = "set_node_args";

    /// Set the node's respawn flag.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_NODE_RESPAWN: &str = "set_node_respawn";

    /// Set the node's respawn delay.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_NODE_RESPAWN_DELAY: &str = "set_node_respawn_delay";

    /// Finalize the node being built, committing it as a record.
    ///
    /// Signature: `() -> void`
    pub const END_NODE: &str = "end_node";

    // --- Executable builder ---

    /// Begin building an executable record.
    ///
    /// Signature: `() -> void`
    pub const BEGIN_EXECUTABLE: &str = "begin_executable";

    /// Set the executable's command.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_EXEC_CMD: &str = "set_exec_cmd";

    /// Set the executable's name.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_EXEC_NAME: &str = "set_exec_name";

    /// Add an argument to the executable being built.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const ADD_EXEC_ARG: &str = "add_exec_arg";

    /// Add an environment variable to the executable being built.
    ///
    /// Signature: `(name_ptr, name_len, value_ptr, value_len) -> void`
    pub const ADD_EXEC_ENV: &str = "add_exec_env";

    /// Finalize the executable being built.
    ///
    /// Signature: `() -> void`
    pub const END_EXECUTABLE: &str = "end_executable";

    // --- Container builder ---

    /// Begin building a container record.
    ///
    /// Signature: `() -> void`
    pub const BEGIN_CONTAINER: &str = "begin_container";

    /// Set the container's ROS package.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_CONTAINER_PKG: &str = "set_container_pkg";

    /// Set the container's executable name.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_CONTAINER_EXEC: &str = "set_container_exec";

    /// Set the container's name.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_CONTAINER_NAME: &str = "set_container_name";

    /// Set the container's namespace.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_CONTAINER_NAMESPACE: &str = "set_container_namespace";

    /// Set the container's additional arguments string.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_CONTAINER_ARGS: &str = "set_container_args";

    /// Begin building a composable node within a container or load-node action.
    ///
    /// Signature: `() -> void`
    pub const BEGIN_COMPOSABLE_NODE: &str = "begin_composable_node";

    /// Set the composable node's ROS package.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_COMP_PKG: &str = "set_comp_pkg";

    /// Set the composable node's plugin class name.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_COMP_PLUGIN: &str = "set_comp_plugin";

    /// Set the composable node's name.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_COMP_NAME: &str = "set_comp_name";

    /// Set the composable node's namespace.
    ///
    /// Signature: `(ptr, len) -> void`
    pub const SET_COMP_NAMESPACE: &str = "set_comp_namespace";

    /// Add a parameter to the composable node being built.
    ///
    /// Signature: `(name_ptr, name_len, value_ptr, value_len) -> void`
    pub const ADD_COMP_PARAM: &str = "add_comp_param";

    /// Add a topic remapping to the composable node being built.
    ///
    /// Signature: `(from_ptr, from_len, to_ptr, to_len) -> void`
    pub const ADD_COMP_REMAP: &str = "add_comp_remap";

    /// Add an extra argument to the composable node being built.
    ///
    /// Signature: `(name_ptr, name_len, value_ptr, value_len) -> void`
    pub const ADD_COMP_EXTRA_ARG: &str = "add_comp_extra_arg";

    /// Finalize the composable node being built.
    ///
    /// Signature: `() -> void`
    pub const END_COMPOSABLE_NODE: &str = "end_composable_node";

    /// Finalize the container being built, committing container + load-node records.
    ///
    /// Signature: `() -> void`
    pub const END_CONTAINER: &str = "end_container";

    // --- Load composable node (into existing container) ---

    /// Begin a load-composable-node action (loads into an existing container by name).
    ///
    /// Signature: `(target_ptr, target_len) -> void`
    pub const BEGIN_LOAD_NODE: &str = "begin_load_node";

    /// Finalize the load-composable-node action.
    /// Composable nodes added between [`BEGIN_LOAD_NODE`] and this call are committed.
    ///
    /// Signature: `() -> void`
    pub const END_LOAD_NODE: &str = "end_load_node";

    // --- String operations ---

    /// Concatenate two strings.
    ///
    /// Signature: `(a_ptr, a_len, b_ptr, b_len) -> (result_ptr, result_len)`
    pub const CONCAT: &str = "concat";

    /// Test string equality.
    ///
    /// Signature: `(a_ptr, a_len, b_ptr, b_len) -> i32` (1 = equal, 0 = not equal)
    pub const STR_EQUALS: &str = "str_equals";

    /// Returns an array of all import names for enumeration/validation.
    pub const ALL: &[&str] = &[
        DECLARE_ARG,
        SET_VAR,
        RESOLVE_VAR,
        SET_ENV,
        UNSET_ENV,
        PUSH_NAMESPACE,
        POP_NAMESPACE,
        SET_GLOBAL_PARAM,
        SET_REMAP,
        SAVE_SCOPE,
        RESTORE_SCOPE,
        FIND_PACKAGE_SHARE,
        RESOLVE_EXEC_PATH,
        EVAL_ENV_VAR,
        EVAL_COMMAND,
        EVAL_PYTHON_EXPR,
        IS_TRUTHY,
        BEGIN_NODE,
        SET_NODE_PKG,
        SET_NODE_EXEC,
        SET_NODE_NAME,
        SET_NODE_NAMESPACE,
        ADD_NODE_PARAM,
        ADD_NODE_PARAM_FILE,
        ADD_NODE_REMAP,
        ADD_NODE_ENV,
        SET_NODE_ARGS,
        SET_NODE_RESPAWN,
        SET_NODE_RESPAWN_DELAY,
        END_NODE,
        BEGIN_EXECUTABLE,
        SET_EXEC_CMD,
        SET_EXEC_NAME,
        ADD_EXEC_ARG,
        ADD_EXEC_ENV,
        END_EXECUTABLE,
        BEGIN_CONTAINER,
        SET_CONTAINER_PKG,
        SET_CONTAINER_EXEC,
        SET_CONTAINER_NAME,
        SET_CONTAINER_NAMESPACE,
        SET_CONTAINER_ARGS,
        BEGIN_COMPOSABLE_NODE,
        SET_COMP_PKG,
        SET_COMP_PLUGIN,
        SET_COMP_NAME,
        SET_COMP_NAMESPACE,
        ADD_COMP_PARAM,
        ADD_COMP_REMAP,
        ADD_COMP_EXTRA_ARG,
        END_COMPOSABLE_NODE,
        END_CONTAINER,
        BEGIN_LOAD_NODE,
        END_LOAD_NODE,
        CONCAT,
        STR_EQUALS,
    ];
}

/// Guest memory layout constants.
///
/// The compiled WASM module uses a bump allocator for string data.
/// The host writes return values to a fixed return area.
pub mod memory {
    /// Start offset of the bump allocator region in guest linear memory.
    ///
    /// Offsets 0..BUMP_BASE are reserved for the data segment (string literals)
    /// and the return area.
    pub const BUMP_BASE: u32 = 0x1_0000; // 64 KB

    /// Fixed offset where the host writes return strings (host -> guest).
    ///
    /// The host writes `(ptr: u32, len: u32)` at this address. The guest reads
    /// the return value from here after a host call that returns a string.
    ///
    /// Layout at RETURN_AREA: `[result_ptr: u32, result_len: u32]`
    pub const RETURN_AREA: u32 = 0x0_FF00;

    /// Size of the return area in bytes.
    pub const RETURN_AREA_SIZE: u32 = 8; // two u32 values

    /// Name of the bump pointer global in compiled modules.
    ///
    /// The codegen emits `(global $bump (mut i32) (i32.const <BUMP_BASE>))`.
    /// The guest increments this when allocating string data at runtime.
    pub const BUMP_GLOBAL: &str = "bump";
}

/// Error types shared between codegen and runtime.
#[derive(Debug, thiserror::Error)]
pub enum WasmError {
    /// The compiled module's ABI version doesn't match the runtime's expected version.
    #[error("ABI version mismatch: module has v{found}, runtime expects v{expected}")]
    AbiVersionMismatch { expected: u32, found: u32 },

    /// Failed to decode a string from guest memory.
    #[error("failed to read string from guest memory at offset {offset}, length {len}")]
    StringDecodingError { offset: u32, len: u32 },

    /// A required WASM export is missing from the module.
    #[error("missing required export: {0}")]
    MissingExport(String),

    /// A host import was called in an invalid state (e.g., `end_node` without `begin_node`).
    #[error("invalid builder state: {0}")]
    InvalidBuilderState(String),

    /// The guest module ran out of fuel (CPU budget exceeded).
    #[error("execution fuel exhausted (infinite loop?)")]
    FuelExhausted,

    /// The guest module's linear memory exceeded the configured limit.
    #[error("memory limit exceeded: {0} bytes")]
    MemoryLimitExceeded(u64),

    /// An `eval_command` call was rejected by the host policy.
    #[error("command execution blocked by policy: {0}")]
    CommandBlocked(String),

    /// Variable resolution failed (undefined variable, no default).
    #[error("undefined variable: {0}")]
    UndefinedVariable(String),

    /// Package resolution failed.
    #[error("package not found: {0}")]
    PackageNotFound(String),

    /// A compilation error in the codegen phase.
    #[error("compilation error: {0}")]
    CompilationError(String),

    /// A generic runtime error.
    #[error("runtime error: {0}")]
    RuntimeError(String),
}

/// Result type alias using [`WasmError`].
pub type Result<T> = std::result::Result<T, WasmError>;

/// Command execution policy for `$(command ...)` substitutions.
///
/// Controls how the runtime handles `eval_command` host calls.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum CommandPolicy {
    /// Reject all command substitutions. Produces [`WasmError::CommandBlocked`].
    Strict,

    /// Allow only commands matching the allowlist patterns.
    Allowlist(Vec<String>),

    /// Execute commands on the host without restriction.
    /// Breaks sandboxing but matches current `parse_launch_file()` behavior.
    #[default]
    Passthrough,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_abi_version_is_positive() {
        const { assert!(ABI_VERSION > 0) };
    }

    #[test]
    fn test_host_module_is_nonempty() {
        assert!(!HOST_MODULE.is_empty());
    }

    #[test]
    fn test_all_imports_nonempty() {
        for name in imports::ALL {
            assert!(!name.is_empty(), "empty import name found");
        }
    }

    #[test]
    fn test_all_imports_unique() {
        let mut seen = std::collections::HashSet::new();
        for name in imports::ALL {
            assert!(seen.insert(name), "duplicate import name: {name}");
        }
    }

    #[test]
    fn test_all_imports_count() {
        // Ensure ALL stays in sync when new imports are added.
        // Update this count when adding new host imports.
        assert_eq!(
            imports::ALL.len(),
            56,
            "imports::ALL count changed — update this test if you added/removed imports"
        );
    }

    #[test]
    fn test_memory_layout_no_overlap() {
        // Return area must fit before bump base.
        const { assert!(memory::RETURN_AREA + memory::RETURN_AREA_SIZE <= memory::BUMP_BASE) };
    }

    #[test]
    fn test_command_policy_default_is_passthrough() {
        assert_eq!(CommandPolicy::default(), CommandPolicy::Passthrough);
    }

    #[test]
    fn test_error_display() {
        let err = WasmError::AbiVersionMismatch {
            expected: 1,
            found: 2,
        };
        assert!(err.to_string().contains("v2"));
        assert!(err.to_string().contains("v1"));
    }
}
