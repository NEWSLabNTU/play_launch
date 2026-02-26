//! WASM type signatures for host import functions.

use play_launch_wasm_common::imports;
use wasm_encoder::ValType;

/// Return the (params, results) signature for a given host import name.
pub(crate) fn import_signature(name: &str) -> anyhow::Result<(Vec<ValType>, Vec<ValType>)> {
    Ok(match name {
        // Context operations — no return
        imports::DECLARE_ARG => (vec![ValType::I32; 4], vec![]),       // name_ptr, name_len, default_ptr, default_len
        imports::SET_VAR => (vec![ValType::I32; 4], vec![]),           // name_ptr, name_len, val_ptr, val_len
        imports::SET_ENV => (vec![ValType::I32; 4], vec![]),
        imports::UNSET_ENV => (vec![ValType::I32; 2], vec![]),         // name_ptr, name_len
        imports::PUSH_NAMESPACE => (vec![ValType::I32; 2], vec![]),
        imports::POP_NAMESPACE => (vec![], vec![]),
        imports::SET_GLOBAL_PARAM => (vec![ValType::I32; 4], vec![]),
        imports::SET_REMAP => (vec![ValType::I32; 4], vec![]),
        imports::SAVE_SCOPE => (vec![], vec![]),
        imports::RESTORE_SCOPE => (vec![], vec![]),

        // String-returning functions -> (ptr, len) result
        imports::RESOLVE_VAR => (vec![ValType::I32; 2], vec![ValType::I32; 2]),
        imports::FIND_PACKAGE_SHARE => (vec![ValType::I32; 2], vec![ValType::I32; 2]),
        imports::RESOLVE_EXEC_PATH => (vec![ValType::I32; 4], vec![ValType::I32; 2]),
        imports::EVAL_ENV_VAR => (vec![ValType::I32; 4], vec![ValType::I32; 2]),
        imports::EVAL_COMMAND => (vec![ValType::I32; 2], vec![ValType::I32; 2]),
        imports::EVAL_PYTHON_EXPR => (vec![ValType::I32; 2], vec![ValType::I32; 2]),
        imports::CONCAT => (vec![ValType::I32; 4], vec![ValType::I32; 2]),
        imports::STR_EQUALS => (vec![ValType::I32; 4], vec![ValType::I32]),

        // Boolean return
        imports::IS_TRUTHY => (vec![ValType::I32; 2], vec![ValType::I32]),

        // Node builder — no return
        imports::BEGIN_NODE => (vec![], vec![]),
        imports::SET_NODE_PKG => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_EXEC => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_NAME => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_NAMESPACE => (vec![ValType::I32; 2], vec![]),
        imports::ADD_NODE_PARAM => (vec![ValType::I32; 4], vec![]),
        imports::ADD_NODE_PARAM_FILE => (vec![ValType::I32; 2], vec![]),
        imports::ADD_NODE_REMAP => (vec![ValType::I32; 4], vec![]),
        imports::ADD_NODE_ENV => (vec![ValType::I32; 4], vec![]),
        imports::SET_NODE_ARGS => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_RESPAWN => (vec![ValType::I32; 2], vec![]),
        imports::SET_NODE_RESPAWN_DELAY => (vec![ValType::I32; 2], vec![]),
        imports::END_NODE => (vec![], vec![]),

        // Executable builder
        imports::BEGIN_EXECUTABLE => (vec![], vec![]),
        imports::SET_EXEC_CMD => (vec![ValType::I32; 2], vec![]),
        imports::SET_EXEC_NAME => (vec![ValType::I32; 2], vec![]),
        imports::ADD_EXEC_ARG => (vec![ValType::I32; 2], vec![]),
        imports::ADD_EXEC_ENV => (vec![ValType::I32; 4], vec![]),
        imports::END_EXECUTABLE => (vec![], vec![]),

        // Container builder
        imports::BEGIN_CONTAINER => (vec![], vec![]),
        imports::SET_CONTAINER_PKG => (vec![ValType::I32; 2], vec![]),
        imports::SET_CONTAINER_EXEC => (vec![ValType::I32; 2], vec![]),
        imports::SET_CONTAINER_NAME => (vec![ValType::I32; 2], vec![]),
        imports::SET_CONTAINER_NAMESPACE => (vec![ValType::I32; 2], vec![]),
        imports::SET_CONTAINER_ARGS => (vec![ValType::I32; 2], vec![]),
        imports::BEGIN_COMPOSABLE_NODE => (vec![], vec![]),
        imports::SET_COMP_NODE_PKG => (vec![ValType::I32; 2], vec![]),
        imports::SET_COMP_NODE_PLUGIN => (vec![ValType::I32; 2], vec![]),
        imports::SET_COMP_NODE_NAME => (vec![ValType::I32; 2], vec![]),
        imports::SET_COMP_NODE_NAMESPACE => (vec![ValType::I32; 2], vec![]),
        imports::ADD_COMP_NODE_PARAM => (vec![ValType::I32; 4], vec![]),
        imports::ADD_COMP_NODE_REMAP => (vec![ValType::I32; 4], vec![]),
        imports::ADD_COMP_NODE_EXTRA_ARG => (vec![ValType::I32; 4], vec![]),
        imports::END_COMPOSABLE_NODE => (vec![], vec![]),
        imports::END_CONTAINER => (vec![], vec![]),

        // Load composable node
        imports::BEGIN_LOAD_NODE => (vec![ValType::I32; 2], vec![]),
        imports::END_LOAD_NODE => (vec![], vec![]),

        _ => anyhow::bail!("Unknown import: {name}"),
    })
}
