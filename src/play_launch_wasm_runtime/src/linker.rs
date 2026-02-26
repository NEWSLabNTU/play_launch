//! Host import registration: all 56 host functions on wasmtime::Linker.

use crate::host::{
    ComposableNodeBuilder, ContainerBuilder, ExecutableBuilder, LaunchHost, LoadNodeBuilder,
    NodeBuilder, ScopeSnapshot,
};
use crate::memory::{get_memory, read_guest_string, read_optional_string, write_guest_string};
use anyhow::{Context, Result};
use play_launch_parser::condition::is_truthy;
use play_launch_wasm_common::{imports, HOST_MODULE};
use wasmtime::{Caller, Linker};

pub fn register_imports(linker: &mut Linker<LaunchHost>) -> Result<()> {
    // --- Context operations ---

    linker.func_wrap(
        HOST_MODULE,
        imports::DECLARE_ARG,
        |mut caller: Caller<'_, LaunchHost>,
         name_ptr: i32,
         name_len: i32,
         default_ptr: i32,
         default_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            let default = read_optional_string(&mem, &caller, default_ptr, default_len)?;
            let host = caller.data_mut();
            if host.context.get_configuration(&name).is_none() {
                if let Some(d) = default {
                    host.context.set_configuration(name, d);
                }
            }
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_VAR,
        |mut caller: Caller<'_, LaunchHost>,
         name_ptr: i32,
         name_len: i32,
         val_ptr: i32,
         val_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            let value = read_guest_string(&mem, &caller, val_ptr, val_len)?;
            caller.data_mut().context.set_configuration(name, value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::RESOLVE_VAR,
        |mut caller: Caller<'_, LaunchHost>, name_ptr: i32, name_len: i32| -> Result<(i32, i32)> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;

            let value = if name == "__dirname" {
                caller
                    .data()
                    .context
                    .current_dir()
                    .map(|p| p.to_string_lossy().to_string())
                    .unwrap_or_default()
            } else if name == "__filename" {
                caller
                    .data()
                    .context
                    .current_filename()
                    .unwrap_or_default()
            } else if let Some(stripped) = name.strip_prefix("__anon_") {
                use std::collections::hash_map::DefaultHasher;
                use std::hash::{Hash, Hasher};
                let mut hasher = DefaultHasher::new();
                stripped.hash(&mut hasher);
                format!("{stripped}_{:016x}", hasher.finish())
            } else {
                caller
                    .data()
                    .context
                    .get_configuration_lenient(&name)
                    .unwrap_or_default()
            };

            write_guest_string(&mem, &mut caller, &value)
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_ENV,
        |mut caller: Caller<'_, LaunchHost>,
         name_ptr: i32,
         name_len: i32,
         val_ptr: i32,
         val_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            let value = read_guest_string(&mem, &caller, val_ptr, val_len)?;
            caller
                .data_mut()
                .context
                .set_environment_variable(name, value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::UNSET_ENV,
        |mut caller: Caller<'_, LaunchHost>, name_ptr: i32, name_len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            caller.data_mut().context.unset_environment_variable(&name);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::PUSH_NAMESPACE,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let ns = read_guest_string(&mem, &caller, ptr, len)?;
            caller.data_mut().context.push_namespace(ns);
            Ok(())
        },
    )?;

    linker.func_wrap(HOST_MODULE, imports::POP_NAMESPACE, |mut caller: Caller<'_, LaunchHost>| {
        caller.data_mut().context.pop_namespace();
    })?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_GLOBAL_PARAM,
        |mut caller: Caller<'_, LaunchHost>,
         name_ptr: i32,
         name_len: i32,
         val_ptr: i32,
         val_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            let value = read_guest_string(&mem, &caller, val_ptr, val_len)?;
            caller.data_mut().context.set_global_parameter(name, value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_REMAP,
        |mut caller: Caller<'_, LaunchHost>,
         from_ptr: i32,
         from_len: i32,
         to_ptr: i32,
         to_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let from = read_guest_string(&mem, &caller, from_ptr, from_len)?;
            let to = read_guest_string(&mem, &caller, to_ptr, to_len)?;
            caller.data_mut().context.add_remapping(from, to);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SAVE_SCOPE,
        |mut caller: Caller<'_, LaunchHost>| {
            let host = caller.data_mut();
            let snapshot = ScopeSnapshot {
                namespace_depth: host.context.namespace_depth(),
                remapping_count: host.context.remapping_count(),
            };
            host.scope_stack.push(snapshot);
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::RESTORE_SCOPE,
        |mut caller: Caller<'_, LaunchHost>| {
            let host = caller.data_mut();
            if let Some(snapshot) = host.scope_stack.pop() {
                host.context
                    .restore_namespace_depth(snapshot.namespace_depth);
                host.context
                    .restore_remapping_count(snapshot.remapping_count);
            }
        },
    )?;

    // --- Package resolution ---

    linker.func_wrap(
        HOST_MODULE,
        imports::FIND_PACKAGE_SHARE,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<(i32, i32)> {
            let mem = get_memory(&mut caller)?;
            let pkg = read_guest_string(&mem, &caller, ptr, len)?;
            let share_dir = find_package_share_dir(&pkg);
            write_guest_string(&mem, &mut caller, &share_dir)
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::RESOLVE_EXEC_PATH,
        |mut caller: Caller<'_, LaunchHost>,
         pkg_ptr: i32,
         pkg_len: i32,
         exec_ptr: i32,
         exec_len: i32|
         -> Result<(i32, i32)> {
            let mem = get_memory(&mut caller)?;
            let pkg = read_guest_string(&mem, &caller, pkg_ptr, pkg_len)?;
            let exec = read_guest_string(&mem, &caller, exec_ptr, exec_len)?;
            let path = play_launch_parser::record::resolve_exec_path(&pkg, &exec);
            write_guest_string(&mem, &mut caller, &path)
        },
    )?;

    // --- Substitution evaluation ---

    linker.func_wrap(
        HOST_MODULE,
        imports::EVAL_ENV_VAR,
        |mut caller: Caller<'_, LaunchHost>,
         name_ptr: i32,
         name_len: i32,
         default_ptr: i32,
         default_len: i32|
         -> Result<(i32, i32)> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            let default = read_optional_string(&mem, &caller, default_ptr, default_len)?;
            let value = std::env::var(&name).ok().or(default).unwrap_or_default();
            write_guest_string(&mem, &mut caller, &value)
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::EVAL_COMMAND,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<(i32, i32)> {
            let mem = get_memory(&mut caller)?;
            let cmd = read_guest_string(&mem, &caller, ptr, len)?;
            let output = std::process::Command::new("sh")
                .arg("-c")
                .arg(&cmd)
                .output()?;
            let result = String::from_utf8_lossy(&output.stdout).trim().to_string();
            write_guest_string(&mem, &mut caller, &result)
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::EVAL_PYTHON_EXPR,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<(i32, i32)> {
            let mem = get_memory(&mut caller)?;
            let expr = read_guest_string(&mem, &caller, ptr, len)?;
            let result = eval_python_expr_simple(&expr);
            write_guest_string(&mem, &mut caller, &result)
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::IS_TRUTHY,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<i32> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            Ok(if is_truthy(&value) { 1 } else { 0 })
        },
    )?;

    // --- String operations ---

    linker.func_wrap(
        HOST_MODULE,
        imports::CONCAT,
        |mut caller: Caller<'_, LaunchHost>,
         a_ptr: i32,
         a_len: i32,
         b_ptr: i32,
         b_len: i32|
         -> Result<(i32, i32)> {
            let mem = get_memory(&mut caller)?;
            let a = read_guest_string(&mem, &caller, a_ptr, a_len)?;
            let b = read_guest_string(&mem, &caller, b_ptr, b_len)?;
            let result = format!("{a}{b}");
            write_guest_string(&mem, &mut caller, &result)
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::STR_EQUALS,
        |mut caller: Caller<'_, LaunchHost>,
         a_ptr: i32,
         a_len: i32,
         b_ptr: i32,
         b_len: i32|
         -> Result<i32> {
            let mem = get_memory(&mut caller)?;
            let a = read_guest_string(&mem, &caller, a_ptr, a_len)?;
            let b = read_guest_string(&mem, &caller, b_ptr, b_len)?;
            Ok(if a == b { 1 } else { 0 })
        },
    )?;

    // --- Node builder ---

    linker.func_wrap(HOST_MODULE, imports::BEGIN_NODE, |mut caller: Caller<'_, LaunchHost>| {
        caller.data_mut().node_builder = Some(NodeBuilder::default());
    })?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_NODE_PKG,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("SET_NODE_PKG called without active node builder")?;
            b.package = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_NODE_EXEC,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("SET_NODE_EXEC called without active node builder")?;
            b.executable = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_NODE_NAME,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("SET_NODE_NAME called without active node builder")?;
            b.name = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_NODE_NAMESPACE,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("SET_NODE_NAMESPACE called without active node builder")?;
            b.namespace = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::ADD_NODE_PARAM,
        |mut caller: Caller<'_, LaunchHost>,
         name_ptr: i32,
         name_len: i32,
         val_ptr: i32,
         val_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            let value = read_guest_string(&mem, &caller, val_ptr, val_len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("ADD_NODE_PARAM called without active node builder")?;
            b.params.push((name, value));
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::ADD_NODE_PARAM_FILE,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("ADD_NODE_PARAM_FILE called without active node builder")?;
            b.param_files.push(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::ADD_NODE_REMAP,
        |mut caller: Caller<'_, LaunchHost>,
         from_ptr: i32,
         from_len: i32,
         to_ptr: i32,
         to_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let from = read_guest_string(&mem, &caller, from_ptr, from_len)?;
            let to = read_guest_string(&mem, &caller, to_ptr, to_len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("ADD_NODE_REMAP called without active node builder")?;
            b.remaps.push((from, to));
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::ADD_NODE_ENV,
        |mut caller: Caller<'_, LaunchHost>,
         name_ptr: i32,
         name_len: i32,
         val_ptr: i32,
         val_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            let value = read_guest_string(&mem, &caller, val_ptr, val_len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("ADD_NODE_ENV called without active node builder")?;
            b.env.push((name, value));
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_NODE_ARGS,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("SET_NODE_ARGS called without active node builder")?;
            b.args = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_NODE_RESPAWN,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("SET_NODE_RESPAWN called without active node builder")?;
            b.respawn = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_NODE_RESPAWN_DELAY,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().node_builder.as_mut()
                .context("SET_NODE_RESPAWN_DELAY called without active node builder")?;
            b.respawn_delay = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::END_NODE,
        |mut caller: Caller<'_, LaunchHost>| -> Result<()> {
            caller.data_mut().end_node()
        },
    )?;

    // --- Executable builder ---

    linker.func_wrap(
        HOST_MODULE,
        imports::BEGIN_EXECUTABLE,
        |mut caller: Caller<'_, LaunchHost>| {
            caller.data_mut().exec_builder = Some(ExecutableBuilder::default());
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_EXEC_CMD,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().exec_builder.as_mut()
                .context("SET_EXEC_CMD called without active executable builder")?;
            b.cmd = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_EXEC_NAME,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().exec_builder.as_mut()
                .context("SET_EXEC_NAME called without active executable builder")?;
            b.name = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::ADD_EXEC_ARG,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().exec_builder.as_mut()
                .context("ADD_EXEC_ARG called without active executable builder")?;
            b.args.push(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::ADD_EXEC_ENV,
        |mut caller: Caller<'_, LaunchHost>,
         name_ptr: i32,
         name_len: i32,
         val_ptr: i32,
         val_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            let value = read_guest_string(&mem, &caller, val_ptr, val_len)?;
            let b = caller.data_mut().exec_builder.as_mut()
                .context("ADD_EXEC_ENV called without active executable builder")?;
            b.env.push((name, value));
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::END_EXECUTABLE,
        |mut caller: Caller<'_, LaunchHost>| -> Result<()> {
            caller.data_mut().end_executable()
        },
    )?;

    // --- Container builder ---

    linker.func_wrap(
        HOST_MODULE,
        imports::BEGIN_CONTAINER,
        |mut caller: Caller<'_, LaunchHost>| {
            caller.data_mut().container_builder = Some(ContainerBuilder::default());
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_CONTAINER_PKG,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().container_builder.as_mut()
                .context("SET_CONTAINER_PKG called without active container builder")?;
            b.package = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_CONTAINER_EXEC,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().container_builder.as_mut()
                .context("SET_CONTAINER_EXEC called without active container builder")?;
            b.executable = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_CONTAINER_NAME,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().container_builder.as_mut()
                .context("SET_CONTAINER_NAME called without active container builder")?;
            b.name = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_CONTAINER_NAMESPACE,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().container_builder.as_mut()
                .context("SET_CONTAINER_NAMESPACE called without active container builder")?;
            b.namespace = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_CONTAINER_ARGS,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().container_builder.as_mut()
                .context("SET_CONTAINER_ARGS called without active container builder")?;
            b.args = Some(value);
            Ok(())
        },
    )?;

    // --- Composable node builder ---

    linker.func_wrap(
        HOST_MODULE,
        imports::BEGIN_COMPOSABLE_NODE,
        |mut caller: Caller<'_, LaunchHost>| {
            caller.data_mut().comp_node_builder = Some(ComposableNodeBuilder::default());
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_COMP_NODE_PKG,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().comp_node_builder.as_mut()
                .context("SET_COMP_NODE_PKG called without active composable node builder")?;
            b.package = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_COMP_NODE_PLUGIN,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().comp_node_builder.as_mut()
                .context("SET_COMP_NODE_PLUGIN called without active composable node builder")?;
            b.plugin = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_COMP_NODE_NAME,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().comp_node_builder.as_mut()
                .context("SET_COMP_NODE_NAME called without active composable node builder")?;
            b.name = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::SET_COMP_NODE_NAMESPACE,
        |mut caller: Caller<'_, LaunchHost>, ptr: i32, len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let value = read_guest_string(&mem, &caller, ptr, len)?;
            let b = caller.data_mut().comp_node_builder.as_mut()
                .context("SET_COMP_NODE_NAMESPACE called without active composable node builder")?;
            b.namespace = Some(value);
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::ADD_COMP_NODE_PARAM,
        |mut caller: Caller<'_, LaunchHost>,
         name_ptr: i32,
         name_len: i32,
         val_ptr: i32,
         val_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let name = read_guest_string(&mem, &caller, name_ptr, name_len)?;
            let value = read_guest_string(&mem, &caller, val_ptr, val_len)?;
            let b = caller.data_mut().comp_node_builder.as_mut()
                .context("ADD_COMP_NODE_PARAM called without active composable node builder")?;
            b.params.push((name, value));
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::ADD_COMP_NODE_REMAP,
        |mut caller: Caller<'_, LaunchHost>,
         from_ptr: i32,
         from_len: i32,
         to_ptr: i32,
         to_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let from = read_guest_string(&mem, &caller, from_ptr, from_len)?;
            let to = read_guest_string(&mem, &caller, to_ptr, to_len)?;
            let b = caller.data_mut().comp_node_builder.as_mut()
                .context("ADD_COMP_NODE_REMAP called without active composable node builder")?;
            b.remaps.push((from, to));
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::ADD_COMP_NODE_EXTRA_ARG,
        |mut caller: Caller<'_, LaunchHost>,
         key_ptr: i32,
         key_len: i32,
         val_ptr: i32,
         val_len: i32|
         -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let key = read_guest_string(&mem, &caller, key_ptr, key_len)?;
            let value = read_guest_string(&mem, &caller, val_ptr, val_len)?;
            let b = caller.data_mut().comp_node_builder.as_mut()
                .context("ADD_COMP_NODE_EXTRA_ARG called without active composable node builder")?;
            b.extra_args.push((key, value));
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::END_COMPOSABLE_NODE,
        |mut caller: Caller<'_, LaunchHost>| -> Result<()> {
            caller.data_mut().end_composable_node()
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::END_CONTAINER,
        |mut caller: Caller<'_, LaunchHost>| -> Result<()> {
            caller.data_mut().end_container()
        },
    )?;

    // --- Load composable node ---

    linker.func_wrap(
        HOST_MODULE,
        imports::BEGIN_LOAD_NODE,
        |mut caller: Caller<'_, LaunchHost>, target_ptr: i32, target_len: i32| -> Result<()> {
            let mem = get_memory(&mut caller)?;
            let target = read_guest_string(&mem, &caller, target_ptr, target_len)?;
            caller.data_mut().load_node_builder = Some(LoadNodeBuilder { target });
            Ok(())
        },
    )?;

    linker.func_wrap(
        HOST_MODULE,
        imports::END_LOAD_NODE,
        |mut caller: Caller<'_, LaunchHost>| -> Result<()> {
            caller.data_mut().end_load_node()
        },
    )?;

    Ok(())
}

fn find_package_share_dir(package: &str) -> String {
    if let Ok(prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
        for prefix in prefix_path.split(':') {
            let share = format!("{prefix}/share/{package}");
            if std::path::Path::new(&share).exists() {
                return share;
            }
        }
    }
    format!("/opt/ros/humble/share/{package}")
}

fn eval_python_expr_simple(expr: &str) -> String {
    let trimmed = expr.trim();
    if let Ok(v) = trimmed.parse::<i64>() {
        return v.to_string();
    }
    if let Ok(v) = trimmed.parse::<f64>() {
        return v.to_string();
    }
    if let Some((a, b)) = trimmed.split_once("==") {
        let a = a.trim().trim_matches(|c| c == '\'' || c == '"');
        let b = b.trim().trim_matches(|c| c == '\'' || c == '"');
        return if a == b { "true" } else { "false" }.to_string();
    }
    if let Some((a, b)) = trimmed.split_once("!=") {
        let a = a.trim().trim_matches(|c| c == '\'' || c == '"');
        let b = b.trim().trim_matches(|c| c == '\'' || c == '"');
        return if a != b { "true" } else { "false" }.to_string();
    }
    trimmed.to_string()
}
