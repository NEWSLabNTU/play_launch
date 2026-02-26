//! Executes play_launch WASM modules, producing RecordJson.

pub mod host;
pub mod linker;
pub mod memory;

use host::LaunchHost;
use play_launch_parser::record::RecordJson;
use play_launch_wasm_common::{ABI_VERSION, ENTRY_POINT};
use std::collections::HashMap;
use wasmtime::{Engine, Linker, Module, Store};

/// Execute a WASM binary (compiled from LaunchProgram IR), returning a RecordJson.
pub fn execute_wasm(
    wasm_bytes: &[u8],
    cli_args: HashMap<String, String>,
) -> anyhow::Result<RecordJson> {
    let engine = Engine::default();
    let module = Module::new(&engine, wasm_bytes)?;

    let host = LaunchHost::new(cli_args);
    let mut store = Store::new(&engine, host);

    let mut wasmtime_linker = Linker::new(&engine);
    linker::register_imports(&mut wasmtime_linker)?;

    let instance = wasmtime_linker.instantiate(&mut store, &module)?;

    // Verify ABI version
    let abi_global = instance
        .get_global(&mut store, "abi_version")
        .ok_or_else(|| anyhow::anyhow!("Missing abi_version export"))?;
    let abi_val = abi_global.get(&mut store).i32().unwrap_or(0) as u32;
    if abi_val != ABI_VERSION {
        anyhow::bail!(
            "ABI version mismatch: expected {ABI_VERSION}, got {abi_val}"
        );
    }

    // Call the plan function
    let plan = instance
        .get_typed_func::<(), ()>(&mut store, ENTRY_POINT)?;
    plan.call(&mut store, ())?;

    // Extract results
    let host = store.into_data();
    Ok(host.into_record_json())
}

#[cfg(test)]
mod tests {
    use super::*;
    use play_launch_parser::ir::*;
    use play_launch_parser::substitution::Substitution;
    use play_launch_wasm_codegen::compile_to_wasm;
    use std::path::PathBuf;

    fn text(s: &str) -> Substitution {
        Substitution::Text(s.to_string())
    }

    fn expr(parts: Vec<Substitution>) -> Expr {
        Expr::new(parts)
    }

    fn lit(s: &str) -> Expr {
        Expr::new(vec![text(s)])
    }

    fn action(kind: ActionKind) -> Action {
        Action {
            kind,
            condition: None,
            span: None,
        }
    }

    fn program(body: Vec<Action>) -> LaunchProgram {
        LaunchProgram {
            source: PathBuf::from("test.launch.xml"),
            body,
        }
    }

    fn round_trip(prog: &LaunchProgram, args: HashMap<String, String>) -> RecordJson {
        let wasm = compile_to_wasm(prog).expect("Compilation failed");
        execute_wasm(&wasm, args).expect("Execution failed")
    }

    // --- Step 1: SpawnNode with literal fields ---

    #[test]
    fn test_spawn_node_basic() {
        let prog = program(vec![action(ActionKind::SpawnNode {
            package: lit("demo_nodes_cpp"),
            executable: lit("talker"),
            name: Some(lit("my_talker")),
            namespace: None,
            params: vec![],
            param_files: vec![],
            remaps: vec![],
            env: vec![],
            args: None,
            respawn: None,
            respawn_delay: None,
        })]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node.len(), 1);
        assert_eq!(result.node[0].executable, "talker");
        assert_eq!(result.node[0].package, Some("demo_nodes_cpp".to_string()));
        assert_eq!(result.node[0].name, Some("my_talker".to_string()));
        // cmd[0] should be the resolved exec path
        assert!(result.node[0].cmd[0].contains("talker"));
        assert!(result.node[0].cmd.contains(&"--ros-args".to_string()));
    }

    #[test]
    fn test_spawn_node_with_params() {
        let prog = program(vec![action(ActionKind::SpawnNode {
            package: lit("demo_nodes_cpp"),
            executable: lit("talker"),
            name: None,
            namespace: None,
            params: vec![ParamDecl {
                name: "rate".to_string(),
                value: lit("10.0"),
            }],
            param_files: vec![],
            remaps: vec![],
            env: vec![],
            args: None,
            respawn: None,
            respawn_delay: None,
        })]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node[0].params, vec![("rate".to_string(), "10.0".to_string())]);
        assert!(result.node[0].cmd.contains(&"rate:=10.0".to_string()));
    }

    #[test]
    fn test_spawn_node_with_remaps() {
        let prog = program(vec![action(ActionKind::SpawnNode {
            package: lit("demo_nodes_cpp"),
            executable: lit("talker"),
            name: None,
            namespace: None,
            params: vec![],
            param_files: vec![],
            remaps: vec![RemapDecl {
                from: lit("chatter"),
                to: lit("/chat"),
            }],
            env: vec![],
            args: None,
            respawn: None,
            respawn_delay: None,
        })]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node[0].remaps, vec![("chatter".to_string(), "/chat".to_string())]);
    }

    // --- Step 2: DeclareArgument + Expr resolution ---

    #[test]
    fn test_declare_arg_with_default() {
        let prog = program(vec![
            action(ActionKind::DeclareArgument {
                name: "robot".to_string(),
                default: Some(lit("bot")),
                description: None,
                choices: None,
            }),
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: Some(expr(vec![
                    Substitution::LaunchConfiguration(vec![text("robot")]),
                    text("_driver"),
                ])),
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
        ]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node[0].name, Some("bot_driver".to_string()));
    }

    #[test]
    fn test_declare_arg_cli_override() {
        let prog = program(vec![
            action(ActionKind::DeclareArgument {
                name: "robot".to_string(),
                default: Some(lit("bot")),
                description: None,
                choices: None,
            }),
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: Some(expr(vec![Substitution::LaunchConfiguration(vec![text("robot")])])),
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
        ]);

        let mut args = HashMap::new();
        args.insert("robot".to_string(), "my_robot".to_string());
        let result = round_trip(&prog, args);
        assert_eq!(result.node[0].name, Some("my_robot".to_string()));
    }

    // --- Step 3: Conditions ---

    #[test]
    fn test_condition_if_true() {
        let prog = program(vec![
            Action {
                kind: ActionKind::SpawnNode {
                    package: lit("demo_nodes_cpp"),
                    executable: lit("talker"),
                    name: Some(lit("included")),
                    namespace: None,
                    params: vec![],
                    param_files: vec![],
                    remaps: vec![],
                    env: vec![],
                    args: None,
                    respawn: None,
                    respawn_delay: None,
                },
                condition: Some(Condition::If(lit("true"))),
                span: None,
            },
            Action {
                kind: ActionKind::SpawnNode {
                    package: lit("demo_nodes_cpp"),
                    executable: lit("listener"),
                    name: Some(lit("excluded")),
                    namespace: None,
                    params: vec![],
                    param_files: vec![],
                    remaps: vec![],
                    env: vec![],
                    args: None,
                    respawn: None,
                    respawn_delay: None,
                },
                condition: Some(Condition::Unless(lit("true"))),
                span: None,
            },
        ]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node.len(), 1);
        assert_eq!(result.node[0].name, Some("included".to_string()));
    }

    #[test]
    fn test_condition_unless_false() {
        let prog = program(vec![Action {
            kind: ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: Some(lit("included")),
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            },
            condition: Some(Condition::Unless(lit("false"))),
            span: None,
        }]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node.len(), 1);
    }

    // --- Step 4: Group + scope ---

    #[test]
    fn test_group_namespace() {
        let prog = program(vec![action(ActionKind::Group {
            namespace: Some(lit("my_ns")),
            body: vec![action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: Some(lit("node1")),
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            })],
        })]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node[0].namespace, Some("/my_ns".to_string()));
    }

    #[test]
    fn test_group_scope_isolation() {
        let prog = program(vec![
            action(ActionKind::Group {
                namespace: Some(lit("scoped")),
                body: vec![action(ActionKind::SpawnNode {
                    package: lit("demo_nodes_cpp"),
                    executable: lit("talker"),
                    name: Some(lit("in_scope")),
                    namespace: None,
                    params: vec![],
                    param_files: vec![],
                    remaps: vec![],
                    env: vec![],
                    args: None,
                    respawn: None,
                    respawn_delay: None,
                })],
            }),
            // After group: namespace should be back to root
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("listener"),
                name: Some(lit("outside")),
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
        ]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node.len(), 2);
        assert_eq!(result.node[0].namespace, Some("/scoped".to_string()));
        assert_eq!(result.node[1].namespace, None); // root namespace
    }

    // --- Step 5: SetEnv/SetParameter ---

    #[test]
    fn test_set_env() {
        let prog = program(vec![
            action(ActionKind::SetEnv {
                name: "MY_VAR".to_string(),
                value: lit("hello"),
            }),
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: None,
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
        ]);

        let result = round_trip(&prog, HashMap::new());
        let env = result.node[0].env.as_ref().expect("env should be set");
        assert!(env.iter().any(|(k, v)| k == "MY_VAR" && v == "hello"));
    }

    #[test]
    fn test_set_parameter() {
        let prog = program(vec![
            action(ActionKind::SetParameter {
                name: "use_sim_time".to_string(),
                value: lit("true"),
            }),
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: None,
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
        ]);

        let result = round_trip(&prog, HashMap::new());
        let gp = result.node[0]
            .global_params
            .as_ref()
            .expect("global_params should be set");
        assert!(gp.iter().any(|(k, v)| k == "use_sim_time" && v == "True"));
    }

    #[test]
    fn test_set_remap() {
        let prog = program(vec![
            action(ActionKind::SetRemap {
                from: lit("/input"),
                to: lit("/output"),
            }),
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: None,
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
        ]);

        let result = round_trip(&prog, HashMap::new());
        assert!(result.node[0].remaps.iter().any(|(f, t)| f == "/input" && t == "/output"));
    }

    // --- Step 6: SpawnContainer + composable nodes ---

    #[test]
    fn test_spawn_container() {
        let prog = program(vec![action(ActionKind::SpawnContainer {
            package: lit("rclcpp_components"),
            executable: lit("component_container"),
            name: lit("my_container"),
            namespace: None,
            args: None,
            nodes: vec![
                ComposableNodeDecl {
                    package: lit("composition"),
                    plugin: lit("composition::Talker"),
                    name: lit("talker"),
                    namespace: None,
                    params: vec![],
                    remaps: vec![],
                    extra_args: vec![],
                    condition: None,
                    span: None,
                },
                ComposableNodeDecl {
                    package: lit("composition"),
                    plugin: lit("composition::Listener"),
                    name: lit("listener"),
                    namespace: None,
                    params: vec![],
                    remaps: vec![],
                    extra_args: vec![],
                    condition: None,
                    span: None,
                },
            ],
        })]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.container.len(), 1);
        assert_eq!(result.container[0].name, "my_container");
        assert_eq!(result.load_node.len(), 2);
        assert_eq!(result.load_node[0].node_name, "talker");
        assert_eq!(result.load_node[1].node_name, "listener");
    }

    // --- Step 7: Include (pre-linked) ---

    #[test]
    fn test_include_pre_linked() {
        let included_body = LaunchProgram {
            source: PathBuf::from("included.launch.xml"),
            body: vec![
                action(ActionKind::DeclareArgument {
                    name: "inc_name".to_string(),
                    default: Some(lit("default_name")),
                    description: None,
                    choices: None,
                }),
                action(ActionKind::SpawnNode {
                    package: lit("demo_nodes_cpp"),
                    executable: lit("talker"),
                    name: Some(expr(vec![Substitution::LaunchConfiguration(vec![text("inc_name")])])),
                    namespace: None,
                    params: vec![],
                    param_files: vec![],
                    remaps: vec![],
                    env: vec![],
                    args: None,
                    respawn: None,
                    respawn_delay: None,
                }),
            ],
        };

        let prog = program(vec![action(ActionKind::Include {
            file: lit("included.launch.xml"),
            args: vec![IncludeArg {
                name: "inc_name".to_string(),
                value: lit("override_name"),
            }],
            body: Some(Box::new(included_body)),
        })]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node.len(), 1);
        assert_eq!(result.node[0].name, Some("override_name".to_string()));
    }

    // --- Step 8: SpawnExecutable ---

    #[test]
    fn test_spawn_executable() {
        let prog = program(vec![action(ActionKind::SpawnExecutable {
            cmd: lit("my_tool"),
            name: None,
            args: vec![lit("--flag"), lit("value")],
            env: vec![],
        })]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node.len(), 1);
        assert_eq!(result.node[0].executable, "my_tool");
        assert_eq!(result.node[0].cmd, vec!["my_tool", "--flag", "value"]);
    }

    // --- Step 9: LoadComposableNode ---

    #[test]
    fn test_load_composable_node() {
        let prog = program(vec![action(ActionKind::LoadComposableNode {
            target: lit("/my_container"),
            nodes: vec![ComposableNodeDecl {
                package: lit("composition"),
                plugin: lit("composition::Talker"),
                name: lit("loaded_talker"),
                namespace: None,
                params: vec![ParamDecl {
                    name: "rate".to_string(),
                    value: lit("5.0"),
                }],
                remaps: vec![],
                extra_args: vec![("use_intra_process_comms".to_string(), "true".to_string())],
                condition: None,
                span: None,
            }],
        })]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.load_node.len(), 1);
        assert_eq!(result.load_node[0].node_name, "loaded_talker");
        assert_eq!(result.load_node[0].target_container_name, "/my_container");
        assert_eq!(result.load_node[0].params, vec![("rate".to_string(), "5.0".to_string())]);
        assert_eq!(
            result.load_node[0].extra_args.get("use_intra_process_comms"),
            Some(&"true".to_string())
        );
    }

    // --- Multiple nodes ---

    #[test]
    fn test_multiple_nodes() {
        let prog = program(vec![
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: Some(lit("node1")),
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("listener"),
                name: Some(lit("node2")),
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
        ]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node.len(), 2);
        assert_eq!(result.node[0].name, Some("node1".to_string()));
        assert_eq!(result.node[1].name, Some("node2".to_string()));
    }

    // --- SetVariable ---

    #[test]
    fn test_set_variable() {
        let prog = program(vec![
            action(ActionKind::SetVariable {
                name: "my_var".to_string(),
                value: lit("hello_world"),
            }),
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: Some(expr(vec![Substitution::LaunchConfiguration(vec![text("my_var")])])),
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
        ]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node[0].name, Some("hello_world".to_string()));
    }

    // --- UnsetEnv ---

    #[test]
    fn test_unset_env() {
        let prog = program(vec![
            action(ActionKind::SetEnv {
                name: "VAR1".to_string(),
                value: lit("val1"),
            }),
            action(ActionKind::SetEnv {
                name: "VAR2".to_string(),
                value: lit("val2"),
            }),
            action(ActionKind::UnsetEnv {
                name: "VAR1".to_string(),
            }),
            action(ActionKind::SpawnNode {
                package: lit("demo_nodes_cpp"),
                executable: lit("talker"),
                name: None,
                namespace: None,
                params: vec![],
                param_files: vec![],
                remaps: vec![],
                env: vec![],
                args: None,
                respawn: None,
                respawn_delay: None,
            }),
        ]);

        let result = round_trip(&prog, HashMap::new());
        let env = result.node[0].env.as_ref().expect("env should be set");
        assert!(!env.iter().any(|(k, _)| k == "VAR1"));
        assert!(env.iter().any(|(k, v)| k == "VAR2" && v == "val2"));
    }

    // --- Respawn ---

    #[test]
    fn test_respawn() {
        let prog = program(vec![action(ActionKind::SpawnNode {
            package: lit("demo_nodes_cpp"),
            executable: lit("talker"),
            name: None,
            namespace: None,
            params: vec![],
            param_files: vec![],
            remaps: vec![],
            env: vec![],
            args: None,
            respawn: Some(lit("true")),
            respawn_delay: Some(lit("2.5")),
        })]);

        let result = round_trip(&prog, HashMap::new());
        assert_eq!(result.node[0].respawn, Some(true));
        assert_eq!(result.node[0].respawn_delay, Some(2.5));
    }

    // --- Node env ---

    #[test]
    fn test_node_env() {
        let prog = program(vec![action(ActionKind::SpawnNode {
            package: lit("demo_nodes_cpp"),
            executable: lit("talker"),
            name: None,
            namespace: None,
            params: vec![],
            param_files: vec![],
            remaps: vec![],
            env: vec![EnvDecl {
                name: lit("NODE_VAR"),
                value: lit("node_val"),
            }],
            args: None,
            respawn: None,
            respawn_delay: None,
        })]);

        let result = round_trip(&prog, HashMap::new());
        let env = result.node[0].env.as_ref().expect("env should be set");
        assert!(env.iter().any(|(k, v)| k == "NODE_VAR" && v == "node_val"));
    }
}
