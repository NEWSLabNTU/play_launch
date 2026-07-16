#![cfg(feature = "ir")]

use play_launch_parser::{
    analyze_launch_file,
    ir::{ActionKind, Condition},
    substitution::Substitution,
};
use std::io::Write;
use tempfile::NamedTempFile;

/// Helper: write XML to a temp file and analyze it.
fn analyze_xml(xml: &str) -> play_launch_parser::ir::LaunchProgram {
    let mut file = NamedTempFile::with_suffix(".launch.xml").unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();
    analyze_launch_file(file.path()).expect("analyze_launch_file should succeed")
}

#[test]
fn test_ir_simple_node() {
    let program = analyze_xml(
        r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" name="my_talker" />
        </launch>"#,
    );

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::SpawnNode {
            package,
            executable,
            name,
            ..
        } => {
            assert_eq!(package.as_literal(), Some("demo_nodes_cpp"));
            assert_eq!(executable.as_literal(), Some("talker"));
            assert_eq!(
                name.as_ref().and_then(|n| n.as_literal()),
                Some("my_talker")
            );
        }
        other => panic!("Expected SpawnNode, got {:?}", other),
    }
    // Span should be set
    assert!(program.body[0].span.is_some());
    // No condition
    assert!(program.body[0].condition.is_none());
}

#[test]
fn test_ir_conditional_branches() {
    let program = analyze_xml(
        r#"<launch>
            <arg name="use_sim" default="true" />
            <node pkg="sim_pkg" exec="sim_node" if="$(var use_sim)" />
            <node pkg="real_pkg" exec="real_node" unless="$(var use_sim)" />
        </launch>"#,
    );

    // Should have 3 actions: DeclareArgument + 2 nodes (both branches preserved)
    assert_eq!(program.body.len(), 3);

    // First: DeclareArgument for "use_sim"
    match &program.body[0].kind {
        ActionKind::DeclareArgument { name, default, .. } => {
            assert_eq!(name, "use_sim");
            assert!(default.is_some());
            assert_eq!(default.as_ref().unwrap().as_literal(), Some("true"));
        }
        other => panic!("Expected DeclareArgument, got {:?}", other),
    }

    // Second: SpawnNode with if condition
    match &program.body[1].condition {
        Some(Condition::If(expr)) => {
            // Should contain a LaunchConfiguration substitution
            assert!(
                expr.parts
                    .iter()
                    .any(|s| matches!(s, Substitution::LaunchConfiguration(_))),
                "If condition should contain LaunchConfiguration"
            );
        }
        other => panic!("Expected If condition, got {:?}", other),
    }
    match &program.body[1].kind {
        ActionKind::SpawnNode { package, .. } => {
            assert_eq!(package.as_literal(), Some("sim_pkg"));
        }
        other => panic!("Expected SpawnNode, got {:?}", other),
    }

    // Third: SpawnNode with unless condition
    match &program.body[2].condition {
        Some(Condition::Unless(expr)) => {
            assert!(
                expr.parts
                    .iter()
                    .any(|s| matches!(s, Substitution::LaunchConfiguration(_))),
                "Unless condition should contain LaunchConfiguration"
            );
        }
        other => panic!("Expected Unless condition, got {:?}", other),
    }
    match &program.body[2].kind {
        ActionKind::SpawnNode { package, .. } => {
            assert_eq!(package.as_literal(), Some("real_pkg"));
        }
        other => panic!("Expected SpawnNode, got {:?}", other),
    }
}

#[test]
fn test_ir_group_scoping() {
    let program = analyze_xml(
        r#"<launch>
            <group><push-ros-namespace namespace="robot1"/>
                <node pkg="demo_nodes_cpp" exec="talker" />
                <node pkg="demo_nodes_cpp" exec="listener" />
            </group>
        </launch>"#,
    );

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::Group { namespace, body } => {
            // Real ROS 2 `<group>` has no `ns`/namespace attribute of its own
            // (see `GroupAction::from_entity`) — namespacing is done via a
            // nested `<push-ros-namespace>` action, which is represented as
            // its own `PushNamespace` entry in `body`, same as it would be
            // anywhere else in the tree (see `test_ir_push_namespace`).
            assert!(namespace.is_none());
            assert_eq!(body.len(), 3);
            match &body[0].kind {
                ActionKind::PushNamespace { namespace } => {
                    assert_eq!(namespace.as_literal(), Some("robot1"));
                }
                other => panic!("Expected PushNamespace, got {:?}", other),
            }
            assert!(matches!(&body[1].kind, ActionKind::SpawnNode { .. }));
            assert!(matches!(&body[2].kind, ActionKind::SpawnNode { .. }));
        }
        other => panic!("Expected Group, got {:?}", other),
    }
}

#[test]
fn test_ir_variable_expressions() {
    let program = analyze_xml(
        r#"<launch>
            <arg name="robot_name" default="turtlebot" />
            <node pkg="robot_pkg" exec="driver" name="$(var robot_name)_driver" />
        </launch>"#,
    );

    assert_eq!(program.body.len(), 2);

    // The node name should be an Expr with substitutions, NOT a resolved string
    match &program.body[1].kind {
        ActionKind::SpawnNode { name, .. } => {
            let name_expr = name.as_ref().expect("Node should have a name");
            // Should NOT be a literal (it contains substitutions)
            assert!(
                !name_expr.is_literal(),
                "Name should contain substitutions, not be a literal"
            );
            // Should contain at least a LaunchConfiguration substitution
            assert!(
                name_expr
                    .parts
                    .iter()
                    .any(|s| matches!(s, Substitution::LaunchConfiguration(_))),
                "Name should contain LaunchConfiguration"
            );
        }
        other => panic!("Expected SpawnNode, got {:?}", other),
    }
}

#[test]
fn test_ir_let_and_arg() {
    let program = analyze_xml(
        r#"<launch>
            <arg name="base" default="myrobot" />
            <let name="full_name" value="$(var base)_v2" />
            <node pkg="pkg" exec="node" name="$(var full_name)" />
        </launch>"#,
    );

    assert_eq!(program.body.len(), 3);

    // First: DeclareArgument
    match &program.body[0].kind {
        ActionKind::DeclareArgument { name, .. } => {
            assert_eq!(name, "base");
        }
        other => panic!("Expected DeclareArgument, got {:?}", other),
    }

    // Second: SetVariable
    match &program.body[1].kind {
        ActionKind::SetVariable { name, value } => {
            assert_eq!(name, "full_name");
            // Value should contain substitutions
            assert!(
                value
                    .parts
                    .iter()
                    .any(|s| matches!(s, Substitution::LaunchConfiguration(_))),
                "Let value should contain LaunchConfiguration"
            );
        }
        other => panic!("Expected SetVariable, got {:?}", other),
    }
}

#[test]
fn test_ir_include_tree() {
    // Create the included file first
    let mut inner_file = NamedTempFile::with_suffix(".launch.xml").unwrap();
    inner_file
        .write_all(
            br#"<launch>
                <node pkg="inner_pkg" exec="inner_node" />
            </launch>"#,
        )
        .unwrap();
    inner_file.flush().unwrap();
    let inner_path = inner_file.path().to_str().unwrap().to_string();

    // Create the outer file that includes the inner
    let xml = format!(
        r#"<launch>
            <include file="{}" />
        </launch>"#,
        inner_path
    );
    let mut outer_file = NamedTempFile::with_suffix(".launch.xml").unwrap();
    outer_file.write_all(xml.as_bytes()).unwrap();
    outer_file.flush().unwrap();

    let program = analyze_launch_file(outer_file.path()).unwrap();

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::Include { file, body, .. } => {
            assert_eq!(file.as_literal(), Some(inner_path.as_str()));
            // body should be Some — the included file was parsed
            let body = body.as_ref().expect("Include body should be set");
            assert_eq!(body.body.len(), 1);
            assert!(matches!(&body.body[0].kind, ActionKind::SpawnNode { .. }));
        }
        other => panic!("Expected Include, got {:?}", other),
    }
}

#[test]
fn test_ir_container_with_composable_nodes() {
    let program = analyze_xml(
        r#"<launch>
            <node_container name="my_container" pkg="rclcpp_components" exec="component_container">
                <composable_node pkg="composition" plugin="composition::Talker" name="talker" />
                <composable_node pkg="composition" plugin="composition::Listener" name="listener" />
            </node_container>
        </launch>"#,
    );

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::SpawnContainer {
            name,
            package,
            executable,
            nodes,
            ..
        } => {
            assert_eq!(name.as_literal(), Some("my_container"));
            assert_eq!(package.as_literal(), Some("rclcpp_components"));
            assert_eq!(executable.as_literal(), Some("component_container"));
            assert_eq!(nodes.len(), 2);
            assert_eq!(nodes[0].name.as_literal(), Some("talker"));
            assert_eq!(nodes[1].name.as_literal(), Some("listener"));
        }
        other => panic!("Expected SpawnContainer, got {:?}", other),
    }
}

#[test]
fn test_ir_set_env_and_unset_env() {
    let program = analyze_xml(
        r#"<launch>
            <set_env name="MY_VAR" value="hello" />
            <unset_env name="MY_VAR" />
        </launch>"#,
    );

    assert_eq!(program.body.len(), 2);
    match &program.body[0].kind {
        ActionKind::SetEnv { name, value } => {
            assert_eq!(name, "MY_VAR");
            assert_eq!(value.as_literal(), Some("hello"));
        }
        other => panic!("Expected SetEnv, got {:?}", other),
    }
    match &program.body[1].kind {
        ActionKind::UnsetEnv { name } => {
            assert_eq!(name, "MY_VAR");
        }
        other => panic!("Expected UnsetEnv, got {:?}", other),
    }
}

#[test]
fn test_ir_set_parameter() {
    let program = analyze_xml(
        r#"<launch>
            <set_parameter name="use_sim_time" value="true" />
        </launch>"#,
    );

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::SetParameter { name, value } => {
            assert_eq!(name, "use_sim_time");
            assert_eq!(value.as_literal(), Some("true"));
        }
        other => panic!("Expected SetParameter, got {:?}", other),
    }
}

#[test]
fn test_ir_push_namespace() {
    let program = analyze_xml(
        r#"<launch>
            <group>
                <push-ros-namespace namespace="robot1" />
                <node pkg="pkg" exec="node" />
            </group>
        </launch>"#,
    );

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::Group { body, .. } => {
            assert_eq!(body.len(), 2);
            match &body[0].kind {
                ActionKind::PushNamespace { namespace } => {
                    assert_eq!(namespace.as_literal(), Some("robot1"));
                }
                other => panic!("Expected PushNamespace, got {:?}", other),
            }
            assert!(matches!(&body[1].kind, ActionKind::SpawnNode { .. }));
        }
        other => panic!("Expected Group, got {:?}", other),
    }
}

#[test]
fn test_ir_executable() {
    let program = analyze_xml(
        r#"<launch>
            <executable cmd="rviz2" name="my_rviz">
                <arg value="--display-config" />
                <arg value="default.rviz" />
            </executable>
        </launch>"#,
    );

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::SpawnExecutable {
            cmd, name, args, ..
        } => {
            assert_eq!(cmd.as_literal(), Some("rviz2"));
            assert_eq!(name.as_ref().and_then(|n| n.as_literal()), Some("my_rviz"));
            assert_eq!(args.len(), 2);
        }
        other => panic!("Expected SpawnExecutable, got {:?}", other),
    }
}

#[test]
fn test_ir_load_composable_node() {
    let program = analyze_xml(
        r#"<launch>
            <load_composable_node target="/my_container">
                <composable_node pkg="composition" plugin="composition::Talker" name="talker" />
            </load_composable_node>
        </launch>"#,
    );

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::LoadComposableNode { target, nodes } => {
            assert_eq!(target.as_literal(), Some("/my_container"));
            assert_eq!(nodes.len(), 1);
            assert_eq!(nodes[0].name.as_literal(), Some("talker"));
        }
        other => panic!("Expected LoadComposableNode, got {:?}", other),
    }
}

#[test]
fn test_ir_span_line_numbers() {
    let program = analyze_xml(
        r#"<launch>
            <node pkg="pkg1" exec="exec1" />
            <node pkg="pkg2" exec="exec2" />
        </launch>"#,
    );

    assert_eq!(program.body.len(), 2);

    let span1 = program.body[0].span.as_ref().expect("should have span");
    let span2 = program.body[1].span.as_ref().expect("should have span");

    // Line numbers should be different (node2 is on a later line)
    assert!(
        span2.line > span1.line,
        "Second node should be on a later line"
    );
}

#[test]
fn test_ir_include_with_args() {
    // Create inner file expecting an arg
    let mut inner = NamedTempFile::with_suffix(".launch.xml").unwrap();
    inner
        .write_all(
            br#"<launch>
                <arg name="node_name" default="default" />
                <node pkg="pkg" exec="exec" name="$(var node_name)" />
            </launch>"#,
        )
        .unwrap();
    inner.flush().unwrap();
    let inner_path = inner.path().to_str().unwrap().to_string();

    let xml = format!(
        r#"<launch>
            <include file="{}">
                <arg name="node_name" value="custom_name" />
            </include>
        </launch>"#,
        inner_path
    );

    let mut outer = NamedTempFile::with_suffix(".launch.xml").unwrap();
    outer.write_all(xml.as_bytes()).unwrap();
    outer.flush().unwrap();

    let program = analyze_launch_file(outer.path()).unwrap();

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::Include { args, body, .. } => {
            assert_eq!(args.len(), 1);
            assert_eq!(args[0].name, "node_name");
            assert_eq!(args[0].value.as_literal(), Some("custom_name"));

            let body = body.as_ref().expect("Include body should be set");
            // Inner file has 2 actions: DeclareArgument + SpawnNode
            assert_eq!(body.body.len(), 2);
        }
        other => panic!("Expected Include, got {:?}", other),
    }
}

#[test]
fn test_ir_set_remap() {
    let program = analyze_xml(
        r#"<launch>
            <set_remap from="/old_topic" to="/new_topic" />
        </launch>"#,
    );

    assert_eq!(program.body.len(), 1);
    match &program.body[0].kind {
        ActionKind::SetRemap { from, to } => {
            assert_eq!(from.as_literal(), Some("/old_topic"));
            assert_eq!(to.as_literal(), Some("/new_topic"));
        }
        other => panic!("Expected SetRemap, got {:?}", other),
    }
}

// --- YAML IR tests ---

#[test]
fn test_ir_yaml_arg_declarations() {
    let mut yaml_file = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    yaml_file
        .write_all(
            b"launch:\n  - arg:\n      name: my_var\n      default: my_value\n  - arg:\n      name: other_var\n      default: other_value\n",
        )
        .unwrap();
    yaml_file.flush().unwrap();

    let program = analyze_launch_file(yaml_file.path()).unwrap();
    assert_eq!(program.body.len(), 2);

    match &program.body[0].kind {
        ActionKind::DeclareArgument { name, default, .. } => {
            assert_eq!(name, "my_var");
            assert_eq!(default.as_ref().unwrap().as_literal(), Some("my_value"));
        }
        other => panic!("Expected DeclareArgument, got {:?}", other),
    }
    match &program.body[1].kind {
        ActionKind::DeclareArgument { name, default, .. } => {
            assert_eq!(name, "other_var");
            assert_eq!(default.as_ref().unwrap().as_literal(), Some("other_value"));
        }
        other => panic!("Expected DeclareArgument, got {:?}", other),
    }
}

#[test]
fn test_ir_yaml_with_description() {
    let mut yaml_file = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    yaml_file
        .write_all(
            b"launch:\n  - arg:\n      name: robot_type\n      default: turtlebot\n      description: Type of robot to spawn\n",
        )
        .unwrap();
    yaml_file.flush().unwrap();

    let program = analyze_launch_file(yaml_file.path()).unwrap();
    assert_eq!(program.body.len(), 1);

    match &program.body[0].kind {
        ActionKind::DeclareArgument {
            name,
            default,
            description,
            ..
        } => {
            assert_eq!(name, "robot_type");
            assert_eq!(default.as_ref().unwrap().as_literal(), Some("turtlebot"));
            assert_eq!(description.as_deref(), Some("Type of robot to spawn"));
        }
        other => panic!("Expected DeclareArgument, got {:?}", other),
    }
}

#[test]
fn test_ir_yaml_empty_launch() {
    let mut yaml_file = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    yaml_file
        .write_all(b"# empty launch file\nother_key: value\n")
        .unwrap();
    yaml_file.flush().unwrap();

    let program = analyze_launch_file(yaml_file.path()).unwrap();
    assert!(program.body.is_empty());
}

#[test]
fn test_ir_yaml_include_produces_declare_argument() {
    // YAML preset
    let mut yaml_file = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    yaml_file
        .write_all(
            b"launch:\n  - arg:\n      name: velocity_smoother_type\n      default: JointTrajectoryController\n",
        )
        .unwrap();
    yaml_file.flush().unwrap();
    let yaml_path = yaml_file.path().to_str().unwrap().to_string();

    // XML includes the YAML preset
    let xml = format!(
        r#"<launch>
            <include file="{}" />
            <node pkg="my_pkg" exec="my_exec" name="$(var velocity_smoother_type)" />
        </launch>"#,
        yaml_path
    );
    let mut outer = NamedTempFile::with_suffix(".launch.xml").unwrap();
    outer.write_all(xml.as_bytes()).unwrap();
    outer.flush().unwrap();

    let program = analyze_launch_file(outer.path()).unwrap();
    assert_eq!(program.body.len(), 2);

    // First action: Include with YAML body containing DeclareArgument (not OpaqueFunction)
    match &program.body[0].kind {
        ActionKind::Include { body, .. } => {
            let body = body.as_ref().expect("Include body should be set");
            assert_eq!(body.body.len(), 1);
            match &body.body[0].kind {
                ActionKind::DeclareArgument { name, default, .. } => {
                    assert_eq!(name, "velocity_smoother_type");
                    assert_eq!(
                        default.as_ref().unwrap().as_literal(),
                        Some("JointTrajectoryController")
                    );
                }
                other => panic!("Expected DeclareArgument in YAML body, got {:?}", other),
            }
        }
        other => panic!("Expected Include, got {:?}", other),
    }

    // Second action: SpawnNode
    assert!(matches!(
        &program.body[1].kind,
        ActionKind::SpawnNode { .. }
    ));
}

#[test]
fn test_ir_yaml_arg_no_default() {
    let mut yaml_file = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    yaml_file
        .write_all(b"launch:\n  - arg:\n      name: required_arg\n")
        .unwrap();
    yaml_file.flush().unwrap();

    let program = analyze_launch_file(yaml_file.path()).unwrap();
    assert_eq!(program.body.len(), 1);

    match &program.body[0].kind {
        ActionKind::DeclareArgument { name, default, .. } => {
            assert_eq!(name, "required_arg");
            assert!(default.is_none());
        }
        other => panic!("Expected DeclareArgument, got {:?}", other),
    }
}
