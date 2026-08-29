//! `$(eval …)` is Python evaluation BY SPECIFICATION, so these
//! exercise the parser and an interpreter together — which is this
//! crate, not the parser alone (nano-ros issue 0897 W2b).
//!
//! They lived as `#[cfg(test)]` modules inside
//! `play_launch_parser::substitution::{eval,types}` while that crate
//! linked pyo3. It no longer does, so a test asserting a `$(eval)`
//! RESULT cannot live there: core can no longer produce one. Moving
//! them is not a downgrade — it puts each test in the crate that owns
//! the capability it tests.

use play_launch_parser::substitution::{context::LaunchContext, eval::*, types::*};

/// Every test here needs the backend, and registration is the caller's
/// job now (that is the whole point of the split), so each one starts
/// here rather than relying on a global constructor.
fn backend() {
    play_launch_parser_pyexec::register();
}

#[cfg(test)]
mod escape_tests {
    use super::*;

    /// The form that broke the golf cart's main launch file. The BOOLEAN VALUES
    /// were measured against real `ros2 launch` first, both branches of each
    /// comparison, so these pin conformance rather than our preference. The
    /// lowercase spelling is ours: `$(eval)` results feeding `if=`/`unless=`
    /// are normalized to "true"/"false" (see CLAUDE.md).
    #[test]
    fn escaped_quotes_inside_a_quoted_template_are_content() {
        backend();
        assert_eq!(
            evaluate_expression(r"'\'ndt\' == \'aruco\''").unwrap(),
            "false"
        );
        assert_eq!(
            evaluate_expression(r"'\'aruco\' == \'aruco\''").unwrap(),
            "true"
        );
        // The `not in [...]` variant from the same file.
        assert_eq!(
            evaluate_expression(r"'\'ndt\' not in [\'cuda_ndt\', \'aruco\']'").unwrap(),
            "true"
        );
        assert_eq!(
            evaluate_expression(r"'\'aruco\' not in [\'cuda_ndt\', \'aruco\']'").unwrap(),
            "false"
        );
    }

    /// The `&quot;`-wrapped form used elsewhere in the same repository. It
    /// worked before and must keep working.
    #[test]
    fn double_quoted_template_still_unwraps() {
        backend();
        assert_eq!(
            evaluate_expression("\"'ndt' == 'cuda_ndt'\"").unwrap(),
            "false"
        );
        assert_eq!(
            evaluate_expression("\"'cuda_ndt' == 'cuda_ndt'\"").unwrap(),
            "true"
        );
    }

    /// Outer quotes that are NOT delimiters: a Python expression that merely
    /// begins and ends with a string literal. Stripping here would corrupt it.
    #[test]
    fn python_string_literals_are_not_treated_as_delimiters() {
        backend();
        assert_eq!(evaluate_expression("'foo' + 'bar'").unwrap(), "foobar");
        assert_eq!(evaluate_expression("'a' == 'a'").unwrap(), "true");
    }

    #[test]
    fn empty_and_degenerate_inputs_do_not_panic() {
        backend();
        // `''` must not be stripped to nothing — eval('') is a SyntaxError.
        assert_eq!(evaluate_expression("''").unwrap(), "");
        assert!(evaluate_expression("").is_err());
    }

    #[test]
    fn test_text_substitution() {
        backend();
        let sub = Substitution::Text("hello".to_string());
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "hello");
    }

    #[test]
    fn test_launch_configuration() {
        backend();
        let sub = Substitution::LaunchConfiguration(vec![Substitution::Text("my_var".to_string())]);
        let mut context = LaunchContext::new();
        context.set_configuration("my_var".to_string(), "value123".to_string());
        assert_eq!(sub.resolve(&context).unwrap(), "value123");
    }

    #[test]
    fn test_undefined_variable() {
        backend();
        let sub =
            Substitution::LaunchConfiguration(vec![Substitution::Text("undefined".to_string())]);
        let context = LaunchContext::new();
        assert!(sub.resolve(&context).is_err());
    }

    #[test]
    fn test_env_var() {
        backend();
        unsafe { std::env::set_var("TEST_VAR", "test_value") };
        let sub = Substitution::EnvironmentVariable {
            name: vec![Substitution::Text("TEST_VAR".to_string())],
            default: None,
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "test_value");
    }

    #[test]
    fn test_env_var_with_default() {
        backend();
        let sub = Substitution::EnvironmentVariable {
            name: vec![Substitution::Text("NONEXISTENT_VAR".to_string())],
            default: Some(vec![Substitution::Text("default_value".to_string())]),
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "default_value");
    }

    #[test]
    fn test_resolve_multiple() {
        backend();
        let subs = vec![
            Substitution::Text("Hello ".to_string()),
            Substitution::LaunchConfiguration(vec![Substitution::Text("name".to_string())]),
            Substitution::Text("!".to_string()),
        ];
        let mut context = LaunchContext::new();
        context.set_configuration("name".to_string(), "World".to_string());
        assert_eq!(
            resolve_substitutions(&subs, &context).unwrap(),
            "Hello World!"
        );
    }

    #[test]
    fn test_dirname_substitution() {
        backend();
        use std::path::PathBuf;

        let sub = Substitution::Dirname;
        let mut context = LaunchContext::new();
        context.set_current_file(PathBuf::from("/home/user/launch/test.launch.xml"));

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "/home/user/launch");
    }

    #[test]
    fn test_filename_substitution() {
        backend();
        use std::path::PathBuf;

        let sub = Substitution::Filename;
        let mut context = LaunchContext::new();
        context.set_current_file(PathBuf::from("/home/user/launch/test.launch.xml"));

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "test.launch.xml");
    }

    #[test]
    fn test_dirname_no_file_set() {
        backend();
        let sub = Substitution::Dirname;
        let context = LaunchContext::new();
        assert!(sub.resolve(&context).is_err());
    }

    #[test]
    fn test_anon_substitution() {
        backend();
        let sub = Substitution::Anon(vec![Substitution::Text("my_node".to_string())]);
        let context = LaunchContext::new();

        let result = sub.resolve(&context).unwrap();
        // Should start with the name and have a timestamp and random suffix
        assert!(result.starts_with("my_node_"));
        // Should have the format name_timestamp_random
        let parts: Vec<&str> = result.split('_').collect();
        assert!(parts.len() >= 3);
    }

    #[test]
    fn test_anon_uniqueness() {
        backend();
        let sub1 = Substitution::Anon(vec![Substitution::Text("node".to_string())]);
        let sub2 = Substitution::Anon(vec![Substitution::Text("node".to_string())]);
        let context = LaunchContext::new();

        let result1 = sub1.resolve(&context).unwrap();
        let result2 = sub2.resolve(&context).unwrap();

        // Should generate different names
        assert_ne!(result1, result2);
    }

    #[test]
    fn test_optenv_with_existing_var() {
        backend();
        unsafe { std::env::set_var("TEST_OPTENV_VAR", "test_value") };
        let sub = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("TEST_OPTENV_VAR".to_string())],
            default: None,
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "test_value");
    }

    #[test]
    fn test_optenv_with_missing_var_no_default() {
        backend();
        // Make sure the variable doesn't exist
        unsafe { std::env::remove_var("NONEXISTENT_OPTENV_VAR") };
        let sub = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("NONEXISTENT_OPTENV_VAR".to_string())],
            default: None,
        };
        let context = LaunchContext::new();
        // Should return empty string, not error
        assert_eq!(sub.resolve(&context).unwrap(), "");
    }

    #[test]
    fn test_optenv_with_missing_var_with_default() {
        backend();
        unsafe { std::env::remove_var("NONEXISTENT_OPTENV_VAR2") };
        let sub = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("NONEXISTENT_OPTENV_VAR2".to_string())],
            default: Some(vec![Substitution::Text("default_value".to_string())]),
        };
        let context = LaunchContext::new();
        assert_eq!(sub.resolve(&context).unwrap(), "default_value");
    }

    #[test]
    fn test_optenv_vs_env_behavior() {
        backend();
        unsafe { std::env::remove_var("MISSING_VAR_TEST") };

        // optenv should not error
        let optenv = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("MISSING_VAR_TEST".to_string())],
            default: None,
        };
        let context = LaunchContext::new();
        assert!(optenv.resolve(&context).is_ok());

        // env should error without default
        let env = Substitution::EnvironmentVariable {
            name: vec![Substitution::Text("MISSING_VAR_TEST".to_string())],
            default: None,
        };
        assert!(env.resolve(&context).is_err());
    }

    #[test]
    fn test_command_simple() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo hello".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "hello");
    }

    #[test]
    fn test_command_with_output_trimming() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo '  spaces  '".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        // Output should be trimmed
        assert_eq!(result, "spaces");
    }

    #[test]
    fn test_command_with_newlines() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("printf 'line1\\nline2\\n'".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        // Trailing newline should be trimmed
        assert_eq!(result, "line1\nline2");
    }

    #[test]
    fn test_command_failed() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("exit 1".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        assert!(result.is_err());
    }

    #[test]
    fn test_command_with_args() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo foo bar".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "foo bar");
    }

    #[test]
    fn test_command_pwd() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("pwd".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        // Should return some directory path (non-empty)
        assert!(!result.is_empty());
        assert!(result.starts_with('/'));
    }

    #[test]
    fn test_command_env_access() {
        backend();
        block_command_substitution(false);
        unsafe { std::env::set_var("TEST_CMD_VAR", "test_value") };
        // Direct exec (no shell) passes $VAR literally, matching ROS 2's
        // subprocess.run(shlex.split(cmd)) which also doesn't expand shell vars.
        // Use printenv to read the env var without shell expansion.
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("printenv TEST_CMD_VAR".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "test_value");
    }

    // Nested substitution resolution tests
    #[test]
    fn test_resolve_nested_var_in_var() {
        backend();
        // $(var $(var name)_config) where name=robot, robot_config=my_robot.yaml
        let sub = Substitution::LaunchConfiguration(vec![
            Substitution::LaunchConfiguration(vec![Substitution::Text("name".to_string())]),
            Substitution::Text("_config".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("name".to_string(), "robot".to_string());
        context.set_configuration("robot_config".to_string(), "my_robot.yaml".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "my_robot.yaml");
    }

    #[test]
    fn test_resolve_nested_env_in_var() {
        backend();
        // $(var $(env ROBOT_NAME)_config) where ROBOT_NAME=turtlebot, turtlebot_config=tb3.yaml
        unsafe { std::env::set_var("TEST_ROBOT_NAME", "turtlebot") };

        let sub = Substitution::LaunchConfiguration(vec![
            Substitution::EnvironmentVariable {
                name: vec![Substitution::Text("TEST_ROBOT_NAME".to_string())],
                default: None,
            },
            Substitution::Text("_config".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("turtlebot_config".to_string(), "tb3.yaml".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "tb3.yaml");
    }

    #[test]
    fn test_resolve_nested_var_in_env_default() {
        backend();
        // $(env MISSING_VAR $(var my_default)) where my_default=fallback_value
        let sub = Substitution::EnvironmentVariable {
            name: vec![Substitution::Text("NONEXISTENT_NESTED_VAR".to_string())],
            default: Some(vec![Substitution::LaunchConfiguration(vec![
                Substitution::Text("my_default".to_string()),
            ])]),
        };

        let mut context = LaunchContext::new();
        context.set_configuration("my_default".to_string(), "fallback_value".to_string());

        unsafe { std::env::remove_var("NONEXISTENT_NESTED_VAR") };
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "fallback_value");
    }

    #[test]
    fn test_resolve_nested_var_in_command() {
        backend();
        block_command_substitution(false);
        // $(command echo $(var greeting)) where greeting=hello
        let sub = Substitution::Command {
            cmd: vec![
                Substitution::Text("echo ".to_string()),
                Substitution::LaunchConfiguration(vec![Substitution::Text("greeting".to_string())]),
            ],
            error_mode: CommandErrorMode::Strict,
        };

        let mut context = LaunchContext::new();
        context.set_configuration("greeting".to_string(), "hello".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "hello");
    }

    #[test]
    fn test_resolve_triple_nested() {
        backend();
        // $(var $(env $(var prefix)_NAME)_suffix)
        // prefix=ROBOT, ROBOT_NAME=turtlebot, turtlebot_suffix=final_value
        unsafe { std::env::set_var("TEST_ROBOT_NAME_TRIPLE", "turtlebot") };

        let sub = Substitution::LaunchConfiguration(vec![
            Substitution::EnvironmentVariable {
                name: vec![
                    Substitution::LaunchConfiguration(vec![Substitution::Text(
                        "prefix".to_string(),
                    )]),
                    Substitution::Text("_NAME_TRIPLE".to_string()),
                ],
                default: None,
            },
            Substitution::Text("_suffix".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("prefix".to_string(), "TEST_ROBOT".to_string());
        context.set_configuration("turtlebot_suffix".to_string(), "final_value".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "final_value");
    }

    #[test]
    fn test_resolve_nested_in_optenv() {
        backend();
        // $(optenv MISSING $(var backup)) where backup=default_val
        let sub = Substitution::OptionalEnvironmentVariable {
            name: vec![Substitution::Text("MISSING_OPTENV_NESTED".to_string())],
            default: Some(vec![Substitution::LaunchConfiguration(vec![
                Substitution::Text("backup".to_string()),
            ])]),
        };

        let mut context = LaunchContext::new();
        context.set_configuration("backup".to_string(), "default_val".to_string());

        unsafe { std::env::remove_var("MISSING_OPTENV_NESTED") };
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "default_val");
    }

    #[test]
    fn test_resolve_complex_nested_string() {
        backend();
        // "prefix_$(var $(env TYPE)_name)_suffix" where TYPE=robot, robot_name=turtlebot
        unsafe { std::env::set_var("TEST_TYPE_NESTED", "robot") };

        let subs = vec![
            Substitution::Text("prefix_".to_string()),
            Substitution::LaunchConfiguration(vec![
                Substitution::EnvironmentVariable {
                    name: vec![Substitution::Text("TEST_TYPE_NESTED".to_string())],
                    default: None,
                },
                Substitution::Text("_name".to_string()),
            ]),
            Substitution::Text("_suffix".to_string()),
        ];

        let mut context = LaunchContext::new();
        context.set_configuration("robot_name".to_string(), "turtlebot".to_string());

        let result = resolve_substitutions(&subs, &context).unwrap();
        assert_eq!(result, "prefix_turtlebot_suffix");
    }

    #[test]
    fn test_resolve_nested_anon() {
        backend();
        // $(anon $(var base_name)) where base_name=my_node
        let sub = Substitution::Anon(vec![Substitution::LaunchConfiguration(vec![
            Substitution::Text("base_name".to_string()),
        ])]);

        let mut context = LaunchContext::new();
        context.set_configuration("base_name".to_string(), "my_node".to_string());

        let result = sub.resolve(&context).unwrap();
        assert!(result.starts_with("my_node_"));
        let parts: Vec<&str> = result.split('_').collect();
        assert!(parts.len() >= 3);
    }

    #[test]
    fn test_resolve_nested_command_with_env() {
        backend();
        block_command_substitution(false);
        // $(command echo $(env USER))
        unsafe { std::env::set_var("TEST_USER_NESTED", "testuser") };

        let sub = Substitution::Command {
            cmd: vec![
                Substitution::Text("echo ".to_string()),
                Substitution::EnvironmentVariable {
                    name: vec![Substitution::Text("TEST_USER_NESTED".to_string())],
                    default: None,
                },
            ],
            error_mode: CommandErrorMode::Strict,
        };

        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "testuser");
    }

    // Eval tests
    #[test]
    fn test_eval_simple_addition() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("1 + 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "3");
    }

    #[test]
    fn test_eval_multiplication() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("4 * 5".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "20");
    }

    #[test]
    fn test_eval_division() {
        backend();
        // Python's / always returns float (10/2 → 5.0)
        let sub = Substitution::Eval(vec![Substitution::Text("10 / 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "5.0");
    }

    #[test]
    fn test_eval_modulo() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("10 % 3".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "1");
    }

    #[test]
    fn test_eval_subtraction() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("10 - 3".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "7");
    }

    #[test]
    fn test_eval_with_parentheses() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("(3 + 4) * 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "14");
    }

    #[test]
    fn test_eval_order_of_operations() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("2 + 3 * 4".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "14");
    }

    #[test]
    fn test_eval_negative_numbers() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("-5 + 10".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "5");
    }

    #[test]
    fn test_eval_float_result() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("5 / 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "2.5");
    }

    #[test]
    fn test_eval_with_var_substitution() {
        backend();
        // $(eval $(var x) + 5) where x=10
        let sub = Substitution::Eval(vec![
            Substitution::LaunchConfiguration(vec![Substitution::Text("x".to_string())]),
            Substitution::Text(" + 5".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("x".to_string(), "10".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "15");
    }

    #[test]
    fn test_eval_complex_with_vars() {
        backend();
        // $(eval ($(var a) + $(var b)) * 2) where a=3, b=4
        let sub = Substitution::Eval(vec![
            Substitution::Text("(".to_string()),
            Substitution::LaunchConfiguration(vec![Substitution::Text("a".to_string())]),
            Substitution::Text(" + ".to_string()),
            Substitution::LaunchConfiguration(vec![Substitution::Text("b".to_string())]),
            Substitution::Text(") * 2".to_string()),
        ]);

        let mut context = LaunchContext::new();
        context.set_configuration("a".to_string(), "3".to_string());
        context.set_configuration("b".to_string(), "4".to_string());

        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "14");
    }

    #[test]
    fn test_eval_division_by_zero() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("1 / 0".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        assert!(result.is_err());
    }

    #[test]
    fn test_eval_invalid_expression() {
        backend();
        // "1 + + 2" is valid Python (unary +), so with the Python fallback it evaluates to 3
        let sub = Substitution::Eval(vec![Substitution::Text("1 + + 2".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "3");

        // Truly invalid expression
        let sub = Substitution::Eval(vec![Substitution::Text("@@@".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        assert!(result.is_err());
    }

    // String comparison tests
    #[test]
    fn test_eval_string_equals_true() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text(
            "'elastic_band' == 'elastic_band'".to_string(),
        )]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "true");
    }

    #[test]
    fn test_eval_string_equals_false() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("'foo' == 'bar'".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "false");
    }

    #[test]
    fn test_eval_string_not_equals_true() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("'foo' != 'bar'".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "true");
    }

    #[test]
    fn test_eval_string_not_equals_false() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("'foo' != 'foo'".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "false");
    }

    #[test]
    fn test_eval_string_with_outer_quotes() {
        backend();
        // Expression wrapped in double quotes (from XML attribute)
        let sub = Substitution::Eval(vec![Substitution::Text(
            "\"'elastic_band' == 'elastic_band'\"".to_string(),
        )]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "true");
    }

    #[test]
    fn test_eval_string_double_quotes() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("\"foo\" == \"foo\"".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "true");
    }

    // Command error mode execution tests
    #[test]
    fn test_command_strict_mode_fails_on_error() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("exit 1".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        // Should fail with Strict mode
        assert!(result.is_err());
    }

    #[test]
    fn test_command_warn_mode_continues_on_error() {
        backend();
        block_command_substitution(false);
        // Use bash -c to test shell constructs that produce stdout then fail.
        // Direct exec doesn't support && — we explicitly invoke bash here.
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text(
                "bash -c 'echo output && exit 1'".to_string(),
            )],
            error_mode: CommandErrorMode::Warn,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        // Should succeed with Warn mode even though command failed
        assert!(result.is_ok());
        // Should return stdout (output)
        assert_eq!(result.unwrap(), "output");
    }

    #[test]
    fn test_command_ignore_mode_continues_on_error() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text(
                "bash -c 'echo output && exit 1'".to_string(),
            )],
            error_mode: CommandErrorMode::Ignore,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        // Should succeed with Ignore mode even though command failed
        assert!(result.is_ok());
        // Should return stdout (output)
        assert_eq!(result.unwrap(), "output");
    }

    #[test]
    fn test_command_all_modes_succeed_on_success() {
        backend();
        block_command_substitution(false);
        // All modes should succeed when command succeeds
        for error_mode in [
            CommandErrorMode::Strict,
            CommandErrorMode::Warn,
            CommandErrorMode::Ignore,
        ] {
            let sub = Substitution::Command {
                cmd: vec![Substitution::Text("echo success".to_string())],
                error_mode,
            };
            let context = LaunchContext::new();
            let result = sub.resolve(&context);
            assert!(result.is_ok());
            assert_eq!(result.unwrap(), "success");
        }
    }

    // shlex splitting edge cases — these test execute_command directly
    #[test]
    fn test_command_shlex_unquoted_with_args() {
        backend();
        // After parser quote-stripping, command arrives unquoted at execute_command.
        // shlex::split splits by whitespace, producing ["echo", "shlex_result"].
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo shlex_result".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "shlex_result");
    }

    #[test]
    fn test_command_shlex_args_with_spaces() {
        backend();
        // Arguments containing spaces must be quoted
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo 'hello world'".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "hello world");
    }

    #[test]
    fn test_command_shlex_mixed_quotes() {
        backend();
        // Double quotes inside single-quoted region are literal
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text(r#"echo 'say "hi"'"#.to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, r#"say "hi""#);
    }

    #[test]
    fn test_command_shlex_backslash_escape() {
        backend();
        // Backslash outside quotes escapes the next character
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text(r"echo hello\ world".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "hello world");
    }

    #[test]
    fn test_command_shlex_unclosed_quote_error() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("echo 'unclosed".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        assert!(result.is_err());
        assert!(
            result.unwrap_err().to_string().contains("unclosed quote"),
            "Error should mention unclosed quote"
        );
    }

    #[test]
    fn test_command_shlex_empty_string_error() {
        backend();
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("   ".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context);
        assert!(result.is_err());
        assert!(
            result.unwrap_err().to_string().contains("Empty command"),
            "Error should mention empty command"
        );
    }

    #[test]
    fn test_command_shlex_multiple_args() {
        backend();
        // Multiple arguments with various quoting styles
        block_command_substitution(false);
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text("printf '%s-%s' foo bar".to_string())],
            error_mode: CommandErrorMode::Strict,
        };
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "foo-bar");
    }

    // String concatenation tests
    #[test]
    fn test_eval_string_concat_simple() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text("'foo' + 'bar'".to_string())]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "foobar");
    }

    #[test]
    fn test_eval_string_concat_multi() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text(
            "'[' + 'Module1, ' + 'Module2, ' + ']'".to_string(),
        )]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "[Module1, Module2, ]");
    }

    #[test]
    fn test_eval_string_concat_with_var() {
        backend();
        // $(eval '$(var prefix)' + '_suffix') — var already resolved by the time eval sees it
        let sub = Substitution::Eval(vec![
            Substitution::Text("'".to_string()),
            Substitution::LaunchConfiguration(vec![Substitution::Text("prefix".to_string())]),
            Substitution::Text("' + '_suffix'".to_string()),
        ]);
        let mut context = LaunchContext::new();
        context.set_configuration("prefix".to_string(), "hello".to_string());
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "hello_suffix");
    }

    #[test]
    fn test_eval_string_concat_empty_quotes() {
        backend();
        let sub = Substitution::Eval(vec![Substitution::Text(
            "\"\" + 'content' + \"\"".to_string(),
        )]);
        let context = LaunchContext::new();
        let result = sub.resolve(&context).unwrap();
        assert_eq!(result, "content");
    }
}
