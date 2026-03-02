//! Substitution parser

use crate::{
    error::{ParseError, Result},
    substitution::types::{CommandErrorMode, Substitution},
};
use lru::LruCache;
use std::{cell::RefCell, num::NonZeroUsize};

// Thread-local LRU cache for substitution parsing (Phase 7.4.1)
//
// Caches parsed AST structures, NOT resolved values.
// This is safe because parsing is context-independent - same input always produces same AST.
// Resolution happens separately with the current context.
//
// Expected hit rate: >80% for launch files with repeated patterns (typical: ~200-500 unique strings)
const SUBSTITUTION_CACHE_SIZE: usize = 1024;

thread_local! {
    static PARSE_CACHE: RefCell<LruCache<String, Vec<Substitution>>> =
        RefCell::new(LruCache::new(NonZeroUsize::new(SUBSTITUTION_CACHE_SIZE).unwrap()));
}

/// Parse substitution string like "$(var x)" or "text $(env Y) more"
/// Supports nested substitutions like "$(var $(env NAME)_config)"
///
/// Uses thread-local LRU cache to avoid re-parsing identical strings.
/// Safe because parsing is context-independent - we cache AST structure, not resolved values.
pub fn parse_substitutions(input: &str) -> Result<Vec<Substitution>> {
    // Fast path: Check cache (thread-local, no locking needed)
    PARSE_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();

        // Check if we have a cached result
        if let Some(cached) = cache.get(input) {
            log::trace!("Substitution parse cache hit: {}", input);
            return Ok(cached.clone());
        }

        // Slow path: Parse and cache
        log::trace!("Substitution parse cache miss: {}", input);
        drop(cache); // Release borrow before recursive call

        let result = parse_substitutions_recursive(input)?;

        // Cache the result
        PARSE_CACHE.with(|cache| {
            let mut cache = cache.borrow_mut();
            cache.put(input.to_string(), result.clone());
        });

        Ok(result)
    })
}

/// Internal recursive parser that handles nested substitutions
fn parse_substitutions_recursive(input: &str) -> Result<Vec<Substitution>> {
    let mut result = Vec::new();
    let mut chars = input.char_indices().peekable();
    let mut last_pos = 0;

    while let Some((i, ch)) = chars.next() {
        if ch == '$'
            && let Some((_, '(')) = chars.peek()
        {
            // Found start of substitution
            // Add any text before this substitution
            if i > last_pos {
                let text = &input[last_pos..i];
                if !text.is_empty() {
                    result.push(Substitution::Text(text.to_string()));
                }
            }

            // Skip the '('
            chars.next();

            // Find matching ')' by counting parentheses
            let sub_start = i + 2; // Position after "$("
            let mut depth = 1;
            let mut sub_end = sub_start;

            for (pos, c) in chars.by_ref() {
                if c == '(' {
                    depth += 1;
                } else if c == ')' {
                    depth -= 1;
                    if depth == 0 {
                        sub_end = pos;
                        break;
                    }
                }
            }

            if depth != 0 {
                return Err(ParseError::InvalidSubstitution(
                    "Unmatched parentheses in substitution".to_string(),
                ));
            }

            // Extract the content inside $()
            let content = &input[sub_start..sub_end];

            // Parse the substitution (which may contain nested substitutions)
            let substitution = parse_substitution_content(content)?;
            result.push(substitution);

            last_pos = sub_end + 1;
        }
    }

    // Add any remaining text
    if last_pos < input.len() {
        let text = &input[last_pos..];
        if !text.is_empty() {
            result.push(Substitution::Text(text.to_string()));
        }
    }

    // If no substitutions found, treat entire input as text
    if result.is_empty() {
        result.push(Substitution::Text(input.to_string()));
    }

    Ok(result)
}

/// Extract the first quoted argument from a string like "'cmd' 'warn'"
/// Returns the content of the first quoted string without the quotes.
/// If there are TWO quoted strings, returns only the first one.
/// If there is only ONE quoted string or unquoted text, returns everything.
fn extract_first_quoted_arg(input: &str) -> &str {
    let trimmed = input.trim();

    // Check for single or double quotes at the start
    if let Some(quote) = trimmed.chars().next()
        && (quote == '\'' || quote == '"')
    {
        // Find the matching closing quote
        if let Some(end_idx) = trimmed[1..].find(quote) {
            let first_quoted_content = &trimmed[1..end_idx + 1];

            // Check if there's a second quoted string after this one
            let after_first = &trimmed[end_idx + 2..].trim_start();
            if !after_first.is_empty()
                && (after_first.starts_with('\'') || after_first.starts_with('"'))
            {
                // There's a second quoted argument, so only return the first
                return first_quoted_content;
            }
            // Otherwise return the full input (handles single quoted arg)
        }
    }

    // Return the full string (handles unquoted args or single quoted args)
    trimmed
}

/// Extract the second quoted argument from a string like "'cmd' 'warn'"
/// Returns the content of the second quoted string without the quotes.
/// Returns None if there is no second argument.
fn extract_second_quoted_arg(input: &str) -> Option<&str> {
    let trimmed = input.trim();

    // Check for single or double quotes at the start
    if let Some(quote) = trimmed.chars().next()
        && (quote == '\'' || quote == '"')
    {
        // Find the matching closing quote for the first arg
        if let Some(end_idx) = trimmed[1..].find(quote) {
            // Look for the second quoted string after the first
            let after_first = trimmed[end_idx + 2..].trim_start();

            if let Some(quote2) = after_first.chars().next()
                && (quote2 == '\'' || quote2 == '"')
            {
                // Find the matching closing quote for the second arg
                if let Some(end_idx2) = after_first[1..].find(quote2) {
                    return Some(&after_first[1..end_idx2 + 1]);
                }
            }
        }
    }

    None
}

/// Parse the content inside a substitution $(...)
/// This handles recursion: "var $(env X)" should parse inner $(env X) first
fn parse_substitution_content(content: &str) -> Result<Substitution> {
    // Find the substitution type (first word)
    let trimmed = content.trim();
    let parts: Vec<&str> = trimmed.splitn(2, ' ').collect();

    if parts.is_empty() {
        return Err(ParseError::InvalidSubstitution(
            "Empty substitution".to_string(),
        ));
    }

    let sub_type = parts[0];
    let args = if parts.len() > 1 {
        Some(parts[1])
    } else {
        None
    };

    parse_single_substitution(sub_type, args)
}

/// Split arguments respecting nested substitutions
/// For "env X default" or "optenv X default", split into (name, default)
/// But if X contains nested substitutions with spaces, don't split inside them
fn split_env_args(args: &str) -> (&str, Option<&str>) {
    let args = args.trim();

    // If args starts with $(, we need to find the matching ) to know where the name ends
    if args.starts_with("$(") {
        let mut depth = 0;
        let mut name_end = args.len();

        for (i, ch) in args.char_indices() {
            if ch == '(' {
                depth += 1;
            } else if ch == ')' {
                depth -= 1;
                if depth == 0 {
                    // Found the end of the nested substitution
                    // Check if there's more content after this
                    name_end = i + 1;
                    // Skip any non-space chars that are part of the name (like "_suffix")
                    while name_end < args.len() {
                        let ch = args.chars().nth(name_end).unwrap();
                        if ch.is_whitespace() {
                            break;
                        }
                        name_end += 1;
                    }
                    break;
                }
            }
        }

        // Everything up to name_end is the name
        let name = args[..name_end].trim();
        // Everything after (if any) is the default
        let default = if name_end < args.len() {
            Some(args[name_end..].trim())
        } else {
            None
        };

        (name, default)
    } else {
        // No nested substitution, just split on first space
        let parts: Vec<&str> = args.splitn(2, ' ').collect();
        let name = parts[0].trim();
        let default = parts.get(1).map(|s| s.trim());
        (name, default)
    }
}

fn parse_single_substitution(sub_type: &str, args: Option<&str>) -> Result<Substitution> {
    match sub_type {
        "var" => {
            let args_str = args
                .ok_or_else(|| {
                    ParseError::InvalidSubstitution("var requires an argument".to_string())
                })?
                .trim();
            // Parse the argument which may contain nested substitutions
            let name_subs = parse_substitutions_recursive(args_str)?;
            Ok(Substitution::LaunchConfiguration(name_subs))
        }
        "env" => {
            let args_str = args.ok_or_else(|| {
                ParseError::InvalidSubstitution("env requires an argument".to_string())
            })?;
            let (name_str, default_str) = split_env_args(args_str);
            let name_subs = parse_substitutions_recursive(name_str)?;
            let default = if let Some(def) = default_str {
                Some(parse_substitutions_recursive(def)?)
            } else {
                None
            };
            Ok(Substitution::EnvironmentVariable {
                name: name_subs,
                default,
            })
        }
        "optenv" => {
            let args_str = args.ok_or_else(|| {
                ParseError::InvalidSubstitution("optenv requires an argument".to_string())
            })?;
            let (name_str, default_str) = split_env_args(args_str);
            let name_subs = parse_substitutions_recursive(name_str)?;
            let default = if let Some(def) = default_str {
                Some(parse_substitutions_recursive(def)?)
            } else {
                None
            };
            Ok(Substitution::OptionalEnvironmentVariable {
                name: name_subs,
                default,
            })
        }
        "command" => {
            let cmd_str = args.ok_or_else(|| {
                ParseError::InvalidSubstitution("command requires an argument".to_string())
            })?;

            // ROS 2 command substitution supports two arguments:
            // $(command 'cmd' 'error_mode')
            // where error_mode is 'warn', 'ignore', or 'strict' (default: strict)
            let first_arg = extract_first_quoted_arg(cmd_str);
            let cmd_subs = parse_substitutions_recursive(first_arg)?;

            // Extract optional second argument for error mode
            let error_mode = if let Some(mode_str) = extract_second_quoted_arg(cmd_str) {
                match mode_str {
                    "warn" => CommandErrorMode::Warn,
                    "ignore" => CommandErrorMode::Ignore,
                    "strict" => CommandErrorMode::Strict,
                    _ => CommandErrorMode::Strict, // Default to strict for unknown modes
                }
            } else {
                CommandErrorMode::Strict // Default when no second argument
            };

            Ok(Substitution::Command {
                cmd: cmd_subs,
                error_mode,
            })
        }
        "find-pkg-share" => {
            let package_str = args
                .ok_or_else(|| {
                    ParseError::InvalidSubstitution(
                        "find-pkg-share requires an argument".to_string(),
                    )
                })?
                .trim();
            let package_subs = parse_substitutions_recursive(package_str)?;
            Ok(Substitution::FindPackageShare(package_subs))
        }
        "dirname" => Ok(Substitution::Dirname),
        "filename" => Ok(Substitution::Filename),
        "anon" => {
            let name_str = args
                .ok_or_else(|| {
                    ParseError::InvalidSubstitution("anon requires a name argument".to_string())
                })?
                .trim();
            let name_subs = parse_substitutions_recursive(name_str)?;
            Ok(Substitution::Anon(name_subs))
        }
        "eval" => {
            let expr_str = args
                .ok_or_else(|| {
                    ParseError::InvalidSubstitution(
                        "eval requires an expression argument".to_string(),
                    )
                })?
                .trim();
            let expr_subs = parse_substitutions_recursive(expr_str)?;
            Ok(Substitution::Eval(expr_subs))
        }
        _ => Err(ParseError::InvalidSubstitution(format!(
            "Unknown substitution type: {}",
            sub_type
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_plain_text() {
        let subs = parse_substitutions("hello world").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0], Substitution::Text("hello world".to_string()));
    }

    #[test]
    fn test_parse_var_substitution() {
        let subs = parse_substitutions("$(var my_var)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::LaunchConfiguration(vec![Substitution::Text("my_var".to_string())])
        );
    }

    #[test]
    fn test_parse_env_substitution() {
        let subs = parse_substitutions("$(env HOME)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::EnvironmentVariable {
                name: vec![Substitution::Text("HOME".to_string())],
                default: None
            }
        );
    }

    #[test]
    fn test_parse_env_with_default() {
        let subs = parse_substitutions("$(env MY_VAR default_value)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::EnvironmentVariable {
                name: vec![Substitution::Text("MY_VAR".to_string())],
                default: Some(vec![Substitution::Text("default_value".to_string())])
            }
        );
    }

    #[test]
    fn test_parse_mixed() {
        let subs = parse_substitutions("prefix $(var x) middle $(env Y) suffix").unwrap();
        assert_eq!(subs.len(), 5);
        assert_eq!(subs[0], Substitution::Text("prefix ".to_string()));
        assert_eq!(
            subs[1],
            Substitution::LaunchConfiguration(vec![Substitution::Text("x".to_string())])
        );
        assert_eq!(subs[2], Substitution::Text(" middle ".to_string()));
        assert_eq!(
            subs[3],
            Substitution::EnvironmentVariable {
                name: vec![Substitution::Text("Y".to_string())],
                default: None
            }
        );
        assert_eq!(subs[4], Substitution::Text(" suffix".to_string()));
    }

    #[test]
    fn test_parse_consecutive_substitutions() {
        let subs = parse_substitutions("$(var a)$(var b)").unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(
            subs[0],
            Substitution::LaunchConfiguration(vec![Substitution::Text("a".to_string())])
        );
        assert_eq!(
            subs[1],
            Substitution::LaunchConfiguration(vec![Substitution::Text("b".to_string())])
        );
    }

    #[test]
    fn test_parse_dirname() {
        let subs = parse_substitutions("$(dirname)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0], Substitution::Dirname);
    }

    #[test]
    fn test_parse_filename() {
        let subs = parse_substitutions("$(filename)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0], Substitution::Filename);
    }

    #[test]
    fn test_parse_dirname_in_path() {
        let subs = parse_substitutions("$(dirname)/config.yaml").unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(subs[0], Substitution::Dirname);
        assert_eq!(subs[1], Substitution::Text("/config.yaml".to_string()));
    }

    #[test]
    fn test_parse_anon() {
        let subs = parse_substitutions("$(anon my_node)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::Anon(vec![Substitution::Text("my_node".to_string())])
        );
    }

    #[test]
    fn test_parse_anon_in_name() {
        let subs = parse_substitutions("node_$(anon suffix)").unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(subs[0], Substitution::Text("node_".to_string()));
        assert_eq!(
            subs[1],
            Substitution::Anon(vec![Substitution::Text("suffix".to_string())])
        );
    }

    #[test]
    fn test_parse_optenv_without_default() {
        let subs = parse_substitutions("$(optenv MY_VAR)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::OptionalEnvironmentVariable {
                name: vec![Substitution::Text("MY_VAR".to_string())],
                default: None
            }
        );
    }

    #[test]
    fn test_parse_optenv_with_default() {
        let subs = parse_substitutions("$(optenv MY_VAR default_value)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::OptionalEnvironmentVariable {
                name: vec![Substitution::Text("MY_VAR".to_string())],
                default: Some(vec![Substitution::Text("default_value".to_string())])
            }
        );
    }

    #[test]
    fn test_parse_optenv_in_string() {
        let subs = parse_substitutions("prefix_$(optenv VAR suffix)_end").unwrap();
        assert_eq!(subs.len(), 3);
        assert_eq!(subs[0], Substitution::Text("prefix_".to_string()));
        assert_eq!(
            subs[1],
            Substitution::OptionalEnvironmentVariable {
                name: vec![Substitution::Text("VAR".to_string())],
                default: Some(vec![Substitution::Text("suffix".to_string())])
            }
        );
        assert_eq!(subs[2], Substitution::Text("_end".to_string()));
    }

    #[test]
    fn test_parse_command() {
        let subs = parse_substitutions("$(command echo hello)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::Command {
                cmd: vec![Substitution::Text("echo hello".to_string())],
                error_mode: CommandErrorMode::Strict,
            }
        );
    }

    #[test]
    fn test_parse_command_with_args() {
        let subs = parse_substitutions("$(command ls -la)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::Command {
                cmd: vec![Substitution::Text("ls -la".to_string())],
                error_mode: CommandErrorMode::Strict,
            }
        );
    }

    #[test]
    fn test_parse_command_in_string() {
        let subs = parse_substitutions("Version: $(command cat /etc/os-release)").unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(subs[0], Substitution::Text("Version: ".to_string()));
        assert_eq!(
            subs[1],
            Substitution::Command {
                cmd: vec![Substitution::Text("cat /etc/os-release".to_string())],
                error_mode: CommandErrorMode::Strict,
            }
        );
    }

    #[test]
    fn test_parse_command_with_pipe() {
        let subs = parse_substitutions("$(command echo test | tr a-z A-Z)").unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(
            subs[0],
            Substitution::Command {
                cmd: vec![Substitution::Text("echo test | tr a-z A-Z".to_string())],
                error_mode: CommandErrorMode::Strict,
            }
        );
    }

    // Nested substitution tests
    #[test]
    fn test_parse_nested_var_in_var() {
        let subs = parse_substitutions("$(var $(var name)_config)").unwrap();
        assert_eq!(subs.len(), 1);

        // Outer should be a LaunchConfiguration
        if let Substitution::LaunchConfiguration(inner_subs) = &subs[0] {
            assert_eq!(inner_subs.len(), 2);
            // First part is the nested $(var name)
            assert!(matches!(
                inner_subs[0],
                Substitution::LaunchConfiguration(_)
            ));
            // Second part is the literal "_config"
            assert_eq!(inner_subs[1], Substitution::Text("_config".to_string()));

            // Check the nested substitution
            if let Substitution::LaunchConfiguration(nested) = &inner_subs[0] {
                assert_eq!(nested.len(), 1);
                assert_eq!(nested[0], Substitution::Text("name".to_string()));
            } else {
                panic!("Expected nested LaunchConfiguration");
            }
        } else {
            panic!("Expected outer LaunchConfiguration");
        }
    }

    #[test]
    fn test_parse_nested_env_in_var() {
        let subs = parse_substitutions("$(var $(env ROBOT_NAME)_config)").unwrap();
        assert_eq!(subs.len(), 1);

        if let Substitution::LaunchConfiguration(inner_subs) = &subs[0] {
            assert_eq!(inner_subs.len(), 2);
            // First part is $(env ROBOT_NAME)
            assert!(matches!(
                inner_subs[0],
                Substitution::EnvironmentVariable { .. }
            ));
            // Second part is "_config"
            assert_eq!(inner_subs[1], Substitution::Text("_config".to_string()));
        } else {
            panic!("Expected LaunchConfiguration");
        }
    }

    #[test]
    fn test_parse_nested_var_in_env_default() {
        let subs = parse_substitutions("$(env MY_VAR $(var default_val))").unwrap();
        assert_eq!(subs.len(), 1);

        if let Substitution::EnvironmentVariable { name, default } = &subs[0] {
            // Name should be simple text
            assert_eq!(name.len(), 1);
            assert_eq!(name[0], Substitution::Text("MY_VAR".to_string()));

            // Default should contain nested substitution
            assert!(default.is_some());
            let default_subs = default.as_ref().unwrap();
            assert_eq!(default_subs.len(), 1);
            assert!(matches!(
                default_subs[0],
                Substitution::LaunchConfiguration(_)
            ));
        } else {
            panic!("Expected EnvironmentVariable");
        }
    }

    #[test]
    fn test_parse_nested_var_in_command() {
        let subs = parse_substitutions("$(command echo $(var value))").unwrap();
        assert_eq!(subs.len(), 1);

        if let Substitution::Command { cmd, error_mode } = &subs[0] {
            assert_eq!(*error_mode, CommandErrorMode::Strict);
            assert_eq!(cmd.len(), 2);
            // First part is "echo "
            assert_eq!(cmd[0], Substitution::Text("echo ".to_string()));
            // Second part is the nested $(var value)
            assert!(matches!(cmd[1], Substitution::LaunchConfiguration(_)));
        } else {
            panic!("Expected Command");
        }
    }

    #[test]
    fn test_parse_nested_optenv_in_string() {
        let subs = parse_substitutions("prefix_$(optenv VAR $(var default))_suffix").unwrap();
        assert_eq!(subs.len(), 3);

        // First: "prefix_"
        assert_eq!(subs[0], Substitution::Text("prefix_".to_string()));

        // Middle: $(optenv VAR $(var default))
        if let Substitution::OptionalEnvironmentVariable { name, default } = &subs[1] {
            assert_eq!(name.len(), 1);
            assert_eq!(name[0], Substitution::Text("VAR".to_string()));

            // Default should have nested var
            assert!(default.is_some());
            let def = default.as_ref().unwrap();
            assert_eq!(def.len(), 1);
            assert!(matches!(def[0], Substitution::LaunchConfiguration(_)));
        } else {
            panic!("Expected OptionalEnvironmentVariable");
        }

        // Last: "_suffix"
        assert_eq!(subs[2], Substitution::Text("_suffix".to_string()));
    }

    #[test]
    fn test_parse_consecutive_nested_substitutions() {
        let subs = parse_substitutions("$(var $(env A))$(var $(env B))").unwrap();
        assert_eq!(subs.len(), 2);

        // Both should be LaunchConfiguration with nested EnvironmentVariable
        for sub in &subs {
            if let Substitution::LaunchConfiguration(inner) = sub {
                assert_eq!(inner.len(), 1);
                assert!(matches!(inner[0], Substitution::EnvironmentVariable { .. }));
            } else {
                panic!("Expected LaunchConfiguration");
            }
        }
    }

    #[test]
    fn test_parse_triple_nested_debug() {
        // Test simpler triple nesting first: $(var $(var $(var x)))
        let subs = parse_substitutions("$(var $(var $(var x)))").unwrap();
        assert_eq!(subs.len(), 1);
    }

    #[test]
    fn test_parse_triple_nested_with_text() {
        // Test with text after inner: $(var $(var $(var x)_y))
        let subs = parse_substitutions("$(var $(var $(var x)_y))").unwrap();
        assert_eq!(subs.len(), 1);
    }

    #[test]
    fn test_parse_triple_nested_with_env() {
        // Test with env in middle: $(var $(env $(var x)_Y))
        let subs = parse_substitutions("$(var $(env $(var x)_Y))").unwrap();
        assert_eq!(subs.len(), 1);
    }

    #[test]
    fn test_parse_triple_nested() {
        // $(var $(env $(var prefix)_NAME)_suffix)
        let subs = parse_substitutions("$(var $(env $(var prefix)_NAME)_suffix)").unwrap();
        assert_eq!(subs.len(), 1);

        // Outermost: LaunchConfiguration
        if let Substitution::LaunchConfiguration(level1) = &subs[0] {
            assert_eq!(level1.len(), 2);

            // First element should be EnvironmentVariable (middle level)
            if let Substitution::EnvironmentVariable { name, .. } = &level1[0] {
                assert_eq!(name.len(), 2);
                // name[0] should be the innermost LaunchConfiguration
                assert!(matches!(name[0], Substitution::LaunchConfiguration(_)));
                // name[1] should be "_NAME"
                assert_eq!(name[1], Substitution::Text("_NAME".to_string()));
            } else {
                panic!("Expected EnvironmentVariable at level 1");
            }

            // level1[1] should be "_suffix"
            assert_eq!(level1[1], Substitution::Text("_suffix".to_string()));
        } else {
            panic!("Expected LaunchConfiguration at top level");
        }
    }

    #[test]
    fn test_parse_nested_anon() {
        let subs = parse_substitutions("$(anon $(var node_name))").unwrap();
        assert_eq!(subs.len(), 1);

        if let Substitution::Anon(inner) = &subs[0] {
            assert_eq!(inner.len(), 1);
            assert!(matches!(inner[0], Substitution::LaunchConfiguration(_)));
        } else {
            panic!("Expected Anon");
        }
    }

    #[test]
    fn test_parse_nested_find_pkg_share() {
        let subs = parse_substitutions("$(find-pkg-share $(var package_name))").unwrap();
        assert_eq!(subs.len(), 1);

        if let Substitution::FindPackageShare(inner) = &subs[0] {
            assert_eq!(inner.len(), 1);
            assert!(matches!(inner[0], Substitution::LaunchConfiguration(_)));
        } else {
            panic!("Expected FindPackageShare");
        }
    }

    // Eval tests
    #[test]
    fn test_parse_eval_simple() {
        let subs = parse_substitutions("$(eval 1 + 2)").unwrap();
        assert_eq!(subs.len(), 1);
        assert!(matches!(subs[0], Substitution::Eval(_)));
    }

    #[test]
    fn test_parse_eval_with_var() {
        let subs = parse_substitutions("$(eval $(var x) + 5)").unwrap();
        assert_eq!(subs.len(), 1);

        if let Substitution::Eval(expr) = &subs[0] {
            // Should have nested var substitution
            assert!(expr.len() > 1);
        } else {
            panic!("Expected Eval");
        }
    }

    #[test]
    fn test_parse_eval_in_string() {
        let subs = parse_substitutions("value is $(eval 10 * 5)").unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(subs[0], Substitution::Text("value is ".to_string()));
        assert!(matches!(subs[1], Substitution::Eval(_)));
    }

    #[test]
    fn test_parse_eval_complex() {
        let subs = parse_substitutions("$(eval (3 + 4) * 2)").unwrap();
        assert_eq!(subs.len(), 1);
        assert!(matches!(subs[0], Substitution::Eval(_)));
    }

    // Command error mode tests
    #[test]
    fn test_parse_command_with_warn_mode() {
        let subs = parse_substitutions("$(command 'exit 1' 'warn')").unwrap();
        assert_eq!(subs.len(), 1);
        if let Substitution::Command { cmd, error_mode } = &subs[0] {
            assert_eq!(*error_mode, CommandErrorMode::Warn);
            assert_eq!(cmd.len(), 1);
            assert_eq!(cmd[0], Substitution::Text("exit 1".to_string()));
        } else {
            panic!("Expected Command");
        }
    }

    #[test]
    fn test_parse_command_with_ignore_mode() {
        let subs = parse_substitutions("$(command 'exit 1' 'ignore')").unwrap();
        assert_eq!(subs.len(), 1);
        if let Substitution::Command { cmd, error_mode } = &subs[0] {
            assert_eq!(*error_mode, CommandErrorMode::Ignore);
            assert_eq!(cmd.len(), 1);
            assert_eq!(cmd[0], Substitution::Text("exit 1".to_string()));
        } else {
            panic!("Expected Command");
        }
    }

    #[test]
    fn test_parse_command_with_strict_mode() {
        let subs = parse_substitutions("$(command 'exit 1' 'strict')").unwrap();
        assert_eq!(subs.len(), 1);
        if let Substitution::Command { cmd, error_mode } = &subs[0] {
            assert_eq!(*error_mode, CommandErrorMode::Strict);
            assert_eq!(cmd.len(), 1);
            assert_eq!(cmd[0], Substitution::Text("exit 1".to_string()));
        } else {
            panic!("Expected Command");
        }
    }

    #[test]
    fn test_parse_command_without_error_mode() {
        // Should default to Strict
        let subs = parse_substitutions("$(command 'echo test')").unwrap();
        assert_eq!(subs.len(), 1);
        if let Substitution::Command { error_mode, .. } = &subs[0] {
            assert_eq!(*error_mode, CommandErrorMode::Strict);
        } else {
            panic!("Expected Command");
        }
    }
}
