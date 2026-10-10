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

/// The substitution grammar of `launch`'s frontends
/// (`launch/frontend/grammar.lark`, Humble), parsed by hand.
///
/// ```text
/// template   := (substitution | text)*         -- top level; text runs to the next `$(`
/// subst      := "$(" IDENT (" " arguments)? ")"
/// arguments  := value (" " value)*
/// value      := (subst | rstring)+  |  'quoted template'  |  "quoted template"
/// quoted(q)  := q (subst_q | text up to q)* q  -- inside, nested substitutions'
///                                                 arguments may not contain q
/// ```
///
/// `\x` is `x` in every piece of text, at the top level as well
/// (`replace_escaped_characters`), so `\$(var a)` is the literal `$(var a)`.
/// A quoted argument loses its quotes HERE, at parse time: `$(eval "'a' +
/// '$(var b)'")` hands Python `'a' + '<b>'` whatever `<b>` contains. The parser
/// this replaces split arguments per substitution by hand and left quotes for
/// `$(eval)` to guess at after substituting, so a value containing a quote
/// (Autoware's `launch_module_list_end` is `""]`) broke the expression.
fn parse_substitutions_recursive(input: &str) -> Result<Vec<Substitution>> {
    if input.is_empty() {
        return Ok(vec![Substitution::Text(String::new())]);
    }
    let chars: Vec<char> = input.chars().collect();
    let mut p = Parser {
        chars: &chars,
        pos: 0,
    };
    let result = p.template()?;
    if result.is_empty() {
        return Ok(vec![Substitution::Text(String::new())]);
    }
    Ok(result)
}

struct Parser<'a> {
    chars: &'a [char],
    pos: usize,
}

/// Where a piece of text ends.
#[derive(Clone, Copy, PartialEq)]
enum Ctx {
    /// The top-level template: only `$(` ends text.
    Top,
    /// Inside a quoted template delimited by this quote.
    Quoted(char),
}

fn err(msg: impl Into<String>) -> ParseError {
    ParseError::InvalidSubstitution(msg.into())
}

/// Append text, merging with a preceding text piece.
fn push_text(out: &mut Vec<Substitution>, text: String) {
    if text.is_empty() {
        return;
    }
    if let Some(Substitution::Text(prev)) = out.last_mut() {
        prev.push_str(&text);
    } else {
        out.push(Substitution::Text(text));
    }
}

impl Parser<'_> {
    fn peek(&self) -> Option<char> {
        self.chars.get(self.pos).copied()
    }

    fn peek_at(&self, n: usize) -> Option<char> {
        self.chars.get(self.pos + n).copied()
    }

    fn at_subst(&self) -> bool {
        self.peek() == Some('$') && self.peek_at(1) == Some('(')
    }

    /// The top-level template.
    fn template(&mut self) -> Result<Vec<Substitution>> {
        let mut out = Vec::new();
        while self.pos < self.chars.len() {
            if self.at_subst() {
                let s = self.substitution(None)?;
                out.push(s);
            } else {
                let t = self.text(Ctx::Top);
                push_text(&mut out, t);
            }
        }
        Ok(out)
    }

    /// Text up to the next substitution (or, quoted, the closing quote),
    /// with `\x` read as `x`.
    fn text(&mut self, ctx: Ctx) -> String {
        let mut out = String::new();
        while let Some(c) = self.peek() {
            if self.at_subst() {
                break;
            }
            if let Ctx::Quoted(q) = ctx
                && c == q
            {
                break;
            }
            if c == '\\' {
                match self.peek_at(1) {
                    Some(next) => {
                        out.push(next);
                        self.pos += 2;
                    }
                    None => {
                        out.push('\\');
                        self.pos += 1;
                    }
                }
                continue;
            }
            out.push(c);
            self.pos += 1;
        }
        out
    }

    /// `$(IDENT args...)`. `quote` is the quote of the template this sits in,
    /// which its unquoted arguments may not contain.
    fn substitution(&mut self, quote: Option<char>) -> Result<Substitution> {
        let start = self.pos;
        self.pos += 2; // "$("
        let ident_start = self.pos;
        while let Some(c) = self.peek() {
            if c.is_alphanumeric() || c == '_' || c == '-' {
                self.pos += 1;
            } else {
                break;
            }
        }
        let ident: String = self.chars[ident_start..self.pos].iter().collect();
        if ident.is_empty() {
            return Err(err(format!(
                "expected a substitution name after `$(` at position {start}"
            )));
        }
        let mut args = Vec::new();
        loop {
            match self.peek() {
                Some(')') => {
                    self.pos += 1;
                    break;
                }
                Some(' ') => {
                    while self.peek() == Some(' ') {
                        self.pos += 1;
                    }
                    if self.peek() == Some(')') {
                        continue;
                    }
                    args.push(self.value(quote)?);
                }
                Some(c) => {
                    return Err(err(format!(
                        "unexpected '{c}' in `$({ident} ...)`: arguments are separated by spaces"
                    )));
                }
                None => {
                    return Err(err(format!(
                        "Unmatched parentheses in substitution `$({ident} ...`"
                    )));
                }
            }
        }
        build(&ident, args)
    }

    /// One argument.
    fn value(&mut self, quote: Option<char>) -> Result<Vec<Substitution>> {
        if quote.is_none()
            && let Some(q @ ('\'' | '"')) = self.peek()
        {
            return self.quoted(q);
        }
        let mut out = Vec::new();
        loop {
            match self.peek() {
                None | Some(' ') | Some(')') => break,
                Some(c) if Some(c) == quote => {
                    return Err(err(format!(
                        "a substitution argument inside a {c}-quoted string cannot contain {c}"
                    )));
                }
                Some('(') => {
                    return Err(err(
                        "unescaped '(' in a substitution argument (write \\( or quote it)",
                    ));
                }
                Some('\'' | '"') if quote.is_none() => {
                    return Err(err(
                        "a quote in the middle of a substitution argument (quote the whole argument)",
                    ));
                }
                _ => {}
            }
            if self.at_subst() {
                let s = self.substitution(quote)?;
                out.push(s);
                continue;
            }
            // An rstring: up to a space, a paren, a quote or a substitution.
            let mut t = String::new();
            while let Some(c) = self.peek() {
                if self.at_subst() || c == ' ' || c == '(' || c == ')' {
                    break;
                }
                if (c == '\'' || c == '"') && (quote.is_none() || Some(c) == quote) {
                    break;
                }
                if c == '\\' {
                    match self.peek_at(1) {
                        Some(next) => {
                            t.push(next);
                            self.pos += 2;
                        }
                        None => {
                            t.push('\\');
                            self.pos += 1;
                        }
                    }
                    continue;
                }
                t.push(c);
                self.pos += 1;
            }
            push_text(&mut out, t);
        }
        if out.is_empty() {
            return Err(err("empty substitution argument"));
        }
        Ok(out)
    }

    /// A quoted template argument, quotes consumed.
    fn quoted(&mut self, q: char) -> Result<Vec<Substitution>> {
        self.pos += 1; // opening quote
        let mut out = Vec::new();
        loop {
            match self.peek() {
                None => {
                    return Err(err(format!(
                        "unterminated {q}-quoted substitution argument"
                    )));
                }
                Some(c) if c == q => {
                    self.pos += 1;
                    break;
                }
                _ => {}
            }
            if self.at_subst() {
                let s = self.substitution(Some(q))?;
                out.push(s);
            } else {
                let t = self.text(Ctx::Quoted(q));
                push_text(&mut out, t);
            }
        }
        if out.is_empty() {
            out.push(Substitution::Text(String::new()));
        }
        Ok(out)
    }
}

/// Build a substitution from its name and arguments, with `launch`'s arity
/// rules for each.
fn build(ident: &str, mut args: Vec<Vec<Substitution>>) -> Result<Substitution> {
    let n = args.len();
    let arity = |lo: usize, hi: usize| -> Result<()> {
        if n < lo || n > hi {
            Err(err(if lo == hi {
                format!("{ident} substitution expects {lo} argument(s), got {n}")
            } else {
                format!("{ident} substitution expects {lo} to {hi} arguments, got {n}")
            }))
        } else {
            Ok(())
        }
    };
    let mut take = || args.remove(0);
    Ok(match ident {
        "var" => {
            arity(1, 2)?;
            let name = take();
            if n == 2 {
                Substitution::Call {
                    name: "var".into(),
                    args: vec![name, take()],
                }
            } else {
                Substitution::LaunchConfiguration(name)
            }
        }
        "env" | "optenv" => {
            arity(1, 2)?;
            let name = take();
            let default = (n == 2).then(&mut take);
            if ident == "env" {
                Substitution::EnvironmentVariable { name, default }
            } else {
                Substitution::OptionalEnvironmentVariable { name, default }
            }
        }
        "command" => {
            arity(1, 2)?;
            let cmd = take();
            let error_mode = if n == 2 {
                match take().as_slice() {
                    [Substitution::Text(t)] => match t.as_str() {
                        "fail" | "strict" => CommandErrorMode::Strict,
                        "warn" => CommandErrorMode::Warn,
                        "ignore" => CommandErrorMode::Ignore,
                        "capture" => CommandErrorMode::Capture,
                        other => {
                            return Err(err(format!(
                                "expected 'on_stderr' to be one of: 'fail', 'ignore', 'warn' or \
                                 'capture', got '{other}'"
                            )));
                        }
                    },
                    _ => {
                        return Err(err(
                            "the command substitution's on_stderr must be literal text",
                        ));
                    }
                }
            } else {
                CommandErrorMode::Strict
            };
            Substitution::Command { cmd, error_mode }
        }
        "find-pkg-share" => {
            arity(1, 1)?;
            Substitution::FindPackageShare(take())
        }
        "dirname" => {
            arity(0, 0)?;
            Substitution::Dirname
        }
        "filename" => {
            arity(0, 0)?;
            Substitution::Filename
        }
        "anon" => {
            arity(1, 1)?;
            Substitution::Anon(take())
        }
        "eval" => {
            arity(1, 1)?;
            Substitution::Eval(take())
        }
        "find-pkg-prefix" | "find-exec" | "file-content" | "not" | "param" => {
            arity(1, 1)?;
            Substitution::Call {
                name: ident.into(),
                args,
            }
        }
        "exec-in-pkg" | "equals" | "not-equals" | "and" | "or" => {
            arity(2, 2)?;
            Substitution::Call {
                name: ident.into(),
                args,
            }
        }
        "if" => {
            arity(2, 3)?;
            Substitution::Call {
                name: ident.into(),
                args,
            }
        }
        "any" | "all" => Substitution::Call {
            name: ident.into(),
            args,
        },
        "launch_log_dir" | "log_dir" => {
            arity(0, 0)?;
            Substitution::Call {
                name: "launch_log_dir".into(),
                args,
            }
        }
        _ => {
            return Err(ParseError::InvalidSubstitution(format!(
                "Unknown substitution type: {}",
                ident
            )));
        }
    })
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
        let subs = parse_substitutions("$(command 'echo hello')").unwrap();
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
        let subs = parse_substitutions("$(command 'ls -la')").unwrap();
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
        let subs = parse_substitutions("Version: $(command 'cat /etc/os-release')").unwrap();
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
        let subs = parse_substitutions("$(command 'echo test | tr a-z A-Z')").unwrap();
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
        let subs = parse_substitutions("$(command 'echo $(var value)')").unwrap();
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
        let subs = parse_substitutions("$(eval '1 + 2')").unwrap();
        assert_eq!(subs.len(), 1);
        assert!(matches!(subs[0], Substitution::Eval(_)));
    }

    #[test]
    fn test_parse_eval_with_var() {
        let subs = parse_substitutions("$(eval '$(var x) + 5')").unwrap();
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
        let subs = parse_substitutions("value is $(eval '10 * 5')").unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(subs[0], Substitution::Text("value is ".to_string()));
        assert!(matches!(subs[1], Substitution::Eval(_)));
    }

    #[test]
    fn test_parse_eval_complex() {
        let subs = parse_substitutions("$(eval '(3 + 4) * 2')").unwrap();
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
