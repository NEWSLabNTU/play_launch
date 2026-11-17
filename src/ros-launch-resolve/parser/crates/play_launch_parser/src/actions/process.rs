//! What a process starts IN and UNDER, beyond its command line (#0085,
//! #0087): the variables launch unset, the command it runs under
//! (`launch-prefix`), and its working directory (`cwd`).
//!
//! Every frontend resolves these here, at the point the action executes, so
//! the XML, YAML and Python paths agree on launch's rules:
//!
//! - `prefix` given on the action wins. Otherwise launch reads the
//!   `launch-prefix` launch configuration (default empty) and applies it only
//!   when `launch-prefix-filter` (a Python `re.match` pattern, default empty,
//!   which matches everything) matches the basename of the command's first
//!   word — `launch.descriptions.Executable.prepare`.
//! - The prefix is split with `shlex.split` and prepended to the command.
//!   That happens at spawn; here it is kept as text.
//! - `cwd` is the substituted text; launch passes it to the subprocess as is.
//! - The unsets are launch's environment deletions in effect here, minus any
//!   name the process's own environment sets again, and minus any name the
//!   launcher's environment does not have (unsetting it changes nothing).

use crate::{
    error::SubstitutionError,
    record::ProcessSpawn,
    substitution::{LaunchContext, Substitution, resolve_substitutions},
};

fn invalid(msg: String) -> SubstitutionError {
    SubstitutionError::InvalidSubstitution(msg)
}

/// Whether `launch-prefix-filter` lets the configured prefix apply to
/// `program`. Python's `re.match`: anchored at the start only.
fn prefix_filter_matches(filter: &str, program: &str) -> Result<bool, SubstitutionError> {
    if filter.is_empty() {
        return Ok(true);
    }
    let base = std::path::Path::new(program)
        .file_name()
        .and_then(|b| b.to_str())
        .unwrap_or(program);
    let re = regex::Regex::new(&format!("^(?:{filter})")).map_err(|e| {
        invalid(format!(
            "launch-prefix-filter '{filter}' is not a pattern: {e}"
        ))
    })?;
    Ok(re.is_match(base))
}

/// The prefix launch runs `program` under, as text, or `None` for none.
pub fn resolve_prefix(
    prefix: Option<&[Substitution]>,
    program: &str,
    context: &LaunchContext,
) -> Result<Option<String>, SubstitutionError> {
    let text = match prefix {
        Some(subs) => resolve_substitutions(subs, context)?,
        None => {
            let configured = context
                .get_configuration_lenient("launch-prefix")
                .unwrap_or_default();
            if configured.trim().is_empty() {
                return Ok(None);
            }
            let filter = context
                .get_configuration_lenient("launch-prefix-filter")
                .unwrap_or_default();
            if !prefix_filter_matches(&filter, program)? {
                return Ok(None);
            }
            configured
        }
    };
    let words = shlex::split(&text)
        .ok_or_else(|| invalid(format!("launch-prefix '{text}' has an unbalanced quote")))?;
    Ok((!words.is_empty()).then_some(text))
}

/// The prefix's words, as launch's `shlex.split` gives them.
pub fn prefix_words(prefix: &str) -> Vec<String> {
    shlex::split(prefix).unwrap_or_default()
}

/// Everything [`ProcessSpawn`] carries for a process declared here.
///
/// `program` is the command's first word (for `launch-prefix-filter`);
/// `own_env` the names the process's environment sets, which an earlier
/// unset does not remove.
pub fn resolve_spawn<'a>(
    prefix: Option<&[Substitution]>,
    cwd: Option<&[Substitution]>,
    program: &str,
    own_env: impl IntoIterator<Item = &'a str>,
    context: &LaunchContext,
) -> Result<ProcessSpawn, SubstitutionError> {
    let own: std::collections::HashSet<&str> = own_env.into_iter().collect();
    Ok(ProcessSpawn {
        // Only what the launcher's own environment has: unsetting a name it
        // lacks changes nothing for the process, and the Python dumper, which
        // sees launch's environment rather than its actions, cannot record
        // one either.
        env_unset: context
            .environment_unset()
            .into_iter()
            .filter(|k| !own.contains(k.as_str()) && launcher_has(k))
            .collect(),
        launch_prefix: resolve_prefix(prefix, program, context)?,
        cwd: cwd
            .map(|subs| resolve_substitutions(subs, context))
            .transpose()?,
    })
}

thread_local! {
    /// The launcher's environment as the launch started, while a walk
    /// rewrites the process's own (#0092): the Python frontend keeps
    /// `os.environ` in step with the launch's environment actions, so once an
    /// `UnsetEnvironmentVariable` has run, `std::env` no longer has the name.
    static LAUNCHER_ENV: std::cell::RefCell<Option<std::collections::HashSet<String>>> =
        const { std::cell::RefCell::new(None) };
}

/// Set (or, with `None`, clear) the names the launcher's environment had
/// when the launch started; see [`LAUNCHER_ENV`].
pub fn set_launcher_environment(names: Option<std::collections::HashSet<String>>) {
    LAUNCHER_ENV.with(|e| *e.borrow_mut() = names);
}

/// Whether the launcher's environment, as the launch started, has `name`.
fn launcher_has(name: &str) -> bool {
    LAUNCHER_ENV
        .with(|e| e.borrow().as_ref().map(|names| names.contains(name)))
        .unwrap_or_else(|| std::env::var_os(name).is_some())
}

/// [`resolve_spawn`] for a frontend that has already resolved its values
/// (the Python mocks perform substitutions themselves).
pub fn spawn_from_resolved<'a>(
    prefix: Option<String>,
    cwd: Option<String>,
    program: &str,
    own_env: impl IntoIterator<Item = &'a str>,
    context: &LaunchContext,
) -> Result<ProcessSpawn, SubstitutionError> {
    let subs = prefix.map(|p| vec![Substitution::Text(p)]);
    let cwd = cwd.map(|c| vec![Substitution::Text(c)]);
    resolve_spawn(subs.as_deref(), cwd.as_deref(), program, own_env, context)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn text(s: &str) -> Vec<Substitution> {
        vec![Substitution::Text(s.to_string())]
    }

    #[test]
    fn an_action_prefix_wins_over_the_configuration() {
        let mut ctx = LaunchContext::new();
        ctx.set_configuration_literal("launch-prefix".into(), "valgrind".into());
        let p = resolve_prefix(Some(&text("env -u DISPLAY")), "/x/talker", &ctx).unwrap();
        assert_eq!(p.as_deref(), Some("env -u DISPLAY"));
    }

    #[test]
    fn the_configuration_applies_when_the_action_gives_none() {
        let mut ctx = LaunchContext::new();
        assert_eq!(resolve_prefix(None, "/x/talker", &ctx).unwrap(), None);
        ctx.set_configuration_literal("launch-prefix".into(), "gdb -ex run --args".into());
        assert_eq!(
            resolve_prefix(None, "/x/talker", &ctx).unwrap().as_deref(),
            Some("gdb -ex run --args")
        );
    }

    /// `re.match(filter, basename(cmd[0]))`: anchored at the start, on the
    /// basename. A filter never applies to an action's own prefix.
    #[test]
    fn the_filter_matches_the_basename_from_its_start() {
        let mut ctx = LaunchContext::new();
        ctx.set_configuration_literal("launch-prefix".into(), "valgrind".into());
        ctx.set_configuration_literal("launch-prefix-filter".into(), "talk".into());
        assert!(resolve_prefix(None, "/x/talker", &ctx).unwrap().is_some());
        assert!(resolve_prefix(None, "/x/listener", &ctx).unwrap().is_none());
        ctx.set_configuration_literal("launch-prefix-filter".into(), "ker".into());
        assert!(resolve_prefix(None, "/x/talker", &ctx).unwrap().is_none());
        assert!(
            resolve_prefix(Some(&text("env")), "/x/talker", &ctx)
                .unwrap()
                .is_some()
        );
    }

    #[test]
    fn an_empty_prefix_is_none_and_an_unbalanced_one_is_an_error() {
        let ctx = LaunchContext::new();
        assert_eq!(resolve_prefix(Some(&text("  ")), "x", &ctx).unwrap(), None);
        assert!(resolve_prefix(Some(&text("env 'A")), "x", &ctx).is_err());
    }

    #[test]
    fn an_unset_the_process_sets_again_is_not_carried() {
        let mut ctx = LaunchContext::new();
        // HOME and PATH are inherited in every test environment.
        ctx.unset_environment_variable("HOME");
        ctx.unset_environment_variable("PATH");
        ctx.unset_environment_variable("PL_NEVER_INHERITED_0085");
        let s = resolve_spawn(None, None, "x", ["HOME"], &ctx).unwrap();
        assert_eq!(s.env_unset, vec!["PATH".to_string()]);
    }
}
