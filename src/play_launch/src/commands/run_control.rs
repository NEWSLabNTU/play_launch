//! Bounded runs and per-run launch prefixes (phase 106 W3).
//!
//! `--run-for` ends a launch after a time, `--fail-on` names what makes it a
//! failure, and `--fail-fast` ends it at the first one; together they turn a
//! launch into a test whose exit code is the verdict (design
//! `docs/design/launch-selection.md` 7.3). `--prefix` runs selected nodes
//! under a command (gdb, valgrind, perf) without editing the launch file
//! (7.1).
//!
//! A failed verdict exits with [`VERDICT_EXIT_CODE`], apart from every other
//! failure (1), so a CI script can tell "the system misbehaved" from "the
//! launch could not run".

use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::Duration,
};

use eyre::{Result, bail};
use glob::{MatchOptions, Pattern};
use ros_launch_resolve::selection::{SelectionSpec, Selector};
use tracing::{info, warn};

/// The exit code of a run a `--fail-on` condition failed.
pub const VERDICT_EXIT_CODE: i32 = 4;

const FQN_MATCH: MatchOptions = MatchOptions {
    case_sensitive: true,
    require_literal_separator: true,
    require_literal_leading_dot: false,
};

/// The error a failed verdict returns; `main` maps it to
/// [`VERDICT_EXIT_CODE`].
#[derive(Debug)]
pub struct VerdictFailed(pub String);

impl std::fmt::Display for VerdictFailed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "the run failed: {}", self.0)
    }
}

impl std::error::Error for VerdictFailed {}

/// One `--fail-on` condition.
#[derive(Debug, Clone)]
pub enum Condition {
    /// A Health issue of this kind opened, optionally only on matching
    /// subjects.
    Health {
        kind: String,
        subject: Option<Pattern>,
    },
    /// Runtime enforcement reported an error-severity finding.
    Contract,
    /// A matching member exited other than by a stop.
    Exit { member: Pattern },
}

impl Condition {
    fn parse(text: &str) -> Result<Self> {
        let pattern = |p: &str| {
            Pattern::new(p).map_err(|e| eyre::eyre!("--fail-on {text}: invalid pattern: {e}"))
        };
        if text == "contract" {
            return Ok(Condition::Contract);
        }
        if let Some(rest) = text.strip_prefix("health:") {
            let (kind, subject) = match rest.split_once(':') {
                Some((k, s)) => (k, Some(pattern(s)?)),
                None => (rest, None),
            };
            if kind.is_empty() {
                bail!("--fail-on {text}: name the Health issue kind, e.g. health:crash_loop");
            }
            return Ok(Condition::Health {
                kind: kind.to_string(),
                subject,
            });
        }
        if let Some(rest) = text.strip_prefix("exit:") {
            return Ok(Condition::Exit {
                member: pattern(rest)?,
            });
        }
        bail!(
            "--fail-on {text}: expected `health:<kind>[:<subject glob>]`, `contract` or \
             `exit:<node glob>`"
        )
    }
}

/// Parsed `--run-for`, `--fail-on`, `--fail-fast` and `--prefix`.
#[derive(Debug, Clone, Default)]
pub struct RunControl {
    pub run_for: Option<Duration>,
    pub fail_on: Vec<(String, Condition)>,
    pub fail_fast: bool,
    pub prefixes: Vec<(Selector, String)>,
}

/// Split `SELECTOR=COMMAND` at the first `=` outside a `[...]` predicate.
fn split_prefix(text: &str) -> Option<(&str, &str)> {
    let mut depth = 0i32;
    for (i, c) in text.char_indices() {
        match c {
            '[' => depth += 1,
            ']' => depth -= 1,
            '=' if depth == 0 => return Some((&text[..i], &text[i + 1..])),
            _ => {}
        }
    }
    None
}

impl RunControl {
    pub fn parse(opts: &crate::cli::options::RunControlOptions) -> Result<Self> {
        let run_for = opts
            .run_for
            .as_deref()
            .map(|d| {
                humantime_serde::re::humantime::parse_duration(d)
                    .map_err(|e| eyre::eyre!("--run-for {d}: {e} (write e.g. 90s, 5m, 1h)"))
            })
            .transpose()?;
        let fail_on = opts
            .fail_on
            .iter()
            .map(|t| Condition::parse(t).map(|c| (t.clone(), c)))
            .collect::<Result<Vec<_>>>()?;
        if opts.fail_fast && fail_on.is_empty() {
            bail!("--fail-fast ends the run at the first --fail-on condition; give one");
        }
        let mut prefixes = Vec::new();
        for p in &opts.prefix {
            let Some((sel, cmd)) = split_prefix(p) else {
                bail!("--prefix {p}: expected SELECTOR=COMMAND, e.g. '/x/node=gdb -ex run --args'");
            };
            let selector: Selector = sel
                .parse()
                .map_err(|e: String| eyre::eyre!("--prefix {p}: {e}"))?;
            if cmd.trim().is_empty() {
                bail!("--prefix {p}: no command after `=`");
            }
            prefixes.push((selector, cmd.trim().to_string()));
        }
        Ok(RunControl {
            run_for,
            fail_on,
            fail_fast: opts.fail_fast,
            prefixes,
        })
    }

    /// Whether the run has a verdict to record.
    pub fn bounded(&self) -> bool {
        self.run_for.is_some() || !self.fail_on.is_empty()
    }
}

/// Replace the launch prefix of every node a `--prefix` selector matches,
/// in the model the run spawns from. A composable has no process of its own
/// unless every composable is one (`isolated`), so its prefix goes on its
/// container, and the log says so. Returns `(node, prefix)` pairs applied.
pub fn apply_prefixes(
    model: &mut ros_launch_manifest_model::SystemModel,
    prefixes: &[(Selector, String)],
    isolated: bool,
) -> Result<Vec<(String, String)>> {
    let mut applied: Vec<(String, String)> = Vec::new();
    for (selector, command) in prefixes {
        let spec = SelectionSpec {
            only: vec![selector.clone()],
            no_deps: true,
            ..Default::default()
        };
        let matched = ros_launch_resolve::selection::evaluate(model, &spec)
            .map_err(|e| eyre::eyre!("--prefix {selector}={command}: {e}"))?;
        for key in &matched.kept {
            let Some(node) = model.structure.nodes.get(key) else {
                continue;
            };
            let target = match (&node.container, isolated) {
                (Some(_), true) => bail!(
                    "--prefix {selector}: `{key}` is a composable node, and under \
                     --container-mode isolated each runs in a process the container forks, \
                     which cannot take a prefix yet; prefix its container instead"
                ),
                (Some(container), false) => {
                    info!(
                        "--prefix: `{key}` is a composable node and shares its container's \
                         process; the prefix goes on `{container}`"
                    );
                    container.clone()
                }
                (None, _) => key.clone(),
            };
            if let Some(n) = model.structure.nodes.get_mut(&target) {
                n.launch_prefix = Some(command.clone());
            }
            if !applied.iter().any(|(k, _)| k == &target) {
                applied.push((target, command.clone()));
            }
        }
    }
    for (k, c) in &applied {
        info!("--prefix: {k} runs under `{c}`");
    }
    Ok(applied)
}

/// What a bounded run watches, handed in by `play()`.
pub struct Watch {
    pub control: RunControl,
    pub member_handle: Arc<crate::member_actor::MemberHandle>,
    pub health: tokio::sync::watch::Receiver<Arc<crate::health::HealthSnapshot>>,
    /// The rule engine's first error-severity finding, when one runs.
    pub contract_error: Option<Arc<Mutex<Option<String>>>>,
    pub pgid: i32,
    pub shutdown_tx: tokio::sync::watch::Sender<bool>,
    pub shutdown: tokio::sync::watch::Receiver<bool>,
}

/// The first condition that held, and when (ms since the watch began).
pub type FailureSlot = Arc<Mutex<Option<(String, u64)>>>;

/// Watch the run for `--fail-on` conditions and end it at `--run-for`.
/// Returns the slot the first failure is written into.
pub fn spawn(w: Watch) -> FailureSlot {
    let slot: FailureSlot = Arc::new(Mutex::new(None));
    let out = slot.clone();
    if w.control
        .fail_on
        .iter()
        .any(|(_, c)| matches!(c, Condition::Contract))
        && w.contract_error.is_none()
    {
        warn!(
            "--fail-on contract: this run checks no contract at run time (no contract resolved, \
             interception off, or --enforce-rules off), so the condition cannot hold"
        );
    }
    tokio::spawn(async move {
        let Watch {
            control,
            member_handle,
            mut health,
            contract_error,
            pgid,
            shutdown_tx,
            mut shutdown,
        } = w;
        let started = std::time::Instant::now();
        let deadline = control.run_for.map(|d| tokio::time::Instant::now() + d);
        let mut exits_seen: HashMap<String, usize> = HashMap::new();
        let mut tick = tokio::time::interval(Duration::from_millis(250));
        loop {
            tokio::select! {
                _ = tick.tick() => {}
                Ok(()) = health.changed() => {}
                _ = async {
                    match deadline {
                        Some(d) => tokio::time::sleep_until(d).await,
                        None => std::future::pending().await,
                    }
                } => {
                    info!(
                        "--run-for {}: time is up; shutting down",
                        humantime_serde::re::humantime::format_duration(control.run_for.unwrap_or_default())
                    );
                    crate::commands::signal_handler::initiate_shutdown(pgid, &shutdown_tx, &member_handle);
                    break;
                }
                _ = shutdown.changed() => {
                    if *shutdown.borrow() {
                        break;
                    }
                }
            }
            if slot.lock().map(|s| s.is_some()).unwrap_or(true) {
                continue;
            }
            let held = check(
                &control,
                &member_handle,
                &health,
                contract_error.as_ref(),
                &mut exits_seen,
            )
            .await;
            if let Some(reason) = held {
                let ms = started.elapsed().as_millis() as u64;
                warn!(
                    "--fail-on: {reason} ({:.1} s into the run)",
                    ms as f64 / 1000.0
                );
                if let Ok(mut s) = slot.lock() {
                    *s = Some((reason, ms));
                }
                if control.fail_fast {
                    info!("--fail-fast: shutting down");
                    crate::commands::signal_handler::initiate_shutdown(
                        pgid,
                        &shutdown_tx,
                        &member_handle,
                    );
                    break;
                }
            }
        }
    });
    out
}

/// The first `--fail-on` condition that holds now, described.
async fn check(
    control: &RunControl,
    member_handle: &crate::member_actor::MemberHandle,
    health: &tokio::sync::watch::Receiver<Arc<crate::health::HealthSnapshot>>,
    contract_error: Option<&Arc<Mutex<Option<String>>>>,
    exits_seen: &mut HashMap<String, usize>,
) -> Option<String> {
    let history = member_handle.all_exit_history();
    for (text, cond) in &control.fail_on {
        match cond {
            Condition::Health { kind, subject } => {
                let snap = health.borrow().clone();
                let hit = snap.open.iter().find(|i| {
                    i.kind == kind.as_str()
                        && subject.as_ref().is_none_or(|p| {
                            let name = i.subject.name.as_str();
                            let bare = name.split_once(':').map(|(_, n)| n).unwrap_or(name);
                            p.matches_with(name, FQN_MATCH) || p.matches_with(bare, FQN_MATCH)
                        })
                });
                if let Some(i) = hit {
                    return Some(format!(
                        "{text}: {} on {}: {}",
                        i.kind, i.subject.name, i.title
                    ));
                }
            }
            Condition::Contract => {
                if let Some(e) = contract_error.and_then(|h| h.lock().ok().and_then(|s| s.clone()))
                {
                    return Some(format!("{text}: {e}"));
                }
            }
            Condition::Exit { member } => {
                for (id, exits) in &history {
                    let fqn = id.split_once(':').map(|(_, n)| n).unwrap_or(id);
                    if !member.matches_with(fqn, FQN_MATCH) {
                        continue;
                    }
                    let seen = exits_seen.entry(id.clone()).or_insert(0);
                    if exits.len() > *seen {
                        *seen = exits.len();
                        // A stop asked for (or a switch-off) is not a failure.
                        if member_handle.stopped_by_user(id) {
                            continue;
                        }
                        let last = exits.last();
                        return Some(format!(
                            "{text}: {id} exited ({})",
                            match last.map(|e| (e.code, e.signal)) {
                                Some((Some(c), _)) => format!("code {c}"),
                                Some((None, Some(s))) => format!("signal {s}"),
                                _ => "no exit code".to_string(),
                            }
                        ));
                    }
                }
            }
        }
    }
    None
}

/// Record the verdict and turn a failure into [`VerdictFailed`].
pub fn finish(control: &RunControl, slot: &FailureSlot) -> Result<()> {
    let failure = slot.lock().ok().and_then(|s| s.clone());
    crate::util::run_log::record_verdict(serde_json::json!({
        "passed": failure.is_none(),
        "failure": failure.as_ref().map(|(r, _)| r),
        "at_ms": failure.as_ref().map(|(_, ms)| ms),
        "run_for_s": control.run_for.map(|d| d.as_secs_f64()),
        "fail_on": control.fail_on.iter().map(|(t, _)| t).collect::<Vec<_>>(),
    }));
    match failure {
        Some((reason, _)) => Err(VerdictFailed(reason).into()),
        None => {
            if !control.fail_on.is_empty() {
                info!("--fail-on: no condition held; the run passed");
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conditions_parse() {
        assert!(matches!(
            Condition::parse("contract").unwrap(),
            Condition::Contract
        ));
        match Condition::parse("health:crash_loop:/planning/**").unwrap() {
            Condition::Health { kind, subject } => {
                assert_eq!(kind, "crash_loop");
                assert!(subject.unwrap().matches_with("/planning/a/b", FQN_MATCH));
            }
            c => panic!("{c:?}"),
        }
        assert!(matches!(
            Condition::parse("exit:/x").unwrap(),
            Condition::Exit { .. }
        ));
        for bad in ["health:", "nope", "exit:[", "health"] {
            assert!(Condition::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn prefix_splits_outside_predicates() {
        assert_eq!(split_prefix("/a=gdb --args"), Some(("/a", "gdb --args")));
        assert_eq!(
            split_prefix("x.launch.xml[part=a]=valgrind"),
            Some(("x.launch.xml[part=a]", "valgrind"))
        );
        assert_eq!(split_prefix("/a"), None);
    }
}
