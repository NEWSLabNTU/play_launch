//! `check` — parse a launch file and check manifest contracts. No ROS install
//! required — the checker is a layer-2 crate.
//!
//! # Exit contract
//!
//! This verb signals its verdict through the PROCESS exit status, so it is
//! the one entry point in [`crate::verbs`] that returns a code rather than
//! `()`. It does not call [`std::process::exit`] itself — that decision
//! belongs to whichever CLI invoked it. The contract, unchanged from when
//! this lived in `ros-launch-resolve-cli`:
//!
//! | outcome                                            | [`run`] returns |
//! |----------------------------------------------------|-----------------|
//! | parse / manifest-load / sched-validation failure    | `Err(..)`       |
//! | an action was DROPPED (unsupported), strict         | `Ok(1)`         |
//! | no manifests found at all                           | `Ok(0)`         |
//! | manifests checked, no Error-severity diagnostic     | `Ok(0)`         |
//! | at least one Error-severity diagnostic (post-filter)| `Ok(1)`         |
//! | a contract file was REFUSED (`manifest-parse`)      | `Ok(3)`         |
//! | `--expect`: exactly the expected rules erred        | `Ok(0)`         |
//! | `--expect`: a rule missing, an extra rule, a drop   | `Ok(1)`         |
//!
//! Phase 85 I4: a refusal is its own verdict, [`EXIT_REFUSED`] (3), so a CI
//! script expecting a rule to fail (exit 1) cannot pass on a checker too old
//! to parse the contract. It wins over every other outcome, `--rule` and
//! `--expect` included: a refused file was not checked at all. 2 is not used
//! because clap exits 2 on a usage error.
//!
//! The error count is taken AFTER `--rule` filtering and spans both
//! per-scope and cross-scope diagnostics, so `--rule` narrows the exit code
//! exactly as it narrows the printed output. A caller that maps `Err(..)` to
//! a non-zero status (both CLIs do, via `main`'s `Result`) reproduces the
//! original behaviour byte for byte.

use std::{collections::HashSet, path::PathBuf};

use eyre::Result;
use ros_launch_manifest_check::{
    Diagnostic, Severity, emit::diagnostic::emit_diagnostics_to, run_checks_with_spans,
};
use ros_launch_manifest_types::parse_manifest_str_with_spans;

use crate::ros::manifest_loader;

/// Everything `check` reads. Owned plain values — see [`crate::verbs`].
pub struct CheckInputs {
    /// Package name, or a path to a launch file.
    pub package_or_path: String,
    /// Launch file name, when `package_or_path` is a package name.
    pub launch_file: Option<String>,
    /// `KEY:=VALUE` launch arguments.
    pub launch_arguments: Vec<String>,
    /// Overlay root for user-supplied contracts.
    pub contracts: Option<PathBuf>,
    /// Disable the provider-sidecar channel.
    pub no_provider_contracts: bool,
    /// Scheduling platform file (v2 `.yaml` or legacy `.toml`).
    pub sched: Option<PathBuf>,
    /// Scheduling target the platform file must declare.
    pub target: String,
    /// Output format: `terminal` (default, with source excerpts) or `json`.
    pub format: String,
    /// Show only diagnostics from these rule ids. Empty = no filter.
    pub rule: Vec<String>,
    /// Print the merged scheduling plan with per-node provenance.
    pub explain: bool,
    /// Export the declared causal graph to this path (`.json` or `.dot`).
    pub export_graph: Option<PathBuf>,
    /// Downgrade dropped (unsupported) launch actions from an error to a
    /// warning. Off by default — see [`report_dropped_actions`] for why
    /// strict is the default.
    pub allow_unsupported_actions: bool,
    /// Emit a derived artifact instead of running the checks. Today:
    /// `diagnostics-params` — `diagnostic_updater` parameters restated from
    /// the declared endpoint bounds (phase 71 W5).
    pub emit: Option<String>,
    /// Phase 85 I4: the rule ids this check is EXPECTED to fail with. Empty
    /// = an ordinary check. Otherwise the verdict is 0 only when every
    /// Error-severity diagnostic is one of these rules, each of them fired,
    /// and no contract was refused -- see [`expect_verdict`].
    pub expect: Vec<String>,
    /// Phase 85 I5: ASCII-only output (`->`, `--`, `-`/`|`/`+`). Also on
    /// whenever stderr is not a terminal.
    pub ascii: bool,
    /// Phase 85 I5: wrap every output line longer than this many characters.
    /// `None` = no wrapping.
    pub width: Option<usize>,
}

/// Exit status for a contract file the grammar refused (`manifest-parse`),
/// distinct from 1 (the contract has errors). Phase 85 I4.
pub const EXIT_REFUSED: i32 = 3;

impl CheckInputs {
    /// Build the two-step `ContractSources` from these inputs.
    ///
    /// The overlay root is discovered (Phase 41.3 §3.2) when `contracts`
    /// isn't given: `$PLAY_LAUNCH_CONTRACTS`, then
    /// `$XDG_CONFIG_HOME/play_launch/contracts`, then
    /// `/etc/play_launch/contracts` — first existing wins.
    fn contract_sources(&self) -> eyre::Result<manifest_loader::ContractSources> {
        Ok(manifest_loader::ContractSources {
            overlay: manifest_loader::discover_overlay_root(self.contracts.as_deref())?,
            provider: !self.no_provider_contracts,
        })
    }
}

/// Run the contract checker. Returns the INTENDED process exit code — see the
/// module-level exit contract. Never calls [`std::process::exit`].
pub fn run(inputs: CheckInputs) -> Result<i32> {
    // Phase 85 I5: what reaches a log is ASCII, and as wide as asked.
    crate::util::out::configure(
        inputs.ascii || !std::io::IsTerminal::is_terminal(&std::io::stderr()),
        inputs.width.unwrap_or(0),
    );
    // Every load and cross-scope diagnostic is rendered below; the loader's
    // own WARN line for each would print a parse failure twice.
    manifest_loader::set_diagnostics_rendered(true);

    // Positional quirk: with a direct launch-file PATH, the second
    // positional (`launch_file`) can swallow the first `KEY:=VALUE` arg.
    // Without this, `check` parsed a DIFFERENT node set than `resolve` did
    // for the same command line and reported the result as authoritative.
    // See [`super::reclassify_launch_file_arg`].
    let (launch_file, launch_arguments) =
        super::reclassify_launch_file_arg(inputs.launch_file.as_deref(), &inputs.launch_arguments);

    // Resolve launch file path (same logic as `play_launch launch`)
    let launch_path = super::resolve_launch_file(&inputs.package_or_path, launch_file.as_deref())?;

    crate::say!("Parsing launch file: {}", launch_path.display());

    // Parse launch arguments (KEY:=VALUE)
    let cli_args = super::parse_launch_arguments(&launch_arguments);

    // Parse launch file → record with scope table
    let record = crate::verbs::parse_launch_file(&launch_path, cli_args)
        .map_err(|e| eyre::eyre!("Parser error: {e}"))?;

    // Convert to LaunchDump (reuse existing deserialization path)
    let json = serde_json::to_string(&record)?;
    let dump: crate::ros::launch_dump::LaunchDump = serde_json::from_str(&json)?;

    crate::say!(
        "Parsed: {} scopes, {} nodes, {} containers, {} composable nodes",
        dump.scopes.len(),
        dump.node.len(),
        dump.container.len(),
        dump.load_node.len(),
    );

    // Gate 1 — actions this parser dropped. Evaluated BEFORE manifests
    // because it is independent of them: the launch trees this catches
    // typically have no contracts at all, and the no-manifest path below
    // returns `Ok(0)` early, which is precisely how a `<timer>` could delete
    // a subtree and still be reported clean.
    let dropped_is_error = report_dropped_actions(&dump, inputs.allow_unsupported_actions);

    // Load and check manifests: overlay > provider sidecar. The provider
    // channel is on by default, so `check` works with no manifest flags at
    // all whenever the launch file ships sidecar contracts.
    let sources = inputs.contract_sources()?;
    let index = manifest_loader::load_manifests(&dump, &sources)?;

    // Phase 71 W5: a consequence, printed rather than written. Every ROS user
    // who runs `diagnostic_updater` configures `FrequencyStatus` and
    // `TimeStampStatus` by hand from numbers that ARE the contract's
    // `min_rate_hz` / `max_rate_hz` / `max_age`. Restating them here is the
    // adoption path for a system with no hazard analysis: declare the bounds
    // once, get the monitor's parameters for free.
    if let Some(what) = inputs.emit.as_deref() {
        if what != "diagnostics-params" {
            eyre::bail!("unknown --emit `{what}` (accepted: diagnostics-params)");
        }
        print!("{}", emit_diagnostics_params(&index));
        return Ok(0);
    }

    // Export the declared causal graph (Phase 42.1). This is an export, not
    // a validation step — it runs regardless of rule filters/errors below
    // and doesn't affect the exit code.
    if let Some(export_path) = &inputs.export_graph {
        crate::ros::causal_graph::export_to_file(&index, export_path)?;
        crate::say!("Exported causal graph to {}", export_path.display());
    }

    // Optional: validate the shared scheduling spec (Linux = validate-now).
    // Loaded after manifests so contract-aware mappers (rate_monotonic,
    // deadline_monotonic) can extract timing facts from `index`.
    //
    // Resolve the platform file through the same channels as contracts
    // (Phase 41.3): explicit `--sched <path>` > overlay > provider sidecar.
    // `sources.overlay` is already the discovered root (see
    // `CheckInputs::contract_sources`), so both channels agree on which root
    // is in play.
    let resolved_platform = crate::ros::sched_loader::resolve_platform_file(
        &dump,
        inputs.sched.as_deref(),
        sources.overlay.as_deref(),
        sources.provider,
        &inputs.target,
    );
    if let Some(resolved) = &resolved_platform {
        crate::say!(
            "Scheduling platform file [{}]: {}",
            resolved.channel,
            resolved.path.display()
        );
        let derived = crate::ros::sched_loader::check_sched(
            &dump,
            Some(&index),
            &resolved.path,
            &inputs.target,
        )?;
        if inputs.explain {
            crate::ros::sched_loader::print_explain(&derived, resolved, Some(&index));
        }
    } else if inputs.explain && index.budgets.is_empty() {
        crate::say!(
            "note: --explain has no effect without a resolved scheduling platform file \
             (pass --sched <path>, or ship one via the overlay/provider channels)"
        );
    }

    // "None found" and "found, but none could be read" are different verdicts.
    // Only the first is a clean exit; conflating them let a contract file that
    // failed to parse report as if the user had shipped no contracts at all.
    //
    // Phase 76 added a third case: no manifest, and a topic graph anyway,
    // derived from the launch file's own remaps. Most rules have nothing to
    // say without a declared requirement, but `causal-dag-global` does -- a
    // cycle is a defect whether or not anyone wrote a budget -- and
    // `graph-from-remaps` reports the graph a later verdict would be computed
    // over. Returning here dropped both, so a tree with no contracts got the
    // same silent exit 0 it got when the graph was empty.
    if index.manifests.is_empty() && index.load_diagnostics.is_empty() {
        crate::say!(
            "No manifests found (overlay={:?}, provider={})",
            sources.overlay,
            sources.provider
        );
        if index.merge_diagnostics.is_empty() {
            return Ok(if dropped_is_error { 1 } else { 0 });
        }
    }

    // Build the rule filter set (empty = no filter, show all)
    let rule_filter: Option<HashSet<&str>> = if inputs.rule.is_empty() {
        None
    } else {
        Some(inputs.rule.iter().map(String::as_str).collect())
    };

    // Render contracts that never loaded, before anything that tallies only
    // the files that did.
    render_load_diagnostics(&index, &inputs.format, rule_filter.as_ref())?;

    // Render per-scope diagnostics
    render_scope_diagnostics(&index, &inputs.format, rule_filter.as_ref())?;

    // Render cross-scope diagnostics (consistency, dangling-entity, budget-overflow)
    render_cross_scope_diagnostics(&index, &inputs.format, rule_filter.as_ref())?;

    // Phase 83: the fault-reaction arithmetic the verdicts above were
    // computed from, one row per (hazard, rung).
    if inputs.explain && inputs.format != "json" && !index.budgets.is_empty() {
        crate::say_raw!("{}", render_budgets(&index.budgets));
    }

    // Summary
    print_summary(&index, rule_filter.as_ref());

    let refused = index
        .load_diagnostics
        .iter()
        .filter(|d| d.rule_id == "manifest-parse")
        .count();

    if !inputs.expect.is_empty() {
        let (code, line) = expect_verdict(&index, &inputs.expect, refused, dropped_is_error);
        crate::say!("{line}");
        return Ok(code);
    }

    if refused > 0 {
        return Ok(EXIT_REFUSED);
    }

    if has_filtered_errors(&index, rule_filter.as_ref()) || dropped_is_error {
        return Ok(1);
    }

    Ok(0)
}

/// `--expect <rule-id>` (phase 85 I4): the verdict of a NEGATIVE test.
///
/// A CI script that wants a contract to fail one named rule used to compare
/// exit codes only, and a contract the checker could not parse also exits
/// non-zero, so a checker too old for the grammar passed every negative test.
/// Here the expected outcome is stated, and anything else is a failure with a
/// line saying which:
///
/// - a refused contract file: [`EXIT_REFUSED`] -- the rules never ran;
/// - an expected rule that did not fire as an error: 1;
/// - an Error-severity diagnostic from a rule not expected: 1;
/// - a dropped (unsupported) launch action, unless downgraded: 1.
///
/// The rule set is taken BEFORE `--rule` filtering: `--rule` narrows what is
/// printed, `--expect` states the whole verdict.
pub fn expect_verdict(
    index: &manifest_loader::ManifestIndex,
    expect: &[String],
    refused: usize,
    dropped_is_error: bool,
) -> (i32, String) {
    use std::collections::BTreeMap;
    let want: std::collections::BTreeSet<&str> = expect.iter().map(String::as_str).collect();
    if refused > 0 {
        return (
            EXIT_REFUSED,
            format!(
                "expect: FAILED -- {refused} contract file(s) were refused \
                 (error[manifest-parse]), so the expected rule(s) [{}] were never checked",
                want.iter().copied().collect::<Vec<_>>().join(", ")
            ),
        );
    }
    let mut fired: BTreeMap<&str, usize> = BTreeMap::new();
    for d in index
        .manifests
        .values()
        .flat_map(|m| m.diagnostics.iter())
        .chain(index.merge_diagnostics.iter())
        .chain(index.load_diagnostics.iter())
        .filter(|d| d.severity == Severity::Error)
    {
        *fired.entry(d.rule_id.as_str()).or_default() += 1;
    }
    let missing: Vec<&str> = want
        .iter()
        .copied()
        .filter(|r| !fired.contains_key(r))
        .collect();
    let extra: Vec<String> = fired
        .iter()
        .filter(|(r, _)| !want.contains(*r))
        .map(|(r, n)| format!("error[{r}] x{n}"))
        .collect();
    let mut why: Vec<String> = Vec::new();
    if !missing.is_empty() {
        why.push(format!(
            "expected {} did not fire",
            missing
                .iter()
                .map(|r| format!("error[{r}]"))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    if !extra.is_empty() {
        why.push(format!("unexpected {}", extra.join(", ")));
    }
    if dropped_is_error {
        why.push("unsupported launch action(s) were dropped".to_string());
    }
    if why.is_empty() {
        (
            0,
            format!(
                "expect: ok -- the errors are exactly [{}]",
                fired
                    .iter()
                    .map(|(r, n)| format!("error[{r}] x{n}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        )
    } else {
        (1, format!("expect: FAILED -- {}", why.join("; ")))
    }
}

/// `check --explain`'s fault-reaction table (phase 83): per hazard, every
/// rung of its ladder with the terms the checker charged -- detection, the
/// windowed rungs waited out on the way, the rung's own route, the settle and
/// how it was reached -- against the interval. A rung the fault removes is
/// listed with the reason it was skipped, so the selection is visible too.
pub fn render_budgets(rows: &[manifest_loader::RungBudget]) -> String {
    use manifest_loader::RungRole;
    let ms = |v: Option<f64>| v.map_or("-".to_string(), |v| format!("{v:.2}"));
    let mut out = String::from("\n-- Fault-reaction budgets (--explain, ms) --\n");
    let head = [
        "HAZARD", "RUNG", "ROLE", "DETECT", "WINDOWS", "ROUTE", "SETTLE", "TOTAL", "FTTI", "SLACK",
    ];
    let mut table: Vec<[String; 10]> = vec![head.map(str::to_string)];
    let mut notes: Vec<String> = Vec::new();
    for r in rows {
        let role = match r.role {
            RungRole::Rung => "rung",
            RungRole::Window => "window",
            RungRole::Skipped => "skipped",
            RungRole::Floor => "floor",
        };
        let settle = match (r.role, r.settle_ms) {
            (RungRole::Window, _) => format!("window >={}", ms(r.window_ms)),
            (_, Some(v)) => format!("{v:.2} {}", r.settle_how),
            (RungRole::Skipped, None) => "-".to_string(),
            (_, None) => "unknown".to_string(),
        };
        let slack = match (r.total_ms, r.ftti_ms) {
            (Some(t), Some(f)) => format!("{:.2}", f - t),
            _ => "-".to_string(),
        };
        let windows = if r.role == RungRole::Skipped {
            "-".to_string()
        } else {
            format!("{:.2}", r.windows_ms)
        };
        table.push([
            r.hazard.clone(),
            r.rung.clone(),
            role.to_string(),
            ms(r.fdti_ms),
            windows,
            ms(r.route_ms),
            settle,
            ms(r.total_ms),
            ms(r.ftti_ms),
            slack,
        ]);
        if !r.note.is_empty() {
            notes.push(format!("  {}/{}: {}", r.hazard, r.rung, r.note));
        }
    }
    let widths: Vec<usize> = (0..10)
        .map(|c| table.iter().map(|row| row[c].len()).max().unwrap_or(0))
        .collect();
    for row in &table {
        let line: Vec<String> = row
            .iter()
            .zip(&widths)
            .enumerate()
            .map(|(c, (cell, w))| {
                if c < 3 {
                    format!("{cell:<w$}")
                } else {
                    format!("{cell:>w$}")
                }
            })
            .collect();
        out.push_str(line.join("  ").trim_end());
        out.push('\n');
    }
    out.push_str(
        "  TOTAL = DETECT + WINDOWS + ROUTE + SETTLE; WINDOWS is the route and window of every \
         windowed rung passed on the way, up to its deadline.\n  A window is a least time \
         (`window >=`); noticing its deadline is the first hop of the ROUTE below it \
         (`window-expiry`), never a second charge.\n",
    );
    for n in notes {
        out.push_str(&n);
        out.push('\n');
    }
    out
}

/// Print the actions the parser dropped, and say whether that is fatal.
///
/// # Why strict is the default
///
/// A dropped action takes its whole subtree with it. `<timer period="3">`
/// around four nodes did not mean "those four start three seconds late", it
/// meant those four nodes were absent from the model — and `check` exited 0,
/// so nothing in CI could be pointed at the failure. A checker that cannot
/// fail on "I did not understand a third of this file" is not a gate.
///
/// The escape hatch is `--allow-unsupported-actions`, for a launch tree that
/// knowingly uses an action this parser does not implement and wants the
/// contract verdict anyway. It downgrades to a warning; it never hides the
/// finding.
fn report_dropped_actions(dump: &crate::ros::launch_dump::LaunchDump, allow: bool) -> bool {
    if dump.dropped_actions.is_empty() {
        return false;
    }
    let level = if allow { "warning" } else { "error" };
    for d in &dump.dropped_actions {
        let where_ = match &d.file {
            Some(f) => format!(" in {f}"),
            None => String::new(),
        };
        // `detail` exists because not every drop loses the same thing: the
        // Python frontend keeps a `TimerAction`'s nodes and loses only the
        // delay, and saying "everything nested inside it is missing" there
        // would send the reader looking for nodes that are present.
        let what = d.detail.clone().unwrap_or_else(|| {
            "it and everything nested inside it is MISSING from the model".to_string()
        });
        crate::say!(
            "{level}: unsupported launch action `{}`{where_} — {what}",
            d.action
        );
    }
    if allow {
        crate::say!(
            "note: --allow-unsupported-actions is set, so the {} dropped action(s) above \
             do not fail this check",
            dump.dropped_actions.len()
        );
        false
    } else {
        crate::say!(
            "note: pass --allow-unsupported-actions to downgrade the {} dropped action(s) \
             above to a warning",
            dump.dropped_actions.len()
        );
        true
    }
}

/// Apply the rule filter to a slice of diagnostics.
fn filter_diagnostics<'a>(
    diags: &'a [Diagnostic],
    filter: Option<&HashSet<&str>>,
) -> Vec<&'a Diagnostic> {
    diags
        .iter()
        .filter(|d| match filter {
            None => true,
            Some(set) => set.contains(d.rule_id.as_str()),
        })
        .collect()
}

/// Render diagnostics for all scopes in the index.
fn render_scope_diagnostics(
    index: &manifest_loader::ManifestIndex,
    format: &str,
    rule_filter: Option<&HashSet<&str>>,
) -> Result<()> {
    for (scope_id, resolved) in &index.manifests {
        let filename = if let Some(ref pkg) = resolved.pkg {
            format!("{}/{}", pkg, resolved.file)
        } else {
            resolved.file.clone()
        };
        let label = if resolved.ns.is_empty() || resolved.ns == "/" {
            format!(
                "{filename} (scope {scope_id}) [{}: {}]",
                resolved.channel,
                resolved.contract_path.display()
            )
        } else {
            format!(
                "{filename} (scope {scope_id}, ns={}) [{}: {}]",
                resolved.ns,
                resolved.channel,
                resolved.contract_path.display()
            )
        };

        let filtered = filter_diagnostics(&resolved.diagnostics, rule_filter);
        if filtered.is_empty() {
            continue;
        }

        if format == "json" {
            print_diagnostics_json(
                &filtered,
                &label,
                Some((&resolved.channel.to_string(), &resolved.contract_path)),
            )?;
        } else {
            // Re-run checks with spans to get a proper CheckResult for the emitter,
            // then filter rules.
            if let Ok(parsed) = parse_manifest_str_with_spans(&resolved.source) {
                let mut check_result = run_checks_with_spans(&parsed.manifest, parsed.spans);
                // Suppress per-manifest `dangling-entity` and
                // `service-wiring` warnings — the cross-scope merge in
                // `manifest_loader` is authoritative. Per-manifest emission
                // creates O(n) duplicates for legitimate cross-scope
                // endpoints.
                check_result
                    .diagnostics
                    .retain(|d| d.rule_id != "dangling-entity" && d.rule_id != "service-wiring");
                // Show what the LOADER decided, not what a fresh re-run
                // derives. The re-run exists only to recover spans the stored
                // diagnostics lack, so anything the loader suppressed —
                // a `scope-budget` sum superseded by a real critical path, say
                // — must not reappear here. Without this the terminal and
                // `--format json` outputs disagree, since the JSON path emits
                // the stored diagnostics directly.
                let kept: std::collections::HashSet<(String, String)> = resolved
                    .diagnostics
                    .iter()
                    .map(|d| (d.rule_id.clone(), d.path.clone()))
                    .collect();
                check_result
                    .diagnostics
                    .retain(|d| kept.contains(&(d.rule_id.clone(), d.path.clone())));
                if let Some(set) = rule_filter {
                    check_result
                        .diagnostics
                        .retain(|d| set.contains(d.rule_id.as_str()));
                }
                // Phase 85 I5: colour only on a terminal without NO_COLOR
                // (codespan's own `Auto` coloured a pipe), and the text
                // through `util::out` for `--ascii` / `--width`.
                let mut buf = if crate::util::out::color_wanted(std::io::IsTerminal::is_terminal(
                    &std::io::stderr(),
                )) {
                    termcolor::Buffer::ansi()
                } else {
                    termcolor::Buffer::no_color()
                };
                emit_diagnostics_to(&mut buf, &check_result, &label, &resolved.source);
                crate::say_raw!("{}", String::from_utf8_lossy(buf.as_slice()));
            } else {
                for diag in &filtered {
                    crate::say!("{diag}");
                }
            }
        }
    }

    Ok(())
}

/// `diagnostic_updater` parameters as a consequence of the declared bounds.
///
/// One `ros__parameters` block per node that declares a rate or age bound on
/// a subscriber. `frequency.min`/`max` are `min_rate_hz`/`max_rate_hz`;
/// `timestamp.max_acceptable` is `max_age` in seconds. `tolerance` and
/// `window_size` are `diagnostic_updater`'s own defaults, named so an author
/// can see them. A subscriber with none of the three emits nothing.
pub fn emit_diagnostics_params(index: &manifest_loader::ManifestIndex) -> String {
    let mut out = String::new();
    out.push_str(
        "# diagnostic_updater parameters, DERIVED from the contract's endpoint bounds by\n\
         # `play_launch check --emit diagnostics-params`. Do not edit: change the contract.\n\
         #   frequency.min / max      <- sub.<ep>.min_rate_hz / max_rate_hz\n\
         #   timestamp.max_acceptable <- sub.<ep>.max_age (seconds)\n",
    );
    let mut nodes: Vec<(String, String)> = Vec::new();
    let mut manifests: Vec<&manifest_loader::ResolvedManifest> = index.manifests.values().collect();
    manifests.sort_by_key(|m| m.scope_id);
    for m in manifests {
        for (name, decl) in &m.manifest.nodes {
            let fqn = manifest_loader::resolve_node_fqn(index, m.scope_id, &m.ns, name);
            let mut body = String::new();
            for (ep, props) in &decl.subscribers {
                if props.min_rate_hz.is_none()
                    && props.max_rate_hz.is_none()
                    && props.max_age.is_none()
                {
                    continue;
                }
                body.push_str(&format!("      {ep}:\n"));
                if props.min_rate_hz.is_some() || props.max_rate_hz.is_some() {
                    body.push_str("        frequency:\n");
                    if let Some(v) = props.min_rate_hz {
                        body.push_str(&format!("          min: {v}\n"));
                    }
                    if let Some(v) = props.max_rate_hz {
                        body.push_str(&format!("          max: {v}\n"));
                    }
                    body.push_str("          tolerance: 0.1\n          window_size: 10\n");
                }
                if let Some(age) = props.max_age {
                    body.push_str(&format!(
                        "        timestamp:\n          min_acceptable: 0.0\n          max_acceptable: {}\n",
                        age.as_millis_f64() / 1000.0
                    ));
                }
            }
            if !body.is_empty() {
                nodes.push((fqn, body));
            }
        }
    }
    nodes.sort();
    for (fqn, body) in nodes {
        out.push_str(&format!(
            "{fqn}:\n  ros__parameters:\n    diagnostic_updater:\n{body}"
        ));
    }
    if out.lines().count() <= 4 {
        out.push_str("# (no subscriber declares min_rate_hz, max_rate_hz or max_age)\n");
    }
    out
}

/// Render contracts that never loaded. Printed BEFORE every other section:
/// a file that failed to parse is missing from every tally that follows, so
/// reading those first would mean reading a clean report about a subset.
fn render_load_diagnostics(
    index: &manifest_loader::ManifestIndex,
    format: &str,
    rule_filter: Option<&HashSet<&str>>,
) -> Result<()> {
    let filtered = filter_diagnostics(&index.load_diagnostics, rule_filter);
    if filtered.is_empty() {
        return Ok(());
    }

    if format == "json" {
        print_diagnostics_json(&filtered, "<load>", None)?;
    } else {
        crate::say!("\n── Contracts that failed to load ──");
        for diag in &filtered {
            crate::say!("  error[{}]: {}", diag.rule_id, diag.message);
        }
    }

    Ok(())
}

/// Render cross-scope diagnostics (consistency, dangling-entity, budget-overflow).
fn render_cross_scope_diagnostics(
    index: &manifest_loader::ManifestIndex,
    format: &str,
    rule_filter: Option<&HashSet<&str>>,
) -> Result<()> {
    let filtered = filter_diagnostics(&index.merge_diagnostics, rule_filter);
    if filtered.is_empty() {
        return Ok(());
    }

    if format == "json" {
        print_diagnostics_json(&filtered, "<cross-scope>", None)?;
    } else {
        crate::say!("\n── Cross-scope diagnostics ──");
        for diag in &filtered {
            let label = match diag.severity {
                Severity::Error => "error",
                Severity::Warning => "warning",
                Severity::Info => "info",
            };
            crate::say!("  {label}[{}]: {}", diag.rule_id, diag.message);
        }
    }

    Ok(())
}

/// Print summary line, accounting for the rule filter.
fn print_summary(index: &manifest_loader::ManifestIndex, rule_filter: Option<&HashSet<&str>>) {
    let (per_scope_errors, per_scope_warnings) = count_severities(
        index.manifests.values().flat_map(|m| m.diagnostics.iter()),
        rule_filter,
    );
    let (cross_errors, cross_warnings) =
        count_severities(index.merge_diagnostics.iter(), rule_filter);
    // Files that never became a manifest. Counted separately because they are
    // absent from `index.manifests`, so the clean/with-errors split below
    // cannot see them at all.
    let (load_errors, load_warnings) = count_severities(index.load_diagnostics.iter(), rule_filter);

    let total_errors = per_scope_errors + cross_errors + load_errors;
    let total_warnings = per_scope_warnings + cross_warnings + load_warnings;
    let (clean_count, error_count) = tally_manifests(
        index.manifests.values().map(|m| m.diagnostics.as_slice()),
        cross_errors,
        rule_filter,
    );

    let filter_note = match rule_filter {
        Some(set) => format!(
            " [filter: {}]",
            set.iter().copied().collect::<Vec<_>>().join(",")
        ),
        None => String::new(),
    };
    if !index.load_diagnostics.is_empty() {
        crate::say!(
            "{} contract file(s) FAILED TO LOAD and were not checked at all",
            index.load_diagnostics.len()
        );
    }
    crate::say!(
        "\n{} manifest(s) checked: {} clean, {} with errors ({} errors, {} warnings){}",
        index.manifests.len(),
        clean_count,
        error_count,
        total_errors,
        total_warnings,
        filter_note,
    );

    let mut overlay_count = 0usize;
    let mut provider_count = 0usize;
    for resolved in index.manifests.values() {
        match resolved.channel {
            manifest_loader::ContractChannel::Overlay => overlay_count += 1,
            manifest_loader::ContractChannel::Provider => provider_count += 1,
        }
    }
    crate::say!(
        "{} contract(s): {} overlay, {} provider",
        index.manifests.len(),
        overlay_count,
        provider_count,
    );
}

/// Split the checked manifests into (clean, with errors).
///
/// A manifest is clean only if it carries no Error-severity diagnostic of
/// its own AND the set it was checked in carries no cross-scope error. A
/// cross-scope diagnostic (consistency, dangling entity, budget overflow) is
/// a finding about how the scopes combine, and `Diagnostic` names no scope,
/// so it cannot be pinned to one manifest: it implicates every manifest of
/// the merge. Before this, a single manifest with a cross-scope error was
/// reported as "1 clean, 0 with errors (1 errors ...)" while `check` exited
/// 1. `cross_errors` is the post-filter count, as the exit code uses.
fn tally_manifests<'a>(
    per_manifest: impl Iterator<Item = &'a [Diagnostic]>,
    cross_errors: usize,
    rule_filter: Option<&HashSet<&str>>,
) -> (usize, usize) {
    let mut clean = 0usize;
    let mut with_errors = 0usize;
    for diags in per_manifest {
        let own_error = filter_diagnostics(diags, rule_filter)
            .iter()
            .any(|d| d.severity == Severity::Error);
        if own_error || cross_errors > 0 {
            with_errors += 1;
        } else {
            clean += 1;
        }
    }
    (clean, with_errors)
}

fn count_severities<'a>(
    diags: impl Iterator<Item = &'a Diagnostic>,
    rule_filter: Option<&HashSet<&str>>,
) -> (usize, usize) {
    let mut errors = 0usize;
    let mut warnings = 0usize;
    for d in diags {
        if let Some(set) = rule_filter
            && !set.contains(d.rule_id.as_str())
        {
            continue;
        }
        match d.severity {
            Severity::Error => errors += 1,
            Severity::Warning => warnings += 1,
            Severity::Info => {}
        }
    }
    (errors, warnings)
}

fn has_filtered_errors(
    index: &manifest_loader::ManifestIndex,
    rule_filter: Option<&HashSet<&str>>,
) -> bool {
    let combined = index
        .manifests
        .values()
        .flat_map(|m| m.diagnostics.iter())
        .chain(index.merge_diagnostics.iter())
        .chain(index.load_diagnostics.iter());
    count_severities(combined, rule_filter).0 > 0
}

fn print_diagnostics_json(
    diagnostics: &[&Diagnostic],
    label: &str,
    channel_and_path: Option<(&str, &std::path::Path)>,
) -> Result<()> {
    let diags: Vec<serde_json::Value> = diagnostics
        .iter()
        .map(|d| {
            let mut obj = serde_json::json!({
                "file": label,
                "rule": d.rule_id,
                "severity": d.severity.to_string(),
                "message": d.message,
                "path": d.path,
                "span": d.span.as_ref().map(|s| {
                    serde_json::json!({"start": s.start, "end": s.end})
                }),
            });
            if let Some((channel, contract_path)) = channel_and_path
                && let Some(map) = obj.as_object_mut()
            {
                map.insert("channel".to_string(), serde_json::json!(channel));
                map.insert(
                    "contract_path".to_string(),
                    serde_json::json!(contract_path.display().to_string()),
                );
            }
            obj
        })
        .collect();
    println!("{}", serde_json::to_string_pretty(&diags)?);
    Ok(())
}

#[cfg(test)]
mod summary_tally_tests {
    use std::collections::HashSet;

    use super::{Diagnostic, Severity, tally_manifests};

    fn diag(rule: &str, severity: Severity) -> Diagnostic {
        Diagnostic {
            rule_id: rule.to_string(),
            severity,
            message: String::new(),
            path: String::new(),
            span: None,
        }
    }

    /// The W20 regression: one manifest, no error of its own, one
    /// cross-scope error. It was counted clean.
    #[test]
    fn a_cross_scope_error_makes_the_manifest_not_clean() {
        let own: Vec<Diagnostic> = vec![diag("E001", Severity::Warning)];
        let tally = tally_manifests([own.as_slice()].into_iter(), 1, None);
        assert_eq!(tally, (0, 1));
    }

    #[test]
    fn a_cross_scope_error_implicates_every_manifest_of_the_merge() {
        let a: Vec<Diagnostic> = Vec::new();
        let b: Vec<Diagnostic> = vec![diag("E001", Severity::Error)];
        let tally = tally_manifests([a.as_slice(), b.as_slice()].into_iter(), 2, None);
        assert_eq!(tally, (0, 2));
    }

    #[test]
    fn without_cross_scope_errors_only_own_errors_count() {
        let a: Vec<Diagnostic> = vec![diag("W001", Severity::Warning)];
        let b: Vec<Diagnostic> = vec![diag("E001", Severity::Error)];
        let tally = tally_manifests([a.as_slice(), b.as_slice()].into_iter(), 0, None);
        assert_eq!(tally, (1, 1));
    }

    /// `--rule` narrows the tally as it narrows the exit code: an own error
    /// the filter excludes does not count.
    #[test]
    fn the_rule_filter_applies_to_own_errors() {
        let a: Vec<Diagnostic> = vec![diag("E001", Severity::Error)];
        let filter: HashSet<&str> = ["E999"].into_iter().collect();
        let tally = tally_manifests([a.as_slice()].into_iter(), 0, Some(&filter));
        assert_eq!(tally, (1, 0));
    }
}

#[cfg(test)]
mod dropped_action_tests {
    use super::report_dropped_actions;
    use crate::ros::launch_dump::{DroppedAction, LaunchDump};

    fn dump_with(actions: Vec<DroppedAction>) -> LaunchDump {
        let mut d = LaunchDump::empty();
        d.dropped_actions = actions;
        d
    }

    fn dropped(action: &str) -> DroppedAction {
        DroppedAction {
            action: action.to_string(),
            file: Some("/tmp/x.launch.xml".to_string()),
            detail: None,
        }
    }

    /// The default. `check` exits 0 on a clean parse, as it always has.
    #[test]
    fn nothing_dropped_is_not_an_error() {
        assert!(!report_dropped_actions(&dump_with(Vec::new()), false));
        assert!(!report_dropped_actions(&dump_with(Vec::new()), true));
    }

    /// The bug this exists for: `check` exited 0 on a launch file whose
    /// nodes the parser had thrown away, so no CI gate could catch it.
    #[test]
    fn a_dropped_action_is_an_error_by_default() {
        assert!(report_dropped_actions(
            &dump_with(vec![dropped("timer")]),
            false
        ));
    }

    /// The escape hatch downgrades, and only downgrades — the finding is
    /// still printed either way.
    #[test]
    fn allow_unsupported_actions_downgrades_to_a_warning() {
        assert!(!report_dropped_actions(
            &dump_with(vec![dropped("timer")]),
            true
        ));
    }

    /// A drop with no known file is still a drop.
    #[test]
    fn a_drop_without_a_file_still_fails() {
        let d = DroppedAction {
            action: "log".to_string(),
            file: None,
            detail: None,
        };
        assert!(report_dropped_actions(&dump_with(vec![d]), false));
    }
}
