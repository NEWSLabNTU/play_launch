//! `play_launch check` — parse a launch file and check manifest contracts.

use crate::{cli::options::CheckArgs, ros::manifest_loader};
use eyre::Result;
use ros_launch_manifest_check::{
    Diagnostic, Severity, emit::diagnostic::emit_diagnostics, run_checks_with_spans,
};
use ros_launch_manifest_types::parse_manifest_str_with_spans;
use std::collections::HashSet;

pub fn handle_check_manifest(args: &CheckArgs) -> Result<()> {
    // Resolve launch file path (same logic as `play_launch launch`)
    let launch_path =
        super::launch::resolve_launch_file(&args.package_or_path, args.launch_file.as_deref())?;

    eprintln!("Parsing launch file: {}", launch_path.display());

    // Parse launch arguments (KEY:=VALUE)
    let cli_args = super::parse_launch_arguments(&args.launch_arguments);

    // Parse launch file → record with scope table
    let record = play_launch_parser::parse_launch_file(&launch_path, cli_args)
        .map_err(|e| eyre::eyre!("Parser error: {e}"))?;

    // Convert to LaunchDump (reuse existing deserialization path)
    let json = serde_json::to_string(&record)?;
    let dump: crate::ros::launch_dump::LaunchDump = serde_json::from_str(&json)?;

    eprintln!(
        "Parsed: {} scopes, {} nodes, {} containers, {} composable nodes",
        dump.scopes.len(),
        dump.node.len(),
        dump.container.len(),
        dump.load_node.len(),
    );

    // Load and check manifests
    let index = manifest_loader::load_manifests(&dump, &args.manifest_dir)?;

    if index.manifests.is_empty() {
        eprintln!("No manifests found in {}", args.manifest_dir.display());
        return Ok(());
    }

    // Build the rule filter set (empty = no filter, show all)
    let rule_filter: Option<HashSet<&str>> = if args.rule.is_empty() {
        None
    } else {
        Some(args.rule.iter().map(String::as_str).collect())
    };

    // Render per-scope diagnostics
    render_scope_diagnostics(&index, &args.format, rule_filter.as_ref())?;

    // Render cross-scope diagnostics (consistency, dangling-entity, budget-overflow)
    render_cross_scope_diagnostics(&index, &args.format, rule_filter.as_ref())?;

    // Summary
    print_summary(&index, rule_filter.as_ref());

    if has_filtered_errors(&index, rule_filter.as_ref()) {
        std::process::exit(1);
    }

    Ok(())
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
            format!("{filename} (scope {scope_id})")
        } else {
            format!("{filename} (scope {scope_id}, ns={})", resolved.ns)
        };

        let filtered = filter_diagnostics(&resolved.diagnostics, rule_filter);
        if filtered.is_empty() {
            continue;
        }

        if format == "json" {
            print_diagnostics_json(&filtered, &label)?;
        } else {
            // Re-run checks with spans to get a proper CheckResult for the emitter,
            // then filter rules.
            if let Ok(parsed) = parse_manifest_str_with_spans(&resolved.source) {
                let mut check_result = run_checks_with_spans(&parsed.manifest, parsed.spans);
                if let Some(set) = rule_filter {
                    check_result
                        .diagnostics
                        .retain(|d| set.contains(d.rule_id.as_str()));
                }
                emit_diagnostics(&check_result, &label, &resolved.source);
            } else {
                for diag in &filtered {
                    eprintln!("{diag}");
                }
            }
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
        print_diagnostics_json(&filtered, "<cross-scope>")?;
    } else {
        eprintln!("\n── Cross-scope diagnostics ──");
        for diag in &filtered {
            let label = match diag.severity {
                Severity::Error => "error",
                Severity::Warning => "warning",
                Severity::Info => "info",
            };
            eprintln!("  {label}[{}]: {}", diag.rule_id, diag.message);
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

    let total_errors = per_scope_errors + cross_errors;
    let total_warnings = per_scope_warnings + cross_warnings;
    let clean_count = index
        .manifests
        .values()
        .filter(|m| {
            filter_diagnostics(&m.diagnostics, rule_filter)
                .iter()
                .all(|d| d.severity != Severity::Error)
        })
        .count();
    let error_count = index.manifests.len() - clean_count;

    let filter_note = match rule_filter {
        Some(set) => format!(
            " [filter: {}]",
            set.iter().copied().collect::<Vec<_>>().join(",")
        ),
        None => String::new(),
    };
    eprintln!(
        "\n{} manifest(s) checked: {} clean, {} with errors ({} errors, {} warnings){}",
        index.manifests.len(),
        clean_count,
        error_count,
        total_errors,
        total_warnings,
        filter_note,
    );
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
        .chain(index.merge_diagnostics.iter());
    count_severities(combined, rule_filter).0 > 0
}

fn print_diagnostics_json(diagnostics: &[&Diagnostic], label: &str) -> Result<()> {
    let diags: Vec<serde_json::Value> = diagnostics
        .iter()
        .map(|d| {
            serde_json::json!({
                "file": label,
                "rule": d.rule_id,
                "severity": d.severity.to_string(),
                "message": d.message,
                "path": d.path,
                "span": d.span.as_ref().map(|s| {
                    serde_json::json!({"start": s.start, "end": s.end})
                }),
            })
        })
        .collect();
    println!("{}", serde_json::to_string_pretty(&diags)?);
    Ok(())
}
