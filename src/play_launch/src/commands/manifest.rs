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

    // Load and check manifests: overlay > provider sidecar. The provider
    // channel is on by default, so `check` works with no manifest flags at
    // all whenever the launch file ships sidecar contracts.
    let sources = args.contract_sources();
    let index = manifest_loader::load_manifests(&dump, &sources)?;

    // Export the declared causal graph (Phase 42.1). This is an export, not
    // a validation step — it runs regardless of rule filters/errors below
    // and doesn't affect the exit code.
    if let Some(export_path) = &args.export_graph {
        crate::ros::causal_graph::export_to_file(&index, export_path)?;
        eprintln!("Exported causal graph to {}", export_path.display());
    }

    // Optional: validate the shared scheduling spec (Linux = validate-now).
    // Loaded after manifests so contract-aware mappers (rate_monotonic,
    // deadline_monotonic) can extract timing facts from `index`.
    //
    // Resolve the platform file through the same channels as contracts
    // (Phase 41.3): explicit `--sched <path>` > overlay > provider sidecar.
    // `sources.overlay` is already the discovered root (see
    // `CheckArgs::contract_sources`), so both channels agree on which root
    // is in play.
    let resolved_platform = crate::ros::sched_loader::resolve_platform_file(
        &dump,
        args.sched.as_deref(),
        sources.overlay.as_deref(),
        sources.provider,
        &args.target,
    );
    if let Some(resolved) = &resolved_platform {
        eprintln!(
            "Scheduling platform file [{}]: {}",
            resolved.channel,
            resolved.path.display()
        );
        let derived = crate::ros::sched_loader::check_sched(
            &dump,
            Some(&index),
            &resolved.path,
            &args.target,
        )?;
        if args.explain {
            crate::ros::sched_loader::print_explain(&derived, resolved, Some(&index));
        }
    } else if args.explain {
        eprintln!(
            "note: --explain has no effect without a resolved scheduling platform file \
             (pass --sched <path>, or ship one via the overlay/provider channels)"
        );
    }

    if index.manifests.is_empty() {
        eprintln!(
            "No manifests found (overlay={:?}, provider={})",
            sources.overlay, sources.provider
        );
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
        print_diagnostics_json(&filtered, "<cross-scope>", None)?;
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

    let mut overlay_count = 0usize;
    let mut provider_count = 0usize;
    for resolved in index.manifests.values() {
        match resolved.channel {
            manifest_loader::ContractChannel::Overlay => overlay_count += 1,
            manifest_loader::ContractChannel::Provider => provider_count += 1,
        }
    }
    eprintln!(
        "{} contract(s): {} overlay, {} provider",
        index.manifests.len(),
        overlay_count,
        provider_count,
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
