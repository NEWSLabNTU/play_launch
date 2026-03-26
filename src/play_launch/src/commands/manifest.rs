//! `play_launch check` — parse a launch file and check manifest contracts.

use crate::{cli::options::CheckArgs, ros::manifest_loader};
use eyre::Result;
use ros_launch_manifest_check::{
    Severity, emit::diagnostic::emit_diagnostics, run_checks_with_spans,
};
use ros_launch_manifest_types::parse_manifest_str_with_spans;

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

    // Render diagnostics
    render_scope_diagnostics(&index, &args.format)?;

    if index.total_errors > 0 {
        std::process::exit(1);
    }

    Ok(())
}

/// Render diagnostics for all scopes in the index.
fn render_scope_diagnostics(index: &manifest_loader::ManifestIndex, format: &str) -> Result<()> {
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

        if format == "json" {
            print_scope_json(&resolved.diagnostics, &label)?;
        } else if !resolved.diagnostics.is_empty() {
            // Re-run checks with spans to get a proper CheckResult for the emitter
            if let Ok(parsed) = parse_manifest_str_with_spans(&resolved.source) {
                let check_result = run_checks_with_spans(&parsed.manifest, parsed.spans);
                emit_diagnostics(&check_result, &label, &resolved.source);
            } else {
                for diag in &resolved.diagnostics {
                    eprintln!("{diag}");
                }
            }
        }
    }

    // Summary
    let clean_count = index
        .manifests
        .values()
        .filter(|m| m.diagnostics.iter().all(|d| d.severity != Severity::Error))
        .count();
    let error_count = index.manifests.len() - clean_count;
    eprintln!(
        "\n{} manifest(s) checked: {} clean, {} with errors ({} total errors, {} warnings)",
        index.manifests.len(),
        clean_count,
        error_count,
        index.total_errors,
        index.total_warnings,
    );

    Ok(())
}

fn print_scope_json(
    diagnostics: &[ros_launch_manifest_check::Diagnostic],
    label: &str,
) -> Result<()> {
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
