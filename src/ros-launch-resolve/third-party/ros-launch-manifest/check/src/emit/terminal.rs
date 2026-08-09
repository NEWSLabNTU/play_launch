//! Terminal output for diagnostics.

use crate::check::{CheckResult, Severity};

/// Print diagnostics to stderr.
pub fn print_diagnostics(result: &CheckResult) {
    for diag in &result.diagnostics {
        let prefix = match diag.severity {
            Severity::Error => "error",
            Severity::Warning => "warning",
            Severity::Info => "info",
        };
        eprintln!(
            "{prefix}[{}]: {} (at {})",
            diag.rule_id, diag.message, diag.path
        );
    }
    let errors = result
        .diagnostics
        .iter()
        .filter(|d| d.severity == Severity::Error)
        .count();
    let warnings = result
        .diagnostics
        .iter()
        .filter(|d| d.severity == Severity::Warning)
        .count();
    if errors > 0 || warnings > 0 {
        eprintln!("{errors} error(s), {warnings} warning(s)");
    }
}
