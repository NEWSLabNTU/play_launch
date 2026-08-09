//! Source-annotated diagnostic output using codespan-reporting.
//!
//! Renders diagnostics with source excerpts, line numbers, and color
//! when source text and byte spans are available.

use crate::check::{CheckResult, Severity};
use codespan_reporting::{
    diagnostic::{self as cs, Label},
    files::SimpleFiles,
    term,
    term::termcolor::{ColorChoice, StandardStream, WriteColor},
};

/// Emit diagnostics to stderr with source-annotated output.
///
/// Falls back to plain text for diagnostics without spans.
pub fn emit_diagnostics(result: &CheckResult, filename: &str, source: &str) {
    let writer = StandardStream::stderr(ColorChoice::Auto);
    emit_diagnostics_to(&mut writer.lock(), result, filename, source);
}

/// Emit diagnostics to an arbitrary writer (useful for testing).
pub fn emit_diagnostics_to(
    writer: &mut dyn WriteColor,
    result: &CheckResult,
    filename: &str,
    source: &str,
) {
    let mut files = SimpleFiles::new();
    let file_id = files.add(filename, source);
    let config = term::Config::default();

    for diag in &result.diagnostics {
        let severity = match diag.severity {
            Severity::Error => cs::Severity::Error,
            Severity::Warning => cs::Severity::Warning,
            Severity::Info => cs::Severity::Note,
        };

        let mut cs_diag = cs::Diagnostic::new(severity)
            .with_code(&diag.rule_id)
            .with_message(&diag.message);

        if let Some(ref span) = diag.span {
            // Clamp span to source bounds
            let start = span.start.min(source.len());
            let end = span.end.min(source.len()).max(start);
            cs_diag = cs_diag.with_labels(vec![
                Label::primary(file_id, start..end).with_message(&diag.path),
            ]);
        } else {
            // No span — add a note with the YAML path
            cs_diag = cs_diag.with_notes(vec![format!("at {}", diag.path)]);
        }

        let _ = term::emit(writer, &config, &files, &cs_diag);
    }

    // Summary line
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
        let _ = writeln!(writer, "{errors} error(s), {warnings} warning(s)");
    }
}

/// Render diagnostics to a string (for testing).
pub fn emit_diagnostics_to_string(result: &CheckResult, filename: &str, source: &str) -> String {
    use codespan_reporting::term::termcolor::Buffer;
    let mut buf = Buffer::no_color();
    emit_diagnostics_to(&mut buf, result, filename, source);
    String::from_utf8_lossy(buf.as_slice()).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::run_checks_with_spans;
    use ros_launch_manifest_types::parse_manifest_str_with_spans;

    #[test]
    fn test_emit_qos_violation_has_line_number() {
        let yaml = "version: 1\ntopics:\n  pointcloud:\n    type: PointCloud2\n    qos:\n      reliability: maybe\n";
        let parsed = parse_manifest_str_with_spans(yaml).unwrap();
        let result = run_checks_with_spans(&parsed.manifest, parsed.spans);

        assert!(result.has_errors());
        let output = emit_diagnostics_to_string(&result, "test.yaml", yaml);

        assert!(
            output.contains("test.yaml"),
            "should contain filename: {output}"
        );
        assert!(
            output.contains("qos-compat"),
            "should contain rule id: {output}"
        );
        assert!(
            output.contains("reliability"),
            "should contain key name: {output}"
        );
    }

    #[test]
    fn test_emit_rate_hierarchy_has_span() {
        let yaml = "version: 1\nnodes:\n  driver:\n    pub:\n      pointcloud:\n        min_rate_hz: 5\ntopics:\n  pointcloud:\n    type: PointCloud2\n    pub: [driver/pointcloud]\n    rate_hz: 10\n";
        let parsed = parse_manifest_str_with_spans(yaml).unwrap();
        let result = run_checks_with_spans(&parsed.manifest, parsed.spans);

        assert!(result.has_errors());
        let output = emit_diagnostics_to_string(&result, "test.yaml", yaml);

        assert!(output.contains("rate-hierarchy"), "output: {output}");
        assert!(output.contains("test.yaml"), "output: {output}");
    }

    #[test]
    fn test_emit_no_span_fallback() {
        // Use run_checks (no spans) — diagnostics should still render
        let yaml = "version: 1\ntopics:\n  pc:\n    type: PointCloud2\n    qos:\n      reliability: maybe\n";
        let m = ros_launch_manifest_types::parse_manifest_str(yaml).unwrap();
        let result = crate::check::run_checks(&m);

        let output = emit_diagnostics_to_string(&result, "test.yaml", yaml);
        assert!(output.contains("qos-compat"), "output: {output}");
        // Without spans, should show "at" note instead of source excerpt
        assert!(output.contains("at "), "should have path note: {output}");
    }

    #[test]
    fn test_emit_clean_manifest_empty() {
        let yaml = "version: 1\nnodes:\n  a:\n    pub: [x]\n";
        let m = ros_launch_manifest_types::parse_manifest_str(yaml).unwrap();
        let result = crate::check::run_checks(&m);

        let output = emit_diagnostics_to_string(&result, "test.yaml", yaml);
        // No diagnostics means no output
        assert!(
            !output.contains("error"),
            "output should be clean: {output}"
        );
    }
}
