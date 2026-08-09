//! Check context and diagnostic types.

use crate::{graph::DataflowGraph, rules};
use ros_launch_manifest_types::{Manifest, SpanIndex};
use std::ops::Range;

/// Severity level for diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Info,
    Warning,
    Error,
}

impl std::fmt::Display for Severity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Severity::Info => write!(f, "info"),
            Severity::Warning => write!(f, "warning"),
            Severity::Error => write!(f, "error"),
        }
    }
}

/// A diagnostic message from a validation rule.
#[derive(Debug, Clone)]
pub struct Diagnostic {
    pub rule_id: String,
    pub severity: Severity,
    pub message: String,
    /// YAML path where the issue was found (e.g., "nodes.ndt.paths.localization").
    pub path: String,
    /// Byte range in the source text (if source spans are available).
    pub span: Option<Range<usize>>,
}

impl std::fmt::Display for Diagnostic {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "[{}] {}: {} (at {})",
            self.rule_id, self.severity, self.message, self.path
        )
    }
}

/// Accumulates diagnostics during checking.
pub struct CheckContext {
    pub diagnostics: Vec<Diagnostic>,
    /// Span index for resolving YAML paths to byte ranges.
    pub spans: Option<SpanIndex>,
}

impl CheckContext {
    pub fn new() -> Self {
        Self {
            diagnostics: Vec::new(),
            spans: None,
        }
    }

    /// Create a context with span resolution support.
    pub fn with_spans(spans: SpanIndex) -> Self {
        Self {
            diagnostics: Vec::new(),
            spans: Some(spans),
        }
    }

    pub fn emit(&mut self, rule_id: &str, severity: Severity, path: &str, message: String) {
        let span = self.spans.as_ref().and_then(|idx| idx.get(path));
        self.diagnostics.push(Diagnostic {
            rule_id: rule_id.to_string(),
            severity,
            message,
            path: path.to_string(),
            span,
        });
    }

    pub fn error(&mut self, rule_id: &str, path: &str, message: String) {
        self.emit(rule_id, Severity::Error, path, message);
    }

    pub fn warning(&mut self, rule_id: &str, path: &str, message: String) {
        self.emit(rule_id, Severity::Warning, path, message);
    }

    pub fn has_errors(&self) -> bool {
        self.diagnostics
            .iter()
            .any(|d| d.severity == Severity::Error)
    }
}

impl Default for CheckContext {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of running all checks.
pub struct CheckResult {
    pub diagnostics: Vec<Diagnostic>,
    pub graph: DataflowGraph,
}

impl CheckResult {
    pub fn has_errors(&self) -> bool {
        self.diagnostics
            .iter()
            .any(|d| d.severity == Severity::Error)
    }

    pub fn errors(&self) -> impl Iterator<Item = &Diagnostic> {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
    }

    pub fn warnings(&self) -> impl Iterator<Item = &Diagnostic> {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Warning)
    }
}

/// Run all validation rules on a manifest.
pub fn run_checks(manifest: &Manifest) -> CheckResult {
    let graph = DataflowGraph::build(manifest);
    let mut ctx = CheckContext::new();

    for rule in rules::default_rules() {
        rule.check(manifest, &graph, &mut ctx);
    }

    CheckResult {
        diagnostics: ctx.diagnostics,
        graph,
    }
}

/// Run all validation rules with span resolution support.
pub fn run_checks_with_spans(manifest: &Manifest, spans: SpanIndex) -> CheckResult {
    let graph = DataflowGraph::build(manifest);
    let mut ctx = CheckContext::with_spans(spans);

    for rule in rules::default_rules() {
        rule.check(manifest, &graph, &mut ctx);
    }

    CheckResult {
        diagnostics: ctx.diagnostics,
        graph,
    }
}
