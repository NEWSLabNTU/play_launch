//! Static checker for launch manifest contracts.
//!
//! Builds a dataflow graph from a [`Manifest`] and runs validation rules
//! to detect inconsistencies (wiring, QoS, rate, latency budgets, drops).

pub mod check;
pub mod emit;
pub mod graph;
pub mod rules;

pub use check::{
    CheckContext, CheckResult, Diagnostic, Severity, run_checks, run_checks_with_spans,
};
