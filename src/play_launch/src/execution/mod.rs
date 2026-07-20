pub mod context;
pub mod node_cmdline;
pub mod rt_helper_client;
pub mod sched_apply;
pub mod sched_plan;

/// Phase 46.3b — the static equivalence gate proving the model-sourced
/// spawn path (`context::prepare_*_contexts_from_model`) produces the same
/// argv/env/params-file-content as the record-sourced path
/// (`context::prepare_*_contexts`) for the same launch. See
/// `.superpowers/sdd/p46-w3-analysis.md` / the 46.3b report.
#[cfg(test)]
mod spawn_equivalence_test;
