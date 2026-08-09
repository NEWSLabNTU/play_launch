//! On-disk scheduling schema (TOML).

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// Top-level system scheduling document.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SystemSched {
    /// Symbolic tiers keyed by name.
    #[serde(default)]
    pub tiers: BTreeMap<String, TierDef>,
    /// Sparse node→tier binding rules.
    #[serde(default)]
    pub assign: Vec<AssignRule>,
}

/// `[tiers.<name>]` — the generic (portable) head plus per-platform sub-tables.
/// The head carries NO priority number; placement lives in the sub-tables.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TierDef {
    /// `best_effort | real_time | time_triggered | interrupt`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub class: Option<String>,
    /// Generic relative deadline (µs); a platform sub-table may tighten it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deadline_us: Option<u64>,
    /// Callback period (µs) for periodic / time_triggered.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub period_us: Option<u64>,
    /// Execution-time budget (µs) — EDF/sporadic.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub budget_us: Option<u64>,
    /// `ignore | warn | skip | fault`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deadline_policy: Option<String>,
    /// Executor spin period (µs).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spin_period_us: Option<u64>,

    // Per-platform placement sub-tables. `posix` is Linux RT.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub posix: Option<TierPlatformSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub freertos: Option<TierPlatformSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub zephyr: Option<TierPlatformSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub threadx: Option<TierPlatformSpec>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nuttx: Option<TierPlatformSpec>,
}

impl TierDef {
    /// Pick the placement sub-table for a resolve target.
    pub fn platform(&self, target: &str) -> Option<&TierPlatformSpec> {
        match target {
            "posix" | "native" => self.posix.as_ref(),
            "freertos" => self.freertos.as_ref(),
            "zephyr" => self.zephyr.as_ref(),
            "threadx" => self.threadx.as_ref(),
            "nuttx" => self.nuttx.as_ref(),
            _ => None,
        }
    }
}

/// `[tiers.<name>.<target>]` — concrete per-platform placement. Same shape on
/// every platform. `priority` is `i64` to admit Zephyr negative coop priorities.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TierPlatformSpec {
    pub priority: i64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stack_bytes: Option<u32>,
    /// CPU core to pin to (SMP); `None` ⇒ unpinned.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub core: Option<u32>,
    /// POSIX scheduler class (e.g. `"SCHED_FIFO"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sched_class: Option<String>,
    /// ThreadX preemption threshold (ignored elsewhere).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preempt_threshold: Option<i64>,
    /// Per-platform deadline override (µs), tighter than the generic head.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deadline_us: Option<u64>,
    /// Per-platform sporadic execution budget (µs) — nano-ros phase-296
    /// W5.9: lets ONE platform's kernel sporadic server (NuttX
    /// SCHED_SPORADIC) engage without flipping every other platform's
    /// executor into cooperative Sporadic gating (a GENERIC head budget
    /// applies everywhere). Same sub-table-override precedent as
    /// `deadline_us`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub budget_us: Option<u64>,
    /// Per-platform sporadic replenishment period (µs) — pairs with
    /// `budget_us` (both required for a sporadic policy).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub period_us: Option<u64>,
    /// Per-platform round-robin time slice (µs) — nano-ros #0266: requests
    /// time-slicing among same-priority tiers. ThreadX-only today (its
    /// `tx_thread_create` takes a per-thread slice); ignored elsewhere.
    /// Same sub-table-scoped precedent as `preempt_threshold`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub time_slice_us: Option<u64>,
}

/// A sparse binding rule. A node matches if it is named in `nodes` OR falls
/// under `scope`. Explicit `nodes` matches win over `scope` matches.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AssignRule {
    pub tier: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub nodes: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scope: Option<String>,
}
