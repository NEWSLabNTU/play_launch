//! Shared, portable scheduling spec for launch-based systems.
//!
//! Consumed by both `play_launch` (Linux RT, target `posix`) and `nano-ros`
//! (RTOS targets). The generic layer (tier class, deadline, binding) is
//! platform-independent and carries no priority numbers; per-platform
//! placement lives in `[tiers.<name>.<target>]` sub-tables.

pub mod parse;
pub mod resolve;
pub mod types;

pub use parse::parse_system_sched;
pub use resolve::{DEFAULT_TIER, ResolvedTier, ResolvedTierTable, SchedError, SchedNode, resolve};
pub use types::{AssignRule, SystemSched, TierDef, TierPlatformSpec};

// --- v2 (derived) model: platform-file schema, SchedMapper, legacy bridge,
// conflict-semantics validation (Phase 41.1). Additive: everything above
// keeps compiling and passing unchanged. ---
pub mod bridge;
pub mod mapper;
pub mod platform;
pub mod validate;

// --- Phase 44.3: chain-aware mapper (clock-segmented chains). Additive:
// v1/v2/legacy-bridge APIs above keep compiling and passing unchanged. ---
pub mod chain;
pub mod chain_aware_mapper;

pub use bridge::parse_legacy_toml;
pub use chain::{
    ChainAwareDetail, ChainElement, ChainSemantics, EffectiveTrigger, MapDiagnostics, MapWarning,
    MapperPath, ResolvedChain, SegmentNode,
};
pub use chain_aware_mapper::{ChainAwareMapper, RankItem, RankedPlan, chain_aware_rank};
pub use mapper::{
    Criticality, MapError, MapperInput, MapperNode, MapperRegistry, PlatformFacts, SchedMapper,
    SchedPlan,
};
pub use platform::{
    POSIX_RT_PRIORITY_MAX, POSIX_RT_PRIORITY_MIN, PlatformError, PlatformFile,
    PlatformOverrideEntry, PlatformResources, PosixOverride, PosixResources, PriorityBand,
    parse_platform_file, parse_platform_file_yaml,
};
pub use validate::{
    BandViolation, Contradiction, band_violations, deadline_priority_contradictions,
    rate_priority_contradictions,
};
