//! Load + validate the shared scheduling spec against a parsed launch dump.
//!
//! Linux is "validate now, apply later": we resolve for the `posix` target and
//! report, but do not (yet) apply `sched_setscheduler`/affinity to processes.

use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    fmt,
    path::{Path, PathBuf},
};

use eyre::Result;
use ros_launch_manifest_sched::{
    ChainAwareDetail, ChainElement, Contradiction, MapError, MapWarning, MapperNode,
    MapperRegistry, PlatformResources, ResolvedChain, ResolvedTier, ResolvedTierTable, SchedNode,
    band_violations, deadline_priority_contradictions, parse_platform_file,
    rate_priority_contradictions,
};
use tracing::{debug, info};

use crate::{
    execution::sched_apply::SchedApplyMode,
    ros::{
        launch_dump::LaunchDump,
        manifest_loader::{ManifestIndex, contract_stem},
        sched_derive::mapper_input_from_dump,
    },
};

/// The mappers `MapperRegistry::with_builtins()` registers — hardcoded here
/// (rather than asking the registry to enumerate itself, which its wave-1 API
/// doesn't expose) purely so an "unknown mapper" error can list the known
/// names.
const KNOWN_MAPPERS: &[&str] = &[
    "manual",
    "rate_monotonic",
    "deadline_monotonic",
    "chain_aware",
];

/// One record in the launch dump that is a schedulable unit (regular node,
/// container, or composable/load_node — all of which are separate
/// fork+exec'd processes under isolated container mode), carrying enough
/// identity to build both the resolver's [`SchedNode`] view and the derive
/// pipeline's `MapperNode` view (which additionally needs the scope id to
/// look up contract facts in a [`ManifestIndex`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScheduledRecord {
    /// Fully-qualified node name.
    pub fqn: String,
    /// Effective namespace (own namespace, falling back to scope namespace).
    pub scope_ns: String,
    /// The record's own scope id (for contract-fact lookup in a
    /// [`ManifestIndex`], which is keyed by the same scope ids). `None` when
    /// the record has no scope (e.g. the synthetic single-node dump built by
    /// `play_launch run`).
    pub scope_id: Option<usize>,
    /// The record's bare (unqualified) name, as declared in the launch file
    /// — matches the key manifests use for `nodes.<name>` contract entries.
    pub bare_name: String,
}

/// Effective namespace: the record's own namespace if non-empty, else the scope's ns.
fn effective_ns(own: Option<&str>, scope: Option<usize>, by_id: &HashMap<usize, &str>) -> String {
    let own = own.map(str::trim).filter(|s| !s.is_empty() && *s != "/");
    let ns = own
        .or_else(|| scope.and_then(|id| by_id.get(&id).copied()))
        .unwrap_or("/");
    // normalize to a single leading slash, no trailing slash
    let t = ns.trim_matches('/');
    if t.is_empty() {
        "/".to_string()
    } else {
        format!("/{t}")
    }
}

/// Join a namespace with a bare node name into a fully-qualified name.
pub(crate) fn join_fqn(ns: &str, bare: &str) -> String {
    if bare.starts_with('/') {
        bare.to_string()
    } else if ns == "/" || ns.is_empty() {
        format!("/{bare}")
    } else {
        format!("{}/{bare}", ns.trim_end_matches('/'))
    }
}

/// Core of `fqn_for`, parameterized on an already-built scope-id -> namespace
/// map so repeated calls (e.g. once per record) don't each rebuild it.
fn fqn_with(
    by_id: &HashMap<usize, &str>,
    namespace: Option<&str>,
    bare: &str,
    scope: Option<usize>,
) -> String {
    let ns = effective_ns(namespace, scope, by_id);
    join_fqn(&ns, bare)
}

/// Fully-qualified name for a record, using its own namespace (fallback: scope ns).
///
/// This is the single source of truth for FQN computation: both
/// `sched_nodes_from_dump` (building the resolver's node view) and the
/// actor-wiring build sites (looking up a member's resolved tier in
/// `SchedPlan`) must agree on the same FQN for the same record, or lookups
/// silently miss. Callers pass the record's own `namespace` (may be `None`
/// or empty), its bare `name`, and its `scope` id; the scope's namespace is
/// used as a fallback when the record has no namespace of its own.
///
/// Builds the scope-id -> namespace map once per call; callers that need the
/// FQN for many records from the same dump (e.g. `sched_nodes_from_dump`)
/// should build the map once themselves and call `fqn_with` directly instead.
pub fn fqn_for(
    dump: &LaunchDump,
    namespace: Option<&str>,
    bare_name: &str,
    scope: Option<usize>,
) -> String {
    let by_id: HashMap<usize, &str> = dump.scopes.iter().map(|s| (s.id, s.ns.as_str())).collect();
    fqn_with(&by_id, namespace, bare_name, scope)
}

/// Flatten a launch dump into the resolver's dependency-free node view.
/// Regular nodes, containers, and composable nodes are all scheduled units on Linux:
/// under isolated container mode each composable is a fork+exec process.
///
/// Superseded as of the v2 derive pipeline (Phase 41.2) by
/// [`scheduled_records_from_dump`], which carries the extra identity
/// (`scope_id`/`bare_name`) the derive stage needs — kept as a lower-level
/// utility (and exercised directly by this module's own tests) since it's
/// still the simplest `Vec<SchedNode>` view for anything that wants to call
/// `ros_launch_manifest_sched::resolve` directly.
#[allow(dead_code)]
pub fn sched_nodes_from_dump(dump: &LaunchDump) -> Vec<SchedNode> {
    // Build scope-id → namespace map once; avoids O(N*M) per-record lookups.
    let by_id: HashMap<usize, &str> = dump.scopes.iter().map(|s| (s.id, s.ns.as_str())).collect();

    let mut out = Vec::new();

    for n in &dump.node {
        let bare = n
            .name
            .clone()
            .or_else(|| n.exec_name.clone())
            .unwrap_or_default();
        if bare.is_empty() {
            continue;
        }
        let ns = effective_ns(n.namespace.as_deref(), n.scope, &by_id);
        out.push(SchedNode {
            name: fqn_with(&by_id, n.namespace.as_deref(), &bare, n.scope),
            scope: ns,
        });
    }

    for c in &dump.container {
        if c.name.is_empty() {
            continue;
        }
        let ns = effective_ns(Some(c.namespace.as_str()), c.scope, &by_id);
        out.push(SchedNode {
            name: fqn_with(&by_id, Some(c.namespace.as_str()), &c.name, c.scope),
            scope: ns,
        });
    }

    for lc in &dump.load_node {
        if lc.node_name.is_empty() {
            continue;
        }
        // Under isolated container mode each composable is a fork+exec process,
        // so it is a schedulable unit and must be selectable.
        let ns = effective_ns(Some(lc.namespace.as_str()), lc.scope, &by_id);
        out.push(SchedNode {
            name: fqn_with(&by_id, Some(lc.namespace.as_str()), &lc.node_name, lc.scope),
            scope: ns,
        });
    }

    out
}

/// Flatten a launch dump into [`ScheduledRecord`]s — the same schedulable
/// units as [`sched_nodes_from_dump`] (regular nodes, containers, composable
/// nodes), but additionally carrying `scope_id` and `bare_name` so the derive
/// pipeline ([`crate::ros::sched_derive`]) can look up per-node contract
/// facts in a [`ManifestIndex`] (keyed by the same scope ids and declared
/// node names as the launch dump).
pub fn scheduled_records_from_dump(dump: &LaunchDump) -> Vec<ScheduledRecord> {
    let by_id: HashMap<usize, &str> = dump.scopes.iter().map(|s| (s.id, s.ns.as_str())).collect();

    let mut out = Vec::new();

    for n in &dump.node {
        let bare = n
            .name
            .clone()
            .or_else(|| n.exec_name.clone())
            .unwrap_or_default();
        if bare.is_empty() {
            continue;
        }
        let scope_ns = effective_ns(n.namespace.as_deref(), n.scope, &by_id);
        out.push(ScheduledRecord {
            fqn: fqn_with(&by_id, n.namespace.as_deref(), &bare, n.scope),
            scope_ns,
            scope_id: n.scope,
            bare_name: bare,
        });
    }

    for c in &dump.container {
        if c.name.is_empty() {
            continue;
        }
        let scope_ns = effective_ns(Some(c.namespace.as_str()), c.scope, &by_id);
        out.push(ScheduledRecord {
            fqn: fqn_with(&by_id, Some(c.namespace.as_str()), &c.name, c.scope),
            scope_ns,
            scope_id: c.scope,
            bare_name: c.name.clone(),
        });
    }

    for lc in &dump.load_node {
        if lc.node_name.is_empty() {
            continue;
        }
        let scope_ns = effective_ns(Some(lc.namespace.as_str()), lc.scope, &by_id);
        out.push(ScheduledRecord {
            fqn: fqn_with(&by_id, Some(lc.namespace.as_str()), &lc.node_name, lc.scope),
            scope_ns,
            scope_id: lc.scope,
            bare_name: lc.node_name.clone(),
        });
    }

    out
}

/// FQNs of composable (load_node) records — the units v1 cannot schedule.
///
/// Not yet called outside tests + `sched_plan` (actor wiring is a later
/// phase-38 task); allowed dead in non-consumer builds.
#[allow(dead_code)]
pub fn composable_fqns(dump: &LaunchDump) -> std::collections::HashSet<String> {
    let by_id: HashMap<usize, &str> = dump.scopes.iter().map(|s| (s.id, s.ns.as_str())).collect();

    dump.load_node
        .iter()
        .filter(|lc| !lc.node_name.is_empty())
        .map(|lc| {
            let ns = effective_ns(Some(lc.namespace.as_str()), lc.scope, &by_id);
            join_fqn(&ns, &lc.node_name)
        })
        .collect()
}

/// Result of the v2 derive→override→validate pipeline (design §5, §6):
/// the final per-node plan (one [`ResolvedTier`] per node, except the
/// synthesized default tier which stays grouped — see
/// [`flatten_to_one_tier_per_node`]), plus which target/mapper produced it
/// and any non-fatal warnings collected along the way (band clamps,
/// unknown-override names, rate/deadline-vs-priority contradictions).
#[derive(Debug)]
pub struct DerivedSchedPlan {
    pub plan: ResolvedTierTable,
    pub target: String,
    pub mapper: String,
    pub warnings: Vec<String>,
    /// Per-node fqn -> why it ended up with its final priority/class (Phase
    /// 41.4, `--explain`). Every member of every tier in `plan` has an entry.
    pub provenance: BTreeMap<String, Provenance>,
    /// Every node FQN that participates in at least one resolved `chains:`
    /// declaration (Phase 44.4) — populated straight from
    /// `MapperInput.chains`, independent of which mapper is selected
    /// (`chain_aware` or otherwise; "harmless" per the brief). Consumed at
    /// launch/replay time by the container co-location warning
    /// (`execution::sched_plan::chain_container_colocation_warnings`) —
    /// `derive_sched_plan`/`check` don't know the runtime `--container-mode`
    /// flag, so the warning itself can't be computed here (see that
    /// function's doc comment for the placement decision).
    pub chain_member_nodes: BTreeSet<String>,
    /// Number of raw rate/deadline-vs-priority contradictions
    /// (`ros_launch_manifest_sched::validate::scan_contradictions`) that
    /// [`should_suppress_contradiction`] filtered out as chain-drain-order-
    /// intended or override-derivative noise (45.1b) — surfaced in the
    /// structured summary line (45.1c) so suppression is visible, not silent.
    pub suppressed_contradictions: usize,
    /// Phase 45.4 — the resolved chains this derivation fed to the mapper
    /// (`MapperInput::chains`, built by `sched_derive::resolve_chains`),
    /// carried straight through so `model_builder` can embed them verbatim
    /// in `execution.sched.chains` — the model's SHARED STRUCTURE (design
    /// `docs/design/system-model-sched-ssot.md`). Never re-derived by the
    /// model builder: this is the single derivation call's output.
    pub chains: Vec<ResolvedChain>,
    /// Phase 45.4 — every schedulable node's extracted timing/criticality/
    /// path facts (`MapperInput::nodes`), carried through so `model_builder`
    /// can translate each into a `model::NodeSchedRequirement` for
    /// `execution.sched.requirements` — the model's SHARED STRUCTURE
    /// (per-(node,path) trigger/deadline/budget + per-node criticality).
    pub nodes: Vec<MapperNode>,
    /// Phase 45.4 — per-(node, path) `chain_aware` PiCAS ranks
    /// (`MapDiagnostics::details`), carried through so `model_builder` can
    /// embed them in `execution.sched.ranks` — the model's LINUX
    /// REALIZATION. Empty for mappers other than `chain_aware` (which is
    /// the only built-in that populates diagnostics `details`).
    pub ranks: Vec<ChainAwareDetail>,
    /// Phase 45.6 (review) — FQNs whose final placement was pinned by an
    /// explicit platform-file `overrides.<selector>` entry (the keys of the
    /// internal `overridden` map `build_provenance` reads to label a node
    /// `Provenance::Override`). Carried through so `model_builder` can embed
    /// it in `execution.sched.overrides` — the fact a model-sourced
    /// `--explain` needs to reproduce the fresh-derive renderer's
    /// override-vs-derived classification EXACTLY, without the
    /// priority-equality heuristic that misfires when an override coincides
    /// with the derived value.
    pub overrides: BTreeSet<String>,
}

/// Why a node ended up with its final scheduling placement (design §7
/// `--explain`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Provenance {
    /// An explicit `overrides.<selector>` entry in the platform file beat
    /// whatever the mapper derived (or promoted the node out of the
    /// non-RT default tier). `selector` is the exact key used in the
    /// platform file's `overrides:` map.
    Override { selector: String },
    /// The mapper derived this placement from a timing fact. `fact` is a
    /// human-readable description of the fact and resulting priority, e.g.
    /// `"100 Hz → prio 38"`.
    Derived { mapper: String, fact: String },
    /// The `chain_aware` mapper derived this placement (Phase 44.4): `line`
    /// is the **already-fully-formatted** provenance string from the sched
    /// crate's [`ChainAwareDetail::provenance`] (e.g. `"derived(chain_aware:
    /// sensing_to_actuation segment drain 2/3) -> prio 44"`), optionally
    /// suffixed with `" (max over N paths)"` when the node's final priority
    /// was the max over more than one of its ranked paths (design step 6 —
    /// node projection). Kept as a **separate** variant from `Derived`
    /// rather than reusing it: `ChainAwareDetail::provenance` already
    /// contains its own `derived(chain_aware: ...)` wrapper, so passing it
    /// as `Derived::fact` would double-wrap under this type's `Display`
    /// impl (`derived(chain_aware: derived(chain_aware: ...))`).
    ChainAware { line: String },
    /// No timing fact and no override — landed in the non-RT default tier.
    Default,
}

impl fmt::Display for Provenance {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Provenance::Override { selector } => write!(f, "override({selector})"),
            Provenance::Derived { mapper, fact } => write!(f, "derived({mapper}: {fact})"),
            Provenance::ChainAware { line } => write!(f, "{line}"),
            Provenance::Default => write!(f, "default (no timing facts)"),
        }
    }
}

/// `true` if `selector` names `fqn` exactly or matches its bare (last path
/// segment) name — the same vocabulary `[[assign]]`'s `nodes` selector and
/// a platform file's `overrides` keys share (mirrors the private
/// `node_selector_matches` in `ros_launch_manifest_sched::resolve`).
fn selector_matches(selector: &str, fqn: &str) -> bool {
    fqn == selector || fqn.rsplit('/').next() == Some(selector)
}

/// Explode every non-default tier into one tier per member, preserving all
/// other fields (priority/sched_class/core/class/...). The synthesized
/// default tier is left grouped, matching the convention the built-in
/// derived mappers (`rate_monotonic`/`deadline_monotonic`) already use for
/// fact-less nodes.
///
/// This gives `apply_overrides` (and the validation helpers, which key off
/// `tier.priority`/`tier.members`) a uniform one-node-per-tier shape to work
/// against, regardless of whether the plan came from a derived mapper
/// (already one-per-node) or the `manual` mapper (grouped by declared
/// tier). Downstream consumers (`execution::sched_plan::SchedPlan::build`)
/// already invert every tier's `members` into a per-FQN lookup, so this
/// reshaping is transparent to them — the resulting per-FQN knobs
/// (`priority`/`sched_class`/`core`/`tier_name`) are identical either way.
fn flatten_to_one_tier_per_node(table: &ResolvedTierTable) -> ResolvedTierTable {
    let mut tiers = Vec::with_capacity(table.tiers.len());
    for t in &table.tiers {
        if t.name == ros_launch_manifest_sched::DEFAULT_TIER {
            tiers.push(t.clone());
            continue;
        }
        for m in &t.members {
            tiers.push(ResolvedTier {
                members: vec![m.clone()],
                ..t.clone()
            });
        }
    }
    ResolvedTierTable { tiers }
}

/// Apply a platform file's explicit `overrides` on top of an
/// already-flattened (one-tier-per-node) derived plan — "override beats
/// derived, always" (design §5/§6). A selector matching no node anywhere in
/// the plan (including the default tier) is a warning, not an error (the
/// spec may simply be ahead of the launch tree, e.g. authored for a node
/// that was since removed).
fn apply_overrides(
    table: &mut ResolvedTierTable,
    overrides: &std::collections::BTreeMap<
        String,
        ros_launch_manifest_sched::PlatformOverrideEntry,
    >,
    warnings: &mut Vec<String>,
    overridden: &mut BTreeMap<String, String>,
) {
    use ros_launch_manifest_sched::PlatformOverrideEntry;

    for (selector, entry) in overrides {
        let PlatformOverrideEntry::Posix(posix) = entry else {
            // Raw (non-posix-target) override entries never reach here —
            // this pipeline only ever resolves for `target: posix`.
            continue;
        };

        if let Some(tier) = table.tiers.iter_mut().find(|t| {
            t.name != ros_launch_manifest_sched::DEFAULT_TIER
                && t.members
                    .first()
                    .is_some_and(|m| selector_matches(selector, m))
        }) {
            if let Some(p) = posix.priority {
                tier.priority = p;
            }
            if let Some(c) = posix.core {
                tier.core = Some(c);
            }
            if let Some(sc) = &posix.sched_class {
                tier.sched_class = Some(sc.clone());
            }
            if tier.class.is_none() {
                tier.class = Some("real_time".to_string());
            }
            if let Some(member) = tier.members.first() {
                overridden.insert(member.clone(), selector.clone());
            }
            continue;
        }

        // Not found in any per-node tier — check the default (non-RT)
        // tier and promote the node out of it if present there.
        let promoted = table
            .tiers
            .iter_mut()
            .find(|t| t.name == ros_launch_manifest_sched::DEFAULT_TIER)
            .and_then(|default_tier| {
                let idx = default_tier
                    .members
                    .iter()
                    .position(|m| selector_matches(selector, m))?;
                Some(default_tier.members.remove(idx))
            });

        if let Some(node_fqn) = promoted {
            // A priority-only override (no explicit sched_class) on a
            // previously non-RT node is assumed to mean "make this RT" —
            // SCHED_FIFO is the crate's own default for derived RT tiers.
            let sched_class = posix
                .sched_class
                .clone()
                .or_else(|| posix.priority.map(|_| "SCHED_FIFO".to_string()));
            overridden.insert(node_fqn.clone(), selector.clone());
            tiers_push_override(
                table,
                node_fqn.clone(),
                posix.priority.unwrap_or(0),
                sched_class,
                posix.core,
            );
            continue;
        }

        let msg = format!(
            "scheduling: override for unknown node `{selector}` (no matching node in the derived plan)"
        );
        // Collect only — the caller (check_sched / SchedPlan::build) is the
        // single authoritative surfacing point (45.1a: fixes the 2x/3x
        // duplicate-print bug where this fired here AND at every print
        // site that loops `derived.warnings`).
        warnings.push(msg);
    }
}

fn tiers_push_override(
    table: &mut ResolvedTierTable,
    node_fqn: String,
    priority: i64,
    sched_class: Option<String>,
    core: Option<u32>,
) {
    table.tiers.push(ResolvedTier {
        name: node_fqn.clone(),
        priority,
        sched_class,
        class: Some("real_time".to_string()),
        core,
        members: vec![node_fqn],
        ..Default::default()
    });
}

/// Every node FQN that participates in at least one of `chains`' elements
/// (Phase 44.4 membership fact). Shared by two call sites reading the exact
/// same `ResolvedChain`/`ChainElement` shape:
///
/// - [`derive_sched_plan`], walking a freshly-built `MapperInput::chains`;
/// - [`crate::execution::sched_plan::SchedPlan::from_model`] (Phase
///   45.5), walking the model's already-resolved
///   `execution.sched.chains` — the SAME type, embedded verbatim
///   (`docs/design/system-model-sched-ssot.md` "Type sharing"), so no
///   second implementation (and no mapper re-run) is needed to reconstruct
///   membership on the model-reader path.
pub fn chain_member_nodes_from_chains(chains: &[ResolvedChain]) -> BTreeSet<String> {
    chains
        .iter()
        .flat_map(|c| c.elements.iter())
        .flat_map(|e| -> Vec<String> {
            match e {
                ChainElement::Segment {
                    nodes_in_topo_order,
                } => nodes_in_topo_order
                    .iter()
                    .map(|sn| sn.node.clone())
                    .collect(),
                ChainElement::Boundary { node, .. } => vec![node.clone()],
            }
        })
        .collect()
}

/// The v2 derive→override→validate pipeline (design §5, §6). Parses a
/// platform file (`.yaml` v2 schema or legacy `.toml`, dispatched by
/// extension — see `ros_launch_manifest_sched::parse_platform_file`),
/// selects the named mapper, derives a plan from launch+contract facts,
/// applies explicit overrides, then validates:
///
/// - **band violations** (only when `resources.rt_priority_band` is set):
///   `SchedApplyMode::Strict` errors, `Warn`/`Off` clamp-and-warn;
/// - **rate/deadline-vs-priority contradictions**: always reported as
///   warnings, never fatal (a hand override can legitimately diverge from
///   the contract's implied order — the operator should just see it).
///
/// `index` is the resolved contract index (if any manifests were loaded) —
/// `None` means every node gets a bare `MapperNode` (no timing facts), which
/// built-in mappers already default to the non-RT tier.
pub fn derive_sched_plan(
    dump: &LaunchDump,
    index: Option<&ManifestIndex>,
    platform_path: &Path,
    requested_target: &str,
    mode: SchedApplyMode,
) -> Result<DerivedSchedPlan> {
    let file = parse_platform_file(platform_path).map_err(|e| {
        eyre::eyre!(
            "failed to parse platform file {}: {e}",
            platform_path.display()
        )
    })?;

    if file.target != requested_target {
        eyre::bail!(
            "platform file {} targets `{}`, but --target requested `{}`",
            platform_path.display(),
            file.target,
            requested_target,
        );
    }

    let registry = MapperRegistry::with_builtins();
    let mapper = registry.get(&file.mapper).ok_or_else(|| {
        eyre::eyre!(
            "unknown mapper `{}` in {} (known mappers: {})",
            file.mapper,
            platform_path.display(),
            KNOWN_MAPPERS.join(", "),
        )
    })?;

    let input = mapper_input_from_dump(dump, index, file.legacy.clone());

    // Every node referenced by a resolved `chains:` declaration (44.4) —
    // independent of the selected mapper (only `chain_aware` acts on
    // `input.chains` for ranking, but chain *membership* is a pure data
    // fact, useful to every mapper/consumer — "harmless" per the brief).
    // Phase 45.5: this is the SAME walk `SchedPlan::from_model` now runs
    // over `execution.sched.chains` (the identical `ResolvedChain`/
    // `ChainElement` shape, embedded verbatim) — factored out so there is
    // one implementation for both the fresh-derive and model-reader paths.
    let chain_member_nodes = chain_member_nodes_from_chains(&input.chains);

    // Defensive FQN-consistency check (44.4 review, Important-3): a chain
    // segment resolves node identity from the CONTRACT side
    // (`manifest_loader::qualify_name`: scope namespace + bare name), while
    // the plan's node set comes from the LAUNCH-DUMP side
    // (`scheduled_records_from_dump`: node's own `namespace=` attribute
    // wins over scope namespace). These diverge for nodes using a bare
    // `namespace=` attribute outside a `<group>`/`<include>` boundary —
    // and a chain-resolved FQN missing from the plan's node set means the
    // chain_aware mapper would emit a phantom tier no process ever matches
    // (the real node silently falls to the default tier). Warn loudly
    // instead of letting that be a silent no-op. Full FQN unification is a
    // tracked follow-up (W4 report, "notes for W5").
    let mut warnings = Vec::new();
    {
        let plan_node_set: BTreeSet<&str> = input.nodes.iter().map(|n| n.name.as_str()).collect();
        for fqn in &chain_member_nodes {
            if !plan_node_set.contains(fqn.as_str()) {
                let msg = format!(
                    "scheduling: chain member `{fqn}` (contract-side FQN) does not match any \
                     schedulable node in the launch tree — its chain-derived priority would \
                     apply to no process (likely a namespace mismatch between the contract \
                     scope and the node's own `namespace=` attribute)"
                );
                // Collect only — see the comment at `apply_overrides`'s
                // unknown-selector warning (45.1a).
                warnings.push(msg);
            }
        }
    }

    // `map_with_diagnostics` (Phase 44.3, defaulted on the trait) works for
    // every mapper: built-ins fall back to `map` + empty diagnostics, only
    // `chain_aware` populates `details`/`warnings`.
    let (derived, diagnostics) = mapper
        .map_with_diagnostics(&input, &file.resources)
        .map_err(|e: MapError| {
            eyre::eyre!("mapper `{}` failed to derive a plan: {e}", file.mapper)
        })?;

    let mut plan = flatten_to_one_tier_per_node(&derived);
    // Snapshot chain members' derived priorities so an override that lowers
    // one can be called out: "overrides win, always" stands, but a pin that
    // drops a chain member below its derived rank silently defeats the
    // chain's drain-toward-sink ordering — the integrator should hear that.
    let derived_chain_prio: BTreeMap<String, i64> = plan
        .tiers
        .iter()
        .flat_map(|t| t.members.iter().map(move |m| (m.clone(), t.priority)))
        .filter(|(m, _)| chain_member_nodes.contains(m))
        .collect();
    let mut overridden: BTreeMap<String, String> = BTreeMap::new();
    apply_overrides(&mut plan, &file.overrides, &mut warnings, &mut overridden);
    // Chain members an override pinned below their chain-derived rank
    // (45.1b): every contradiction where this node is the fact-implied
    // "should lead" side is a mechanical, already-explained downstream
    // consequence of THIS warning — see `should_suppress_contradiction`.
    let mut override_pinned_below_chain_rank: BTreeSet<String> = BTreeSet::new();
    for tier in &plan.tiers {
        for member in &tier.members {
            if let Some(derived_prio) = derived_chain_prio.get(member)
                && overridden.contains_key(member)
                && tier.priority < *derived_prio
            {
                let msg = format!(
                    "override pins chain member `{member}` to priority {} (derived {}) — \
                     below its chain-derived rank; drain-toward-sink ordering within its \
                     chain is no longer guaranteed",
                    tier.priority, derived_prio
                );
                // Collect only (45.1a) — see `apply_overrides`.
                warnings.push(msg);
                override_pinned_below_chain_rank.insert(member.clone());
            }
        }
    }

    // Chain-aware mapper diagnostics (Phase 44.4 §3): `BandTooNarrow` is a
    // real resource-fit problem (the platform's `rt_priority_band` can't
    // hold the ranked classes) — gated by `mode` like `band_violations`
    // below (Strict errors, Warn/Off clamp-and-warn — the mapper itself
    // already clamped, so there's nothing left to *do* here besides
    // surface it). `ChainInfeasible` is never fatal (a chain that consumes
    // its whole budget in sampling cost alone is a modeling fact, not a
    // band-fit violation — its members simply fall through to non-chain
    // ranking, same as the `chain-sampling-feasibility` checker warning at
    // manifest-load time).
    for w in &diagnostics.warnings {
        let msg = describe_map_warning(w);
        match w {
            MapWarning::BandTooNarrow { .. } if mode == SchedApplyMode::Strict => {
                eyre::bail!("{msg}");
            }
            _ => {
                // Collect only (45.1a) — see `apply_overrides`.
                warnings.push(msg);
            }
        }
    }

    // Band violations — only meaningful with an explicit posix band.
    if let PlatformResources::Posix(res) = &file.resources
        && let Some(band) = res.rt_priority_band
    {
        let violations = band_violations(&plan, &band);
        if !violations.is_empty() {
            match mode {
                SchedApplyMode::Strict => {
                    let detail = violations
                        .iter()
                        .map(|v| {
                            format!(
                                "`{}` priority {} outside [{}, {}]",
                                v.tier, v.priority, v.band.min, v.band.max
                            )
                        })
                        .collect::<Vec<_>>()
                        .join("; ");
                    eyre::bail!("scheduling: priority band violation(s): {detail}");
                }
                SchedApplyMode::Warn | SchedApplyMode::Off => {
                    for v in &violations {
                        let clamped = v.priority.clamp(v.band.min, v.band.max);
                        let msg = format!(
                            "scheduling: `{}` priority {} outside band [{}, {}], clamping to {}",
                            v.tier, v.priority, v.band.min, v.band.max, clamped
                        );
                        // Collect only (45.1a) — see `apply_overrides`.
                        warnings.push(msg);
                        if let Some(t) = plan.tiers.iter_mut().find(|t| t.name == v.tier) {
                            t.priority = clamped;
                        }
                    }
                }
            }
        }
    }

    // Rate/deadline-vs-priority contradictions (design §6): always
    // reported, never fatal. Cite the contract file(s) that declared the
    // contradicted facts — BOTH nodes' scopes (deduped when they resolve to
    // the same contract; in a multi-include launch tree they can legitimately
    // differ) — and the platform file that produced the final priority. The
    // sched crate's `Contradiction` type carries no location (it's a pure
    // host-agnostic pairwise scan), so these file references are attached on
    // the play_launch side instead of touching the sched crate (per the
    // brief's "do NOT change the sched crate unless trivial+additive").
    let scope_by_fqn = fqn_to_scope_id(dump);
    let contract_ref_for = |fqn: &str| -> Option<String> {
        let scope_id = scope_by_fqn.get(fqn).copied().flatten()?;
        let rm = index?.manifests.get(&scope_id)?;
        Some(format!("{} [{}]", rm.contract_path.display(), rm.channel))
    };
    // Every node whose final priority was set by an explicit override (45.1b
    // review): clause 2 of `should_suppress_contradiction` must not fold away
    // a contradiction whose winner is independently overridden.
    let overridden_nodes: BTreeSet<String> = overridden.keys().cloned().collect();
    let mut suppressed_contradictions = 0usize;
    for c in rate_priority_contradictions(&input, &plan)
        .into_iter()
        .chain(deadline_priority_contradictions(&input, &plan))
    {
        if should_suppress_contradiction(
            &c,
            &chain_member_nodes,
            &override_pinned_below_chain_rank,
            &overridden_nodes,
        ) {
            suppressed_contradictions += 1;
            continue;
        }
        let ref_a = contract_ref_for(&c.node_a);
        let ref_b = contract_ref_for(&c.node_b);
        let contract_ref = match (ref_a, ref_b) {
            (Some(a), Some(b)) if a == b => a,
            (Some(a), Some(b)) => format!("{a}, {b}"),
            (Some(one), None) | (None, Some(one)) => one,
            (None, None) => "<contract unknown>".to_string(),
        };
        let msg = format!(
            "scheduling: `{}` vs `{}` priority order contradicts their `{}` order \
             (contract: {contract_ref}; platform file: {})",
            c.node_a,
            c.node_b,
            c.kind,
            platform_path.display(),
        );
        // Collect only (45.1a) — see `apply_overrides`.
        warnings.push(msg);
    }

    let provenance = build_provenance(
        &plan,
        &input,
        &file.mapper,
        &overridden,
        &diagnostics.details,
    );

    Ok(DerivedSchedPlan {
        plan,
        target: file.target,
        mapper: file.mapper,
        warnings,
        provenance,
        chain_member_nodes,
        suppressed_contradictions,
        // Phase 45.4 — single-derive wiring: `input`/`diagnostics` were
        // already fully consumed above (contradiction scans, provenance);
        // move their chain/node/rank facts out here rather than re-deriving
        // them a second time for the model's `execution.sched`.
        chains: input.chains,
        nodes: input.nodes,
        ranks: diagnostics.details,
        // Phase 45.6 (review) — the exact set of overridden node FQNs the
        // provenance renderer keyed off (`overridden`'s keys), carried out
        // for the model so a model-sourced `--explain` classifies
        // override(...) from fact, not from a priority-equality heuristic.
        overrides: overridden.into_keys().collect(),
    })
}

/// Human-readable rendering of a [`MapWarning`] (Phase 44.3's chain-aware
/// mapper diagnostics), matching the style of this pipeline's other
/// `"scheduling: ..."` warnings.
fn describe_map_warning(w: &MapWarning) -> String {
    match w {
        MapWarning::ChainInfeasible {
            chain,
            sampling_cost_ms,
            budget_ms,
        } => format!(
            "scheduling: chain '{chain}' is infeasible (sampling_cost {sampling_cost_ms:.2}ms \
             meets or exceeds its {budget_ms}ms budget from clock boundaries alone) — excluded \
             from priority shaping; its members keep their local-fact (non-chain) priorities. \
             See the `chain-sampling-feasibility` manifest diagnostic for the per-boundary \
             breakdown."
        ),
        MapWarning::BandTooNarrow {
            distinct_classes,
            band_width,
            clamped,
        } => {
            let who = if clamped.is_empty() {
                String::new()
            } else {
                format!(" (clamped: {})", clamped.join(", "))
            };
            format!(
                "scheduling: rt_priority_band (width {band_width}) is narrower than the \
                 {distinct_classes} distinct priority classes remaining after every legal \
                 collapse — the lowest classes{who} were clamped into the band minimum \
                 (introduces cross-chain/bucket ties, never inversions)"
            )
        }
    }
}

/// The chain-aware contradiction-suppression predicate (45.1b, design study
/// §5.3). `scan_contradictions` (`ros_launch_manifest_sched::validate`) is a
/// generic, mapper- and chain-agnostic pairwise scan: for any two nodes with
/// a known `rate_hz`/`deadline_us` fact, it flags `(node_a, node_b)` when
/// `node_a`'s fact implies it should be prioritized at least as high as
/// `node_b`'s, but the FINAL plan gives `node_a` a strictly lower priority
/// than `node_b` — i.e. `node_b` is always the side that actually "won" the
/// final priority order despite the raw fact saying it shouldn't have.
///
/// The `chain_aware` mapper deliberately ranks its own members by
/// drain-toward-sink chain position, not by raw `rate_hz`/`deadline_us` — so
/// whenever the winning side (`node_b`) is a chain member, the "contradiction"
/// is the mapper doing exactly what it's documented to do, not a defect.
/// Real Autoware data (the 111-line flood this predicate was built against)
/// confirms this covers every false positive: in all 35 unique contradictions
/// produced by that plan, `node_b` was always a chain member — including the
/// 9 that are downstream echoes of the two `override pins chain member`
/// warnings (an override can only ever pin a node that IS a chain member, by
/// construction of that warning), so clause (1) alone happens to subsume
/// clause (2) on that dataset. Clause (2) stays as an explicit, independent
/// rule (not just relying on the coincidence above) for the case clause (1)
/// alone can't catch: an override could in principle pin a chain member so
/// far down that some entirely non-chain node ends up "winning" the pair —
/// that contradiction's `node_b` would NOT be a chain member, but it is
/// still a mechanical, already-explained consequence of the override (named
/// in `override_pinned_below_chain_rank`, built alongside the "override
/// pins" warning above), not new information — so it's folded here too
/// rather than emitted as an independent top-level warning.
///
/// Genuine signal — a contradiction between two nodes where neither is
/// chain-ranked and neither is an override-pinned chain member — is never
/// suppressed by either clause and is reported as before.
///
/// **Clause 2's double-override guard** (W1 review, Important): clause 2 only
/// fires when the winner (`node_b`) is NOT itself explicitly overridden
/// (`!overridden.contains(node_b)`). Clause 1 already handled the case where
/// `node_b` is a chain member, so clause 2 only ever reaches this check when
/// `node_b` is non-chain — and if that non-chain winner's priority is itself
/// the product of a *separate* operator override, the contradiction is a
/// two-override interaction, not a single mechanical consequence of `node_a`'s
/// pin: absent `node_a`'s override, `node_a` would sit at its (higher)
/// chain-derived rank, yet `node_b`'s independent override could still push it
/// above that rank, so the contradiction would persist. Attributing it solely
/// to `node_a`'s pin (and folding it away) would silently drop a genuinely
/// two-cause contradiction — the exact signal-loss this wave fights. So when
/// both sides are independently overridden, the contradiction survives.
fn should_suppress_contradiction(
    c: &Contradiction,
    chain_member_nodes: &BTreeSet<String>,
    override_pinned_below_chain_rank: &BTreeSet<String>,
    overridden: &BTreeSet<String>,
) -> bool {
    // Clause 1: the winner (node_b) is chain-ranked (drain-toward-sink), so
    // the "contradiction" against its raw timing fact is the mapper's own
    // documented design, not a defect.
    if chain_member_nodes.contains(&c.node_b) {
        return true;
    }
    // Clause 2: the loser (node_a) was pinned below its chain-derived rank by
    // a single override already named in an "override pins chain member"
    // warning — AND the winner (node_b) is not itself independently
    // overridden (else the pair is a two-override interaction; see the
    // double-override guard note above).
    override_pinned_below_chain_rank.contains(&c.node_a) && !overridden.contains(&c.node_b)
}

/// Category id for a sched warning message (45.1c) — a `rule_id`-equivalent
/// matching the manifest-checker's convention (`check/src/rules/*.rs`,
/// e.g. `qos-compat`, `state-consistency`) closely enough to `grep`/filter
/// on, without introducing a full `Diagnostic` for warnings that (unlike the
/// manifest checker's) have no source span to render — see
/// `render_sched_warnings_summary`'s doc comment for why
/// `codespan-reporting` isn't used here. Matched by literal prefix/substring
/// against this file's own `format!` templates (`apply_overrides`,
/// `derive_sched_plan`, `describe_map_warning`) — brittle to template wording
/// changes by construction, but every template is defined a few lines above
/// its match arm here, in the same file.
///
/// Covers exactly the warnings [`derive_sched_plan`] can push into
/// [`DerivedSchedPlan::warnings`] — the only warnings this renderer ever
/// sees. The `time_triggered`/`SCHED_DEADLINE not applied` warning is
/// deliberately NOT categorized here: it originates in
/// `execution::sched_plan::SchedPlan::{build,from_model}` (the runtime apply
/// path), lands in a different struct (`SchedPlan::warnings`), and is
/// surfaced there via its own `tracing::warn!` — it never reaches
/// `derive_sched_plan`/`check_sched`, so a branch for it here would be dead.
fn categorize_sched_warning(msg: &str) -> &'static str {
    if msg.starts_with("scheduling: override for unknown node") {
        "sched:override-unknown-node"
    } else if msg.starts_with("scheduling: chain member") {
        "sched:fqn-mismatch"
    } else if msg.starts_with("override pins chain member") {
        "sched:override-inversion"
    } else if msg.contains("is infeasible (sampling_cost") {
        "sched:chain-infeasible"
    } else if msg.starts_with("scheduling: rt_priority_band") {
        "sched:band-too-narrow"
    } else if msg.contains("outside band [") {
        "sched:band-violation-clamp"
    } else if msg.contains("priority order contradicts") {
        "sched:contradiction"
    } else {
        "sched:other"
    }
}

/// Render `warnings` the way the manifest-checker half of `check` already
/// renders its own diagnostics (`check/src/emit/diagnostic.rs`,
/// `codespan-reporting`-based): a per-rule id, deduped identical lines, and
/// a trailing `{errors} error(s), {warnings} warning(s)`-style summary
/// (45.1c). Deliberately does NOT force these through `codespan-reporting`
/// or a full `Diagnostic` — sched warnings have no source span (the
/// contradiction detector, `ros_launch_manifest_sched::validate`, is a pure
/// host-agnostic pairwise scan with no YAML location; the manifest-checker's
/// span rendering has nothing to attach to here) — a structured id +
/// message + dedup + count is the actual goal (brief), not literal reuse of
/// the span-rendering path. `suppressed` is the count of chain-aware/
/// override-derivative contradictions `should_suppress_contradiction`
/// filtered out (45.1b) — surfaced so the suppression itself stays visible
/// instead of silently vanishing 26+ warnings with no trace.
fn render_sched_warnings_summary(warnings: &[String], suppressed: usize) -> String {
    use std::fmt::Write as _;

    let mut seen = std::collections::HashSet::new();
    let mut deduped: Vec<&String> = Vec::new();
    for w in warnings {
        if seen.insert(w.as_str()) {
            deduped.push(w);
        }
    }
    let dupes = warnings.len() - deduped.len();

    let mut out = String::new();
    if deduped.is_empty() && suppressed == 0 {
        return out;
    }
    let _ = write!(out, "{} scheduling warning(s)", deduped.len());
    if suppressed > 0 {
        let _ = write!(out, " ({suppressed} suppressed: chain-intended)");
    }
    if dupes > 0 {
        let _ = write!(out, " ({dupes} duplicate line(s) deduped)");
    }
    let _ = writeln!(out);
    for w in &deduped {
        let _ = writeln!(out, "  warning[{}]: {w}", categorize_sched_warning(w));
    }
    out
}

/// FQN -> scope id, for locating which manifest (if any) declared a node's
/// contract facts — used to cite a contract file in contradiction warnings.
fn fqn_to_scope_id(dump: &LaunchDump) -> HashMap<String, Option<usize>> {
    scheduled_records_from_dump(dump)
        .into_iter()
        .map(|r| (r.fqn, r.scope_id))
        .collect()
}

/// Human-readable description of the timing fact that produced `priority`
/// under `mapper`, for a node's `Provenance::Derived` line (design §7, e.g.
/// `"100 Hz → prio 38"`). The `manual` mapper (and any future mapper that
/// doesn't derive from a single rate/deadline fact) is described by the
/// tier that placed the node instead — `Display` already prefixes the
/// mapper name (`derived(<mapper>: <fact>)`), so repeating it here would be
/// redundant (W4 review, Cosmetic #2).
fn describe_derived_fact(
    mapper: &str,
    node: Option<&ros_launch_manifest_sched::MapperNode>,
    tier_name: &str,
    priority: i64,
) -> String {
    let fmt_num = |v: f64| {
        if v.fract() == 0.0 {
            format!("{v:.0}")
        } else {
            format!("{v}")
        }
    };
    match (
        mapper,
        node.and_then(|n| n.rate_hz),
        node.and_then(|n| n.deadline_us),
    ) {
        ("rate_monotonic", Some(rate), _) => format!("{} Hz → prio {priority}", fmt_num(rate)),
        ("deadline_monotonic", _, Some(deadline_us)) => {
            format!("{deadline_us} us deadline → prio {priority}")
        }
        _ => format!("tier '{tier_name}' → prio {priority}"),
    }
}

/// Build the per-node provenance map (design §7) from the final plan: an
/// override always wins the label; failing that, membership in the
/// synthesized default tier means no timing facts; failing that, a node
/// ranked by the `chain_aware` mapper (present in `details`, Phase 44.4 §5)
/// gets its winning path's pre-formatted provenance line; anything else is
/// the generic mapper-derived placement (`rate_monotonic`/`deadline_monotonic`/
/// `manual`).
fn build_provenance(
    plan: &ResolvedTierTable,
    input: &ros_launch_manifest_sched::MapperInput,
    mapper: &str,
    overridden: &BTreeMap<String, String>,
    details: &[ChainAwareDetail],
) -> BTreeMap<String, Provenance> {
    // Node -> every ChainAwareDetail ranked for one of its paths (Phase
    // 44.4 §5/§6): a node can own >1 ranked path (e.g. two segments of the
    // same chain, or paths in two different chains); the plan's final
    // priority for the node is the mapper's own max-over-paths projection
    // (design step 6) — pick the matching detail (max priority, ties broken
    // by path name for determinism) as the "winning" provenance line, and
    // note the path count when >1 (brief deliverable 5: "(max over N
    // paths)").
    let mut by_node: HashMap<&str, Vec<&ChainAwareDetail>> = HashMap::new();
    for d in details {
        by_node.entry(d.node.as_str()).or_default().push(d);
    }

    let mut out = BTreeMap::new();
    for tier in &plan.tiers {
        for member in &tier.members {
            let provenance = if let Some(selector) = overridden.get(member) {
                Provenance::Override {
                    selector: selector.clone(),
                }
            } else if tier.name == ros_launch_manifest_sched::DEFAULT_TIER {
                Provenance::Default
            } else if let Some(ds) = by_node.get(member.as_str()) {
                let winner = ds
                    .iter()
                    .max_by(|a, b| {
                        a.priority
                            .cmp(&b.priority)
                            .then_with(|| a.path.cmp(&b.path))
                    })
                    .expect("by_node groups are always non-empty");
                let mut line = winner.provenance.clone();
                if ds.len() > 1 {
                    line.push_str(&format!(" (max over {} paths)", ds.len()));
                }
                Provenance::ChainAware { line }
            } else {
                let node = input.nodes.iter().find(|n| &n.name == member);
                Provenance::Derived {
                    mapper: mapper.to_string(),
                    fact: describe_derived_fact(mapper, node, &tier.name, tier.priority),
                }
            };
            out.insert(member.clone(), provenance);
        }
    }
    out
}

/// Load a scheduling platform file (v2 `.yaml` or legacy `.toml`), derive +
/// override + validate a plan for `target`, and print the resolved table —
/// `play_launch check --sched`'s implementation. Reports (never hard-fails
/// on) band violations/contradictions: `check` is a static report, not an
/// apply — `derive_sched_plan` is called with `SchedApplyMode::Warn` so
/// out-of-band priorities are clamped for display rather than aborting the
/// command.
pub fn check_sched(
    dump: &LaunchDump,
    index: Option<&ManifestIndex>,
    sched_path: &Path,
    target: &str,
) -> Result<DerivedSchedPlan> {
    let derived = derive_sched_plan(dump, index, sched_path, target, SchedApplyMode::Warn)?;

    eprintln!(
        "Scheduling ({}, mapper={}): {} tier(s)",
        derived.target,
        derived.mapper,
        derived.plan.tiers.len()
    );
    for t in &derived.plan.tiers {
        eprintln!(
            "  tier {:<16} prio={:<4} sched_class={:<10} core={:<4} members={}",
            t.name,
            t.priority,
            t.sched_class.as_deref().unwrap_or("-"),
            t.core.map(|c| c.to_string()).unwrap_or_else(|| "-".into()),
            t.members.len(),
        );
        for m in &t.members {
            eprintln!("      {m}");
        }
    }
    // Single authoritative surfacing point for `check` (45.1a): structured,
    // deduped, with a summary line (45.1c) — `render_explain` must NOT
    // re-print these, even under `--explain`.
    eprint!(
        "{}",
        render_sched_warnings_summary(&derived.warnings, derived.suppressed_contradictions)
    );
    Ok(derived)
}

/// `play_launch check --sched ... --explain` (design §7): print the merged
/// logical view — one row per schedulable node with its final
/// class/priority/core and *why* (provenance), followed by footer lines
/// naming the platform file and every per-scope contract that fed the
/// mappers.
///
/// Plain aligned text (brief: "no external table crates") — column widths
/// are computed from the actual data so the table stays readable for both
/// short demo fixtures and long FQNs.
pub fn print_explain(
    derived: &DerivedSchedPlan,
    platform: &ResolvedPlatformFile,
    index: Option<&ManifestIndex>,
) {
    eprint!("{}", render_explain(derived, platform, index));
}

/// One `--explain` table row (Phase 45.6, design "Consumers become
/// readers" — "same renderer, one code path"): per-node final
/// class/priority/core + provenance, independent of whether the caller
/// derived it fresh ([`explain_rows_from_derived`], the `check` path) or
/// read it back from an already-resolved [`ros_launch_manifest_model::
/// SystemModel`] ([`explain_rows_from_model`], the `resolve`/`replay
/// --model` path). [`render_explain_table`] is the one formatter both
/// converge on.
#[derive(Debug)]
pub(crate) struct ExplainRow {
    fqn: String,
    class: String,
    priority: i64,
    core: Option<u32>,
    provenance: Provenance,
}

/// Build `--explain`'s rows from a freshly-derived [`DerivedSchedPlan`]
/// (the `check` path — `derived.plan`/`derived.provenance` are this
/// invocation's own mapper output, not read back from anywhere).
fn explain_rows_from_derived(derived: &DerivedSchedPlan) -> Vec<ExplainRow> {
    derived
        .plan
        .tiers
        .iter()
        .flat_map(|t| {
            t.members.iter().map(move |m| ExplainRow {
                fqn: m.clone(),
                class: t
                    .sched_class
                    .clone()
                    .unwrap_or_else(|| "SCHED_OTHER".to_string()),
                priority: t.priority,
                core: t.core,
                provenance: derived
                    .provenance
                    .get(m)
                    .cloned()
                    .unwrap_or(Provenance::Default),
            })
        })
        .collect()
}

/// Phase 45.6 — build `--explain`'s rows directly from a `SystemModel`'s
/// ALREADY-RESOLVED `execution` layer: `tiers`+`bindings` for every
/// RT-scheduled node's final class/priority/core (the exact representation
/// [`crate::execution::sched_plan::SchedPlan::from_model`] applies at
/// runtime), `structure.nodes` for the full schedulable-node identity set
/// (RT nodes AND non-RT ones — Phase 45.4's `execution.bindings` only ever
/// carries non-default tiers, so a plain binding lookup alone would silently
/// drop every non-RT node the `check` path still lists as `default`),
/// and `execution.sched.ranks` for chain-aware provenance. No mapper
/// re-run: every value here is a field read.
///
/// Provenance reconstruction is best-effort where the model doesn't carry
/// enough to be exact (documented per-branch below); it is EXACT for the
/// two cases the design doc calls out by name — chain-aware rows and the
/// non-RT default — which is what Phase 45.6's parity test exercises:
///
/// - No binding at all → [`Provenance::Default`] (byte-for-byte identical
///   to the `check` path's default-tier row).
/// - The FQN is in `execution.sched.overrides` (the exact set of overridden
///   nodes `resolve` embedded, Phase 45.6 review) → [`Provenance::Override`],
///   checked FIRST, unconditionally — the same ordering [`build_provenance`]
///   uses (`if let Some(selector) = overridden.get(member)`), so an override
///   whose applied value coincides with the mapper's own derived priority is
///   still labeled `override(...)`, not silently misclassified as
///   `derived(...)`. The selector STRING is reconstructed as the FQN's bare
///   (last-segment) name — the model carries the overridden *set* but not the
///   verbatim selector text, so the label text remains an approximation
///   (matches every rt_workspace/Autoware fixture, which key overrides by
///   bare node name); the override *category* is now exact.
/// - Else a binding whose FQN carries a `ranks` entry (only the `chain_aware`
///   mapper populates `ranks`): pick the winning (max-priority) rank the
///   same way [`build_provenance`] does → [`Provenance::ChainAware`] with the
///   exact same provenance line (`"(max over N paths)"` suffix under the same
///   condition) `check --explain` would show — the byte-for-byte parity case.
/// - Else (a binding with no `ranks` entry — every non-`chain_aware` mapper):
///   for `deadline_monotonic`, the deadline it derived from is
///   `min(requirements[fqn].paths[].max_latency_ms)`, which IS embedded in
///   the model (`execution.sched.requirements`) — so this reproduces
///   `check`'s `"<deadline_us> us deadline → prio <p>"` exactly (Phase 45.6
///   review, Finding 2). For `rate_monotonic`, the `rate_hz` fact is the max
///   over the node's topic-level publications / `pub.<ep>.min_rate_hz`
///   endpoint facts — a genuinely different source than anything in
///   `requirements` (whose per-path `EffectiveTrigger::Timer{rate_hz}` is
///   only present for timer-triggered paths and isn't the same aggregate) —
///   so it is NOT embedded, and this falls back to the same tier-name
///   description [`describe_derived_fact`] uses for `manual`
///   (`"tier '<name>' → prio <p>"`): honest about the applied outcome without
///   inventing a fact. That `rate_monotonic` fallback is the one genuine
///   provenance-text gap; chain_aware (45.6's target) and now
///   `deadline_monotonic` are exact.
pub(crate) fn explain_rows_from_model(
    model: &ros_launch_manifest_model::SystemModel,
    target: &str,
) -> Vec<ExplainRow> {
    let sched = model.execution.sched.as_ref();
    let mapper_name = sched.and_then(|s| s.mapper.clone());
    let empty_overrides = std::collections::BTreeSet::new();
    let overrides = sched.map(|s| &s.overrides).unwrap_or(&empty_overrides);

    let mut ranks_by_node: HashMap<&str, Vec<&ChainAwareDetail>> = HashMap::new();
    let mut req_by_node: HashMap<&str, &ros_launch_manifest_model::NodeSchedRequirement> =
        HashMap::new();
    if let Some(s) = sched {
        for d in &s.ranks {
            ranks_by_node.entry(d.node.as_str()).or_default().push(d);
        }
        for r in &s.requirements {
            req_by_node.insert(r.node_fqn.as_str(), r);
        }
    }

    let mut rows = Vec::new();
    for fqn in model.structure.nodes.keys() {
        let Some(tier_name) = model.execution.bindings.get(fqn) else {
            rows.push(ExplainRow {
                fqn: fqn.clone(),
                class: "SCHED_OTHER".to_string(),
                priority: 0,
                core: None,
                provenance: Provenance::Default,
            });
            continue;
        };
        let Some(tier) = model.execution.tiers.get(tier_name) else {
            continue;
        };
        let Some(spec) = tier.platform(target) else {
            continue;
        };

        // Mirror `build_provenance`'s classification order exactly: override
        // FIRST (from fact, not a priority heuristic — Phase 45.6 review
        // Finding 1), then chain-aware ranks, then generic derived.
        let provenance = if overrides.contains(fqn) {
            Provenance::Override {
                selector: bare_node_name(fqn),
            }
        } else if let Some(ds) = ranks_by_node.get(fqn.as_str()) {
            let winner = ds
                .iter()
                .max_by(|a, b| {
                    a.priority
                        .cmp(&b.priority)
                        .then_with(|| a.path.cmp(&b.path))
                })
                .expect("ranks_by_node groups are always non-empty");
            let mut line = winner.provenance.clone();
            if ds.len() > 1 {
                line.push_str(&format!(" (max over {} paths)", ds.len()));
            }
            Provenance::ChainAware { line }
        } else {
            let mapper = mapper_name.clone().unwrap_or_else(|| "manual".to_string());
            let fact = derived_fact_from_model(
                &mapper,
                req_by_node.get(fqn.as_str()).copied(),
                tier_name,
                spec.priority,
            );
            Provenance::Derived { mapper, fact }
        };

        rows.push(ExplainRow {
            fqn: fqn.clone(),
            class: spec
                .sched_class
                .clone()
                .unwrap_or_else(|| "SCHED_OTHER".to_string()),
            priority: spec.priority,
            core: spec.core,
            provenance,
        });
    }
    rows
}

/// Model-sourced analogue of [`describe_derived_fact`] (Phase 45.6 review,
/// Finding 2): reproduce the fresh-derive renderer's per-node "fact" text
/// from the model's `execution.sched.requirements` where the fact is
/// embedded. `deadline_monotonic`'s deadline is
/// `min(paths[].max_latency_ms)` — embedded — so its
/// `"<deadline_us> us deadline → prio <p>"` line matches `check` exactly.
/// `rate_monotonic`'s rate fact is NOT embedded (different source; see
/// [`explain_rows_from_model`]'s doc comment), so it — like `manual` and any
/// unknown mapper — falls back to the tier-name description, matching
/// `describe_derived_fact`'s own `manual`/fallback arm.
fn derived_fact_from_model(
    mapper: &str,
    req: Option<&ros_launch_manifest_model::NodeSchedRequirement>,
    tier_name: &str,
    priority: i64,
) -> String {
    if mapper == "deadline_monotonic"
        && let Some(deadline_us) = req.and_then(min_deadline_us_from_requirement)
    {
        return format!("{deadline_us} us deadline → prio {priority}");
    }
    format!("tier '{tier_name}' → prio {priority}")
}

/// The tightest declared deadline across a node's paths, in microseconds —
/// `min(paths[].max_latency_ms) * 1000`, rounded — matching
/// `sched_derive::extract_path_facts`' `deadline_us` derivation exactly (the
/// value `deadline_monotonic` ranks on). `None` when the node declares no
/// path with a `max_latency_ms`.
fn min_deadline_us_from_requirement(
    req: &ros_launch_manifest_model::NodeSchedRequirement,
) -> Option<u64> {
    req.paths
        .iter()
        .filter_map(|p| p.max_latency_ms)
        .fold(None, |acc: Option<f64>, v| {
            Some(acc.map_or(v, |a| a.min(v)))
        })
        .map(|ms| (ms * 1000.0).round() as u64)
}

/// The bare (last path segment) name of an FQN, e.g. `/a/b/c` → `c` — the
/// override-selector convention every rt_workspace/Autoware platform file
/// observed so far uses (see [`explain_rows_from_model`]'s doc comment).
fn bare_node_name(fqn: &str) -> String {
    fqn.rsplit('/').next().unwrap_or(fqn).to_string()
}

/// Pure table formatter shared by every `--explain` source (Phase 45.6,
/// design "same renderer, one code path"): sort (priority desc, FQN asc),
/// compute column widths, render the table, append `footer` verbatim.
/// [`render_explain`] (fresh derive) and [`render_explain_from_model`]
/// (embedded model) build `rows` differently but converge here.
fn render_explain_table(mut rows: Vec<ExplainRow>, footer: &str) -> String {
    use std::fmt::Write as _;

    rows.sort_by(|a, b| b.priority.cmp(&a.priority).then_with(|| a.fqn.cmp(&b.fqn)));

    let fqn_w = rows.iter().map(|r| r.fqn.len()).max().unwrap_or(3).max(3);
    let class_w = rows.iter().map(|r| r.class.len()).max().unwrap_or(5).max(5);

    let mut out = String::new();
    let _ = writeln!(
        out,
        "{:fqn_w$}  {:class_w$}  {:>4}  {:>4}  PROVENANCE",
        "FQN", "CLASS", "PRIO", "CORE"
    );
    for r in &rows {
        let core = r.core.map(|c| c.to_string()).unwrap_or_else(|| "-".into());
        let _ = writeln!(
            out,
            "{:fqn_w$}  {:class_w$}  {:>4}  {:>4}  {}",
            r.fqn, r.class, r.priority, core, r.provenance
        );
    }

    // 45.1a: warnings are NOT re-printed here — `check_sched` (the only
    // caller of `render_explain`, via `print_explain`) already surfaced
    // them once before this table renders. Re-looping `derived.warnings`
    // here was site #3 of the original 2x/3x duplicate-print bug (study
    // §4.2).

    out.push_str(footer);
    out
}

/// The `check`/`resolve` footer: "system file: <channel>(<path>)" plus one
/// "contract[scope N, file]: <channel>(<path>)" line per resolved manifest
/// — shared by both callers that have a [`ResolvedPlatformFile`] +
/// [`ManifestIndex`] in hand (`check` always; `resolve` too, since it
/// resolves both before building its model).
pub(crate) fn explain_footer(
    platform: &ResolvedPlatformFile,
    index: Option<&ManifestIndex>,
) -> String {
    use std::fmt::Write as _;

    let mut out = String::new();
    let _ = writeln!(
        out,
        "system file: {}({})",
        platform.channel,
        platform.path.display()
    );
    if let Some(index) = index {
        let mut scopes: Vec<_> = index.manifests.iter().collect();
        scopes.sort_by_key(|(scope_id, _)| **scope_id);
        for (scope_id, resolved) in scopes {
            let filename = if let Some(ref pkg) = resolved.pkg {
                format!("{}/{}", pkg, resolved.file)
            } else {
                resolved.file.clone()
            };
            let _ = writeln!(
                out,
                "contract[scope {scope_id}, {filename}]: {}({})",
                resolved.channel,
                resolved.contract_path.display()
            );
        }
    }
    out
}

/// Footer for a bare-model `--explain` render with no resolved platform
/// file / manifest index in hand (`replay --model` on a host where the
/// scheduling-platform-file channels were never consulted, since the model
/// already carries the applied plan) — names the mapper the model itself
/// recorded (`execution.sched.mapper`) instead of a file path.
pub(crate) fn explain_footer_from_model(model: &ros_launch_manifest_model::SystemModel) -> String {
    use std::fmt::Write as _;

    let mut out = String::new();
    match model
        .execution
        .sched
        .as_ref()
        .and_then(|s| s.mapper.as_deref())
    {
        Some(mapper) => {
            let _ = writeln!(out, "system file: (embedded in model; mapper={mapper})");
        }
        None => {
            let _ = writeln!(
                out,
                "system file: (embedded in model; no scheduling platform file resolved)"
            );
        }
    }
    out
}

/// Pure string-builder for `--explain`'s output (design §7) — split out from
/// [`print_explain`] so tests can assert on the rendered text directly
/// instead of only smoke-testing that printing doesn't panic. `check`'s own
/// call path: a fresh derive, never re-read from a model.
fn render_explain(
    derived: &DerivedSchedPlan,
    platform: &ResolvedPlatformFile,
    index: Option<&ManifestIndex>,
) -> String {
    let rows = explain_rows_from_derived(derived);
    let footer = explain_footer(platform, index);
    render_explain_table(rows, &footer)
}

/// Phase 45.6 — `--explain` sourced from an already-resolved `SystemModel`
/// instead of a fresh derive: `resolve --explain` (render the model just
/// built) and `replay --model --explain` (render the stored model) both
/// call this. `footer` is caller-supplied so a caller with a fresh
/// `ResolvedPlatformFile`/`ManifestIndex` in hand (`resolve`, which
/// resolves both before building its model) can pass the byte-identical
/// [`explain_footer`] text `check --explain` would show for the same
/// inputs; a caller with only the bare model (`replay --model`, which never
/// consults the platform-file channels once a model is given) passes
/// [`explain_footer_from_model`]'s graceful fallback instead.
pub fn render_explain_from_model(
    model: &ros_launch_manifest_model::SystemModel,
    target: &str,
    footer: &str,
) -> String {
    let rows = explain_rows_from_model(model, target);
    render_explain_table(rows, footer)
}

/// Which channel supplied a resolved platform file (Phase 41.3 design §3.1;
/// mirrors [`crate::ros::manifest_loader::ContractChannel`], plus the
/// explicit-CLI-path channel that contracts don't have — `--contracts` has
/// no equivalent "always wins" per-scope override, but `--sched <path>`
/// does).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlatformFileChannel {
    /// `--sched <path>` given directly — bypasses discovery entirely.
    Explicit,
    /// `<overlay-root>/<pkg>/launch/<stem>.system.<target>.yaml`.
    Overlay,
    /// `<launch-file-dir>/<stem>.system.<target>.yaml`, shipped next to the
    /// launch file.
    Provider,
}

impl fmt::Display for PlatformFileChannel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            PlatformFileChannel::Explicit => "explicit",
            PlatformFileChannel::Overlay => "overlay",
            PlatformFileChannel::Provider => "provider",
        })
    }
}

/// A platform file resolved through the shipping channels, plus which
/// channel supplied it (kept for `check`'s provenance line and, per the
/// W3→W4 handoff, `--explain`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedPlatformFile {
    pub path: PathBuf,
    pub channel: PlatformFileChannel,
}

/// Resolve the overlay path for a platform file targeting `target`, rooted
/// at the ROOT launch scope (platform files are per-launch, not
/// per-include-scope — unlike contracts, which resolve per file scope).
///
/// Layout: `<overlay_root>/<pkg>/launch/<stem>.system.<target>.yaml`. Falls
/// back to the `_` package-dir convention when the root scope has no pkg
/// (mirrors `manifest_loader::resolve_overlay_path`).
pub(crate) fn resolve_platform_overlay_path(
    root_scope: &crate::ros::launch_dump::ScopeEntry,
    overlay_root: &Path,
    target: &str,
) -> Option<PathBuf> {
    let file = root_scope.file()?;
    let stem = contract_stem(file);
    let pkg_dir = root_scope.pkg().unwrap_or("_");
    Some(
        overlay_root
            .join(pkg_dir)
            .join("launch")
            .join(format!("{stem}.system.{target}.yaml")),
    )
}

/// Resolve the provider-sidecar path for a platform file targeting `target`,
/// rooted at the ROOT launch scope. Layout:
/// `<launch-file-dir>/<stem>.system.<target>.yaml`, where the directory
/// comes from the root scope origin's canonicalized launch-file path.
/// Returns `None` when the root scope has no recorded path (older
/// `record.json`, or an origin the parser couldn't resolve to an absolute
/// path) — provider lookup is simply unavailable.
pub(crate) fn resolve_platform_provider_path(
    root_scope: &crate::ros::launch_dump::ScopeEntry,
    target: &str,
) -> Option<PathBuf> {
    let origin = root_scope.origin.as_ref()?;
    let launch_path = origin.path.as_deref()?;
    let dir = Path::new(launch_path).parent()?;
    let stem = contract_stem(&origin.file);
    Some(dir.join(format!("{stem}.system.{target}.yaml")))
}

/// Find the ROOT launch scope (the top-level file scope with no parent) —
/// platform-file resolution is per-launch, not per-include-scope, so it
/// always looks here regardless of which scope any given node lives in.
pub(crate) fn root_scope(dump: &LaunchDump) -> Option<&crate::ros::launch_dump::ScopeEntry> {
    dump.scopes
        .iter()
        .find(|s| s.parent.is_none() && s.is_file_scope())
        .or_else(|| dump.scopes.first())
}

/// Resolve a scheduling platform file through the shipping channels (Phase
/// 41.3 design §3.1): explicit `--sched <path>` > user overlay
/// (`<overlay_root>/<pkg>/launch/<stem>.system.<target>.yaml`) > provider
/// sidecar shipped next to the launch file
/// (`<launch-file-dir>/<stem>.system.<target>.yaml`). Only `.yaml` v2 files
/// ship via these channels — legacy `.toml` bridge files are explicit-path
/// only (never discovered).
///
/// `overlay_root` should already be the result of
/// [`crate::ros::manifest_loader::discover_overlay_root`] — this function
/// does not itself consult `$PLAY_LAUNCH_CONTRACTS`/XDG/`/etc` so contract
/// and platform-file resolution always agree on which root is in play for a
/// given invocation.
///
/// `provider` gates the provider-sidecar channel: `--no-provider-contracts`
/// disables it for platform files exactly as it does for contracts (W3
/// review Important #2) — pass `ContractSources::provider` so both file
/// kinds honor the same flag.
///
/// Returns `None` when nothing resolves in any channel — the caller's
/// existing "no `--sched`" behavior (scheduling disabled) applies unchanged.
pub fn resolve_platform_file(
    dump: &LaunchDump,
    explicit: Option<&Path>,
    overlay_root: Option<&Path>,
    provider: bool,
    target: &str,
) -> Option<ResolvedPlatformFile> {
    if let Some(p) = explicit {
        debug!("platform file: explicit --sched {}", p.display());
        return Some(ResolvedPlatformFile {
            path: p.to_path_buf(),
            channel: PlatformFileChannel::Explicit,
        });
    }

    let Some(scope) = root_scope(dump) else {
        debug!("platform file: no root scope in launch dump — cannot resolve");
        return None;
    };

    if let Some(root) = overlay_root
        && let Some(path) = resolve_platform_overlay_path(scope, root, target)
        && path.exists()
    {
        info!("Scheduling platform file [overlay]: {}", path.display());
        return Some(ResolvedPlatformFile {
            path,
            channel: PlatformFileChannel::Overlay,
        });
    }

    if provider
        && let Some(path) = resolve_platform_provider_path(scope, target)
        && path.exists()
    {
        info!("Scheduling platform file [provider]: {}", path.display());
        return Some(ResolvedPlatformFile {
            path,
            channel: PlatformFileChannel::Provider,
        });
    }

    debug!(
        "platform file: no channel resolved a file for target `{target}` \
         (overlay_root={overlay_root:?}, provider={provider}) — scheduling disabled"
    );
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::launch_dump::{ComposableNodeRecord, NodeRecord, ScopeEntry};

    // Minimal NodeRecord/ScopeEntry construction: use serde_json to avoid
    // enumerating every field of these large records by hand.
    // NodeRecord requires: executable (String), params_files (Vec), cmd (Vec).
    fn dump_with_one_node() -> LaunchDump {
        let json = serde_json::json!({
            "node": [{
                "executable": "ndt_localizer",
                "name": "ndt_localizer",
                "exec_name": "ndt_localizer",
                "params_files": [],
                "cmd": [],
                "scope": 1
            }],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null},
                {"id": 1, "ns": "/localization", "parent": 0}
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    #[test]
    fn maps_node_scope_to_fqn() {
        let dump = dump_with_one_node();
        let nodes = sched_nodes_from_dump(&dump);
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0].name, "/localization/ndt_localizer");
        assert_eq!(nodes[0].scope, "/localization");
        // Silence unused-import lint in case NodeRecord/ScopeEntry drift.
        let _ = (
            std::any::type_name::<NodeRecord>(),
            std::any::type_name::<ScopeEntry>(),
        );
    }

    #[test]
    fn includes_composable_nodes_and_uses_node_namespace() {
        // Node with an explicit namespace "/explicit" in scope 0 (scope ns = "/"):
        // effective_ns must prefer the node's own namespace over scope ns.
        // load_node entry with namespace "/composed": must appear in result.
        let json = serde_json::json!({
            "node": [{
                "executable": "some_exec",
                "name": "ndt_localizer",
                "exec_name": "ndt_localizer",
                "namespace": "/explicit",
                "params_files": [],
                "cmd": [],
                "scope": 0
            }],
            "load_node": [{
                "package": "my_pkg",
                "plugin": "my_pkg::MyNode",
                "target_container_name": "my_container",
                "node_name": "my_node",
                "namespace": "/composed",
                "scope": 0
            }],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null}
            ]
        });
        let dump: LaunchDump = serde_json::from_value(json).expect("valid LaunchDump");
        let nodes = sched_nodes_from_dump(&dump);

        assert_eq!(nodes.len(), 2, "expected node + composable");

        // Node with explicit namespace "/explicit" (scope ns is "/") → own ns wins.
        let n = nodes
            .iter()
            .find(|n| n.name.contains("ndt_localizer"))
            .unwrap();
        assert_eq!(n.name, "/explicit/ndt_localizer");
        assert_eq!(n.scope, "/explicit");

        // Composable node with namespace "/composed".
        let c = nodes.iter().find(|n| n.name.contains("my_node")).unwrap();
        assert_eq!(c.name, "/composed/my_node");
        assert_eq!(c.scope, "/composed");

        // Silence unused-import lint.
        let _ = std::any::type_name::<ComposableNodeRecord>();
    }

    #[test]
    fn composable_fqns_includes_load_node_excludes_regular_node() {
        let json = serde_json::json!({
            "node": [{
                "executable": "some_exec",
                "name": "ndt_localizer",
                "exec_name": "ndt_localizer",
                "namespace": "/explicit",
                "params_files": [],
                "cmd": [],
                "scope": 0
            }],
            "load_node": [{
                "package": "my_pkg",
                "plugin": "my_pkg::MyNode",
                "target_container_name": "my_container",
                "node_name": "my_node",
                "namespace": "/composed",
                "scope": 0
            }],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null}
            ]
        });
        let dump: LaunchDump = serde_json::from_value(json).expect("valid LaunchDump");
        let composables = composable_fqns(&dump);

        assert!(composables.contains("/composed/my_node"));
        assert!(!composables.contains("/explicit/ndt_localizer"));
        assert_eq!(composables.len(), 1);
    }

    // ── v2 derive pipeline (Phase 41.2) ──

    /// Two regular nodes in the root scope, no contracts.
    fn dump_two_plain_nodes() -> LaunchDump {
        let json = serde_json::json!({
            "node": [
                {
                    "executable": "fast_node",
                    "name": "fast_node",
                    "exec_name": "fast_node",
                    "params_files": [],
                    "cmd": [],
                    "scope": 0
                },
                {
                    "executable": "slow_node",
                    "name": "slow_node",
                    "exec_name": "slow_node",
                    "params_files": [],
                    "cmd": [],
                    "scope": 0
                }
            ],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null}
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    /// A contract index giving `/fast_node` 100 Hz and `/slow_node` 10 Hz
    /// (topic-level rate facts on topics each publishes).
    fn index_with_rates() -> ManifestIndex {
        use crate::ros::manifest_loader::{ContractChannel, ResolvedManifest, ResolvedTopic};
        let mut index = ManifestIndex::default();
        for (topic, node, rate) in [
            ("/fast_topic", "/fast_node", 100.0),
            ("/slow_topic", "/slow_node", 10.0),
        ] {
            index.topics.insert(
                topic.to_string(),
                ResolvedTopic {
                    fqn: topic.to_string(),
                    msg_type: "std_msgs/msg/String".to_string(),
                    qos: None,
                    publishers: vec![format!("{node}/out")],
                    subscribers: vec![],
                    rate_hz: Some(rate),
                    max_transport_ms: None,
                    drop: None,
                    scope_ids: vec![0],
                },
            );
        }
        // A resolved manifest at scope 0 — exercised by the `--explain`
        // footer tests (contract-file citation) and by
        // `contradiction_warning_cites_contract_and_platform_file` (the
        // contract-file half of a contradiction warning).
        index.manifests.insert(
            0,
            ResolvedManifest {
                scope_id: 0,
                pkg: Some("rate_fixture".to_string()),
                file: "two_nodes.launch.xml".to_string(),
                ns: "/".to_string(),
                channel: ContractChannel::Provider,
                contract_path: PathBuf::from(
                    "/opt/share/rate_fixture/launch/two_nodes.contract.yaml",
                ),
                manifest: Default::default(),
                source: String::new(),
                diagnostics: vec![],
            },
        );
        index
    }

    /// A contract index declaring one chain (`sensing_chain`): `/fast_node`
    /// has a Timer path (`tick`, 50 Hz — a Boundary) that outputs
    /// `/link_topic`; `/slow_node` has an Input path (`react`, consuming
    /// `/link_topic`, 8ms deadline — a Segment) — a 2-element chain
    /// exercising both `ChainElement` kinds (Phase 44.4).
    fn index_with_chain() -> ManifestIndex {
        use crate::ros::manifest_loader::{
            ContractChannel, ResolvedManifest, ResolvedNodePath, ResolvedTopic,
        };
        use ros_launch_manifest_types::{
            ChainDecl, ChainSegment, ChainSemantics, Manifest, NodeDecl, PathDecl, Trigger,
        };

        let mut index = ManifestIndex::default();
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/fast_node".to_string(),
            path_name: "tick".to_string(),
            path: PathDecl {
                trigger: Some(Trigger::Timer { rate_hz: 50.0 }),
                output: vec!["tick_out".to_string()],
                // Deliberately the LOOSER deadline of the two paths: the
                // chain_aware mapper ranks the downstream Segment (`react`)
                // above the Boundary (`tick`) regardless of raw per-path
                // deadlines (chain context, not local DM order) — picking
                // `tick`'s own deadline looser than `react`'s here keeps
                // the generic (mapper-agnostic) `deadline-vs-priority`
                // contradiction check agreeing with the chain-aware
                // placement, so this fixture doesn't also (correctly, but
                // distractingly for this test's purpose) trip that
                // unrelated warning.
                max_latency_ms: Some(20.0),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/slow_node".to_string(),
            path_name: "react".to_string(),
            path: PathDecl {
                trigger: Some(Trigger::Input(vec!["react_in".to_string()])),
                max_latency_ms: Some(1.0),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.topics.insert(
            "/link_topic".to_string(),
            ResolvedTopic {
                fqn: "/link_topic".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/fast_node/tick_out".to_string()],
                subscribers: vec!["/slow_node/react_in".to_string()],
                rate_hz: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );

        let mut nodes = BTreeMap::new();
        nodes.insert(
            "fast_node".to_string(),
            NodeDecl {
                criticality: Some("high".to_string()),
                ..Default::default()
            },
        );
        let mut chains = BTreeMap::new();
        chains.insert(
            "sensing_chain".to_string(),
            ChainDecl {
                semantics: ChainSemantics::Reaction,
                max_latency_ms: 100.0,
                segments: vec![
                    ChainSegment::Path {
                        scope: "/".to_string(),
                        path: "tick".to_string(),
                    },
                    ChainSegment::Via {
                        via: "/link_topic".to_string(),
                    },
                    ChainSegment::Path {
                        scope: "/".to_string(),
                        path: "react".to_string(),
                    },
                ],
            },
        );
        let manifest = Manifest {
            version: 1,
            nodes,
            chains,
            ..Default::default()
        };
        index.manifests.insert(
            0,
            ResolvedManifest {
                scope_id: 0,
                pkg: Some("chain_fixture".to_string()),
                file: "two_nodes.launch.xml".to_string(),
                ns: "/".to_string(),
                channel: ContractChannel::Provider,
                contract_path: PathBuf::from(
                    "/opt/share/chain_fixture/launch/two_nodes.contract.yaml",
                ),
                manifest,
                source: String::new(),
                diagnostics: vec![],
            },
        );
        index
    }

    /// RAII temp-file guard (same pattern as `execution::sched_plan` tests).
    struct TempFileGuard(std::path::PathBuf);
    impl Drop for TempFileGuard {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    fn with_temp_file<F: FnOnce(&Path)>(ext: &str, contents: &str, f: F) {
        let path = std::env::temp_dir().join(format!(
            "play_launch_sched_loader_test_{}_{}.{ext}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::write(&path, contents).expect("write temp file");
        let guard = TempFileGuard(path);
        f(&guard.0);
    }

    const V2_RM_BAND: &str = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
";

    const V2_CHAIN_AWARE_BAND: &str = "\
target: posix
mapper: chain_aware
resources:
  rt_priority_band: { min: 5, max: 45 }
";

    #[test]
    fn derive_rate_monotonic_orders_by_contract_rate() {
        let dump = dump_two_plain_nodes();
        let index = index_with_rates();
        with_temp_file("yaml", V2_RM_BAND, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");
            assert_eq!(derived.mapper, "rate_monotonic");
            assert_eq!(derived.target, "posix");
            let fast = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.members == vec!["/fast_node".to_string()])
                .expect("fast_node tier");
            let slow = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.members == vec!["/slow_node".to_string()])
                .expect("slow_node tier");
            assert_eq!(fast.priority, 40, "100 Hz → band max");
            assert_eq!(slow.priority, 10, "10 Hz → band min");
            assert!(derived.warnings.is_empty(), "{:?}", derived.warnings);
        });
    }

    #[test]
    fn derive_unknown_mapper_errors_listing_known_names() {
        let dump = dump_two_plain_nodes();
        let yaml = "target: posix\nmapper: bogus_algo\n";
        with_temp_file("yaml", yaml, |path| {
            let err = derive_sched_plan(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect_err("unknown mapper must error");
            let msg = err.to_string();
            assert!(msg.contains("bogus_algo"), "{msg}");
            assert!(
                msg.contains("manual")
                    && msg.contains("rate_monotonic")
                    && msg.contains("deadline_monotonic"),
                "error must list known mappers: {msg}"
            );
        });
    }

    #[test]
    fn derive_target_mismatch_errors() {
        let dump = dump_two_plain_nodes();
        let yaml = "target: zephyr\nmapper: manual\n";
        with_temp_file("yaml", yaml, |path| {
            let err = derive_sched_plan(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect_err("target mismatch must error");
            let msg = err.to_string();
            assert!(msg.contains("zephyr") && msg.contains("posix"), "{msg}");
        });
    }

    #[test]
    fn derive_override_beats_derived_and_unknown_override_warns() {
        let dump = dump_two_plain_nodes();
        let index = index_with_rates();
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  slow_node: { priority: 39, core: 0 }
  ghost_node: { priority: 12 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");
            let slow = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.members == vec!["/slow_node".to_string()])
                .expect("slow_node tier");
            assert_eq!(slow.priority, 39, "override beats derived (10)");
            assert_eq!(slow.core, Some(0));
            assert!(
                derived.warnings.iter().any(|w| w.contains("ghost_node")),
                "unknown override must warn: {:?}",
                derived.warnings
            );
            // The pinned slow_node (39) now outranks fast_node's derived 40?
            // No — 39 < 40, rate order preserved, so no contradiction.
            assert!(
                !derived.warnings.iter().any(|w| w.contains("contradicts")),
                "{:?}",
                derived.warnings
            );
        });
    }

    #[test]
    fn derive_override_promotes_factless_node_out_of_default_tier() {
        let dump = dump_two_plain_nodes();
        // No contract index at all: both nodes are fact-less (default tier).
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  fast_node: { priority: 20 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived = derive_sched_plan(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect("derive");
            let fast = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.members == vec!["/fast_node".to_string()])
                .expect("fast_node promoted out of default");
            assert_eq!(fast.priority, 20);
            assert_eq!(fast.sched_class.as_deref(), Some("SCHED_FIFO"));
            let default_tier = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.name == ros_launch_manifest_sched::DEFAULT_TIER)
                .expect("default tier still present");
            assert_eq!(default_tier.members, vec!["/slow_node".to_string()]);
        });
    }

    #[test]
    fn derive_band_violation_warn_clamps_strict_errors() {
        let dump = dump_two_plain_nodes();
        let index = index_with_rates();
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  slow_node: { priority: 80 }
";
        with_temp_file("yaml", yaml, |path| {
            // Warn: clamp + warning.
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("warn mode clamps, not errors");
            let slow = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.members == vec!["/slow_node".to_string()])
                .unwrap();
            assert_eq!(slow.priority, 40, "80 clamped to band max");
            assert!(
                derived.warnings.iter().any(|w| w.contains("clamping")),
                "{:?}",
                derived.warnings
            );

            // Strict: hard error.
            let err = derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Strict)
                .expect_err("strict mode must error on band violation");
            assert!(err.to_string().contains("band"), "{err}");
        });
    }

    #[test]
    fn derive_rate_contradiction_from_override_is_warned() {
        let dump = dump_two_plain_nodes();
        let index = index_with_rates();
        // Pin the SLOW node above the fast one — contradicts the rate order.
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  slow_node: { priority: 40 }
  fast_node: { priority: 12 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("contradictions are warnings, never fatal");
            assert!(
                derived
                    .warnings
                    .iter()
                    .any(|w| w.contains("contradicts") && w.contains("rate_hz")),
                "expected a rate contradiction warning: {:?}",
                derived.warnings
            );
        });
    }

    // --- 45.1b: `should_suppress_contradiction` predicate -----------------

    fn contradiction(node_a: &str, node_b: &str, kind: &'static str) -> Contradiction {
        Contradiction {
            node_a: node_a.to_string(),
            node_b: node_b.to_string(),
            kind,
        }
    }

    #[test]
    fn suppresses_when_winning_side_is_a_chain_member() {
        // A non-chain node's raw deadline_us implies it should outrank the
        // chain member, but the chain member (`node_b`, the side that
        // actually won the final priority order) is ranked by chain
        // drain-position, not the raw fact — this is the mapper doing what
        // it's documented to do, not a defect (design study §5.2, bucket
        // NONCHAIN-vs-CHAIN / CHAIN-INTERNAL).
        let c = contradiction("/nonchain_tight_deadline", "/chain_member", "deadline_us");
        let chain_members = BTreeSet::from(["/chain_member".to_string()]);
        let pinned = BTreeSet::new();
        let overridden = BTreeSet::new();
        assert!(should_suppress_contradiction(
            &c,
            &chain_members,
            &pinned,
            &overridden
        ));
    }

    #[test]
    fn keeps_contradiction_between_two_non_chain_nodes() {
        // Neither side is chain-ranked and neither is an override-pinned
        // chain member — this is genuine signal (design study §5.2: two
        // plain nodes whose raw facts and final priorities actually
        // disagree, with no chain_aware involvement to explain it away).
        let c = contradiction("/nonchain_a", "/nonchain_b", "rate_hz");
        let chain_members = BTreeSet::new();
        let pinned = BTreeSet::new();
        let overridden = BTreeSet::new();
        assert!(!should_suppress_contradiction(
            &c,
            &chain_members,
            &pinned,
            &overridden
        ));
    }

    #[test]
    fn keeps_contradiction_when_losing_side_is_chain_member_but_winner_is_not() {
        // The fact-implied "should lead" side (`node_a`) happens to be a
        // chain member, but the actual winner (`node_b`) is plain and
        // NOT an override-pinned chain member — only `node_b`'s status
        // controls suppression (see `should_suppress_contradiction`'s doc
        // comment); this case is still genuine signal.
        let c = contradiction("/chain_member", "/nonchain_b", "deadline_us");
        let chain_members = BTreeSet::from(["/chain_member".to_string()]);
        let pinned = BTreeSet::new();
        let overridden = BTreeSet::new();
        assert!(!should_suppress_contradiction(
            &c,
            &chain_members,
            &pinned,
            &overridden
        ));
    }

    #[test]
    fn suppresses_override_derivative_contradiction_even_when_winner_is_not_a_chain_member() {
        // `node_a` was pinned below its chain-derived rank by an explicit
        // override (already named in a separate "override pins chain
        // member" warning) — every contradiction this produces against
        // some OTHER, non-chain node is a mechanical downstream echo of
        // that one already-reported decision (design study §5.2, bucket
        // OVERRIDE-DERIVATIVE), not new information, even though `node_b`
        // itself is plain. `node_b` is NOT itself overridden, so clause 2
        // fires.
        let c = contradiction("/pinned_chain_member", "/nonchain_winner", "deadline_us");
        let chain_members = BTreeSet::from(["/pinned_chain_member".to_string()]);
        let pinned = BTreeSet::from(["/pinned_chain_member".to_string()]);
        let overridden = BTreeSet::from(["/pinned_chain_member".to_string()]);
        assert!(should_suppress_contradiction(
            &c,
            &chain_members,
            &pinned,
            &overridden
        ));
    }

    #[test]
    fn keeps_contradiction_when_both_sides_independently_overridden() {
        // Double-override guard (W1 review, Important): `node_a` was pinned
        // below its chain rank by one override, but the winner (`node_b`,
        // non-chain) got its winning priority from a SEPARATE, independent
        // override. The contradiction is then a two-override interaction —
        // absent node_a's pin it would sit at its higher chain-derived rank,
        // yet node_b's own override could still push it above that rank, so
        // the contradiction is not purely a mechanical consequence of
        // node_a's pin. Attributing it solely to node_a's override and
        // folding it away would silently drop genuine two-cause signal — so
        // clause 2 must NOT fire.
        let c = contradiction(
            "/pinned_chain_member",
            "/overridden_nonchain",
            "deadline_us",
        );
        let chain_members = BTreeSet::from(["/pinned_chain_member".to_string()]);
        let pinned = BTreeSet::from(["/pinned_chain_member".to_string()]);
        let overridden = BTreeSet::from([
            "/pinned_chain_member".to_string(),
            "/overridden_nonchain".to_string(),
        ]);
        assert!(!should_suppress_contradiction(
            &c,
            &chain_members,
            &pinned,
            &overridden
        ));
    }

    #[test]
    fn derive_legacy_toml_manual_mapper_matches_v1_resolution() {
        let dump = dump_two_plain_nodes();
        let toml = r#"
[tiers.rt]
class = "real_time"

[tiers.rt.posix]
priority = 20
sched_class = "SCHED_FIFO"

[[assign]]
tier = "rt"
nodes = ["fast_node"]
"#;
        with_temp_file("toml", toml, |path| {
            let derived = derive_sched_plan(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect("legacy toml derives via bridge + manual mapper");
            assert_eq!(derived.mapper, "manual");
            assert_eq!(derived.target, "posix");
            let rt = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.members.contains(&"/fast_node".to_string()))
                .expect("fast_node assigned");
            assert_eq!(rt.name, "rt", "tier identity preserved for diagnostics");
            assert_eq!(rt.priority, 20);
            assert_eq!(rt.sched_class.as_deref(), Some("SCHED_FIFO"));
            let default_tier = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.name == ros_launch_manifest_sched::DEFAULT_TIER)
                .expect("default tier");
            assert_eq!(default_tier.members, vec!["/slow_node".to_string()]);
        });
    }

    // ── Phase 44.4: chain_aware pipeline wiring ──

    #[test]
    fn derive_chain_aware_ranks_boundary_and_segment_with_chain_aware_provenance() {
        let dump = dump_two_plain_nodes();
        let index = index_with_chain();
        with_temp_file("yaml", V2_CHAIN_AWARE_BAND, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");
            assert_eq!(derived.mapper, "chain_aware");
            assert!(derived.warnings.is_empty(), "{:?}", derived.warnings);
            assert_eq!(
                derived.chain_member_nodes,
                std::collections::BTreeSet::from([
                    "/fast_node".to_string(),
                    "/slow_node".to_string()
                ])
            );

            // Sink-first: the Segment (react, chain-adjacent to nothing
            // downstream — it IS the sink) outranks the Boundary (tick).
            let fast = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.members == vec!["/fast_node".to_string()])
                .expect("fast_node tier");
            let slow = derived
                .plan
                .tiers
                .iter()
                .find(|t| t.members == vec!["/slow_node".to_string()])
                .expect("slow_node tier");
            assert!(
                slow.priority > fast.priority,
                "segment (sink) must outrank boundary: slow={} fast={}",
                slow.priority,
                fast.priority
            );

            match derived.provenance.get("/fast_node").expect("fast_node") {
                Provenance::ChainAware { line } => {
                    assert!(line.contains("derived(chain_aware:"), "{line}");
                    assert!(line.contains("boundary"), "{line}");
                    assert!(
                        line.contains(&format!("-> prio {}", fast.priority)),
                        "{line}"
                    );
                }
                other => panic!("expected ChainAware provenance, got {other:?}"),
            }
            match derived.provenance.get("/slow_node").expect("slow_node") {
                Provenance::ChainAware { line } => {
                    assert!(line.contains("derived(chain_aware:"), "{line}");
                    assert!(line.contains("segment"), "{line}");
                }
                other => panic!("expected ChainAware provenance, got {other:?}"),
            }
        });
    }

    /// Two INDEPENDENT single-boundary chains (`chain_a` over `/fast_node`,
    /// `chain_b` over `/slow_node`) — no `via` linking them at all. Distinct
    /// chains never collapse into each other under band compression (design
    /// step 7: "never across the chain/non-chain divide or a criticality
    /// bucket [or a different chain]"), so a width-1 band is guaranteed to
    /// overflow with exactly 2 irreducible classes — unlike
    /// `index_with_chain`'s single 2-element chain, where both elements
    /// share one `coarse_group` and DO legally collapse together under
    /// scarcity.
    fn index_with_two_independent_chains() -> ManifestIndex {
        use crate::ros::manifest_loader::{ContractChannel, ResolvedManifest, ResolvedNodePath};
        use ros_launch_manifest_types::{
            ChainDecl, ChainSegment, ChainSemantics, Manifest, PathDecl, Trigger,
        };

        let mut index = ManifestIndex::default();
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/fast_node".to_string(),
            path_name: "tick".to_string(),
            path: PathDecl {
                trigger: Some(Trigger::Timer { rate_hz: 50.0 }),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/slow_node".to_string(),
            path_name: "tick2".to_string(),
            path: PathDecl {
                trigger: Some(Trigger::Timer { rate_hz: 10.0 }),
                ..Default::default()
            },
            scope_id: 0,
        });

        let mut chains = BTreeMap::new();
        for (name, path) in [("chain_a", "tick"), ("chain_b", "tick2")] {
            chains.insert(
                name.to_string(),
                ChainDecl {
                    semantics: ChainSemantics::Reaction,
                    max_latency_ms: 1000.0,
                    // A single-element chain is valid (first == last == a
                    // Path segment); no `via` needed since there's nothing
                    // to connect.
                    segments: vec![ChainSegment::Path {
                        scope: "/".to_string(),
                        path: path.to_string(),
                    }],
                },
            );
        }
        let manifest = Manifest {
            version: 1,
            chains,
            ..Default::default()
        };
        index.manifests.insert(
            0,
            ResolvedManifest {
                scope_id: 0,
                pkg: Some("two_chain_fixture".to_string()),
                file: "two_nodes.launch.xml".to_string(),
                ns: "/".to_string(),
                channel: ContractChannel::Provider,
                contract_path: PathBuf::from(
                    "/opt/share/two_chain_fixture/launch/two_nodes.contract.yaml",
                ),
                manifest,
                source: String::new(),
                diagnostics: vec![],
            },
        );
        index
    }

    #[test]
    fn derive_chain_aware_band_too_narrow_warns_in_warn_mode_errors_in_strict() {
        let dump = dump_two_plain_nodes();
        let index = index_with_two_independent_chains();
        // A single-priority band: two DIFFERENT chains' items never
        // collapse into each other (design step 7), so 2 irreducible
        // classes can't fit into width 1 — guaranteed `BandTooNarrow`.
        let narrow_yaml = "\
target: posix
mapper: chain_aware
resources:
  rt_priority_band: { min: 45, max: 45 }
";
        with_temp_file("yaml", narrow_yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("warn mode clamps, does not error");
            assert!(
                derived
                    .warnings
                    .iter()
                    .any(|w| w.contains("rt_priority_band") && w.contains("narrower")),
                "{:?}",
                derived.warnings
            );
        });
        with_temp_file("yaml", narrow_yaml, |path| {
            let err = derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Strict)
                .expect_err("strict mode must refuse a too-narrow band");
            let msg = err.to_string();
            assert!(msg.contains("narrower"), "{msg}");
        });
    }

    // ── Phase 45.5: chain_member_nodes_from_chains (shared by derive + from_model) ──

    #[test]
    fn chain_member_nodes_from_chains_walks_segments_and_boundaries() {
        use ros_launch_manifest_sched::{
            ChainSemantics as SchedChainSemantics, Criticality, SegmentNode,
        };

        let chains = vec![ResolvedChain {
            name: "c".to_string(),
            criticality: Criticality::Medium,
            max_latency_ms: 10.0,
            semantics: SchedChainSemantics::Reaction,
            elements: vec![
                ChainElement::Boundary {
                    node: "/a".to_string(),
                    path: "tick".to_string(),
                    period_ms: 10.0,
                    exec_ms: None,
                },
                ChainElement::Segment {
                    nodes_in_topo_order: vec![
                        SegmentNode {
                            node: "/b".to_string(),
                            path: "x".to_string(),
                        },
                        SegmentNode {
                            node: "/c".to_string(),
                            path: "y".to_string(),
                        },
                    ],
                },
            ],
        }];
        assert_eq!(
            chain_member_nodes_from_chains(&chains),
            BTreeSet::from(["/a".to_string(), "/b".to_string(), "/c".to_string()])
        );
    }

    #[test]
    fn chain_member_nodes_from_chains_empty_for_no_chains() {
        assert!(chain_member_nodes_from_chains(&[]).is_empty());
    }

    // ── Phase 41.4: provenance + `--explain` ──

    #[test]
    fn render_explain_manual_mapper_cites_tier_not_mapper_twice() {
        // W4 review Cosmetic #2: the manual mapper's provenance must not
        // read `derived(manual: assigned by `manual` ...)` — it cites the
        // tier that placed the node instead.
        let dump = dump_two_plain_nodes();
        let toml = r#"
[tiers.rt]
class = "real_time"

[tiers.rt.posix]
priority = 20
sched_class = "SCHED_FIFO"

[[assign]]
tier = "rt"
nodes = ["fast_node"]
"#;
        with_temp_file("toml", toml, |path| {
            let derived = derive_sched_plan(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect("legacy toml derives via bridge + manual mapper");

            match derived.provenance.get("/fast_node").expect("fast_node") {
                Provenance::Derived { mapper, fact } => {
                    assert_eq!(mapper, "manual");
                    assert_eq!(fact, "tier 'rt' → prio 20");
                }
                other => panic!("expected Derived, got {other:?}"),
            }

            let resolved = ResolvedPlatformFile {
                path: path.to_path_buf(),
                channel: PlatformFileChannel::Explicit,
            };
            let text = render_explain(&derived, &resolved, None);
            assert!(
                text.contains("derived(manual: tier 'rt' → prio 20)"),
                "{text}"
            );
            assert!(
                !text.contains("assigned by"),
                "redundant mapper-name wording must be gone: {text}"
            );
        });
    }

    #[test]
    fn provenance_marks_override_derived_and_default() {
        let dump = dump_two_plain_nodes();
        let index = index_with_rates();
        // fast_node/slow_node both have rate facts (derived); pin slow_node
        // via an override; a third fact-less node lands in the default tier.
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  slow_node: { priority: 39, core: 0 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");

            match derived.provenance.get("/fast_node").expect("fast_node") {
                Provenance::Derived { mapper, fact } => {
                    assert_eq!(mapper, "rate_monotonic");
                    assert!(fact.contains("100"), "{fact}");
                    assert!(fact.contains("prio 40"), "{fact}");
                }
                other => panic!("expected Derived, got {other:?}"),
            }

            match derived.provenance.get("/slow_node").expect("slow_node") {
                Provenance::Override { selector } => assert_eq!(selector, "slow_node"),
                other => panic!("expected Override, got {other:?}"),
            }
        });
    }

    #[test]
    fn provenance_default_for_factless_node() {
        let dump = dump_two_plain_nodes();
        // No contract index: both nodes are fact-less.
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived = derive_sched_plan(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect("derive");
            assert_eq!(
                derived.provenance.get("/fast_node"),
                Some(&Provenance::Default)
            );
            assert_eq!(
                derived.provenance.get("/slow_node"),
                Some(&Provenance::Default)
            );
        });
    }

    #[test]
    fn provenance_promoted_factless_node_is_override_not_default() {
        let dump = dump_two_plain_nodes();
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  fast_node: { priority: 20 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived = derive_sched_plan(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect("derive");
            match derived.provenance.get("/fast_node").expect("fast_node") {
                Provenance::Override { selector } => assert_eq!(selector, "fast_node"),
                other => panic!("expected Override, got {other:?}"),
            }
            assert_eq!(
                derived.provenance.get("/slow_node"),
                Some(&Provenance::Default)
            );
        });
    }

    #[test]
    fn provenance_display_matches_design_format() {
        assert_eq!(
            Provenance::Override {
                selector: "sensor_node".to_string()
            }
            .to_string(),
            "override(sensor_node)"
        );
        assert_eq!(
            Provenance::Derived {
                mapper: "rate_monotonic".to_string(),
                fact: "100 Hz → prio 38".to_string(),
            }
            .to_string(),
            "derived(rate_monotonic: 100 Hz → prio 38)"
        );
        assert_eq!(Provenance::Default.to_string(), "default (no timing facts)");
    }

    /// Two nodes in DIFFERENT scopes, each scope with its own contract file
    /// (distinct paths) — for the dual-scope contradiction-citation test
    /// (W4 review Important #1).
    fn dump_two_nodes_two_scopes() -> LaunchDump {
        let json = serde_json::json!({
            "node": [
                {
                    "executable": "fast_node",
                    "name": "fast_node",
                    "exec_name": "fast_node",
                    "params_files": [],
                    "cmd": [],
                    "scope": 0
                },
                {
                    "executable": "slow_node",
                    "name": "slow_node",
                    "exec_name": "slow_node",
                    "params_files": [],
                    "cmd": [],
                    "scope": 1
                }
            ],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null},
                {"id": 1, "ns": "/", "parent": 0}
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    fn index_with_rates_two_scopes() -> ManifestIndex {
        use crate::ros::manifest_loader::{ContractChannel, ResolvedManifest, ResolvedTopic};
        let mut index = ManifestIndex::default();
        for (topic, node, rate, scope_id) in [
            ("/fast_topic", "/fast_node", 100.0, 0usize),
            ("/slow_topic", "/slow_node", 10.0, 1usize),
        ] {
            index.topics.insert(
                topic.to_string(),
                ResolvedTopic {
                    fqn: topic.to_string(),
                    msg_type: "std_msgs/msg/String".to_string(),
                    qos: None,
                    publishers: vec![format!("{node}/out")],
                    subscribers: vec![],
                    rate_hz: Some(rate),
                    max_transport_ms: None,
                    drop: None,
                    scope_ids: vec![scope_id],
                },
            );
            index.manifests.insert(
                scope_id,
                ResolvedManifest {
                    scope_id,
                    pkg: Some(format!("pkg{scope_id}")),
                    file: format!("scope{scope_id}.launch.xml"),
                    ns: "/".to_string(),
                    channel: ContractChannel::Provider,
                    contract_path: PathBuf::from(format!(
                        "/opt/share/pkg{scope_id}/launch/scope{scope_id}.contract.yaml"
                    )),
                    manifest: Default::default(),
                    source: String::new(),
                    diagnostics: vec![],
                },
            );
        }
        index
    }

    #[test]
    fn contradiction_warning_cites_both_scopes_contracts() {
        let dump = dump_two_nodes_two_scopes();
        let index = index_with_rates_two_scopes();
        // Invert the rate order: slow (scope 1) pinned above fast (scope 0).
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  slow_node: { priority: 40 }
  fast_node: { priority: 12 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("contradictions are warnings, never fatal");
            let msg = derived
                .warnings
                .iter()
                .find(|w| w.contains("contradicts"))
                .expect("expected a contradiction warning");
            // BOTH scopes' contract files must be cited (W4 review
            // Important #1) — a multi-include tree can put the two nodes
            // of a contradicting pair in different contracts.
            assert!(msg.contains("scope0.contract.yaml"), "{msg}");
            assert!(msg.contains("scope1.contract.yaml"), "{msg}");
        });
    }

    #[test]
    fn contradiction_warning_dedupes_identical_contract_citation() {
        // Same-scope pair (both nodes in scope 0, one shared contract):
        // the contract path must appear exactly once, not twice.
        let dump = dump_two_plain_nodes();
        let index = index_with_rates();
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  slow_node: { priority: 40 }
  fast_node: { priority: 12 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");
            let msg = derived
                .warnings
                .iter()
                .find(|w| w.contains("contradicts"))
                .expect("expected a contradiction warning");
            assert_eq!(
                msg.matches("two_nodes.contract.yaml").count(),
                1,
                "identical contract citation must be deduped: {msg}"
            );
        });
    }

    #[test]
    fn contradiction_warning_cites_contract_and_platform_file() {
        let dump = dump_two_plain_nodes();
        let index = index_with_rates();
        // Pin the SLOW node above the fast one — contradicts the rate order
        // (reuses `derive_rate_contradiction_from_override_is_warned`'s
        // scenario, but this test asserts the file-citation wording added
        // in Phase 41.4).
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  slow_node: { priority: 40 }
  fast_node: { priority: 12 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("contradictions are warnings, never fatal");
            let msg = derived
                .warnings
                .iter()
                .find(|w| w.contains("contradicts"))
                .expect("expected a contradiction warning");
            assert!(msg.contains("platform file:"), "{msg}");
            assert!(msg.contains(&path.display().to_string()), "{msg}");
        });
    }

    #[test]
    fn render_explain_shows_override_derived_default_and_footer() {
        let dump = dump_two_plain_nodes();
        let index = index_with_rates();
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
overrides:
  slow_node: { priority: 39, core: 0 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");
            let resolved = ResolvedPlatformFile {
                path: path.to_path_buf(),
                channel: PlatformFileChannel::Explicit,
            };
            let text = render_explain(&derived, &resolved, Some(&index));

            // Header + per-node rows.
            assert!(text.contains("PROVENANCE"), "{text}");
            assert!(
                text.contains("/fast_node") && text.contains("derived(rate_monotonic: 100 Hz"),
                "{text}"
            );
            assert!(
                text.contains("/slow_node") && text.contains("override(slow_node)"),
                "{text}"
            );

            // Footer: platform file + per-scope contract, channel-tagged.
            assert!(
                text.contains(&format!("system file: explicit({})", path.display())),
                "{text}"
            );
            assert!(text.contains("contract[scope 0,"), "{text}");
            assert!(text.contains("provider("), "{text}");
        });
    }

    #[test]
    fn render_explain_shows_chain_aware_provenance_and_max_over_paths_note() {
        let dump = dump_two_plain_nodes();
        let index = index_with_chain();
        with_temp_file("yaml", V2_CHAIN_AWARE_BAND, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");
            let resolved = ResolvedPlatformFile {
                path: path.to_path_buf(),
                channel: PlatformFileChannel::Explicit,
            };
            let text = render_explain(&derived, &resolved, Some(&index));
            eprintln!("--- chain_aware --explain output ---\n{text}");

            // Both nodes show a `derived(chain_aware: ...)` line, matching
            // the design's `--explain` provenance format (§8):
            // `derived(chain_aware: <chain> ... ) -> prio <N>`. Neither has
            // >1 ranked path here, so no "(max over N paths)" note.
            assert!(text.contains("derived(chain_aware:"), "{text}");
            assert!(text.contains("sensing_chain"), "{text}");
            assert!(!text.contains("max over"), "{text}");
            assert!(text.contains("system file: explicit("), "{text}");
            assert!(text.contains("contract[scope 0,"), "{text}");
        });
    }

    #[test]
    fn render_explain_default_node_has_no_provenance_entry_falls_back() {
        let dump = dump_two_plain_nodes();
        let yaml = "\
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived = derive_sched_plan(&dump, None, path, "posix", SchedApplyMode::Warn)
                .expect("derive");
            let resolved = ResolvedPlatformFile {
                path: path.to_path_buf(),
                channel: PlatformFileChannel::Provider,
            };
            let text = render_explain(&derived, &resolved, None);
            assert!(text.contains("default (no timing facts)"), "{text}");
            assert!(text.contains("system file: provider("), "{text}");
        });
    }

    // ── Phase 45.6: model-sourced --explain ──

    /// Phase 45.6 parity test (the brief's core deliverable): render
    /// `--explain` from a fresh derive (the `check` path,
    /// `explain_rows_from_derived`/`render_explain`) and from the SAME
    /// derivation embedded into a `SystemModel` the way `resolve` actually
    /// builds it (`model_builder::build_system_model`, the `resolve`/
    /// `replay --model` path, `explain_rows_from_model`/
    /// `render_explain_from_model`) — the two must show byte-identical
    /// per-node rows (chain-aware provenance included) and, since `resolve`
    /// still has the fresh `ResolvedPlatformFile`/`ManifestIndex` in scope,
    /// a byte-identical footer too.
    #[test]
    fn model_sourced_explain_matches_fresh_derive_for_chain_aware_plan() {
        let dump = dump_two_plain_nodes();
        let index = index_with_chain();
        with_temp_file("yaml", V2_CHAIN_AWARE_BAND, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");
            assert_eq!(derived.mapper, "chain_aware");

            // Build the model exactly the way `resolve` does (same
            // `SchedInputs` shape `model_builder::build_system_model`
            // expects) — no separate/hand-rolled construction.
            let sched_inputs = crate::ros::model_builder::SchedInputs {
                derived: &derived,
                declared_tiers: None,
            };
            let model = crate::ros::model_builder::build_system_model(
                &dump,
                &index,
                Some(&sched_inputs),
                Default::default(),
                &Default::default(),
            );
            assert!(
                model.execution.sched.is_some(),
                "chain_aware mapper must embed execution.sched"
            );

            let resolved = ResolvedPlatformFile {
                path: path.to_path_buf(),
                channel: PlatformFileChannel::Explicit,
            };
            let check_text = render_explain(&derived, &resolved, Some(&index));
            let footer = explain_footer(&resolved, Some(&index));
            let model_text = render_explain_from_model(&model, "posix", &footer);

            eprintln!("--- check --sched --explain ---\n{check_text}");
            eprintln!("--- resolve/replay --model --explain ---\n{model_text}");

            // Chain-aware provenance: exact per-node row parity for both
            // chain members (design requirement: "chain provenance from
            // ranks... must be equivalent").
            for fqn in ["/fast_node", "/slow_node"] {
                let check_row = check_text
                    .lines()
                    .find(|l| l.trim_start().starts_with(fqn))
                    .unwrap_or_else(|| {
                        panic!("check --explain row for {fqn} missing:\n{check_text}")
                    });
                let model_row = model_text
                    .lines()
                    .find(|l| l.trim_start().starts_with(fqn))
                    .unwrap_or_else(|| {
                        panic!("model --explain row for {fqn} missing:\n{model_text}")
                    });
                assert_eq!(check_row, model_row, "row mismatch for {fqn}");
                assert!(check_row.contains("derived(chain_aware:"), "{check_row}");
            }

            // Footer parity: `resolve` has the same `ResolvedPlatformFile`/
            // `ManifestIndex` in scope as `check`, so the footer is
            // byte-identical, not just "equivalent".
            let check_footer: Vec<&str> = check_text
                .lines()
                .skip_while(|l| !l.starts_with("system file:"))
                .collect();
            let model_footer: Vec<&str> = model_text
                .lines()
                .skip_while(|l| !l.starts_with("system file:"))
                .collect();
            assert_eq!(check_footer, model_footer);
        });
    }

    /// Phase 45.6 review, Finding 1 (correctness): an override whose applied
    /// priority COINCIDES with what the mapper would have derived anyway must
    /// still be labeled `override(...)` on the model path — not silently
    /// misclassified as `derived(chain_aware: ...)` by a priority-equality
    /// heuristic. `/fast_node`'s chain_aware-derived priority under
    /// `V2_CHAIN_AWARE_BAND` is 44; pinning it to exactly 44 via an override
    /// is the reviewer's reproduction. `execution.sched.overrides` (the exact
    /// overridden-node set `resolve` embeds) is what makes this exact.
    #[test]
    fn model_sourced_explain_override_coinciding_with_derived_is_still_override() {
        let dump = dump_two_plain_nodes();
        let index = index_with_chain();
        // chain_aware, band 5..45, override fast_node to its OWN derived 44.
        let yaml = "\
target: posix
mapper: chain_aware
resources:
  rt_priority_band: { min: 5, max: 45 }
overrides:
  fast_node: { priority: 44 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");
            // Precondition: the override value really does coincide with the
            // derived rank (else the test isn't exercising the coincidence).
            let rank = derived
                .ranks
                .iter()
                .find(|d| d.node == "/fast_node")
                .expect("fast_node ranked");
            assert_eq!(
                rank.priority, 44,
                "test premise: fast_node's derived rank must equal the override value"
            );
            assert!(
                derived.overrides.contains("/fast_node"),
                "fast_node must be in the derived overrides set"
            );

            let sched_inputs = crate::ros::model_builder::SchedInputs {
                derived: &derived,
                declared_tiers: None,
            };
            let model = crate::ros::model_builder::build_system_model(
                &dump,
                &index,
                Some(&sched_inputs),
                Default::default(),
                &Default::default(),
            );
            // The overridden-node set is embedded in the model.
            assert!(
                model
                    .execution
                    .sched
                    .as_ref()
                    .unwrap()
                    .overrides
                    .contains("/fast_node"),
                "execution.sched.overrides must carry fast_node"
            );

            let resolved = ResolvedPlatformFile {
                path: path.to_path_buf(),
                channel: PlatformFileChannel::Explicit,
            };
            let check_text = render_explain(&derived, &resolved, Some(&index));
            let footer = explain_footer(&resolved, Some(&index));
            let model_text = render_explain_from_model(&model, "posix", &footer);

            let check_row = check_text
                .lines()
                .find(|l| l.trim_start().starts_with("/fast_node"))
                .expect("check fast_node row");
            let model_row = model_text
                .lines()
                .find(|l| l.trim_start().starts_with("/fast_node"))
                .expect("model fast_node row");

            // check labels it override(...) (reads its live overridden map);
            // the model path must match — NOT `derived(chain_aware: ...)`.
            assert!(
                check_row.contains("override(fast_node)"),
                "check row should be override: {check_row}"
            );
            assert!(
                model_row.contains("override(fast_node)"),
                "model row must classify override (Finding 1 fix): {model_row}"
            );
            assert!(
                !model_row.contains("derived(chain_aware:"),
                "model row must NOT misclassify a coinciding override as derived: {model_row}"
            );
            assert_eq!(check_row, model_row, "coincidence-override row must match");
        });
    }

    /// Phase 45.6 review, Finding 2 (accuracy): `deadline_monotonic`'s
    /// derived fact — `min(paths[].max_latency_ms)` — IS embedded in the
    /// model (`execution.sched.requirements`), so the model-path renderer
    /// reproduces `check`'s `"<deadline_us> us deadline → prio <p>"` exactly,
    /// not the generic tier-name fallback.
    #[test]
    fn model_sourced_explain_deadline_monotonic_shows_deadline_fact() {
        use crate::ros::manifest_loader::{ContractChannel, ResolvedManifest, ResolvedNodePath};
        use ros_launch_manifest_types::{PathDecl, Trigger};

        let dump = dump_two_plain_nodes();
        // Two nodes with declared path deadlines (no chains): fast_node 5ms,
        // slow_node 20ms → deadline_monotonic ranks fast_node higher.
        let mut index = ManifestIndex::default();
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/fast_node".to_string(),
            path_name: "p".to_string(),
            path: PathDecl {
                trigger: Some(Trigger::Input(vec!["in".to_string()])),
                max_latency_ms: Some(5.0),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/slow_node".to_string(),
            path_name: "p".to_string(),
            path: PathDecl {
                trigger: Some(Trigger::Input(vec!["in".to_string()])),
                max_latency_ms: Some(20.0),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.manifests.insert(
            0,
            ResolvedManifest {
                scope_id: 0,
                pkg: Some("dm_fixture".to_string()),
                file: "two_nodes.launch.xml".to_string(),
                ns: "/".to_string(),
                channel: ContractChannel::Provider,
                contract_path: PathBuf::from(
                    "/opt/share/dm_fixture/launch/two_nodes.contract.yaml",
                ),
                manifest: Default::default(),
                source: String::new(),
                diagnostics: vec![],
            },
        );
        let yaml = "\
target: posix
mapper: deadline_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
";
        with_temp_file("yaml", yaml, |path| {
            let derived =
                derive_sched_plan(&dump, Some(&index), path, "posix", SchedApplyMode::Warn)
                    .expect("derive");
            assert_eq!(derived.mapper, "deadline_monotonic");
            assert!(
                derived.ranks.is_empty(),
                "deadline_monotonic emits no ranks"
            );

            let sched_inputs = crate::ros::model_builder::SchedInputs {
                derived: &derived,
                declared_tiers: None,
            };
            let model = crate::ros::model_builder::build_system_model(
                &dump,
                &index,
                Some(&sched_inputs),
                Default::default(),
                &Default::default(),
            );

            let resolved = ResolvedPlatformFile {
                path: path.to_path_buf(),
                channel: PlatformFileChannel::Explicit,
            };
            let check_text = render_explain(&derived, &resolved, Some(&index));
            let footer = explain_footer(&resolved, Some(&index));
            let model_text = render_explain_from_model(&model, "posix", &footer);

            for fqn in ["/fast_node", "/slow_node"] {
                let check_row = check_text
                    .lines()
                    .find(|l| l.trim_start().starts_with(fqn))
                    .unwrap_or_else(|| panic!("check row for {fqn}:\n{check_text}"));
                let model_row = model_text
                    .lines()
                    .find(|l| l.trim_start().starts_with(fqn))
                    .unwrap_or_else(|| panic!("model row for {fqn}:\n{model_text}"));
                assert!(
                    check_row.contains("us deadline → prio"),
                    "check row should cite the deadline fact: {check_row}"
                );
                assert_eq!(
                    check_row, model_row,
                    "deadline_monotonic row must match on the model path for {fqn}"
                );
            }
            // Specifically the tighter 5ms → 5000us deadline, embedded and
            // reproduced (not a tier-name fallback).
            assert!(
                model_text.contains("5000 us deadline → prio"),
                "{model_text}"
            );
        });
    }

    /// Backward-compat (45.5/45.6): a model with no `execution.sched` and no
    /// bindings at all (no platform file resolved) — `--explain` must not
    /// panic and must render something graceful (every node falls back to
    /// `Provenance::Default`, matching the `check` path's own "no timing
    /// facts" rows).
    #[test]
    fn model_sourced_explain_graceful_when_sched_less() {
        let model = ros_launch_manifest_model::SystemModel::default();
        let rows = explain_rows_from_model(&model, "posix");
        assert!(
            rows.is_empty(),
            "an empty model (no structure.nodes) has nothing to show: {rows:?}"
        );
        let text = render_explain_from_model(&model, "posix", "system file: (none)\n");
        assert!(text.contains("PROVENANCE"));
        assert!(text.contains("system file: (none)"));
    }

    // ── Phase 41.3: platform-file shipping channels ──

    /// Builds a dump whose only scope is the ROOT launch scope (no parent,
    /// `origin: Some(...)`) — platform-file resolution only ever looks here.
    fn dump_with_root_scope(pkg: Option<&str>, file: &str, path: Option<&Path>) -> LaunchDump {
        let mut origin = serde_json::json!({"file": file});
        if let Some(p) = pkg {
            origin["pkg"] = serde_json::json!(p);
        }
        if let Some(p) = path {
            origin["path"] = serde_json::json!(p.to_string_lossy());
        }
        let json = serde_json::json!({
            "node": [],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {"id": 0, "ns": "/", "parent": null, "origin": origin}
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    #[test]
    fn resolve_platform_file_explicit_bypasses_discovery() {
        let tmp = tempfile::TempDir::new().unwrap();
        let dump = dump_with_root_scope(Some("mypkg"), "bringup.launch.xml", None);

        // Even with an overlay dir present, an explicit path always wins and
        // is returned even though it doesn't exist on disk.
        let overlay_root = tmp.path().join("overlay");
        std::fs::create_dir_all(&overlay_root).unwrap();
        let explicit = tmp.path().join("explicit.system.posix.yaml");

        let resolved =
            resolve_platform_file(&dump, Some(&explicit), Some(&overlay_root), true, "posix")
                .expect("explicit path always resolves");
        assert_eq!(resolved.channel, PlatformFileChannel::Explicit);
        assert_eq!(resolved.path, explicit);
    }

    #[test]
    fn resolve_platform_file_overlay_beats_provider() {
        let tmp = tempfile::TempDir::new().unwrap();
        let root = tmp.path();

        let launch_dir = root.join("install/mypkg/share/mypkg/launch");
        std::fs::create_dir_all(&launch_dir).unwrap();
        let launch_file = launch_dir.join("bringup.launch.xml");
        std::fs::write(&launch_file, "<launch/>").unwrap();
        std::fs::write(
            launch_dir.join("bringup.system.posix.yaml"),
            "target: posix\nmapper: manual\nprovider_marker: true\n",
        )
        .unwrap();

        let overlay_root = root.join("overlay");
        std::fs::create_dir_all(overlay_root.join("mypkg/launch")).unwrap();
        std::fs::write(
            overlay_root.join("mypkg/launch/bringup.system.posix.yaml"),
            "target: posix\nmapper: manual\noverlay_marker: true\n",
        )
        .unwrap();

        let dump = dump_with_root_scope(Some("mypkg"), "bringup.launch.xml", Some(&launch_file));

        let resolved = resolve_platform_file(&dump, None, Some(&overlay_root), true, "posix")
            .expect("overlay file present");
        assert_eq!(resolved.channel, PlatformFileChannel::Overlay);
        assert_eq!(
            resolved.path,
            overlay_root.join("mypkg/launch/bringup.system.posix.yaml")
        );
    }

    #[test]
    fn resolve_platform_file_provider_sidecar_only_works() {
        let tmp = tempfile::TempDir::new().unwrap();
        let root = tmp.path();

        let launch_dir = root.join("install/mypkg/share/mypkg/launch");
        std::fs::create_dir_all(&launch_dir).unwrap();
        let launch_file = launch_dir.join("bringup.launch.xml");
        std::fs::write(&launch_file, "<launch/>").unwrap();
        std::fs::write(
            launch_dir.join("bringup.system.posix.yaml"),
            "target: posix\nmapper: manual\n",
        )
        .unwrap();

        let dump = dump_with_root_scope(Some("mypkg"), "bringup.launch.xml", Some(&launch_file));

        // No overlay root at all — provider sidecar is the only channel.
        let resolved = resolve_platform_file(&dump, None, None, true, "posix")
            .expect("provider sidecar present");
        assert_eq!(resolved.channel, PlatformFileChannel::Provider);
        assert_eq!(resolved.path, launch_dir.join("bringup.system.posix.yaml"));
    }

    #[test]
    fn resolve_platform_file_no_provider_contracts_disables_provider_channel() {
        // W3 review Important #2: `--no-provider-contracts` (provider:
        // false) must gate the platform-file provider channel exactly as it
        // gates contracts — a sidecar that exists on disk is NOT resolved.
        let tmp = tempfile::TempDir::new().unwrap();
        let root = tmp.path();

        let launch_dir = root.join("install/mypkg/share/mypkg/launch");
        std::fs::create_dir_all(&launch_dir).unwrap();
        let launch_file = launch_dir.join("bringup.launch.xml");
        std::fs::write(&launch_file, "<launch/>").unwrap();
        std::fs::write(
            launch_dir.join("bringup.system.posix.yaml"),
            "target: posix\nmapper: manual\n",
        )
        .unwrap();

        let dump = dump_with_root_scope(Some("mypkg"), "bringup.launch.xml", Some(&launch_file));

        // provider: false — the existing sidecar must be ignored.
        assert!(resolve_platform_file(&dump, None, None, false, "posix").is_none());

        // But the overlay channel is unaffected by the provider gate.
        let overlay_root = root.join("overlay");
        std::fs::create_dir_all(overlay_root.join("mypkg/launch")).unwrap();
        std::fs::write(
            overlay_root.join("mypkg/launch/bringup.system.posix.yaml"),
            "target: posix\nmapper: manual\n",
        )
        .unwrap();
        let resolved = resolve_platform_file(&dump, None, Some(&overlay_root), false, "posix")
            .expect("overlay resolves regardless of the provider gate");
        assert_eq!(resolved.channel, PlatformFileChannel::Overlay);
    }

    #[test]
    fn resolve_platform_file_wrong_target_is_ignored() {
        let tmp = tempfile::TempDir::new().unwrap();
        let root = tmp.path();

        let launch_dir = root.join("install/mypkg/share/mypkg/launch");
        std::fs::create_dir_all(&launch_dir).unwrap();
        let launch_file = launch_dir.join("bringup.launch.xml");
        std::fs::write(&launch_file, "<launch/>").unwrap();
        // Only a posix file is shipped — a zephyr request must not match it.
        std::fs::write(
            launch_dir.join("bringup.system.posix.yaml"),
            "target: posix\nmapper: manual\n",
        )
        .unwrap();

        let dump = dump_with_root_scope(Some("mypkg"), "bringup.launch.xml", Some(&launch_file));

        assert!(resolve_platform_file(&dump, None, None, true, "zephyr").is_none());
    }

    #[test]
    fn resolve_platform_file_none_when_nothing_resolves() {
        let tmp = tempfile::TempDir::new().unwrap();
        let launch_dir = tmp.path().join("install/mypkg/share/mypkg/launch");
        std::fs::create_dir_all(&launch_dir).unwrap();
        let launch_file = launch_dir.join("bringup.launch.xml");
        std::fs::write(&launch_file, "<launch/>").unwrap();
        // No .system.<target>.yaml file anywhere (overlay or provider).

        let dump = dump_with_root_scope(Some("mypkg"), "bringup.launch.xml", Some(&launch_file));
        let overlay_root = tmp.path().join("overlay");

        assert!(resolve_platform_file(&dump, None, Some(&overlay_root), true, "posix").is_none());
        assert!(resolve_platform_file(&dump, None, None, true, "posix").is_none());
    }

    #[test]
    fn resolve_platform_file_none_when_root_scope_has_no_origin_path() {
        // Root scope with an origin but no recorded `path` (old record.json,
        // or unresolved origin) — provider lookup can't determine a
        // directory; overlay still works if configured, but here it isn't.
        let dump = dump_with_root_scope(Some("mypkg"), "bringup.launch.xml", None);
        assert!(resolve_platform_file(&dump, None, None, true, "posix").is_none());
    }
}
