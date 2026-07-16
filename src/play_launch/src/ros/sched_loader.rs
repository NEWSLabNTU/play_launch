//! Load + validate the shared scheduling spec against a parsed launch dump.
//!
//! Linux is "validate now, apply later": we resolve for the `posix` target and
//! report, but do not (yet) apply `sched_setscheduler`/affinity to processes.

use std::{
    collections::HashMap,
    fmt,
    path::{Path, PathBuf},
};

use eyre::Result;
use ros_launch_manifest_sched::{
    MapError, MapperRegistry, PlatformResources, ResolvedTier, ResolvedTierTable, SchedNode,
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
const KNOWN_MAPPERS: &[&str] = &["manual", "rate_monotonic", "deadline_monotonic"];

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
        tracing::warn!("{msg}");
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

    let derived = mapper.map(&input, &file.resources).map_err(|e: MapError| {
        eyre::eyre!("mapper `{}` failed to derive a plan: {e}", file.mapper)
    })?;

    let mut plan = flatten_to_one_tier_per_node(&derived);
    let mut warnings = Vec::new();
    apply_overrides(&mut plan, &file.overrides, &mut warnings);

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
                        tracing::warn!("{msg}");
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
    // reported, never fatal.
    for c in rate_priority_contradictions(&input, &plan)
        .into_iter()
        .chain(deadline_priority_contradictions(&input, &plan))
    {
        let msg = format!(
            "scheduling: `{}` vs `{}` priority order contradicts their `{}` order (check the \
             contract and platform-file overrides)",
            c.node_a, c.node_b, c.kind,
        );
        tracing::warn!("{msg}");
        warnings.push(msg);
    }

    Ok(DerivedSchedPlan {
        plan,
        target: file.target,
        mapper: file.mapper,
        warnings,
    })
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
) -> Result<()> {
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
    for w in &derived.warnings {
        eprintln!("  warning: {w}");
    }
    Ok(())
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
fn resolve_platform_overlay_path(
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
fn resolve_platform_provider_path(
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
fn root_scope(dump: &LaunchDump) -> Option<&crate::ros::launch_dump::ScopeEntry> {
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
/// Returns `None` when nothing resolves in any channel — the caller's
/// existing "no `--sched`" behavior (scheduling disabled) applies unchanged.
pub fn resolve_platform_file(
    dump: &LaunchDump,
    explicit: Option<&Path>,
    overlay_root: Option<&Path>,
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

    if let Some(path) = resolve_platform_provider_path(scope, target)
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
         (overlay_root={overlay_root:?}) — scheduling disabled"
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
        use crate::ros::manifest_loader::ResolvedTopic;
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

        let resolved = resolve_platform_file(&dump, Some(&explicit), Some(&overlay_root), "posix")
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

        let resolved = resolve_platform_file(&dump, None, Some(&overlay_root), "posix")
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
        let resolved =
            resolve_platform_file(&dump, None, None, "posix").expect("provider sidecar present");
        assert_eq!(resolved.channel, PlatformFileChannel::Provider);
        assert_eq!(resolved.path, launch_dir.join("bringup.system.posix.yaml"));
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

        assert!(resolve_platform_file(&dump, None, None, "zephyr").is_none());
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

        assert!(resolve_platform_file(&dump, None, Some(&overlay_root), "posix").is_none());
        assert!(resolve_platform_file(&dump, None, None, "posix").is_none());
    }

    #[test]
    fn resolve_platform_file_none_when_root_scope_has_no_origin_path() {
        // Root scope with an origin but no recorded `path` (old record.json,
        // or unresolved origin) — provider lookup can't determine a
        // directory; overlay still works if configured, but here it isn't.
        let dump = dump_with_root_scope(Some("mypkg"), "bringup.launch.xml", None);
        assert!(resolve_platform_file(&dump, None, None, "posix").is_none());
    }
}
