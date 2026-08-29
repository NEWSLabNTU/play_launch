//! Manifest loading and resolution using the scope table from record.json.
//!
//! For each file scope in the launch tree, resolves a contract YAML file via
//! the overlay/provider channels (see `ContractSources`), parses it, runs
//! static checks, and builds a resolved index with fully-qualified
//! topic/node names.

use super::launch_dump::{LaunchDump, ScopeEntry};
use super::sched_loader::scheduled_records_from_dump;
use ros_launch_manifest_check::{Diagnostic, Severity, run_checks_with_spans};
use ros_launch_manifest_types::{
    EffectiveTrigger, Manifest, filter_manifest, parse_manifest_with_spans, resolve_args,
    substitute_manifest,
};
use std::{
    collections::{BTreeMap, HashMap},
    fmt,
    path::{Path, PathBuf},
};
use tracing::{debug, info, warn};

/// Which of the two resolution channels supplied a scope's contract.
///
/// Resolution order (first hit wins): overlay > provider. See
/// `docs/superpowers/specs/2026-07-15-contract-shipping-design.md` §Resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContractChannel {
    /// `<overlay-root>/<pkg>/launch/<stem>.contract.yaml` (`--contracts <dir>`).
    Overlay,
    /// `<launch-file-dir>/<stem>.contract.yaml`, shipped next to the launch
    /// file (on by default; disable with `--no-provider-contracts`).
    Provider,
}

impl fmt::Display for ContractChannel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ContractChannel::Overlay => "overlay",
            ContractChannel::Provider => "provider",
        })
    }
}

/// Configuration for the two-step contract resolution.
///
/// For every file scope, `load_manifests` tries each configured channel in
/// order (overlay, then provider) and uses the first file that exists on
/// disk.
#[derive(Debug, Clone, Default)]
pub struct ContractSources {
    /// Overlay root (`--contracts <dir>`). `None` disables the overlay channel.
    pub overlay: Option<PathBuf>,
    /// Whether the provider-sidecar channel is enabled (on by default;
    /// `--no-provider-contracts` sets this to `false`).
    pub provider: bool,
}

/// Discover the overlay root shared by contracts and platform files (Phase
/// 41.3 design §3.2).
///
/// First existing of, in order (no cross-root merging):
/// 1. `explicit` (the `--contracts <dir>` flag) — when given, the rest are
///    not consulted. It MUST exist: naming a directory that is not there is
///    a user error, and the previous behaviour of accepting it silently
///    meant a typo produced "0 overlay, 1 provider" and exit 0, so a CI gate
///    reported success having validated against the vendor's provider
///    contract rather than the overlay it was told to use. `--sched`
///    already errors on a missing explicit path; this matches it.
/// 2. `$PLAY_LAUNCH_CONTRACTS` — skipped when it does not name a directory.
///    Deliberately lenient, unlike `--contracts`: it is often exported
///    globally in a shell profile, so a machine without that directory must
///    not fail every invocation. The flag is per-command and unambiguous;
///    the variable is ambient.
/// 3. `$XDG_CONFIG_HOME/play_launch/contracts` (default
///    `~/.config/play_launch/contracts`, when `XDG_CONFIG_HOME` is unset)
/// 4. `/etc/play_launch/contracts`
///
/// Candidates 3-4 are ambient discovery: they are skipped when absent, which
/// is the point of them. Returns `Ok(None)` when nothing resolves — both the
/// contract and platform-file overlay channels are then disabled.
pub fn discover_overlay_root(explicit: Option<&Path>) -> eyre::Result<Option<PathBuf>> {
    if let Some(p) = explicit {
        if !p.is_dir() {
            return Err(eyre::eyre!(
                "--contracts {} is not a directory.\n\
                 \n\
                 The overlay root must exist. Create it, or drop --contracts to \
                 use the provider contracts shipped next to the launch file.",
                p.display()
            ));
        }
        debug!("overlay root: explicit --contracts {}", p.display());
        return Ok(Some(p.to_path_buf()));
    }

    if let Ok(v) = std::env::var("PLAY_LAUNCH_CONTRACTS") {
        let p = PathBuf::from(v);
        if p.is_dir() {
            debug!("overlay root: $PLAY_LAUNCH_CONTRACTS {}", p.display());
            return Ok(Some(p));
        }
    }

    // Per the XDG Base Directory spec, an empty $XDG_CONFIG_HOME must be
    // treated as unset (fall back to ~/.config).
    let xdg_base = std::env::var("XDG_CONFIG_HOME")
        .ok()
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var("HOME")
                .ok()
                .map(|h| PathBuf::from(h).join(".config"))
        });
    if let Some(base) = xdg_base {
        let p = base.join("play_launch").join("contracts");
        if p.is_dir() {
            debug!("overlay root: XDG_CONFIG_HOME {}", p.display());
            return Ok(Some(p));
        }
    }

    let etc = PathBuf::from("/etc/play_launch/contracts");
    if etc.is_dir() {
        debug!("overlay root: /etc {}", etc.display());
        return Ok(Some(etc));
    }

    debug!("overlay root: none of the standard locations exist (no overlay)");
    Ok(None)
}

/// A loaded and resolved manifest bound to a specific scope.
#[derive(Debug, Clone)]
#[allow(dead_code)] // Fields used by 31.5 runtime monitors and 31.6 audit
pub struct ResolvedManifest {
    /// Scope ID this manifest is bound to.
    pub scope_id: usize,
    /// Package name (from scope origin).
    pub pkg: Option<String>,
    /// Launch file name (from scope origin).
    pub file: String,
    /// Namespace prefix applied to relative names.
    pub ns: String,
    /// Which resolution channel supplied this scope's contract file.
    pub channel: ContractChannel,
    /// The actual contract file resolved for this scope (whichever channel
    /// supplied it).
    pub contract_path: PathBuf,
    /// The raw parsed manifest.
    pub manifest: Manifest,
    /// Original YAML source text (for codespan-reporting).
    pub source: String,
    /// Static check diagnostics.
    pub diagnostics: Vec<Diagnostic>,
}

/// Resolved topic entry with fully-qualified names.
///
/// A topic may be declared in multiple scopes across the manifest tree
/// (e.g., the publisher's manifest declares it with full contract, and
/// each subscriber's manifest declares the type for standalone checking).
/// The checker merges declarations: contract fields (`type`, `rate_hz`,
/// `qos`) must agree, while endpoint lists are combined.
#[derive(Debug, Clone)]
#[allow(dead_code)] // Fields used by 31.5 runtime monitors and 31.6 audit
pub struct ResolvedTopic {
    /// Fully-qualified topic name (with namespace prefix).
    pub fqn: String,
    /// Message type (must agree across all declaring scopes).
    pub msg_type: String,
    /// QoS declaration (must agree across all declaring scopes if declared).
    pub qos: Option<ros_launch_manifest_types::QosDecl>,
    /// Publisher endpoint FQNs (merged from all declaring scopes).
    pub publishers: Vec<String>,
    /// Subscriber endpoint FQNs (merged from all declaring scopes).
    pub subscribers: Vec<String>,
    /// Expected rate (Hz, must agree across all declaring scopes if declared).
    pub rate_hz: Option<f64>,
    /// Rate DERIVED by propagating from the timers that drive this topic —
    /// kept beside `rate_hz` rather than overwriting it, so a consumer can
    /// always tell an author's claim from the graph's consequence. `None`
    /// when the rate is not derivable (`once`, `spontaneous`, an external
    /// publisher, a cycle) — never zero, which would be a claim.
    pub derived_rate_hz: Option<f64>,
    /// Worst-case transport latency (ms, must agree across all declaring scopes).
    pub max_transport_ms: Option<f64>,
    /// Drop tolerance (must agree across all declaring scopes if declared).
    pub drop: Option<ros_launch_manifest_types::DropSpec>,
    /// All scope IDs that declared this topic.
    pub scope_ids: Vec<usize>,
}

/// Resolved service entry with fully-qualified names.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct ResolvedService {
    /// Fully-qualified service name (with namespace prefix).
    pub fqn: String,
    /// Service type (must agree across all declaring scopes).
    pub srv_type: String,
    /// Server endpoint FQNs (merged).
    pub servers: Vec<String>,
    /// Client endpoint FQNs (merged).
    pub clients: Vec<String>,
    /// All scope IDs that declared this service.
    pub scope_ids: Vec<usize>,
}

/// Resolved node path entry.
#[derive(Debug, Clone)]
#[allow(dead_code)] // Fields used by 31.5 runtime monitors
pub struct ResolvedNodePath {
    /// Fully-qualified node name.
    pub node_fqn: String,
    /// Path name.
    pub path_name: String,
    /// The path declaration.
    pub path: ros_launch_manifest_types::PathDecl,
    /// Scope ID.
    pub scope_id: usize,
}

/// Resolved scope-level path with topic names as input/output.
///
/// Scope paths declare an end-to-end timing budget across the scope's
/// subtree. Input and output are topic names (relative or absolute);
/// the checker resolves them using the scope's namespace.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct ResolvedScopePath {
    /// Scope ID this path belongs to.
    pub scope_id: usize,
    /// Path name (local to the scope).
    pub path_name: String,
    /// Resolved input topic FQNs (entry points).
    pub input_topics: Vec<String>,
    /// Resolved output topic FQNs (exit points).
    pub output_topics: Vec<String>,
    /// The original path declaration (latency, drops, correlation, etc.).
    pub path: ros_launch_manifest_types::PathDecl,
}

/// The complete resolved manifest index for the launch tree.
#[derive(Debug, Default, Clone)]
pub struct ManifestIndex {
    /// Manifests by scope ID.
    pub manifests: HashMap<usize, ResolvedManifest>,
    /// All resolved topics (FQN → topic info, merged across scopes).
    pub topics: BTreeMap<String, ResolvedTopic>,
    /// All resolved services (FQN → service info, merged across scopes).
    pub services: BTreeMap<String, ResolvedService>,
    /// R1-P2 — all resolved actions (FQN → info, merged across scopes;
    /// same shape as services: type must agree, endpoint lists union).
    /// Previously the loader silently dropped `actions:` — the model's
    /// `structure.actions` was always empty.
    pub actions: BTreeMap<String, ResolvedService>,
    /// All resolved node paths.
    pub node_paths: Vec<ResolvedNodePath>,
    /// Resolved scope-level paths (input/output as resolved topic FQNs).
    pub scope_paths: Vec<ResolvedScopePath>,
    /// Parent scope ID for each scope (from launch tree). None for root.
    pub scope_parents: HashMap<usize, Option<usize>>,
    /// Externally-provided topics, keyed by FQN. Built from every
    /// manifest's `external_topics:` block plus per-topic `external:`
    /// flags. Each entry records which side of the topic is provided
    /// by an external system. `dangling-entity` skips that side.
    pub externals: BTreeMap<String, ros_launch_manifest_types::ExternalSide>,
    /// Cross-scope consistency diagnostics (collected after merge).
    pub merge_diagnostics: Vec<Diagnostic>,
    /// Total errors across all manifests.
    pub total_errors: usize,
    /// Total warnings across all manifests.
    pub total_warnings: usize,
    /// Bare node identity -> launch-dump-verified FQN, keyed by
    /// `(scope_id, bare_node_name)`.
    ///
    /// Contracts only ever declare a node by its bare name
    /// (`nodes.<name>`) — they carry no `namespace=` attribute, since
    /// that's a launch-file-only concept. Naively qualifying a contract
    /// node as `scope.ns + bare_name` silently diverges from the node's
    /// *real* fully-qualified name whenever the launch file gives the node
    /// its own `namespace=` attribute that overrides the declaring scope's
    /// namespace (see `sched_loader::fqn_for`'s precedence rule: own
    /// namespace wins over scope). That divergence used to mean a chain's
    /// derived priorities (and any other contract-identity-keyed fact)
    /// would silently attach to a name no real process ever answers to.
    ///
    /// Built once per [`load_manifests`] call from the real launch dump's
    /// [`super::sched_loader::ScheduledRecord`]s (the same identity the
    /// apply layer keys on) and consulted by [`resolve_node_fqn`] /
    /// [`resolve_endpoint_ref`] — the single reconciliation point every
    /// consumer (`resolve_node_paths`/`resolve_topics`/`resolve_services`
    /// here, `manifest_graph::build_global_graph`, `causal_graph::build_export`)
    /// goes through, so they can never disagree about a node's identity.
    /// Falls back to naive `scope.ns + bare_name` qualification when the
    /// dump has no matching record (no launch dump available, or the
    /// contract names a node the dump doesn't contain).
    pub node_identity: HashMap<(usize, String), String>,
}

/// Walk from `scope_id` up through `parent` links until a file scope
/// (`origin: Some(...)`) is found (inclusive — if `scope_id` itself is
/// already a file scope, it's returned unchanged). Returns `None` if the
/// chain runs out (root reached with no file scope, or a dangling/unknown
/// scope id) — the caller then indexes the record only under its own
/// immediate scope id.
///
/// This is the walk Fix B (W6 report §4c) needs: a node stamped with a
/// bare `<group>`'s scope id must reconcile under the *file* scope's id
/// too, since contract resolution (`resolve_node_paths` -> `resolve_node_fqn`)
/// only ever queries `node_identity` keyed by the manifest's own file scope
/// id, never by an intervening group's id.
fn nearest_file_scope_id(
    scopes_by_id: &HashMap<usize, &ScopeEntry>,
    scope_id: usize,
) -> Option<usize> {
    let mut current = scope_id;
    loop {
        let scope = scopes_by_id.get(&current)?;
        if scope.is_file_scope() {
            return Some(current);
        }
        current = scope.parent?;
    }
}

/// Load manifests for all file scopes in the launch tree.
///
/// For each scope with a known `(pkg, file)` origin, resolves the contract
/// file via the two-step channel order (overlay > provider, see
/// `ContractSources`) and loads the first one found. Scopes without a
/// matching contract file in any enabled channel are silently skipped.
pub fn load_manifests(
    launch_dump: &LaunchDump,
    sources: &ContractSources,
) -> eyre::Result<ManifestIndex> {
    let mut index = ManifestIndex::default();
    let mut loaded = 0usize;
    let mut skipped = 0usize;
    let mut overlay_loaded = 0usize;
    let mut provider_loaded = 0usize;

    // Snapshot scope tree (parent links) for cross-scope checks.
    // Includes all scopes (file and group), not just those with manifests.
    for scope in &launch_dump.scopes {
        index.scope_parents.insert(scope.id, scope.parent);
    }

    // Map every scope id to its `ScopeEntry` (for `is_file_scope()`/`parent`
    // walks below) — group-nesting reconciliation needs to know which
    // ancestor scope ids are file scopes, not just their parent links.
    let scopes_by_id: HashMap<usize, &ScopeEntry> =
        launch_dump.scopes.iter().map(|s| (s.id, s)).collect();

    // Build the contract-identity -> launch-dump-FQN reconciliation map
    // (see `ManifestIndex::node_identity` doc) before any node/topic/
    // service/path resolution below, so every resolution in this function
    // (and every downstream consumer that reads `index.node_identity`)
    // agrees on the same node identity.
    //
    // A record's own `scope_id` is its *immediate* enclosing scope, which
    // may be a bare (non-`<include>`) `<group>` — the parser stamps those
    // with their own distinct scope id, one level deeper than the file
    // scope contract resolution (`resolve_node_fqn`) actually looks up
    // under. Index the record under BOTH its immediate scope id (for any
    // consumer that happens to have that id in hand) AND its nearest
    // ancestor FILE scope id (what `resolve_node_paths`/`resolve_node_fqn`
    // actually query) — see W6 report §4c / Fix B.
    for record in scheduled_records_from_dump(launch_dump) {
        let Some(scope_id) = record.scope_id else {
            continue;
        };
        let mut keys = vec![scope_id];
        if let Some(file_scope_id) = nearest_file_scope_id(&scopes_by_id, scope_id)
            && file_scope_id != scope_id
        {
            keys.push(file_scope_id);
        }
        for key_scope_id in keys {
            if let Some(prev) = index
                .node_identity
                .insert((key_scope_id, record.bare_name.clone()), record.fqn.clone())
                && prev != record.fqn
            {
                warn!(
                    "ambiguous node identity: two schedulable records named `{}` in scope {} \
                     ({prev} vs {}); contract facts for this name may be misattributed",
                    record.bare_name, key_scope_id, record.fqn
                );
            }
        }
    }

    for scope in &launch_dump.scopes {
        if !scope.is_file_scope() {
            continue;
        }

        let Some((path, channel)) = resolve_contract_path(scope, sources) else {
            debug!(
                "No manifest for scope {} ({}/{}): no channel matched",
                scope.id,
                scope.pkg().unwrap_or("?"),
                scope.file().unwrap_or("?"),
            );
            skipped += 1;
            continue;
        };

        debug!(
            "Loading manifest for scope {} ({}/{}) via {channel}: {}",
            scope.id,
            scope.pkg().unwrap_or("?"),
            scope.file().unwrap_or("?"),
            path.display()
        );

        let parsed = match parse_manifest_with_spans(&path) {
            Ok(p) => p,
            Err(e) => {
                warn!("Failed to parse manifest {}: {}", path.display(), e);
                continue;
            }
        };

        let source = parsed.source;

        // Resolve args: merge scope args over manifest defaults, then substitute $(var ...)
        let manifest = if parsed.manifest.args.is_empty() {
            parsed.manifest
        } else {
            let resolved_args = match resolve_args(&parsed.manifest.args, &scope.args) {
                Ok(a) => a,
                Err(e) => {
                    warn!(
                        "Failed to resolve args for scope {} ({}/{}): {}",
                        scope.id,
                        scope.pkg().unwrap_or("?"),
                        scope.file().unwrap_or("?"),
                        e
                    );
                    continue;
                }
            };
            match substitute_manifest(&parsed.manifest, &resolved_args) {
                Ok(m) => m,
                Err(e) => {
                    warn!(
                        "Failed to substitute args for scope {} ({}/{}): {}",
                        scope.id,
                        scope.pkg().unwrap_or("?"),
                        scope.file().unwrap_or("?"),
                        e
                    );
                    continue;
                }
            }
        };

        // Filter entities by if:/unless: conditions (after substitution)
        let mut manifest = manifest;
        filter_manifest(&mut manifest);

        // Run static checks with span resolution.
        // Drop per-manifest `dangling-entity` and `service-wiring`
        // diagnostics — when running in cross-scope mode (load_manifests),
        // the cross-scope index is the authoritative source for those
        // checks. Per-manifest emission produces O(n) duplicate warnings
        // for legitimate sub-only / pub-only / cross-scope-served entries.
        let mut check_result = run_checks_with_spans(&manifest, parsed.spans);
        check_result
            .diagnostics
            .retain(|d| d.rule_id != "dangling-entity" && d.rule_id != "service-wiring");

        // Log diagnostics
        for diag in &check_result.diagnostics {
            let scope_label = format!(
                "[scope {} {}/{}]",
                scope.id,
                scope.pkg().unwrap_or("?"),
                scope.file().unwrap_or("?")
            );
            match diag.severity {
                Severity::Error => warn!("{scope_label} {diag}"),
                Severity::Warning => debug!("{scope_label} {diag}"),
                Severity::Info => debug!("{scope_label} {diag}"),
            }
        }

        let errors = check_result
            .diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
            .count();
        let warnings = check_result
            .diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Warning)
            .count();
        index.total_errors += errors;
        index.total_warnings += warnings;

        // Resolve names with namespace prefix and merge across scopes
        resolve_topics(&manifest, scope, &mut index);
        resolve_services(&manifest, scope, &mut index);
        resolve_actions(&manifest, scope, &mut index);
        resolve_node_paths(&manifest, scope, &mut index);
        resolve_scope_paths(&manifest, scope, &mut index);

        index.manifests.insert(
            scope.id,
            ResolvedManifest {
                scope_id: scope.id,
                pkg: scope.pkg().map(String::from),
                file: scope.file().unwrap_or("").to_string(),
                ns: scope.ns.clone(),
                channel,
                contract_path: path.clone(),
                manifest,
                source,
                diagnostics: check_result.diagnostics,
            },
        );

        match channel {
            ContractChannel::Overlay => overlay_loaded += 1,
            ContractChannel::Provider => provider_loaded += 1,
        }
        loaded += 1;
    }

    // Cross-scope post-merge checks
    run_cross_scope_checks(&mut index);

    // Fold merge diagnostics into the total counts and log them
    for diag in &index.merge_diagnostics {
        match diag.severity {
            Severity::Error => {
                index.total_errors += 1;
                warn!("[cross-scope] {diag}");
            }
            Severity::Warning => {
                index.total_warnings += 1;
                debug!("[cross-scope] {diag}");
            }
            Severity::Info => debug!("[cross-scope] {diag}"),
        }
    }

    if loaded > 0 {
        info!(
            "Loaded {loaded} manifest(s) [{overlay_loaded} overlay, {provider_loaded} provider] \
             ({skipped} scopes without manifests, {} errors, {} warnings)",
            index.total_errors, index.total_warnings
        );
    } else {
        debug!(
            "No manifests found (overlay={:?}, provider={})",
            sources.overlay, sources.provider
        );
    }

    Ok(index)
}

/// Run validation checks that require the merged cross-scope view.
///
/// After all manifests are loaded and merged, this checks for:
/// - Topics with 0 publishers across the entire tree (dangling sub)
/// - Services with 0 servers across the entire tree (dangling client)
/// - Cross-scope path budget-overflow (parent vs child paths matched
///   by resolved (input, output) topics)
/// - Critical-path latency for each scope path (Phase 35.3/35.4):
///   builds the global dataflow graph, traces actual paths between
///   scope-path input and output topics within the scope subtree,
///   and reports if declared latency is less than the critical path.
/// - Rate hierarchy on merged topics (Phase 35.5): publisher
///   `min_rate_hz` from one scope vs subscriber `min_rate_hz` in another.
fn run_cross_scope_checks(index: &mut ManifestIndex) {
    use super::manifest_graph::build_global_graph;
    use ros_launch_manifest_types::ExternalSide;

    // Build the merged externals map from every loaded manifest's
    // top-level `external_topics:` block plus any per-topic `external:`
    // flag on individual topic decls. Keys are resolved FQNs (qualified
    // by the declaring scope's namespace).
    collect_externals(index);

    // Cross-check external_topics type against internal topic type.
    // When both an `external_topics:` entry and a `topics:` entry of
    // the same FQN exist with `type:` set, they must agree.
    check_external_topic_consistency(index);

    // Dangling topic: 0 publishers across the merged tree
    for (fqn, topic) in &index.topics {
        let ext = index.externals.get(fqn).copied();
        if topic.publishers.is_empty() && !topic.subscribers.is_empty() {
            if matches!(ext, Some(ExternalSide::Pub | ExternalSide::Both)) {
                continue;
            }
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "dangling-entity".to_string(),
                severity: Severity::Warning,
                message: format!(
                    "topic '{}' has 0 publishers across the manifest tree (declared in {} scope(s)) — \
                     may be published by an external system",
                    fqn,
                    topic.scope_ids.len()
                ),
                path: format!("topics.{fqn}"),
                span: None,
            });
        }
        if !topic.publishers.is_empty() && topic.subscribers.is_empty() {
            if matches!(ext, Some(ExternalSide::Sub | ExternalSide::Both)) {
                continue;
            }
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "dangling-entity".to_string(),
                severity: Severity::Warning,
                message: format!(
                    "topic '{}' has 0 subscribers across the manifest tree (declared in {} scope(s)) — \
                     may be consumed by an external system",
                    fqn,
                    topic.scope_ids.len()
                ),
                path: format!("topics.{fqn}"),
                span: None,
            });
        }
    }

    // Dangling service: 0 servers across the merged tree
    for (fqn, service) in &index.services {
        if service.servers.is_empty() && !service.clients.is_empty() {
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "dangling-entity".to_string(),
                severity: Severity::Error,
                message: format!(
                    "service '{}' has 0 servers across the manifest tree (declared in {} scope(s))",
                    fqn,
                    service.scope_ids.len()
                ),
                path: format!("services.{fqn}"),
                span: None,
            });
        }
    }

    // Cross-scope path budget-overflow: when a child scope and an ancestor
    // scope declare paths with the same resolved (input, output) topics,
    // the child's max_latency_ms must not exceed the ancestor's.
    check_path_budget_overflow(index);
    check_concurrency_declaration(index);
    check_sync_window_budget(index);

    // Build the global dataflow graph once and reuse it for the
    // critical-path and rate-hierarchy checks.
    let graph = build_global_graph(index);

    // Cross-scope causal-DAG check (Phase 42 finding): a cycle spanning
    // multiple contract files is invisible to the per-manifest `causal-dag`
    // rule, which only sees one manifest's own `DataflowGraph` at a time.
    // Advisory (warning), not error — see `causal_dag_global` module docs.
    super::causal_dag_global::check_causal_dag_global(index, &graph);

    // Critical-path latency check via the global dataflow graph.
    check_scope_path_critical_path(index, &graph);
    check_route_exclusion_blocking(index, &graph);
    check_lifespan_against_age(index, &graph);

    // Cross-scope rate hierarchy: publisher rate vs subscriber demand
    // even when pub and sub live in different manifests.
    check_cross_scope_rate_hierarchy(index, &graph);

    // Cross-scope QoS pub/sub compatibility: every (pub, sub) edge in
    // the merged graph runs through the DDS offered ≥ requested matrix
    // (Issue #45). Per-endpoint QoS overrides are overlaid on the
    // topic-level default before comparison.
    check_cross_scope_qos_match(index, &graph);

    // Rate propagation: derive every topic's rate from the timers that drive
    // it, record it, and report a declared `rate_hz` that the graph already
    // implies.
    derive_and_check_topic_rates(index, &graph);

    // A service server's promised response time, against the blocking its own
    // declarations imply.
    check_response_against_blocking(index);
}

/// `response-blocking`: a service server promising a response faster than its
/// own longest callback can finish.
///
/// This is a one-line blocking argument, not a response-time analysis (which
/// `contract-axes.md` deliberately leaves out of scope). Both `rclcpp`'s
/// implicit callback group and nano-ros's `default_cbg_type` are **mutually
/// exclusive**, so a request arriving just after a 20 ms callback starts waits
/// for that callback to finish before the service handler runs at all. A node
/// that declares `max_response: 5ms` alongside a 20 ms path has therefore
/// promised something its own declarations rule out — no priority assignment
/// helps, because the blocking is on the same thread.
///
/// Skipped when the node claims concurrency (phase 67's `concurrency:`),
/// which is exactly what that vocabulary is for: an author who says these
/// paths may run at once is saying the exclusive-group premise does not hold.
/// Skipped, too, when nothing is comparable — a server with no declared path
/// budgets gets no verdict rather than a favourable one.
fn check_response_against_blocking(index: &mut ManifestIndex) {
    let mut findings: Vec<(String, String, f64, String, f64)> = Vec::new();

    for resolved in index.manifests.values() {
        for (node_name, node) in &resolved.manifest.nodes {
            if node.concurrency.is_some() {
                continue;
            }
            let longest = node
                .paths
                .iter()
                .filter_map(|(name, p)| {
                    p.max_latency.map(|d| (name.clone(), d.as_millis_f64()))
                })
                .fold(None, |acc: Option<(String, f64)>, v| match acc {
                    Some(a) if a.1 >= v.1 => Some(a),
                    _ => Some(v),
                });
            let Some((slow_path, slow_ms)) = longest else {
                continue;
            };
            for (srv_name, props) in &node.srv {
                let Some(max_response) = props.max_response.map(|d| d.as_millis_f64()) else {
                    continue;
                };
                if slow_ms >= max_response {
                    findings.push((
                        node_name.clone(),
                        srv_name.clone(),
                        max_response,
                        slow_path.clone(),
                        slow_ms,
                    ));
                }
            }
        }
    }

    findings.sort_by(|a, b| (&a.0, &a.1).cmp(&(&b.0, &b.1)));
    for (node, srv, max_response, slow_path, slow_ms) in findings {
        index.merge_diagnostics.push(Diagnostic {
            rule_id: "response-blocking".to_string(),
            severity: Severity::Warning,
            message: format!(
                "node '{node}' promises service '{srv}' a response within {max_response}ms, but \
                 its own path '{slow_path}' declares {slow_ms}ms. Callbacks default to a \
                 MUTUALLY EXCLUSIVE group in both rclcpp and nano-ros, so a request arriving \
                 just after '{slow_path}' starts waits for it to finish — the promise is ruled \
                 out by the node's own declarations, and no priority assignment changes it. \
                 Declare `concurrency:` if these really can run at once"
            ),
            path: format!("nodes.{node}.srv.{srv}.max_response"),
            span: None,
        });
    }
}

/// Propagate rates from timer boundaries, store the result on each topic, and
/// report a hand-written `rate_hz` the graph already accounts for.
///
/// A rate is a consequence, not a fact: a timer path publishes at its own
/// rate and an input-triggered path publishes at the rate its inputs arrive,
/// so the number on `topics.<t>.rate_hz` is a second copy of something the
/// declarations already determine. Measured on `rt_workspace`, where the
/// number 100 appears nine times: three of those copies are this field.
///
/// Two diagnostics, and the difference between them is the entire argument for
/// deriving rather than declaring:
///
/// - **`rate-mismatch`** (warning) — the author's number and the derived one
///   DISAGREE. One of them is wrong and nothing else in the toolchain would
///   have noticed.
/// - **`derivable-rate`** (info) — they agree, so the declaration is
///   redundant and can be deleted.
///
/// A topic whose rate cannot be derived gets NEITHER, and its declared value
/// stands: `once`, `spontaneous`, an externally-driven publisher and a
/// feedback cycle are all genuine absences of information, and a contract is
/// the only place that number can come from. That is why the derivation
/// returns a reason rather than a zero.
fn derive_and_check_topic_rates(
    index: &mut ManifestIndex,
    graph: &super::manifest_graph::GlobalDataflowGraph,
) {
    use super::manifest_graph::{DerivedRate, derive_topic_rates};

    let derived = derive_topic_rates(index, graph);
    let mut findings: Vec<(String, f64, Option<f64>)> = Vec::new();

    for (fqn, rate) in &derived {
        let DerivedRate::Hz(hz) = rate else {
            continue;
        };
        if let Some(topic) = index.topics.get_mut(fqn) {
            topic.derived_rate_hz = Some(*hz);
            findings.push((fqn.clone(), *hz, topic.rate_hz));
        }
    }

    findings.sort_by(|a, b| a.0.cmp(&b.0));
    for (fqn, hz, declared) in findings {
        let Some(declared) = declared else {
            continue;
        };
        // A relative tolerance, because a derived rate is a quotient
        // (`1000.0 / period`) and an authored one is a round number: 29.97 and
        // 30 disagreeing to the seventh decimal is not a finding.
        let differs = (declared - hz).abs() > (declared.abs().max(hz.abs()) * 1e-6);
        if differs {
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "rate-mismatch".to_string(),
                severity: Severity::Warning,
                message: format!(
                    "topic '{fqn}' declares rate_hz {declared} but the graph derives \
                     {hz:.4} Hz from the timers that drive it. One of the two is wrong, \
                     and nothing else checks this — a declared rate is a second copy of \
                     something the `trigger:`/`output:` facts already determine"
                ),
                path: format!("topics.{fqn}.rate_hz"),
                span: None,
            });
        } else {
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "derivable-rate".to_string(),
                severity: Severity::Info,
                message: format!(
                    "topic '{fqn}' declares rate_hz {declared}, which the graph already \
                     derives from the timers that drive it. The declaration is redundant \
                     and can be deleted"
                ),
                path: format!("topics.{fqn}.rate_hz"),
                span: None,
            });
        }
    }
}

/// Build the global dataflow graph and verify each scope path's
/// declared `max_latency_ms` against the critical path between its
/// input and output topics within the scope's subtree.
///
/// This is the topology-aware replacement for the per-manifest
/// `scope-budget` sum check (which was incorrect for parallel branches).
fn check_scope_path_critical_path(
    index: &mut ManifestIndex,
    graph: &super::manifest_graph::GlobalDataflowGraph,
) {
    use super::manifest_graph::{critical_path, subgraph_for_scope_path, subtree_scope_ids};

    // Snapshot scope paths to avoid borrow conflict with merge_diagnostics.
    let scope_paths = index.scope_paths.clone();
    let mut superseded: Vec<(usize, String)> = Vec::new();

    for scope_path in &scope_paths {
        let Some(declared) = scope_path.path.max_latency.map(|d| d.as_millis_f64()) else {
            continue;
        };

        let subtree = subtree_scope_ids(index, scope_path.scope_id);
        let subgraph = subgraph_for_scope_path(
            graph,
            subtree,
            &scope_path.input_topics,
            &scope_path.output_topics,
        );

        let Some(cp) = critical_path(&subgraph) else {
            continue;
        };

        // The topology-aware verdict SUPERSEDES the per-manifest sum for this
        // path. The fallback sums every node in the manifest, which its own
        // rule documents as "pessimistic for parallel (fork-join) topologies";
        // once a real route exists, leaving both in place reports two
        // different totals for one path and sends an author chasing the wrong
        // one. Suppressed only where a route was actually computed — a scope
        // path with no traceable route keeps the fallback, which is then the
        // only estimate available.
        superseded.push((scope_path.scope_id, scope_path.path_name.clone()));

        // `jitter-feasibility`: a clock crossing contributes its WHOLE period
        // to end-to-end jitter, whatever the callback costs — a message
        // arriving just after a tick waits a full period, one arriving just
        // before waits none. So sampling jitter alone can exceed a declared
        // `max_jitter`, and when it does no scheduling assignment can help:
        // the only fixes are a faster boundary rate or a looser requirement.
        // Same structural-infeasibility shape as `chain-sampling-feasibility`.
        //
        // This is the half of the jitter requirement that needs no best-case
        // fact. Execution jitter needs `min_latency`, which is optional and
        // usually measured; sampling jitter is derivable from the trigger
        // alone.
        if let Some(max_jitter) = scope_path.path.max_jitter.map(|d| d.as_millis_f64())
            && cp.sampling_cost_ms > max_jitter
        {
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "jitter-feasibility".to_string(),
                severity: Severity::Warning,
                message: format!(
                    "scope path '{}' (scope {}) declares max_jitter ({max_jitter:.2}ms) below \
                     the sampling jitter its own route already carries ({:.2}ms, one period \
                     per clock boundary crossed: {}). No priority assignment can reduce this \
                     — raise the boundary's rate or the requirement",
                    scope_path.path_name,
                    scope_path.scope_id,
                    cp.sampling_cost_ms,
                    cp.nodes.join(" → "),
                ),
                path: format!("paths.{}.max_jitter", scope_path.path_name),
                span: None,
            });
        }

        // `scope-sampling-feasibility`: the sampling term ALONE meeting or
        // exceeding the budget is a different and worse statement than the
        // total exceeding it. A total over budget may be fixable by
        // scheduling; a sampling cost over budget cannot be fixed by any
        // priority assignment, because it is the time the route spends waiting
        // for clocks rather than running. The only fixes are a faster boundary
        // or a looser budget.
        //
        // Carried over from the retired `chain-sampling-feasibility`, which
        // said the same thing about an authored chain. Emitted BEFORE
        // `scope-budget` so the structural verdict reads first — the budget
        // warning necessarily fires too, and on its own it invites an author
        // to go optimise callbacks that are not the problem.
        if cp.sampling_cost_ms >= declared {
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "scope-sampling-feasibility".to_string(),
                severity: Severity::Warning,
                message: format!(
                    "scope path '{}' (scope {}) is structurally infeasible: sampling cost \
                     alone ({:.2}ms, one period per clock boundary crossed: {}) meets or \
                     exceeds the declared max_latency ({declared}ms). No priority assignment \
                     can reduce this — raise the boundary's rate or the budget",
                    scope_path.path_name,
                    scope_path.scope_id,
                    cp.sampling_cost_ms,
                    cp.nodes.join(" \u{2192} "),
                ),
                path: format!("paths.{}", scope_path.path_name),
                span: None,
            });
        }

        if cp.total_ms > declared {
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "scope-budget".to_string(),
                severity: Severity::Warning,
                message: format!(
                    "scope path '{}' (scope {}) max_latency_ms ({}) is less than \
                     critical path: {} = {:.2}ms{}",
                    scope_path.path_name,
                    scope_path.scope_id,
                    declared,
                    cp.nodes.join(" → "),
                    cp.total_ms,
                    // Name the sampling term when there is one, the way
                    // `chain-budget` does: it is the part of the total that no
                    // priority assignment can remove, so an author reading
                    // "reduce this" needs to know how much of it is not theirs
                    // to reduce.
                    if cp.sampling_cost_ms > 0.0 {
                        format!(
                            " ({:.2}ms event-segment + {:.2}ms sampling_cost)",
                            cp.total_ms - cp.sampling_cost_ms,
                            cp.sampling_cost_ms
                        )
                    } else {
                        String::new()
                    },
                ),
                path: format!("paths.{}", scope_path.path_name),
                span: None,
            });
        }
    }

    // Drop the per-manifest sum for every path a real route was found for.
    // The fallback sums EVERY node in the manifest — its own rule documents
    // that as "pessimistic for parallel (fork-join) topologies" — so once a
    // route exists, leaving both in place reports two different totals for one
    // path and sends an author chasing the wrong one. Suppressed only where a
    // route was computed: a path with no traceable route keeps the fallback,
    // which is then the only estimate there is.
    for (scope_id, path_name) in superseded {
        let key = format!("paths.{path_name}");
        if let Some(m) = index.manifests.get_mut(&scope_id) {
            let before = m.diagnostics.len();
            m.diagnostics
                .retain(|d| !(d.rule_id == "scope-budget" && d.path == key));
            let dropped = before - m.diagnostics.len();
            index.total_warnings = index.total_warnings.saturating_sub(dropped);
        }
    }
}

/// `path-exclusion` — report that a route's total omits blocking it is
/// subject to.
///
/// The critical path sums the latency of each path it crosses, which assumes
/// each runs the moment its message arrives. That is false when the traversed
/// path shares a callback group with another path of the same node: the two
/// serialise, so an arriving message can wait for a sibling already running.
///
/// This **reports** rather than computes. Deciding whether summing is VALID
/// needs only the exclusion relation; bounding the blocking needs a
/// response-time analysis, and the published work reports UNBOUNDED worst-case
/// response times for some chains under the default ROS 2 executor — a result
/// about ROS 2, not about this arithmetic. Turning a silent unsound assumption
/// into a visible one is the part that matters here.
///
/// Only fires where the sibling has a declared `max_latency`, so the note can
/// say how much is at stake rather than gesturing at it.
fn check_route_exclusion_blocking(
    index: &mut ManifestIndex,
    graph: &super::manifest_graph::GlobalDataflowGraph,
) {
    use super::manifest_graph::{critical_path, subgraph_for_scope_path, subtree_scope_ids};

    let scope_paths = index.scope_paths.clone();
    let mut findings: Vec<(String, usize, Vec<String>)> = Vec::new();

    for scope_path in &scope_paths {
        let subtree = subtree_scope_ids(index, scope_path.scope_id);
        let subgraph = subgraph_for_scope_path(
            graph,
            subtree,
            &scope_path.input_topics,
            &scope_path.output_topics,
        );
        let Some(cp) = critical_path(&subgraph) else {
            continue;
        };

        let mut notes: Vec<String> = Vec::new();
        for (node_fqn, path_name) in &cp.vertices {
            let (Some(node), Some(path)) = (graph.nodes.get(node_fqn), path_name.as_deref()) else {
                continue;
            };
            let blockers: Vec<(String, f64)> = node
                .exclusive_siblings_of(path)
                .into_iter()
                .filter_map(|sib| {
                    node.paths
                        .get(&sib)
                        .and_then(|p| p.max_latency)
                        .map(|d| (sib, d.as_millis_f64()))
                })
                .collect();
            if blockers.is_empty() {
                continue;
            }
            let worst = blockers.iter().map(|(_, ms)| *ms).fold(0.0_f64, f64::max);
            let names: Vec<&str> = blockers.iter().map(|(n, _)| n.as_str()).collect();
            notes.push(format!(
                "{node_fqn}/{path} may wait for [{}] (up to {worst:.2}ms)",
                names.join(", ")
            ));
        }
        if !notes.is_empty() {
            findings.push((scope_path.path_name.clone(), scope_path.scope_id, notes));
        }
    }

    for (path_name, scope_id, notes) in findings {
        index.merge_diagnostics.push(Diagnostic {
            rule_id: "path-exclusion".to_string(),
            severity: Severity::Info,
            message: format!(
                "scope path '{path_name}' (scope {scope_id}): the critical path assumes each \
                 node runs on arrival, but these traversals serialise with a sibling callback \
                 and may be delayed by it — {}. Declare `concurrency.exclusive` on the node if \
                 they can in fact run concurrently; absent that declaration every path of a \
                 node is assumed to serialise, matching rclcpp's default callback group",
                notes.join("; ")
            ),
            path: format!("paths.{path_name}"),
            span: None,
        });
    }
}

/// Verify the rate hierarchy across merged topics.
///
/// For each topic with `rate_hz` declared:
/// - Each publisher endpoint's `min_rate_hz` must be >= `rate_hz`
///   (publisher must produce at least as fast as the channel rate)
/// - The effective delivery rate `rate_hz * (1 - max_drop_rate)` must
///   be >= each subscriber endpoint's `min_rate_hz` (subscriber demand)
///
/// Looks up endpoint properties via the global graph's `GlobalNode` map,
/// which carries each node's pub/sub `EndpointProps`. This makes the
/// check work across scopes — a publisher's rate from `localization.yaml`
/// is checked against a subscriber's demand in `control.yaml`.
fn check_cross_scope_rate_hierarchy(
    index: &mut ManifestIndex,
    graph: &super::manifest_graph::GlobalDataflowGraph,
) {
    // Snapshot topic FQNs to avoid borrow conflict with merge_diagnostics.
    let topics: Vec<(String, ResolvedTopic)> = index
        .topics
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();

    for (fqn, topic) in &topics {
        let Some(rate) = topic.rate_hz else {
            continue;
        };

        // Compute effective delivery after max_drop_rate
        let drop_rate = topic
            .drop
            .as_ref()
            .and_then(|d| d.max_count.as_ref())
            .map(|c| c.drop_rate())
            .unwrap_or(0.0);
        let effective_delivery = rate * (1.0 - drop_rate);

        // Check each publisher endpoint's min_rate_hz against rate_hz
        for pub_ref in &topic.publishers {
            let Some((node_fqn, ep_name)) = split_endpoint_ref_for_check(pub_ref) else {
                continue;
            };
            let Some(node) = graph.nodes.get(&node_fqn) else {
                continue;
            };
            let Some(props) = node.publishers.get(&ep_name) else {
                continue;
            };
            let Some(pub_min) = props.min_rate_hz else {
                continue;
            };
            if pub_min < rate {
                index.merge_diagnostics.push(Diagnostic {
                    rule_id: "rate-hierarchy".to_string(),
                    severity: Severity::Error,
                    message: format!(
                        "topic '{fqn}' rate_hz ({rate}) > publisher '{pub_ref}' \
                         min_rate_hz ({pub_min}) — publisher cannot sustain channel rate"
                    ),
                    path: format!("topics.{fqn}.rate_hz"),
                    span: None,
                });
            }
        }

        // Check effective delivery against each subscriber's min_rate_hz
        for sub_ref in &topic.subscribers {
            let Some((node_fqn, ep_name)) = split_endpoint_ref_for_check(sub_ref) else {
                continue;
            };
            let Some(node) = graph.nodes.get(&node_fqn) else {
                continue;
            };
            let Some(props) = node.subscribers.get(&ep_name) else {
                continue;
            };
            // Skip state subscribers — they read latest, no rate guarantee needed
            if props.state.unwrap_or(false) {
                continue;
            }
            let Some(sub_min) = props.min_rate_hz else {
                continue;
            };
            if effective_delivery < sub_min {
                index.merge_diagnostics.push(Diagnostic {
                    rule_id: "rate-hierarchy".to_string(),
                    severity: Severity::Error,
                    message: format!(
                        "topic '{fqn}' effective delivery rate ({effective_delivery:.2} \
                         = {rate} × (1 - {drop_rate:.3})) < subscriber '{sub_ref}' \
                         min_rate_hz ({sub_min})"
                    ),
                    path: format!("topics.{fqn}.rate_hz"),
                    span: None,
                });
            }
        }
    }
}

/// Cross-check `external_topics:` entries against the merged
/// `index.topics`. When an external entry declares a `type:` that
/// disagrees with an internal `topics:` declaration of the same FQN,
/// emit a `consistency` error.
fn check_external_topic_consistency(index: &mut ManifestIndex) {
    let mut diags = Vec::new();
    for resolved in index.manifests.values() {
        let ns = &resolved.ns;
        for (key, decl) in &resolved.manifest.external_topics {
            let Some(ext_type) = &decl.msg_type else {
                continue;
            };
            let fqn = qualify_name(ns, key);
            let Some(internal) = index.topics.get(&fqn) else {
                continue;
            };
            if internal.msg_type != *ext_type {
                diags.push(Diagnostic {
                    rule_id: "consistency".to_string(),
                    severity: Severity::Error,
                    message: format!(
                        "topic '{fqn}' external_topics type '{ext_type}' (in scope {}) \
                         disagrees with internal type '{}'",
                        resolved.scope_id, internal.msg_type
                    ),
                    path: format!("external_topics.{key}.type"),
                    span: None,
                });
            }
        }
    }
    index.merge_diagnostics.extend(diags);
}

/// Build the merged `externals` map from every loaded manifest's
/// top-level `external_topics:` block and per-topic `external:` flag.
/// Keys are qualified to FQN against the declaring scope's namespace.
/// When the same FQN is declared multiple times with different sides,
/// the more permissive side wins (`Both` ≥ either single side).
fn collect_externals(index: &mut ManifestIndex) {
    use ros_launch_manifest_types::ExternalSide;
    let mut externals: BTreeMap<String, ExternalSide> = BTreeMap::new();

    let merge =
        |externals: &mut BTreeMap<String, ExternalSide>, fqn: String, side: ExternalSide| {
            let merged = match externals.get(&fqn).copied() {
                None => side,
                Some(existing) if existing == side => existing,
                Some(_) => ExternalSide::Both,
            };
            externals.insert(fqn, merged);
        };

    for resolved in index.manifests.values() {
        let ns = &resolved.ns;
        for (key, decl) in &resolved.manifest.external_topics {
            let fqn = qualify_name(ns, key);
            merge(&mut externals, fqn, decl.side);
        }
        for (key, topic) in &resolved.manifest.topics {
            if let Some(side) = topic.external {
                let fqn = qualify_name(ns, key);
                merge(&mut externals, fqn, side);
            }
        }
    }

    index.externals = externals;
}

/// Split an endpoint FQN like `/ns/node/endpoint` into `(node_fqn, endpoint_name)`.
fn split_endpoint_ref_for_check(ep_ref: &str) -> Option<(String, String)> {
    let pos = ep_ref.rfind('/')?;
    let node = &ep_ref[..pos];
    let ep = &ep_ref[pos + 1..];
    if node.is_empty() || ep.is_empty() {
        return None;
    }
    Some((node.to_string(), ep.to_string()))
}

/// Cross-scope QoS pub/sub compatibility check (Issue #45).
///
/// For every merged topic, walk every (publisher, subscriber) pair,
/// compute the effective QoS for each endpoint by overlaying its
/// per-endpoint override on the topic-level default, and apply the
/// DDS offered ≥ requested matrix on `reliability` and `durability`.
/// A field is checked only when both sides have it specified —
/// unspecified fields are skipped (no implicit ROS 2 default is
/// assumed).
fn check_cross_scope_qos_match(
    index: &mut ManifestIndex,
    graph: &super::manifest_graph::GlobalDataflowGraph,
) {
    use ros_launch_manifest_types::QosDecl;

    // Snapshot topics to avoid borrow conflict with merge_diagnostics.
    let topics: Vec<(String, ResolvedTopic)> = index
        .topics
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();

    for (fqn, topic) in &topics {
        let topic_qos = topic.qos.as_ref();

        // Resolve every publisher endpoint to (ref, effective_qos).
        let mut pubs: Vec<(String, QosDecl)> = Vec::new();
        for pub_ref in &topic.publishers {
            let Some((node_fqn, ep_name)) = split_endpoint_ref_for_check(pub_ref) else {
                continue;
            };
            let ep_qos = graph
                .nodes
                .get(&node_fqn)
                .and_then(|n| n.publishers.get(&ep_name))
                .and_then(|p| p.qos.as_ref());
            pubs.push((pub_ref.clone(), QosDecl::effective(topic_qos, ep_qos)));
        }

        let mut subs: Vec<(String, QosDecl)> = Vec::new();
        for sub_ref in &topic.subscribers {
            let Some((node_fqn, ep_name)) = split_endpoint_ref_for_check(sub_ref) else {
                continue;
            };
            let ep_qos = graph
                .nodes
                .get(&node_fqn)
                .and_then(|n| n.subscribers.get(&ep_name))
                .and_then(|p| p.qos.as_ref());
            subs.push((sub_ref.clone(), QosDecl::effective(topic_qos, ep_qos)));
        }

        for (pub_ref, pub_qos) in &pubs {
            for (sub_ref, sub_qos) in &subs {
                if let (Some(pub_rel), Some(sub_rel)) = (
                    pub_qos.reliability.as_deref(),
                    sub_qos.reliability.as_deref(),
                ) && !reliability_compatible(pub_rel, sub_rel)
                {
                    index.merge_diagnostics.push(Diagnostic {
                        rule_id: "qos-match".to_string(),
                        severity: Severity::Error,
                        message: format!(
                            "incompatible QoS on topic '{fqn}' field 'reliability': \
                             pub '{pub_ref}' offers '{pub_rel}', sub '{sub_ref}' \
                             requests '{sub_rel}'"
                        ),
                        path: format!("topics.{fqn}.qos.reliability"),
                        span: None,
                    });
                }

                if let (Some(pub_dur), Some(sub_dur)) =
                    (pub_qos.durability.as_deref(), sub_qos.durability.as_deref())
                    && !durability_compatible(pub_dur, sub_dur)
                {
                    index.merge_diagnostics.push(Diagnostic {
                        rule_id: "qos-match".to_string(),
                        severity: Severity::Error,
                        message: format!(
                            "incompatible QoS on topic '{fqn}' field 'durability': \
                             pub '{pub_ref}' offers '{pub_dur}', sub '{sub_ref}' \
                             requests '{sub_dur}'"
                        ),
                        path: format!("topics.{fqn}.qos.durability"),
                        span: None,
                    });
                }
            }
        }
    }
}

/// DDS reliability compatibility — offered must be at least as strong
/// as requested. `reliable` ≥ `best_effort`. Unknown values pass; the
/// per-manifest `qos-compat` rule reports invalid value tokens.
fn reliability_compatible(pub_v: &str, sub_v: &str) -> bool {
    match (pub_v, sub_v) {
        ("reliable", "reliable") => true,
        ("reliable", "best_effort") => true,
        ("best_effort", "reliable") => false,
        ("best_effort", "best_effort") => true,
        _ => true,
    }
}

/// DDS durability compatibility — offered must be at least as strong
/// as requested. `transient_local` ≥ `volatile`. Unknown values pass.
fn durability_compatible(pub_v: &str, sub_v: &str) -> bool {
    match (pub_v, sub_v) {
        ("transient_local", "transient_local") => true,
        ("transient_local", "volatile") => true,
        ("volatile", "transient_local") => false,
        ("volatile", "volatile") => true,
        _ => true,
    }
}

/// Match scope paths across the scope tree by resolved (input, output)
/// topic identity. When a parent and child both declare a path with
/// the same boundaries, the child's budget must not exceed the parent's.
/// `sync-budget` — a fan-in path's synchronisation window must fit inside its
/// own latency budget.
///
/// The COMPLEMENT of the manifest crate's `sync-feasibility`, which bounds the
/// same window from below (it must be wide enough to span the slowest input's
/// period). This bounds it from above: whatever the synchroniser waits for is
/// time the path has already spent, so a window wider than the path's
/// `max_latency` cannot be met however the system is scheduled.
///
/// Reads the three window fields that had no consumer: `sync.timeout`,
/// `sync.max_interval` and the path's `tolerance`.
fn check_sync_window_budget(index: &mut ManifestIndex) {
    let paths = index.node_paths.clone();
    for np in &paths {
        let Some(budget) = np.path.max_latency.map(|d| d.as_millis_f64()) else {
            continue;
        };
        // Each window is a lower bound on how long this path can take.
        let mut windows: Vec<(&str, f64)> = Vec::new();
        if let Some(sync) = &np.path.sync {
            if let Some(t) = sync.timeout {
                windows.push(("sync.timeout", t.as_millis_f64()));
            }
            if let Some(mi) = sync.max_interval {
                windows.push(("sync.max_interval", mi.as_millis_f64()));
            }
        }
        if let Some(tol) = np.path.tolerance {
            windows.push(("tolerance", tol.as_millis_f64()));
        }
        for (field, window) in windows {
            if window <= budget {
                continue;
            }
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "sync-budget".to_string(),
                severity: Severity::Warning,
                message: format!(
                    "path '{}' on {} declares {field} ({window:.2}ms) wider than its own \
                     max_latency ({budget:.2}ms). The synchroniser may wait that long before \
                     the callback even starts, so the budget cannot be met by any scheduling \
                     assignment — this is a contradiction in the declaration, not a \
                     performance problem",
                    np.path_name, np.node_fqn,
                ),
                path: format!("nodes.{}.paths.{}", np.node_fqn, np.path_name),
                span: None,
            });
        }
    }
}

/// `lifespan-age` — a subscriber cannot require data older than the topic's
/// `lifespan` allows to exist.
///
/// DDS discards a sample once it is older than `lifespan`, so an endpoint
/// asking for data up to `max_age` old on a topic with a shorter lifespan is
/// asking for samples the middleware has already thrown away. The two
/// declarations contradict each other and no runtime behaviour can satisfy
/// both.
///
/// `lifespan` had exactly three read sites before this — a `model_builder`
/// copy, a merge-equality check and a deprecation lint — and so could not fail
/// anything.
fn check_lifespan_against_age(
    index: &mut ManifestIndex,
    graph: &super::manifest_graph::GlobalDataflowGraph,
) {
    use ros_launch_manifest_types::QosDecl;
    let topics: Vec<(String, Option<QosDecl>, Vec<String>)> = index
        .topics
        .iter()
        .map(|(fqn, t)| (fqn.clone(), t.qos.clone(), t.subscribers.clone()))
        .collect();

    let mut findings: Vec<(String, String, f64, f64)> = Vec::new();
    for (topic_fqn, topic_qos, subscribers) in &topics {
        for ep_ref in subscribers {
            let Some(pos) = ep_ref.rfind('/') else {
                continue;
            };
            let (node_fqn, ep) = (&ep_ref[..pos], &ep_ref[pos + 1..]);
            let Some(props) = graph
                .nodes
                .get(node_fqn)
                .and_then(|n| n.subscribers.get(ep))
            else {
                continue;
            };
            let Some(max_age) = props.max_age.map(|d| d.as_millis_f64()) else {
                continue;
            };
            let effective = QosDecl::effective(topic_qos.as_ref(), props.qos.as_ref());
            let Some(lifespan) = effective.lifespan.map(|d| d.as_millis_f64()) else {
                continue;
            };
            if max_age > lifespan {
                findings.push((ep_ref.clone(), topic_fqn.clone(), max_age, lifespan));
            }
        }
    }

    for (ep_ref, topic, max_age, lifespan) in findings {
        index.merge_diagnostics.push(Diagnostic {
            rule_id: "lifespan-age".to_string(),
            severity: Severity::Warning,
            message: format!(
                "subscriber '{ep_ref}' accepts data up to {max_age:.2}ms old, but topic \
                 '{topic}' has lifespan {lifespan:.2}ms — DDS discards a sample once it is \
                 older than its lifespan, so the data this endpoint is willing to use no \
                 longer exists to deliver"
            ),
            path: format!("topics.{topic}.qos.lifespan"),
            span: None,
        });
    }
}

/// `concurrency-decl` — every path named in `concurrency.exclusive` must be a
/// path this node declares.
///
/// Without this the field is decorative: a typo'd or stale path name silently
/// forms a group of one, which changes nothing and reports nothing. The whole
/// point of adding vocabulary is that it is checked — an unchecked field is
/// indistinguishable from a comment.
fn check_concurrency_declaration(index: &mut ManifestIndex) {
    let mut findings: Vec<(usize, String, String, Vec<String>)> = Vec::new();
    for resolved in index.manifests.values() {
        for (node_name, node) in &resolved.manifest.nodes {
            let Some(decl) = &node.concurrency else {
                continue;
            };
            let unknown: Vec<String> = decl
                .exclusive
                .iter()
                .flatten()
                .filter(|p| !node.paths.contains_key(*p))
                .cloned()
                .collect();
            if !unknown.is_empty() {
                let declared: Vec<String> = node.paths.keys().cloned().collect();
                findings.push((
                    resolved.scope_id,
                    node_name.clone(),
                    declared.join(", "),
                    unknown,
                ));
            }
        }
    }
    for (scope_id, node_name, declared, unknown) in findings {
        index.merge_diagnostics.push(Diagnostic {
            rule_id: "concurrency-decl".to_string(),
            severity: Severity::Error,
            message: format!(
                "node '{node_name}' (scope {scope_id}) declares concurrency.exclusive over \
                 path(s) it does not have: {}. Declared paths: [{declared}]. A name that \
                 matches no path forms a group of one, which silently changes nothing",
                unknown.join(", "),
            ),
            path: format!("nodes.{node_name}.concurrency.exclusive"),
            span: None,
        });
    }
}

fn check_path_budget_overflow(index: &mut ManifestIndex) {
    // Build a parent-chain lookup: for each scope ID, the set of all
    // ancestor scope IDs (transitive parents).
    let mut ancestors_of: HashMap<usize, Vec<usize>> = HashMap::new();
    for resolved in index.manifests.values() {
        let chain = ancestor_chain(index, resolved.scope_id);
        ancestors_of.insert(resolved.scope_id, chain);
    }

    // Snapshot scope paths to avoid borrow conflicts
    let paths: Vec<ResolvedScopePath> = index.scope_paths.clone();

    // For each pair (child, ancestor) of paths with matching (input, output),
    // verify child budget ≤ ancestor budget.
    for child in &paths {
        let Some(child_lat) = child.path.max_latency.map(|d| d.as_millis_f64()) else {
            continue;
        };
        let Some(ancestors) = ancestors_of.get(&child.scope_id) else {
            continue;
        };
        for ancestor in &paths {
            if !ancestors.contains(&ancestor.scope_id) {
                continue;
            }
            let Some(ancestor_lat) = ancestor.path.max_latency.map(|d| d.as_millis_f64()) else {
                continue;
            };
            if !path_endpoints_match(child, ancestor) {
                continue;
            }
            if child_lat > ancestor_lat {
                index.merge_diagnostics.push(Diagnostic {
                    rule_id: "budget-overflow".to_string(),
                    severity: Severity::Error,
                    message: format!(
                        "scope path '{}' (scope {}) max_latency_ms ({}) exceeds ancestor \
                         path '{}' (scope {}) max_latency_ms ({}) — child budget cannot \
                         exceed parent budget on the same (input, output) topics",
                        child.path_name,
                        child.scope_id,
                        child_lat,
                        ancestor.path_name,
                        ancestor.scope_id,
                        ancestor_lat,
                    ),
                    path: format!("paths.{}", child.path_name),
                    span: None,
                });
            }
        }
    }
}

/// Build the ancestor chain for a scope ID by walking parent links.
/// Returns scope IDs from immediate parent to root (excluding the scope itself).
fn ancestor_chain(index: &ManifestIndex, scope_id: usize) -> Vec<usize> {
    let mut chain = Vec::new();
    let mut current = scope_id;
    while let Some(parent_opt) = index.scope_parents.get(&current) {
        match parent_opt {
            Some(parent_id) => {
                chain.push(*parent_id);
                current = *parent_id;
            }
            None => break,
        }
    }
    chain
}

/// Check if two scope paths have matching resolved (input, output) topics.
fn path_endpoints_match(a: &ResolvedScopePath, b: &ResolvedScopePath) -> bool {
    if a.scope_id == b.scope_id {
        return false;
    }
    if a.input_topics.len() != b.input_topics.len() {
        return false;
    }
    if a.output_topics.len() != b.output_topics.len() {
        return false;
    }
    for t in &a.input_topics {
        if !b.input_topics.contains(t) {
            return false;
        }
    }
    for t in &a.output_topics {
        if !b.output_topics.contains(t) {
            return false;
        }
    }
    true
}

/// Strip a launch file's extension to get its contract stem, shared by all
/// three resolution channels. `bringup.launch.xml` → `bringup`, likewise for
/// `.launch.py` and `.launch.yaml` (and the bare `.launch`).
pub(crate) fn contract_stem(file: &str) -> &str {
    file.strip_suffix(".launch.xml")
        .or_else(|| file.strip_suffix(".launch.py"))
        .or_else(|| file.strip_suffix(".launch.yaml"))
        .or_else(|| file.strip_suffix(".launch"))
        .unwrap_or(file)
}

/// Resolve the user-overlay contract path for a given scope.
///
/// Layout: `<overlay_root>/<pkg>/launch/<stem>.contract.yaml`
/// If pkg is None, looks in `<overlay_root>/_/launch/<stem>.contract.yaml`.
///
/// `pub(crate)`: also reused by `commands::contract::handle_contract_eject`
/// (Phase 41.4) to compute the destination path for an ejected contract.
pub fn resolve_overlay_path(scope: &ScopeEntry, overlay_root: &Path) -> Option<PathBuf> {
    let file = scope.file()?;
    let stem = contract_stem(file);
    let pkg_dir = scope.pkg().unwrap_or("_");
    Some(
        overlay_root
            .join(pkg_dir)
            .join("launch")
            .join(format!("{stem}.contract.yaml")),
    )
}

/// Resolve the provider-sidecar contract path for a given scope.
///
/// Layout: `<launch-file-dir>/<stem>.contract.yaml`, where the directory
/// comes from the scope origin's canonicalized launch-file path (Phase 40
/// / W1). Returns `None` when the scope has no recorded path (older
/// `record.json` produced before W1, or an origin the parser couldn't
/// resolve to an absolute path) — provider lookup is simply unavailable
/// for that scope.
///
/// `pub(crate)`: also reused by `commands::contract::handle_contract_eject`
/// (Phase 41.4) to find the provider contract to eject.
pub fn resolve_provider_path(scope: &ScopeEntry) -> Option<PathBuf> {
    let origin = scope.origin.as_ref()?;
    let launch_path = origin.path.as_deref()?;
    let dir = Path::new(launch_path).parent()?;
    let stem = contract_stem(&origin.file);
    Some(dir.join(format!("{stem}.contract.yaml")))
}

/// Resolve the contract file for a scope using the two-step channel
/// order: overlay > provider. First channel whose candidate file exists
/// on disk wins; returns the resolved path plus which channel supplied it.
fn resolve_contract_path(
    scope: &ScopeEntry,
    sources: &ContractSources,
) -> Option<(PathBuf, ContractChannel)> {
    // 1. Overlay — <overlay-root>/<pkg>/launch/<stem>.contract.yaml
    if let Some(overlay_root) = &sources.overlay
        && let Some(path) = resolve_overlay_path(scope, overlay_root)
        && path.exists()
    {
        return Some((path, ContractChannel::Overlay));
    }

    // 2. Provider — <launch-file-dir>/<stem>.contract.yaml
    if sources.provider {
        match resolve_provider_path(scope) {
            Some(path) if path.exists() => return Some((path, ContractChannel::Provider)),
            Some(_) => {}
            None => {
                debug!(
                    "scope {} ({}/{}): provider lookup skipped — no launch-file path \
                     recorded (record.json predates Phase 40 or origin path unresolved)",
                    scope.id,
                    scope.pkg().unwrap_or("?"),
                    scope.file().unwrap_or("?"),
                );
            }
        }
    }

    None
}

/// Resolve and merge topic declarations from this scope into the index.
///
/// If a topic with the same resolved FQN already exists in the index
/// (from another scope), merge the declarations: validate that contract
/// fields agree, union the publisher/subscriber endpoint lists, and
/// emit `consistency` diagnostics on mismatches.
fn resolve_topics(manifest: &Manifest, scope: &ScopeEntry, index: &mut ManifestIndex) {
    let ns = scope.ns.clone();

    for (topic_name, topic_decl) in &manifest.topics {
        let fqn = qualify_name(&ns, topic_name);

        // Qualify publisher/subscriber endpoint references, reconciling
        // the node part against the launch dump (see `resolve_endpoint_ref`).
        let mut publishers: Vec<String> = Vec::new();
        for ep_ref in &topic_decl.publishers {
            publishers.push(resolve_endpoint_ref(index, scope.id, &ns, ep_ref));
        }
        let mut subscribers: Vec<String> = Vec::new();
        for ep_ref in &topic_decl.subscribers {
            subscribers.push(resolve_endpoint_ref(index, scope.id, &ns, ep_ref));
        }

        if let Some(existing) = index.topics.get_mut(&fqn) {
            // Merge with existing declaration — validate agreement
            merge_topic(
                existing,
                scope,
                topic_name,
                topic_decl,
                publishers,
                subscribers,
                &mut index.merge_diagnostics,
            );
        } else {
            index.topics.insert(
                fqn.clone(),
                ResolvedTopic {
                    fqn,
                    msg_type: topic_decl.msg_type.clone(),
                    qos: topic_decl.qos.clone(),
                    publishers,
                    subscribers,
                    rate_hz: topic_decl.rate_hz,
                    // Filled in by `derive_and_check_topic_rates` once the
                    // whole tree is merged — a rate propagates across scopes,
                    // so it cannot be known while one scope is being read.
                    derived_rate_hz: None,
                    max_transport_ms: topic_decl.max_transport.map(|d| d.as_millis_f64()),
                    drop: topic_decl.drop.clone(),
                    scope_ids: vec![scope.id],
                },
            );
        }
    }
}

/// Merge a new topic declaration into an existing ResolvedTopic entry.
/// Validates contract fields agree, unions endpoint lists, emits diagnostics.
fn merge_topic(
    existing: &mut ResolvedTopic,
    scope: &ScopeEntry,
    topic_name: &str,
    decl: &ros_launch_manifest_types::TopicDecl,
    publishers: Vec<String>,
    subscribers: Vec<String>,
    diagnostics: &mut Vec<Diagnostic>,
) {
    // Validate type agreement (required field, must always match)
    if existing.msg_type != decl.msg_type {
        diagnostics.push(Diagnostic {
            rule_id: "consistency".to_string(),
            severity: Severity::Error,
            message: format!(
                "topic '{}' type mismatch: '{}' (existing) vs '{}' in scope {} ({}/{})",
                existing.fqn,
                existing.msg_type,
                decl.msg_type,
                scope.id,
                scope.pkg().unwrap_or("?"),
                scope.file().unwrap_or("?"),
            ),
            path: format!("topics.{topic_name}.type"),
            span: None,
        });
    }

    // Validate rate_hz agreement (only when both declared)
    if let (Some(a), Some(b)) = (existing.rate_hz, decl.rate_hz) {
        if (a - b).abs() > f64::EPSILON {
            diagnostics.push(Diagnostic {
                rule_id: "consistency".to_string(),
                severity: Severity::Error,
                message: format!(
                    "topic '{}' rate_hz mismatch: {} (existing) vs {} in scope {} ({}/{})",
                    existing.fqn,
                    a,
                    b,
                    scope.id,
                    scope.pkg().unwrap_or("?"),
                    scope.file().unwrap_or("?"),
                ),
                path: format!("topics.{topic_name}.rate_hz"),
                span: None,
            });
        }
    } else if existing.rate_hz.is_none() {
        existing.rate_hz = decl.rate_hz;
    }

    // Validate max_transport_ms agreement
    if let (Some(a), Some(b)) = (
        existing.max_transport_ms,
        decl.max_transport.map(|d| d.as_millis_f64()),
    ) {
        if (a - b).abs() > f64::EPSILON {
            diagnostics.push(Diagnostic {
                rule_id: "consistency".to_string(),
                severity: Severity::Error,
                message: format!(
                    "topic '{}' max_transport_ms mismatch: {} vs {} in scope {} ({}/{})",
                    existing.fqn,
                    a,
                    b,
                    scope.id,
                    scope.pkg().unwrap_or("?"),
                    scope.file().unwrap_or("?"),
                ),
                path: format!("topics.{topic_name}.max_transport_ms"),
                span: None,
            });
        }
    } else if existing.max_transport_ms.is_none() {
        existing.max_transport_ms = decl.max_transport.map(|d| d.as_millis_f64());
    }

    // Validate QoS agreement
    if let (Some(a), Some(b)) = (existing.qos.as_ref(), decl.qos.as_ref()) {
        if !qos_matches(a, b) {
            diagnostics.push(Diagnostic {
                rule_id: "consistency".to_string(),
                severity: Severity::Error,
                message: format!(
                    "topic '{}' qos mismatch in scope {} ({}/{})",
                    existing.fqn,
                    scope.id,
                    scope.pkg().unwrap_or("?"),
                    scope.file().unwrap_or("?"),
                ),
                path: format!("topics.{topic_name}.qos"),
                span: None,
            });
        }
    } else if existing.qos.is_none() {
        existing.qos = decl.qos.clone();
    }

    // Merge drop budget: fill when only one side declares; both declared
    // must agree (same N/W count + max_consecutive) — mismatched loss
    // budgets are a consistency error like rate_hz/qos.
    match (existing.drop.as_ref(), decl.drop.as_ref()) {
        (Some(a), Some(b)) => {
            let same = a.max_consecutive == b.max_consecutive
                && a.max_count.as_ref().map(|c| (c.n, c.w))
                    == b.max_count.as_ref().map(|c| (c.n, c.w));
            if !same {
                diagnostics.push(Diagnostic {
                    rule_id: "consistency".to_string(),
                    severity: Severity::Error,
                    message: format!(
                        "topic '{}' drop budget mismatch in scope {} ({}/{})",
                        existing.fqn,
                        scope.id,
                        scope.pkg().unwrap_or("?"),
                        scope.file().unwrap_or("?"),
                    ),
                    path: format!("topics.{topic_name}.drop"),
                    span: None,
                });
            }
        }
        (None, Some(_)) => existing.drop = decl.drop.clone(),
        _ => {}
    }

    // Merge endpoint lists (deduplicated)
    for p in publishers {
        if !existing.publishers.contains(&p) {
            existing.publishers.push(p);
        }
    }
    for s in subscribers {
        if !existing.subscribers.contains(&s) {
            existing.subscribers.push(s);
        }
    }

    // Track contributing scope
    if !existing.scope_ids.contains(&scope.id) {
        existing.scope_ids.push(scope.id);
    }
}

/// Compare two QoS declarations for equality.
fn qos_matches(
    a: &ros_launch_manifest_types::QosDecl,
    b: &ros_launch_manifest_types::QosDecl,
) -> bool {
    a.reliability == b.reliability
        && a.durability == b.durability
        && a.depth == b.depth
        && a.history == b.history
        && a.lifespan.map(|d| d.as_millis_f64()) == b.lifespan.map(|d| d.as_millis_f64())
        && a.liveliness == b.liveliness
}

/// Resolve and merge service declarations from this scope into the index.
fn resolve_services(manifest: &Manifest, scope: &ScopeEntry, index: &mut ManifestIndex) {
    let ns = scope.ns.clone();

    for (service_name, service_decl) in &manifest.services {
        let fqn = qualify_name(&ns, service_name);

        let mut servers: Vec<String> = Vec::new();
        for ep_ref in &service_decl.server {
            servers.push(resolve_endpoint_ref(index, scope.id, &ns, ep_ref));
        }
        let mut clients: Vec<String> = Vec::new();
        for ep_ref in &service_decl.client {
            clients.push(resolve_endpoint_ref(index, scope.id, &ns, ep_ref));
        }

        if let Some(existing) = index.services.get_mut(&fqn) {
            // Validate type agreement
            if existing.srv_type != service_decl.srv_type {
                index.merge_diagnostics.push(Diagnostic {
                    rule_id: "consistency".to_string(),
                    severity: Severity::Error,
                    message: format!(
                        "service '{}' type mismatch: '{}' vs '{}' in scope {} ({}/{})",
                        existing.fqn,
                        existing.srv_type,
                        service_decl.srv_type,
                        scope.id,
                        scope.pkg().unwrap_or("?"),
                        scope.file().unwrap_or("?"),
                    ),
                    path: format!("services.{service_name}.type"),
                    span: None,
                });
            }

            for s in servers {
                if !existing.servers.contains(&s) {
                    existing.servers.push(s);
                }
            }
            for c in clients {
                if !existing.clients.contains(&c) {
                    existing.clients.push(c);
                }
            }
            if !existing.scope_ids.contains(&scope.id) {
                existing.scope_ids.push(scope.id);
            }
        } else {
            index.services.insert(
                fqn.clone(),
                ResolvedService {
                    fqn,
                    srv_type: service_decl.srv_type.clone(),
                    servers,
                    clients,
                    scope_ids: vec![scope.id],
                },
            );
        }
    }
}

/// R1-P2 — resolve + merge `actions:` (mirror of `resolve_services`;
/// actions share the server/client wiring shape).
fn resolve_actions(manifest: &Manifest, scope: &ScopeEntry, index: &mut ManifestIndex) {
    let ns = &scope.ns;

    for (action_name, action_decl) in &manifest.actions {
        let fqn = qualify_name(ns, action_name);

        let servers: Vec<String> = action_decl
            .server
            .iter()
            .map(|ep_ref| qualify_endpoint_ref(ns, ep_ref))
            .collect();
        let clients: Vec<String> = action_decl
            .client
            .iter()
            .map(|ep_ref| qualify_endpoint_ref(ns, ep_ref))
            .collect();

        if let Some(existing) = index.actions.get_mut(&fqn) {
            if existing.srv_type != action_decl.action_type {
                index.merge_diagnostics.push(Diagnostic {
                    rule_id: "consistency".to_string(),
                    severity: Severity::Error,
                    message: format!(
                        "action '{}' type mismatch: '{}' vs '{}' in scope {} ({}/{})",
                        existing.fqn,
                        existing.srv_type,
                        action_decl.action_type,
                        scope.id,
                        scope.pkg().unwrap_or("?"),
                        scope.file().unwrap_or("?"),
                    ),
                    path: format!("actions.{action_name}.type"),
                    span: None,
                });
            }
            for s in servers {
                if !existing.servers.contains(&s) {
                    existing.servers.push(s);
                }
            }
            for c in clients {
                if !existing.clients.contains(&c) {
                    existing.clients.push(c);
                }
            }
            if !existing.scope_ids.contains(&scope.id) {
                existing.scope_ids.push(scope.id);
            }
        } else {
            index.actions.insert(
                fqn.clone(),
                ResolvedService {
                    fqn,
                    srv_type: action_decl.action_type.clone(),
                    servers,
                    clients,
                    scope_ids: vec![scope.id],
                },
            );
        }
    }
}

/// Resolve node path declarations.
fn resolve_node_paths(manifest: &Manifest, scope: &ScopeEntry, index: &mut ManifestIndex) {
    let ns = scope.ns.clone();

    for (node_name, node_decl) in &manifest.nodes {
        let node_fqn = resolve_node_fqn(index, scope.id, &ns, node_name);

        for (path_name, path_decl) in &node_decl.paths {
            index.node_paths.push(ResolvedNodePath {
                node_fqn: node_fqn.clone(),
                path_name: path_name.clone(),
                path: path_decl.clone(),
                scope_id: scope.id,
            });
        }
    }
}

/// Resolve scope-level path declarations.
///
/// Scope path `input:`/`output:` are topic names (relative or absolute).
/// Resolves each one using the scope's namespace via `qualify_name()`.
///
/// The input side reads [`PathDecl::effective_trigger`], not the legacy
/// `input:` list, because both spellings are live and the `explicit-trigger`
/// lint actively tells authors to migrate to the second one:
///
/// ```yaml
/// paths: { e2e: { input: /a,                     output: [/b] } }   # legacy
/// paths: { e2e: { trigger: { input: [/a] },      output: [/b] } }   # v2
/// ```
///
/// Reading `decl.input` alone made the v2 spelling resolve to ZERO input
/// topics, which does not fail — it produces a scope path with no route, so
/// the mapper falls back to ranking by budget and the derivation is silently
/// not used. Measured on `rt_workspace`: provenance went from
/// `points_to_cmd segment drain 2/2` to `non-chain budget_ms=5` and both the
/// `sched:override-inversion` and incomplete-evidence warnings disappeared,
/// while the two priorities stayed 39/38 by coincidence of the fixture. That
/// is the same false-pass shape phase 68 W4's gate was built to catch, one
/// layer down.
fn resolve_scope_paths(manifest: &Manifest, scope: &ScopeEntry, index: &mut ManifestIndex) {
    let ns = &scope.ns;

    for (path_name, decl) in &manifest.paths {
        // A scope path names two ENDS of a requirement, so only an
        // input-triggered one has a topic to start from; `timer`/`once`/
        // `spontaneous` carry no input endpoint and resolve to an empty list,
        // exactly as an absent `input:` always did.
        let declared_inputs = match decl.effective_trigger() {
            EffectiveTrigger::Input(endpoints) => endpoints,
            _ => Vec::new(),
        };
        let input_topics: Vec<String> = declared_inputs
            .iter()
            .map(|t| qualify_name(ns, t))
            .collect();
        let output_topics: Vec<String> = decl.output.iter().map(|t| qualify_name(ns, t)).collect();

        index.scope_paths.push(ResolvedScopePath {
            scope_id: scope.id,
            path_name: path_name.clone(),
            input_topics,
            output_topics,
            path: decl.clone(),
        });
    }
}

/// Prefix a relative name with the scope namespace.
///
/// If `ns` is empty or "/", returns `/<name>`.
/// If `ns` is "/perception", returns `/perception/<name>`.
/// If `name` already starts with "/", returns it as-is (absolute).
fn qualify_name(ns: &str, name: &str) -> String {
    if name.starts_with('/') {
        return name.to_string();
    }
    let ns = ns.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{name}")
    } else {
        format!("{ns}/{name}")
    }
}

/// Qualify an endpoint reference like "node/endpoint" with namespace.
///
/// "cropbox/output" in ns="/perception" → "/perception/cropbox/output"
fn qualify_endpoint_ref(ns: &str, ep_ref: &str) -> String {
    if ep_ref.starts_with('/') {
        return ep_ref.to_string();
    }
    let ns = ns.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{ep_ref}")
    } else {
        format!("{ns}/{ep_ref}")
    }
}

/// Resolve a bare node name declared in a scope's contract to its real,
/// launch-dump-verified FQN — the single reconciliation point described on
/// [`ManifestIndex::node_identity`]. Every consumer that needs "the FQN of
/// contract node X declared in scope S" must go through this function (or
/// [`resolve_endpoint_ref`]) rather than re-deriving `scope.ns + name`
/// locally, or it risks silently disagreeing with the rest of the pipeline
/// whenever a node's own `namespace=` attribute overrides its scope.
///
/// Absolute names (`name.starts_with('/')`) pass through unchanged, matching
/// [`qualify_name`]'s behavior.
pub(crate) fn resolve_node_fqn(
    index: &ManifestIndex,
    scope_id: usize,
    ns: &str,
    node_name: &str,
) -> String {
    if node_name.starts_with('/') {
        return node_name.to_string();
    }
    if let Some(fqn) = index.node_identity.get(&(scope_id, node_name.to_string())) {
        return fqn.clone();
    }
    qualify_name(ns, node_name)
}

/// Resolve an endpoint reference like `"node/endpoint"` the same way
/// [`resolve_node_fqn`] resolves a bare node name — splits off the trailing
/// endpoint segment, reconciles the node part against the launch dump, and
/// rejoins. Absolute refs pass through unchanged.
pub(crate) fn resolve_endpoint_ref(
    index: &ManifestIndex,
    scope_id: usize,
    ns: &str,
    ep_ref: &str,
) -> String {
    if ep_ref.starts_with('/') {
        return ep_ref.to_string();
    }
    match ep_ref.rfind('/') {
        Some(pos) => {
            let node_part = &ep_ref[..pos];
            let ep_part = &ep_ref[pos + 1..];
            format!(
                "{}/{ep_part}",
                resolve_node_fqn(index, scope_id, ns, node_part)
            )
        }
        // Malformed ref (no node/endpoint separator) — keep the old
        // best-effort behavior rather than panicking or dropping it.
        None => qualify_endpoint_ref(ns, ep_ref),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_qualify_name() {
        assert_eq!(qualify_name("", "chatter"), "/chatter");
        assert_eq!(qualify_name("/", "chatter"), "/chatter");
        assert_eq!(
            qualify_name("/perception", "cropped"),
            "/perception/cropped"
        );
        assert_eq!(qualify_name("/a/b", "topic"), "/a/b/topic");
        // Absolute names pass through
        assert_eq!(qualify_name("/perception", "/absolute"), "/absolute");
    }

    #[test]
    fn test_qualify_endpoint_ref() {
        assert_eq!(qualify_endpoint_ref("", "node/ep"), "/node/ep");
        assert_eq!(
            qualify_endpoint_ref("/perception", "cropbox/output"),
            "/perception/cropbox/output"
        );
        assert_eq!(
            qualify_endpoint_ref("/ns", "/absolute/ref"),
            "/absolute/ref"
        );
    }

    // ── Integration tests using fixture YAML files ──

    use super::super::launch_dump::{LaunchDump, ScopeOrigin};

    fn fixture_dir() -> PathBuf {
        // Owned by the manifest repository, which is a git dependency since
        // phase-55 W2 — its checkout path is unpredictable, so it hands the
        // path out itself rather than us guessing a relative one.
        ros_launch_manifest_check::fixture_dir()
    }

    /// Materialize an overlay tree in a tempdir from the checker fixture
    /// YAMLs (`<fixture_dir>/<pkg>/manifest.yaml`) and load manifests
    /// through it via `ContractSources { overlay: Some(_), provider: false }`.
    ///
    /// For every file scope in `dump`, copies the fixture's
    /// `<pkg>/manifest.yaml` (skipped when no such fixture exists, e.g.
    /// scopes deliberately pointing at nonexistent packages) to
    /// `<tmp>/<pkg>/launch/<stem>.contract.yaml`, matching the overlay
    /// channel's resolution layout. `pkg: None` uses the `_` convention
    /// shared with the provider/overlay channels. The tempdir is dropped
    /// before returning — safe, since `load_manifests` reads every
    /// resolved file synchronously and stores no open handles.
    fn overlay_index(dump: &LaunchDump) -> ManifestIndex {
        let tmp = tempfile::TempDir::new().unwrap();
        for scope in &dump.scopes {
            let Some(origin) = &scope.origin else {
                continue;
            };
            let pkg_dir = origin.pkg.as_deref().unwrap_or("_");
            let stem = contract_stem(&origin.file);
            let src = fixture_dir().join(pkg_dir).join("manifest.yaml");
            if !src.exists() {
                continue;
            }
            let dest_dir = tmp.path().join(pkg_dir).join("launch");
            std::fs::create_dir_all(&dest_dir).unwrap();
            std::fs::copy(&src, dest_dir.join(format!("{stem}.contract.yaml"))).unwrap();
        }
        let sources = ContractSources {
            overlay: Some(tmp.path().to_path_buf()),
            provider: false,
        };
        load_manifests(dump, &sources).unwrap()
    }

    fn make_dump(scopes: Vec<ScopeEntry>) -> LaunchDump {
        LaunchDump {
            node: vec![],
            load_node: vec![],
            container: vec![],
            lifecycle_node: vec![],
            file_data: HashMap::new(),
            variables: HashMap::new(),
            scopes,
        }
    }

    fn scope(id: usize, pkg: &str, file: &str, ns: &str, parent: Option<usize>) -> ScopeEntry {
        ScopeEntry {
            id,
            origin: Some(ScopeOrigin {
                pkg: Some(pkg.to_string()),
                file: file.to_string(),
                path: None,
            }),
            ns: ns.to_string(),
            args: HashMap::new(),
            parent,
        }
    }

    // ── General cases ──

    #[test]
    fn test_single_scope_single_manifest() {
        let dump = make_dump(vec![scope(
            0,
            "manifest_simple",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        assert_eq!(index.manifests.len(), 1);
        assert!(index.manifests.contains_key(&0));
        assert_eq!(index.manifests[&0].manifest.nodes.len(), 2);
        assert_eq!(index.total_errors, 0);

        // Topics resolved with "/" prefix
        assert!(
            index.topics.contains_key("/chatter"),
            "topics: {:?}",
            index.topics.keys().collect::<Vec<_>>()
        );
        let chatter = &index.topics["/chatter"];
        assert_eq!(chatter.msg_type, "std_msgs/msg/String");
        assert_eq!(chatter.publishers, vec!["/talker/chatter"]);
        assert_eq!(chatter.subscribers, vec!["/listener/chatter"]);
        assert_eq!(chatter.scope_ids, vec![0]);
    }

    #[test]
    fn test_multiple_scopes_no_collision() {
        let dump = make_dump(vec![
            scope(0, "manifest_simple", "manifest.launch.xml", "/a", None),
            scope(1, "manifest_simple", "manifest.launch.xml", "/b", Some(0)),
        ]);
        let index = overlay_index(&dump);

        assert_eq!(index.manifests.len(), 2);
        // Same manifest loaded under different namespaces — topics don't collide
        assert!(index.topics.contains_key("/a/chatter"));
        assert!(index.topics.contains_key("/b/chatter"));
        assert_ne!(
            index.topics["/a/chatter"].scope_ids,
            index.topics["/b/chatter"].scope_ids,
        );
    }

    #[test]
    fn test_deep_namespace_prefixing() {
        let dump = make_dump(vec![scope(
            0,
            "manifest_pipeline",
            "manifest.launch.xml",
            "/planning/scenario_planning/lane_driving/motion_planning",
            None,
        )]);
        let index = overlay_index(&dump);

        let deep_ns = "/planning/scenario_planning/lane_driving/motion_planning";
        let expected_topic = format!("{deep_ns}/cropped_points");
        assert!(
            index.topics.contains_key(&expected_topic),
            "expected {expected_topic}, got: {:?}",
            index.topics.keys().collect::<Vec<_>>()
        );
        let topic = &index.topics[&expected_topic];
        assert_eq!(
            topic.publishers,
            vec![format!("{deep_ns}/cropbox/cropped_points")]
        );

        // Node paths also qualified
        let fqns: Vec<&str> = index
            .node_paths
            .iter()
            .map(|p| p.node_fqn.as_str())
            .collect();
        assert!(
            fqns.contains(&format!("{deep_ns}/cropbox").as_str()),
            "fqns: {fqns:?}"
        );
    }

    #[test]
    fn test_violations_warn_not_fail() {
        // Manifest with errors should still be loaded — errors counted but not fatal
        let dump = make_dump(vec![scope(
            0,
            "manifest_violations",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        assert!(
            index.total_errors >= 5,
            "expected >=5 errors, got {}",
            index.total_errors
        );
        assert!(index.total_warnings > 0);
        // Manifest IS loaded despite errors
        assert_eq!(index.manifests.len(), 1);
        assert!(!index.topics.is_empty(), "topics should still be resolved");
    }

    #[test]
    fn test_missing_manifest_silently_skipped() {
        let dump = make_dump(vec![scope(
            0,
            "no_such_pkg",
            "no_such.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        assert!(index.manifests.is_empty());
        assert_eq!(index.total_errors, 0);
        assert!(index.topics.is_empty());
    }

    #[test]
    fn test_mixed_scopes_some_with_some_without() {
        // Mix of scopes: some with manifests, some without, some group scopes
        let dump = make_dump(vec![
            scope(0, "manifest_simple", "manifest.launch.xml", "", None),
            // Group scope (no origin)
            ScopeEntry {
                id: 1,
                origin: None,
                ns: "/system".to_string(),
                args: HashMap::new(),
                parent: Some(0),
            },
            // File scope with no matching manifest
            scope(
                2,
                "nonexistent_pkg",
                "missing.launch.xml",
                "/system",
                Some(1),
            ),
            // Another valid manifest
            scope(
                3,
                "manifest_ndt",
                "manifest.launch.xml",
                "/localization",
                Some(0),
            ),
        ]);
        let index = overlay_index(&dump);

        // Only 2 manifests loaded (simple + ndt), group skipped, missing skipped
        assert_eq!(index.manifests.len(), 2);
        assert!(index.manifests.contains_key(&0));
        assert!(index.manifests.contains_key(&3));
        assert!(index.topics.contains_key("/chatter"));
        assert!(index.topics.contains_key("/localization/ndt_pose"));
    }

    // ── Autoware edge cases ──

    #[test]
    fn test_same_file_multiple_invocations_different_namespaces() {
        // Mirrors Autoware: load_topic_state_monitor.launch.xml included 10 times
        // under /system with different topic args. Each gets its own scope ID.
        let dump = make_dump(vec![
            scope(
                0,
                "manifest_simple",
                "manifest.launch.xml",
                "/system/topic_monitor_1",
                None,
            ),
            scope(
                1,
                "manifest_simple",
                "manifest.launch.xml",
                "/system/topic_monitor_2",
                None,
            ),
            scope(
                2,
                "manifest_simple",
                "manifest.launch.xml",
                "/system/topic_monitor_3",
                None,
            ),
            scope(
                3,
                "manifest_simple",
                "manifest.launch.xml",
                "/system/topic_monitor_4",
                None,
            ),
            scope(
                4,
                "manifest_simple",
                "manifest.launch.xml",
                "/system/topic_monitor_5",
                None,
            ),
        ]);
        let index = overlay_index(&dump);

        // All 5 invocations load successfully
        assert_eq!(index.manifests.len(), 5);
        // Each has its own topic with distinct namespace
        for i in 1..=5 {
            let key = format!("/system/topic_monitor_{i}/chatter");
            assert!(index.topics.contains_key(&key), "missing {key}");
            assert_eq!(index.topics[&key].scope_ids, vec![i - 1]);
        }
        // 5 scopes × 1 topic = 5 distinct topic entries
        let chatter_topics: Vec<_> = index
            .topics
            .keys()
            .filter(|k| k.ends_with("/chatter"))
            .collect();
        assert_eq!(chatter_topics.len(), 5);
    }

    #[test]
    fn test_group_scopes_interleaved_with_file_scopes() {
        // Mirrors Autoware: group scopes (origin: None) wrap includes for namespace context
        // Pattern: root → group(/system) → file(/system) → group(/system) → file(/system)
        let dump = make_dump(vec![
            scope(0, "manifest_simple", "manifest.launch.xml", "/", None),
            // Group scope wrapping the system namespace
            ScopeEntry {
                id: 1,
                origin: None,
                ns: "/system".to_string(),
                args: HashMap::new(),
                parent: Some(0),
            },
            scope(
                2,
                "manifest_ndt",
                "manifest.launch.xml",
                "/system/localization",
                Some(1),
            ),
            // Another group scope
            ScopeEntry {
                id: 3,
                origin: None,
                ns: "/perception".to_string(),
                args: HashMap::new(),
                parent: Some(0),
            },
            scope(
                4,
                "manifest_pipeline",
                "manifest.launch.xml",
                "/perception",
                Some(3),
            ),
        ]);
        let index = overlay_index(&dump);

        // 3 file scopes loaded, 2 group scopes skipped
        assert_eq!(index.manifests.len(), 3);
        assert!(index.manifests.contains_key(&0)); // root
        assert!(index.manifests.contains_key(&2)); // NDT under /system/localization
        assert!(index.manifests.contains_key(&4)); // pipeline under /perception
        assert!(!index.manifests.contains_key(&1)); // group skipped
        assert!(!index.manifests.contains_key(&3)); // group skipped

        assert!(index.topics.contains_key("/chatter")); // root
        assert!(index.topics.contains_key("/system/localization/ndt_pose")); // NDT
        assert!(index.topics.contains_key("/perception/cropped_points")); // pipeline
    }

    #[test]
    fn test_deep_nesting_resolves_from_own_scope() {
        // Mirrors Autoware: depth-16 nesting. Each scope resolves manifest from its own
        // (pkg, file), not from ancestors. Namespace is the scope's ns field.
        let dump = make_dump(vec![
            scope(0, "manifest_simple", "manifest.launch.xml", "/", None),
            ScopeEntry {
                id: 1,
                origin: None,
                ns: "/".to_string(),
                args: HashMap::new(),
                parent: Some(0),
            },
            scope(2, "nonexistent", "tier1.launch.xml", "/", Some(1)),
            ScopeEntry {
                id: 3,
                origin: None,
                ns: "/planning".to_string(),
                args: HashMap::new(),
                parent: Some(2),
            },
            scope(4, "nonexistent", "tier2.launch.xml", "/planning", Some(3)),
            ScopeEntry {
                id: 5,
                origin: None,
                ns: "/planning/scenario".to_string(),
                args: HashMap::new(),
                parent: Some(4),
            },
            scope(
                6,
                "nonexistent",
                "tier3.launch.xml",
                "/planning/scenario",
                Some(5),
            ),
            ScopeEntry {
                id: 7,
                origin: None,
                ns: "/planning/scenario/lane".to_string(),
                args: HashMap::new(),
                parent: Some(6),
            },
            scope(
                8,
                "nonexistent",
                "tier4.launch.xml",
                "/planning/scenario/lane",
                Some(7),
            ),
            ScopeEntry {
                id: 9,
                origin: None,
                ns: "/planning/scenario/lane/motion".to_string(),
                args: HashMap::new(),
                parent: Some(8),
            },
            // Deep leaf scope — this is the one that has a manifest
            scope(
                10,
                "manifest_periodic",
                "manifest.launch.xml",
                "/planning/scenario/lane/motion",
                Some(9),
            ),
        ]);
        let index = overlay_index(&dump);

        // Only scope 0 (simple) and scope 10 (periodic) have manifests
        assert_eq!(index.manifests.len(), 2);
        assert!(index.manifests.contains_key(&0));
        assert!(index.manifests.contains_key(&10));

        // Deep scope resolves with its own namespace, not ancestors'
        let deep_topic = "/planning/scenario/lane/motion/control_cmd";
        assert!(
            index.topics.contains_key(deep_topic),
            "expected {deep_topic}, got: {:?}",
            index.topics.keys().collect::<Vec<_>>()
        );
        assert_eq!(index.topics[deep_topic].scope_ids, vec![10]);
    }

    #[test]
    fn test_scopes_without_entities_still_load_manifests() {
        // Mirrors Autoware: 108/169 scopes have zero entities. Manifests are loaded
        // and checked for pre-validation even if no nodes reference this scope.
        use super::super::launch_dump::NodeRecord;

        let mut dump = make_dump(vec![
            scope(0, "manifest_simple", "manifest.launch.xml", "", None),
            scope(
                1,
                "manifest_pipeline",
                "manifest.launch.xml",
                "/perception",
                Some(0),
            ),
        ]);
        // Only add a node to scope 0, scope 1 has no entities
        dump.node.push(NodeRecord {
            executable: "talker".to_string(),
            package: Some("demo_nodes_cpp".to_string()),
            name: Some("talker".to_string()),
            namespace: None,
            exec_name: Some("talker".to_string()),
            params: vec![],
            params_files: vec![],
            param_sources: Vec::new(),
            remaps: vec![],
            ros_args: None,
            args: None,
            cmd: vec![],
            env: None,
            respawn: None,
            respawn_delay: None,
            global_params: None,
            scope: Some(0),
        });

        let index = overlay_index(&dump);

        // Both manifests loaded, even though scope 1 has no entities
        assert_eq!(index.manifests.len(), 2);
        assert!(index.manifests.contains_key(&1));
        assert!(!index.topics.is_empty());
    }

    #[test]
    fn test_node_own_namespace_override_reconciles_to_launch_fqn() {
        // Phase 44.6 fix pin: a node's own `namespace=` launch-XML attribute
        // (here "/sensors/cropbox_group") overrides its declaring scope's
        // namespace ("/perception") — the contract (which only ever names
        // the node bare, "cropbox") must not silently attribute its facts
        // to the naive scope-qualified "/perception/cropbox"; it must
        // reconcile against the real launch-dump FQN.
        use super::super::launch_dump::NodeRecord;

        let mut dump = make_dump(vec![scope(
            0,
            "manifest_pipeline",
            "manifest.launch.xml",
            "/perception",
            None,
        )]);
        dump.node.push(NodeRecord {
            executable: "cropbox".to_string(),
            package: Some("manifest_pipeline".to_string()),
            name: Some("cropbox".to_string()),
            namespace: Some("/sensors/cropbox_group".to_string()),
            exec_name: Some("cropbox".to_string()),
            params: vec![],
            params_files: vec![],
            param_sources: Vec::new(),
            remaps: vec![],
            ros_args: None,
            args: None,
            cmd: vec![],
            env: None,
            respawn: None,
            respawn_delay: None,
            global_params: None,
            scope: Some(0),
        });

        let index = overlay_index(&dump);

        let real_fqn = "/sensors/cropbox_group/cropbox";
        let naive_fqn = "/perception/cropbox";

        // The identity map resolves the contract's bare "cropbox" (declared
        // in scope 0) to the real launch-dump FQN.
        assert_eq!(
            index.node_identity.get(&(0, "cropbox".to_string())),
            Some(&real_fqn.to_string())
        );

        // `nodes.cropbox.paths.main`'s facts attach to the real FQN, not the
        // naive scope-qualified one.
        let main_path = index
            .node_paths
            .iter()
            .find(|p| p.path_name == "main")
            .expect("cropbox.main path resolved");
        assert_eq!(main_path.node_fqn, real_fqn);
        assert_eq!(
            main_path.path.max_latency.map(|d| d.as_millis_f64()),
            Some(5.0)
        );

        // The pub/sub endpoint refs on the merged topics reconcile the same
        // way — `cropbox/raw_points`/`cropbox/cropped_points` must key on
        // the real FQN so rate/QoS/graph checks agree with the scheduling
        // plan's node set.
        assert!(
            index.topics["/sensing/lidar/pointcloud"]
                .subscribers
                .contains(&format!("{real_fqn}/raw_points")),
            "subscribers: {:?}",
            index.topics["/sensing/lidar/pointcloud"].subscribers
        );
        assert!(
            index.topics["/perception/cropped_points"]
                .publishers
                .contains(&format!("{real_fqn}/cropped_points"))
        );

        // Never the naive scope-qualified identity.
        assert_ne!(main_path.node_fqn, naive_fqn);

        // `sched_derive`'s extraction must key the fact on the same real
        // FQN a `ScheduledRecord` (built from the launch dump) carries —
        // this is what actually routes a derived/chain priority to the
        // real process.
        let input = crate::ros::sched_derive::mapper_input_from_dump(
            &dump,
            Some(&index),
            None,
            &std::collections::BTreeMap::new(),
        );
        let node = input
            .nodes
            .iter()
            .find(|n| n.name == real_fqn)
            .unwrap_or_else(|| panic!("no scheduled node named {real_fqn}: {:?}", input.nodes));
        assert_eq!(node.path_budget_ms, Some(5.0));
        assert_eq!(node.deadline_us, Some(5_000));
    }

    #[test]
    fn test_node_behind_bare_group_reconciles_across_file_scope() {
        // Phase 44 W6 Fix B (group-nesting FQN gap, §4c): reproduces the
        // `vehicle_cmd_gate` shape — a node sits behind a bare (non-
        // `<include>`) `<group>` *within its own file's scope subtree*.
        // The parser stamps that group with its own distinct scope id (1),
        // one level deeper than the file scope (0) that
        // `resolve_node_paths`/`resolve_node_fqn` actually query
        // `node_identity` under. Before the fix, `node_identity` was keyed
        // only by the record's immediate scope id (1) — a guaranteed miss
        // at scope 0, silently falling back to naive `scope.ns + bare_name`
        // qualification instead of the real launch FQN.
        use super::super::launch_dump::NodeRecord;

        let mut dump = make_dump(vec![
            scope(
                0,
                "manifest_pipeline",
                "manifest.launch.xml",
                "/perception",
                None,
            ),
            // Bare group: no origin, own scope id, nested namespace — same
            // shape as autoware's `<group ns="/control">` wrapping
            // `vehicle_cmd_gate` inside `control.launch.xml`'s own file scope.
            ScopeEntry {
                id: 1,
                origin: None,
                ns: "/perception/inner".to_string(),
                args: HashMap::new(),
                parent: Some(0),
            },
        ]);
        dump.node.push(NodeRecord {
            executable: "cropbox".to_string(),
            package: Some("manifest_pipeline".to_string()),
            name: Some("cropbox".to_string()),
            namespace: None,
            exec_name: Some("cropbox".to_string()),
            params: vec![],
            params_files: vec![],
            param_sources: Vec::new(),
            remaps: vec![],
            ros_args: None,
            args: None,
            cmd: vec![],
            env: None,
            respawn: None,
            respawn_delay: None,
            global_params: None,
            scope: Some(1),
        });

        let index = overlay_index(&dump);

        let real_fqn = "/perception/inner/cropbox";
        let naive_fqn = "/perception/cropbox";

        // node_identity reconciles under BOTH the node's immediate (group)
        // scope id and the manifest's own file scope id (0) — the latter
        // is what `resolve_node_paths` actually queries.
        assert_eq!(
            index.node_identity.get(&(1, "cropbox".to_string())),
            Some(&real_fqn.to_string())
        );
        assert_eq!(
            index.node_identity.get(&(0, "cropbox".to_string())),
            Some(&real_fqn.to_string()),
            "node_identity must also reconcile under the file scope id (0), \
             not just the node's immediate bare-group scope id (1)"
        );

        // `nodes.cropbox.paths.main`'s facts attach to the real FQN, never
        // the naive scope-qualified phantom.
        let main_path = index
            .node_paths
            .iter()
            .find(|p| p.path_name == "main")
            .expect("cropbox.main path resolved");
        assert_eq!(main_path.node_fqn, real_fqn);
        assert_ne!(main_path.node_fqn, naive_fqn);

        // The scheduling extraction pipeline agrees: chain/derived facts
        // route to the real process, not a phantom no process answers to.
        let input = crate::ros::sched_derive::mapper_input_from_dump(
            &dump,
            Some(&index),
            None,
            &std::collections::BTreeMap::new(),
        );
        let node = input
            .nodes
            .iter()
            .find(|n| n.name == real_fqn)
            .unwrap_or_else(|| panic!("no scheduled node named {real_fqn}: {:?}", input.nodes));
        assert_eq!(node.path_budget_ms, Some(5.0));
    }

    #[test]
    fn test_large_argument_sets_dont_affect_resolution() {
        // Mirrors Autoware: root scope has 174 args. Args are scope metadata,
        // not used by manifest loader.
        let mut s = scope(0, "manifest_simple", "manifest.launch.xml", "", None);
        for i in 0..200 {
            s.args.insert(format!("arg_{i}"), format!("value_{i}"));
        }
        let dump = make_dump(vec![s]);
        let index = overlay_index(&dump);

        assert_eq!(index.manifests.len(), 1);
        assert!(index.topics.contains_key("/chatter"));
    }

    #[test]
    fn test_root_scope_slash_namespace() {
        // topic "chatter" at ns="/" should become "/chatter", not "//chatter"
        let dump = make_dump(vec![scope(
            0,
            "manifest_simple",
            "manifest.launch.xml",
            "/",
            None,
        )]);
        let index = overlay_index(&dump);

        assert!(
            index.topics.contains_key("/chatter"),
            "topics: {:?}",
            index.topics.keys().collect::<Vec<_>>()
        );
        // Ensure no double-slash
        for key in index.topics.keys() {
            assert!(!key.contains("//"), "double slash in topic FQN: {key}");
        }
        for path in &index.node_paths {
            assert!(
                !path.node_fqn.contains("//"),
                "double slash in node FQN: {}",
                path.node_fqn
            );
        }
    }

    #[test]
    fn test_all_fixtures_load_without_panic() {
        // Smoke test: every fixture loads and resolves under various namespaces
        let fixtures = [
            "manifest_simple",
            "manifest_pipeline",
            "manifest_ndt",
            "manifest_periodic",
            "manifest_violations",
            "manifest_multi_scope",
        ];
        let namespaces = ["", "/", "/ns", "/a/b/c/d/e"];
        for (i, fixture) in fixtures.iter().enumerate() {
            for (j, ns) in namespaces.iter().enumerate() {
                let id = i * namespaces.len() + j;
                let dump = make_dump(vec![scope(id, fixture, "manifest.launch.xml", ns, None)]);
                let index = overlay_index(&dump);
                assert!(
                    index.manifests.len() <= 1,
                    "fixture {fixture} at ns {ns}: unexpected manifest count"
                );
                // No double-slashes in any resolved name
                for key in index.topics.keys() {
                    assert!(!key.contains("//"), "{fixture}@{ns}: double slash in {key}");
                }
            }
        }
    }

    #[test]
    fn test_pipeline_full_resolution() {
        // Comprehensive check of pipeline manifest: nodes, topics, paths, imports, exports
        let dump = make_dump(vec![scope(
            0,
            "manifest_pipeline",
            "manifest.launch.xml",
            "/perception",
            None,
        )]);
        let index = overlay_index(&dump);

        assert_eq!(index.total_errors, 0, "pipeline should be clean");

        // 4 nodes, each with 1 path
        let fqns: Vec<&str> = index
            .node_paths
            .iter()
            .map(|p| p.node_fqn.as_str())
            .collect();
        assert!(fqns.contains(&"/perception/cropbox"));
        assert!(fqns.contains(&"/perception/ground_filter"));
        assert!(fqns.contains(&"/perception/fusion"));
        assert!(fqns.contains(&"/perception/tracker"));

        // 6 topics: 5 relative + 1 absolute (/sensing/lidar/pointcloud)
        assert_eq!(index.topics.len(), 6);
        assert!(index.topics.contains_key("/perception/cropped_points"));
        assert!(index.topics.contains_key("/perception/no_ground_points"));
        assert!(index.topics.contains_key("/perception/camera_detections"));
        assert!(index.topics.contains_key("/perception/fused_objects"));
        assert!(index.topics.contains_key("/perception/tracked_objects"));
        // Absolute key passes through unchanged regardless of scope ns
        assert!(index.topics.contains_key("/sensing/lidar/pointcloud"));

        // Scope paths
        let scope0_paths: Vec<&ResolvedScopePath> = index
            .scope_paths
            .iter()
            .filter(|p| p.scope_id == 0)
            .collect();
        assert_eq!(scope0_paths.len(), 1);
        assert_eq!(scope0_paths[0].path_name, "perception");
    }

    #[test]
    fn test_ndt_state_and_required_resolution() {
        let dump = make_dump(vec![scope(
            0,
            "manifest_ndt",
            "manifest.launch.xml",
            "/loc",
            None,
        )]);
        let index = overlay_index(&dump);

        assert_eq!(index.total_errors, 0);

        // Topics with full namespace
        assert!(index.topics.contains_key("/loc/downsampled_points"));
        assert!(index.topics.contains_key("/loc/ndt_pose"));
        assert!(index.topics.contains_key("/loc/ekf_initial_pose"));

        // Node paths
        let ndt_paths: Vec<_> = index
            .node_paths
            .iter()
            .filter(|p| p.node_fqn == "/loc/ndt_scan_matcher")
            .collect();
        assert_eq!(ndt_paths.len(), 1);
        assert_eq!(ndt_paths[0].path_name, "main");
        assert_eq!(
            ndt_paths[0].path.max_latency.map(|d| d.as_millis_f64()),
            Some(30.0)
        );

        // EKF has 2 paths
        let ekf_paths: Vec<_> = index
            .node_paths
            .iter()
            .filter(|p| p.node_fqn == "/loc/ekf_localizer")
            .collect();
        assert_eq!(ekf_paths.len(), 2);
    }

    #[test]
    fn test_periodic_empty_input_paths() {
        let dump = make_dump(vec![scope(
            0,
            "manifest_periodic",
            "manifest.launch.xml",
            "/ctrl",
            None,
        )]);
        let index = overlay_index(&dump);

        assert_eq!(index.total_errors, 0);

        // All 3 nodes have periodic paths with empty input
        for np in &index.node_paths {
            assert!(
                np.path.input.is_empty(),
                "node {} path {} should have empty input (periodic)",
                np.node_fqn,
                np.path_name
            );
        }
    }

    // ── Args and substitution ──

    #[test]
    fn test_args_resolved_from_scope() {
        // All args are mandatory — must be provided by scope args
        let mut s = scope(0, "manifest_args", "manifest.launch.xml", "", None);
        s.args
            .insert("input_topic".into(), "/resolved/input".into());
        s.args
            .insert("output_topic".into(), "/resolved/output".into());
        s.args.insert("node_enabled".into(), "true".into());
        let dump = make_dump(vec![s]);
        let index = overlay_index(&dump);

        assert_eq!(index.manifests.len(), 1);
        let input_topic = &index.topics["/input_data"];
        assert_eq!(input_topic.msg_type, "/resolved/input");
        let output_topic = &index.topics["/output_data"];
        assert_eq!(output_topic.msg_type, "/resolved/output");
    }

    #[test]
    fn test_args_missing_from_scope_skips_manifest() {
        // Scope args don't include required args — manifest should be skipped (warned)
        let dump = make_dump(vec![scope(
            0,
            "manifest_args",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        // Manifest skipped because required args are missing
        assert!(index.manifests.is_empty());
    }

    #[test]
    fn test_args_no_args_section_still_works() {
        // manifest_simple has no args: section — should work as before
        let dump = make_dump(vec![scope(
            0,
            "manifest_simple",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);
        assert_eq!(index.manifests.len(), 1);
        assert!(index.topics.contains_key("/chatter"));
    }

    // ── Conditions ──

    #[test]
    fn test_conditions_default_args() {
        // Provide scope args matching the "default" Autoware configuration
        let mut s = scope(0, "manifest_conditions", "manifest.launch.xml", "", None);
        s.args.insert("use_feature_a".into(), "true".into());
        s.args.insert("use_feature_b".into(), "false".into());
        s.args.insert("sensor_model".into(), "velodyne".into());
        let dump = make_dump(vec![s]);
        let index = overlay_index(&dump);

        assert_eq!(index.manifests.len(), 1);
        let m = &index.manifests[&0].manifest;

        // always_present, feature_a_node, sensor_specific should be present
        assert!(m.nodes.contains_key("always_present"));
        assert!(m.nodes.contains_key("feature_a_node"));
        assert!(m.nodes.contains_key("sensor_specific"));
        // feature_b_node excluded (use_feature_b=false)
        assert!(!m.nodes.contains_key("feature_b_node"));
        // legacy_node excluded (unless: use_feature_a which is true)
        assert!(!m.nodes.contains_key("legacy_node"));

        // Topics: always_topic + feature_a_topic present, feature_b_topic excluded
        assert!(index.topics.contains_key("/always_topic"));
        assert!(index.topics.contains_key("/feature_a_topic"));
        assert!(!index.topics.contains_key("/feature_b_topic"));
    }

    #[test]
    fn test_conditions_overridden_args() {
        // Override: use_feature_a=false, use_feature_b=true
        let mut s = scope(0, "manifest_conditions", "manifest.launch.xml", "", None);
        s.args.insert("use_feature_a".into(), "false".into());
        s.args.insert("use_feature_b".into(), "true".into());
        s.args.insert("sensor_model".into(), "velodyne".into());
        let dump = make_dump(vec![s]);
        let index = overlay_index(&dump);

        let m = &index.manifests[&0].manifest;

        // feature_a_node excluded, feature_b_node included, legacy_node included (unless false)
        assert!(m.nodes.contains_key("always_present"));
        assert!(!m.nodes.contains_key("feature_a_node"));
        assert!(m.nodes.contains_key("feature_b_node"));
        assert!(m.nodes.contains_key("legacy_node"));
        // sensor_specific still present (sensor_model=velodyne)
        assert!(m.nodes.contains_key("sensor_specific"));
    }

    #[test]
    fn test_conditions_sensor_model_mismatch() {
        // Override sensor_model to something other than velodyne — all 3 args required
        let mut s = scope(0, "manifest_conditions", "manifest.launch.xml", "", None);
        s.args.insert("use_feature_a".into(), "true".into());
        s.args.insert("use_feature_b".into(), "false".into());
        s.args.insert("sensor_model".into(), "hesai".into());
        let dump = make_dump(vec![s]);
        let index = overlay_index(&dump);

        let m = &index.manifests[&0].manifest;

        // sensor_specific excluded (hesai != velodyne)
        assert!(!m.nodes.contains_key("sensor_specific"));
    }

    // ── Cross-scope merge and consistency (Phase 34.5) ──

    #[test]
    fn test_cross_scope_merge_compatible_types() {
        // Two scopes declare the same topic with matching types — should merge cleanly.
        let dump = make_dump(vec![
            scope(
                0,
                "manifest_consistency_pub",
                "manifest.launch.xml",
                "",
                None,
            ),
            scope(
                1,
                "manifest_consistency_sub",
                "manifest.launch.xml",
                "",
                Some(0),
            ),
        ]);
        let index = overlay_index(&dump);

        // Both manifests loaded
        assert_eq!(index.manifests.len(), 2);

        // Topic merged into a single entry
        assert!(index.topics.contains_key("/shared_data"));
        let topic = &index.topics["/shared_data"];

        // Both scopes contributed
        assert_eq!(topic.scope_ids.len(), 2);
        assert!(topic.scope_ids.contains(&0));
        assert!(topic.scope_ids.contains(&1));

        // Endpoints merged from both sides
        assert_eq!(topic.publishers, vec!["/source/out"]);
        assert_eq!(topic.subscribers, vec!["/consumer/in"]);

        // Type matches both sides
        assert_eq!(topic.msg_type, "std_msgs/msg/String");

        // No consistency errors
        let consistency_errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "consistency")
            .collect();
        assert!(
            consistency_errors.is_empty(),
            "expected no consistency errors, got: {consistency_errors:?}"
        );
    }

    #[test]
    fn test_cross_scope_merge_type_mismatch() {
        // Two scopes declare the same topic with DIFFERENT types — consistency error.
        let dump = make_dump(vec![
            scope(
                0,
                "manifest_consistency_pub",
                "manifest.launch.xml",
                "",
                None,
            ),
            scope(
                1,
                "manifest_consistency_bad",
                "manifest.launch.xml",
                "",
                Some(0),
            ),
        ]);
        let index = overlay_index(&dump);

        // Both manifests still loaded (errors are not fatal)
        assert_eq!(index.manifests.len(), 2);

        // Topic exists with the first-seen type
        assert!(index.topics.contains_key("/shared_data"));

        // Consistency error emitted
        let consistency_errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "consistency")
            .collect();
        assert_eq!(
            consistency_errors.len(),
            1,
            "expected 1 consistency error, got: {consistency_errors:?}"
        );
        let err = consistency_errors[0];
        assert_eq!(err.severity, Severity::Error);
        assert!(err.message.contains("type mismatch"));
        assert!(err.message.contains("/shared_data"));

        // Error counted in totals
        assert!(index.total_errors >= 1);
    }

    #[test]
    fn test_cross_scope_dangling_sub_no_publisher() {
        // Subscriber-only manifest with no publisher anywhere — dangling-entity warning.
        let dump = make_dump(vec![scope(
            0,
            "manifest_consistency_sub",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        let topic = &index.topics["/shared_data"];
        assert!(topic.publishers.is_empty());
        assert!(!topic.subscribers.is_empty());

        // dangling-entity warning emitted from cross-scope check
        let dangling_warnings: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "dangling-entity" && d.message.contains("/shared_data"))
            .collect();
        assert_eq!(
            dangling_warnings.len(),
            1,
            "expected 1 dangling-entity warning for /shared_data, got: {dangling_warnings:?}"
        );
        assert_eq!(dangling_warnings[0].severity, Severity::Warning);
    }

    #[test]
    fn test_cross_scope_dangling_resolved_when_publisher_present() {
        // Same as above, but with the publisher manifest also loaded — no warning.
        let dump = make_dump(vec![
            scope(
                0,
                "manifest_consistency_pub",
                "manifest.launch.xml",
                "",
                None,
            ),
            scope(
                1,
                "manifest_consistency_sub",
                "manifest.launch.xml",
                "",
                Some(0),
            ),
        ]);
        let index = overlay_index(&dump);

        let topic = &index.topics["/shared_data"];
        assert!(!topic.publishers.is_empty());
        assert!(!topic.subscribers.is_empty());

        let dangling_warnings: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "dangling-entity" && d.message.contains("/shared_data"))
            .collect();
        assert!(
            dangling_warnings.is_empty(),
            "expected no dangling-entity warnings, got: {dangling_warnings:?}"
        );
    }

    #[test]
    fn test_path_budget_overflow_child_exceeds_parent() {
        // Parent scope path: 100ms. Child scope path with same input/output: 200ms → error.
        let dump = make_dump(vec![
            scope(0, "manifest_path_parent", "manifest.launch.xml", "", None),
            scope(
                1,
                "manifest_path_child_overflow",
                "manifest.launch.xml",
                "",
                Some(0),
            ),
        ]);
        let index = overlay_index(&dump);

        let overflow_errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "budget-overflow")
            .collect();
        assert_eq!(
            overflow_errors.len(),
            1,
            "expected 1 budget-overflow error, got: {overflow_errors:?}"
        );
        let err = overflow_errors[0];
        assert_eq!(err.severity, Severity::Error);
        assert!(err.message.contains("max_latency_ms"));
        assert!(err.message.contains("200"));
        assert!(err.message.contains("100"));
    }

    #[test]
    fn test_path_budget_child_within_parent_ok() {
        // Parent path: 100ms. Child path with same input/output: 50ms → OK.
        let dump = make_dump(vec![
            scope(0, "manifest_path_parent", "manifest.launch.xml", "", None),
            scope(
                1,
                "manifest_path_child_ok",
                "manifest.launch.xml",
                "",
                Some(0),
            ),
        ]);
        let index = overlay_index(&dump);

        let overflow_errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "budget-overflow")
            .collect();
        assert!(
            overflow_errors.is_empty(),
            "expected no budget-overflow errors, got: {overflow_errors:?}"
        );
    }

    #[test]
    fn test_path_budget_unrelated_scopes_no_overflow() {
        // Two unrelated scopes (no parent-child relationship) declaring paths
        // with the same boundaries should NOT trigger budget-overflow,
        // because the rule applies only along the scope tree.
        let dump = make_dump(vec![
            scope(0, "manifest_path_parent", "manifest.launch.xml", "", None),
            // Sibling scope (parent = None, not descendant of scope 0)
            scope(
                1,
                "manifest_path_child_overflow",
                "manifest.launch.xml",
                "",
                None,
            ),
        ]);
        let index = overlay_index(&dump);

        let overflow_errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "budget-overflow")
            .collect();
        assert!(
            overflow_errors.is_empty(),
            "expected no budget-overflow errors for unrelated scopes, got: {overflow_errors:?}"
        );
    }

    #[test]
    fn test_path_resolved_topic_names() {
        // Verify scope path input/output topics are resolved to FQNs.
        let dump = make_dump(vec![scope(
            0,
            "manifest_path_parent",
            "manifest.launch.xml",
            "/perception",
            None,
        )]);
        let index = overlay_index(&dump);

        let scope_path = index
            .scope_paths
            .iter()
            .find(|p| p.scope_id == 0 && p.path_name == "pipeline")
            .expect("pipeline path should be present");

        // Absolute names pass through unchanged
        assert_eq!(scope_path.input_topics, vec!["/sensor/raw"]);
        assert_eq!(scope_path.output_topics, vec!["/perception/objects"]);
    }

    #[test]
    fn test_cross_scope_publisher_only_loads_standalone() {
        // Publisher-only manifest validates standalone (no errors). The
        // cross-scope `dangling-entity` rule emits a "no subscribers"
        // warning for the unconsumed topic — this is expected and the
        // user is meant to suppress it via `external_topics: { ...:
        // external: sub }` when the consumer is intentionally outside
        // the tree. Test asserts: no errors, exactly one warning, and
        // the topic is recorded with no subscribers.
        let dump = make_dump(vec![scope(
            0,
            "manifest_consistency_pub",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        assert_eq!(index.manifests.len(), 1);
        assert!(index.topics.contains_key("/shared_data"));

        let topic = &index.topics["/shared_data"];
        assert_eq!(topic.publishers, vec!["/source/out"]);
        assert!(topic.subscribers.is_empty());

        let errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.severity == Severity::Error)
            .collect();
        assert!(
            errors.is_empty(),
            "expected no errors on publisher-only standalone manifest, got: {errors:?}"
        );
        let no_sub_warnings: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| {
                d.rule_id == "dangling-entity"
                    && d.message.contains("/shared_data")
                    && d.message.contains("0 subscribers")
            })
            .collect();
        assert_eq!(
            no_sub_warnings.len(),
            1,
            "expected exactly one 'no subscribers' warning on /shared_data, got: {no_sub_warnings:?}"
        );
    }

    #[test]
    fn test_external_topics_suppresses_dangling_pub() {
        use ros_launch_manifest_types::ExternalSide;
        // A manifest declaring an external producer should not emit
        // a dangling-entity warning for that topic's missing pub side.
        let dump = make_dump(vec![scope(
            0,
            "manifest_external_pub",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        // The fixture must declare /external/in as an external pub
        // topic AND a subscriber that wires it locally; this confirms
        // the external mark suppresses what would otherwise be a
        // "no publishers" warning.
        assert_eq!(
            index.externals.get("/external/in"),
            Some(&ExternalSide::Pub),
            "externals map should record /external/in as external pub"
        );
        let dangling: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "dangling-entity" && d.message.contains("/external/in"))
            .collect();
        assert!(
            dangling.is_empty(),
            "external pub mark should suppress dangling-entity, got: {dangling:?}"
        );
    }

    // ── Phase 35: critical-path / graph-aware budget checks ──

    #[test]
    fn test_global_graph_builds_for_pipeline() {
        use super::super::manifest_graph::build_global_graph;

        let dump = make_dump(vec![scope(
            0,
            "manifest_pipeline",
            "manifest.launch.xml",
            "/perception",
            None,
        )]);
        let index = overlay_index(&dump);
        let graph = build_global_graph(&index);

        // 4 nodes from the pipeline fixture
        assert_eq!(graph.nodes.len(), 4);
        assert!(graph.nodes.contains_key("/perception/cropbox"));
        assert!(graph.nodes.contains_key("/perception/ground_filter"));
        assert!(graph.nodes.contains_key("/perception/fusion"));
        assert!(graph.nodes.contains_key("/perception/tracker"));

        // Edges exist (one per pub-sub pair on each topic)
        assert!(!graph.edges.is_empty(), "expected edges in pipeline graph");

        // Topic publisher/subscriber lookup
        assert!(
            graph
                .topic_publishers
                .contains_key("/perception/cropped_points")
        );
        assert!(
            graph
                .topic_subscribers
                .contains_key("/perception/cropped_points")
        );
    }

    #[test]
    fn test_critical_path_parallel_takes_max_not_sum() {
        // Parallel pipeline:
        //   in → lidar (50ms) → fusion (20ms) → out
        //   in → camera (30ms) ↗
        // Critical path = max(50, 30) + 20 = 70ms (NOT sum 100ms)
        // The fixture declares max_latency_ms: 70 — should pass cleanly.
        let dump = make_dump(vec![scope(
            0,
            "manifest_parallel_pipeline",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        // The declared 70ms == critical path, so no scope-budget warning.
        let budget_warnings: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "scope-budget")
            .collect();
        assert!(
            budget_warnings.is_empty(),
            "expected no scope-budget warnings for parallel pipeline at exact \
             critical path budget, got: {budget_warnings:?}"
        );
    }

    #[test]
    fn test_critical_path_diagnostic_when_budget_too_tight() {
        use super::super::manifest_graph::{
            build_global_graph, critical_path, subgraph_for_scope_path, subtree_scope_ids,
        };

        let dump = make_dump(vec![scope(
            0,
            "manifest_parallel_pipeline",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);
        let graph = build_global_graph(&index);

        let subtree = subtree_scope_ids(&index, 0);
        let subgraph = subgraph_for_scope_path(
            &graph,
            subtree,
            &["/sensor/raw".to_string()],
            &["/perception/fused_objects".to_string()],
        );

        let cp = critical_path(&subgraph).expect("critical path should exist");

        // Should be 70 (max(50, 30) + 20), NOT 100 (sum)
        assert!(
            (cp.total_ms - 70.0).abs() < 0.001,
            "expected critical path 70ms, got {}",
            cp.total_ms
        );
        // Path should include the slowest branch (lidar) and fusion
        assert!(
            cp.nodes.iter().any(|n| n.contains("lidar_detector")),
            "critical path should pass through lidar (slower branch): {:?}",
            cp.nodes
        );
        assert!(
            cp.nodes.iter().any(|n| n.contains("fusion")),
            "critical path should reach fusion sink: {:?}",
            cp.nodes
        );
    }

    #[test]
    fn test_critical_path_mixed_transport_per_subscriber_override() {
        // Issue #44 — per-subscriber max_transport_ms override.
        //
        // Topology has two parallel branches that join at fusion. Topic
        // default transport is 10ms. The lidar branch overrides to 0ms
        // on the subscriber (intra-process collocation); the camera
        // branch inherits the 10ms default (network bridge).
        //
        // Critical path with the override:
        //   lidar:  50 + 0  + 20 = 70
        //   camera: 30 + 10 + 20 = 60
        //   max = 70
        //
        // Without per-sub override the lidar branch would be
        // 50 + 10 + 20 = 80, which would inflate the budget by 10ms.
        use super::super::manifest_graph::{
            build_global_graph, critical_path, subgraph_for_scope_path, subtree_scope_ids,
        };

        let dump = make_dump(vec![scope(
            0,
            "manifest_mixed_transport",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);
        let graph = build_global_graph(&index);

        // Inspect the lidar→fusion edge to confirm the override is applied.
        let lidar_edge = graph
            .edges
            .iter()
            .find(|e| {
                e.from.contains("lidar_detector")
                    && e.to.contains("fusion")
                    && e.sub_endpoint == "lidar"
            })
            .expect("lidar→fusion edge missing");
        assert_eq!(
            lidar_edge.max_transport_ms,
            Some(0.0),
            "lidar→fusion edge should use sub-endpoint override (0ms), got {:?}",
            lidar_edge.max_transport_ms
        );

        // Camera edge should inherit the topic default.
        let camera_edge = graph
            .edges
            .iter()
            .find(|e| {
                e.from.contains("camera_detector")
                    && e.to.contains("fusion")
                    && e.sub_endpoint == "camera"
            })
            .expect("camera→fusion edge missing");
        assert_eq!(
            camera_edge.max_transport_ms,
            Some(10.0),
            "camera→fusion edge should inherit topic default (10ms), got {:?}",
            camera_edge.max_transport_ms
        );

        // Critical path should reflect the per-sub override → 70ms.
        let subtree = subtree_scope_ids(&index, 0);
        let subgraph = subgraph_for_scope_path(
            &graph,
            subtree,
            &["/sensor/raw".to_string()],
            &["/perception/fused_objects".to_string()],
        );
        let cp = critical_path(&subgraph).expect("critical path should exist");
        assert!(
            (cp.total_ms - 70.0).abs() < 0.001,
            "expected critical path 70ms (lidar 50 + 0 transport + fusion 20), \
             got {}",
            cp.total_ms
        );
        assert!(
            cp.nodes.iter().any(|n| n.contains("lidar_detector")),
            "critical path should pass through the lidar branch (slowest with \
             override), got: {:?}",
            cp.nodes
        );

        // The declared 70ms scope budget matches the critical path → no
        // scope-budget warning expected.
        let budget_warnings: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "scope-budget")
            .collect();
        assert!(
            budget_warnings.is_empty(),
            "expected no scope-budget warnings at exact critical path budget, \
             got: {budget_warnings:?}"
        );
    }

    #[test]
    fn test_cross_scope_qos_match_pubsub_mismatch() {
        // Issue #45 — cross-scope qos-match.
        //
        // Pub scope declares topic with `reliability: best_effort`. Sub
        // scope declares per-endpoint override `reliability: reliable`.
        // After cross-scope merge:
        //   effective pub qos = best_effort (inherits topic default)
        //   effective sub qos = reliable     (endpoint override)
        // Pub offered (best_effort) < sub requested (reliable) → error.
        let dump = make_dump(vec![
            scope(
                0,
                "manifest_qos_match_pubsub_pub",
                "manifest.launch.xml",
                "",
                None,
            ),
            scope(
                1,
                "manifest_qos_match_pubsub_sub",
                "manifest.launch.xml",
                "",
                Some(0),
            ),
        ]);
        let index = overlay_index(&dump);

        let qos_errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "qos-match")
            .collect();

        assert!(
            qos_errors.iter().any(|d| d.message.contains("reliability")
                && d.message.contains("best_effort")
                && d.message.contains("reliable")),
            "expected cross-scope reliability mismatch error, got: {qos_errors:?}"
        );
    }

    // ── Phase 35.5: cross-scope rate hierarchy ──

    #[test]
    fn test_cross_scope_rate_publisher_too_slow() {
        // Pub min_rate_hz=5 in one scope, topic rate_hz=10 → error.
        let dump = make_dump(vec![
            scope(0, "manifest_rate_pub_slow", "manifest.launch.xml", "", None),
            scope(
                1,
                "manifest_rate_sub_demand",
                "manifest.launch.xml",
                "",
                Some(0),
            ),
        ]);
        let index = overlay_index(&dump);

        let rate_errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "rate-hierarchy")
            .collect();

        // Two errors expected:
        //   1. pub min_rate_hz (5) < topic rate_hz (10)
        //   2. effective delivery (10) < sub min_rate_hz (8) — actually 10 >= 8, OK
        // Wait — effective delivery is 10 with no drop, sub demands 8, so OK.
        // Only the publisher error should fire.
        assert!(
            rate_errors
                .iter()
                .any(|d| d.message.contains("publisher cannot sustain")),
            "expected publisher rate error, got: {rate_errors:?}"
        );
    }

    #[test]
    fn test_cross_scope_rate_publisher_only_no_error() {
        // Just the publisher manifest — no subscriber demand to check.
        // Pub min_rate_hz=5, channel rate_hz=10 still triggers the
        // pub-vs-channel error (publisher can't sustain).
        let dump = make_dump(vec![scope(
            0,
            "manifest_rate_pub_slow",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        let rate_errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "rate-hierarchy")
            .collect();

        // The pub-vs-channel error fires regardless of subscribers
        assert!(
            rate_errors
                .iter()
                .any(|d| d.message.contains("publisher cannot sustain")),
            "expected pub rate error, got: {rate_errors:?}"
        );
    }

    #[test]
    fn test_cross_scope_rate_subscriber_only_no_error() {
        // Sub demands 8 Hz but no publisher in the merged tree.
        // The dangling-entity check fires, but rate-hierarchy can't
        // check publisher rate (none exists).
        let dump = make_dump(vec![scope(
            0,
            "manifest_rate_sub_demand",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = overlay_index(&dump);

        let rate_errors: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "rate-hierarchy")
            .collect();
        assert!(
            rate_errors.is_empty(),
            "expected no rate errors when no publisher exists, got: {rate_errors:?}"
        );

        // dangling-entity warning IS expected
        let dangling: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "dangling-entity" && d.message.contains("/shared_stream"))
            .collect();
        assert_eq!(dangling.len(), 1);
    }

    #[test]
    fn test_subtree_scope_ids_includes_descendants() {
        use super::super::manifest_graph::subtree_scope_ids;

        // Build a 3-level scope tree: 0 → 1 → 2
        let dump = make_dump(vec![
            scope(0, "manifest_simple", "manifest.launch.xml", "/", None),
            scope(1, "manifest_simple", "manifest.launch.xml", "/a", Some(0)),
            scope(2, "manifest_simple", "manifest.launch.xml", "/a/b", Some(1)),
        ]);
        let index = overlay_index(&dump);

        let subtree_of_0 = subtree_scope_ids(&index, 0);
        assert!(subtree_of_0.contains(&0));
        assert!(subtree_of_0.contains(&1));
        assert!(subtree_of_0.contains(&2));

        let subtree_of_1 = subtree_scope_ids(&index, 1);
        assert!(!subtree_of_1.contains(&0));
        assert!(subtree_of_1.contains(&1));
        assert!(subtree_of_1.contains(&2));

        let subtree_of_2 = subtree_scope_ids(&index, 2);
        assert!(!subtree_of_2.contains(&0));
        assert!(!subtree_of_2.contains(&1));
        assert!(subtree_of_2.contains(&2));
    }

    // ── Phase 40.2/40.6: two-step contract resolution (overlay/provider) ──

    fn write_marker_manifest(path: &std::path::Path, marker: &str) {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).unwrap();
        }
        std::fs::write(
            path,
            format!(
                "version: 1\n\
                 nodes:\n  \
                 source:\n    pub:\n      {marker}: {{}}\n\
                 topics:\n  \
                 {marker}:\n    type: std_msgs/msg/String\n    pub: [source/{marker}]\n"
            ),
        )
        .unwrap();
    }

    fn scope_with_path(
        id: usize,
        pkg: Option<&str>,
        file: &str,
        path: Option<&std::path::Path>,
    ) -> ScopeEntry {
        ScopeEntry {
            id,
            origin: Some(ScopeOrigin {
                pkg: pkg.map(String::from),
                file: file.to_string(),
                path: path.map(|p| p.to_string_lossy().into_owned()),
            }),
            ns: "".to_string(),
            args: HashMap::new(),
            parent: None,
        }
    }

    #[test]
    fn test_contract_stem_all_launch_suffixes() {
        assert_eq!(contract_stem("bringup.launch.xml"), "bringup");
        assert_eq!(contract_stem("bringup.launch.py"), "bringup");
        assert_eq!(contract_stem("bringup.launch.yaml"), "bringup");
        assert_eq!(contract_stem("bringup.launch"), "bringup");
        // Unrecognized suffix passes through unchanged.
        assert_eq!(contract_stem("bringup.xml"), "bringup.xml");
    }

    #[test]
    fn test_resolve_provider_path_none_when_origin_path_missing() {
        let scope = scope_with_path(0, Some("mypkg"), "bringup.launch.xml", None);
        assert!(resolve_provider_path(&scope).is_none());
    }

    #[test]
    fn test_resolve_provider_path_uses_origin_dir() {
        let launch_file = PathBuf::from("/opt/ros/share/mypkg/launch/bringup.launch.xml");
        let scope = scope_with_path(0, Some("mypkg"), "bringup.launch.xml", Some(&launch_file));
        let path = resolve_provider_path(&scope).unwrap();
        assert_eq!(
            path,
            PathBuf::from("/opt/ros/share/mypkg/launch/bringup.contract.yaml")
        );
    }

    #[test]
    fn test_precedence_overlay_beats_provider() {
        let tmp = tempfile::TempDir::new().unwrap();
        let root = tmp.path();

        let launch_dir = root.join("install/mypkg/share/mypkg/launch");
        let launch_file = launch_dir.join("bringup.launch.xml");
        write_marker_manifest(&launch_dir.join("bringup.contract.yaml"), "provider_marker");
        std::fs::write(&launch_file, "<launch/>").unwrap();

        let overlay_root = root.join("overlay");
        write_marker_manifest(
            &overlay_root.join("mypkg/launch/bringup.contract.yaml"),
            "overlay_marker",
        );

        let scope = scope_with_path(0, Some("mypkg"), "bringup.launch.xml", Some(&launch_file));
        let dump = make_dump(vec![scope]);

        // Both channels present — overlay wins.
        let sources = ContractSources {
            overlay: Some(overlay_root.clone()),
            provider: true,
        };
        let index = load_manifests(&dump, &sources).unwrap();
        assert_eq!(index.manifests[&0].channel, ContractChannel::Overlay);
        assert!(index.topics.contains_key("/overlay_marker"));

        // Overlay removed — provider wins.
        let sources = ContractSources {
            overlay: None,
            provider: true,
        };
        let dump = make_dump(vec![scope_with_path(
            0,
            Some("mypkg"),
            "bringup.launch.xml",
            Some(&launch_file),
        )]);
        let index = load_manifests(&dump, &sources).unwrap();
        assert_eq!(index.manifests[&0].channel, ContractChannel::Provider);
        assert!(index.topics.contains_key("/provider_marker"));
    }

    #[test]
    fn test_provider_skipped_when_origin_path_is_none() {
        // Old record.json (pre-Phase-40) has no origin.path — provider
        // lookup must not panic and must fall through (here: to nothing,
        // since no other channel is configured).
        let scope = scope_with_path(0, Some("mypkg"), "bringup.launch.xml", None);
        let dump = make_dump(vec![scope]);
        let sources = ContractSources {
            overlay: None,
            provider: true,
        };
        let index = load_manifests(&dump, &sources).unwrap();
        assert!(index.manifests.is_empty());
    }

    #[test]
    fn test_no_provider_contracts_flag_disables_provider_channel() {
        let tmp = tempfile::TempDir::new().unwrap();
        let root = tmp.path();
        let launch_dir = root.join("share/mypkg/launch");
        let launch_file = launch_dir.join("bringup.launch.xml");
        write_marker_manifest(&launch_dir.join("bringup.contract.yaml"), "provider_marker");
        std::fs::write(&launch_file, "<launch/>").unwrap();

        let scope = scope_with_path(0, Some("mypkg"), "bringup.launch.xml", Some(&launch_file));
        let dump = make_dump(vec![scope]);

        // provider: false emulates --no-provider-contracts — sidecar exists
        // on disk but must not be picked up.
        let sources = ContractSources {
            overlay: None,
            provider: false,
        };
        let index = load_manifests(&dump, &sources).unwrap();
        assert!(index.manifests.is_empty());
    }

    #[test]
    fn test_overlay_underscore_pkg_convention() {
        let tmp = tempfile::TempDir::new().unwrap();
        let overlay_root = tmp.path().join("overlay");
        write_marker_manifest(
            &overlay_root.join("_/launch/bringup.contract.yaml"),
            "overlay_marker",
        );

        // pkg: None — uses the "_" convention shared by overlay/provider.
        let scope = scope_with_path(0, None, "bringup.launch.xml", None);
        let dump = make_dump(vec![scope]);
        let sources = ContractSources {
            overlay: Some(overlay_root),
            provider: false,
        };
        let index = load_manifests(&dump, &sources).unwrap();
        assert_eq!(index.manifests.len(), 1);
        assert_eq!(index.manifests[&0].channel, ContractChannel::Overlay);
        assert!(index.topics.contains_key("/overlay_marker"));
    }

    // ── Phase 41.3: overlay-root discovery ──

    /// Serializes tests that mutate `PLAY_LAUNCH_CONTRACTS`/`XDG_CONFIG_HOME`
    /// — `std::env` is process-global, and `cargo test` runs tests on
    /// multiple threads within the same process by default.
    static ENV_GUARD: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// RAII guard that snapshots and restores an env var, so a panicking
    /// assertion inside a test still leaves the environment clean for
    /// whichever test runs next while holding `ENV_GUARD`.
    struct EnvVarRestore {
        key: &'static str,
        original: Option<String>,
    }

    impl EnvVarRestore {
        fn unset(key: &'static str) -> Self {
            let original = std::env::var(key).ok();
            // SAFETY: serialized by `ENV_GUARD`, held for the guard's lifetime.
            unsafe { std::env::remove_var(key) };
            Self { key, original }
        }

        fn set(key: &'static str, value: &std::path::Path) -> Self {
            let original = std::env::var(key).ok();
            // SAFETY: serialized by `ENV_GUARD`, held for the guard's lifetime.
            unsafe { std::env::set_var(key, value) };
            Self { key, original }
        }
    }

    impl Drop for EnvVarRestore {
        fn drop(&mut self) {
            // SAFETY: serialized by `ENV_GUARD`.
            match &self.original {
                Some(v) => unsafe { std::env::set_var(self.key, v) },
                None => unsafe { std::env::remove_var(self.key) },
            }
        }
    }

    #[test]
    fn test_discover_overlay_root_explicit_wins_over_other_channels() {
        let _lock = ENV_GUARD.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        let env_dir = tmp.path().join("env_root");
        std::fs::create_dir_all(&env_dir).unwrap();
        let _env = EnvVarRestore::set("PLAY_LAUNCH_CONTRACTS", &env_dir);

        let explicit = tmp.path().join("explicit_root");
        std::fs::create_dir_all(&explicit).unwrap();
        assert_eq!(
            discover_overlay_root(Some(&explicit)).unwrap(),
            Some(explicit.clone())
        );
    }

    /// A `--contracts` path that does not exist is a user error, not a
    /// reason to quietly use something else.
    ///
    /// It used to be accepted unconditionally, so the overlay simply resolved
    /// no files and the provider sidecar supplied the contract instead:
    /// "0 overlay, 1 provider", exit 0. A CI gate would report success having
    /// validated against the vendor's contract rather than the overlay it was
    /// told to use. `--sched` already hard-errors on a missing explicit path.
    #[test]
    fn test_discover_overlay_root_explicit_must_exist() {
        let _lock = ENV_GUARD.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        // A valid fallback is present, to prove we do not silently take it.
        let env_dir = tmp.path().join("env_root");
        std::fs::create_dir_all(&env_dir).unwrap();
        let _env = EnvVarRestore::set("PLAY_LAUNCH_CONTRACTS", &env_dir);

        let missing = tmp.path().join("does_not_exist");
        let err = discover_overlay_root(Some(&missing)).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("is not a directory"), "{msg}");
        assert!(msg.contains("does_not_exist"), "names the path: {msg}");
    }

    #[test]
    fn test_discover_overlay_root_env_var_wins_over_xdg() {
        let _lock = ENV_GUARD.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        let env_dir = tmp.path().join("env_root");
        std::fs::create_dir_all(&env_dir).unwrap();
        let xdg_dir = tmp.path().join("xdg_root/play_launch/contracts");
        std::fs::create_dir_all(&xdg_dir).unwrap();

        let _env = EnvVarRestore::set("PLAY_LAUNCH_CONTRACTS", &env_dir);
        let _xdg = EnvVarRestore::set("XDG_CONFIG_HOME", &tmp.path().join("xdg_root"));

        assert_eq!(discover_overlay_root(None).unwrap(), Some(env_dir));
    }

    #[test]
    fn test_discover_overlay_root_falls_back_to_xdg_when_env_var_missing() {
        let _lock = ENV_GUARD.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        let xdg_base = tmp.path().join("xdg_root");
        let xdg_dir = xdg_base.join("play_launch/contracts");
        std::fs::create_dir_all(&xdg_dir).unwrap();

        let _env = EnvVarRestore::unset("PLAY_LAUNCH_CONTRACTS");
        let _xdg = EnvVarRestore::set("XDG_CONFIG_HOME", &xdg_base);

        assert_eq!(discover_overlay_root(None).unwrap(), Some(xdg_dir));
    }

    #[test]
    fn test_discover_overlay_root_env_var_ignored_when_dir_missing() {
        let _lock = ENV_GUARD.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        let xdg_base = tmp.path().join("xdg_root");
        let xdg_dir = xdg_base.join("play_launch/contracts");
        std::fs::create_dir_all(&xdg_dir).unwrap();

        // PLAY_LAUNCH_CONTRACTS points at a directory that doesn't exist —
        // must be skipped in favor of XDG, not just silently returned.
        let _env = EnvVarRestore::set("PLAY_LAUNCH_CONTRACTS", &tmp.path().join("nope"));
        let _xdg = EnvVarRestore::set("XDG_CONFIG_HOME", &xdg_base);

        assert_eq!(discover_overlay_root(None).unwrap(), Some(xdg_dir));
    }

    #[test]
    fn test_discover_overlay_root_empty_xdg_config_home_treated_as_unset() {
        let _lock = ENV_GUARD.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        let home = tmp.path().join("home");
        let default_xdg_dir = home.join(".config/play_launch/contracts");
        std::fs::create_dir_all(&default_xdg_dir).unwrap();

        let _env = EnvVarRestore::unset("PLAY_LAUNCH_CONTRACTS");
        // XDG spec: empty value must be treated as unset → fall back to
        // $HOME/.config, not to a relative "play_launch/contracts" path.
        let _xdg = EnvVarRestore::set("XDG_CONFIG_HOME", std::path::Path::new(""));
        let _home = EnvVarRestore::set("HOME", &home);

        assert_eq!(discover_overlay_root(None).unwrap(), Some(default_xdg_dir));
    }

    #[test]
    fn test_discover_overlay_root_none_when_nothing_exists() {
        let _lock = ENV_GUARD.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();

        let _env = EnvVarRestore::set("PLAY_LAUNCH_CONTRACTS", &tmp.path().join("nope1"));
        let _xdg = EnvVarRestore::set("XDG_CONFIG_HOME", &tmp.path().join("nope2"));

        // This assumes /etc/play_launch/contracts doesn't exist on the test
        // host — true for any stock CI/dev box; skip if it somehow does.
        if std::path::Path::new("/etc/play_launch/contracts").is_dir() {
            eprintln!(
                "SKIP: /etc/play_launch/contracts exists on this host, \
                 can't test the all-missing case"
            );
            return;
        }

        assert_eq!(discover_overlay_root(None).unwrap(), None);
    }

    /// A scope path written in the Vocabulary v2 spelling must resolve to the
    /// same input topics as the legacy `input:` list.
    ///
    /// This is not style. `explicit-trigger` actively tells authors to migrate
    /// to `trigger:`, and reading `decl.input` alone made that migration
    /// resolve to ZERO input topics — which does not fail. It produces a scope
    /// path with no route, so the mapper silently falls back to ranking by
    /// budget. On `rt_workspace` the provenance went from
    /// `points_to_cmd segment drain 2/2` to `non-chain budget_ms=5` while both
    /// priorities stayed 39/38 by coincidence of the fixture, so no numeric
    /// check would have caught it.
    #[test]
    fn a_scope_path_resolves_the_same_from_either_trigger_spelling() {
        use ros_launch_manifest_types::parse::parse_manifest_str;

        let legacy = r#"
version: 1
paths:
  e2e:
    input: points_raw
    output: [cmd]
    max_latency: 30ms
"#;
        let v2 = r#"
version: 1
paths:
  e2e:
    trigger: { input: [points_raw] }
    output: [cmd]
    max_latency: 30ms
"#;

        let resolve_one = |yaml: &str| {
            let manifest = parse_manifest_str(yaml).expect("parses");
            let scope = ScopeEntry {
                id: 0,
                origin: None,
                ns: "/perception".to_string(),
                args: Default::default(),
                parent: None,
            };
            let mut index = ManifestIndex::default();
            resolve_scope_paths(&manifest, &scope, &mut index);
            index.scope_paths
        };

        let legacy_paths = resolve_one(legacy);
        let v2_paths = resolve_one(v2);

        assert_eq!(legacy_paths.len(), 1);
        assert_eq!(
            legacy_paths[0].input_topics,
            vec!["/perception/points_raw".to_string()],
            "legacy spelling lost its input"
        );
        assert_eq!(
            v2_paths[0].input_topics, legacy_paths[0].input_topics,
            "the v2 spelling must resolve the same input topics as the legacy one — \
             an empty list here is a scope path with no route, which degrades \
             silently rather than failing"
        );
        assert_eq!(v2_paths[0].output_topics, legacy_paths[0].output_topics);
    }
}
