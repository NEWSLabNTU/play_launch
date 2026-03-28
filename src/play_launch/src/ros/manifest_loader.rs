//! Manifest loading and resolution using the scope table from record.json.
//!
//! For each file scope in the launch tree, looks up a manifest YAML file at
//! `<manifest_dir>/<pkg>/<file>.yaml`, parses it, runs static checks, and
//! builds a resolved index with fully-qualified topic/node names.

use super::launch_dump::{LaunchDump, ScopeEntry};
use ros_launch_manifest_check::{Diagnostic, Severity, run_checks_with_spans};
use ros_launch_manifest_types::{
    Manifest, filter_manifest, parse_manifest_with_spans, resolve_args, substitute_manifest,
};
use std::{
    collections::{BTreeMap, HashMap},
    path::{Path, PathBuf},
};
use tracing::{debug, info, warn};

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
    /// The raw parsed manifest.
    pub manifest: Manifest,
    /// Original YAML source text (for codespan-reporting).
    pub source: String,
    /// Static check diagnostics.
    pub diagnostics: Vec<Diagnostic>,
}

/// Resolved topic entry with fully-qualified names.
#[derive(Debug, Clone)]
#[allow(dead_code)] // Fields used by 31.5 runtime monitors and 31.6 audit
pub struct ResolvedTopic {
    /// Fully-qualified topic name (with namespace prefix).
    pub fqn: String,
    /// Message type.
    pub msg_type: String,
    /// QoS declaration (if any).
    pub qos: Option<ros_launch_manifest_types::QosDecl>,
    /// Publisher endpoint FQNs (namespace-prefixed).
    pub publishers: Vec<String>,
    /// Subscriber endpoint FQNs (namespace-prefixed).
    pub subscribers: Vec<String>,
    /// Expected rate (Hz).
    pub rate_hz: Option<f64>,
    /// Drop tolerance.
    pub drop: Option<ros_launch_manifest_types::DropSpec>,
    /// Scope ID this topic belongs to.
    pub scope_id: usize,
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

/// The complete resolved manifest index for the launch tree.
#[derive(Debug, Default)]
pub struct ManifestIndex {
    /// Manifests by scope ID.
    pub manifests: HashMap<usize, ResolvedManifest>,
    /// All resolved topics (FQN → topic info).
    pub topics: BTreeMap<String, ResolvedTopic>,
    /// All resolved node paths.
    pub node_paths: Vec<ResolvedNodePath>,
    /// Scope-level paths by scope ID.
    pub scope_paths: HashMap<usize, Vec<(String, ros_launch_manifest_types::PathDecl)>>,
    /// Total errors across all manifests.
    pub total_errors: usize,
    /// Total warnings across all manifests.
    pub total_warnings: usize,
}

/// Load manifests for all file scopes in the launch tree.
///
/// For each scope with a known `(pkg, file)` origin, looks up
/// `<manifest_dir>/<pkg>/<file>.yaml`. Scopes without a matching
/// manifest file are silently skipped.
pub fn load_manifests(
    launch_dump: &LaunchDump,
    manifest_dir: &Path,
) -> eyre::Result<ManifestIndex> {
    let mut index = ManifestIndex::default();
    let mut loaded = 0usize;
    let mut skipped = 0usize;

    for scope in &launch_dump.scopes {
        if !scope.is_file_scope() {
            continue;
        }

        let manifest_path = resolve_manifest_path(scope, manifest_dir);
        let Some(path) = manifest_path else {
            skipped += 1;
            continue;
        };

        if !path.exists() {
            debug!(
                "No manifest for scope {} ({}/{}): {} not found",
                scope.id,
                scope.pkg().unwrap_or("?"),
                scope.file().unwrap_or("?"),
                path.display()
            );
            skipped += 1;
            continue;
        }

        debug!(
            "Loading manifest for scope {} ({}/{}): {}",
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

        // Run static checks with span resolution
        let check_result = run_checks_with_spans(&manifest, parsed.spans);

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

        // Resolve names with namespace prefix
        resolve_topics(&manifest, scope, &mut index);
        resolve_node_paths(&manifest, scope, &mut index);
        resolve_scope_paths(&manifest, scope, &mut index);

        index.manifests.insert(
            scope.id,
            ResolvedManifest {
                scope_id: scope.id,
                pkg: scope.pkg().map(String::from),
                file: scope.file().unwrap_or("").to_string(),
                ns: scope.ns.clone(),
                manifest,
                source,
                diagnostics: check_result.diagnostics,
            },
        );

        loaded += 1;
    }

    if loaded > 0 {
        info!(
            "Loaded {loaded} manifest(s) ({skipped} scopes without manifests, {} errors, {} warnings)",
            index.total_errors, index.total_warnings
        );
    } else {
        debug!("No manifests found in {}", manifest_dir.display());
    }

    Ok(index)
}

/// Resolve the manifest file path for a given scope.
///
/// Layout: `<manifest_dir>/<pkg>/<file>.yaml`
/// If pkg is None, looks in `<manifest_dir>/_/<file>.yaml`.
fn resolve_manifest_path(scope: &ScopeEntry, manifest_dir: &Path) -> Option<PathBuf> {
    let file = scope.file()?;

    // Strip launch file extension and add .yaml
    let stem = file
        .strip_suffix(".launch.xml")
        .or_else(|| file.strip_suffix(".launch.py"))
        .or_else(|| file.strip_suffix(".launch"))
        .unwrap_or(file);

    let pkg_dir = scope.pkg().unwrap_or("_");
    Some(manifest_dir.join(pkg_dir).join(format!("{stem}.yaml")))
}

/// Resolve topic declarations into fully-qualified names.
fn resolve_topics(manifest: &Manifest, scope: &ScopeEntry, index: &mut ManifestIndex) {
    let ns = &scope.ns;

    for (topic_name, topic_decl) in &manifest.topics {
        let fqn = qualify_name(ns, topic_name);

        // Qualify publisher/subscriber endpoint references
        let publishers: Vec<String> = topic_decl
            .publishers
            .iter()
            .map(|ep_ref| qualify_endpoint_ref(ns, ep_ref))
            .collect();
        let subscribers: Vec<String> = topic_decl
            .subscribers
            .iter()
            .map(|ep_ref| qualify_endpoint_ref(ns, ep_ref))
            .collect();

        index.topics.insert(
            fqn.clone(),
            ResolvedTopic {
                fqn,
                msg_type: topic_decl.msg_type.clone(),
                qos: topic_decl.qos.clone(),
                publishers,
                subscribers,
                rate_hz: topic_decl.rate_hz,
                drop: topic_decl.drop.clone(),
                scope_id: scope.id,
            },
        );
    }
}

/// Resolve node path declarations.
fn resolve_node_paths(manifest: &Manifest, scope: &ScopeEntry, index: &mut ManifestIndex) {
    let ns = &scope.ns;

    for (node_name, node_decl) in &manifest.nodes {
        let node_fqn = qualify_name(ns, node_name);

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
fn resolve_scope_paths(manifest: &Manifest, scope: &ScopeEntry, index: &mut ManifestIndex) {
    if manifest.paths.is_empty() {
        return;
    }

    let paths: Vec<(String, ros_launch_manifest_types::PathDecl)> = manifest
        .paths
        .iter()
        .map(|(name, decl)| (name.clone(), decl.clone()))
        .collect();

    index.scope_paths.insert(scope.id, paths);
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

    #[test]
    fn test_resolve_manifest_path() {
        use super::super::launch_dump::{ScopeEntry, ScopeOrigin};

        let scope = ScopeEntry {
            id: 1,
            origin: Some(ScopeOrigin {
                pkg: Some("tier4_perception_launch".to_string()),
                file: "perception.launch.xml".to_string(),
            }),
            ns: "/perception".to_string(),
            args: HashMap::new(),
            parent: Some(0),
        };

        let path = resolve_manifest_path(&scope, Path::new("/manifests")).unwrap();
        assert_eq!(
            path,
            PathBuf::from("/manifests/tier4_perception_launch/perception.yaml")
        );
    }

    #[test]
    fn test_resolve_manifest_path_no_pkg() {
        use super::super::launch_dump::{ScopeEntry, ScopeOrigin};

        let scope = ScopeEntry {
            id: 2,
            origin: Some(ScopeOrigin {
                pkg: None,
                file: "local.launch.py".to_string(),
            }),
            ns: "".to_string(),
            args: HashMap::new(),
            parent: Some(0),
        };

        let path = resolve_manifest_path(&scope, Path::new("/manifests")).unwrap();
        assert_eq!(path, PathBuf::from("/manifests/_/local.yaml"));
    }

    #[test]
    fn test_resolve_manifest_path_group_scope() {
        use super::super::launch_dump::ScopeEntry;

        let scope = ScopeEntry {
            id: 3,
            origin: None,
            ns: "/group".to_string(),
            args: HashMap::new(),
            parent: Some(0),
        };

        // Group scopes have no file, so no manifest path
        assert!(resolve_manifest_path(&scope, Path::new("/manifests")).is_none());
    }

    // ── Integration tests using fixture YAML files ──

    use super::super::launch_dump::{LaunchDump, ScopeOrigin};

    fn fixture_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../ros-launch-manifest/tests/fixtures")
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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        assert_eq!(chatter.scope_id, 0);
    }

    #[test]
    fn test_multiple_scopes_no_collision() {
        let dump = make_dump(vec![
            scope(0, "manifest_simple", "manifest.launch.xml", "/a", None),
            scope(1, "manifest_simple", "manifest.launch.xml", "/b", Some(0)),
        ]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert_eq!(index.manifests.len(), 2);
        // Same manifest loaded under different namespaces — topics don't collide
        assert!(index.topics.contains_key("/a/chatter"));
        assert!(index.topics.contains_key("/b/chatter"));
        assert_ne!(
            index.topics["/a/chatter"].scope_id,
            index.topics["/b/chatter"].scope_id,
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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        // All 5 invocations load successfully
        assert_eq!(index.manifests.len(), 5);
        // Each has its own topic with distinct namespace
        for i in 1..=5 {
            let key = format!("/system/topic_monitor_{i}/chatter");
            assert!(index.topics.contains_key(&key), "missing {key}");
            assert_eq!(index.topics[&key].scope_id, i - 1);
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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        assert_eq!(index.topics[deep_topic].scope_id, 10);
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

        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        // Both manifests loaded, even though scope 1 has no entities
        assert_eq!(index.manifests.len(), 2);
        assert!(index.manifests.contains_key(&1));
        assert!(!index.topics.is_empty());
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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
                let index = load_manifests(&dump, &fixture_dir()).unwrap();
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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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

        // 5 topics
        assert_eq!(index.topics.len(), 5);
        assert!(index.topics.contains_key("/perception/cropped_points"));
        assert!(index.topics.contains_key("/perception/no_ground_points"));
        assert!(index.topics.contains_key("/perception/camera_detections"));
        assert!(index.topics.contains_key("/perception/fused_objects"));
        assert!(index.topics.contains_key("/perception/tracked_objects"));

        // Scope paths
        assert!(index.scope_paths.contains_key(&0));
        let scope_paths = &index.scope_paths[&0];
        assert_eq!(scope_paths.len(), 1);
        assert_eq!(scope_paths[0].0, "perception");
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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        assert_eq!(ndt_paths[0].path.max_latency_ms, Some(30.0));

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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
    fn test_args_with_defaults_resolved() {
        // manifest_args fixture has args with defaults — they resolve even without scope args
        let dump = make_dump(vec![scope(
            0,
            "manifest_args",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert_eq!(index.manifests.len(), 1);
        // Default values should be substituted into topic types
        let input_topic = &index.topics["/input_data"];
        assert_eq!(input_topic.msg_type, "/default/input");
        let output_topic = &index.topics["/output_data"];
        assert_eq!(output_topic.msg_type, "/default/output");
    }

    #[test]
    fn test_args_overridden_by_scope() {
        let mut s = scope(0, "manifest_args", "manifest.launch.xml", "", None);
        s.args
            .insert("input_topic".into(), "sensor_msgs/msg/PointCloud2".into());
        s.args
            .insert("output_topic".into(), "autoware_msgs/msg/Objects".into());
        let dump = make_dump(vec![s]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert_eq!(index.manifests.len(), 1);
        // Scope args should override manifest defaults
        let input_topic = &index.topics["/input_data"];
        assert_eq!(input_topic.msg_type, "sensor_msgs/msg/PointCloud2");
        let output_topic = &index.topics["/output_data"];
        assert_eq!(output_topic.msg_type, "autoware_msgs/msg/Objects");
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
        let index = load_manifests(&dump, &fixture_dir()).unwrap();
        assert_eq!(index.manifests.len(), 1);
        assert!(index.topics.contains_key("/chatter"));
    }

    // ── Conditions ──

    #[test]
    fn test_conditions_default_args() {
        // With default args: use_feature_a=true, use_feature_b=false, sensor_model=velodyne
        let dump = make_dump(vec![scope(
            0,
            "manifest_conditions",
            "manifest.launch.xml",
            "",
            None,
        )]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

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
        let dump = make_dump(vec![s]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        let m = &index.manifests[&0].manifest;

        // feature_a_node excluded, feature_b_node included, legacy_node included (unless false)
        assert!(m.nodes.contains_key("always_present"));
        assert!(!m.nodes.contains_key("feature_a_node"));
        assert!(m.nodes.contains_key("feature_b_node"));
        assert!(m.nodes.contains_key("legacy_node"));
        // sensor_specific still present (sensor_model default = velodyne)
        assert!(m.nodes.contains_key("sensor_specific"));
    }

    #[test]
    fn test_conditions_sensor_model_mismatch() {
        // Override sensor_model to something other than velodyne
        let mut s = scope(0, "manifest_conditions", "manifest.launch.xml", "", None);
        s.args.insert("sensor_model".into(), "hesai".into());
        let dump = make_dump(vec![s]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        let m = &index.manifests[&0].manifest;

        // sensor_specific excluded (hesai != velodyne)
        assert!(!m.nodes.contains_key("sensor_specific"));
    }
}
