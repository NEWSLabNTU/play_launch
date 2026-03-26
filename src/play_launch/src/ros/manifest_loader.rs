//! Manifest loading and resolution using the scope table from record.json.
//!
//! For each file scope in the launch tree, looks up a manifest YAML file at
//! `<manifest_dir>/<pkg>/<file>.yaml`, parses it, runs static checks, and
//! builds a resolved index with fully-qualified topic/node names.

use super::launch_dump::{LaunchDump, ScopeEntry};
use ros_launch_manifest_check::{Diagnostic, Severity, run_checks};
use ros_launch_manifest_types::{Manifest, parse_manifest};
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use tracing::{debug, info, warn};

/// A loaded and resolved manifest bound to a specific scope.
#[derive(Debug, Clone)]
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
    /// Static check diagnostics.
    pub diagnostics: Vec<Diagnostic>,
}

/// Resolved topic entry with fully-qualified names.
#[derive(Debug, Clone)]
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
pub fn load_manifests(launch_dump: &LaunchDump, manifest_dir: &Path) -> eyre::Result<ManifestIndex> {
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

        let manifest = match parse_manifest(&path) {
            Ok(m) => m,
            Err(e) => {
                warn!(
                    "Failed to parse manifest {}: {}",
                    path.display(),
                    e
                );
                continue;
            }
        };

        // Run static checks
        let check_result = run_checks(&manifest);

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

        let errors = check_result.diagnostics.iter().filter(|d| d.severity == Severity::Error).count();
        let warnings = check_result.diagnostics.iter().filter(|d| d.severity == Severity::Warning).count();
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
                diagnostics: check_result.diagnostics,
                manifest,
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
        assert_eq!(qualify_name("/perception", "cropped"), "/perception/cropped");
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
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../ros-launch-manifest/tests/fixtures")
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

    #[test]
    fn test_load_simple_manifest() {
        let dump = make_dump(vec![scope(0, "manifest_simple", "manifest.launch.xml", "", None)]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert_eq!(index.manifests.len(), 1);
        assert!(index.manifests.contains_key(&0));
        assert_eq!(index.manifests[&0].manifest.nodes.len(), 2);

        // Topics resolved with "/" prefix
        assert!(index.topics.contains_key("/chatter"), "topics: {:?}", index.topics.keys().collect::<Vec<_>>());
        let chatter = &index.topics["/chatter"];
        assert_eq!(chatter.msg_type, "std_msgs/msg/String");
        assert_eq!(chatter.publishers, vec!["/talker/chatter"]);
        assert_eq!(chatter.subscribers, vec!["/listener/chatter"]);
    }

    #[test]
    fn test_load_with_namespace() {
        let dump = make_dump(vec![scope(0, "manifest_simple", "manifest.launch.xml", "/demo", None)]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert!(index.topics.contains_key("/demo/chatter"),
            "expected /demo/chatter, got: {:?}", index.topics.keys().collect::<Vec<_>>());
        assert_eq!(index.topics["/demo/chatter"].publishers, vec!["/demo/talker/chatter"]);
    }

    #[test]
    fn test_load_violations_has_errors() {
        let dump = make_dump(vec![scope(0, "manifest_violations", "manifest.launch.xml", "", None)]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert!(index.total_errors >= 5, "expected >=5 errors, got {}", index.total_errors);
        assert!(index.total_warnings > 0);
    }

    #[test]
    fn test_load_missing_manifest_skipped() {
        let dump = make_dump(vec![scope(0, "no_such_pkg", "no_such.launch.xml", "", None)]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert!(index.manifests.is_empty());
        assert_eq!(index.total_errors, 0);
    }

    #[test]
    fn test_load_group_scope_skipped() {
        let dump = make_dump(vec![ScopeEntry {
            id: 0,
            origin: None,
            ns: "/group".to_string(),
            args: HashMap::new(),
            parent: None,
        }]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();
        assert!(index.manifests.is_empty());
    }

    #[test]
    fn test_load_pipeline_node_paths() {
        let dump = make_dump(vec![scope(0, "manifest_pipeline", "manifest.launch.xml", "/perception", None)]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        // 4 nodes with 1 path each + fusion has 1 path = 4 total
        assert!(index.node_paths.len() >= 4, "expected >=4 node paths, got {}", index.node_paths.len());

        let fqns: Vec<&str> = index.node_paths.iter().map(|p| p.node_fqn.as_str()).collect();
        assert!(fqns.contains(&"/perception/cropbox"), "fqns: {fqns:?}");
        assert!(fqns.contains(&"/perception/tracker"), "fqns: {fqns:?}");

        // Scope-level paths
        assert!(index.scope_paths.contains_key(&0));
    }

    #[test]
    fn test_load_multiple_scopes() {
        let dump = make_dump(vec![
            scope(0, "manifest_simple", "manifest.launch.xml", "", None),
            scope(1, "manifest_pipeline", "manifest.launch.xml", "/perception", Some(0)),
        ]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert_eq!(index.manifests.len(), 2);
        assert!(index.topics.contains_key("/chatter"));
        assert!(index.topics.contains_key("/perception/cropped_points"));
    }

    #[test]
    fn test_load_ndt_clean() {
        let dump = make_dump(vec![scope(0, "manifest_ndt", "manifest.launch.xml", "/localization", None)]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert_eq!(index.total_errors, 0, "NDT should be clean");
        assert!(index.topics.contains_key("/localization/ndt_pose"));
    }

    #[test]
    fn test_load_periodic_clean() {
        let dump = make_dump(vec![scope(0, "manifest_periodic", "manifest.launch.xml", "/control", None)]);
        let index = load_manifests(&dump, &fixture_dir()).unwrap();

        assert_eq!(index.total_errors, 0, "periodic should be clean");
        assert_eq!(index.manifests[&0].manifest.nodes.len(), 3);
    }
}
