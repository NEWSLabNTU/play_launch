//! Domain extensions (phase 91 E4): load the rlx files a contract `use:`s,
//! check the keys it writes against them, and evaluate what they derive and
//! check.
//!
//! Order, which the design fixes (rlm `extension-system.md` section 4):
//!
//! 1. [`prepare`], before the fault walk: find and type-check the used
//!    extensions, refuse a key no used extension declares and a value its
//!    declaration refuses, and derive `hazard.detection` and `hazard.ftti`.
//!    A derived bound is written into the hazard as a bound with a
//!    derivation record (phase 91 E1), so every core rule, `--explain` and
//!    the model read it as they read a written one.
//! 2. [`derived_settle`], called by the settle lookup for each (hazard,
//!    rung): `settle.duration` is derived per hazard, since it may read the
//!    hazard's keys (an entry speed).
//! 3. [`run_checks`], after the fault walk: checks and reports, which may
//!    read `reaction_total` (the hazard's floor total).
//!
//! At each of the three, a `derive` or `check` that states `worst_case:` is
//! proved by Z3 at the contract's point ([`worst_case`], phase 91 E5): the
//! proved claims become a `worst case: proved ...` line in the derivation's
//! trace and the check's message; a refuted one is `worst-case-refuted`
//! (an error, with the counterexample and the domain that would make it
//! hold), an undecided one `worst-case-unknown` (a warning, an error under
//! `check --strict`). The prover is rlx's (`rlx::prove`); this module only
//! hands it the contract's point and reports.
//!
//! Finding extensions and profiles (phase 91 E6, E7): the `rlm/` directory
//! beside the contract, then `share/<pkg>/rlm/` of each package registered
//! under the ament resource `rlm_extensions` ([`search_dirs`]), plus the
//! reference profiles bundled with the checker; for each `use:` entry the
//! version `rlm.lock` names while it is installed and accepted, else the
//! highest the requirement accepts ([`prepare`], [`super::rlm_lock`]). A
//! profile gives its scope the severity scale and criticality map
//! ([`severity_scale`]).
//!
//! A `param` key names a node parameter. In `settle` it is a parameter of
//! the node whose path states the safe state (as a braking profile's are);
//! in `hazard` it is a parameter of the node that publishes the hazard's
//! guard, the one place a hazard is bound to a node.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use rlx::{
    eval::{Inputs, Value},
    typeck::{ItemRef, KeyKind, KeyTy, Need},
};
use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::{
    Bound, BudgetSource, Derivation, ExtKey, ExtValue, SafeState, SourceKind,
    profile::{self, Profile},
};

use super::manifest_loader::{ManifestIndex, ResolvedHazard, RungRole};

/// One extension file a contract uses.
#[derive(Debug, Clone)]
pub struct LoadedExt {
    pub name: String,
    pub version: String,
    pub sha256: String,
    pub path: PathBuf,
    source: String,
}

impl LoadedExt {
    /// The type-checked file. It passed at load, so it passes again.
    fn checked(&self) -> rlx::Checked<'_> {
        rlx::check_source(self.source.as_bytes()).expect("checked when it was loaded")
    }

    fn label(&self) -> String {
        format!("{} {}", self.name, self.version)
    }

    fn items(&self) -> Vec<ItemRef> {
        self.checked()
            .file()
            .items
            .iter()
            .filter_map(|i| match i {
                rlx::ast::Item::Derive(d) => Some(ItemRef::Derive(d.target.dotted())),
                rlx::ast::Item::Check(c) => Some(ItemRef::Check(c.name.name.clone())),
                rlx::ast::Item::Report(r) => Some(ItemRef::Report(r.name.name.clone())),
                _ => None,
            })
            .collect()
    }
}

/// One profile a contract uses (phase 91 E6): a severity scale and its
/// criticality map.
#[derive(Debug, Clone)]
pub struct LoadedProfile {
    pub profile: Profile,
    /// sha256 of the file's bytes as shipped (a profile is YAML data; it has
    /// no canonical form to hash).
    pub sha256: String,
    /// The file; `None` for a reference profile bundled with the checker.
    pub path: Option<PathBuf>,
}

impl LoadedProfile {
    fn label(&self) -> String {
        format!("{} {}", self.profile.name, self.profile.version)
    }
}

/// The severity scale a scope's hazards draw from, with the criticality
/// each level derives.
#[derive(Debug, Clone)]
pub struct SeverityScale {
    pub levels: Vec<String>,
    pub criticality: Vec<Option<ros_launch_manifest_sched::Criticality>>,
    /// Where the scale came from, for messages: `profile iec61508 1.0.0`,
    /// `severity_levels`, or `the default (ISO 26262)`.
    pub source: String,
    /// True when the contract wrote neither a profile nor `severity_levels`.
    pub default: bool,
}

/// The scale scope `scope_id`'s hazards draw from: the profile it uses,
/// else its `severity_levels:` (split into thirds), else ISO 26262's.
pub fn severity_scale(index: &ManifestIndex, scope_id: usize) -> SeverityScale {
    use ros_launch_manifest_sched::Criticality;
    let crit = |c: Option<&str>| match c? {
        "low" => Some(Criticality::Low),
        "medium" => Some(Criticality::Medium),
        _ => Some(Criticality::High),
    };
    if let Some(p) = index.profiles.get(&scope_id) {
        return SeverityScale {
            levels: p.profile.scale.clone(),
            criticality: p.profile.criticality.iter().map(|c| crit(*c)).collect(),
            source: format!("profile {}", p.label()),
            default: false,
        };
    }
    let (levels, source, default) = match index
        .manifests
        .get(&scope_id)
        .filter(|m| !m.manifest.severity_levels.is_empty())
    {
        Some(m) => (
            m.manifest.severity_levels.clone(),
            "severity_levels".to_string(),
            false,
        ),
        None => (
            ros_launch_manifest_types::severity::DEFAULT_SEVERITY_LEVELS
                .iter()
                .map(|s| s.to_string())
                .collect(),
            "the default (ISO 26262)".to_string(),
            true,
        ),
    };
    let n = levels.len();
    SeverityScale {
        criticality: (0..n).map(|r| Criticality::from_severity(r, n)).collect(),
        levels,
        source,
        default,
    }
}

fn diag(rule: &str, severity: Severity, path: String, message: String) -> Diagnostic {
    Diagnostic {
        rule_id: rule.to_string(),
        severity,
        message,
        path,
        span: None,
    }
}

/// Where a contract's extensions and profiles are looked for: the `rlm/`
/// directory beside it, then `share/<pkg>/rlm/` of each package that
/// registers the ament resource `rlm_extensions`, in `AMENT_PREFIX_PATH`
/// order (phase 91 E7). A package registers with
/// `ament_index_register_resource(rlm_extensions)` and installs its files
/// to `share/${PROJECT_NAME}/rlm`; one that does not register is not
/// searched, so the checker never walks every package's `share/`.
fn search_dirs(contract: &Path) -> Vec<PathBuf> {
    let mut dirs = Vec::new();
    if let Some(parent) = contract.parent() {
        dirs.push(parent.join("rlm"));
    }
    if let Some(prefixes) = std::env::var_os("AMENT_PREFIX_PATH") {
        for prefix in std::env::split_paths(&prefixes) {
            let share = prefix.join("share");
            let index = share.join("ament_index/resource_index/rlm_extensions");
            let Ok(entries) = std::fs::read_dir(&index) else {
                continue;
            };
            let mut pkgs: Vec<String> = entries
                .filter_map(|e| e.ok())
                .filter_map(|e| e.file_name().into_string().ok())
                .filter(|n| !n.starts_with('.'))
                .collect();
            pkgs.sort();
            dirs.extend(pkgs.iter().map(|p| share.join(p).join("rlm")));
        }
    }
    dirs
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// A profile file, parsed and its `functions:` type-checked.
fn load_profile(source: &str, path: Option<&Path>, shown: &str) -> Result<LoadedProfile, String> {
    let profile = profile::parse_profile(source)?;
    if let Some(src) = profile.functions_source()
        && let Err(errs) = rlx::check_source(src.as_bytes())
    {
        let lines: Vec<String> = errs
            .iter()
            .map(|d| d.render(shown, src.as_bytes()).trim_end().to_string())
            .collect();
        return Err(format!("`functions:` does not load: {}", lines.join("; ")));
    }
    Ok(LoadedProfile {
        profile,
        sha256: sha256_hex(source.as_bytes()),
        path: path.map(Path::to_path_buf),
    })
}

/// The reference profiles bundled with the checker (rlm
/// `types::profile::REFERENCE`).
fn bundled_profiles() -> Vec<LoadedProfile> {
    profile::REFERENCE
        .iter()
        .map(|(file, src)| load_profile(src, None, file).expect("a reference profile loads"))
        .collect()
}

fn discover(dirs: &[PathBuf], diags: &mut Vec<Diagnostic>) -> (Vec<LoadedExt>, Vec<LoadedProfile>) {
    let mut exts = Vec::new();
    let mut profiles = Vec::new();
    for dir in dirs {
        let Ok(entries) = std::fs::read_dir(dir) else {
            continue;
        };
        let mut files: Vec<PathBuf> = entries.filter_map(|e| e.ok()).map(|e| e.path()).collect();
        files.sort();
        for path in files {
            let is_profile = path
                .file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.ends_with(profile::FILE_SUFFIX));
            let is_rlx = path.extension().is_some_and(|x| x == "rlx");
            if !is_profile && !is_rlx {
                continue;
            }
            let Ok(source) = std::fs::read_to_string(&path) else {
                continue;
            };
            let shown = path.display().to_string();
            if is_profile {
                match load_profile(&source, Some(&path), &shown) {
                    Ok(p) => profiles.push(p),
                    Err(e) => diags.push(diag(
                        "extension-invalid",
                        Severity::Error,
                        shown.clone(),
                        format!("profile {shown} does not load: {e}"),
                    )),
                }
                continue;
            }
            match rlx::check_source(source.as_bytes()) {
                Ok(checked) => {
                    let id = rlx::eval::identity(&checked);
                    exts.push(LoadedExt {
                        name: id.name,
                        version: id.version,
                        sha256: id.sha256,
                        path: path.clone(),
                        source: source.clone(),
                    });
                }
                Err(errs) => {
                    let lines: Vec<String> = errs
                        .iter()
                        .map(|d| d.render(&shown, source.as_bytes()).trim_end().to_string())
                        .collect();
                    diags.push(diag(
                        "extension-invalid",
                        Severity::Error,
                        shown.clone(),
                        format!("extension {shown} does not load: {}", lines.join("; ")),
                    ));
                }
            }
        }
    }
    (exts, profiles)
}

fn version_key(v: &str) -> (u64, u64, u64) {
    let p: Vec<u64> = v.split('.').map(|x| x.parse().unwrap_or(0)).collect();
    (
        p.first().copied().unwrap_or(0),
        p.get(1).copied().unwrap_or(0),
        p.get(2).copied().unwrap_or(0),
    )
}

/// The extensions of scope `scope_id`, empty when its contract uses none.
fn exts_of(index: &ManifestIndex, scope_id: usize) -> &[LoadedExt] {
    index
        .extensions
        .get(&scope_id)
        .map(Vec::as_slice)
        .unwrap_or(&[])
}

/// The extensions among `exts` that declare `path` (`hazard.max_altitude`).
fn declaring<'a>(exts: &'a [LoadedExt], path: &str) -> Vec<&'a LoadedExt> {
    exts.iter()
        .filter(|e| {
            e.checked()
                .env
                .key(path)
                .is_some_and(|k| k.kind == KeyKind::Key)
        })
        .collect()
}

/// Something a `use:` entry may resolve to.
#[derive(Clone, Copy)]
enum Candidate<'a> {
    Ext(&'a LoadedExt),
    Profile(&'a LoadedProfile),
}

impl Candidate<'_> {
    fn name(&self) -> &str {
        match self {
            Candidate::Ext(e) => &e.name,
            Candidate::Profile(p) => &p.profile.name,
        }
    }
    fn version(&self) -> &str {
        match self {
            Candidate::Ext(e) => &e.version,
            Candidate::Profile(p) => &p.profile.version,
        }
    }
    fn sha256(&self) -> &str {
        match self {
            Candidate::Ext(e) => &e.sha256,
            Candidate::Profile(p) => &p.sha256,
        }
    }
    /// Order among equal versions: a file on disk before a bundled profile.
    fn on_disk(&self) -> bool {
        !matches!(self, Candidate::Profile(p) if p.path.is_none())
    }
}

/// Step 1: resolve each contract's `use:` (honouring `rlm.lock`), load,
/// validate and derive the hazard-level bounds.
pub fn prepare(index: &mut ManifestIndex) {
    use super::rlm_lock::{self, LockMode};
    let mode = index.lock_mode;
    let (drift_rule, drift_severity) = match mode {
        LockMode::Locked => ("lock-mismatch", Severity::Error),
        _ => ("lock-drift", Severity::Warning),
    };
    let bundled = bundled_profiles();
    let mut diags = Vec::new();
    let mut scopes: Vec<usize> = index.manifests.keys().copied().collect();
    scopes.sort_unstable();
    for scope in scopes {
        let m = &index.manifests[&scope];
        if m.manifest.uses.is_empty() {
            continue;
        }
        let contract = m.contract_path.clone();
        let lock_path = rlm_lock::path_for(&contract);
        let dirs = search_dirs(&contract);
        let (exts, mut profiles) = discover(&dirs, &mut diags);
        profiles.extend(bundled.iter().cloned());
        let candidates: Vec<Candidate> = exts
            .iter()
            .map(Candidate::Ext)
            .chain(profiles.iter().map(Candidate::Profile))
            .collect();
        let lock = if mode == LockMode::Update {
            None
        } else {
            match rlm_lock::read(&contract) {
                Ok(l) => l,
                Err(e) => {
                    diags.push(diag(
                        "lock-invalid",
                        Severity::Error,
                        lock_path.display().to_string(),
                        e,
                    ));
                    None
                }
            }
        };
        if mode == LockMode::Locked && lock.is_none() {
            diags.push(diag(
                "lock-missing",
                Severity::Error,
                "use".to_string(),
                format!(
                    "`check --locked`: {} uses extensions and has no {} beside it -- write one \
                     with `check --update-lock`",
                    contract.display(),
                    lock_path.display()
                ),
            ));
        }
        let mut chosen_exts = Vec::new();
        let mut chosen_profiles: Vec<LoadedProfile> = Vec::new();
        for u in &m.manifest.uses {
            let accepted: Vec<Candidate> = candidates
                .iter()
                .copied()
                .filter(|c| c.name() == u.name && u.req.matches(c.version()))
                .collect();
            let highest = accepted
                .iter()
                .copied()
                .max_by_key(|c| (version_key(c.version()), c.on_disk()));
            let Some(highest) = highest else {
                let seen: Vec<String> = candidates
                    .iter()
                    .filter(|c| c.name() == u.name)
                    .map(|c| c.version().to_string())
                    .collect();
                diags.push(diag(
                    "extension-not-found",
                    Severity::Error,
                    format!("use.{}", u.name),
                    format!(
                        "`use: {{ {}: \"{}\" }}`: no such extension {} -- searched {}",
                        u.name,
                        u.req,
                        if seen.is_empty() {
                            "was found".to_string()
                        } else {
                            format!("accepts the versions found ({})", seen.join(", "))
                        },
                        if dirs.is_empty() {
                            "nothing".to_string()
                        } else {
                            dirs.iter()
                                .map(|d| d.display().to_string())
                                .collect::<Vec<_>>()
                                .join(", ")
                        },
                    ),
                ));
                continue;
            };
            let mut pick = highest;
            if let Some(lock) = &lock {
                let mut drift = |why: String| {
                    diags.push(diag(
                        drift_rule,
                        drift_severity,
                        format!("use.{}", u.name),
                        format!("{}: {why}", lock_path.display()),
                    ))
                };
                match lock.resolved.get(&u.name) {
                    None => drift(format!(
                        "`{}` is not locked; resolved {} {} -- run `check --update-lock`",
                        u.name,
                        highest.name(),
                        highest.version()
                    )),
                    Some(e) => {
                        let same: Vec<Candidate> = accepted
                            .iter()
                            .copied()
                            .filter(|c| c.version() == e.version)
                            .collect();
                        if let Some(c) = same.iter().find(|c| c.sha256() == e.sha256) {
                            pick = *c;
                        } else if let Some(c) = same.first() {
                            pick = *c;
                            drift(format!(
                                "{} {} changed since it was locked: sha256 {} locked, {} now \
                                 -- review the change, then `check --update-lock`",
                                u.name,
                                e.version,
                                &e.sha256[..e.sha256.len().min(12)],
                                &c.sha256()[..12]
                            ));
                        } else {
                            drift(format!(
                                "{} {} is locked but {}; resolved {} -- run `check \
                                 --update-lock`",
                                u.name,
                                e.version,
                                if candidates
                                    .iter()
                                    .any(|c| c.name() == u.name && c.version() == e.version)
                                {
                                    format!("`{}` no longer accepts it", u.req)
                                } else {
                                    "not installed".to_string()
                                },
                                highest.version()
                            ));
                        }
                    }
                }
            }
            match pick {
                Candidate::Ext(e) => chosen_exts.push(e.clone()),
                Candidate::Profile(p) => chosen_profiles.push(p.clone()),
            }
        }
        if let [first, rest @ ..] = chosen_profiles.as_slice() {
            if !rest.is_empty() {
                diags.push(diag(
                    "profile-multiple",
                    Severity::Error,
                    "use".to_string(),
                    format!(
                        "a contract uses one profile; this one uses {} -- the scale is {}'s",
                        chosen_profiles
                            .iter()
                            .map(LoadedProfile::label)
                            .collect::<Vec<_>>()
                            .join(" and "),
                        first.label()
                    ),
                ));
            }
            if !m.manifest.severity_levels.is_empty() {
                diags.push(diag(
                    "severity-scale-conflict",
                    Severity::Error,
                    "severity_levels".to_string(),
                    format!(
                        "`severity_levels:` beside the profile {}: the profile states the scale \
                         and its criticality map -- delete `severity_levels:`",
                        first.label()
                    ),
                ));
            }
            let grammar = ros_launch_manifest_types::GRAMMAR_VERSION;
            if let Some(req) = &first.profile.requires_rlm
                && !req.matches(grammar)
            {
                diags.push(diag(
                    "extension-invalid",
                    Severity::Error,
                    format!("use.{}", first.profile.name),
                    format!(
                        "profile {} requires rlm {req}; this checker reads rlm {grammar}",
                        first.label()
                    ),
                ));
            }
            index.profiles.insert(scope, first.clone());
        }
        index.extensions.insert(scope, chosen_exts);
    }
    validate_keys(index, &mut diags);
    derive_hazard_bounds(index, &mut diags);
    index.merge_diagnostics.extend(diags);
}

/// `check --update-lock`: write each contract directory's `rlm.lock` from
/// what [`prepare`] resolved. Two contracts in one directory that resolved
/// one name differently cannot share a lock, and nothing is written.
pub fn write_locks(index: &ManifestIndex) -> Result<Vec<PathBuf>, String> {
    use super::rlm_lock::{self, Entry, Lock};
    let mut locks: BTreeMap<PathBuf, Lock> = BTreeMap::new();
    let mut scopes: Vec<_> = index.manifests.values().collect();
    scopes.sort_by_key(|m| m.scope_id);
    for m in scopes.into_iter().filter(|m| !m.manifest.uses.is_empty()) {
        let path = rlm_lock::path_for(&m.contract_path);
        let lock = locks.entry(path.clone()).or_insert_with(|| Lock {
            version: 1,
            ..Lock::default()
        });
        let mut entries: Vec<(String, Entry)> = exts_of(index, m.scope_id)
            .iter()
            .map(|e| {
                (
                    e.name.clone(),
                    Entry {
                        version: e.version.clone(),
                        sha256: e.sha256.clone(),
                        kind: "extension".into(),
                    },
                )
            })
            .collect();
        if let Some(p) = index.profiles.get(&m.scope_id) {
            entries.push((
                p.profile.name.clone(),
                Entry {
                    version: p.profile.version.clone(),
                    sha256: p.sha256.clone(),
                    kind: "profile".into(),
                },
            ));
        }
        for (name, e) in entries {
            if let Some(prev) = lock.resolved.get(&name)
                && *prev != e
            {
                return Err(format!(
                    "{}: two contracts in this directory resolve {name} differently ({} and \
                     {}); one lock cannot hold both",
                    path.display(),
                    prev.version,
                    e.version
                ));
            }
            lock.resolved.insert(name, e);
        }
    }
    for (path, lock) in &locks {
        std::fs::write(path, rlm_lock::render(lock))
            .map_err(|e| format!("{}: {e}", path.display()))?;
    }
    Ok(locks.into_keys().collect())
}

/// `check --explain`'s extensions block (phase 91 E7): per contract, each
/// extension and profile it resolved, with the version, the full sha256 and
/// where it was found.
pub fn render_resolution(index: &ManifestIndex) -> String {
    let mut out = String::new();
    let mut scopes: Vec<_> = index.manifests.values().collect();
    scopes.sort_by_key(|m| m.scope_id);
    for m in scopes.into_iter().filter(|m| !m.manifest.uses.is_empty()) {
        if out.is_empty() {
            out.push_str("\n-- Extensions (--explain) --\n");
        }
        let lock = super::rlm_lock::path_for(&m.contract_path);
        out.push_str(&format!(
            "{}  (lock: {})\n",
            m.contract_path.display(),
            if lock.is_file() {
                lock.display().to_string()
            } else {
                "none".to_string()
            }
        ));
        for e in exts_of(index, m.scope_id) {
            out.push_str(&format!(
                "    {} {}  extension  sha256 {}  {}\n",
                e.name,
                e.version,
                e.sha256,
                e.path.display()
            ));
        }
        if let Some(p) = index.profiles.get(&m.scope_id) {
            out.push_str(&format!(
                "    {}  profile  sha256 {}  {}\n",
                p.label(),
                p.sha256,
                p.path
                    .as_ref()
                    .map_or("bundled with the checker".to_string(), |p| p
                        .display()
                        .to_string())
            ));
        }
    }
    out
}

/// Every extension key the contracts write must be declared by exactly one
/// used extension (a bare key) or by the extension it names (a qualified
/// `<extension>/<key>`, phase 91 E7), and its value must be one that
/// declaration accepts.
fn validate_keys(index: &ManifestIndex, diags: &mut Vec<Diagnostic>) {
    let check =
        |exts: &[LoadedExt], ctx: &str, k: &ExtKey, at: String, diags: &mut Vec<Diagnostic>| {
            let (qualifier, key) = split_key(&k.key);
            let path = format!("{ctx}.{key}");
            let used = || {
                if exts.is_empty() {
                    "none loaded".to_string()
                } else {
                    exts.iter()
                        .map(LoadedExt::label)
                        .collect::<Vec<_>>()
                        .join(", ")
                }
            };
            let candidates: Vec<&LoadedExt> = match qualifier {
                Some(q) => {
                    let Some(e) = exts.iter().find(|e| e.name == q) else {
                        diags.push(diag(
                            "extension-key-unknown",
                            Severity::Error,
                            at,
                            format!(
                                "`{}` qualifies `{key}` with `{q}`, which is not an extension \
                                 this contract uses ({})",
                                k.key,
                                used()
                            ),
                        ));
                        return;
                    };
                    declaring(std::slice::from_ref(e), &path)
                }
                None => declaring(exts, &path),
            };
            match candidates.as_slice() {
                [] => diags.push(diag(
                    "extension-key-unknown",
                    Severity::Error,
                    at,
                    format!(
                        "`{}` is not a key of `{ctx}` in {} -- an unknown key would otherwise \
                         be dropped in silence",
                        k.key,
                        match qualifier {
                            Some(q) => format!("extension {q}"),
                            None => format!("any extension this contract uses ({})", used()),
                        }
                    ),
                )),
                [one] => {
                    let checked = one.checked();
                    let ty = checked.env.key(&path).expect("declared").ty.clone();
                    if let Err(e) = parse_static(&checked.env, &ty, &k.value) {
                        diags.push(diag(
                            "extension-value",
                            Severity::Error,
                            at,
                            format!("`{}` ({}): {e}", k.key, one.label()),
                        ));
                    }
                }
                many => diags.push(diag(
                    "extension-key-ambiguous",
                    Severity::Error,
                    at,
                    format!(
                        "`{key}` is declared in `{ctx}` by {} -- a bare key names one extension's; \
                         write the qualified spelling: {}",
                        many.iter()
                            .map(|e| e.label())
                            .collect::<Vec<_>>()
                            .join(" and "),
                        many.iter()
                            .map(|e| format!("`{}/{key}:`", e.name))
                            .collect::<Vec<_>>()
                            .join(" or ")
                    ),
                )),
            }
        };
    for h in &index.hazards {
        let exts = exts_of(index, h.scope_id);
        for k in &h.decl.ext {
            check(
                exts,
                "hazard",
                k,
                format!("hazards.{}.{}", h.name, k.key),
                diags,
            );
        }
    }
    let mut scopes: Vec<_> = index.manifests.values().collect();
    scopes.sort_by_key(|m| m.scope_id);
    for m in scopes {
        let exts = exts_of(index, m.scope_id);
        for (n, nd) in &m.manifest.nodes {
            for (p, pd) in &nd.paths {
                let Some(ss) = &pd.safe_state else { continue };
                for k in &ss.settle_ext {
                    check(
                        exts,
                        "settle",
                        k,
                        format!("nodes.{n}.paths.{p}.safe_state.settle.{}", k.key),
                        diags,
                    );
                }
            }
        }
    }
}

/// A value's shape, checked without looking up parameters: a quantity in
/// the right dimension, a parameter NAME for a `param` key, a table.
fn parse_static(env: &rlx::typeck::Env, ty: &KeyTy, v: &ExtValue) -> Result<(), String> {
    match (ty, v) {
        (KeyTy::Table { .. }, ExtValue::Map(pairs)) => {
            let refs: Vec<(&str, &str)> = pairs
                .iter()
                .map(|(a, b)| (a.as_str(), b.as_str()))
                .collect();
            env.parse_table(&refs, ty)
                .map(|_| ())
                .map_err(|d| d.message)
        }
        (_, ExtValue::Map(_)) => Err("a map was written, but this key is not a table".into()),
        (_, ExtValue::Scalar(text)) => env.parse_value(text, ty).map(|_| ()).map_err(|d| d.message),
    }
}

/// Where a `param` key's parameter lives.
enum ParamHost<'a> {
    /// The node publishing the hazard's guard.
    Guard(&'a ResolvedHazard),
    /// The node whose path states the safe state.
    Node(&'a str),
}

fn param_number(index: &ManifestIndex, host: &ParamHost, name: &str) -> Result<f64, String> {
    let nodes: Vec<String> = match host {
        ParamHost::Node(fqn) => vec![fqn.to_string()],
        ParamHost::Guard(h) => {
            let mut v: Vec<String> = h
                .guards
                .iter()
                .flat_map(|g| g.members.iter())
                .filter_map(|t| index.topics.get(t))
                .flat_map(|t| t.publishers.iter())
                .filter_map(|ep| ep.rsplit_once('/').map(|(n, _)| n.to_string()))
                .collect();
            v.sort();
            v.dedup();
            v
        }
    };
    let found: Vec<(String, f64)> = nodes
        .iter()
        .filter_map(|n| {
            index
                .launch_params
                .get(n)
                .and_then(|p| p.get(name))
                .and_then(|lp| lp.number)
                .map(|x| (n.clone(), x))
        })
        .collect();
    match found.as_slice() {
        [(_, x)] => Ok(*x),
        [] => Err(format!(
            "parameter `{name}` has no numeric launch value on {}",
            if nodes.is_empty() {
                "any node (the hazard's guard has no publisher)".to_string()
            } else {
                nodes.join(", ")
            }
        )),
        many => Err(format!(
            "parameter `{name}` is set on several guard publishers ({}); a hazard's \
             parameter must name one",
            many.iter()
                .map(|(n, _)| n.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        )),
    }
}

/// A written key split into its qualifier and key (phase 91 E7):
/// `vehicle.braking/decel` is `(Some("vehicle.braking"), "decel")`, a bare
/// `decel` is `(None, "decel")`.
fn split_key(written: &str) -> (Option<&str>, &str) {
    match written.split_once('/') {
        Some((ext, key)) => (Some(ext), key),
        None => (None, written),
    }
}

/// Whether the written key `k` is extension `ext`'s `key`: bare, or
/// qualified with `ext`'s name.
fn names(k: &ExtKey, ext: &str, key: &str) -> bool {
    match split_key(&k.key) {
        (None, bare) => bare == key,
        (Some(q), bare) => q == ext && bare == key,
    }
}

/// The inputs an item needs, from the hazard's keys, the safe state's keys
/// and the core values. A key the contract does not write is left out: the
/// evaluator uses its default or names it as missing.
#[allow(clippy::too_many_arguments)]
fn gather(
    index: &ManifestIndex,
    ext: &str,
    checked: &rlx::Checked,
    needs: &[Need],
    h: &ResolvedHazard,
    settle: Option<(&str, &SafeState)>,
    reaction_total_ms: Option<f64>,
) -> Result<(Inputs, Vec<(String, String)>), String> {
    let mut inputs = Inputs::new();
    let mut written: Vec<(String, String)> = Vec::new();
    for need in needs {
        let (ctx, key) = need
            .path
            .split_once('.')
            .unwrap_or(("", need.path.as_str()));
        let core = |text: String| -> Result<Value, String> {
            let ContractValue::Quantity(q) = checked
                .env
                .parse_value(&text, &need.ty)
                .map_err(|d| d.message)?
            else {
                unreachable!("a core input is a quantity")
            };
            Ok(Value::from_quantity(q, Some(&text)))
        };
        use rlx::value::ContractValue;
        if need.path == "reaction_total" {
            if let Some(ms) = reaction_total_ms {
                let text = format!("{ms:.2}ms");
                inputs.insert(need.path.clone(), core(text.clone())?);
                written.push((need.path.clone(), text));
            }
            continue;
        }
        if need.path == "settle.response" && need.kind != KeyKind::Key {
            if let Some(ms) = settle
                .and_then(|(_, ss)| ss.settle_response.as_ref())
                .and_then(|b| b.ms())
            {
                let text = format!("{ms}ms");
                inputs.insert(need.path.clone(), core(text.clone())?);
                written.push((need.path.clone(), text));
            }
            continue;
        }
        let (keys, host): (&[ExtKey], ParamHost) = match (ctx, settle) {
            ("hazard", _) => (&h.decl.ext, ParamHost::Guard(h)),
            ("settle", Some((node, ss))) => (&ss.settle_ext, ParamHost::Node(node)),
            _ => continue,
        };
        let Some(k) = keys.iter().find(|k| names(k, ext, key)) else {
            continue;
        };
        let value = match (&need.ty, &k.value) {
            (KeyTy::Table { .. }, ExtValue::Map(pairs)) => {
                let refs: Vec<(&str, &str)> = pairs
                    .iter()
                    .map(|(a, b)| (a.as_str(), b.as_str()))
                    .collect();
                let rows = checked
                    .env
                    .parse_table(&refs, &need.ty)
                    .map_err(|d| d.message)?;
                written.push((need.path.clone(), format!("{} entries", rows.len())));
                Value::table(&rows)
            }
            (KeyTy::Param { .. }, ExtValue::Scalar(name)) => {
                let raw = param_number(index, &host, name)?;
                written.push((need.path.clone(), format!("{name} = {raw}")));
                rlx::eval::param_value(raw, &need.ty).expect("a param key")
            }
            (_, ExtValue::Scalar(text)) => {
                let ContractValue::Quantity(q) = checked
                    .env
                    .parse_value(text, &need.ty)
                    .map_err(|d| d.message)?
                else {
                    return Err(format!("`{key}` names a parameter where a value belongs"));
                };
                written.push((need.path.clone(), text.clone()));
                Value::from_quantity(q, Some(text))
            }
            (_, ExtValue::Map(_)) => return Err(format!("`{key}` is a table, not a value")),
        };
        inputs.insert(need.path.clone(), value);
    }
    Ok((inputs, written))
}

/// Whether the hazard (and safe state) write any key the item reads: an
/// item none of whose keys the contract writes is not about this hazard.
fn writes_any(ext: &str, needs: &[Need], h: &ResolvedHazard, settle: Option<&SafeState>) -> bool {
    needs.iter().any(|n| match n.path.split_once('.') {
        Some(("hazard", k)) => h.decl.ext.iter().any(|e| names(e, ext, k)),
        Some(("settle", k)) => {
            settle.is_some_and(|ss| ss.settle_ext.iter().any(|e| names(e, ext, k)))
        }
        _ => false,
    })
}

fn trace_lines(t: &rlx::eval::Trace) -> Vec<String> {
    t.lines
        .iter()
        .map(|l| format!("{}{}", "  ".repeat(l.depth), l.text))
        .collect()
}

fn derivation(
    ext: &LoadedExt,
    written: Vec<(String, String)>,
    trace: &rlx::eval::Trace,
    worst_case: Option<&str>,
) -> Derivation {
    let mut trace = trace_lines(trace);
    trace.extend(worst_case.map(str::to_string));
    Derivation {
        by: ext.name.clone(),
        version: ext.version.clone(),
        sha256: Some(ext.sha256.clone()),
        inputs: written,
        trace,
    }
}

const WORST_CASE_REFUTED: &str = "worst-case-refuted";
const WORST_CASE_UNKNOWN: &str = "worst-case-unknown";

/// What the worst-case proofs say about one item at one contract point.
struct WorstCase {
    /// `worst case: proved increasing in a, b; decreasing in c`, over the
    /// claims that were proved; `None` when none was.
    line: Option<String>,
    /// `(rule, severity, message)` for each claim that was not.
    findings: Vec<(&'static str, Severity, String)>,
}

impl WorstCase {
    fn suffix(&self) -> String {
        self.line
            .as_ref()
            .map_or(String::new(), |l| format!("\n{l}"))
    }

    fn report(&self, at: &str, diags: &mut Vec<Diagnostic>) {
        for (rule, severity, message) in &self.findings {
            diags.push(diag(rule, *severity, at.to_string(), message.clone()));
        }
    }
}

/// The `worst_case` claims of `item`, in order: `(direction, key)`.
fn stated_claims(checked: &rlx::Checked, item: &ItemRef) -> Vec<(rlx::ast::Direction, String)> {
    checked
        .file()
        .items
        .iter()
        .find_map(|i| match (i, item) {
            (rlx::ast::Item::Derive(d), ItemRef::Derive(t)) if &d.target.dotted() == t => {
                Some(&d.body)
            }
            (rlx::ast::Item::Check(c), ItemRef::Check(n)) if &c.name.name == n => Some(&c.body),
            _ => None,
        })
        .map(|b| {
            b.worst_case
                .iter()
                .flat_map(|g| g.vars.iter().map(|v| (g.direction, v.dotted())))
                .collect()
        })
        .unwrap_or_default()
}

/// `increasing in a, b; decreasing in c`: claims grouped as written.
fn statement(claims: &[(rlx::ast::Direction, String)]) -> String {
    let mut groups: Vec<(rlx::ast::Direction, Vec<&str>)> = Vec::new();
    for (d, v) in claims {
        match groups.last_mut() {
            Some((g, vars)) if g == d => vars.push(v),
            _ => groups.push((*d, vec![v])),
        }
    }
    groups
        .iter()
        .map(|(d, vars)| {
            let word = match d {
                rlx::ast::Direction::Increasing => "increasing",
                rlx::ast::Direction::Decreasing => "decreasing",
            };
            format!("{word} in {}", vars.join(", "))
        })
        .collect::<Vec<_>>()
        .join("; ")
}

/// Prove `item`'s worst-case claims at the contract's point `inputs`: its
/// tables' keys as written, and its values the point a suggested domain
/// must admit. The result is a pure function of the extension's text and
/// the point, so it is memoized for the process: the settle lookup asks once
/// per (hazard, rung).
#[cfg(feature = "smt")]
fn worst_case(
    ext: &LoadedExt,
    checked: &rlx::Checked,
    item: &ItemRef,
    inputs: &Inputs,
) -> WorstCase {
    use rlx::prove::{Options, Verdict};
    use std::{
        collections::HashMap,
        sync::{Mutex, OnceLock},
    };
    static MEMO: OnceLock<Mutex<HashMap<String, Vec<rlx::prove::Claim>>>> = OnceLock::new();
    if stated_claims(checked, item).is_empty() {
        return WorstCase {
            line: None,
            findings: Vec::new(),
        };
    }
    let opts = Options {
        table_keys: inputs
            .iter()
            .filter_map(|(k, v)| match v {
                Value::Table(rows) => Some((k.clone(), rows.iter().map(|r| r.0).collect())),
                Value::Quantity { .. } => None,
            })
            .collect(),
        point: inputs.clone(),
        ..Options::default()
    };
    let key = format!("{}|{item:?}|{inputs:?}", ext.sha256);
    let memo = MEMO.get_or_init(|| Mutex::new(HashMap::new()));
    let cached = memo.lock().expect("not poisoned").get(&key).cloned();
    let claims = cached.unwrap_or_else(|| {
        let claims = rlx::prove::prove(checked, item, &opts);
        memo.lock()
            .expect("not poisoned")
            .insert(key, claims.clone());
        claims
    });
    let proved: Vec<(rlx::ast::Direction, String)> = claims
        .iter()
        .filter(|c| c.verdict == Verdict::Proved)
        .map(|c| (c.direction, c.var.clone()))
        .collect();
    let tables = claims
        .iter()
        .find(|c| c.verdict == Verdict::Proved && !c.tables.is_empty())
        .map_or(String::new(), |c| format!(" (for {})", c.tables.join(", ")));
    WorstCase {
        line: (!proved.is_empty())
            .then(|| format!("worst case: proved {}{tables}", statement(&proved))),
        findings: claims
            .iter()
            .filter_map(|c| match c.verdict {
                Verdict::Proved => None,
                Verdict::Refuted(_) => Some((
                    WORST_CASE_REFUTED,
                    Severity::Error,
                    format!("{}: {}", ext.label(), c.report()),
                )),
                Verdict::Unknown(_) => Some((
                    WORST_CASE_UNKNOWN,
                    Severity::Warning,
                    format!("{}: {}", ext.label(), c.report()),
                )),
            })
            .collect(),
    }
}

/// Without `smt` nothing is proved, and the build says so rather than
/// leaving the claim to read as checked.
#[cfg(not(feature = "smt"))]
fn worst_case(
    ext: &LoadedExt,
    checked: &rlx::Checked,
    item: &ItemRef,
    _inputs: &Inputs,
) -> WorstCase {
    let claims = stated_claims(checked, item);
    WorstCase {
        line: None,
        findings: if claims.is_empty() {
            Vec::new()
        } else {
            let subject = match item {
                ItemRef::Derive(t) => t.clone(),
                ItemRef::Check(n) => format!("check {n}"),
                ItemRef::Report(n) => format!("report {n}"),
            };
            vec![(
                WORST_CASE_UNKNOWN,
                Severity::Warning,
                format!(
                    "{}: {subject}: worst case {} not proved: this build has no `smt` \
                     feature, so there is no Z3 to prove it",
                    ext.label(),
                    statement(&claims)
                ),
            )]
        },
    }
}

/// `check --strict`: an undecided worst case is an error.
pub fn strict(index: &mut ManifestIndex) {
    for d in index
        .merge_diagnostics
        .iter_mut()
        .filter(|d| d.rule_id == WORST_CASE_UNKNOWN)
    {
        d.severity = Severity::Error;
    }
}

/// `hazard.detection` and `hazard.ftti`, for every hazard whose contract
/// writes the keys a used extension derives them from.
fn derive_hazard_bounds(index: &mut ManifestIndex, diags: &mut Vec<Diagnostic>) {
    let mut updates: Vec<(usize, String, &'static str, Bound)> = Vec::new();
    for h in &index.hazards {
        for ext in exts_of(index, h.scope_id) {
            let checked = ext.checked();
            for (target, key) in [("hazard.detection", "detection"), ("hazard.ftti", "ftti")] {
                let item = ItemRef::Derive(target.to_string());
                let Some(needs) = checked.env.reads(&item) else {
                    continue;
                };
                if !writes_any(&ext.name, &needs, h, None) {
                    continue;
                }
                let at = format!("hazards.{}.{key}", h.name);
                let written_already = match key {
                    "detection" => h.decl.detection.is_some(),
                    _ => h.decl.ftti.is_some(),
                };
                if written_already {
                    diags.push(diag(
                        "derive-conflict",
                        Severity::Error,
                        at,
                        format!(
                            "hazard '{}' writes `{key}`, which {} derives -- keep one",
                            h.name,
                            ext.label()
                        ),
                    ));
                    continue;
                }
                let result = gather(index, &ext.name, &checked, &needs, h, None, None).and_then(
                    |(inputs, written)| {
                        rlx::eval::eval_derive(&checked, target, &inputs)
                            .map(|ev| (ev, written, inputs))
                            .map_err(|d| {
                                d.render(&ext.path.display().to_string(), ext.source.as_bytes())
                                    .trim_end()
                                    .to_string()
                            })
                    },
                );
                match result {
                    Ok((ev, written, inputs)) => {
                        let wc = worst_case(ext, &checked, &item, &inputs);
                        wc.report(&at, diags);
                        let mut b = ros_launch_manifest_types::spelled(
                            ros_launch_manifest_types::field_table::Context::Hazard,
                            key,
                            ros_launch_manifest_types::duration::Duration::from_millis_f64(
                                ev.si * 1000.0,
                            ),
                        );
                        b.source = Some(BudgetSource {
                            kind: SourceKind::Derived,
                            measured: None,
                            derived: Some(Box::new(derivation(
                                ext,
                                written,
                                &ev.trace,
                                wc.line.as_deref(),
                            ))),
                        });
                        diags.push(diag(
                            &format!("{key}-derived"),
                            Severity::Info,
                            at,
                            format!(
                                "hazard '{}': {key} = {} derived by {}\n{}{}",
                                h.name,
                                ev.display,
                                ext.label(),
                                ev.trace.render().trim_end(),
                                wc.suffix()
                            ),
                        ));
                        updates.push((h.scope_id, h.name.clone(), key, b));
                    }
                    Err(e) => diags.push(diag(
                        "extension-eval",
                        Severity::Error,
                        at,
                        format!(
                            "hazard '{}': {} cannot derive `{key}`: {e}",
                            h.name,
                            ext.label()
                        ),
                    )),
                }
            }
        }
    }
    for (scope, name, key, b) in updates {
        for h in index
            .hazards
            .iter_mut()
            .filter(|h| h.scope_id == scope && h.name == name)
        {
            match key {
                "detection" => h.decl.detection = Some(b.clone()),
                _ => h.decl.ftti = Some(b.clone()),
            }
        }
        if let Some(m) = index.manifests.get_mut(&scope)
            && let Some(hd) = m.manifest.hazards.get_mut(&name)
        {
            match key {
                "detection" => hd.detection = Some(b),
                _ => hd.ftti = Some(b),
            }
        }
    }
}

/// What the settle lookup charges for a safe state whose settle a used
/// extension derives: `None` when no used extension derives
/// `settle.duration` from the keys it writes.
pub(crate) fn derived_settle(
    index: &ManifestIndex,
    h: &ResolvedHazard,
    scope_id: usize,
    node_fqn: &str,
    at: &str,
    ss: &SafeState,
    diags: &mut Vec<Diagnostic>,
) -> Option<super::value_rules::Settle> {
    if ss.settle_ext.is_empty() {
        return None;
    }
    for ext in exts_of(index, scope_id) {
        let checked = ext.checked();
        let Some(needs) = checked
            .env
            .reads(&ItemRef::Derive("settle.duration".to_string()))
        else {
            continue;
        };
        if !writes_any(&ext.name, &needs, h, Some(ss)) {
            continue;
        }
        let reads_response = needs.iter().any(|n| n.path == "settle.response");
        let result = gather(
            index,
            &ext.name,
            &checked,
            &needs,
            h,
            Some((node_fqn, ss)),
            None,
        )
        .and_then(|(inputs, _)| {
            rlx::eval::eval_derive(&checked, "settle.duration", &inputs)
                .map(|ev| (ev, inputs))
                .map_err(|d| {
                    d.render(&ext.path.display().to_string(), ext.source.as_bytes())
                        .trim_end()
                        .to_string()
                })
        });
        return match result {
            Ok((ev, inputs)) => {
                let item = ItemRef::Derive("settle.duration".to_string());
                let wc = worst_case(ext, &checked, &item, &inputs);
                wc.report(at, diags);
                let derived_ms = ev.si * 1000.0;
                let response_ms = ss.settle_response.as_ref().and_then(|b| b.ms());
                // The response is the settle's (phase 90 G6): added unless
                // the extension already reads it.
                let ms = if reads_response {
                    derived_ms
                } else {
                    derived_ms + response_ms.unwrap_or(0.0)
                };
                diags.push(diag(
                    "settle-derived",
                    Severity::Info,
                    at.to_string(),
                    format!(
                        "hazard '{}': settle = {ms:.2}ms derived by {} (sha256 {}){}\n{}{}",
                        h.name,
                        ext.label(),
                        &ext.sha256[..12],
                        match (reads_response, response_ms) {
                            (false, Some(r)) => format!(" + response {r:.2}ms (written)"),
                            _ => String::new(),
                        },
                        ev.trace.render().trim_end(),
                        wc.suffix()
                    ),
                ));
                Some(super::value_rules::Settle {
                    ms: Some(ms),
                    how: "derived",
                    response_ms,
                })
            }
            Err(e) => {
                diags.push(diag(
                    "extension-eval",
                    Severity::Error,
                    at.to_string(),
                    format!(
                        "hazard '{}': {} cannot derive the settle: {e}",
                        h.name,
                        ext.label()
                    ),
                ));
                Some(super::value_rules::Settle {
                    ms: None,
                    how: "none",
                    response_ms: None,
                })
            }
        };
    }
    None
}

/// Whether hazard `h`'s `deadline_by` names a check a used extension
/// declares, so [`run_checks`] evaluates it.
pub(crate) fn resolves_deadline(index: &ManifestIndex, h: &ResolvedHazard) -> Result<(), String> {
    let Some(c) = &h.decl.deadline_by else {
        return Ok(());
    };
    let Some(ext) = exts_of(index, h.scope_id)
        .iter()
        .find(|e| e.name == c.extension)
    else {
        return Err(format!(
            "`deadline_by: {c}` names `{}`, which this contract does not `use:`",
            c.extension
        ));
    };
    if ext.items().contains(&ItemRef::Check(c.check.clone())) {
        Ok(())
    } else {
        Err(format!(
            "`deadline_by: {c}`: {} declares no check `{}`",
            ext.label(),
            c.check
        ))
    }
}

/// Step 3: every check and report a used extension declares, for each
/// hazard that writes its keys (and the check a `deadline_by` names,
/// always). A failing check is an error when it is the hazard's
/// requirement (`deadline_by`), a warning otherwise.
pub fn run_checks(index: &mut ManifestIndex) {
    let mut diags = Vec::new();
    let floor: BTreeMap<&str, f64> = index
        .budgets
        .iter()
        .filter(|r| r.role == RungRole::Floor)
        .filter_map(|r| r.total_ms.map(|t| (r.hazard.as_str(), t)))
        .collect();
    for h in &index.hazards {
        for ext in exts_of(index, h.scope_id) {
            let checked = ext.checked();
            for item in ext.items() {
                let (name, is_check) = match &item {
                    ItemRef::Check(n) => (n.clone(), true),
                    ItemRef::Report(n) => (n.clone(), false),
                    ItemRef::Derive(_) => continue,
                };
                let named = h
                    .decl
                    .deadline_by
                    .as_ref()
                    .is_some_and(|c| c.extension == ext.name && c.check == name);
                let Some(needs) = checked.env.reads(&item) else {
                    continue;
                };
                if !named && !writes_any(&ext.name, &needs, h, None) {
                    continue;
                }
                let at = format!("hazards.{}", h.name);
                let rt = floor.get(h.name.as_str()).copied();
                if rt.is_none() && needs.iter().any(|n| n.path == "reaction_total") {
                    diags.push(diag(
                        "extension-eval",
                        if named {
                            Severity::Error
                        } else {
                            Severity::Warning
                        },
                        at,
                        format!(
                            "hazard '{}': {}/{name} reads reaction_total, and the hazard's \
                             reaction has no total (see the fault-reaction findings)",
                            h.name, ext.name
                        ),
                    ));
                    continue;
                }
                let gathered = gather(index, &ext.name, &checked, &needs, h, None, rt);
                let render = |d: rlx::diag::Diagnostic| {
                    d.render(&ext.path.display().to_string(), ext.source.as_bytes())
                        .trim_end()
                        .to_string()
                };
                if is_check {
                    match gathered.and_then(|(i, _)| {
                        rlx::eval::eval_check(&checked, &name, &i)
                            .map(|out| (out, i))
                            .map_err(render)
                    }) {
                        Ok((out, inputs)) => {
                            let wc = worst_case(ext, &checked, &item, &inputs);
                            wc.report(&at, &mut diags);
                            let verdict = if out.pass { "PASS" } else { "FAIL" };
                            let severity = match (out.pass, named) {
                                (true, _) => Severity::Info,
                                (false, true) => Severity::Error,
                                (false, false) => Severity::Warning,
                            };
                            diags.push(diag(
                                "extension-check",
                                severity,
                                at,
                                format!(
                                    "hazard '{}': {}/{name} {verdict} ({}{})\n{}{}",
                                    h.name,
                                    ext.name,
                                    ext.label(),
                                    if named {
                                        ", the hazard's deadline_by"
                                    } else {
                                        ""
                                    },
                                    out.trace.render().trim_end(),
                                    wc.suffix()
                                ),
                            ));
                        }
                        Err(e) => diags.push(diag(
                            "extension-eval",
                            if named {
                                Severity::Error
                            } else {
                                Severity::Warning
                            },
                            at,
                            format!(
                                "hazard '{}': {}/{name} cannot be evaluated: {e}",
                                h.name, ext.name
                            ),
                        )),
                    }
                } else {
                    match gathered.and_then(|(i, _)| {
                        rlx::eval::eval_report(&checked, &name, &i).map_err(render)
                    }) {
                        Ok(ev) => diags.push(diag(
                            "extension-report",
                            Severity::Info,
                            at,
                            format!(
                                "hazard '{}': {}/{name} = {} (report, no verdict)\n{}",
                                h.name,
                                ext.name,
                                ev.display,
                                ev.trace.render().trim_end()
                            ),
                        )),
                        Err(e) => diags.push(diag(
                            "extension-eval",
                            Severity::Warning,
                            at,
                            format!(
                                "hazard '{}': report {}/{name} cannot be evaluated: {e}",
                                h.name, ext.name
                            ),
                        )),
                    }
                }
            }
        }
    }
    index.merge_diagnostics.extend(diags);
    // The settle lookup runs per (hazard, rung), so one claim can be
    // reported from several sites with one message; keep the first.
    let mut seen = std::collections::BTreeSet::new();
    index.merge_diagnostics.retain(|d| {
        !(d.rule_id == WORST_CASE_REFUTED || d.rule_id == WORST_CASE_UNKNOWN)
            || seen.insert((d.rule_id.clone(), d.path.clone(), d.message.clone()))
    });
}
