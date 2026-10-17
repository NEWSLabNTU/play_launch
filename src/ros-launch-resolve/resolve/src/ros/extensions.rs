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
//! Finding extensions: the `rlm/` directory beside the contract, then
//! `share/<pkg>/rlm/` under each `AMENT_PREFIX_PATH` prefix; for each `use:`
//! entry the highest version its requirement accepts. A lock file and the
//! ament resource index are E7's.
//!
//! A `param` key names a node parameter. In `settle` it is a parameter of
//! the node whose path states the safe state (as a braking profile's are);
//! in `hazard` it is a parameter of the node that publishes the hazard's
//! guard, the one place a hazard is bound to a node.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::{
    Bound, BudgetSource, Derivation, ExtKey, ExtValue, SafeState, SourceKind,
};
use rlx::eval::{Inputs, Value};
use rlx::typeck::{ItemRef, KeyKind, KeyTy, Need};

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

fn diag(rule: &str, severity: Severity, path: String, message: String) -> Diagnostic {
    Diagnostic {
        rule_id: rule.to_string(),
        severity,
        message,
        path,
        span: None,
    }
}

fn search_dirs(contract: &Path) -> Vec<PathBuf> {
    let mut dirs = Vec::new();
    if let Some(parent) = contract.parent() {
        dirs.push(parent.join("rlm"));
    }
    if let Some(prefixes) = std::env::var_os("AMENT_PREFIX_PATH") {
        for prefix in std::env::split_paths(&prefixes) {
            let share = prefix.join("share");
            let Ok(pkgs) = std::fs::read_dir(&share) else {
                continue;
            };
            let mut found: Vec<PathBuf> = pkgs
                .filter_map(|e| e.ok())
                .map(|e| e.path().join("rlm"))
                .filter(|p| p.is_dir())
                .collect();
            found.sort();
            dirs.extend(found);
        }
    }
    dirs
}

fn discover(dirs: &[PathBuf], diags: &mut Vec<Diagnostic>) -> Vec<LoadedExt> {
    let mut out = Vec::new();
    for dir in dirs {
        let Ok(entries) = std::fs::read_dir(dir) else {
            continue;
        };
        let mut files: Vec<PathBuf> = entries
            .filter_map(|e| e.ok())
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "rlx"))
            .collect();
        files.sort();
        for path in files {
            let Ok(source) = std::fs::read_to_string(&path) else {
                continue;
            };
            match rlx::check_source(source.as_bytes()) {
                Ok(checked) => {
                    let id = rlx::eval::identity(&checked);
                    out.push(LoadedExt {
                        name: id.name,
                        version: id.version,
                        sha256: id.sha256,
                        path: path.clone(),
                        source: source.clone(),
                    });
                }
                Err(errs) => {
                    let shown = path.display().to_string();
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
    out
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
        .filter(|e| e.checked().env.key(path).is_some_and(|k| k.kind == KeyKind::Key))
        .collect()
}

/// Step 1: load, validate and derive the hazard-level bounds.
pub fn prepare(index: &mut ManifestIndex) {
    let mut diags = Vec::new();
    let mut scopes: Vec<usize> = index.manifests.keys().copied().collect();
    scopes.sort_unstable();
    for scope in scopes {
        let m = &index.manifests[&scope];
        if m.manifest.uses.is_empty() {
            continue;
        }
        let dirs = search_dirs(&m.contract_path);
        let available = discover(&dirs, &mut diags);
        let mut chosen = Vec::new();
        for u in &m.manifest.uses {
            let pick = available
                .iter()
                .filter(|e| e.name == u.name && u.req.matches(&e.version))
                .max_by_key(|e| version_key(&e.version));
            match pick {
                Some(e) => chosen.push(e.clone()),
                None => {
                    let seen: Vec<String> = available
                        .iter()
                        .filter(|e| e.name == u.name)
                        .map(|e| e.version.clone())
                        .collect();
                    diags.push(diag(
                        "extension-not-found",
                        Severity::Error,
                        format!("use.{}", u.name),
                        format!(
                            "`use: {{ {}: \"{}\" }}`: no such extension {} -- searched {}{}",
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
                            ""
                        ),
                    ));
                }
            }
        }
        index.extensions.insert(scope, chosen);
    }
    validate_keys(index, &mut diags);
    derive_hazard_bounds(index, &mut diags);
    index.merge_diagnostics.extend(diags);
}

/// Every extension key the contracts write must be declared by exactly one
/// used extension, and its value must be one that declaration accepts.
fn validate_keys(index: &ManifestIndex, diags: &mut Vec<Diagnostic>) {
    let check = |exts: &[LoadedExt],
                 ctx: &str,
                 k: &ExtKey,
                 at: String,
                 diags: &mut Vec<Diagnostic>| {
        let path = format!("{ctx}.{}", k.key);
        let decl = declaring(exts, &path);
        match decl.as_slice() {
            [] => diags.push(diag(
                "extension-key-unknown",
                Severity::Error,
                at,
                format!(
                    "`{}` is not a key of `{ctx}` in any extension this contract uses ({}) -- \
                     an unknown key would otherwise be dropped in silence",
                    k.key,
                    if exts.is_empty() {
                        "none loaded".to_string()
                    } else {
                        exts.iter().map(LoadedExt::label).collect::<Vec<_>>().join(", ")
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
                    "`{}` is declared by {} -- the qualified spelling that tells them apart \
                     is not implemented yet",
                    k.key,
                    many.iter().map(|e| e.label()).collect::<Vec<_>>().join(" and ")
                ),
            )),
        }
    };
    for h in &index.hazards {
        let exts = exts_of(index, h.scope_id);
        for k in &h.decl.ext {
            check(exts, "hazard", k, format!("hazards.{}.{}", h.name, k.key), diags);
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
            let refs: Vec<(&str, &str)> =
                pairs.iter().map(|(a, b)| (a.as_str(), b.as_str())).collect();
            env.parse_table(&refs, ty).map(|_| ()).map_err(|d| d.message)
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
            many.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>().join(", ")
        )),
    }
}

/// The inputs an item needs, from the hazard's keys, the safe state's keys
/// and the core values. A key the contract does not write is left out: the
/// evaluator uses its default or names it as missing.
#[allow(clippy::too_many_arguments)]
fn gather(
    index: &ManifestIndex,
    checked: &rlx::Checked,
    needs: &[Need],
    h: &ResolvedHazard,
    settle: Option<(&str, &SafeState)>,
    reaction_total_ms: Option<f64>,
) -> Result<(Inputs, Vec<(String, String)>), String> {
    let mut inputs = Inputs::new();
    let mut written: Vec<(String, String)> = Vec::new();
    for need in needs {
        let (ctx, key) = need.path.split_once('.').unwrap_or(("", need.path.as_str()));
        let core = |text: String| -> Result<Value, String> {
            let ContractValue::Quantity(q) =
                checked.env.parse_value(&text, &need.ty).map_err(|d| d.message)?
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
            if let Some(ms) = settle.and_then(|(_, ss)| ss.settle_response.as_ref()).and_then(|b| b.ms())
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
        let Some(k) = keys.iter().find(|k| k.key == key) else {
            continue;
        };
        let value = match (&need.ty, &k.value) {
            (KeyTy::Table { .. }, ExtValue::Map(pairs)) => {
                let refs: Vec<(&str, &str)> =
                    pairs.iter().map(|(a, b)| (a.as_str(), b.as_str())).collect();
                let rows = checked.env.parse_table(&refs, &need.ty).map_err(|d| d.message)?;
                written.push((need.path.clone(), format!("{} entries", rows.len())));
                Value::table(&rows)
            }
            (KeyTy::Param { .. }, ExtValue::Scalar(name)) => {
                let raw = param_number(index, &host, name)?;
                written.push((need.path.clone(), format!("{name} = {raw}")));
                rlx::eval::param_value(raw, &need.ty).expect("a param key")
            }
            (_, ExtValue::Scalar(text)) => {
                let ContractValue::Quantity(q) =
                    checked.env.parse_value(text, &need.ty).map_err(|d| d.message)?
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
fn writes_any(needs: &[Need], h: &ResolvedHazard, settle: Option<&SafeState>) -> bool {
    needs.iter().any(|n| match n.path.split_once('.') {
        Some(("hazard", k)) => h.decl.ext.iter().any(|e| e.key == k),
        Some(("settle", k)) => settle.is_some_and(|ss| ss.settle_ext.iter().any(|e| e.key == k)),
        _ => false,
    })
}

fn trace_lines(t: &rlx::eval::Trace) -> Vec<String> {
    t.lines
        .iter()
        .map(|l| format!("{}{}", "  ".repeat(l.depth), l.text))
        .collect()
}

fn derivation(ext: &LoadedExt, written: Vec<(String, String)>, trace: &rlx::eval::Trace) -> Derivation {
    Derivation {
        by: ext.name.clone(),
        version: ext.version.clone(),
        sha256: Some(ext.sha256.clone()),
        inputs: written,
        trace: trace_lines(trace),
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
                if !writes_any(&needs, h, None) {
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
                let result = gather(index, &checked, &needs, h, None, None).and_then(
                    |(inputs, written)| {
                        rlx::eval::eval_derive(&checked, target, &inputs)
                            .map(|ev| (ev, written))
                            .map_err(|d| {
                                d.render(&ext.path.display().to_string(), ext.source.as_bytes())
                                    .trim_end()
                                    .to_string()
                            })
                    },
                );
                match result {
                    Ok((ev, written)) => {
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
                            derived: Some(Box::new(derivation(ext, written, &ev.trace))),
                        });
                        diags.push(diag(
                            &format!("{key}-derived"),
                            Severity::Info,
                            at,
                            format!(
                                "hazard '{}': {key} = {} derived by {}\n{}",
                                h.name,
                                ev.display,
                                ext.label(),
                                ev.trace.render().trim_end()
                            ),
                        ));
                        updates.push((h.scope_id, h.name.clone(), key, b));
                    }
                    Err(e) => diags.push(diag(
                        "extension-eval",
                        Severity::Error,
                        at,
                        format!("hazard '{}': {} cannot derive `{key}`: {e}", h.name, ext.label()),
                    )),
                }
            }
        }
    }
    for (scope, name, key, b) in updates {
        for h in index.hazards.iter_mut().filter(|h| h.scope_id == scope && h.name == name) {
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
        let Some(needs) = checked.env.reads(&ItemRef::Derive("settle.duration".to_string())) else {
            continue;
        };
        if !writes_any(&needs, h, Some(ss)) {
            continue;
        }
        let reads_response = needs.iter().any(|n| n.path == "settle.response");
        let result = gather(index, &checked, &needs, h, Some((node_fqn, ss)), None).and_then(
            |(inputs, _)| {
                rlx::eval::eval_derive(&checked, "settle.duration", &inputs).map_err(|d| {
                    d.render(&ext.path.display().to_string(), ext.source.as_bytes())
                        .trim_end()
                        .to_string()
                })
            },
        );
        return match result {
            Ok(ev) => {
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
                        "hazard '{}': settle = {ms:.2}ms derived by {} (sha256 {}){}\n{}",
                        h.name,
                        ext.label(),
                        &ext.sha256[..12],
                        match (reads_response, response_ms) {
                            (false, Some(r)) => format!(" + response {r:.2}ms (written)"),
                            _ => String::new(),
                        },
                        ev.trace.render().trim_end()
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
                    format!("hazard '{}': {} cannot derive the settle: {e}", h.name, ext.label()),
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
    let Some(ext) = exts_of(index, h.scope_id).iter().find(|e| e.name == c.extension) else {
        return Err(format!(
            "`deadline_by: {c}` names `{}`, which this contract does not `use:`",
            c.extension
        ));
    };
    if ext.items().contains(&ItemRef::Check(c.check.clone())) {
        Ok(())
    } else {
        Err(format!("`deadline_by: {c}`: {} declares no check `{}`", ext.label(), c.check))
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
                if !named && !writes_any(&needs, h, None) {
                    continue;
                }
                let at = format!("hazards.{}", h.name);
                let rt = floor.get(h.name.as_str()).copied();
                if rt.is_none() && needs.iter().any(|n| n.path == "reaction_total") {
                    diags.push(diag(
                        "extension-eval",
                        if named { Severity::Error } else { Severity::Warning },
                        at,
                        format!(
                            "hazard '{}': {}/{name} reads reaction_total, and the hazard's \
                             reaction has no total (see the fault-reaction findings)",
                            h.name, ext.name
                        ),
                    ));
                    continue;
                }
                let gathered = gather(index, &checked, &needs, h, None, rt);
                let render = |d: rlx::diag::Diagnostic| {
                    d.render(&ext.path.display().to_string(), ext.source.as_bytes())
                        .trim_end()
                        .to_string()
                };
                if is_check {
                    match gathered
                        .and_then(|(i, _)| rlx::eval::eval_check(&checked, &name, &i).map_err(render))
                    {
                        Ok(out) => {
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
                                    "hazard '{}': {}/{name} {verdict} ({}{})\n{}",
                                    h.name,
                                    ext.name,
                                    ext.label(),
                                    if named { ", the hazard's deadline_by" } else { "" },
                                    out.trace.render().trim_end()
                                ),
                            ));
                        }
                        Err(e) => diags.push(diag(
                            "extension-eval",
                            if named { Severity::Error } else { Severity::Warning },
                            at,
                            format!("hazard '{}': {}/{name} cannot be evaluated: {e}", h.name, ext.name),
                        )),
                    }
                } else {
                    match gathered
                        .and_then(|(i, _)| rlx::eval::eval_report(&checked, &name, &i).map_err(render))
                    {
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
                            format!("hazard '{}': report {}/{name} cannot be evaluated: {e}", h.name, ext.name),
                        )),
                    }
                }
            }
        }
    }
    index.merge_diagnostics.extend(diags);
}
