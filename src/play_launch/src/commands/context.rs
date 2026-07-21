//! Context extraction command — show per-node or per-launch-file context from
//! a SystemModel (`system_model.yaml`). Phase 49: reads the model, not the
//! retired `record.json`. Scopes are the launch-file include tree (Phase 48:
//! `<group>` scopes are folded into their file; a node's namespace is a
//! property of the node — its FQN — not of the scope).

use crate::cli::options::ContextArgs;
use eyre::{Result, WrapErr, bail};
use ros_launch_manifest_model::{NodeInstance, ParamValue, ScopeInfo, SystemModel};
use std::collections::BTreeMap;
use std::path::Path;

pub fn handle_context(args: &ContextArgs) -> Result<()> {
    let path = Path::new(&args.model);
    let yaml = std::fs::read_to_string(path)
        .wrap_err_with(|| format!("reading SystemModel {}", path.display()))?;
    let model = SystemModel::from_yaml_str(&yaml)
        .map_err(|e| eyre::eyre!("loading SystemModel {}: {e}", path.display()))?;

    if args.tree {
        print_tree(&model);
    } else if let Some(fqn) = &args.node {
        show_node_context(&model, fqn)?;
    } else if let Some(launch_args) = &args.launch {
        show_launch_context(&model, &launch_args[0], &launch_args[1], args.all)?;
    } else {
        bail!("Specify --tree, --node <FQN>, or --launch <PKG> <FILE>");
    }

    Ok(())
}

/// The namespace of a node, derived from its FQN (the part before the final
/// `/`). `/a/b/node` → `/a/b`; `/node` → `/`.
fn namespace_of(fqn: &str) -> &str {
    match fqn.rsplit_once('/') {
        Some(("", _)) | None => "/",
        Some((ns, _)) => ns,
    }
}

/// Walk a scope's parent chain (root → leaf) via `ScopeInfo.parent`.
fn scope_chain<'a>(
    scopes: &'a BTreeMap<String, ScopeInfo>,
    key: &'a str,
) -> Vec<(&'a str, &'a ScopeInfo)> {
    let mut chain = Vec::new();
    let mut cur = Some(key);
    while let Some(k) = cur {
        let Some(info) = scopes.get(k) else { break };
        chain.push((k, info));
        cur = info.parent.as_deref();
    }
    chain.reverse();
    chain
}

/// Count of node instances owning each scope key.
fn entity_counts(model: &SystemModel) -> BTreeMap<&str, usize> {
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    for inst in model.structure.nodes.values() {
        *counts.entry(inst.scope.as_str()).or_default() += 1;
    }
    counts
}

/// ANSI color codes (empty strings when not a terminal).
struct Colors {
    pkg: &'static str,
    file: &'static str,
    dim: &'static str,
    reset: &'static str,
}

impl Colors {
    fn auto() -> Self {
        use std::io::IsTerminal;
        if std::io::stdout().is_terminal() {
            Self {
                pkg: "\x1b[36m",  // cyan
                file: "\x1b[33m", // yellow
                dim: "\x1b[2m",   // dim
                reset: "\x1b[0m",
            }
        } else {
            Self {
                pkg: "",
                file: "",
                dim: "",
                reset: "",
            }
        }
    }
}

fn print_tree(model: &SystemModel) {
    let scopes = &model.structure.scopes;
    if scopes.is_empty() {
        println!("No scopes in the SystemModel (structure.scopes is empty)");
        return;
    }

    let c = Colors::auto();
    let counts = entity_counts(model);

    // children map: parent key (None = root) → child keys, in stable order.
    let mut children: BTreeMap<Option<&str>, Vec<&str>> = BTreeMap::new();
    for (key, info) in scopes {
        children
            .entry(info.parent.as_deref())
            .or_default()
            .push(key.as_str());
    }

    fn print_scope(
        scopes: &BTreeMap<String, ScopeInfo>,
        children: &BTreeMap<Option<&str>, Vec<&str>>,
        counts: &BTreeMap<&str, usize>,
        c: &Colors,
        key: &str,
        indent: usize,
    ) {
        let info = &scopes[key];
        let prefix = "  ".repeat(indent);
        let pkg = info.package.as_deref().unwrap_or("(none)");
        let file = info.file.as_deref().unwrap_or(key);
        let count = counts.get(key).copied().unwrap_or(0);
        let count_str = if count > 0 {
            format!("  {}({} entities){}", c.dim, count, c.reset)
        } else {
            String::new()
        };
        println!(
            "{prefix}{cpkg}{pkg}{reset} {cfile}{file}{reset}{count}",
            cpkg = c.pkg,
            reset = c.reset,
            cfile = c.file,
            count = count_str,
        );
        if let Some(child_keys) = children.get(&Some(key)) {
            for child in child_keys {
                print_scope(scopes, children, counts, c, child, indent + 1);
            }
        }
    }

    if let Some(roots) = children.get(&None) {
        for root in roots {
            print_scope(scopes, &children, &counts, &c, root, 0);
        }
    }
}

fn show_node_context(model: &SystemModel, fqn: &str) -> Result<()> {
    let Some(inst) = model.structure.nodes.get(fqn) else {
        eprintln!("Node not found: {fqn}");
        eprintln!("\nAvailable nodes:");
        for (name, i) in &model.structure.nodes {
            eprintln!("  [{}] {name}", node_kind(i));
        }
        bail!("Node '{fqn}' not found");
    };

    println!("node: {fqn}");
    println!("kind: {}", node_kind(inst));
    println!("namespace: {}", namespace_of(fqn));
    print_scope_origin(&model.structure.scopes, &inst.scope);

    println!("launch:");
    if let Some(pkg) = &inst.pkg {
        println!("  package: {pkg}");
    }
    if let Some(exec) = &inst.exec {
        println!("  executable: {exec}");
    }
    if let Some(plugin) = &inst.plugin {
        println!("  plugin: {plugin}");
    }
    if let Some(container) = &inst.container {
        println!("  container: {container}");
    }
    if let Some(nn) = &inst.node_name {
        println!("  node_name: {nn}");
    }
    if let Some(crit) = &inst.criticality {
        println!("  criticality: {crit}");
    }
    if inst.lifecycle {
        println!("  lifecycle: true");
    }
    print_params("  params", &inst.params);
    print_string_list("  params_files", &inst.params_files);
    print_remaps("  remaps", &inst.remaps);
    print_env("  env", &inst.env);
    if !inst.ros_args.is_empty() {
        println!("  ros_args: {}", inst.ros_args.join(" "));
    }
    if let Some(true) = inst.respawn {
        println!("  respawn: true");
    }
    print_cmd(&inst.raw_cmd);
    Ok(())
}

fn show_launch_context(model: &SystemModel, pkg: &str, file: &str, show_all: bool) -> Result<()> {
    let scopes = &model.structure.scopes;
    // Match by file; require the package only when the scope has a known one
    // (path-based includes carry no package — match those on file alone).
    let matches: Vec<&str> = scopes
        .iter()
        .filter(|(_, info)| {
            info.file.as_deref() == Some(file) && info.package.as_ref().is_none_or(|p| p == pkg)
        })
        .map(|(k, _)| k.as_str())
        .collect();

    if matches.is_empty() {
        eprintln!("Launch file not found: {pkg}/{file}");
        eprintln!("\nAvailable scopes:");
        for (key, info) in scopes {
            eprintln!(
                "  {key}  ({}/{})",
                info.package.as_deref().unwrap_or("?"),
                info.file.as_deref().unwrap_or("?"),
            );
        }
        bail!("Launch file '{pkg}/{file}' not found");
    }

    if matches.len() > 1 && !show_all {
        eprintln!(
            "{pkg}/{file} is included {} times. Use --all, or pick a scope key:",
            matches.len()
        );
        for key in &matches {
            eprintln!("  {key}");
        }
        bail!("Ambiguous launch file — use --all");
    }

    let counts_owner = model_scope_owners(model);
    for (i, &key) in matches.iter().enumerate() {
        if i > 0 {
            println!("---");
        }
        let info = &scopes[key];
        println!("launch_file: {pkg}/{file}");
        println!("scope: {key}");

        // Scope chain (root → this include).
        println!("scope_chain:");
        for (ck, ci) in scope_chain(scopes, key) {
            println!(
                "  - {} ({}/{})",
                ck,
                ci.package.as_deref().unwrap_or("null"),
                ci.file.as_deref().unwrap_or("?"),
            );
        }

        if !info.args.is_empty() {
            println!("args:");
            for (k, v) in &info.args {
                println!("  {k}: {v}");
            }
        }
        if let Some(manifest) = &info.manifest {
            println!("manifest: {manifest}");
        }

        // Entities owned by this scope.
        if let Some(members) = counts_owner.get(key) {
            println!("entities:");
            for (fqn, inst) in members {
                println!("  - [{}] {fqn}", node_kind(inst));
            }
        }

        // Child includes.
        let child_keys: Vec<&str> = scopes
            .iter()
            .filter(|(_, ci)| ci.parent.as_deref() == Some(key))
            .map(|(k, _)| k.as_str())
            .collect();
        if !child_keys.is_empty() {
            println!("includes:");
            for ck in child_keys {
                let ci = &scopes[ck];
                println!(
                    "  - {} ({}/{})",
                    ck,
                    ci.package.as_deref().unwrap_or("null"),
                    ci.file.as_deref().unwrap_or("?"),
                );
            }
        }
    }

    Ok(())
}

/// Group node instances by owning scope key (for the launch-file entity list).
fn model_scope_owners(model: &SystemModel) -> BTreeMap<&str, Vec<(&str, &NodeInstance)>> {
    let mut owners: BTreeMap<&str, Vec<(&str, &NodeInstance)>> = BTreeMap::new();
    for (fqn, inst) in &model.structure.nodes {
        owners
            .entry(inst.scope.as_str())
            .or_default()
            .push((fqn.as_str(), inst));
    }
    owners
}

fn node_kind(inst: &NodeInstance) -> &'static str {
    if inst.is_container {
        "container"
    } else if inst.plugin.is_some() {
        "load_node"
    } else {
        "node"
    }
}

// --- Output helpers ---

fn print_scope_origin(scopes: &BTreeMap<String, ScopeInfo>, scope_key: &str) {
    let Some(info) = scopes.get(scope_key) else {
        return;
    };
    println!("origin:");
    println!("  scope: {scope_key}");
    println!("  pkg: {}", info.package.as_deref().unwrap_or("null"));
    println!("  file: {}", info.file.as_deref().unwrap_or("?"));
    println!("scope_chain:");
    for (ck, ci) in scope_chain(scopes, scope_key) {
        println!(
            "  - {} ({}/{})",
            ck,
            ci.package.as_deref().unwrap_or("?"),
            ci.file.as_deref().unwrap_or("?"),
        );
    }
}

fn param_value_to_string(v: &ParamValue) -> String {
    match v {
        ParamValue::Bool(b) => b.to_string(),
        ParamValue::Int(i) => i.to_string(),
        ParamValue::Float(f) => f.to_string(),
        ParamValue::Str(s) => s.clone(),
        ParamValue::StrList(l) => format!("[{}]", l.join(", ")),
    }
}

fn print_params(label: &str, params: &BTreeMap<String, ParamValue>) {
    if !params.is_empty() {
        println!("{label}:");
        for (k, v) in params {
            println!("    {k}: {}", param_value_to_string(v));
        }
    }
}

fn print_remaps(label: &str, remaps: &[ros_launch_manifest_model::Remap]) {
    if !remaps.is_empty() {
        println!("{label}:");
        for r in remaps {
            println!("    {} -> {}", r.from, r.to);
        }
    }
}

fn print_string_list(label: &str, items: &[String]) {
    if !items.is_empty() {
        println!("{label}:");
        for item in items {
            let display = if item.len() > 80 {
                format!("{}...", &item[..77])
            } else {
                item.clone()
            };
            println!("    - {display}");
        }
    }
}

fn print_env(label: &str, env: &[ros_launch_manifest_model::EnvVar]) {
    if !env.is_empty() {
        println!("{label}:");
        for v in env {
            println!("    {}: {}", v.name, v.value);
        }
    }
}

fn print_cmd(cmd: &[String]) {
    if !cmd.is_empty() {
        println!("cmd:");
        for c in cmd {
            println!("  - {c}");
        }
    }
}
