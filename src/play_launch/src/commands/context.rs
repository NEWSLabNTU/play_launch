//! Context extraction command — show per-node or per-launch-file context
//! from a scoped record.json.

use crate::{
    cli::options::ContextArgs,
    ros::launch_dump::{
        ComposableNodeRecord, LaunchDump, NodeContainerRecord, NodeRecord, ScopeEntry,
        load_launch_dump,
    },
};
use eyre::{Result, bail};
use std::path::Path;

pub fn handle_context(args: &ContextArgs) -> Result<()> {
    let path = Path::new(&args.record);
    let dump = load_launch_dump(path)?;

    if args.tree {
        print_tree(&dump);
    } else if let Some(ref fqn) = args.node {
        show_node_context(&dump, fqn)?;
    } else if let Some(ref launch_args) = args.launch {
        let pkg = &launch_args[0];
        let file = &launch_args[1];
        show_launch_context(&dump, pkg, file, args.namespace.as_deref(), args.all)?;
    } else {
        bail!("Specify --tree, --node <FQN>, or --launch <PKG> <FILE>");
    }

    Ok(())
}

/// Build node FQN from a record's name/namespace fields.
fn node_fqn_from_node(n: &NodeRecord) -> String {
    let ns = n.namespace.as_deref().unwrap_or("/");
    let name = n
        .name
        .as_deref()
        .or(n.exec_name.as_deref())
        .unwrap_or(&n.executable);
    format_fqn(ns, name)
}

fn node_fqn_from_container(c: &NodeContainerRecord) -> String {
    format_fqn(&c.namespace, &c.name)
}

fn node_fqn_from_load_node(ln: &ComposableNodeRecord) -> String {
    format_fqn(&ln.namespace, &ln.node_name)
}

fn format_fqn(ns: &str, name: &str) -> String {
    if name.starts_with('/') {
        return name.to_string();
    }
    if ns.ends_with('/') {
        format!("{}{}", ns, name)
    } else {
        format!("{}/{}", ns, name)
    }
}

/// Walk scope parent chain from leaf to root.
fn scope_chain(scopes: &[ScopeEntry], scope_id: usize) -> Vec<&ScopeEntry> {
    let mut chain = Vec::new();
    let mut current = Some(scope_id);
    while let Some(id) = current {
        if id >= scopes.len() {
            break;
        }
        let s = &scopes[id];
        chain.push(s);
        current = s.parent;
    }
    chain.reverse();
    chain
}

/// ANSI color codes (empty strings when not a terminal)
struct Colors {
    pkg: &'static str,
    file: &'static str,
    ns: &'static str,
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
                ns: "\x1b[32m",   // green
                dim: "\x1b[2m",   // dim
                reset: "\x1b[0m",
            }
        } else {
            Self {
                pkg: "",
                file: "",
                ns: "",
                dim: "",
                reset: "",
            }
        }
    }
}

fn print_tree(dump: &LaunchDump) {
    if dump.scopes.is_empty() {
        println!("No scopes found in record.json");
        return;
    }

    let c = Colors::auto();

    // Count entities per scope
    let mut counts = std::collections::HashMap::new();
    for n in &dump.node {
        if let Some(s) = n.scope {
            *counts.entry(s).or_insert(0usize) += 1;
        }
    }
    for container in &dump.container {
        if let Some(s) = container.scope {
            *counts.entry(s).or_insert(0usize) += 1;
        }
    }
    for ln in &dump.load_node {
        if let Some(s) = ln.scope {
            *counts.entry(s).or_insert(0usize) += 1;
        }
    }

    // Build children map
    let mut children: std::collections::HashMap<Option<usize>, Vec<usize>> =
        std::collections::HashMap::new();
    for s in &dump.scopes {
        children.entry(s.parent).or_default().push(s.id);
    }

    fn print_scope(
        scopes: &[ScopeEntry],
        children: &std::collections::HashMap<Option<usize>, Vec<usize>>,
        counts: &std::collections::HashMap<usize, usize>,
        c: &Colors,
        id: usize,
        indent: usize,
    ) {
        let s = &scopes[id];
        let prefix = "  ".repeat(indent);
        let pkg = s.pkg().unwrap_or("(none)");
        let count = counts.get(&id).copied().unwrap_or(0);
        let count_str = if count > 0 {
            format!("  {}({} entities){}", c.dim, count, c.reset)
        } else {
            String::new()
        };
        if s.is_file_scope() {
            println!(
                "{}{dim}[{id:2}]{reset} {cpkg}{pkg}{reset} {cfile}{file}{reset}  {cns}ns={ns}{reset}{count}",
                prefix,
                dim = c.dim,
                id = s.id,
                reset = c.reset,
                cpkg = c.pkg,
                pkg = pkg,
                cfile = c.file,
                file = s.file().unwrap_or("?"),
                cns = c.ns,
                ns = s.ns,
                count = count_str,
            );
        } else {
            println!(
                "{}{dim}[{id:2}]{reset} {dim}<group>{reset}  {cns}ns={ns}{reset}{count}",
                prefix,
                dim = c.dim,
                id = s.id,
                reset = c.reset,
                cns = c.ns,
                ns = s.ns,
                count = count_str,
            );
        }
        if let Some(child_ids) = children.get(&Some(id)) {
            for &child_id in child_ids {
                print_scope(scopes, children, counts, c, child_id, indent + 1);
            }
        }
    }

    // Print from roots
    if let Some(root_ids) = children.get(&None) {
        for &root_id in root_ids {
            print_scope(&dump.scopes, &children, &counts, &c, root_id, 0);
        }
    }
}

fn show_node_context(dump: &LaunchDump, fqn: &str) -> Result<()> {
    // Search across all entity types
    // Nodes
    for n in &dump.node {
        if node_fqn_from_node(n) == fqn {
            println!("node: {}", fqn);
            println!("kind: node");
            print_scope_info(&dump.scopes, n.scope);
            println!("launch:");
            print_kv("  executable", &n.executable);
            print_opt("  package", &n.package);
            print_opt("  name", &n.name);
            print_opt("  namespace", &n.namespace);
            print_opt("  exec_name", &n.exec_name);
            print_params("  params", &n.params);
            print_string_list("  params_files", &n.params_files);
            print_remaps("  remaps", &n.remaps);
            print_env("  env", &n.env);
            print_opt_params("  global_params", &n.global_params);
            if let Some(true) = n.respawn {
                println!("  respawn: true");
            }
            print_cmd(&n.cmd);
            return Ok(());
        }
    }

    // Containers
    for c in &dump.container {
        if node_fqn_from_container(c) == fqn {
            println!("node: {}", fqn);
            println!("kind: container");
            print_scope_info(&dump.scopes, c.scope);
            println!("launch:");
            print_kv("  executable", &c.executable);
            print_kv("  package", &c.package);
            print_kv("  name", &c.name);
            print_kv("  namespace", &c.namespace);
            print_params("  params", &c.params);
            print_remaps("  remaps", &c.remaps);
            print_cmd(&c.cmd);
            return Ok(());
        }
    }

    // Load nodes
    for ln in &dump.load_node {
        if node_fqn_from_load_node(ln) == fqn {
            println!("node: {}", fqn);
            println!("kind: load_node");
            print_scope_info(&dump.scopes, ln.scope);
            println!("launch:");
            print_kv("  package", &ln.package);
            print_kv("  plugin", &ln.plugin);
            print_kv("  target_container", &ln.target_container_name);
            print_kv("  node_name", &ln.node_name);
            print_kv("  namespace", &ln.namespace);
            print_params("  params", &ln.params);
            print_remaps("  remaps", &ln.remaps);
            return Ok(());
        }
    }

    // Not found — list available
    eprintln!("Node not found: {}", fqn);
    eprintln!("\nAvailable nodes:");
    for n in &dump.node {
        eprintln!("  [node] {}", node_fqn_from_node(n));
    }
    for c in &dump.container {
        eprintln!("  [container] {}", node_fqn_from_container(c));
    }
    for ln in &dump.load_node {
        eprintln!("  [load_node] {}", node_fqn_from_load_node(ln));
    }
    bail!("Node '{}' not found", fqn);
}

fn show_launch_context(
    dump: &LaunchDump,
    pkg: &str,
    file: &str,
    namespace: Option<&str>,
    show_all: bool,
) -> Result<()> {
    let matches: Vec<&ScopeEntry> = dump
        .scopes
        .iter()
        .filter(|s| s.pkg() == Some(pkg) && s.file().unwrap_or("?") == file)
        .filter(|s| namespace.is_none() || s.ns == namespace.unwrap())
        .collect();

    if matches.is_empty() {
        eprintln!("Launch file not found: {}/{}", pkg, file);
        eprintln!("\nAvailable scopes:");
        for s in &dump.scopes {
            eprintln!(
                "  [{}] {}/{}  ns={}",
                s.id,
                s.pkg().unwrap_or("?"),
                s.file().unwrap_or("?"),
                s.ns
            );
        }
        bail!("Launch file '{}/{}' not found", pkg, file);
    }

    if matches.len() > 1 && !show_all && namespace.is_none() {
        eprintln!(
            "Multiple invocations of {}/{}. Use --namespace or --all.",
            pkg, file
        );
        for s in &matches {
            eprintln!("  ns={}  (scope {})", s.ns, s.id);
        }
        bail!("Ambiguous launch file — use --namespace or --all");
    }

    for (i, scope) in matches.iter().enumerate() {
        if i > 0 {
            println!("---");
        }
        println!("launch_file: {}/{}", pkg, file);
        println!("scope_id: {}", scope.id);
        println!("ns: {}", scope.ns);

        // Scope chain
        println!("scope_chain:");
        for s in scope_chain(&dump.scopes, scope.id) {
            println!("  - pkg: {}", s.pkg().unwrap_or("null"));
            println!("    file: {}", s.file().unwrap_or("?"));
            println!("    ns: {}", s.ns);
        }

        // Args
        if !scope.args.is_empty() {
            println!("args:");
            let mut args: Vec<_> = scope.args.iter().collect();
            args.sort_by_key(|(k, _)| k.as_str());
            for (k, v) in args {
                println!("  {}: {}", k, v);
            }
        }

        // Entities in this scope
        let mut entities = Vec::new();
        for n in &dump.node {
            if n.scope == Some(scope.id) {
                entities.push(format!("[node] {}", node_fqn_from_node(n)));
            }
        }
        for c in &dump.container {
            if c.scope == Some(scope.id) {
                entities.push(format!("[container] {}", node_fqn_from_container(c)));
            }
        }
        for ln in &dump.load_node {
            if ln.scope == Some(scope.id) {
                entities.push(format!("[load_node] {}", node_fqn_from_load_node(ln)));
            }
        }
        if !entities.is_empty() {
            println!("entities:");
            for e in &entities {
                println!("  - {}", e);
            }
        }

        // Child scopes
        let children: Vec<&ScopeEntry> = dump
            .scopes
            .iter()
            .filter(|s| s.parent == Some(scope.id))
            .collect();
        if !children.is_empty() {
            println!("includes:");
            for cs in &children {
                println!("  - pkg: {}", cs.pkg().unwrap_or("null"));
                println!("    file: {}", cs.file().unwrap_or("?"));
                println!("    ns: {}", cs.ns);
            }
        }
    }

    Ok(())
}

// --- Output helpers ---

fn print_scope_info(scopes: &[ScopeEntry], scope_id: Option<usize>) {
    if let Some(id) = scope_id
        && let Some(s) = scopes.get(id)
    {
        println!("origin:");
        println!("  pkg: {}", s.pkg().unwrap_or("null"));
        println!("  file: {}", s.file().unwrap_or("?"));
        println!("  ns: {}", s.ns);
        println!("scope_chain:");
        for sc in scope_chain(scopes, id) {
            println!(
                "  - {}/{}  ns={}",
                sc.pkg().unwrap_or("?"),
                sc.file().unwrap_or("?"),
                sc.ns
            );
        }
    }
}

fn print_kv(label: &str, value: &str) {
    println!("{}: {}", label, value);
}

fn print_opt(label: &str, value: &Option<String>) {
    if let Some(v) = value {
        println!("{}: {}", label, v);
    }
}

fn print_params(label: &str, params: &[(String, String)]) {
    if !params.is_empty() {
        println!("{}:", label);
        for (k, v) in params {
            println!(
                "{}  {}: {}",
                " ".repeat(label.len() - label.trim_start().len() + 2),
                k,
                v
            );
        }
    }
}

fn print_opt_params(label: &str, params: &Option<Vec<(String, String)>>) {
    if let Some(p) = params {
        print_params(label, p);
    }
}

fn print_remaps(label: &str, remaps: &[(String, String)]) {
    if !remaps.is_empty() {
        println!("{}:", label);
        for (from, to) in remaps {
            println!("    {} -> {}", from, to);
        }
    }
}

fn print_string_list(label: &str, items: &[String]) {
    if !items.is_empty() {
        println!("{}:", label);
        for item in items {
            // Truncate long param file content
            let display = if item.len() > 80 {
                format!("{}...", &item[..77])
            } else {
                item.clone()
            };
            println!("    - {}", display);
        }
    }
}

fn print_env(label: &str, env: &Option<Vec<(String, String)>>) {
    if let Some(vars) = env
        && !vars.is_empty()
    {
        println!("{}:", label);
        for (k, v) in vars {
            println!("    {}: {}", k, v);
        }
    }
}

fn print_cmd(cmd: &[String]) {
    if !cmd.is_empty() {
        println!("cmd:");
        for c in cmd {
            println!("  - {}", c);
        }
    }
}
