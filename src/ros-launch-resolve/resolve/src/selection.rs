//! Selecting part of a launch tree (phase 106 W1).
//!
//! A selection is a list of `only` and `exclude` selectors evaluated over a
//! resolved [`SystemModel`]: the union of the `only`s (everything when there
//! are none), minus the `exclude`s, then the closure (a kept composable keeps
//! its container). The model stays whole; the result names which members are
//! kept, and every consumer (the run, the web UI's switches, `export`,
//! `resolve --explain`) calls this one evaluator.
//!
//! Design: `docs/design/launch-selection.md` sections 5 and 9.
//!
//! Two rules a later change must keep:
//! - a launch file included more than once is selected only when the
//!   selector says which inclusions it means (`[<arg>=<value>]`,
//!   `[under=…]`, `[nth=…]`, `[*]`, or `#<id>`). Matching all of them
//!   silently would turn "exclude the `vector_map` monitor" into "exclude all
//!   ten" with a run that looks plausible;
//! - a selector that matches nothing, and a closure that would bring back a
//!   member an `exclude` named, are errors, never a silent no-op.

use std::{
    collections::{BTreeMap, BTreeSet, HashMap, HashSet},
    fmt,
    str::FromStr,
};

use glob::{MatchOptions, Pattern};
use ros_launch_manifest_model::{NodeInstance, ParamValue, ScopeInfo, SystemModel};

/// Launch-file suffixes that make a bare selector a scope.
const LAUNCH_SUFFIXES: &[&str] = &[
    ".launch.xml",
    ".launch.py",
    ".launch.yaml",
    ".xml",
    ".py",
    ".yaml",
];

/// In a node FQN, `*` stays inside one segment and `**` crosses segments.
const FQN_MATCH: MatchOptions = MatchOptions {
    case_sensitive: true,
    require_literal_separator: true,
    require_literal_leading_dot: false,
};

// ---------------------------------------------------------------------------
// Selectors
// ---------------------------------------------------------------------------

/// What a selector matches on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// An inclusion of a launch file, and everything under it.
    Scope,
    /// Nodes by model key (FQN).
    Node,
    /// Nodes by ROS package.
    Pkg,
    /// Plain nodes by executable.
    Exec,
    /// Composables by plugin type.
    Plugin,
    /// A container and its composables.
    Container,
}

impl Kind {
    fn prefix(self) -> &'static str {
        match self {
            Kind::Scope => "scope",
            Kind::Node => "node",
            Kind::Pkg => "pkg",
            Kind::Exec => "exec",
            Kind::Plugin => "plugin",
            Kind::Container => "container",
        }
    }

    fn from_prefix(s: &str) -> Option<Kind> {
        Some(match s {
            "scope" => Kind::Scope,
            "node" => Kind::Node,
            "pkg" => Kind::Pkg,
            "exec" => Kind::Exec,
            "plugin" => Kind::Plugin,
            "container" => Kind::Container,
            _ => return None,
        })
    }
}

/// A predicate telling repeated inclusions of one file apart.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Pred {
    /// `[<arg>=<value>]`: the inclusion's launch configuration has that value.
    Arg { key: String, value: String },
    /// `[under=<pkg>/<file>]`: that file is among the inclusion's ancestors.
    Under(String),
    /// `[nth=<n>]`: the n-th remaining inclusion, in inclusion order.
    Nth(usize),
    /// `[*]`: every inclusion.
    All,
}

impl fmt::Display for Pred {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Pred::Arg { key, value } => write!(f, "[{key}={value}]"),
            Pred::Under(s) => write!(f, "[under={s}]"),
            Pred::Nth(n) => write!(f, "[nth={n}]"),
            Pred::All => write!(f, "[*]"),
        }
    }
}

/// One selector, as written on the command line or by the web UI.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Selector {
    pub kind: Kind,
    /// The pattern: `[<pkg>/]<file>` for a scope, a glob otherwise.
    pub pattern: String,
    /// `#<id>`: one inclusion by its scope id (command line only).
    pub id: Option<String>,
    pub preds: Vec<Pred>,
}

impl fmt::Display for Selector {
    /// The canonical, prefixed form: what `--explain`, Copy command, an
    /// export's header and `run_info.json` record.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.kind.prefix(), self.pattern)?;
        if let Some(id) = &self.id {
            write!(f, "#{id}")?;
        }
        for p in &self.preds {
            write!(f, "{p}")?;
        }
        Ok(())
    }
}

fn has_launch_suffix(s: &str) -> bool {
    LAUNCH_SUFFIXES.iter().any(|x| s.ends_with(x))
}

fn has_glob(s: &str) -> bool {
    s.contains(['*', '?', '['])
}

impl FromStr for Selector {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, String> {
        let text = text.trim();
        if text.is_empty() {
            return Err("an empty selector".into());
        }
        let (kind, rest) = match text.split_once(':') {
            Some((head, rest)) if Kind::from_prefix(head).is_some() => {
                (Kind::from_prefix(head).unwrap(), rest)
            }
            _ if text.starts_with('/') => (Kind::Node, text),
            _ => {
                let head = text.split(['[', '#']).next().unwrap_or(text);
                if has_launch_suffix(head) {
                    (Kind::Scope, text)
                } else {
                    return Err(format!(
                        "`{text}`: say what it names: `/…` or `node:` for a node, a launch-file \
                         name or `scope:` for an include, or `pkg:`, `exec:`, `plugin:`, \
                         `container:`"
                    ));
                }
            }
        };
        if rest.is_empty() {
            return Err(format!("`{text}`: nothing after `{}:`", kind.prefix()));
        }
        if kind != Kind::Scope {
            // Brackets in a node or package pattern are glob classes.
            Pattern::new(rest).map_err(|e| format!("`{text}`: invalid pattern: {e}"))?;
            return Ok(Selector {
                kind,
                pattern: rest.to_string(),
                id: None,
                preds: Vec::new(),
            });
        }
        let cut = rest.find(['[', '#']).unwrap_or(rest.len());
        let pattern = &rest[..cut];
        let mut tail = &rest[cut..];
        if pattern.is_empty() {
            return Err(format!("`{text}`: no launch file named"));
        }
        let mut id = None;
        if let Some(t) = tail.strip_prefix('#') {
            let end = t.find('[').unwrap_or(t.len());
            let digits = &t[..end];
            if digits.is_empty() || !digits.chars().all(|c| c.is_ascii_digit()) {
                return Err(format!(
                    "`{text}`: `#` takes a scope id, as `context --all` prints it"
                ));
            }
            id = Some(digits.to_string());
            tail = &t[end..];
        }
        let mut preds = Vec::new();
        while !tail.is_empty() {
            let Some(body) = tail.strip_prefix('[') else {
                return Err(format!("`{text}`: unexpected `{tail}` after the file name"));
            };
            let Some(end) = body.find(']') else {
                return Err(format!("`{text}`: `[` without `]`"));
            };
            let p = &body[..end];
            tail = &body[end + 1..];
            preds.push(if p == "*" {
                Pred::All
            } else if let Some(v) = p.strip_prefix("under=") {
                Pred::Under(v.to_string())
            } else if let Some(v) = p.strip_prefix("nth=") {
                let n: usize = v
                    .parse()
                    .ok()
                    .filter(|n| *n > 0)
                    .ok_or_else(|| format!("`{text}`: `nth=` takes a number from 1"))?;
                Pred::Nth(n)
            } else if let Some((k, v)) = p.split_once('=') {
                Pred::Arg {
                    key: k.to_string(),
                    value: v.to_string(),
                }
            } else {
                return Err(format!(
                    "`{text}`: `[{p}]` is not a predicate; use `[<arg>=<value>]`, \
                     `[under=<pkg>/<file>]`, `[nth=<n>]` or `[*]`"
                ));
            });
        }
        Ok(Selector {
            kind,
            pattern: pattern.to_string(),
            id,
            preds,
        })
    }
}

/// What to select: the parsed flags.
#[derive(Debug, Clone, Default)]
pub struct SelectionSpec {
    pub only: Vec<Selector>,
    pub exclude: Vec<Selector>,
    /// Do not keep the containers of kept composables.
    pub no_deps: bool,
    /// Keep nothing (`--start none`); every member starts Excluded.
    pub start_none: bool,
}

impl SelectionSpec {
    /// Parse `--only` and `--exclude` values; every bad one is reported.
    pub fn parse(
        only: &[String],
        exclude: &[String],
        no_deps: bool,
        start_none: bool,
    ) -> eyre::Result<Self> {
        let mut errors = Vec::new();
        let mut parse_all = |v: &[String]| -> Vec<Selector> {
            v.iter()
                .filter_map(|s| s.parse().map_err(|e| errors.push(e)).ok())
                .collect()
        };
        let only = parse_all(only);
        let exclude = parse_all(exclude);
        if !errors.is_empty() {
            eyre::bail!("invalid selector:\n  {}", errors.join("\n  "));
        }
        if start_none && (!only.is_empty() || !exclude.is_empty()) {
            eyre::bail!(
                "--start none keeps nothing, so --only and --exclude have nothing to act on"
            );
        }
        Ok(SelectionSpec {
            only,
            exclude,
            no_deps,
            start_none,
        })
    }

    /// Whether this selects anything other than the whole tree.
    pub fn is_whole(&self) -> bool {
        self.only.is_empty() && self.exclude.is_empty() && !self.start_none
    }

    /// The selection as command-line flags, canonical form, shell-quoted.
    pub fn to_flags(&self) -> Vec<String> {
        let mut out = Vec::new();
        if self.start_none {
            out.push("--start none".to_string());
        }
        for s in &self.only {
            out.push(format!("--only {}", shell_quote(&s.to_string())));
        }
        for s in &self.exclude {
            out.push(format!("--exclude {}", shell_quote(&s.to_string())));
        }
        if self.no_deps {
            out.push("--no-deps".to_string());
        }
        out
    }
}

/// Quote for a POSIX shell when the text needs it.
pub fn shell_quote(s: &str) -> String {
    if !s.is_empty()
        && s.chars()
            .all(|c| c.is_ascii_alphanumeric() || "/-_.:=@%+,".contains(c))
    {
        s.to_string()
    } else {
        format!("'{}'", s.replace('\'', r"'\''"))
    }
}

// ---------------------------------------------------------------------------
// The tree index
// ---------------------------------------------------------------------------

/// The scope tree and node membership, built once per model.
pub struct TreeIndex<'m> {
    model: &'m SystemModel,
    /// Scope key → children keys, in inclusion order.
    pub children: BTreeMap<String, Vec<String>>,
    /// Root scope keys.
    pub roots: Vec<String>,
    /// Scope key → its nodes (direct members), in model order.
    pub direct: HashMap<String, Vec<String>>,
    /// Scope key → inclusion order among the inclusions of one file.
    order: HashMap<String, u64>,
}

/// The number after `#` in a scope key, 0 when there is none.
fn scope_number(key: &str) -> u64 {
    key.rsplit_once('#')
        .and_then(|(_, n)| n.parse().ok())
        .unwrap_or(0)
}

impl<'m> TreeIndex<'m> {
    pub fn new(model: &'m SystemModel) -> Self {
        let scopes = &model.structure.scopes;
        let mut children: BTreeMap<String, Vec<String>> = BTreeMap::new();
        let mut roots = Vec::new();
        for (k, info) in scopes {
            match info.parent.as_deref().filter(|p| scopes.contains_key(*p)) {
                Some(p) => children.entry(p.to_string()).or_default().push(k.clone()),
                None => roots.push(k.clone()),
            }
        }
        // Inclusion order: the first node each scope's subtree holds, in
        // model (traversal) order, then the scope number.
        let mut first_node: HashMap<&str, usize> = HashMap::new();
        let mut direct: HashMap<String, Vec<String>> = HashMap::new();
        for (i, (key, node)) in model.structure.nodes.iter().enumerate() {
            direct
                .entry(node.scope.clone())
                .or_default()
                .push(key.clone());
            let mut s = Some(node.scope.as_str());
            while let Some(k) = s {
                first_node.entry(k).or_insert(i);
                s = scopes.get(k).and_then(|i| i.parent.as_deref());
            }
        }
        let order: HashMap<String, u64> = scopes
            .keys()
            .map(|k| {
                let n = first_node.get(k.as_str()).copied().unwrap_or(usize::MAX) as u64;
                (k.clone(), n.saturating_mul(1 << 16) + scope_number(k))
            })
            .collect();
        let sort = |v: &mut Vec<String>| v.sort_by_key(|k| (order[k], k.clone()));
        for v in children.values_mut() {
            sort(v);
        }
        sort(&mut roots);
        TreeIndex {
            model,
            children,
            roots,
            direct,
            order,
        }
    }

    fn info(&self, key: &str) -> Option<&ScopeInfo> {
        self.model.structure.scopes.get(key)
    }

    /// Whether `ancestor` is `scope` or one of its ancestors.
    fn is_within(&self, scope: &str, ancestor: &str) -> bool {
        let mut s = Some(scope);
        while let Some(k) = s {
            if k == ancestor {
                return true;
            }
            s = self.info(k).and_then(|i| i.parent.as_deref());
        }
        false
    }

    /// Every node in the subtree of `scope`.
    pub fn subtree_nodes(&self, scope: &str) -> Vec<String> {
        let mut out = Vec::new();
        let mut stack = vec![scope.to_string()];
        while let Some(s) = stack.pop() {
            if let Some(n) = self.direct.get(&s) {
                out.extend(n.iter().cloned());
            }
            if let Some(c) = self.children.get(&s) {
                stack.extend(c.iter().cloned());
            }
        }
        out
    }

    /// `<pkg>/<file>` of a scope, or its key when either is unknown.
    pub fn file_of(&self, key: &str) -> String {
        match self.info(key) {
            Some(ScopeInfo {
                package: Some(p),
                file: Some(f),
                ..
            }) => format!("{p}/{f}"),
            _ => key.split('#').next().unwrap_or(key).to_string(),
        }
    }

    /// The inclusions of the same file as `key`, in inclusion order.
    fn siblings_of_file(&self, key: &str) -> Vec<String> {
        let file = self.file_of(key);
        let mut v: Vec<String> = self
            .model
            .structure
            .scopes
            .keys()
            .filter(|k| self.file_of(k) == file)
            .cloned()
            .collect();
        v.sort_by_key(|k| self.order[k]);
        v
    }

    /// The most specific stable selector for one inclusion: the file by name
    /// when it is included once; else with the argument that tells it apart;
    /// else with its parent; else with its position.
    pub fn stable_selector(&self, key: &str) -> Selector {
        let file = self.file_of(key);
        let group = self.siblings_of_file(key);
        let mut sel = Selector {
            kind: Kind::Scope,
            pattern: file,
            id: None,
            preds: Vec::new(),
        };
        if group.len() <= 1 {
            return sel;
        }
        sel.preds = self.distinguishing_preds(key, &group);
        sel
    }

    fn distinguishing_preds(&self, key: &str, group: &[String]) -> Vec<Pred> {
        let args = |k: &str| self.info(k).map(|i| &i.args);
        // An argument whose value is unique to this inclusion, preferring one
        // the include passed (its value differs from the parent's).
        if let Some(mine) = args(key) {
            let parent_args = self
                .info(key)
                .and_then(|i| i.parent.as_deref())
                .and_then(|p| args(p));
            let mut candidates: Vec<(&String, &String)> = mine
                .iter()
                .filter(|(k, v)| {
                    group
                        .iter()
                        .filter(|g| g.as_str() != key)
                        .all(|g| args(g).and_then(|a| a.get(*k)) != Some(*v))
                })
                .collect();
            candidates.sort_by_key(|(k, v)| {
                let passed = parent_args.and_then(|p| p.get(*k)) != Some(*v);
                (!passed, k.len(), (*k).clone())
            });
            if let Some((k, v)) = candidates.first() {
                return vec![Pred::Arg {
                    key: (*k).clone(),
                    value: (*v).clone(),
                }];
            }
        }
        let parent = self.info(key).and_then(|i| i.parent.clone());
        if let Some(p) = &parent {
            let same_parent: Vec<&String> = group
                .iter()
                .filter(|g| self.info(g).and_then(|i| i.parent.as_ref()) == Some(p))
                .collect();
            let under = Pred::Under(self.file_of(p));
            if same_parent.len() == 1 && self.file_unique(p) {
                return vec![under];
            }
            if let Some(pos) = same_parent.iter().position(|g| g.as_str() == key) {
                if self.file_unique(p) {
                    return vec![under, Pred::Nth(pos + 1)];
                }
            }
        }
        let pos = group.iter().position(|g| g == key).unwrap_or(0);
        vec![Pred::Nth(pos + 1)]
    }

    fn file_unique(&self, key: &str) -> bool {
        self.siblings_of_file(key).len() == 1
    }

    /// A short label telling one inclusion of a repeated file apart.
    pub fn repeat_label(&self, key: &str) -> Option<String> {
        let group = self.siblings_of_file(key);
        if group.len() <= 1 {
            return None;
        }
        Some(
            self.distinguishing_preds(key, &group)
                .iter()
                .map(|p| match p {
                    Pred::Arg { key, value } => format!("{key}={value}"),
                    Pred::Under(s) => format!("under {s}"),
                    Pred::Nth(n) => format!("#{n}"),
                    Pred::All => "*".into(),
                })
                .collect::<Vec<_>>()
                .join(", "),
        )
    }
}

// ---------------------------------------------------------------------------
// Evaluation
// ---------------------------------------------------------------------------

/// A container the closure kept for the composables that need it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Added {
    pub container: String,
    pub needed_by: Vec<String>,
}

/// A topic kept nodes subscribe to whose publishers are all excluded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CutInput {
    pub topic: String,
    pub msg_type: String,
    pub subscribers: Vec<String>,
    pub publishers: Vec<String>,
}

/// How many members one selector matched.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Matched {
    pub selector: String,
    pub exclude: bool,
    pub count: usize,
}

/// The result of evaluating a selection over a model.
#[derive(Debug, Clone, Default)]
pub struct Selection {
    /// The selection as given, in canonical form.
    pub spec_only: Vec<String>,
    pub spec_exclude: Vec<String>,
    pub no_deps: bool,
    pub start_none: bool,
    /// Kept member keys, in model order.
    pub kept: Vec<String>,
    /// Excluded member keys, in model order.
    pub excluded: Vec<String>,
    pub added: Vec<Added>,
    pub matched: Vec<Matched>,
    pub cut_inputs: Vec<CutInput>,
    pub warnings: Vec<String>,
    kept_set: HashSet<String>,
}

impl Selection {
    pub fn is_kept(&self, key: &str) -> bool {
        self.kept_set.contains(key)
    }

    /// Whether the run differs from the whole tree.
    pub fn is_partial(&self) -> bool {
        !self.excluded.is_empty()
    }

    /// The selection as command-line flags, canonical form.
    pub fn flags(&self) -> Vec<String> {
        let mut out = Vec::new();
        if self.start_none {
            out.push("--start".into());
            out.push("none".into());
        }
        for s in &self.spec_only {
            out.push("--only".into());
            out.push(s.clone());
        }
        for s in &self.spec_exclude {
            out.push("--exclude".into());
            out.push(s.clone());
        }
        if self.no_deps {
            out.push("--no-deps".into());
        }
        out
    }

    /// A one-line summary for the startup log.
    pub fn summary(&self) -> String {
        format!(
            "selection: {} of {} members kept, {} excluded{}{}",
            self.kept.len(),
            self.kept.len() + self.excluded.len(),
            self.excluded.len(),
            if self.added.is_empty() {
                String::new()
            } else {
                format!(
                    ", {} container(s) added for their composables",
                    self.added.len()
                )
            },
            if self.cut_inputs.is_empty() {
                String::new()
            } else {
                format!(", {} input topic(s) cut", self.cut_inputs.len())
            },
        )
    }
}

/// Keys of members a selector matches. `Err` explains why it cannot be used.
fn match_selector(
    tree: &TreeIndex<'_>,
    model: &SystemModel,
    sel: &Selector,
) -> Result<BTreeSet<String>, String> {
    let nodes = &model.structure.nodes;
    let glob = |p: &str| Pattern::new(p).map_err(|e| format!("`{sel}`: invalid pattern: {e}"));
    let out: BTreeSet<String> = match sel.kind {
        Kind::Node => {
            let p = glob(&sel.pattern)?;
            nodes
                .keys()
                .filter(|k| p.matches_with(k, FQN_MATCH))
                .cloned()
                .collect()
        }
        Kind::Pkg | Kind::Exec | Kind::Plugin => {
            let p = glob(&sel.pattern)?;
            let field = |n: &NodeInstance| -> Option<String> {
                match sel.kind {
                    Kind::Pkg => n.pkg.clone(),
                    Kind::Exec => n.exec.clone(),
                    _ => n.plugin.clone(),
                }
            };
            nodes
                .iter()
                .filter(|(_, n)| field(n).is_some_and(|v| p.matches(&v)))
                .map(|(k, _)| k.clone())
                .collect()
        }
        Kind::Container => {
            let p = glob(&sel.pattern)?;
            let containers: BTreeSet<&str> = nodes
                .values()
                .filter_map(|n| n.container.as_deref())
                .filter(|c| p.matches_with(c, FQN_MATCH))
                .collect();
            nodes
                .iter()
                .filter(|(k, n)| {
                    containers.contains(k.as_str())
                        || n.container
                            .as_deref()
                            .is_some_and(|c| containers.contains(c))
                })
                .map(|(k, _)| k.clone())
                .collect()
        }
        Kind::Scope => match_scope(tree, model, sel)?
            .iter()
            .flat_map(|s| tree.subtree_nodes(s))
            .collect(),
    };
    if out.is_empty() {
        return Err(format!("`{sel}` matches no member of this launch"));
    }
    Ok(out)
}

/// The scope keys a scope selector names.
fn match_scope(
    tree: &TreeIndex<'_>,
    model: &SystemModel,
    sel: &Selector,
) -> Result<Vec<String>, String> {
    let (pkg_pat, file_pat) = match sel.pattern.split_once('/') {
        Some((p, f)) => (Some(p), f),
        None => (None, sel.pattern.as_str()),
    };
    let pkg_glob = pkg_pat
        .map(Pattern::new)
        .transpose()
        .map_err(|e| format!("`{sel}`: invalid pattern: {e}"))?;
    let file_glob = Pattern::new(file_pat).map_err(|e| format!("`{sel}`: invalid pattern: {e}"))?;
    let mut candidates: Vec<String> = model
        .structure
        .scopes
        .iter()
        .filter(|(_, i)| {
            i.file.as_deref().is_some_and(|f| file_glob.matches(f))
                && pkg_glob
                    .as_ref()
                    .is_none_or(|p| i.package.as_deref().is_some_and(|pk| p.matches(pk)))
        })
        .map(|(k, _)| k.clone())
        .collect();
    candidates.sort_by_key(|k| tree.order[k]);
    if candidates.is_empty() {
        return Err(format!("`{sel}` matches no launch file in this tree"));
    }
    // A bare file name found in several packages names nothing in particular.
    let packages: BTreeSet<String> = candidates
        .iter()
        .filter_map(|k| tree.info(k).and_then(|i| i.package.clone()))
        .collect();
    if pkg_pat.is_none() && !has_glob(file_pat) && packages.len() > 1 {
        return Err(format!(
            "`{sel}` names a file in {} packages; say which:\n    {}",
            packages.len(),
            packages
                .iter()
                .map(|p| format!("scope:{p}/{file_pat}"))
                .collect::<Vec<_>>()
                .join("\n    ")
        ));
    }
    if let Some(id) = &sel.id {
        candidates.retain(|k| k.rsplit_once('#').is_some_and(|(_, n)| n == id));
        if candidates.is_empty() {
            return Err(format!(
                "`{sel}`: no inclusion of that file has scope id #{id}"
            ));
        }
    }
    let wants_all = sel.preds.contains(&Pred::All);
    for p in &sel.preds {
        match p {
            Pred::Arg { key, value } => candidates.retain(|k| {
                tree.info(k)
                    .is_some_and(|i| i.args.get(key).is_some_and(|v| v == value))
            }),
            Pred::Under(anc) => {
                let s: Selector = Selector {
                    kind: Kind::Scope,
                    pattern: anc.clone(),
                    id: None,
                    preds: vec![Pred::All],
                };
                let ancestors = match_scope(tree, model, &s)
                    .map_err(|e| format!("`{sel}`: in `under=`: {e}"))?;
                candidates.retain(|k| {
                    let parent = tree.info(k).and_then(|i| i.parent.clone());
                    parent.is_some_and(|p| ancestors.iter().any(|a| tree.is_within(&p, a)))
                });
            }
            Pred::Nth(_) | Pred::All => {}
        }
    }
    if let Some(n) = sel.preds.iter().find_map(|p| match p {
        Pred::Nth(n) => Some(*n),
        _ => None,
    }) {
        candidates = candidates.get(n - 1).cloned().into_iter().collect();
    }
    if candidates.is_empty() {
        return Err(format!("`{sel}` matches no inclusion of that file"));
    }
    // A file included more than once, named without saying which.
    if sel.id.is_none() && sel.preds.is_empty() && !wants_all {
        let mut by_file: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for k in &candidates {
            by_file.entry(tree.file_of(k)).or_default().push(k.clone());
        }
        let repeated: Vec<(&String, &Vec<String>)> =
            by_file.iter().filter(|(_, v)| v.len() > 1).collect();
        if !repeated.is_empty() {
            let mut msg = String::new();
            for (file, keys) in repeated {
                let name = file.rsplit('/').next().unwrap_or(file);
                msg.push_str(&format!(
                    "`{name}` is included {} times; say which:\n",
                    keys.len()
                ));
                for k in keys {
                    msg.push_str(&format!("    {}\n", tree.stable_selector(k)));
                }
                msg.push_str(&format!("    or scope:{file}[*] for all {}\n", keys.len()));
            }
            return Err(msg.trim_end().to_string());
        }
    }
    if sel.preds.iter().any(|p| matches!(p, Pred::Nth(_))) {
        // Reported by the caller as a warning: a position shifts when the
        // parent is edited.
    }
    Ok(candidates)
}

/// Map an endpoint ref (`<node FQN>/<endpoint>`) to its node key: the longest
/// node key that is a path prefix of the ref.
fn endpoint_node<'a>(nodes: &'a HashSet<&'a str>, r: &str) -> Option<&'a str> {
    let mut end = r.len();
    while let Some(i) = r[..end].rfind('/') {
        if i == 0 {
            break;
        }
        if let Some(k) = nodes.get(&r[..i]) {
            return Some(k);
        }
        end = i;
    }
    None
}

fn param_bool(v: &ParamValue) -> bool {
    match v {
        ParamValue::Bool(b) => *b,
        ParamValue::Str(s) => s.eq_ignore_ascii_case("true"),
        _ => false,
    }
}

fn unquote(s: &str) -> &str {
    let s = s.trim();
    s.strip_prefix('\'')
        .and_then(|x| x.strip_suffix('\''))
        .or_else(|| s.strip_prefix('"').and_then(|x| x.strip_suffix('"')))
        .unwrap_or(s)
}

/// The namespace part of a node key (`""` at the root).
fn namespace_of(key: &str) -> &str {
    key.rsplit_once('/').map(|(ns, _)| ns).unwrap_or("")
}

/// The ROS name a node registers under, when the launch file named it.
fn ros_fqn(key: &str, n: &NodeInstance) -> String {
    match &n.node_name {
        Some(name) if name.starts_with('/') => name.clone(),
        Some(name) => format!("{}/{}", namespace_of(key), name),
        None => key.to_string(),
    }
}

/// Evaluate a selection over a model.
pub fn evaluate(model: &SystemModel, spec: &SelectionSpec) -> eyre::Result<Selection> {
    let tree = TreeIndex::new(model);
    let nodes = &model.structure.nodes;
    let mut errors = Vec::new();
    let mut matched = Vec::new();
    let mut warnings = Vec::new();

    let mut run = |sels: &[Selector], exclude: bool| -> BTreeSet<String> {
        let mut all = BTreeSet::new();
        for s in sels {
            match match_selector(&tree, model, s) {
                Ok(m) => {
                    matched.push(Matched {
                        selector: s.to_string(),
                        exclude,
                        count: m.len(),
                    });
                    if s.preds.iter().any(|p| matches!(p, Pred::Nth(_))) {
                        warnings.push(format!(
                            "`{s}` selects by position; editing the including file can shift it"
                        ));
                    }
                    all.extend(m);
                }
                Err(e) => errors.push(e),
            }
        }
        all
    };
    let only = run(&spec.only, false);
    let excluded_explicitly = run(&spec.exclude, true);
    if !errors.is_empty() {
        eyre::bail!("{}", errors.join("\n"));
    }

    let mut kept: HashSet<String> = if spec.start_none {
        HashSet::new()
    } else if spec.only.is_empty() {
        nodes.keys().cloned().collect()
    } else {
        only.into_iter().collect()
    };
    for k in &excluded_explicitly {
        kept.remove(k);
    }

    let mut sel = complete(model, kept, &excluded_explicitly, spec.no_deps, warnings)?;
    sel.spec_only = spec.only.iter().map(|s| s.to_string()).collect();
    sel.spec_exclude = spec.exclude.iter().map(|s| s.to_string()).collect();
    sel.start_none = spec.start_none;
    sel.matched = matched;
    Ok(sel)
}

/// The closure, the cut inputs and the named traps over a kept set. `off`
/// names the members the user explicitly switched off or excluded: the
/// closure never brings one back.
fn complete(
    model: &SystemModel,
    mut kept: HashSet<String>,
    off: &BTreeSet<String>,
    no_deps: bool,
    mut warnings: Vec<String>,
) -> eyre::Result<Selection> {
    let nodes = &model.structure.nodes;
    // The closure: a kept composable keeps its container.
    let mut added: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut closure_errors = Vec::new();
    for (k, n) in nodes {
        let Some(c) = &n.container else { continue };
        if !kept.contains(k) || kept.contains(c) || !nodes.contains_key(c) {
            continue;
        }
        if no_deps {
            warnings.push(format!(
                "`{k}` is kept without its container `{c}` (--no-deps): it loads only if \
                 something else starts that container"
            ));
        } else if off.contains(c) {
            closure_errors.push(format!(
                "`{k}` is kept and needs its container `{c}`, which an --exclude names; \
                 exclude the composable too, or stop excluding the container"
            ));
        } else {
            added.entry(c.clone()).or_default().push(k.clone());
        }
    }
    if !closure_errors.is_empty() {
        closure_errors.dedup();
        eyre::bail!("{}", closure_errors.join("\n"));
    }
    for c in added.keys() {
        kept.insert(c.clone());
    }

    // Cut inputs: topics kept nodes take whose publishers are all excluded.
    let node_keys: HashSet<&str> = nodes.keys().map(|k| k.as_str()).collect();
    let mut cut_inputs = Vec::new();
    for (topic, w) in &model.structure.topics {
        let pubs: BTreeSet<&str> = w
            .publishers
            .iter()
            .filter_map(|r| endpoint_node(&node_keys, r))
            .collect();
        if pubs.is_empty() || pubs.iter().any(|p| kept.contains(*p)) {
            continue;
        }
        let subs: BTreeSet<&str> = w
            .subscribers
            .iter()
            .filter_map(|r| endpoint_node(&node_keys, r))
            .filter(|n| kept.contains(*n))
            .collect();
        if subs.is_empty() {
            continue;
        }
        cut_inputs.push(CutInput {
            topic: topic.clone(),
            msg_type: w.msg_type.clone(),
            subscribers: subs.iter().map(|s| s.to_string()).collect(),
            publishers: pubs.iter().map(|s| s.to_string()).collect(),
        });
    }

    // The named traps.
    let excluded_names: HashMap<String, &str> = nodes
        .iter()
        .filter(|(k, _)| !kept.contains(*k))
        .flat_map(|(k, n)| [(k.clone(), k.as_str()), (ros_fqn(k, n), k.as_str())])
        .collect();
    let mut sim_time = 0usize;
    for (k, n) in nodes.iter().filter(|(k, _)| kept.contains(*k)) {
        let params = n.resolved_params(k);
        if let Some(ParamValue::StrList(names)) = params.get("node_names") {
            for name in names {
                let name = unquote(name);
                let fqn = if name.starts_with('/') {
                    name.to_string()
                } else {
                    format!("{}/{}", namespace_of(k), name)
                };
                if let Some(ex) = excluded_names.get(&fqn) {
                    warnings.push(format!(
                        "lifecycle manager `{k}` manages `{ex}`, which is excluded; a manager \
                         that cannot reach a node it manages may abort the whole bringup"
                    ));
                }
            }
        }
        if params.get("use_sim_time").is_some_and(param_bool) {
            sim_time += 1;
        }
    }
    if sim_time > 0
        && let Some(c) = cut_inputs.iter().find(|c| c.topic == "/clock")
    {
        warnings.push(format!(
            "use_sim_time on {sim_time} kept node(s), and every /clock publisher ({}) is \
             excluded: their time will not advance",
            c.publishers.join(", ")
        ));
    }
    for c in cut_inputs
        .iter()
        .filter(|c| c.topic == "/tf" || c.topic == "/tf_static")
    {
        warnings.push(format!(
            "TF: {} kept node(s) subscribe to {} and every publisher of it ({}) is excluded",
            c.subscribers.len(),
            c.topic,
            c.publishers.join(", ")
        ));
    }

    let kept_list: Vec<String> = nodes
        .keys()
        .filter(|k| kept.contains(*k))
        .cloned()
        .collect();
    let excluded: Vec<String> = nodes
        .keys()
        .filter(|k| !kept.contains(*k))
        .cloned()
        .collect();
    Ok(Selection {
        no_deps,
        kept: kept_list,
        excluded,
        added: added
            .into_iter()
            .map(|(container, needed_by)| Added {
                container,
                needed_by,
            })
            .collect(),
        cut_inputs,
        warnings,
        kept_set: kept,
        ..Default::default()
    })
}

/// Evaluate an explicit kept set, as the web UI's switches produce it: the
/// members switched on, with `off` the ones switched off. The same closure,
/// cut inputs and traps as [`evaluate`].
pub fn evaluate_set(
    model: &SystemModel,
    kept: HashSet<String>,
    off: &BTreeSet<String>,
    no_deps: bool,
) -> eyre::Result<Selection> {
    complete(model, kept, off, no_deps, Vec::new())
}

// ---------------------------------------------------------------------------
// --explain
// ---------------------------------------------------------------------------

/// The marked tree and the reports, as `resolve --explain` prints them.
pub fn render_explain(model: &SystemModel, sel: &Selection) -> String {
    let tree = TreeIndex::new(model);
    let added: HashSet<&str> = sel.added.iter().map(|a| a.container.as_str()).collect();
    let mut out = String::new();
    out.push_str(&sel.summary());
    out.push('\n');
    if !sel.matched.is_empty() {
        out.push_str("\nSelectors\n");
        for m in &sel.matched {
            out.push_str(&format!(
                "  {} {}  {} member(s)\n",
                if m.exclude { "--exclude" } else { "--only   " },
                m.selector,
                m.count
            ));
        }
    }
    out.push_str("\nTree   [x] kept  [ ] excluded  [~] partly kept  [+] kept for a composable\n");
    fn walk(
        tree: &TreeIndex<'_>,
        sel: &Selection,
        added: &HashSet<&str>,
        key: &str,
        depth: usize,
        out: &mut String,
    ) {
        let all = tree.subtree_nodes(key);
        if all.is_empty() {
            // A file that starts nothing (a parameter loader) has nothing to
            // switch.
            return;
        }
        let kept = all.iter().filter(|n| sel.is_kept(n)).count();
        let mark = if all.is_empty() || kept == 0 {
            "[ ]"
        } else if kept == all.len() {
            "[x]"
        } else {
            "[~]"
        };
        let label = match tree.repeat_label(key) {
            Some(l) => format!("{}  ({l})", tree.file_of(key)),
            None => tree.file_of(key),
        };
        let pad = "  ".repeat(depth + 1);
        out.push_str(&format!("{pad}{mark} {label}  {kept}/{}\n", all.len()));
        if mark != "[~]" {
            return;
        }
        for n in tree.direct.get(key).into_iter().flatten() {
            let m = if added.contains(n.as_str()) {
                "[+]"
            } else if sel.is_kept(n) {
                "[x]"
            } else {
                "[ ]"
            };
            out.push_str(&format!("{pad}  {m} {n}\n"));
        }
        for c in tree.children.get(key).into_iter().flatten() {
            walk(tree, sel, added, c, depth + 1, out);
        }
    }
    for r in &tree.roots {
        walk(&tree, sel, &added, r, 0, &mut out);
    }
    if !sel.added.is_empty() {
        out.push_str("\nKept for their composables (--no-deps to leave them out)\n");
        for a in &sel.added {
            out.push_str(&format!(
                "  [+] {}  needed by {} composable(s)\n",
                a.container,
                a.needed_by.len()
            ));
        }
    }
    if !sel.cut_inputs.is_empty() {
        out.push_str("\nCut inputs: kept nodes subscribe, every publisher is excluded\n");
        for c in &sel.cut_inputs {
            let pubs = if c.publishers.len() > 3 {
                format!(
                    "{}, and {} more",
                    c.publishers[..3].join(", "),
                    c.publishers.len() - 3
                )
            } else {
                c.publishers.join(", ")
            };
            out.push_str(&format!(
                "  {}{}  {} kept subscriber(s); published by {pubs}\n",
                c.topic,
                if c.msg_type.is_empty() {
                    String::new()
                } else {
                    format!(" [{}]", c.msg_type)
                },
                c.subscribers.len()
            ));
        }
        out.push_str(&format!(
            "  To feed them from a bag:\n    ros2 bag play <bag> --topics {}\n",
            sel.cut_inputs
                .iter()
                .map(|c| c.topic.as_str())
                .collect::<Vec<_>>()
                .join(" ")
        ));
    }
    if !sel.warnings.is_empty() {
        out.push_str("\nWarnings\n");
        for w in &sel.warnings {
            out.push_str(&format!("  {w}\n"));
        }
    }
    out
}

#[cfg(test)]
mod tests;
