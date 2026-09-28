//! The four contract keys of a takeover scenario (rlm v0.1.46, phase 83):
//! `when:` (a fault by value), `window:` (a timed rung), `exit:` (the
//! driver's answer) and `entry_speed` + `settle: { decel, jerk }` (a settle
//! derived from the braking profile rather than assumed).
//!
//! The arithmetic that consumes them lives in `check_fault_reaction`; this
//! module holds what that function needs and the rules that stand on their
//! own:
//!
//! - `when-requires-reported`, `when-field-unknown` -- a predicate is only
//!   meaningful on a detector's output, and names a field that exists.
//! - `ladder-window-floor`, `window-param`, `window-unbound` -- a window is
//!   never the floor, and is the number the image runs.
//! - `mode-exit-target`, `mode-exit-unwired` -- an exit leaves the ladder, and
//!   something that implements the rung takes the input that ends it.
//! - `settle-derived`, `settle-entry-missing`, `settle-param-unresolved`,
//!   `settle-conflict` -- the settle a profile derives, and every reason it
//!   cannot.
//!
//! Every message starts with the contract file and line it is about, since
//! these rules run on the merged index where the per-manifest emitter's
//! source excerpts are not available.

use super::{
    launch_dump::LaunchDump,
    manifest_graph::GlobalDataflowGraph,
    manifest_loader::{ManifestIndex, ResolvedHazard, resolve_node_fqn},
    sched_loader::fqn_for,
};
use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::{
    EffectiveTrigger, FaultKind, NodeDecl, PredicateValue, SafeState, SettleProfile, SpanIndex,
    ValuePredicate,
};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    path::PathBuf,
};

// ---------------------------------------------------------------------------
// Where a finding is
// ---------------------------------------------------------------------------

/// `<contract file>:<line>` for a dotted contract path in `scope_id`'s
/// manifest. Falls back to the nearest enclosing key that the span index
/// knows, and to the bare file name when none is found.
pub(crate) fn site(index: &ManifestIndex, scope_id: usize, dotted: &str) -> String {
    let Some(m) = index.manifests.get(&scope_id) else {
        return String::new();
    };
    let file = m
        .contract_path
        .file_name()
        .map(|f| f.to_string_lossy().into_owned())
        .unwrap_or_default();
    let spans = SpanIndex::build(&m.source);
    let mut p = dotted.to_string();
    loop {
        if let Some(r) = spans.get(&p) {
            // The span index counts characters; the contracts it reads are
            // ASCII, and a line count is the same either way up to the key.
            let line = m
                .source
                .chars()
                .take(r.start)
                .filter(|c| *c == '\n')
                .count()
                + 1;
            return format!("{file}:{line}");
        }
        match p.rsplit_once('.') {
            Some((head, _)) => p = head.to_string(),
            None => return file,
        }
    }
}

fn diag(rule: &str, severity: Severity, message: String, path: String) -> Diagnostic {
    Diagnostic {
        rule_id: rule.to_string(),
        severity,
        message,
        path,
        span: None,
    }
}

// ---------------------------------------------------------------------------
// Launch parameter values
// ---------------------------------------------------------------------------

/// One parameter value as the launch tree gives it to a node.
#[derive(Debug, Clone, PartialEq)]
pub struct LaunchParam {
    /// The value as written.
    pub text: String,
    /// The value as a number, when it is one (a bool is not).
    pub number: Option<f64>,
}

impl LaunchParam {
    fn from_text(text: &str) -> Self {
        LaunchParam {
            text: text.to_string(),
            number: text.trim().parse::<f64>().ok().filter(|v| v.is_finite()),
        }
    }

    fn from_value(v: &ros_launch_manifest_model::ParamValue) -> Self {
        use ros_launch_manifest_model::ParamValue as V;
        match v {
            V::Int(i) => LaunchParam {
                text: i.to_string(),
                number: Some(*i as f64),
            },
            V::Float(f) => LaunchParam {
                text: f.to_string(),
                number: Some(*f),
            },
            V::Bool(b) => LaunchParam {
                text: b.to_string(),
                number: None,
            },
            V::Str(s) => LaunchParam::from_text(s),
            V::StrList(l) => LaunchParam {
                text: format!("[{}]", l.join(", ")),
                number: None,
            },
        }
    }
}

/// Every node's resolved launch parameter values, keyed by node FQN, in ROS's
/// precedence: the ordered source list where the record carries one (globals
/// first), otherwise parameter files then inline values; a later source
/// wins. File sections are matched by the manifest crate's own
/// `param_file_values`, so this cannot disagree with what a spawn applies.
pub(crate) fn launch_param_values(
    dump: &LaunchDump,
) -> HashMap<String, BTreeMap<String, LaunchParam>> {
    use super::launch_dump::ParamSource;
    let mut out: HashMap<String, BTreeMap<String, LaunchParam>> = HashMap::new();
    for n in &dump.node {
        let Some(bare) = n.name.clone().or_else(|| n.exec_name.clone()) else {
            continue;
        };
        if bare.is_empty() {
            continue;
        }
        let fqn = fqn_for(dump, n.namespace.as_deref(), &bare, n.scope);
        let mut values: BTreeMap<String, LaunchParam> = BTreeMap::new();
        let file = |content: &str, values: &mut BTreeMap<String, LaunchParam>| {
            for (k, v) in ros_launch_manifest_model::param_file_values(content, &fqn) {
                values.insert(k, LaunchParam::from_value(&v));
            }
        };
        for (k, v) in n.global_params.as_deref().unwrap_or(&[]) {
            values.insert(k.clone(), LaunchParam::from_text(v));
        }
        if n.param_sources.is_empty() {
            for content in &n.params_files {
                file(content, &mut values);
            }
            for (k, v) in &n.params {
                values.insert(k.clone(), LaunchParam::from_text(v));
            }
        } else {
            for src in &n.param_sources {
                match src {
                    ParamSource::Inline { name, value } => {
                        values.insert(name.clone(), LaunchParam::from_text(value));
                    }
                    ParamSource::File { content } => file(content, &mut values),
                }
            }
        }
        out.insert(fqn, values);
    }
    for c in &dump.load_node {
        if c.node_name.is_empty() {
            continue;
        }
        let fqn = fqn_for(dump, Some(c.namespace.as_str()), &c.node_name, c.scope);
        let values = c
            .params
            .iter()
            .map(|(k, v)| (k.clone(), LaunchParam::from_text(v)))
            .collect();
        out.insert(fqn, values);
    }
    out
}

/// The contract declaration of the node whose launch FQN is `fqn`, with the
/// scope it is declared in and its contract name.
pub(crate) fn contract_node<'a>(
    index: &'a ManifestIndex,
    fqn: &str,
) -> Option<(usize, &'a str, &'a NodeDecl)> {
    let mut scopes: Vec<_> = index.manifests.values().collect();
    scopes.sort_by_key(|m| m.scope_id);
    scopes.into_iter().find_map(|m| {
        m.manifest.nodes.iter().find_map(|(name, decl)| {
            (resolve_node_fqn(index, m.scope_id, &m.ns, name) == fqn).then_some((
                m.scope_id,
                name.as_str(),
                decl,
            ))
        })
    })
}

// ---------------------------------------------------------------------------
// The message definitions, through the ament index
// ---------------------------------------------------------------------------

const PRIMITIVES: &[&str] = &[
    "bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32",
    "uint32", "int64", "uint64", "string", "wstring",
];

/// A `.msg` file, read for what a predicate needs: field names and types,
/// and the constants a value may name.
#[derive(Debug, Default)]
struct MsgDef {
    /// (type as written, name)
    fields: Vec<(String, String)>,
    /// (name, value as written)
    constants: Vec<(String, String)>,
}

fn parse_msg(text: &str) -> MsgDef {
    let mut def = MsgDef::default();
    for raw in text.lines() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let mut parts = line.splitn(2, char::is_whitespace);
        let (Some(ty), Some(rest)) = (parts.next(), parts.next()) else {
            continue;
        };
        let rest = rest.trim();
        if let Some((name, value)) = rest.split_once('=') {
            def.constants
                .push((name.trim().to_string(), value.trim().to_string()));
        } else {
            let name = rest.split_whitespace().next().unwrap_or("").to_string();
            def.fields.push((ty.to_string(), name));
        }
    }
    def
}

/// `pkg/msg/Name`, `pkg/Name` -> (`pkg`, `Name`).
fn split_type(ty: &str) -> Option<(String, String)> {
    let parts: Vec<&str> = ty.split('/').collect();
    match parts.as_slice() {
        [pkg, "msg", name] | [pkg, name] if !pkg.is_empty() && !name.is_empty() => {
            Some((pkg.to_string(), name.to_string()))
        }
        _ => None,
    }
}

/// The `.msg` for `pkg/Name` on `AMENT_PREFIX_PATH`, first prefix wins.
fn find_msg(pkg: &str, name: &str) -> Option<PathBuf> {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").ok()?;
    prefixes
        .split(':')
        .filter(|p| !p.is_empty())
        .map(|p| {
            PathBuf::from(p)
                .join("share")
                .join(pkg)
                .join("msg")
                .join(format!("{name}.msg"))
        })
        .find(|p| p.is_file())
}

/// What a dotted field path is in a message type.
enum FieldLookup {
    /// A scalar field. `owner` is the message declaring it, whose constants
    /// a value may name.
    Scalar {
        ty: String,
        owner: String,
        constants: Vec<(String, String)>,
    },
    /// The field exists and is not a scalar (a message or an array).
    NotScalar { ty: String },
    /// No such field; the fields the message does have.
    Unknown { owner: String, fields: Vec<String> },
    /// A message definition on the way could not be found.
    Unresolved { ty: String },
}

fn lookup_field(msg_type: &str, dotted: &str) -> FieldLookup {
    let Some((mut pkg, mut name)) = split_type(msg_type) else {
        return FieldLookup::Unresolved {
            ty: msg_type.to_string(),
        };
    };
    let segments: Vec<&str> = dotted.split('.').collect();
    for (i, seg) in segments.iter().enumerate() {
        let owner = format!("{pkg}/msg/{name}");
        let Some(path) = find_msg(&pkg, &name) else {
            return FieldLookup::Unresolved { ty: owner };
        };
        let Ok(text) = std::fs::read_to_string(&path) else {
            return FieldLookup::Unresolved { ty: owner };
        };
        let def = parse_msg(&text);
        let Some((ty, _)) = def.fields.iter().find(|(_, n)| n == seg) else {
            return FieldLookup::Unknown {
                owner,
                fields: def.fields.iter().map(|(_, n)| n.clone()).collect(),
            };
        };
        let is_array = ty.ends_with(']');
        let base = ty.split('[').next().unwrap_or(ty);
        let base = base.split("<=").next().unwrap_or(base);
        let primitive = PRIMITIVES.contains(&base);
        if i + 1 == segments.len() {
            return if primitive && !is_array {
                FieldLookup::Scalar {
                    ty: ty.clone(),
                    owner,
                    constants: def.constants,
                }
            } else {
                FieldLookup::NotScalar { ty: ty.clone() }
            };
        }
        if primitive || is_array {
            return FieldLookup::NotScalar { ty: ty.clone() };
        }
        // Descend: `pkg/Type`, `Header` (std_msgs), or a bare type in the
        // same package.
        (pkg, name) = match split_type(base) {
            Some(pn) => pn,
            None if base == "Header" => ("std_msgs".to_string(), "Header".to_string()),
            None => (pkg, base.to_string()),
        };
    }
    FieldLookup::Unresolved {
        ty: msg_type.to_string(),
    }
}

// ---------------------------------------------------------------------------
// when-requires-reported, when-field-unknown
// ---------------------------------------------------------------------------

fn check_predicate_field(
    index: &ManifestIndex,
    scope_id: usize,
    owner: &str,
    at: &str,
    topic: &str,
    pred: &ValuePredicate,
    diags: &mut Vec<Diagnostic>,
) {
    let where_ = site(index, scope_id, at);
    let Some(msg_type) = index
        .topics
        .get(topic)
        .map(|t| t.msg_type.clone())
        .filter(|t| !t.is_empty())
    else {
        diags.push(diag(
            "when-field-unknown",
            Severity::Warning,
            format!(
                "{where_}: {owner} reads field '{}' of '{topic}', whose message type no \
                 contract in the tree declares -- the field is UNCHECKED. Declare \
                 `topics.{topic}.type`",
                pred.field
            ),
            at.to_string(),
        ));
        return;
    };
    let lookup = lookup_field(&msg_type, &pred.field);
    let found = match lookup {
        FieldLookup::Scalar {
            ty,
            owner: msg,
            constants,
        } => {
            if let PredicateValue::Constant(c) = &pred.value
                && !constants.iter().any(|(n, _)| n == c)
            {
                let known: Vec<String> =
                    constants.iter().map(|(n, v)| format!("{n}={v}")).collect();
                Some(format!(
                    "{where_}: {owner} compares '{}' ({ty}) with '{c}', which {msg} does not \
                     declare as a constant (it declares {})",
                    pred.field,
                    if known.is_empty() {
                        "none".to_string()
                    } else {
                        known.join(", ")
                    }
                ))
            } else {
                None
            }
        }
        FieldLookup::NotScalar { ty } => Some(format!(
            "{where_}: {owner} reads field '{}' of {msg_type}, which is `{ty}` -- not a scalar. \
             A predicate compares one bool, number or string; name the field inside it",
            pred.field
        )),
        FieldLookup::Unknown { owner: msg, fields } => Some(format!(
            "{where_}: {owner} reads field '{}' of '{topic}', and {msg} has no such field (it \
             has {})",
            pred.field,
            fields.join(", ")
        )),
        FieldLookup::Unresolved { ty } => {
            diags.push(diag(
                "when-field-unknown",
                Severity::Warning,
                format!(
                    "{where_}: {owner} reads field '{}' of {msg_type}, but {} is not on the \
                     ament index (AMENT_PREFIX_PATH) -- the field is UNCHECKED. Source the \
                     workspace that provides it",
                    pred.field,
                    if ty == msg_type {
                        "its definition".to_string()
                    } else {
                        format!("the nested {ty}")
                    }
                ),
                at.to_string(),
            ));
            None
        }
    };
    if let Some(message) = found {
        diags.push(diag(
            "when-field-unknown",
            Severity::Error,
            message,
            at.to_string(),
        ));
    }
}

/// `when-requires-reported` and `when-field-unknown`, over every hazard and
/// function that carries a predicate.
pub(crate) fn check_predicates(index: &ManifestIndex, diags: &mut Vec<Diagnostic>) {
    for h in &index.hazards {
        let Some(pred) = &h.decl.when else {
            continue;
        };
        let at = format!("hazards.{}.when", h.name);
        if !h.decl.on.contains(&FaultKind::Reported) {
            let on = if h.decl.on.is_empty() {
                "`on:` omitted, which means omission, late and loss".to_string()
            } else {
                let names: Vec<&str> = h
                    .decl
                    .on
                    .iter()
                    .map(|k| match k {
                        FaultKind::Omission => "omission",
                        FaultKind::Late => "late",
                        FaultKind::Loss => "loss",
                        FaultKind::Reported => "reported",
                    })
                    .collect();
                format!("`on: [{}]`", names.join(", "))
            };
            diags.push(diag(
                "when-requires-reported",
                Severity::Error,
                format!(
                    "{}: hazard '{}' names a value fault ({} {} {}) with {on}. A value is seen \
                     only by a node that reads the payload, and that node's output is a \
                     `reported` fault -- add `reported` to `on:`, or drop `when:`",
                    site(index, h.scope_id, &at),
                    h.name,
                    pred.field,
                    pred.op.symbol(),
                    pred.value
                ),
                at.clone(),
            ));
        }
        for g in &h.guards {
            for member in &g.members {
                check_predicate_field(
                    index,
                    h.scope_id,
                    &format!("hazard '{}'", h.name),
                    &at,
                    member,
                    pred,
                    diags,
                );
            }
        }
    }
    for f in &index.functions {
        let Some(pred) = &f.when else {
            continue;
        };
        let at = format!("functions.{}.when", f.name);
        for member in &f.group.members {
            check_predicate_field(
                index,
                f.scope_id,
                &format!("function '{}'", f.name),
                &at,
                member,
                pred,
                diags,
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Ladder selection per hazard (F1)
// ---------------------------------------------------------------------------

/// The functions and bare topics a hazard's fault takes away, by name, in its
/// scope (phase 83 F1).
///
/// A function is removed when one of its topics is a guard of the hazard (or
/// the hazard guards the function by name) AND the fault's classes reach its
/// kind: a `reported` fault removes the VALUE functions on its topic, an
/// omission, late or loss fault removes both kinds, because silence also
/// means "not known to be in range". An omitted `on:` is the three silence
/// classes, as everywhere else.
pub(crate) fn removed_by(h: &ResolvedHazard, index: &ManifestIndex) -> BTreeSet<String> {
    let silence = h.decl.on.is_empty()
        || h.decl
            .on
            .iter()
            .any(|k| matches!(k, FaultKind::Omission | FaultKind::Late | FaultKind::Loss));
    let reported = h.decl.on.contains(&FaultKind::Reported);
    let guarded: BTreeSet<&str> = h
        .guards
        .iter()
        .flat_map(|g| g.members.iter().map(String::as_str))
        .collect();
    let named: BTreeSet<&str> = h
        .decl
        .guards
        .iter()
        .flat_map(|g| g.members.iter().map(String::as_str))
        .collect();
    let mut out = BTreeSet::new();
    for f in index.functions.iter().filter(|f| f.scope_id == h.scope_id) {
        let touches = named.contains(f.name.as_str())
            || f.group.members.iter().any(|m| guarded.contains(m.as_str()));
        let kind_reached = if f.when.is_some() {
            silence || reported
        } else {
            silence
        };
        if touches && kind_reached {
            out.insert(f.name.clone());
        }
    }
    // A bare topic named in `requires:` is a function of one, lost by silence.
    if silence {
        for t in guarded.iter().chain(named.iter()) {
            out.insert((*t).to_string());
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Windows and exits
// ---------------------------------------------------------------------------

/// `ladder-window-floor`, `window-param`, `window-unbound`,
/// `mode-exit-target`, `mode-exit-unwired`.
pub(crate) fn check_windows_and_exits(
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
    params: &HashMap<String, BTreeMap<String, LaunchParam>>,
    diags: &mut Vec<Diagnostic>,
) {
    for m in &index.modes {
        let at = format!("modes.{}", m.name);
        let in_scope = |name: &str| {
            index
                .modes
                .iter()
                .find(|x| x.scope_id == m.scope_id && x.name == name)
        };
        // The floor of THIS mode's ladder.
        if let Some(last) = m.decl.fallback.last()
            && let Some(floor) = in_scope(last)
            && let Some(w) = &floor.decl.window
        {
            diags.push(diag(
                "ladder-window-floor",
                Severity::Error,
                format!(
                    "{}: mode '{}' falls to '{last}' last, and '{last}' has a {:.0}ms window -- \
                     a floor you leave after a timeout is not a floor. The bottom of a ladder \
                     must be a safe state the system can stay in",
                    site(index, floor.scope_id, &format!("modes.{last}.window")),
                    m.name,
                    w.duration.as_millis_f64()
                ),
                format!("{at}.fallback"),
            ));
        }
        if let Some(w) = &m.decl.window {
            let wat = format!("{at}.window");
            let where_ = site(index, m.scope_id, &wat);
            let ms = w.duration.as_millis_f64();
            match &w.param {
                None => diags.push(diag(
                    "window-unbound",
                    Severity::Warning,
                    format!(
                        "{where_}: mode '{}' waits {ms:.0}ms, but the window names no `param:` \
                         -- nothing ties the number to what the image runs. Write \
                         `window: {{ duration: {}, param: <node>.<parameter> }}`",
                        m.name, w.duration
                    ),
                    wat.clone(),
                )),
                Some(p) => {
                    let ns = index
                        .manifests
                        .get(&m.scope_id)
                        .map(|r| r.ns.clone())
                        .unwrap_or_default();
                    let fqn = resolve_node_fqn(index, m.scope_id, &ns, &p.node);
                    let declared = contract_node(index, &fqn)
                        .and_then(|(_, _, d)| d.params.as_ref())
                        .is_some_and(|ps| ps.contains_key(&p.name));
                    let value = params.get(&fqn).and_then(|v| v.get(&p.name));
                    let problem = match value {
                        None => Some(format!(
                            "has no resolved launch value for {fqn}{}",
                            if declared {
                                String::new()
                            } else {
                                format!(", and '{}' does not declare it under `params:`", p.node)
                            }
                        )),
                        Some(v) => match v.number {
                            None => Some(format!(
                                "resolves to `{}`, which is not a number of seconds",
                                v.text
                            )),
                            Some(s) if ((s * 1000.0) - ms).abs() > 1e-6 * ms.max(1.0) => {
                                Some(format!(
                                    "resolves to {} s = {:.2}ms, not the {ms:.2}ms the window \
                                     declares",
                                    v.text,
                                    s * 1000.0
                                ))
                            }
                            Some(_) => None,
                        },
                    };
                    if let Some(problem) = problem {
                        diags.push(diag(
                            "window-param",
                            Severity::Error,
                            format!(
                                "{where_}: mode '{}' waits {ms:.2}ms, bound to `{p}`, which \
                                 {problem}. The contract and the image must read one number",
                                m.name
                            ),
                            wat.clone(),
                        ));
                    }
                }
            }
        }
        let Some(exit) = &m.decl.exit else {
            continue;
        };
        let eat = format!("{at}.exit");
        let where_ = site(index, m.scope_id, &eat);
        // mode-exit-target
        match in_scope(&exit.to) {
            None => diags.push(diag(
                "mode-exit-target",
                Severity::Error,
                format!(
                    "{where_}: mode '{}' exits to '{}', which is not a mode in its scope",
                    m.name, exit.to
                ),
                format!("{eat}.to"),
            )),
            Some(_) => {
                let ladders: Vec<&str> = index
                    .modes
                    .iter()
                    .filter(|x| x.scope_id == m.scope_id && x.decl.fallback.contains(&exit.to))
                    .map(|x| x.name.as_str())
                    .collect();
                if !ladders.is_empty() {
                    diags.push(diag(
                        "mode-exit-target",
                        Severity::Error,
                        format!(
                            "{where_}: mode '{}' exits to '{}', which is a rung of the \
                             `fallback:` ladder of {} -- an exit ends the ladder, it is not a \
                             step down it",
                            m.name,
                            exit.to,
                            ladders
                                .iter()
                                .map(|l| format!("'{l}'"))
                                .collect::<Vec<_>>()
                                .join(", ")
                        ),
                        format!("{eat}.to"),
                    ));
                }
            }
        }
        // mode-exit-unwired
        let Some(f) = index
            .functions
            .iter()
            .find(|f| f.scope_id == m.scope_id && f.name == exit.on)
        else {
            diags.push(diag(
                "mode-exit-unwired",
                Severity::Error,
                format!(
                    "{where_}: mode '{}' exits on '{}', which is not a function in its scope",
                    m.name, exit.on
                ),
                format!("{eat}.on"),
            ));
            continue;
        };
        let Some(rp) = m.decl.reaction.as_ref().and_then(|r| {
            index
                .scope_paths
                .iter()
                .find(|p| p.scope_id == m.scope_id && &p.path_name == r)
        }) else {
            diags.push(diag(
                "mode-exit-unwired",
                Severity::Error,
                format!(
                    "{where_}: mode '{}' has an exit but no `reaction:` scope path, so no node \
                     implements the rung and nothing can take the input that ends it",
                    m.name
                ),
                format!("{eat}.on"),
            ));
            continue;
        };
        // The nodes that implement the rung: the publishers of its output.
        let implementers: BTreeSet<String> = rp
            .output_topics
            .iter()
            .filter_map(|t| index.topics.get(t))
            .flat_map(|t| t.publishers.iter())
            .filter_map(|p| p.rsplit_once('/').map(|(n, _)| n.to_string()))
            .collect();
        let wired = f.group.members.iter().any(|topic| {
            index.topics.get(topic).is_some_and(|t| {
                t.subscribers.iter().any(|s| {
                    let Some((node, ep)) = s.rsplit_once('/') else {
                        return false;
                    };
                    implementers.contains(node)
                        && graph.nodes.get(node).is_some_and(|n| {
                            n.paths.values().any(|p| {
                                matches!(p.effective_trigger(),
                                    EffectiveTrigger::Input(ref eps) if eps.iter().any(|e| e == ep))
                            })
                        })
                })
            })
        });
        if !wired {
            diags.push(diag(
                "mode-exit-unwired",
                Severity::Error,
                format!(
                    "{where_}: mode '{}' exits on '{}' ({}), but no node that implements the \
                     rung ({}) takes it on a path trigger -- without a trigger there is no take \
                     to trace and no evidence the exit was seen",
                    m.name,
                    exit.on,
                    f.group.members.join(", "),
                    if implementers.is_empty() {
                        "none publishes its output".to_string()
                    } else {
                        implementers.iter().cloned().collect::<Vec<_>>().join(", ")
                    }
                ),
                format!("{eat}.on"),
            ));
        }
    }
}

// ---------------------------------------------------------------------------
// The derived settle
// ---------------------------------------------------------------------------

/// What a rung's safe state contributes to its reaction time, and how the
/// number was reached.
#[derive(Debug, Clone, Default)]
pub struct Settle {
    pub ms: Option<f64>,
    /// `derived` | `literal` | `none`, for the budget table.
    pub how: &'static str,
}

/// The settle a safe state contributes for hazard `h`, with the
/// `settle-*` findings. `node_fqn` is the node whose path declares `ss`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn settle_for(
    index: &ManifestIndex,
    params: &HashMap<String, BTreeMap<String, LaunchParam>>,
    h: &ResolvedHazard,
    rung: &str,
    node_fqn: &str,
    path_name: &str,
    ss: &SafeState,
    diags: &mut Vec<Diagnostic>,
) -> Settle {
    let literal = ss.settle.map(|d| d.as_millis_f64());
    let literal_settle = Settle {
        ms: literal,
        how: if literal.is_some() { "literal" } else { "none" },
    };
    let Some(profile) = &ss.settle_profile else {
        return literal_settle;
    };
    let (scope_id, node_name, decl) = match contract_node(index, node_fqn) {
        Some(n) => n,
        None => return literal_settle,
    };
    let at = format!("nodes.{node_name}.paths.{path_name}.safe_state.settle");
    let where_ = site(index, scope_id, &at);
    let read = |name: &str| -> Result<(f64, String), String> {
        let declared = decl.params.as_ref().is_some_and(|p| p.contains_key(name));
        if !declared {
            return Err(format!(
                "'{name}' is not declared under `nodes.{node_name}.params`"
            ));
        }
        match params.get(node_fqn).and_then(|v| v.get(name)) {
            None => Err(format!(
                "'{name}' has no resolved launch value for {node_fqn}"
            )),
            Some(LaunchParam { number: None, text }) => {
                Err(format!("'{name}' resolves to `{text}`, not a number"))
            }
            Some(LaunchParam {
                number: Some(v),
                text,
            }) => Ok((*v, text.clone())),
        }
    };
    let (decel, jerk) = match (read(&profile.decel), read(&profile.jerk)) {
        (Ok(a), Ok(j)) => (a, j),
        (a, j) => {
            let why: Vec<String> = [a.err(), j.err()].into_iter().flatten().collect();
            diags.push(diag(
                "settle-param-unresolved",
                Severity::Error,
                format!(
                    "{where_}: the braking profile of {node_fqn}/{path_name} cannot be read: {}. \
                     The settle is the parameter the image runs, or it is not derived at all",
                    why.join("; ")
                ),
                at.clone(),
            ));
            return literal_settle;
        }
    };
    let Some(v0) = h.decl.entry_speed else {
        diags.push(diag(
            "settle-entry-missing",
            Severity::Warning,
            format!(
                "{}: hazard '{}' reaches rung '{rung}', whose settle is a braking profile \
                 ({node_fqn}: {} = {}, {} = {}), but the hazard states no `entry_speed` -- \
                 {}",
                site(index, h.scope_id, &format!("hazards.{}", h.name)),
                h.name,
                profile.decel,
                decel.1,
                profile.jerk,
                jerk.1,
                match literal {
                    Some(l) => format!("falling back to the literal settle {l:.2}ms"),
                    None => "and there is no literal settle to fall back to".to_string(),
                }
            ),
            format!("hazards.{}.entry_speed", h.name),
        ));
        return literal_settle;
    };
    let (a, j) = (decel.0.abs(), jerk.0.abs());
    let Some(t) = SettleProfile::settle_s(v0, a, j) else {
        diags.push(diag(
            "settle-param-unresolved",
            Severity::Error,
            format!(
                "{where_}: the braking profile of {node_fqn}/{path_name} is degenerate \
                 ({} = {}, {} = {}): a zero deceleration or jerk never stops the vehicle",
                profile.decel, decel.1, profile.jerk, jerk.1
            ),
            at.clone(),
        ));
        return literal_settle;
    };
    let ms = t * 1000.0;
    let v_r = a * a / (2.0 * j);
    let arithmetic = if v0 <= v_r {
        format!(
            "v_r = a^2/(2j) = {a}^2/(2*{j}) = {v_r:.4} m/s; v0 = {v0} <= v_r, so t = sqrt(2 v0/j) \
             = sqrt(2*{v0}/{j}) = {ms:.2}ms"
        )
    } else {
        format!(
            "v_r = a^2/(2j) = {a}^2/(2*{j}) = {v_r:.4} m/s; v0 = {v0} > v_r, so t = a/j + \
             (v0 - v_r)/a = {a}/{j} + ({v0} - {v_r:.4})/{a} = {:.2} + {:.2} = {ms:.2}ms",
            a / j * 1000.0,
            (v0 - v_r) / a * 1000.0
        )
    };
    diags.push(diag(
        "settle-derived",
        Severity::Info,
        format!(
            "{where_}: hazard '{}', rung '{rung}': settle from {node_fqn}'s braking profile, \
             a = |{}| = {a} m/s^2, j = |{}| = {j} m/s^3, v0 = entry_speed {v0} m/s: {arithmetic}",
            h.name, profile.decel, profile.jerk
        ),
        at.clone(),
    ));
    if let Some(l) = literal
        && (l - ms).abs() > 0.01 * ms
    {
        diags.push(diag(
            "settle-conflict",
            Severity::Warning,
            format!(
                "{where_}: hazard '{}', rung '{rung}': the literal settle {l:.2}ms and the \
                 profile's {ms:.2}ms differ by {:.1}% -- one of them describes a different \
                 vehicle or a different speed. The derived value is charged",
                h.name,
                (l - ms).abs() / ms * 100.0
            ),
            at.clone(),
        ));
    }
    Settle {
        ms: Some(ms),
        how: "derived",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `.msg` is read for fields and constants; comments and defaults do
    /// not become fields, and a constant is never mistaken for one.
    #[test]
    fn a_msg_file_yields_its_fields_and_constants() {
        let def = parse_msg(
            "# header comment\nuint8 NO_COMMAND = 0\nuint8 MANUAL=4 # trailing\n\n\
             builtin_interfaces/Time stamp\nuint8 mode\nfloat64 limit 3.0\nstring<=8 tag\n\
             int32[] samples\n",
        );
        let fields: Vec<&str> = def.fields.iter().map(|(_, n)| n.as_str()).collect();
        assert_eq!(fields, ["stamp", "mode", "limit", "tag", "samples"]);
        let constants: Vec<(&str, &str)> = def
            .constants
            .iter()
            .map(|(n, v)| (n.as_str(), v.as_str()))
            .collect();
        assert_eq!(constants, [("NO_COMMAND", "0"), ("MANUAL", "4")]);
    }

    #[test]
    fn a_type_splits_in_both_spellings_and_nothing_else() {
        assert_eq!(
            split_type("autoware_vehicle_msgs/msg/ControlModeReport"),
            Some(("autoware_vehicle_msgs".into(), "ControlModeReport".into()))
        );
        assert_eq!(
            split_type("std_msgs/Header"),
            Some(("std_msgs".into(), "Header".into()))
        );
        assert_eq!(split_type("Header"), None);
        assert_eq!(split_type("a/srv/B/c"), None);
    }

    /// A launch value is a number only when it parses as one; a bool is not.
    #[test]
    fn a_launch_value_is_a_number_only_when_it_is_one() {
        assert_eq!(LaunchParam::from_text("10.0").number, Some(10.0));
        assert_eq!(LaunchParam::from_text(" -2.5 ").number, Some(-2.5));
        assert_eq!(LaunchParam::from_text("true").number, None);
        assert_eq!(LaunchParam::from_text("fast").number, None);
        use ros_launch_manifest_model::ParamValue as V;
        assert_eq!(LaunchParam::from_value(&V::Int(30)).number, Some(30.0));
        assert_eq!(LaunchParam::from_value(&V::Bool(true)).number, None);
    }

    /// F1: which functions a fault removes. `reported` takes the value
    /// function on its topic and leaves the plain one; silence takes both;
    /// a function on another topic is never touched.
    #[test]
    fn a_fault_removes_functions_by_class() {
        use super::super::manifest_loader::ResolvedFunction;
        use ros_launch_manifest_types::{
            CompareOp, FaultKind as F, GuardGroup, HazardDecl, PredicateValue,
        };
        let group = |t: &str| GuardGroup {
            members: vec![t.to_string()],
            all_of: false,
        };
        let pred = Some(ValuePredicate {
            field: "autonomous".into(),
            op: CompareOp::Equals,
            value: PredicateValue::Bool(false),
        });
        let mut index = ManifestIndex::default();
        for (name, topic, when) in [
            ("alive", "/avail", None),
            ("in_odd", "/avail", pred.clone()),
            ("other", "/elsewhere", pred),
        ] {
            index.functions.push(ResolvedFunction {
                scope_id: 0,
                name: name.into(),
                group: group(topic),
                when,
            });
        }
        let hazard = |on: Vec<F>| ResolvedHazard {
            scope_id: 0,
            name: "h".into(),
            guards: vec![group("/avail")],
            decl: HazardDecl {
                guards: vec![group("/avail")],
                on,
                ..Default::default()
            },
        };
        let names = |on| -> Vec<String> {
            removed_by(&hazard(on), &index)
                .into_iter()
                .filter(|n| !n.starts_with('/'))
                .collect()
        };
        assert_eq!(names(vec![F::Reported]), ["in_odd"]);
        assert_eq!(names(vec![F::Omission]), ["alive", "in_odd"]);
        assert_eq!(
            names(vec![]),
            ["alive", "in_odd"],
            "omitted `on:` is silence"
        );
        assert_eq!(names(vec![F::Reported, F::Late]), ["alive", "in_odd"]);
    }
}
