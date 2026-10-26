//! Contract values read from the launch tree (phase 94; rlm
//! `docs/design/contract-values.md`).
//!
//! A contract quantity may read a number instead of restating it:
//! `$(param [<node>] <name>) <unit>`, `$(var <name> [@<node>]) <unit>`,
//! `$(let <name>)`. The contract parser keeps each such value as an
//! expression; this module is the launch tree it is evaluated against, per
//! scope instance, BEFORE any rule reads the contract. The contract is then
//! parsed again with the numbers in place, so no rule, model field or
//! downstream reader sees an expression, and the per-mode re-runs (phase
//! 75), which clone the resolved index, see the same numbers.
//!
//! The rules, each reported here with the contract file and line:
//!
//! - `param-unset`: no launch source sets the parameter and the contract
//!   declares no `default:` -- the node runs its compiled-in default, which
//!   no contract can see;
//! - `param-node-unknown`: the node is not in the launch tree (near misses
//!   named);
//! - `param-node-missing`: a one-argument `$(param)` under no node;
//! - `param-not-number`: the value is a string, a list or a boolean;
//! - `param-dimension`: the value's dimension is not the key's (rlm);
//! - `var-unset`: no launch file sets the configuration;
//! - `var-ambiguous`: the contract's launch file assigned the configuration
//!   more than one value, so "the value" names none of them; `$(var x
//!   @<node>)` reads the one in force where the node was declared;
//! - `value-range`: a negative duration, a rate that is not positive, a
//!   count that is not whole (rlm).

use super::{
    launch_dump::{ConfigEvent, ConfigEventKind, LaunchDump, ParamOrigin, ScopeEntry},
    manifest_loader::{ManifestIndex, resolve_node_fqn},
};
use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::{
    Manifest, SpanIndex,
    parse::ParseError,
    values::{self, Lookup, ParamNode, Read, RefAt, Resolution},
};
use std::collections::HashMap;

/// The launch tree one scope instance of a contract reads.
pub(crate) struct ScopeLookup<'a> {
    pub index: &'a ManifestIndex,
    pub scope: &'a ScopeEntry,
    /// The launch configurations at the end of the run (the record's
    /// `variables`).
    pub variables: &'a std::collections::HashMap<String, String>,
    /// The contract's first parse: its node keys and `params:` defaults.
    pub manifest: &'a Manifest,
    /// The contract's file name, for a `default:`'s provenance.
    pub contract_file: &'a str,
}

/// Up to three names within a small edit distance of `want`, nearest first.
fn near_misses<'a>(want: &str, candidates: impl Iterator<Item = &'a str>) -> Vec<String> {
    let budget = (want.len() / 3).max(2);
    let mut hits: Vec<(usize, &str)> = candidates
        .map(|c| (super::manifest_loader::edit_distance(want, c), c))
        .filter(|(d, _)| *d <= budget)
        .collect();
    hits.sort();
    hits.dedup();
    hits.into_iter()
        .take(3)
        .map(|(_, c)| c.to_string())
        .collect()
}

fn or_list(names: &[String]) -> String {
    names
        .iter()
        .map(|n| format!("`{n}`"))
        .collect::<Vec<_>>()
        .join(", ")
}

impl ScopeLookup<'_> {
    fn fqn_of(&self, key: &str) -> String {
        resolve_node_fqn(self.index, self.scope.id, &self.scope.ns, key)
    }

    /// The contract node key whose launch FQN is `fqn`, when there is one.
    fn key_of(&self, fqn: &str) -> Option<&str> {
        self.manifest
            .nodes
            .keys()
            .find(|k| self.fqn_of(k) == fqn)
            .map(String::as_str)
    }

    /// The parameter sources of `fqn`, in order, for a message.
    fn sources_of(&self, fqn: &str) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for lp in self
            .index
            .launch_params
            .get(fqn)
            .into_iter()
            .flat_map(|v| v.values())
        {
            if !lp.at.is_empty() && !out.contains(&lp.at) {
                out.push(lp.at.clone());
            }
        }
        out
    }
}

impl ScopeLookup<'_> {
    /// A `$(var)` read from the configuration log (phase 94 W2): the state
    /// where the contract's launch file finished, refused when that file
    /// assigned the name more than one value, or, with `@<node>`, the state
    /// where the node was declared.
    fn var_from_log(&self, at: &RefAt, name: &str, node: Option<&str>) -> Read {
        let log = &self.index.launch_config_log;
        let file = self.scope.file().unwrap_or("the launch file").to_string();
        let (state, place) = match node {
            Some(n) => {
                let fqn = if n.starts_with('/') {
                    n.to_string()
                } else if self.manifest.nodes.contains_key(n) {
                    self.fqn_of(n)
                } else {
                    let near = near_misses(n, self.manifest.nodes.keys().map(String::as_str));
                    return Read::Error {
                        code: "param-node-unknown",
                        message: format!(
                            "`{}`: `{n}` is not a node of this contract{}",
                            at.written,
                            if near.is_empty() {
                                String::new()
                            } else {
                                format!("; did you mean {}?", or_list(&near))
                            }
                        ),
                    };
                };
                let Some(&seq) = self.index.launch_config_seq.get(&fqn) else {
                    let near = near_misses(
                        &fqn,
                        self.index.launch_config_seq.keys().map(String::as_str),
                    );
                    return Read::Error {
                        code: "param-node-unknown",
                        message: format!(
                            "`{}`: the launch tree starts no node {fqn}{}",
                            at.written,
                            if near.is_empty() {
                                String::new()
                            } else {
                                format!("; did you mean {}?", or_list(&near))
                            }
                        ),
                    };
                };
                (config_state(log, seq), format!("where {fqn} was declared"))
            }
            None => {
                let (start, end) = self.scope.config_span.unwrap_or((0, log.len()));
                let assigned: Vec<&ConfigEvent> = log[start.min(log.len())..end.min(log.len())]
                    .iter()
                    .filter(|e| e.kind == ConfigEventKind::Set && e.name.as_deref() == Some(name))
                    .collect();
                let mut distinct: Vec<&ConfigEvent> = Vec::new();
                for e in &assigned {
                    if !distinct.iter().any(|d| same_value(d, e)) {
                        distinct.push(e);
                    }
                }
                if distinct.len() > 1 {
                    let listed: Vec<String> = assigned
                        .iter()
                        .map(|e| {
                            format!("{} ({})", e.value.as_deref().unwrap_or(""), assigned_at(e))
                        })
                        .collect();
                    let point = self
                        .manifest
                        .nodes
                        .keys()
                        .next()
                        .map(String::as_str)
                        .unwrap_or("<node>");
                    return Read::Error {
                        code: "var-ambiguous",
                        message: format!(
                            "`{}` was assigned {}. Name the point: `$(var {name} @{point})`.",
                            at.written,
                            and_list(&listed)
                        ),
                    };
                }
                (config_state(log, end), format!("where {file} finishes"))
            }
        };
        match state.get(name) {
            Some(ev) => {
                let text = ev.value.clone().unwrap_or_default();
                match text.trim().parse::<f64>() {
                    Ok(raw) if raw.is_finite() => Read::Value {
                        raw,
                        at: assigned_at(ev),
                        node: None,
                    },
                    _ => Read::Error {
                        code: "param-not-number",
                        message: format!(
                            "`{}` reads the launch configuration `{name}`, which is `{text}` \
                             ({}), not a number",
                            at.written,
                            assigned_at(ev)
                        ),
                    },
                }
            }
            None => {
                let mut names: Vec<&str> = state.keys().map(String::as_str).collect();
                names.sort_unstable();
                let near = near_misses(name, names.into_iter());
                // Assigned, but not in force here: inside a scoped group
                // that ended, or after the point named.
                let elsewhere: Vec<String> = log
                    .iter()
                    .filter(|e| e.kind == ConfigEventKind::Set && e.name.as_deref() == Some(name))
                    .map(assigned_at)
                    .collect();
                Read::Error {
                    code: "var-unset",
                    message: format!(
                        "`{}`: {}{}",
                        at.written,
                        if elsewhere.is_empty() {
                            format!("no launch file sets the configuration `{name}` {place}")
                        } else {
                            format!(
                                "no launch configuration `{name}` is in force {place} (it is \
                                 assigned at {}, out of scope here)",
                                elsewhere.join(", ")
                            )
                        },
                        if near.is_empty() {
                            String::new()
                        } else {
                            format!("; did you mean {}?", or_list(&near))
                        }
                    ),
                }
            }
        }
    }
}

/// `a`, `a and b`, `a, b and c`.
fn and_list(items: &[String]) -> String {
    match items {
        [] => String::new(),
        [one] => one.clone(),
        [head @ .., last] => format!("{} and {last}", head.join(", ")),
    }
}

/// Whether two assignments gave the same value: numerically when both are
/// numbers (`0.5` and `0.50`), else as text.
fn same_value(a: &ConfigEvent, b: &ConfigEvent) -> bool {
    let (x, y) = (
        a.value.as_deref().unwrap_or("").trim(),
        b.value.as_deref().unwrap_or("").trim(),
    );
    match (x.parse::<f64>(), y.parse::<f64>()) {
        (Ok(p), Ok(q)) => p == q,
        _ => x == y,
    }
}

/// The launch configurations in force after the first `end` events of
/// `log`, each with the event that set it. A scoped group's push saves
/// them and its pop puts them back; a reset clears them.
pub(crate) fn config_state(log: &[ConfigEvent], end: usize) -> HashMap<String, &ConfigEvent> {
    let mut state: HashMap<String, &ConfigEvent> = HashMap::new();
    let mut saved: Vec<HashMap<String, &ConfigEvent>> = Vec::new();
    for ev in &log[..end.min(log.len())] {
        match ev.kind {
            ConfigEventKind::Set => {
                if let Some(n) = &ev.name {
                    state.insert(n.clone(), ev);
                }
            }
            ConfigEventKind::Unset => {
                if let Some(n) = &ev.name {
                    state.remove(n);
                }
            }
            ConfigEventKind::Push => saved.push(state.clone()),
            ConfigEventKind::Pop => {
                if let Some(s) = saved.pop() {
                    state = s;
                }
            }
            ConfigEventKind::Reset => state.clear(),
        }
    }
    state
}

/// Where an assignment was made, for a message or `--explain`:
/// `command line`, `<let> bringup.launch.xml:7`, `<arg> default
/// bringup.launch.xml:3`, `include argument parent.launch.xml:12`,
/// `SetLaunchConfiguration bringup.launch.py` (the Python frontend records
/// no line).
pub(crate) fn assigned_at(ev: &ConfigEvent) -> String {
    // Named as the file spells it: a `.launch.py` has no `<let>`.
    let py = ev.file.as_deref().is_some_and(|f| f.ends_with(".py"));
    let what = match (ev.origin.as_deref(), py) {
        (Some("cli"), _) => return "command line".to_string(),
        (Some("arg"), false) => "<arg> default",
        (Some("arg"), true) => "DeclareLaunchArgument default",
        (Some("let"), false) => "<let>",
        (Some("let"), true) => "SetLaunchConfiguration",
        (Some("include"), _) => "include argument",
        (Some("group"), false) => "group",
        (Some("group"), true) => "GroupAction",
        _ => "set",
    };
    match (&ev.file, ev.line) {
        (Some(f), Some(l)) => format!("{what} {f}:{l}"),
        (Some(f), None) => format!("{what} {f}"),
        _ => what.to_string(),
    }
}

/// The names a substitution text reads with `$(var <name>)`, in order.
fn var_names(text: &str) -> Vec<&str> {
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(i) = rest.find("$(var ") {
        rest = &rest[i + 6..];
        let name = rest
            .split(|c: char| c.is_whitespace() || c == ')')
            .next()
            .unwrap_or("");
        if !name.is_empty() && !out.contains(&name) {
            out.push(name);
        }
    }
    out
}

/// What an inline `<param>` written as a substitution read (phase 94
/// W2.3): `<param> bringup.launch.xml:29 = $(var lidar_hz)`, then each
/// configuration it names as it stood where the node was declared.
pub(crate) fn param_chain(
    dump: &LaunchDump,
    origin: &ParamOrigin,
    seq: Option<usize>,
) -> Vec<String> {
    let site = match (&origin.file, origin.line) {
        (Some(f), Some(l)) => format!("{f}:{l}"),
        (Some(f), None) => f.clone(),
        _ => "the launch file".to_string(),
    };
    let mut out = vec![format!("<param> {site} = {}", origin.written)];
    if let Some(seq) = seq
        && !dump.config_log.is_empty()
    {
        let state = config_state(&dump.config_log, seq);
        for name in var_names(&origin.written) {
            if let Some(ev) = state.get(name) {
                out.push(format!(
                    "{name} = {}, {}",
                    ev.value.as_deref().unwrap_or(""),
                    assigned_at(ev)
                ));
            }
        }
    }
    out
}

impl Lookup for ScopeLookup<'_> {
    fn param(&self, at: &RefAt, node: &ParamNode, name: &str) -> Read {
        let (fqn, key) = match node {
            ParamNode::Here(None) => {
                let having: Vec<String> = self
                    .manifest
                    .nodes
                    .iter()
                    .filter(|(k, d)| {
                        d.params.as_ref().is_some_and(|p| p.contains_key(name))
                            || self
                                .index
                                .launch_params
                                .get(&self.fqn_of(k))
                                .is_some_and(|v| v.contains_key(name))
                    })
                    .map(|(k, _)| k.clone())
                    .collect();
                return Read::Error {
                    code: "param-node-missing",
                    message: format!(
                        "`{}` is under no node, so `$(param {name})` must name one: `$(param \
                         <node> {name})`. {}",
                        at.key_path(),
                        if having.is_empty() {
                            format!("No node of this contract has a `{name}` parameter.")
                        } else {
                            format!(
                                "Nodes of this contract with a `{name}` parameter: {}.",
                                having.join(", ")
                            )
                        }
                    ),
                };
            }
            ParamNode::Key(k) if !self.manifest.nodes.contains_key(k) => {
                let near = near_misses(k, self.manifest.nodes.keys().map(String::as_str));
                return Read::Error {
                    code: "param-node-unknown",
                    message: format!(
                        "`{}`: `{k}` is not a node of this contract{}",
                        at.written,
                        if near.is_empty() {
                            format!(
                                " (its nodes: {})",
                                self.manifest
                                    .nodes
                                    .keys()
                                    .cloned()
                                    .collect::<Vec<_>>()
                                    .join(", ")
                            )
                        } else {
                            format!("; did you mean {}?", or_list(&near))
                        }
                    ),
                };
            }
            ParamNode::Here(Some(k)) | ParamNode::Key(k) => (self.fqn_of(k), Some(k.clone())),
            ParamNode::Fqn(f) => (f.clone(), self.key_of(f).map(str::to_string)),
        };
        let Some(values) = self.index.launch_params.get(&fqn) else {
            let near = near_misses(&fqn, self.index.launch_params.keys().map(String::as_str));
            return Read::Error {
                code: "param-node-unknown",
                message: format!(
                    "`{}`: the launch tree starts no node {fqn}{}",
                    at.written,
                    if near.is_empty() {
                        String::new()
                    } else {
                        format!("; did you mean {}?", or_list(&near))
                    }
                ),
            };
        };
        match values.get(name) {
            Some(lp) => match lp.number {
                Some(raw) => Read::Value {
                    raw,
                    at: if lp.at.is_empty() {
                        "the launch tree".to_string()
                    } else {
                        lp.provenance()
                    },
                    node: Some(fqn),
                },
                None => Read::Error {
                    code: "param-not-number",
                    message: format!(
                        "`{}` reads `{name}` on {fqn}, which is `{}` ({}{}), not a number",
                        at.written,
                        lp.text,
                        lp.kind(),
                        if lp.at.is_empty() {
                            String::new()
                        } else {
                            format!(", {}", lp.at)
                        }
                    ),
                },
            },
            None => {
                let default = key
                    .as_deref()
                    .and_then(|k| self.manifest.nodes.get(k))
                    .and_then(|d| d.params.as_ref())
                    .and_then(|p| p.get(name))
                    .and_then(|p| p.default.as_deref());
                if let Some(d) = default {
                    return match d.trim().parse::<f64>() {
                        Ok(raw) => Read::Value {
                            raw,
                            at: format!(
                                "`params.{name}.default` in {} (no launch source sets it)",
                                self.contract_file
                            ),
                            node: Some(fqn),
                        },
                        Err(_) => Read::Error {
                            code: "param-not-number",
                            message: format!(
                                "`{}`: `{name}`'s `default: {d}` is not a number",
                                at.written
                            ),
                        },
                    };
                }
                let sources = self.sources_of(&fqn);
                Read::Error {
                    code: "param-unset",
                    message: format!(
                        "`{}`: no launch source sets `{name}` on {fqn}{}, so it runs the code's \
                         default, which a contract cannot see. Set it in the {}, or declare \
                         `params: {{ {name}: {{ type: double, default: … }} }}`.",
                        at.written,
                        if sources.is_empty() {
                            " (it has no parameter sources)".to_string()
                        } else {
                            format!(" (sources: {})", sources.join(", "))
                        },
                        if sources.iter().any(|s| s.ends_with(".yaml")) {
                            "parameter file"
                        } else {
                            "launch file"
                        }
                    ),
                }
            }
        }
    }

    fn var(&self, at: &RefAt, name: &str, node: Option<&str>) -> Read {
        if !self.index.launch_config_log.is_empty() {
            return self.var_from_log(at, name, node);
        }
        if let Some(n) = node {
            // Without the configuration log (the Python dumper records
            // none) the point a value is read at cannot be told apart from
            // the file's end.
            return Read::Error {
                code: "var-ambiguous",
                message: format!(
                    "`{}`: reading `{name}` where `{n}` was declared needs the launch \
                     configuration log, which the Python launch dumper does not record; \
                     use the default parser",
                    at.written
                ),
            };
        }
        // The value in force when the contract's launch file finished. An
        // include scope's table is that state (the parser records it after
        // the file has run); the root's holds the command line, and the
        // state at its end is the record's `variables`.
        let file = self.scope.file().unwrap_or("the launch file").to_string();
        let found = if self.scope.parent.is_none() {
            match self.scope.args.get(name) {
                Some(v) => Some((v.clone(), "command line".to_string())),
                None => self.variables.get(name).map(|v| (v.clone(), file.clone())),
            }
        } else {
            self.scope.args.get(name).map(|v| (v.clone(), file.clone()))
        };
        match found {
            Some((text, at_label)) => match text.trim().parse::<f64>() {
                Ok(raw) if raw.is_finite() => Read::Value {
                    raw,
                    at: at_label,
                    node: None,
                },
                _ => Read::Error {
                    code: "param-not-number",
                    message: format!(
                        "`{}` reads the launch configuration `{name}`, which is `{text}` \
                         ({at_label}), not a number",
                        at.written
                    ),
                },
            },
            None => {
                let mut names: Vec<&str> = self
                    .scope
                    .args
                    .keys()
                    .chain(self.variables.keys())
                    .map(String::as_str)
                    .collect();
                names.sort_unstable();
                names.dedup();
                let near = near_misses(name, names.into_iter());
                Read::Error {
                    code: "var-unset",
                    message: format!(
                        "`{}`: no launch file sets the configuration `{name}` where {file} \
                         finishes{}",
                        at.written,
                        if near.is_empty() {
                            String::new()
                        } else {
                            format!("; did you mean {}?", or_list(&near))
                        }
                    ),
                }
            }
        }
    }
}

/// `<file>:<line>` of a dotted path in a contract's source, falling back to
/// the nearest enclosing key.
pub(crate) fn line_site(file: &str, source: &str, spans: &SpanIndex, dotted: &str) -> String {
    let mut p = dotted.to_string();
    loop {
        if let Some(r) = spans.get(&p) {
            let line = source.chars().take(r.start).filter(|c| *c == '\n').count() + 1;
            return format!("{file}:{line}");
        }
        match p.rsplit_once('.') {
            Some((head, _)) => p = head.to_string(),
            None => return file.to_string(),
        }
    }
}

/// The instance tag of a diagnostic about a contract bound to more than one
/// scope: ` [<ns>]`. Empty when the contract has one instance.
pub(crate) fn instance_tag(instances: usize, ns: &str) -> String {
    if instances > 1 {
        format!(" [{ns}]")
    } else {
        String::new()
    }
}

/// Parse one scope instance of a contract with its values read from the
/// launch tree: the first parse collects the expressions, they are
/// evaluated against `scope`, and the contract is parsed again with the
/// numbers in place. The diagnostics are the evaluation's errors, each with
/// the file, line and (for a contract bound to several scopes) instance.
pub(crate) fn parse_scope_instance(
    source: &str,
    contract_file: &str,
    scope: &ScopeEntry,
    instances: usize,
    index: &ManifestIndex,
    dump: &LaunchDump,
) -> Result<
    (
        ros_launch_manifest_types::parse::ParseResult,
        Resolution,
        Vec<Diagnostic>,
    ),
    ParseError,
> {
    use ros_launch_manifest_types::parse::{ParseResult, parse_manifest_str_with_values};
    let first = parse_manifest_str_with_values(source, None)?;
    let spans = SpanIndex::build(source);
    if first.value_refs.is_empty() && first.lets.is_empty() {
        return Ok((
            ParseResult {
                manifest: first,
                source: source.to_string(),
                spans,
            },
            Resolution::default(),
            Vec::new(),
        ));
    }
    let lookup = ScopeLookup {
        index,
        scope,
        variables: &dump.variables,
        manifest: &first,
        contract_file,
    };
    let resolution = first.resolve_values(&lookup);
    let mut manifest = parse_manifest_str_with_values(source, Some(&resolution.values))?;
    for r in &mut manifest.value_refs {
        r.resolved = resolution.values.get(&r.path).cloned();
    }
    let tag = instance_tag(instances, &scope.ns);
    let diags = resolution
        .errors
        .iter()
        .map(|e| Diagnostic {
            rule_id: e.code.to_string(),
            severity: Severity::Error,
            message: format!(
                "{}{tag}: {}",
                line_site(contract_file, source, &spans, &e.path),
                e.message
            ),
            path: e.path.clone(),
            span: None,
        })
        .collect();
    Ok((
        ParseResult {
            manifest,
            source: source.to_string(),
            spans,
        },
        resolution,
        diags,
    ))
}

/// An extension key's value read from the launch tree (`decel: $(param
/// target_acceleration) m/s^2`; the caller spells the bare `decel:
/// target_acceleration` the same way), evaluated through the one path every
/// reference takes, against the contract of `scope_id`. `node` is the node
/// (key or FQN) the key sits under; `None` under a hazard.
pub(crate) fn ext_value(
    index: &ManifestIndex,
    scope_id: usize,
    text: &str,
    key: &str,
    want: rlx::dim::Dim,
    node: Option<&str>,
) -> Result<Option<values::Resolved>, values::ValueError> {
    let Some(m) = index.manifests.get(&scope_id) else {
        return Ok(None);
    };
    let Some(scope) = index.launch_scopes.get(&scope_id) else {
        return Ok(None);
    };
    let file = m
        .contract_path
        .file_name()
        .map(|f| f.to_string_lossy().into_owned())
        .unwrap_or_default();
    let lookup = ScopeLookup {
        index,
        scope,
        variables: &index.launch_variables,
        manifest: &m.manifest,
        contract_file: &file,
    };
    m.manifest.evaluate_text(text, key, want, node, &lookup)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set(name: &str, value: &str, origin: &str, line: usize) -> ConfigEvent {
        ConfigEvent {
            kind: ConfigEventKind::Set,
            name: Some(name.into()),
            value: Some(value.into()),
            origin: Some(origin.into()),
            file: Some("bringup.launch.xml".into()),
            line: Some(line),
        }
    }

    fn bare(kind: ConfigEventKind) -> ConfigEvent {
        ConfigEvent {
            kind,
            name: None,
            value: None,
            origin: None,
            file: None,
            line: None,
        }
    }

    /// A scoped group's pop puts back what its push saved; a reset clears.
    #[test]
    fn the_state_replays_groups_and_resets() {
        let log = vec![
            set("x", "1", "arg", 3),
            bare(ConfigEventKind::Push),
            set("x", "2", "let", 5),
            set("y", "9", "let", 6),
            bare(ConfigEventKind::Pop),
            set("z", "4", "let", 8),
            bare(ConfigEventKind::Reset),
        ];
        let value =
            |end: usize, n: &str| config_state(&log, end).get(n).and_then(|e| e.value.clone());
        assert_eq!(value(4, "x").as_deref(), Some("2"));
        assert_eq!(value(4, "y").as_deref(), Some("9"));
        assert_eq!(value(5, "x").as_deref(), Some("1"));
        assert_eq!(value(5, "y"), None);
        assert_eq!(value(6, "z").as_deref(), Some("4"));
        assert!(config_state(&log, 7).is_empty());
        // An end past the log is the whole log.
        assert!(config_state(&log, 99).is_empty());
    }

    #[test]
    fn an_assignment_is_named_as_its_file_spells_it() {
        assert_eq!(
            assigned_at(&set("x", "1", "let", 7)),
            "<let> bringup.launch.xml:7"
        );
        assert_eq!(assigned_at(&set("x", "1", "cli", 1)), "command line");
        let mut py = set("x", "1", "let", 1);
        py.file = Some("bringup.launch.py".into());
        py.line = None;
        assert_eq!(assigned_at(&py), "SetLaunchConfiguration bringup.launch.py");
    }

    #[test]
    fn numbers_equal_as_numbers() {
        assert!(same_value(
            &set("x", "0.5", "let", 1),
            &set("x", "0.50", "let", 2)
        ));
        assert!(!same_value(
            &set("x", "0.5", "let", 1),
            &set("x", "2.0", "let", 2)
        ));
        assert!(!same_value(
            &set("x", "a", "let", 1),
            &set("x", "b", "let", 2)
        ));
    }

    #[test]
    fn the_vars_a_substitution_reads() {
        assert_eq!(var_names("$(var a)"), ["a"]);
        assert_eq!(var_names("x_$(var a)_$(var b)_$(var a)"), ["a", "b"]);
        assert!(var_names("10").is_empty());
    }
}
