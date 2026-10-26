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
//! - `value-range`: a negative duration, a rate that is not positive, a
//!   count that is not whole (rlm).

use super::{
    launch_dump::{LaunchDump, ScopeEntry},
    manifest_loader::{ManifestIndex, resolve_node_fqn},
};
use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::{
    Manifest, SpanIndex,
    parse::ParseError,
    values::{self, Lookup, ParamNode, Read, RefAt, Resolution},
};

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
                        lp.at.clone()
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
        if let Some(n) = node {
            // Wave 2 reads the configuration log; until then the point a
            // value is read at cannot be told apart from the file's end.
            return Read::Error {
                code: "var-ambiguous",
                message: format!(
                    "`{}`: reading `{name}` where `{n}` was declared needs the launch \
                     configuration log, which this parser does not record",
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
