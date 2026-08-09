//! YAML parser for manifest files using yaml-rust2.
//!
//! Parses YAML into typed [`Manifest`] AST. Handles both plain list
//! and map forms for endpoints (e.g., `pub: [a, b]` vs `pub: {a: {min_rate_hz: 10}}`).

use crate::{span::SpanIndex, types::*};
use std::{collections::BTreeMap, path::Path};
use yaml_rust2::{Yaml, YamlLoader};

/// Parse error with optional source location.
#[derive(Debug, thiserror::Error)]
pub enum ParseError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("YAML syntax error: {0}")]
    YamlSyntax(String),
    #[error("at '{path}': {message}")]
    Field { path: String, message: String },
}

/// Result of parsing a manifest, including source text and span index
/// for diagnostic reporting.
#[derive(Debug, Clone)]
pub struct ParseResult {
    pub manifest: Manifest,
    /// Original source text (for codespan-reporting).
    pub source: String,
    /// Span index mapping YAML key paths to byte ranges.
    pub spans: SpanIndex,
}

/// Parse a manifest file from disk.
pub fn parse_manifest(path: &Path) -> Result<Manifest, ParseError> {
    let content = std::fs::read_to_string(path)?;
    parse_manifest_str(&content)
}

/// Parse a manifest file from disk, returning source and spans.
pub fn parse_manifest_with_spans(path: &Path) -> Result<ParseResult, ParseError> {
    let content = std::fs::read_to_string(path)?;
    parse_manifest_str_with_spans(&content)
}

/// Parse a manifest from a YAML string.
pub fn parse_manifest_str(source: &str) -> Result<Manifest, ParseError> {
    let docs =
        YamlLoader::load_from_str(source).map_err(|e| ParseError::YamlSyntax(e.to_string()))?;
    if docs.is_empty() {
        return Ok(Manifest {
            version: 1,
            ..Default::default()
        });
    }
    parse_manifest_yaml(&docs[0], "")
}

/// Parse a manifest from a YAML string, returning source and spans.
pub fn parse_manifest_str_with_spans(source: &str) -> Result<ParseResult, ParseError> {
    let manifest = parse_manifest_str(source)?;
    let spans = SpanIndex::build(source);
    Ok(ParseResult {
        manifest,
        source: source.to_string(),
        spans,
    })
}

fn parse_manifest_yaml(doc: &Yaml, ctx: &str) -> Result<Manifest, ParseError> {
    Ok(Manifest {
        version: yaml_u32(doc, "version").unwrap_or(1),
        args: parse_args(doc),
        exclude_patterns: yaml_string_list(doc, "exclude_patterns"),
        nodes: parse_nodes(doc, ctx)?,
        topics: parse_topics(doc, ctx)?,
        services: parse_services(doc, ctx)?,
        actions: parse_actions(doc, ctx)?,
        includes: parse_includes(doc, ctx)?,
        paths: parse_paths(doc, ctx)?,
        external_topics: parse_external_topics(doc, ctx)?,
        chains: parse_chains(doc, ctx)?,
    })
}

// ── Nodes ──

fn parse_nodes(doc: &Yaml, ctx: &str) -> Result<BTreeMap<String, NodeDecl>, ParseError> {
    let mut nodes = BTreeMap::new();
    let section = &doc["nodes"];
    if section.is_badvalue() {
        return Ok(nodes);
    }
    let hash = section
        .as_hash()
        .ok_or_else(|| field_err(ctx, "nodes", "expected mapping"))?;
    for (k, v) in hash {
        let name = yaml_str_owned(k);
        let path = format_path(ctx, &format!("nodes.{name}"));
        let node = parse_node_decl(v, &path)?;
        // Validate endpoint uniqueness
        validate_endpoint_uniqueness(&node, &path)?;
        nodes.insert(name, node);
    }
    Ok(nodes)
}

fn parse_node_decl(yaml: &Yaml, ctx: &str) -> Result<NodeDecl, ParseError> {
    if yaml.is_null() || yaml.is_badvalue() {
        return Ok(NodeDecl::default());
    }
    Ok(NodeDecl {
        if_condition: yaml_string(yaml, "if"),
        unless_condition: yaml_string(yaml, "unless"),
        lifecycle: yaml_bool(yaml, "lifecycle"),
        publishers: parse_endpoints(yaml, "pub", ctx)?,
        subscribers: parse_endpoints(yaml, "sub", ctx)?,
        srv: parse_srv_endpoints(yaml, "srv", ctx)?,
        cli: parse_endpoints(yaml, "cli", ctx)?,
        paths: parse_paths(yaml, ctx)?,
        criticality: yaml_string(yaml, "criticality"),
    })
}

/// Parse endpoints: either a list `[a, b]` or a map `{a: {min_rate_hz: 10}, b: null}`.
fn parse_endpoints(
    doc: &Yaml,
    key: &str,
    ctx: &str,
) -> Result<BTreeMap<String, EndpointProps>, ParseError> {
    let mut eps = BTreeMap::new();
    let section = &doc[key];
    if section.is_badvalue() {
        return Ok(eps);
    }
    match section {
        Yaml::Array(arr) => {
            for item in arr {
                let name = yaml_str_owned(item);
                eps.insert(name, EndpointProps::default());
            }
        }
        Yaml::Hash(hash) => {
            for (k, v) in hash {
                let name = yaml_str_owned(k);
                let path = format_path(ctx, &format!("{key}.{name}"));
                let props = parse_endpoint_props(v, &path)?;
                eps.insert(name, props);
            }
        }
        _ => {
            return Err(field_err(ctx, key, "expected list or mapping"));
        }
    }
    Ok(eps)
}

fn parse_endpoint_props(yaml: &Yaml, ctx: &str) -> Result<EndpointProps, ParseError> {
    if yaml.is_null() || yaml.is_badvalue() {
        return Ok(EndpointProps::default());
    }
    let props = EndpointProps {
        min_rate_hz: yaml_f64(yaml, "min_rate_hz"),
        max_rate_hz: yaml_f64(yaml, "max_rate_hz"),
        jitter_ms: yaml_f64(yaml, "jitter_ms"),
        max_age_ms: yaml_f64(yaml, "max_age_ms"),
        state: yaml_bool(yaml, "state"),
        required: yaml_bool(yaml, "required"),
        qos: parse_qos(yaml),
        max_transport_ms: yaml_f64(yaml, "max_transport_ms"),
        buffer: parse_buffer(yaml, ctx)?,
    };
    if props.buffer.is_some() && props.state != Some(true) {
        return Err(field_err(
            ctx,
            "buffer",
            "'buffer' is only meaningful on a subscriber with 'state: true'",
        ));
    }
    Ok(props)
}

/// Parse the `buffer:` discriminator (Vocabulary v2, Phase 44.1 §3).
fn parse_buffer(doc: &Yaml, ctx: &str) -> Result<Option<Buffer>, ParseError> {
    let raw = match yaml_string(doc, "buffer") {
        Some(s) => s,
        None => return Ok(None),
    };
    match raw.as_str() {
        "latest" => Ok(Some(Buffer::Latest)),
        "queue" => Ok(Some(Buffer::Queue)),
        other => Err(field_err(
            ctx,
            "buffer",
            &format!("invalid buffer '{other}', expected 'latest' or 'queue'"),
        )),
    }
}

fn parse_srv_endpoints(
    doc: &Yaml,
    key: &str,
    ctx: &str,
) -> Result<BTreeMap<String, SrvEndpointProps>, ParseError> {
    let mut eps = BTreeMap::new();
    let section = &doc[key];
    if section.is_badvalue() {
        return Ok(eps);
    }
    match section {
        Yaml::Array(arr) => {
            for item in arr {
                let name = yaml_str_owned(item);
                eps.insert(name, SrvEndpointProps::default());
            }
        }
        Yaml::Hash(hash) => {
            for (k, v) in hash {
                let name = yaml_str_owned(k);
                let props = SrvEndpointProps {
                    max_response_ms: if v.is_null() || v.is_badvalue() {
                        None
                    } else {
                        yaml_f64(v, "max_response_ms")
                    },
                };
                eps.insert(name, props);
            }
        }
        _ => {
            return Err(field_err(ctx, key, "expected list or mapping"));
        }
    }
    Ok(eps)
}

fn validate_endpoint_uniqueness(node: &NodeDecl, ctx: &str) -> Result<(), ParseError> {
    let mut seen = std::collections::HashSet::new();
    for name in node
        .publishers
        .keys()
        .chain(node.subscribers.keys())
        .chain(node.srv.keys())
        .chain(node.cli.keys())
    {
        if !seen.insert(name.as_str()) {
            return Err(field_err(
                ctx,
                name,
                "duplicate endpoint name across pub/sub/srv/cli",
            ));
        }
    }
    Ok(())
}

// ── Topics ──

fn parse_topics(doc: &Yaml, ctx: &str) -> Result<BTreeMap<String, TopicDecl>, ParseError> {
    let mut topics = BTreeMap::new();
    let section = &doc["topics"];
    if section.is_badvalue() {
        return Ok(topics);
    }
    let hash = section
        .as_hash()
        .ok_or_else(|| field_err(ctx, "topics", "expected mapping"))?;
    for (k, v) in hash {
        let name = yaml_str_owned(k);
        let path = format_path(ctx, &format!("topics.{name}"));
        let topic = parse_topic_decl(v, &path)?;
        topics.insert(name, topic);
    }
    Ok(topics)
}

fn parse_topic_decl(yaml: &Yaml, ctx: &str) -> Result<TopicDecl, ParseError> {
    // Shorthand: just a type string
    if let Some(s) = yaml.as_str() {
        return Ok(TopicDecl {
            if_condition: None,
            unless_condition: None,
            msg_type: s.to_string(),
            publishers: vec![],
            subscribers: vec![],
            qos: None,
            rate_hz: None,
            max_transport_ms: None,
            drop: None,
            external: None,
        });
    }
    Ok(TopicDecl {
        if_condition: yaml_string(yaml, "if"),
        unless_condition: yaml_string(yaml, "unless"),
        msg_type: yaml_string(yaml, "type")
            .ok_or_else(|| field_err(ctx, "type", "topic must have a type"))?,
        publishers: yaml_string_list(yaml, "pub"),
        subscribers: yaml_string_list(yaml, "sub"),
        qos: parse_qos(yaml),
        rate_hz: yaml_f64(yaml, "rate_hz"),
        max_transport_ms: yaml_f64(yaml, "max_transport_ms"),
        drop: parse_drop_spec(yaml, "drop", ctx)?,
        external: parse_external_side(yaml, "external", ctx)?,
    })
}

/// Parse the manifest-level `external_topics:` block.
fn parse_external_topics(
    doc: &Yaml,
    ctx: &str,
) -> Result<BTreeMap<String, ExternalTopicDecl>, ParseError> {
    let mut out = BTreeMap::new();
    let section = &doc["external_topics"];
    if section.is_badvalue() {
        return Ok(out);
    }
    let hash = section
        .as_hash()
        .ok_or_else(|| field_err(ctx, "external_topics", "expected mapping"))?;
    for (k, v) in hash {
        let name = yaml_str_owned(k);
        let path = format_path(ctx, &format!("external_topics.{name}"));
        // Accept either `side:` or `external:` as the side selector.
        // Try whichever is present; if both are present, `side:` wins.
        let side_present = !v["side"].is_badvalue();
        let external_present = !v["external"].is_badvalue();
        let side = if side_present {
            parse_external_side(v, "side", &path)?
        } else if external_present {
            parse_external_side(v, "external", &path)?
        } else {
            None
        };
        let side = side.ok_or_else(|| {
            field_err(
                &path,
                "side",
                "external_topics entry must have a 'side: pub|sub|both' (or 'external:') field",
            )
        })?;
        out.insert(
            name,
            ExternalTopicDecl {
                side,
                msg_type: yaml_string(v, "type"),
                qos: parse_qos(v),
            },
        );
    }
    Ok(out)
}

/// Parse an `ExternalSide` enum from a string-valued YAML field.
fn parse_external_side(
    doc: &Yaml,
    key: &str,
    ctx: &str,
) -> Result<Option<ExternalSide>, ParseError> {
    let raw = match yaml_string(doc, key) {
        Some(s) => s,
        None => return Ok(None),
    };
    match raw.as_str() {
        "pub" => Ok(Some(ExternalSide::Pub)),
        "sub" => Ok(Some(ExternalSide::Sub)),
        "both" => Ok(Some(ExternalSide::Both)),
        _ => Err(field_err(
            ctx,
            key,
            &format!("invalid external side '{raw}', expected 'pub', 'sub', or 'both'"),
        )),
    }
}

// ── Services & Actions ──

fn parse_services(doc: &Yaml, ctx: &str) -> Result<BTreeMap<String, ServiceDecl>, ParseError> {
    let mut out = BTreeMap::new();
    let section = &doc["services"];
    if section.is_badvalue() {
        return Ok(out);
    }
    let hash = section
        .as_hash()
        .ok_or_else(|| field_err(ctx, "services", "expected mapping"))?;
    for (k, v) in hash {
        let name = yaml_str_owned(k);
        out.insert(
            name,
            ServiceDecl {
                if_condition: yaml_string(v, "if"),
                unless_condition: yaml_string(v, "unless"),
                srv_type: yaml_string(v, "type").unwrap_or_default(),
                server: yaml_string_list(v, "server"),
                client: yaml_string_list(v, "client"),
            },
        );
    }
    Ok(out)
}

fn parse_actions(doc: &Yaml, ctx: &str) -> Result<BTreeMap<String, ActionDecl>, ParseError> {
    let mut out = BTreeMap::new();
    let section = &doc["actions"];
    if section.is_badvalue() {
        return Ok(out);
    }
    let hash = section
        .as_hash()
        .ok_or_else(|| field_err(ctx, "actions", "expected mapping"))?;
    for (k, v) in hash {
        let name = yaml_str_owned(k);
        out.insert(
            name,
            ActionDecl {
                if_condition: yaml_string(v, "if"),
                unless_condition: yaml_string(v, "unless"),
                action_type: yaml_string(v, "type").unwrap_or_default(),
                server: yaml_string_list(v, "server"),
                client: yaml_string_list(v, "client"),
            },
        );
    }
    Ok(out)
}

// ── Includes ──

fn parse_includes(doc: &Yaml, ctx: &str) -> Result<BTreeMap<String, IncludeDecl>, ParseError> {
    let mut out = BTreeMap::new();
    let section = &doc["includes"];
    if section.is_badvalue() {
        return Ok(out);
    }
    let hash = section
        .as_hash()
        .ok_or_else(|| field_err(ctx, "includes", "expected mapping"))?;
    for (k, v) in hash {
        let name = yaml_str_owned(k);
        let path = format_path(ctx, &format!("includes.{name}"));
        if let Some(manifest_path) = yaml_string(v, "manifest") {
            out.insert(
                name,
                IncludeDecl::External {
                    manifest: manifest_path,
                },
            );
        } else {
            // Inline scope
            let inner = parse_manifest_yaml(v, &path)?;
            out.insert(name, IncludeDecl::Inline(Box::new(inner)));
        }
    }
    Ok(out)
}

// ── Args ──

/// Parse `args:` section. All args are mandatory — values from scope table.
///
/// Accepts:
/// - List form: `args: [a, b]` → all free strings
/// - Map form with null values: `args: { a:, b: }` → all free strings
/// - Map form with type: `args: { x: { type: bool } }` → typed
/// - Map form with choices: `args: { x: { choices: [a, b] } }` → enum
fn parse_args(doc: &Yaml) -> BTreeMap<String, ArgDecl> {
    let mut out = BTreeMap::new();
    let section = &doc["args"];
    if section.is_badvalue() {
        return out;
    }
    match section {
        Yaml::Array(arr) => {
            for item in arr {
                out.insert(yaml_str_owned(item), ArgDecl::String);
            }
        }
        Yaml::Hash(hash) => {
            for (k, v) in hash {
                let name = yaml_str_owned(k);
                let decl = parse_arg_decl(v);
                out.insert(name, decl);
            }
        }
        _ => {}
    }
    out
}

fn parse_arg_decl(yaml: &Yaml) -> ArgDecl {
    if yaml.is_null() || yaml.is_badvalue() {
        return ArgDecl::String;
    }
    // Check for { type: bool }
    if let Some(type_str) = yaml_string(yaml, "type") {
        if type_str == "bool" {
            return ArgDecl::Bool;
        }
        // "string" or any other type → free string
        return ArgDecl::String;
    }
    // Check for { choices: [a, b, c] }
    let choices = yaml_string_list(yaml, "choices");
    if !choices.is_empty() {
        return ArgDecl::Choices(choices);
    }
    ArgDecl::String
}

// ── Paths ──

fn parse_paths(doc: &Yaml, ctx: &str) -> Result<BTreeMap<String, PathDecl>, ParseError> {
    let mut out = BTreeMap::new();
    let section = &doc["paths"];
    if section.is_badvalue() {
        return Ok(out);
    }
    let hash = section
        .as_hash()
        .ok_or_else(|| field_err(ctx, "paths", "expected mapping"))?;
    for (k, v) in hash {
        let name = yaml_str_owned(k);
        let path_ctx = format_path(ctx, &format!("paths.{name}"));
        let path = parse_path_decl(v, &path_ctx)?;
        out.insert(name, path);
    }
    Ok(out)
}

fn parse_path_decl(yaml: &Yaml, ctx: &str) -> Result<PathDecl, ParseError> {
    let input = parse_string_or_list(yaml, "input");
    let trigger = parse_trigger(yaml, ctx)?;

    // Vocabulary v2 §1: explicit trigger.input and legacy input: may both
    // be present; they must agree (as sets) or it's a parse error.
    if let Some(Trigger::Input(trigger_eps)) = &trigger
        && !input.is_empty()
    {
        let mut a: Vec<&str> = trigger_eps.iter().map(String::as_str).collect();
        let mut b: Vec<&str> = input.iter().map(String::as_str).collect();
        a.sort_unstable();
        b.sort_unstable();
        if a != b {
            return Err(field_err(
                ctx,
                "trigger.input",
                "disagrees with the legacy 'input:' list on this path",
            ));
        }
    }

    let sync = parse_sync(yaml, ctx)?;

    let path = PathDecl {
        if_condition: yaml_string(yaml, "if"),
        unless_condition: yaml_string(yaml, "unless"),
        input,
        output: yaml_string_list(yaml, "output"),
        max_latency_ms: yaml_f64(yaml, "max_latency_ms"),
        correlation: yaml_string(yaml, "correlation"),
        tolerance_ms: yaml_f64(yaml, "tolerance_ms"),
        drop: parse_drop_spec(yaml, "drop", ctx)?,
        trigger,
        sync,
    };

    // Vocabulary v2 §2: sync only meaningful with an input trigger with
    // >=2 endpoints — cheap and unambiguous to check at parse time.
    if path.sync.is_some() {
        let ok = matches!(
            path.effective_trigger(),
            EffectiveTrigger::Input(eps) if eps.len() >= 2
        );
        if !ok {
            return Err(field_err(
                ctx,
                "sync",
                "'sync:' requires an input trigger with at least 2 endpoints",
            ));
        }
    }

    Ok(path)
}

// ── Trigger ──

/// Parse `trigger:` (Vocabulary v2, Phase 44.1 §1). Bare-string forms
/// (`once`, `spontaneous`) or single-key mapping forms (`timer:`,
/// `input:`).
fn parse_trigger(doc: &Yaml, ctx: &str) -> Result<Option<Trigger>, ParseError> {
    let section = &doc["trigger"];
    if section.is_badvalue() {
        return Ok(None);
    }
    if let Some(s) = section.as_str() {
        return match s {
            "once" => Ok(Some(Trigger::Once)),
            "spontaneous" => Ok(Some(Trigger::Spontaneous)),
            other => Err(field_err(
                ctx,
                "trigger",
                &format!(
                    "invalid trigger '{other}', expected 'once', 'spontaneous', \
                     or a mapping with 'timer' or 'input'"
                ),
            )),
        };
    }
    let hash = section.as_hash().ok_or_else(|| {
        field_err(
            ctx,
            "trigger",
            "expected 'once', 'spontaneous', or a mapping with 'timer' or 'input'",
        )
    })?;
    if hash.len() != 1 {
        return Err(field_err(
            ctx,
            "trigger",
            "expected exactly one of 'timer' or 'input'",
        ));
    }
    let (k, v) = hash.iter().next().expect("len == 1");
    let key = yaml_str_owned(k);
    match key.as_str() {
        "timer" => {
            let rate_hz = yaml_f64(v, "rate_hz")
                .ok_or_else(|| field_err(ctx, "trigger.timer", "requires 'rate_hz'"))?;
            if rate_hz <= 0.0 {
                return Err(field_err(ctx, "trigger.timer.rate_hz", "must be > 0"));
            }
            Ok(Some(Trigger::Timer { rate_hz }))
        }
        "input" => {
            let endpoints = yaml_direct_string_list(v);
            Ok(Some(Trigger::Input(endpoints)))
        }
        other => Err(field_err(
            ctx,
            "trigger",
            &format!("invalid trigger key '{other}', expected 'timer' or 'input'"),
        )),
    }
}

// ── Sync ──

/// Parse `sync:` (Vocabulary v2, Phase 44.1 §2).
fn parse_sync(doc: &Yaml, ctx: &str) -> Result<Option<Sync>, ParseError> {
    let section = &doc["sync"];
    if section.is_badvalue() {
        return Ok(None);
    }
    let policy_str = yaml_string(section, "policy").ok_or_else(|| {
        field_err(
            ctx,
            "sync.policy",
            "required: 'exact', 'approximate', or 'timeout_any'",
        )
    })?;
    let policy = match policy_str.as_str() {
        "exact" => SyncPolicy::Exact,
        "approximate" => SyncPolicy::Approximate,
        "timeout_any" => SyncPolicy::TimeoutAny,
        other => {
            return Err(field_err(
                ctx,
                "sync.policy",
                &format!(
                    "invalid policy '{other}', expected 'exact', 'approximate', or 'timeout_any'"
                ),
            ));
        }
    };
    let max_interval_ms = yaml_f64(section, "max_interval_ms");
    let timeout_ms = yaml_f64(section, "timeout_ms");
    match policy {
        SyncPolicy::TimeoutAny => {
            if timeout_ms.is_none() {
                return Err(field_err(
                    ctx,
                    "sync.timeout_ms",
                    "required when policy is 'timeout_any'",
                ));
            }
        }
        SyncPolicy::Exact | SyncPolicy::Approximate => {
            if max_interval_ms.is_none() {
                return Err(field_err(
                    ctx,
                    "sync.max_interval_ms",
                    "required when policy is 'exact' or 'approximate'",
                ));
            }
        }
    }
    Ok(Some(Sync {
        policy,
        max_interval_ms,
        timeout_ms,
    }))
}

// ── Chains ──

/// Parse the top-level `chains:` section (Vocabulary v2, Phase 44.1 §4).
fn parse_chains(doc: &Yaml, ctx: &str) -> Result<BTreeMap<String, ChainDecl>, ParseError> {
    let mut out = BTreeMap::new();
    let section = &doc["chains"];
    if section.is_badvalue() {
        return Ok(out);
    }
    let hash = section
        .as_hash()
        .ok_or_else(|| field_err(ctx, "chains", "expected mapping"))?;
    for (k, v) in hash {
        let name = yaml_str_owned(k);
        let path = format_path(ctx, &format!("chains.{name}"));
        let chain = parse_chain_decl(v, &path)?;
        out.insert(name, chain);
    }
    Ok(out)
}

fn parse_chain_decl(yaml: &Yaml, ctx: &str) -> Result<ChainDecl, ParseError> {
    let semantics_str = yaml_string(yaml, "semantics")
        .ok_or_else(|| field_err(ctx, "semantics", "required: 'reaction' or 'age'"))?;
    let semantics = match semantics_str.as_str() {
        "reaction" => ChainSemantics::Reaction,
        "age" => ChainSemantics::Age,
        other => {
            return Err(field_err(
                ctx,
                "semantics",
                &format!("invalid semantics '{other}', expected 'reaction' or 'age'"),
            ));
        }
    };
    let max_latency_ms = yaml_f64(yaml, "max_latency_ms")
        .ok_or_else(|| field_err(ctx, "max_latency_ms", "required"))?;
    let segments = parse_chain_segments(yaml, ctx)?;
    Ok(ChainDecl {
        semantics,
        max_latency_ms,
        segments,
    })
}

fn parse_chain_segments(yaml: &Yaml, ctx: &str) -> Result<Vec<ChainSegment>, ParseError> {
    let section = &yaml["segments"];
    let arr = section
        .as_vec()
        .ok_or_else(|| field_err(ctx, "segments", "expected a non-empty list"))?;
    if arr.is_empty() {
        return Err(field_err(ctx, "segments", "must be non-empty"));
    }
    let mut segments = Vec::with_capacity(arr.len());
    for (i, item) in arr.iter().enumerate() {
        let seg_ctx = format_path(ctx, &format!("segments[{i}]"));
        segments.push(parse_chain_segment(item, &seg_ctx)?);
    }
    // Local shape only (Phase 44.1 §4) — cross-file link resolution
    // (does every `via` exist? is it produced/consumed correctly?) is
    // the checker's `chain-link` rule (W2).
    if !matches!(segments.first(), Some(ChainSegment::Path { .. })) {
        return Err(field_err(
            ctx,
            "segments",
            "first segment must be a path segment ({ scope, path })",
        ));
    }
    if !matches!(segments.last(), Some(ChainSegment::Path { .. })) {
        return Err(field_err(
            ctx,
            "segments",
            "last segment must be a path segment ({ scope, path })",
        ));
    }
    for w in segments.windows(2) {
        if matches!(w[0], ChainSegment::Via { .. }) && matches!(w[1], ChainSegment::Via { .. }) {
            return Err(field_err(
                ctx,
                "segments",
                "two adjacent 'via' segments are not allowed",
            ));
        }
    }
    Ok(segments)
}

fn parse_chain_segment(yaml: &Yaml, ctx: &str) -> Result<ChainSegment, ParseError> {
    let has_via = !yaml["via"].is_badvalue();
    let has_scope = !yaml["scope"].is_badvalue();
    let has_path = !yaml["path"].is_badvalue();
    if has_via && (has_scope || has_path) {
        return Err(field_err(
            ctx,
            "segment",
            "'via' cannot be combined with 'scope'/'path'",
        ));
    }
    if has_via {
        let via =
            yaml_string(yaml, "via").ok_or_else(|| field_err(ctx, "via", "expected a string"))?;
        return Ok(ChainSegment::Via { via });
    }
    let scope = yaml_string(yaml, "scope")
        .ok_or_else(|| field_err(ctx, "scope", "path segment requires 'scope'"))?;
    let path = yaml_string(yaml, "path")
        .ok_or_else(|| field_err(ctx, "path", "path segment requires 'path'"))?;
    Ok(ChainSegment::Path { scope, path })
}

// ── Drop ──

fn parse_drop_spec(doc: &Yaml, key: &str, ctx: &str) -> Result<Option<DropSpec>, ParseError> {
    let section = &doc[key];
    if section.is_badvalue() {
        return Ok(None);
    }
    // Shorthand: "5 / 100"
    if let Some(s) = section.as_str() {
        let count: DropCount = s.parse().map_err(|e: String| field_err(ctx, key, &e))?;
        return Ok(Some(DropSpec {
            max_count: Some(count),
            max_consecutive: None,
        }));
    }
    // Full form: { max_count: "5 / 100", max_consecutive: 3 }
    let max_count = yaml_string(section, "max_count")
        .map(|s| {
            s.parse::<DropCount>()
                .map_err(|e| field_err(ctx, &format!("{key}.max_count"), &e.to_string()))
        })
        .transpose()?;
    let max_consecutive = yaml_u32(section, "max_consecutive");

    Ok(Some(DropSpec {
        max_count,
        max_consecutive,
    }))
}

// ── QoS ──

fn parse_qos(doc: &Yaml) -> Option<QosDecl> {
    let section = &doc["qos"];
    if section.is_badvalue() {
        return None;
    }
    Some(QosDecl {
        reliability: yaml_string(section, "reliability"),
        durability: yaml_string(section, "durability"),
        depth: yaml_u32(section, "depth"),
        history: yaml_string(section, "history"),
        lifespan_ms: yaml_u64(section, "lifespan_ms"),
        liveliness: yaml_string(section, "liveliness"),
    })
}

// ── YAML helpers ──

fn yaml_str_owned(y: &Yaml) -> String {
    match y {
        Yaml::String(s) => s.clone(),
        Yaml::Integer(i) => i.to_string(),
        Yaml::Real(s) => s.clone(),
        Yaml::Boolean(b) => b.to_string(),
        _ => String::new(),
    }
}

fn yaml_string(doc: &Yaml, key: &str) -> Option<String> {
    doc[key].as_str().map(|s| s.to_string())
}

fn yaml_f64(doc: &Yaml, key: &str) -> Option<f64> {
    match &doc[key] {
        Yaml::Real(s) => s.parse().ok(),
        Yaml::Integer(i) => Some(*i as f64),
        _ => None,
    }
}

fn yaml_u32(doc: &Yaml, key: &str) -> Option<u32> {
    doc[key].as_i64().map(|i| i as u32)
}

fn yaml_u64(doc: &Yaml, key: &str) -> Option<u64> {
    doc[key].as_i64().map(|i| i as u64)
}

fn yaml_bool(doc: &Yaml, key: &str) -> Option<bool> {
    doc[key].as_bool()
}

fn yaml_string_list(doc: &Yaml, key: &str) -> Vec<String> {
    match &doc[key] {
        Yaml::Array(arr) => arr.iter().map(yaml_str_owned).collect(),
        _ => vec![],
    }
}

/// Parse a field that can be a single string or a list of strings.
fn parse_string_or_list(doc: &Yaml, key: &str) -> Vec<String> {
    yaml_direct_string_list(&doc[key])
}

/// Interpret a YAML value directly (not indexed under a key) as a single
/// string or a list of strings. Used for nested trigger sub-values like
/// `trigger: { input: [a, b] }`, where `v` is already the array/string.
fn yaml_direct_string_list(y: &Yaml) -> Vec<String> {
    match y {
        Yaml::String(s) => vec![s.clone()],
        Yaml::Array(arr) => arr.iter().map(yaml_str_owned).collect(),
        _ => vec![],
    }
}

fn field_err(ctx: &str, field: &str, message: &str) -> ParseError {
    ParseError::Field {
        path: format_path(ctx, field),
        message: message.to_string(),
    }
}

fn format_path(ctx: &str, field: &str) -> String {
    if ctx.is_empty() {
        field.to_string()
    } else {
        format!("{ctx}.{field}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_minimal_manifest() {
        let yaml = r#"
version: 1
nodes:
  talker:
    pub: [chatter]
  listener:
    sub: [chatter]
topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
"#;
        let m = parse_manifest_str(yaml).unwrap();
        assert_eq!(m.version, 1);
        assert_eq!(m.nodes.len(), 2);
        assert_eq!(m.topics.len(), 1);

        let talker = &m.nodes["talker"];
        assert!(talker.publishers.contains_key("chatter"));
        assert!(talker.subscribers.is_empty());

        let topic = &m.topics["chatter"];
        assert_eq!(topic.msg_type, "std_msgs/msg/String");
        assert_eq!(topic.publishers, vec!["talker/chatter"]);
        assert_eq!(topic.subscribers, vec!["listener/chatter"]);
    }

    #[test]
    fn test_endpoint_properties() {
        let yaml = r#"
version: 1
nodes:
  ndt:
    sub:
      sensor_points:
        min_rate_hz: 10
      initial_pose:
        required: true
      regularization_pose:
        state: true
    pub:
      ndt_pose:
        min_rate_hz: 10
      exe_time_ms:
    srv:
      trigger_node:
        max_response_ms: 100
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let ndt = &m.nodes["ndt"];

        assert_eq!(ndt.subscribers["sensor_points"].min_rate_hz, Some(10.0));
        assert_eq!(ndt.subscribers["initial_pose"].required, Some(true));
        assert_eq!(ndt.subscribers["regularization_pose"].state, Some(true));
        assert_eq!(ndt.publishers["ndt_pose"].min_rate_hz, Some(10.0));
        assert!(ndt.publishers.contains_key("exe_time_ms"));
        assert_eq!(ndt.srv["trigger_node"].max_response_ms, Some(100.0));
    }

    #[test]
    fn test_lifecycle_node() {
        let yaml = r#"
version: 1
nodes:
  lidar_driver:
    lifecycle: true
    pub:
      pointcloud:
        min_rate_hz: 10
  regular_node:
    pub:
      data: {}
"#;
        let m = parse_manifest_str(yaml).unwrap();
        assert_eq!(m.nodes["lidar_driver"].lifecycle, Some(true));
        assert_eq!(m.nodes["regular_node"].lifecycle, None);
    }

    #[test]
    fn test_node_criticality() {
        let yaml = r#"
version: 1
nodes:
  control_node:
    criticality: high
  regular_node:
    pub:
      data: {}
"#;
        let m = parse_manifest_str(yaml).unwrap();
        assert_eq!(m.nodes["control_node"].criticality.as_deref(), Some("high"));
        assert_eq!(m.nodes["regular_node"].criticality, None);
    }

    #[test]
    fn test_paths() {
        let yaml = r#"
version: 1
nodes:
  ndt:
    sub: [sensor_points]
    pub: [ndt_pose, exe_time_ms]
    paths:
      localization:
        input: sensor_points
        output: [ndt_pose]
        max_latency_ms: 50
        drop:
          max_count: 10 / 100
          max_consecutive: 5
      debug:
        input: sensor_points
        output: [exe_time_ms]
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let ndt = &m.nodes["ndt"];
        assert_eq!(ndt.paths.len(), 2);

        let loc = &ndt.paths["localization"];
        assert_eq!(loc.input, vec!["sensor_points"]);
        assert_eq!(loc.output, vec!["ndt_pose"]);
        assert_eq!(loc.max_latency_ms, Some(50.0));
        let drop = loc.drop.as_ref().unwrap();
        assert_eq!(drop.max_count.as_ref().unwrap().n, 10);
        assert_eq!(drop.max_count.as_ref().unwrap().w, 100);
        assert_eq!(drop.max_consecutive, Some(5));

        let debug = &ndt.paths["debug"];
        assert_eq!(debug.input, vec!["sensor_points"]);
        assert!(debug.max_latency_ms.is_none());
    }

    #[test]
    fn test_topic_with_contract() {
        let yaml = r#"
version: 1
topics:
  pointcloud:
    type: sensor_msgs/msg/PointCloud2
    pub: [lidar_driver/pointcloud]
    sub: [cropbox_filter/input]
    qos:
      reliability: best_effort
      depth: 1
    rate_hz: 10
    drop: 1 / 100
  debug_output: sensor_msgs/msg/PointCloud2
"#;
        let m = parse_manifest_str(yaml).unwrap();
        assert_eq!(m.topics.len(), 2);

        let pc = &m.topics["pointcloud"];
        assert_eq!(pc.msg_type, "sensor_msgs/msg/PointCloud2");
        assert_eq!(pc.rate_hz, Some(10.0));
        let qos = pc.qos.as_ref().unwrap();
        assert_eq!(qos.reliability.as_deref(), Some("best_effort"));
        assert_eq!(qos.depth, Some(1));
        let drop = pc.drop.as_ref().unwrap();
        assert_eq!(drop.max_count.as_ref().unwrap().n, 1);
        assert_eq!(drop.max_count.as_ref().unwrap().w, 100);

        // Shorthand form
        let debug = &m.topics["debug_output"];
        assert_eq!(debug.msg_type, "sensor_msgs/msg/PointCloud2");
        assert!(debug.publishers.is_empty());
    }

    #[test]
    fn test_includes() {
        let yaml = r#"
version: 1
includes:
  lidar:
    manifest: tier4_perception_launch/lidar_perception.launch.yaml
  safety:
    nodes:
      emergency_stop:
        pub: [stop_cmd]
        sub: [diagnostics]
"#;
        let m = parse_manifest_str(yaml).unwrap();
        assert_eq!(m.includes.len(), 2);

        match &m.includes["lidar"] {
            IncludeDecl::External { manifest } => {
                assert_eq!(
                    manifest,
                    "tier4_perception_launch/lidar_perception.launch.yaml"
                );
            }
            _ => panic!("expected External"),
        }

        match &m.includes["safety"] {
            IncludeDecl::Inline(inner) => {
                assert!(inner.nodes.contains_key("emergency_stop"));
            }
            _ => panic!("expected Inline"),
        }
    }

    #[test]
    fn test_scope_paths() {
        let yaml = r#"
version: 1
paths:
  main:
    input: raw_data
    output: [detections]
    max_latency_ms: 50
"#;
        let m = parse_manifest_str(yaml).unwrap();

        let main = &m.paths["main"];
        assert_eq!(main.input, vec!["raw_data"]);
        assert_eq!(main.output, vec!["detections"]);
        assert_eq!(main.max_latency_ms, Some(50.0));
    }

    #[test]
    fn test_multi_input_path() {
        let yaml = r#"
version: 1
nodes:
  fusion:
    sub: [lidar_objects, camera_objects]
    pub: [fused]
    paths:
      fusion:
        input: [lidar_objects, camera_objects]
        output: [fused]
        correlation: timestamp
        tolerance_ms: 50
        max_latency_ms: 20
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let path = &m.nodes["fusion"].paths["fusion"];
        assert_eq!(path.input, vec!["lidar_objects", "camera_objects"]);
        assert_eq!(path.correlation.as_deref(), Some("timestamp"));
        assert_eq!(path.tolerance_ms, Some(50.0));
    }

    #[test]
    fn test_endpoint_uniqueness_violation() {
        let yaml = r#"
version: 1
nodes:
  relay:
    pub: [data]
    sub: [data]
"#;
        let result = parse_manifest_str(yaml);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("duplicate endpoint name"), "got: {err}");
    }

    #[test]
    fn test_services_and_actions() {
        let yaml = r#"
version: 1
services:
  configure:
    type: std_srvs/srv/SetBool
    server: [driver/configure]
    client: [controller/configure]
actions:
  navigate:
    type: nav2_msgs/action/NavigateToPose
    server: [navigator/navigate]
    client: [planner/navigate]
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let svc = &m.services["configure"];
        assert_eq!(svc.srv_type, "std_srvs/srv/SetBool");
        assert_eq!(svc.server, vec!["driver/configure"]);

        let act = &m.actions["navigate"];
        assert_eq!(act.action_type, "nav2_msgs/action/NavigateToPose");
        assert_eq!(act.client, vec!["planner/navigate"]);
    }

    #[test]
    fn test_empty_manifest() {
        let m = parse_manifest_str("").unwrap();
        assert_eq!(m.version, 1);
        assert!(m.nodes.is_empty());
    }

    #[test]
    fn test_endpoint_qos_override() {
        let yaml = r#"
version: 1
nodes:
  lidar:
    pub:
      pointcloud:
        qos:
          reliability: best_effort
          depth: 5
  perception:
    sub:
      pointcloud:
        qos: { reliability: reliable }
        max_transport_ms: 0
  remote_viz:
    sub:
      pointcloud:
        max_transport_ms: 10
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let lidar_pub = &m.nodes["lidar"].publishers["pointcloud"];
        let lidar_qos = lidar_pub.qos.as_ref().unwrap();
        assert_eq!(lidar_qos.reliability.as_deref(), Some("best_effort"));
        assert_eq!(lidar_qos.depth, Some(5));
        assert_eq!(lidar_pub.max_transport_ms, None);

        let perception_sub = &m.nodes["perception"].subscribers["pointcloud"];
        let perception_qos = perception_sub.qos.as_ref().unwrap();
        assert_eq!(perception_qos.reliability.as_deref(), Some("reliable"));
        assert_eq!(perception_sub.max_transport_ms, Some(0.0));

        let remote_sub = &m.nodes["remote_viz"].subscribers["pointcloud"];
        assert!(remote_sub.qos.is_none());
        assert_eq!(remote_sub.max_transport_ms, Some(10.0));
    }

    #[test]
    fn test_endpoint_effective_qos_overlay() {
        use crate::types::QosDecl;
        let topic = QosDecl {
            reliability: Some("reliable".into()),
            durability: Some("transient_local".into()),
            depth: Some(10),
            ..Default::default()
        };
        let endpoint = QosDecl {
            reliability: Some("best_effort".into()),
            ..Default::default()
        };
        let eff = QosDecl::effective(Some(&topic), Some(&endpoint));
        // Endpoint override wins for reliability.
        assert_eq!(eff.reliability.as_deref(), Some("best_effort"));
        // Topic-level fields are inherited.
        assert_eq!(eff.durability.as_deref(), Some("transient_local"));
        assert_eq!(eff.depth, Some(10));

        // Empty endpoint qos = full inherit.
        let eff = QosDecl::effective(Some(&topic), Some(&QosDecl::default()));
        assert_eq!(eff.reliability.as_deref(), Some("reliable"));

        // No topic-level: endpoint stands alone.
        let eff = QosDecl::effective(None, Some(&endpoint));
        assert_eq!(eff.reliability.as_deref(), Some("best_effort"));
        assert_eq!(eff.durability, None);

        // Neither side: all None.
        let eff = QosDecl::effective(None, None);
        assert_eq!(eff.reliability, None);
        assert_eq!(eff.durability, None);
    }

    #[test]
    fn test_external_topics_block() {
        use crate::types::ExternalSide;
        let yaml = r#"
version: 1
external_topics:
  /tf:
    external: pub
    type: tf2_msgs/msg/TFMessage
  /vehicle/engage:
    external: pub
  /debug/marker:
    external: sub
  /passthrough/relay:
    external: both
    type: std_msgs/msg/String
    qos: { reliability: best_effort }
"#;
        let m = parse_manifest_str(yaml).unwrap();
        assert_eq!(m.external_topics.len(), 4);

        let tf = &m.external_topics["/tf"];
        assert_eq!(tf.side, ExternalSide::Pub);
        assert_eq!(tf.msg_type.as_deref(), Some("tf2_msgs/msg/TFMessage"));

        let engage = &m.external_topics["/vehicle/engage"];
        assert_eq!(engage.side, ExternalSide::Pub);
        assert!(engage.msg_type.is_none());

        let marker = &m.external_topics["/debug/marker"];
        assert_eq!(marker.side, ExternalSide::Sub);

        let passthrough = &m.external_topics["/passthrough/relay"];
        assert_eq!(passthrough.side, ExternalSide::Both);
        let qos = passthrough.qos.as_ref().unwrap();
        assert_eq!(qos.reliability.as_deref(), Some("best_effort"));
    }

    #[test]
    fn test_topic_external_flag() {
        use crate::types::ExternalSide;
        let yaml = r#"
version: 1
nodes:
  consumer:
    sub: [data]
topics:
  /sensor/data:
    type: std_msgs/msg/String
    external: pub
    sub: [consumer/data]
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let topic = &m.topics["/sensor/data"];
        assert_eq!(topic.external, Some(ExternalSide::Pub));
    }

    #[test]
    fn test_external_invalid_side_errors() {
        let yaml = r#"
version: 1
external_topics:
  /tf: { external: maybe }
"#;
        let err = parse_manifest_str(yaml).unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("invalid external side") || msg.contains("'maybe'"),
            "got: {msg}"
        );
    }

    #[test]
    fn test_drop_shorthand() {
        let yaml = r#"
version: 1
topics:
  pointcloud:
    type: sensor_msgs/msg/PointCloud2
    drop: 5 / 100
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let drop = m.topics["pointcloud"].drop.as_ref().unwrap();
        assert_eq!(drop.max_count.as_ref().unwrap().n, 5);
        assert_eq!(drop.max_count.as_ref().unwrap().w, 100);
        assert!(drop.max_consecutive.is_none());
    }
}
