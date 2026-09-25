//! `contract capture` — write a contract's STRUCTURE out of a recorded run.
//!
//! The inverse walk of `scripts/verify_graph.py`: that one grades a model's
//! inferred topic graph against what a run created, this one writes the graph
//! the run created out as a contract file, so an author starting from a system
//! with no contract at all starts from its real wiring rather than a blank
//! file.
//!
//! # What a run can say, and what it cannot
//!
//! Structure and facts are observable. **Requirements are not.** A run can say
//! a topic published at 9.97 Hz; it cannot say whether 10 Hz was *required*,
//! and a contract states requirements. So this emits `nodes:`, their endpoints
//! and `topics:` with types and wiring — and puts every measured number in as
//! a COMMENT. It emits no `rate_hz`, no `min_rate_hz`, no `max_latency`, no
//! `max_age`, no `max_jitter`, no `paths:` and no `criticality`. Same
//! discipline `play_launch measure` follows when it prints costs as comments
//! under a header saying where they belong; the two are meant to be met
//! together.
//!
//! It also emits no CONDITIONS. A run is ONE branch of the launch file, with
//! every `if:`/`unless:` already resolved, and nothing on disk records which
//! way they went. A capture of a system launched twice with different
//! arguments is two different captures, and diffing either against a
//! hand-written contract that carries conditions will show phantom
//! differences. The emitted header says so.
//!
//! # What it reads, under `play_log/<ts>/interception/`
//!
//! | file | for |
//! |---|---|
//! | `endpoints.tsv` | the WIRING, and the message TYPE. Every publisher and subscription CREATED (phase 77), remap-resolved, with `pkg/msg/Name` since issue #0047 |
//! | `node_identity.tsv` | model key → pid → real ROS node name, for the #0017 comment |
//! | `stats_summary.json`, `frontier_summary.json` | the observed counts and rates, and the type as a FALLBACK |
//!
//! The wiring comes from `endpoints.tsv` and only from there: an endpoint
//! exists whether or not a message ever flowed through it, while the summaries
//! are keyed by TRAFFIC and a subscription on a pipeline whose sensor is absent
//! looks there exactly like one that does not exist. The type comes from there
//! FIRST for the same reason — phase 77 measured 982 endpoints created and 63
//! carrying a message on one Autoware run.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

use eyre::{Result, bail};
use ros_launch_manifest_model::SystemModel;

/// Everything `contract capture` reads. Owned plain values — see
/// [`crate::verbs`].
pub struct CaptureInputs {
    /// A `play_log/<timestamp>` directory, its `interception/` subdirectory,
    /// or a file inside it.
    pub run_dir: PathBuf,
    /// The SystemModel the run was launched from. Required for the same
    /// reason `measure` requires one: a contract's node keys have to be the
    /// keys the launch dump uses, and only the model knows them.
    pub model: PathBuf,
    /// Keep `/rosout`, `/parameter_events`, `/tf` and friends, which every
    /// node touches and no launch file describes.
    pub include_infra: bool,
}

/// Topics every ROS node touches that no launch file describes. Same set
/// `verify_graph.py` excludes, for the same reason: comparing them only ever
/// produces noise on both sides, and a contract that declared them would put
/// `/rosout` and `/parameter_events` on all 144 nodes of a real system.
const INFRA_TOPICS: &[&str] = &[
    "/parameter_events",
    "/rosout",
    "/clock",
    "/tf",
    "/tf_static",
];
const INFRA_SUFFIXES: &[&str] = &["/_container/component_events"];

fn is_infra(topic: &str) -> bool {
    INFRA_TOPICS.contains(&topic) || INFRA_SUFFIXES.iter().any(|s| topic.ends_with(s))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Direction {
    /// Sorted before `Pub` deliberately: `sub:` is rendered first, so a
    /// reader meets a node's inputs before its outputs.
    Sub,
    Pub,
}

impl Direction {
    fn as_str(self) -> &'static str {
        match self {
            Direction::Sub => "sub",
            Direction::Pub => "pub",
        }
    }

    fn parse(s: &str) -> Option<Self> {
        match s {
            "pub" => Some(Direction::Pub),
            "sub" => Some(Direction::Sub),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct EndpointRow {
    member: String,
    fqn: String,
    direction: Direction,
    topic: String,
    /// `pkg/msg/Name`, or empty when introspection could not answer (or when
    /// the bundle predates the column).
    msg_type: String,
}

#[derive(Debug, Default, Clone, PartialEq)]
struct TopicFacts {
    msg_type: Option<String>,
    pub_count: Option<f64>,
    take_count: Option<f64>,
    avg_pub_rate_hz: Option<f64>,
    duration_ms: Option<f64>,
}

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

/// Accept a run directory, its `interception/` dir, or a file inside it.
fn run_paths(arg: &Path) -> (PathBuf, PathBuf) {
    let mut run = arg.to_path_buf();
    if run.is_file() {
        run = run.parent().map(Path::to_path_buf).unwrap_or_default();
    }
    if run.file_name().is_some_and(|n| n == "interception") {
        let parent = run.parent().map(Path::to_path_buf).unwrap_or_default();
        (parent, run)
    } else {
        let idir = run.join("interception");
        (run, idir)
    }
}

/// `(member, node FQN, direction, topic, msg type)` rows a run CREATED.
///
/// One line is `member<TAB>pid<TAB>node FQN<TAB>pub|sub<TAB>topic` and, since
/// issue #0047, `<TAB>pkg/msg/Name`. Both names travel because neither alone
/// identifies the node: the member is the model key play_launch spawned the
/// PROCESS under, and under an isolated container every forked composable
/// inherits the container's member.
///
/// Both widths are accepted, discriminated by COLUMN COUNT. Five columns is a
/// bundle written before #0047 and its type is unknown (empty here, which then
/// falls back to the traffic-keyed summaries); six is current, and its sixth
/// field is empty only where introspection could not answer. The count works
/// as a discriminator precisely because the writer always emits the field — an
/// omitted-when-unknown column would make the two widths ambiguous.
fn load_endpoints(body: &str) -> (BTreeSet<EndpointRow>, usize) {
    let mut rows = BTreeSet::new();
    let mut malformed = 0usize;
    for line in body.lines() {
        if line.is_empty() {
            continue;
        }
        let parts: Vec<&str> = line.split('\t').collect();
        let direction = parts.get(3).and_then(|d| Direction::parse(d));
        match (parts.len(), direction) {
            (5 | 6, Some(direction)) => {
                rows.insert(EndpointRow {
                    member: parts[0].to_string(),
                    fqn: parts[2].to_string(),
                    direction,
                    topic: parts[4].to_string(),
                    msg_type: parts.get(5).copied().unwrap_or_default().to_string(),
                });
            }
            _ => malformed += 1,
        }
    }
    (rows, malformed)
}

/// `model key -> {real ROS FQN}`, from the interceptor's own report.
///
/// Authoritative: read off the `rcl_node_t*` the publish/subscribe init hooks
/// already hold, so there is no inference in it. One key maps to several names
/// when one process hosts several nodes (a container).
fn load_identity(body: &str) -> BTreeMap<String, BTreeSet<String>> {
    let mut identity: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for line in body.lines() {
        let parts: Vec<&str> = line.split('\t').collect();
        if parts.len() == 3 {
            identity
                .entry(parts[0].to_string())
                .or_default()
                .insert(parts[2].to_string());
        }
    }
    identity
}

/// Merge `stats_summary.json` and `frontier_summary.json`, both keyed by topic
/// hash with a `name` and an optional `msg_type`.
///
/// Since issue #0047 the type here is a FALLBACK — `endpoints.tsv` carries one
/// per endpoint, and these files only know about topics that carried a
/// message. The measured counts and rates have no other source and still come
/// from here.
fn merge_topic_facts(blobs: &[&str]) -> BTreeMap<String, TopicFacts> {
    let mut facts: BTreeMap<String, TopicFacts> = BTreeMap::new();
    for blob in blobs {
        let Ok(serde_json::Value::Object(map)) = serde_json::from_str::<serde_json::Value>(blob)
        else {
            continue;
        };
        for (key, entry) in map {
            // `_events_dropped_total_best_effort` and friends.
            if key.starts_with('_') {
                continue;
            }
            let Some(entry) = entry.as_object() else {
                continue;
            };
            // A hash whose chunked name never assembled.
            let Some(name) = entry.get("name").and_then(|v| v.as_str()) else {
                continue;
            };
            let rec = facts.entry(name.to_string()).or_default();
            if rec.msg_type.is_none() {
                rec.msg_type = entry
                    .get("msg_type")
                    .and_then(|v| v.as_str())
                    .filter(|s| !s.is_empty())
                    .map(str::to_string);
            }
            let num = |field: &str| entry.get(field).and_then(|v| v.as_f64());
            rec.pub_count = rec.pub_count.or_else(|| num("pub_count"));
            rec.take_count = rec.take_count.or_else(|| num("take_count"));
            rec.avg_pub_rate_hz = rec.avg_pub_rate_hz.or_else(|| num("avg_pub_rate_hz"));
            rec.duration_ms = rec.duration_ms.or_else(|| num("duration_ms"));
        }
    }
    facts
}

/// `(model node keys, keys whose name the launch file did not declare)`.
///
/// `node_name: None` is the discriminator from issue #0017 — exactly the set
/// whose model key is its EXECUTABLE and therefore NOT its ROS name. Those
/// keys get a comment naming the real name the run reported.
fn model_node_keys(model: &SystemModel) -> (BTreeSet<String>, BTreeSet<String>) {
    let mut keys = BTreeSet::new();
    let mut unnamed = BTreeSet::new();
    for (key, node) in &model.structure.nodes {
        keys.insert(key.clone());
        if node.node_name.is_none() {
            unnamed.insert(key.clone());
        }
    }
    (keys, unnamed)
}

// ---------------------------------------------------------------------------
// Joining a run's endpoints onto the model's node keys
// ---------------------------------------------------------------------------

/// Which of the run's two names the CONTRACT may be written in.
///
/// A contract's node key is reconciled against the launch dump, so it has to
/// be a key the model carries. The registered FQN is the ROS truth and wins
/// when the model knows it; a node the launch file did not name registers a
/// FQN the model cannot know, and the model key is the only join there. When
/// neither is a model node the endpoint is REFUSED — claiming it would put a
/// node in the contract that the launch file does not produce.
///
/// Same rule as `verify_graph.py::resolve_node`, minus its fallback: that one
/// is grading and reports the mismatch, this one is WRITING and must not.
fn resolve_node<'a>(
    member: &'a str,
    fqn: &'a str,
    model_nodes: &BTreeSet<String>,
) -> Option<(&'a str, Option<&'a str>)> {
    if model_nodes.contains(fqn) {
        Some((fqn, None))
    } else if model_nodes.contains(member) {
        Some((member, Some(fqn)))
    } else {
        None
    }
}

/// Pick one endpoint name per `(node, direction, topic)`.
///
/// The endpoint name inside a node is arbitrary — the `topics:` block does the
/// wiring, by `<node key>/<endpoint>` ref — so the only requirements are that
/// it is unique within its node and contains no `/`, which would make the ref
/// ambiguous to split. Last topic segment where that is unique, the whole
/// flattened topic path where it is not, and a numeric suffix in the (unseen)
/// case that two topics still flatten alike.
///
/// Uniqueness is across BOTH directions, not per direction: a node that
/// publishes and subscribes the same topic would otherwise give two endpoints
/// one ref.
fn endpoint_names(
    observed: &BTreeMap<String, BTreeSet<(Direction, String)>>,
) -> BTreeMap<(String, Direction, String), String> {
    let sanitize = |s: &str| -> String {
        s.chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() || c == '_' {
                    c
                } else {
                    '_'
                }
            })
            .collect()
    };

    let mut names = BTreeMap::new();
    for (node, entries) in observed {
        let mut taken: BTreeSet<String> = BTreeSet::new();
        // Sorted so the same run always produces the same file.
        for (direction, topic) in entries {
            let trimmed = topic.trim_start_matches('/');
            let mut base = sanitize(trimmed.rsplit('/').next().unwrap_or(""));
            if base.is_empty() || taken.contains(&base) {
                base = sanitize(trimmed);
            }
            let mut candidate = base.clone();
            let mut n = 2;
            while candidate.is_empty() || taken.contains(&candidate) {
                candidate = format!("{base}_{n}");
                n += 1;
            }
            taken.insert(candidate.clone());
            names.insert((node.clone(), *direction, topic.clone()), candidate);
        }
    }
    names
}

// ---------------------------------------------------------------------------
// Cycles
// ---------------------------------------------------------------------------

fn node_of(endpoint_ref: &str) -> &str {
    endpoint_ref.rsplit_once('/').map_or("", |(n, _)| n)
}

/// Drop the fewest subscriber refs that leave the emitted graph acyclic.
///
/// `causal-dag` is the one rule that fires at ERROR severity on a file this
/// shape. The checker builds a vertex per declared endpoint and an edge for
/// every (publisher, subscriber) pair on a topic, and refuses a cycle. Real
/// systems have them — Autoware's planning simulator closes the physical loop,
/// and a controller reading back what it commanded is a feedback pair — so a
/// capture that ignored this would routinely emit a file its own checker
/// rejects, which is the failure this verb exists to avoid.
///
/// The contract's answer to a legitimate cycle is `state: true` on the
/// subscriber that closes it, marking that edge as a stored value rather than
/// a causal dependency. A capture cannot use it, for two reasons and the
/// second is the hard one:
///
/// 1. A RUN CANNOT TELL WHICH EDGE THAT IS. Both look identical on the wire.
///    Guessing would invent a semantic every downstream rule inherits, where
///    an omission only leaves the graph sparse — the same ruling the topic
///    inference makes about an undecidable remap direction.
/// 2. It would not work anyway. MEASURED on the pinned checker: with the
///    absolute node keys this file uses, `state: true` does NOT silence
///    `causal-dag`, because `is_state_endpoint` splits an endpoint ref on its
///    FIRST slash and so looks up a node named "". It does silence the
///    cross-scope `causal-dag-global` warning, whose split is on the last
///    slash. So on this file the flag is half-inert.
///
/// The dropped refs are named in the refusal block, with the cycle they
/// closed. Deterministic: topics are walked in sorted order and the first hop
/// of the discovered cycle, by (topic, ref), is the one cut.
fn break_cycles(
    by_topic: &mut BTreeMap<String, BTreeMap<Direction, Vec<String>>>,
) -> Vec<(String, String, Vec<String>)> {
    let mut dropped = Vec::new();
    loop {
        let mut edges: BTreeMap<&str, BTreeSet<&str>> = BTreeMap::new();
        let mut via: BTreeMap<(&str, &str), Vec<(&str, &str)>> = BTreeMap::new();
        for (topic, sides) in by_topic.iter() {
            let pubs = sides.get(&Direction::Pub);
            let subs = sides.get(&Direction::Sub);
            let (Some(pubs), Some(subs)) = (pubs, subs) else {
                continue;
            };
            for p in pubs {
                for s in subs {
                    let (sp, sn) = (node_of(p), node_of(s));
                    if sp == sn {
                        continue; // a node reading its own output is not an edge
                    }
                    edges.entry(sp).or_default().insert(sn);
                    via.entry((sp, sn))
                        .or_default()
                        .push((topic.as_str(), s.as_str()));
                }
            }
        }

        let Some(cycle) = find_cycle(&edges) else {
            return dropped;
        };
        let mut hops: Vec<(&str, &str)> = Vec::new();
        for (a, b) in cycle.iter().zip(cycle.iter().cycle().skip(1)) {
            if let Some(list) = via.get(&(*a, *b)) {
                hops.extend(list.iter().copied());
            }
        }
        hops.sort();
        let Some((topic, r)) = hops.first().copied() else {
            // No hop carries a ref: nothing can be cut, and looping again
            // would find the same cycle forever.
            return dropped;
        };
        let (topic, r) = (topic.to_string(), r.to_string());
        let cycle: Vec<String> = cycle.into_iter().map(str::to_string).collect();
        if let Some(subs) = by_topic
            .get_mut(&topic)
            .and_then(|s| s.get_mut(&Direction::Sub))
        {
            subs.retain(|x| x != &r);
        }
        dropped.push((topic, r, cycle));
    }
}

/// Any directed cycle, as a list of nodes, or `None`. Iterative depth-first.
fn find_cycle<'a>(edges: &BTreeMap<&'a str, BTreeSet<&'a str>>) -> Option<Vec<&'a str>> {
    #[derive(Clone, Copy, PartialEq)]
    enum Color {
        White,
        Grey,
        Black,
    }
    let mut color: BTreeMap<&str, Color> = BTreeMap::new();
    let empty = BTreeSet::new();
    for root in edges.keys() {
        if color.get(root).copied().unwrap_or(Color::White) != Color::White {
            continue;
        }
        let mut stack: Vec<(&str, std::collections::btree_set::Iter<'_, &str>)> =
            vec![(root, edges.get(root).unwrap_or(&empty).iter())];
        let mut path: Vec<&str> = vec![root];
        color.insert(root, Color::Grey);
        while let Some((node, it)) = stack.last_mut() {
            let node = *node;
            match it.next() {
                None => {
                    color.insert(node, Color::Black);
                    stack.pop();
                    path.pop();
                }
                Some(next) => match color.get(next).copied().unwrap_or(Color::White) {
                    Color::Grey => {
                        let at = path.iter().position(|n| n == next).unwrap_or(0);
                        return Some(path[at..].to_vec());
                    }
                    Color::Black => {}
                    Color::White => {
                        color.insert(next, Color::Grey);
                        path.push(next);
                        stack.push((next, edges.get(next).unwrap_or(&empty).iter()));
                    }
                },
            }
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

/// `(type, note)` for one topic — the endpoint record first.
///
/// The endpoint record (issue #0047) is preferred over the traffic-keyed
/// summaries because it is the only source that knows about an endpoint no
/// message ever crossed, which is the majority of them.
///
/// Disagreement between endpoints on one topic is reported, not hidden. DDS
/// will not match two endpoints whose types differ, so a topic with more than
/// one is a real finding about the system — but it is not a reason to refuse
/// the topic, which would lose the wiring too. One is emitted (the summary's,
/// when it is among them, else the first in sorted order) and the rest are
/// named in a comment.
fn resolve_type(
    seen: Option<&BTreeSet<String>>,
    summary: Option<&str>,
) -> (Option<String>, Option<String>) {
    let seen = match seen {
        Some(s) if !s.is_empty() => s,
        _ => return (summary.map(str::to_string), None),
    };
    if seen.len() == 1 {
        return (seen.iter().next().cloned(), None);
    }
    let chosen = match summary {
        Some(s) if seen.contains(s) => s.to_string(),
        _ => seen.iter().next().cloned().unwrap_or_default(),
    };
    let all: Vec<&str> = seen.iter().map(String::as_str).collect();
    let note = format!(
        "endpoints on this topic reported {} different message types: {}. DDS does not match \
         endpoints whose types differ, so this wiring does not all work; `{chosen}` is emitted \
         and the disagreement is yours to resolve.",
        seen.len(),
        all.join(", ")
    );
    (Some(chosen), Some(note))
}

/// A topic or node key is a ROS name, which YAML reads as a plain scalar.
/// Quoted anyway when it could be read as anything else — an empty string, a
/// name whose characters would start a flow collection or a tag, or (measured
/// on a `<executable>`, whose model key is its whole command line) one
/// carrying spaces.
///
/// The same function quotes an endpoint REF inside a `pub:`/`sub:` flow
/// sequence, where a `,` or a `]` in a key would silently split the list into
/// two refs rather than fail — a wrong wiring is worse than a parse error.
fn yaml_key(name: &str) -> String {
    let plain = !name.is_empty()
        && !name.starts_with('-')
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || "_/~.:-".contains(c));
    if plain {
        name.to_string()
    } else {
        serde_json::to_string(name).unwrap_or_else(|_| format!("\"{name}\""))
    }
}

fn yaml_flow_list(refs: &[String]) -> String {
    refs.iter()
        .map(|r| yaml_key(r))
        .collect::<Vec<_>>()
        .join(", ")
}

struct TopicSpec {
    msg_type: String,
    publishers: Vec<String>,
    subscribers: Vec<String>,
    comments: Vec<String>,
}

struct NodeSpec {
    real_name: Option<String>,
    /// direction → endpoint name → the topic it is wired to
    endpoints: BTreeMap<Direction, BTreeMap<String, String>>,
}

fn header(run: &Path, model: &Path, endpoints: usize, nodes: usize, topics: usize) -> String {
    format!(
        "\
# Captured from a run by `play_launch contract capture` — STRUCTURE ONLY.
#   run:   {run}
#   model: {model}
#   {endpoints} endpoint(s) created, {nodes} node(s), {topics} topic(s) emitted
#
# This file states what the run WIRED. It states no requirement, because a run
# cannot observe one: a measured rate is a fact about one execution, and
# whether that rate is REQUIRED is a decision only you can record. So there is
# no `rate_hz`, no `min_rate_hz`, no `max_latency`, no `max_age`, no
# `max_jitter`, no `paths:` and no `criticality` below — add them, and the
# file becomes a contract instead of a description. Observed numbers are here
# as comments, which is where they belong until you decide otherwise
# (`play_launch measure <run-dir> --model <m.yaml>` does the same for costs).
#
# It also states no CONDITIONS. A run is ONE branch of the launch file, with
# every `if:`/`unless:` already resolved and nothing on disk recording which
# way each went. Launch the same file with different arguments and you get a
# different capture; a contract that should mirror the launch file's structure
# has to have the conditions put back by hand. Do not read a diff against a
# hand-written contract as a disagreement until you have accounted for that.
#
# `type:` comes from the interceptor's introspection of the type support each
# publisher and subscription was CREATED with (issue #0047), so a topic nothing
# ever published on is described here like any other. It falls back to the
# traffic-keyed summaries for a bundle recorded before that column existed; a
# topic whose type is in neither place could not be emitted at all — the
# grammar requires one — and those are listed at the end.
#
# Node keys below are ABSOLUTE FQNs, which the checker passes through verbatim.
# That is the only spelling guaranteed to name the node the run observed: a bare
# name is reconciled through the launch dump, and for a node the launch file did
# not name that lookup MISSES and the key silently resolves to a node that does
# not exist (issue #0048; measured: `talker-1` -> `/talker-1`, with `check`
# reporting clean). One rule reads an absolute ref wrongly and you should know
# which: the checker splits an endpoint ref on its FIRST slash when deciding
# whether `state: true` is set, so that flag has no effect here. Everything
# else — vertex lookup included — matches the ref exactly.
",
        run = run.display(),
        model = model.display(),
    )
}

fn render(
    nodes: &BTreeMap<String, NodeSpec>,
    topics: &BTreeMap<String, TopicSpec>,
    refusals: &BTreeMap<String, BTreeSet<String>>,
    run: &Path,
    model: &Path,
) -> String {
    let endpoint_count: usize = nodes
        .values()
        .map(|n| n.endpoints.values().map(BTreeMap::len).sum::<usize>())
        .sum();
    let mut out = header(run, model, endpoint_count, nodes.len(), topics.len());
    out.push_str("\nversion: 1\n");

    if !nodes.is_empty() {
        out.push_str("\nnodes:\n");
        for (node, spec) in nodes {
            out.push_str(&format!("  {}:\n", yaml_key(node)));
            if let Some(real) = &spec.real_name {
                out.push_str(
                    "    # The launch file did not name this node, so its model key is its\n\
                     \x20   # EXECUTABLE and is NOT its ROS name (issue #0017). The run registered\n",
                );
                out.push_str(&format!(
                    "    # it as {real}. The key below is the one the contract\n"
                ));
                out.push_str("    # must use — it is the one the launch dump knows.\n");
            }
            for direction in [Direction::Sub, Direction::Pub] {
                let Some(eps) = spec.endpoints.get(&direction) else {
                    continue;
                };
                if eps.is_empty() {
                    continue;
                }
                out.push_str(&format!("    {}:\n", direction.as_str()));
                for (name, topic) in eps {
                    out.push_str(&format!("      # -> {topic}\n"));
                    out.push_str(&format!("      {name}: {{}}\n"));
                }
            }
        }
    }

    if !topics.is_empty() {
        out.push_str("\ntopics:\n");
        for (topic, spec) in topics {
            out.push_str(&format!("  {}:\n", yaml_key(topic)));
            for line in &spec.comments {
                out.push_str(&format!("    # {line}\n"));
            }
            out.push_str(&format!("    type: {}\n", spec.msg_type));
            out.push_str(&format!(
                "    pub: [{}]\n",
                yaml_flow_list(&spec.publishers)
            ));
            out.push_str(&format!(
                "    sub: [{}]\n",
                yaml_flow_list(&spec.subscribers)
            ));
        }
    }

    if !refusals.is_empty() {
        out.push_str(
            "\n# ------------------------------------------------------------------\n\
             # NOT emitted. Each of these is an observation this file cannot honestly\n\
             # carry; none is a defect on its own.\n\
             # ------------------------------------------------------------------\n",
        );
        for (reason, items) in refusals {
            out.push_str(&format!("#\n# {reason} ({}):\n", items.len()));
            for item in items.iter().take(60) {
                out.push_str(&format!("#   {item}\n"));
            }
            if items.len() > 60 {
                out.push_str(&format!("#   ... and {} more\n", items.len() - 60));
            }
        }
    }

    out
}

// ---------------------------------------------------------------------------
// The verb
// ---------------------------------------------------------------------------

pub fn capture(inputs: CaptureInputs) -> Result<String> {
    let (run, idir) = run_paths(&inputs.run_dir);
    let ep_path = idir.join("endpoints.tsv");
    if !ep_path.is_file() {
        bail!(
            "no endpoints.tsv under {}.\n\
             The wiring can only come from the endpoints a run CREATED, and only the \
             interceptor records those. Re-run with interception enabled \
             (`interception: {{enabled: true}}` in --config) and try again.",
            idir.display()
        );
    }

    let model_text = std::fs::read_to_string(&inputs.model)
        .map_err(|e| eyre::eyre!("reading {}: {e}", inputs.model.display()))?;
    let model = SystemModel::from_yaml_str(&model_text)
        .map_err(|e| eyre::eyre!("parsing {}: {e}", inputs.model.display()))?;
    let (model_nodes, model_unnamed) = model_node_keys(&model);

    let ep_body = std::fs::read_to_string(&ep_path)
        .map_err(|e| eyre::eyre!("reading {}: {e}", ep_path.display()))?;
    let (rows, malformed) = load_endpoints(&ep_body);
    let identity = std::fs::read_to_string(idir.join("node_identity.tsv"))
        .map(|b| load_identity(&b))
        .unwrap_or_default();
    let summaries: Vec<String> = ["stats_summary.json", "frontier_summary.json"]
        .iter()
        .filter_map(|f| std::fs::read_to_string(idir.join(f)).ok())
        .collect();
    let facts = merge_topic_facts(&summaries.iter().map(String::as_str).collect::<Vec<_>>());

    let text = build(
        &rows,
        malformed,
        &identity,
        &facts,
        &model_nodes,
        &model_unnamed,
        inputs.include_infra,
        &run,
        &inputs.model,
    )?;
    Ok(text)
}

/// The whole of the capture, on values rather than files — so the tests can
/// exercise it without a bundle on disk, and the verb above is only I/O.
#[allow(clippy::too_many_arguments)]
fn build(
    rows: &BTreeSet<EndpointRow>,
    malformed: usize,
    identity: &BTreeMap<String, BTreeSet<String>>,
    facts: &BTreeMap<String, TopicFacts>,
    model_nodes: &BTreeSet<String>,
    model_unnamed: &BTreeSet<String>,
    include_infra: bool,
    run: &Path,
    model_path: &Path,
) -> Result<String> {
    let mut refusals: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    if malformed > 0 {
        refusals
            .entry(format!("{malformed} unparseable line(s) in endpoints.tsv"))
            .or_default()
            .insert("a truncated last line is expected on a killed run".to_string());
    }

    // 1. Join every observed endpoint onto a model node key.
    let mut observed: BTreeMap<String, BTreeSet<(Direction, String)>> = BTreeMap::new();
    let mut real_names: BTreeMap<String, String> = BTreeMap::new();
    let mut endpoint_types: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for row in rows {
        if is_infra(&row.topic) && !include_infra {
            continue;
        }
        // The type is a property of the TOPIC, so it is collected before the
        // node join: an endpoint on a node the contract may not claim still
        // tells you what the topic carries.
        if !row.msg_type.is_empty() {
            endpoint_types
                .entry(row.topic.clone())
                .or_default()
                .insert(row.msg_type.clone());
        }
        let Some((node, aka)) = resolve_node(&row.member, &row.fqn, model_nodes) else {
            refusals
                .entry(
                    "endpoints on a node the model does not carry — a process play_launch \
                     spawned can host nodes the launch file never named (a container's own \
                     node, an rclcpp-internal one), and a contract may not claim one"
                        .to_string(),
                )
                .or_default()
                .insert(format!(
                    "{} (member {})  {}  {}",
                    row.fqn,
                    row.member,
                    row.direction.as_str(),
                    row.topic
                ));
            continue;
        };
        observed
            .entry(node.to_string())
            .or_default()
            .insert((row.direction, row.topic.clone()));
        // The #0017 comment is only honest when the model key names exactly
        // one running node. A container's model key maps to every composable
        // it forked, and naming one of them "the real name of this node" would
        // be a false claim about the other four.
        if let Some(aka) = aka
            && model_unnamed.contains(node)
            && identity.get(node).is_some_and(|names| names.len() == 1)
        {
            real_names.insert(node.to_string(), aka.to_string());
        }
    }

    if observed.is_empty() {
        bail!(
            "no endpoint in this run joins to a node in {}.\n\
             Either the run and the model are from different launches, or every endpoint was \
             on a node play_launch did not spawn.",
            model_path.display()
        );
    }

    let names = endpoint_names(&observed);

    // 2. Group by topic, and refuse any topic whose type never reached disk.
    //    `type:` is mandatory in the grammar, so an untyped topic is not a
    //    degraded entry — it is an unparseable file.
    let mut by_topic: BTreeMap<String, BTreeMap<Direction, Vec<String>>> = BTreeMap::new();
    for (node, entries) in &observed {
        for (direction, topic) in entries {
            let name = &names[&(node.clone(), *direction, topic.clone())];
            by_topic
                .entry(topic.clone())
                .or_default()
                .entry(*direction)
                .or_default()
                .push(format!("{node}/{name}"));
        }
    }
    for sides in by_topic.values_mut() {
        for refs in sides.values_mut() {
            refs.sort();
        }
    }

    let mut cycle_cut: BTreeSet<String> = BTreeSet::new();
    for (topic, r, cycle) in break_cycles(&mut by_topic) {
        cycle_cut.insert(topic.clone());
        refusals
            .entry(
                "subscriber endpoints CUT to break a causal cycle — `causal-dag` refuses a \
                 cyclic dataflow graph outright (an error), and a cycle is legitimate only \
                 when one edge carries a stored value rather than a causal dependency \
                 (`state: true` on that subscriber). A run cannot tell which edge that is, \
                 and on this file's absolute node keys the flag would not take effect anyway \
                 — the checker's `is_state_endpoint` splits a ref on its FIRST slash. Decide \
                 which edge is stored, shorten that node's key to a bare name, and put the \
                 edge back with `state: true`"
                    .to_string(),
            )
            .or_default()
            .insert(format!(
                "{r}  on {topic}   (cycle: {} -> {})",
                cycle.join(" -> "),
                cycle[0]
            ));
    }

    let mut topics: BTreeMap<String, TopicSpec> = BTreeMap::new();
    let mut emitted: BTreeSet<(String, Direction, String)> = BTreeSet::new();
    for (topic, sides) in &by_topic {
        let empty = Vec::new();
        let pubs = sides.get(&Direction::Pub).unwrap_or(&empty);
        let subs = sides.get(&Direction::Sub).unwrap_or(&empty);
        let fact = facts.get(topic).cloned().unwrap_or_default();
        let (msg_type, type_note) =
            resolve_type(endpoint_types.get(topic), fact.msg_type.as_deref());
        let Some(msg_type) = msg_type else {
            refusals
                .entry(
                    "topics whose message type is nowhere on disk — since issue #0047 the \
                     type is recorded per ENDPOINT, off the type support the init hook holds, \
                     so this is no longer the created-but-never-exercised case. It is either \
                     a bundle recorded before that change (a five-column endpoints.tsv, whose \
                     only type source is the traffic-keyed summaries) or an endpoint whose \
                     type support introspection could not read. Fill these in by hand \
                     (`ros2 topic info -v`), or re-record"
                        .to_string(),
                )
                .or_default()
                .insert(format!("{topic}  ({} pub, {} sub)", pubs.len(), subs.len()));
            continue;
        };

        let mut comments = Vec::new();
        if let Some(note) = type_note {
            comments.push(note);
        }
        if let Some(pub_count) = fact.pub_count {
            comments.push(format!(
                "observed: {} published, {} taken, {} Hz mean over {:.1} s. A FACT about this \
                 run, not a requirement — if a rate is required, say so with `rate_hz:` and \
                 the tool will check it.",
                pub_count as u64,
                fact.take_count.unwrap_or(0.0) as u64,
                fact.avg_pub_rate_hz
                    .map(|v| format!("{v:.2}"))
                    .unwrap_or_else(|| "?".to_string()),
                fact.duration_ms.unwrap_or(0.0) / 1000.0,
            ));
        }
        if pubs.is_empty() {
            comments.push(
                "nothing in this run published it. Either a publisher outside the launch \
                 (`external: pub`) or a node that did not start — the run cannot tell you \
                 which."
                    .to_string(),
            );
        }
        if cycle_cut.contains(topic) {
            comments.push(
                "a subscriber on this topic was CUT to break a causal cycle — see the end of \
                 this file. The wiring below is deliberately less than what the run created."
                    .to_string(),
            );
        } else if subs.is_empty() {
            comments.push(
                "nothing in this run subscribed to it. Either a consumer outside the launch \
                 (`external: sub`) or one that did not start."
                    .to_string(),
            );
        }

        for (direction, refs) in [(Direction::Pub, pubs), (Direction::Sub, subs)] {
            for r in refs {
                emitted.insert((r.clone(), direction, topic.clone()));
            }
        }
        topics.insert(
            topic.clone(),
            TopicSpec {
                msg_type,
                publishers: pubs.clone(),
                subscribers: subs.clone(),
                comments,
            },
        );
    }

    // 3. Nodes carry only the endpoints whose topic survived step 2: an
    //    endpoint ref pointing at a topic that is not declared is a dangling
    //    reference, which is a worse output than an omission.
    let mut nodes: BTreeMap<String, NodeSpec> = BTreeMap::new();
    for (node, entries) in &observed {
        let mut endpoints: BTreeMap<Direction, BTreeMap<String, String>> = BTreeMap::new();
        for (direction, topic) in entries {
            let name = &names[&(node.clone(), *direction, topic.clone())];
            if !emitted.contains(&(format!("{node}/{name}"), *direction, topic.clone())) {
                continue;
            }
            endpoints
                .entry(*direction)
                .or_default()
                .insert(name.clone(), topic.clone());
        }
        if endpoints.is_empty() {
            continue;
        }
        nodes.insert(
            node.clone(),
            NodeSpec {
                real_name: real_names.get(node).cloned(),
                endpoints,
            },
        );
    }

    if topics.is_empty() {
        bail!(
            "nothing could be emitted from {}: no topic in it has a message type on disk.\n\
             Re-record the run with a current build — since issue #0047 the type is written \
             per endpoint, so even a silent one is describable.",
            run.display()
        );
    }

    Ok(render(&nodes, &topics, &refusals, run, model_path))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(member: &str, fqn: &str, d: Direction, topic: &str, ty: &str) -> EndpointRow {
        EndpointRow {
            member: member.to_string(),
            fqn: fqn.to_string(),
            direction: d,
            topic: topic.to_string(),
            msg_type: ty.to_string(),
        }
    }

    fn model_keys(keys: &[&str]) -> BTreeSet<String> {
        keys.iter().map(|s| s.to_string()).collect()
    }

    fn build_of(rows: &[EndpointRow], model: &[&str]) -> String {
        build(
            &rows.iter().cloned().collect(),
            0,
            &BTreeMap::new(),
            &BTreeMap::new(),
            &model_keys(model),
            &BTreeSet::new(),
            false,
            Path::new("play_log/run"),
            Path::new("m.yaml"),
        )
        .expect("capture")
    }

    /// The whole gate in one assertion: the emitted text must parse as a
    /// MANIFEST, not merely as YAML. Same lesson as `measure`'s
    /// `the_fragment_is_accepted_by_the_platform_file_loader` — "it parses" is
    /// a claim about the consumer, so only the consumer can check it.
    fn parses_as_a_manifest(text: &str) -> ros_launch_manifest_types::Manifest {
        ros_launch_manifest_types::parse_manifest_str(text)
            .unwrap_or_else(|e| panic!("capture's own output must parse:\n{e}\n---\n{text}"))
    }

    #[test]
    fn the_output_parses_as_a_manifest() {
        let text = build_of(
            &[
                row(
                    "/talker",
                    "/talker",
                    Direction::Pub,
                    "/chatter",
                    "std_msgs/msg/String",
                ),
                row(
                    "/listener",
                    "/listener",
                    Direction::Sub,
                    "/chatter",
                    "std_msgs/msg/String",
                ),
            ],
            &["/talker", "/listener"],
        );
        let manifest = parses_as_a_manifest(&text);
        assert_eq!(manifest.nodes.len(), 2);
        assert_eq!(manifest.topics.len(), 1);
        let topic = manifest.topics.get("/chatter").expect("topic emitted");
        assert_eq!(topic.msg_type, "std_msgs/msg/String");
    }

    /// The capability #0047 unlocked: an endpoint that was created and never
    /// used has a type in `endpoints.tsv` and appears in no summary at all.
    #[test]
    fn a_silent_endpoint_is_emitted_from_the_endpoint_record_alone() {
        let text = build_of(
            &[
                row(
                    "/silent",
                    "/silent",
                    Direction::Pub,
                    "/silent_topic",
                    "std_msgs/msg/String",
                ),
                row(
                    "/silent",
                    "/silent",
                    Direction::Sub,
                    "/silent_input",
                    "std_msgs/msg/Int32",
                ),
            ],
            &["/silent"],
        );
        let manifest = parses_as_a_manifest(&text);
        assert_eq!(
            manifest.topics["/silent_topic"].msg_type,
            "std_msgs/msg/String"
        );
        assert_eq!(
            manifest.topics["/silent_input"].msg_type,
            "std_msgs/msg/Int32"
        );
        // No traffic was recorded, so no observed-rate comment may appear:
        // a topic nothing crossed is in no summary at all.
        assert!(!text.contains("Hz mean"), "{text}");
        // ...and both sides of each topic say what is missing rather than
        // leaving an empty list unexplained.
        assert!(
            text.contains("nothing in this run subscribed to it"),
            "{text}"
        );
        assert!(text.contains("nothing in this run published it"), "{text}");
    }

    /// A five-column bundle (pre-#0047) still captures what the summaries know.
    #[test]
    fn the_summary_type_is_the_fallback_for_an_old_bundle() {
        let (rows, malformed) = load_endpoints("/talker\t42\t/talker\tpub\t/chatter\n");
        assert_eq!(malformed, 0);
        let mut facts = BTreeMap::new();
        facts.insert(
            "/chatter".to_string(),
            TopicFacts {
                msg_type: Some("std_msgs/msg/String".to_string()),
                pub_count: Some(29.0),
                take_count: Some(29.0),
                avg_pub_rate_hz: Some(1.03),
                duration_ms: Some(28_000.0),
                ..Default::default()
            },
        );
        let text = build(
            &rows,
            0,
            &BTreeMap::new(),
            &facts,
            &model_keys(&["/talker"]),
            &BTreeSet::new(),
            false,
            Path::new("play_log/run"),
            Path::new("m.yaml"),
        )
        .unwrap();
        parses_as_a_manifest(&text);
        assert!(text.contains("type: std_msgs/msg/String"), "{text}");
        // The measured numbers are FACTS, and stay comments.
        let observed = text
            .lines()
            .find(|l| l.contains("observed: 29 published"))
            .expect("observed line");
        assert!(observed.trim_start().starts_with('#'), "{observed}");
        assert!(observed.contains("1.03 Hz"), "{observed}");
    }

    /// The discipline, asserted as an absence: no requirement key anywhere.
    #[test]
    fn no_requirement_is_ever_emitted() {
        let mut facts = BTreeMap::new();
        facts.insert(
            "/chatter".to_string(),
            TopicFacts {
                msg_type: Some("std_msgs/msg/String".to_string()),
                pub_count: Some(100.0),
                avg_pub_rate_hz: Some(9.97),
                duration_ms: Some(10_000.0),
                ..Default::default()
            },
        );
        let rows: BTreeSet<EndpointRow> = [
            row(
                "/talker",
                "/talker",
                Direction::Pub,
                "/chatter",
                "std_msgs/msg/String",
            ),
            row(
                "/listener",
                "/listener",
                Direction::Sub,
                "/chatter",
                "std_msgs/msg/String",
            ),
        ]
        .into_iter()
        .collect();
        let text = build(
            &rows,
            0,
            &BTreeMap::new(),
            &facts,
            &model_keys(&["/talker", "/listener"]),
            &BTreeSet::new(),
            false,
            Path::new("play_log/run"),
            Path::new("m.yaml"),
        )
        .unwrap();
        // The observed rate is present as prose and absent as a field: the
        // parsed manifest is the only place that distinction is checkable.
        assert!(text.contains("9.97 Hz mean"), "{text}");
        let manifest = parses_as_a_manifest(&text);
        let topic = manifest.topics.get("/chatter").unwrap();
        assert_eq!(topic.rate_hz, None, "a measured rate is not a requirement");
        for key in [
            "rate_hz:",
            "min_rate_hz:",
            "max_latency",
            "max_age",
            "max_jitter",
            "criticality",
            "paths:",
        ] {
            let offending: Vec<&str> = text
                .lines()
                .filter(|l| l.contains(key) && !l.trim_start().starts_with('#'))
                .collect();
            assert!(
                offending.is_empty(),
                "{key} emitted as a field: {offending:?}"
            );
        }
    }

    /// An endpoint on a node the model does not carry is refused BY NAME —
    /// and the topic it sits on still takes its TYPE from it, because the type
    /// is a property of the topic and the refusal is about the node.
    #[test]
    fn an_unmodelled_node_is_refused_with_a_reason() {
        let text = build_of(
            &[
                // The modelled endpoint has no type of its own (introspection
                // could not answer); the refused one does.
                row("/talker", "/talker", Direction::Pub, "/chatter", ""),
                row(
                    "/ghost",
                    "/ghost/_internal",
                    Direction::Sub,
                    "/chatter",
                    "std_msgs/msg/String",
                ),
            ],
            &["/talker"],
        );
        let manifest = parses_as_a_manifest(&text);
        assert!(
            text.contains("endpoints on a node the model does not carry"),
            "{text}"
        );
        assert!(text.contains("/ghost/_internal (member /ghost)"), "{text}");
        assert_eq!(manifest.topics["/chatter"].msg_type, "std_msgs/msg/String");
        // ...and the refused endpoint is NOT wired in.
        assert_eq!(manifest.topics["/chatter"].subscribers.len(), 0, "{text}");
        assert!(!manifest.nodes.contains_key("/ghost"), "{text}");
    }

    /// Infra topics are dropped by default and kept on request.
    #[test]
    fn infra_topics_are_excluded_unless_asked_for() {
        let rows: BTreeSet<EndpointRow> = [
            row(
                "/talker",
                "/talker",
                Direction::Pub,
                "/rosout",
                "rcl_interfaces/msg/Log",
            ),
            row(
                "/talker",
                "/talker",
                Direction::Pub,
                "/chatter",
                "std_msgs/msg/String",
            ),
        ]
        .into_iter()
        .collect();
        let without = build(
            &rows,
            0,
            &BTreeMap::new(),
            &BTreeMap::new(),
            &model_keys(&["/talker"]),
            &BTreeSet::new(),
            false,
            Path::new("r"),
            Path::new("m.yaml"),
        )
        .unwrap();
        assert!(!without.contains("/rosout"), "{without}");
        let with = build(
            &rows,
            0,
            &BTreeMap::new(),
            &BTreeMap::new(),
            &model_keys(&["/talker"]),
            &BTreeSet::new(),
            true,
            Path::new("r"),
            Path::new("m.yaml"),
        )
        .unwrap();
        assert!(with.contains("/rosout"), "{with}");
    }

    /// A cycle is CUT, named, and the emitted graph is acyclic — the file the
    /// capture writes must pass its own checker's `causal-dag`.
    #[test]
    fn a_cycle_is_cut_and_named() {
        let t = "std_msgs/msg/String";
        let mut rows = Vec::new();
        for (a, b) in [("/a", "/ab"), ("/b", "/bc"), ("/c", "/ca")] {
            rows.push(row(a, a, Direction::Pub, b, t));
        }
        for (n, topic) in [("/b", "/ab"), ("/c", "/bc"), ("/a", "/ca")] {
            rows.push(row(n, n, Direction::Sub, topic, t));
        }
        let text = build_of(&rows, &["/a", "/b", "/c"]);
        let manifest = parses_as_a_manifest(&text);
        assert!(text.contains("CUT to break a causal cycle"), "{text}");
        assert!(text.contains("(cycle: /a -> /b -> /c -> /a)"), "{text}");
        // Exactly one subscriber lost, and it is the deterministic one.
        let sub_count: usize = manifest.topics.values().map(|t| t.subscribers.len()).sum();
        assert_eq!(sub_count, 2, "{text}");
        assert_eq!(manifest.topics["/ab"].subscribers.len(), 0, "{text}");
        assert!(
            text.contains("a subscriber on this topic was CUT"),
            "{text}"
        );
    }

    /// The #0017 comment names the real node, and the key stays the model's.
    #[test]
    fn an_unnamed_node_keeps_its_model_key_and_says_why() {
        let rows: BTreeSet<EndpointRow> = [row(
            "/identity_test/talker-1",
            "/identity_test/talker",
            Direction::Pub,
            "/chatter",
            "std_msgs/msg/String",
        )]
        .into_iter()
        .collect();
        let mut identity = BTreeMap::new();
        identity.insert(
            "/identity_test/talker-1".to_string(),
            ["/identity_test/talker".to_string()]
                .into_iter()
                .collect::<BTreeSet<_>>(),
        );
        let text = build(
            &rows,
            0,
            &identity,
            &BTreeMap::new(),
            &model_keys(&["/identity_test/talker-1"]),
            &model_keys(&["/identity_test/talker-1"]),
            false,
            Path::new("r"),
            Path::new("m.yaml"),
        )
        .unwrap();
        let manifest = parses_as_a_manifest(&text);
        assert!(
            manifest.nodes.contains_key("/identity_test/talker-1"),
            "{text}"
        );
        assert!(text.contains("issue #0017"), "{text}");
        assert!(
            text.contains("The run registered\n    # it as /identity_test/talker."),
            "{text}"
        );
    }

    /// A container's model key hosts several nodes, so no single "real name"
    /// claim is honest for it.
    #[test]
    fn a_container_key_gets_no_real_name_comment() {
        let rows: BTreeSet<EndpointRow> = [row(
            "/container",
            "/comp_a",
            Direction::Pub,
            "/chatter",
            "std_msgs/msg/String",
        )]
        .into_iter()
        .collect();
        let mut identity = BTreeMap::new();
        identity.insert(
            "/container".to_string(),
            ["/comp_a".to_string(), "/comp_b".to_string()]
                .into_iter()
                .collect::<BTreeSet<_>>(),
        );
        let text = build(
            &rows,
            0,
            &identity,
            &BTreeMap::new(),
            &model_keys(&["/container"]),
            &model_keys(&["/container"]),
            false,
            Path::new("r"),
            Path::new("m.yaml"),
        )
        .unwrap();
        assert!(!text.contains("issue #0017"), "{text}");
    }

    /// Endpoint names are unique within a node and never contain a `/`, which
    /// would make the `<node>/<endpoint>` ref ambiguous to split.
    #[test]
    fn endpoint_names_collide_into_flattened_paths() {
        let t = "std_msgs/msg/String";
        let text = build_of(
            &[
                row("/n", "/n", Direction::Pub, "/a/data", t),
                row("/n", "/n", Direction::Pub, "/b/data", t),
                // Same topic on both sides: one ref each, never the same one.
                row("/n", "/n", Direction::Sub, "/a/data", t),
            ],
            &["/n"],
        );
        let manifest = parses_as_a_manifest(&text);
        let node = &manifest.nodes["/n"];
        let mut all: Vec<&String> = node
            .publishers
            .keys()
            .chain(node.subscribers.keys())
            .collect();
        let count = all.len();
        all.sort();
        all.dedup();
        assert_eq!(all.len(), count, "endpoint names must be unique: {text}");
        assert!(all.iter().all(|n| !n.contains('/')), "{all:?}");
    }

    /// A `<executable>`'s model key is its whole command line — spaces,
    /// slashes and all — so the ref that names it must survive a flow
    /// sequence. Measured on the `silent_endpoint` fixture.
    #[test]
    fn a_key_with_spaces_is_quoted_on_both_sides() {
        let key = "/python3 /tmp/a, b/silent_publisher.py";
        let text = build_of(
            &[row(
                key,
                "/silent_pub",
                Direction::Pub,
                "/silent_topic",
                "a/msg/A",
            )],
            &[key],
        );
        let manifest = parses_as_a_manifest(&text);
        assert_eq!(
            manifest.topics["/silent_topic"].publishers,
            vec![format!("{key}/silent_topic")],
            "a comma in a node key must not split the flow list: {text}"
        );
        assert!(manifest.nodes.contains_key(key), "{text}");
    }

    #[test]
    fn a_malformed_line_is_counted_not_fatal() {
        let (rows, malformed) = load_endpoints(
            "/talker\t42\t/talker\tpub\t/chatter\tstd_msgs/msg/String\n\
             garbage\n\
             /talker\t42\t/talker\tsideways\t/chatter\tstd_msgs/msg/String\n",
        );
        assert_eq!(rows.len(), 1);
        assert_eq!(malformed, 2);
    }

    #[test]
    fn a_type_disagreement_is_reported_not_hidden() {
        let (chosen, note) = resolve_type(
            Some(
                &["a/msg/A".to_string(), "b/msg/B".to_string()]
                    .into_iter()
                    .collect(),
            ),
            Some("b/msg/B"),
        );
        assert_eq!(chosen.as_deref(), Some("b/msg/B"));
        assert!(note.unwrap().contains("DDS does not match"));
    }

    #[test]
    fn an_untyped_topic_is_refused_rather_than_emitted_without_a_type() {
        let text = build_of(
            &[
                row(
                    "/talker",
                    "/talker",
                    Direction::Pub,
                    "/typed",
                    "std_msgs/msg/String",
                ),
                row("/talker", "/talker", Direction::Pub, "/untyped", ""),
            ],
            &["/talker"],
        );
        let manifest = parses_as_a_manifest(&text);
        assert!(manifest.topics.contains_key("/typed"));
        assert!(!manifest.topics.contains_key("/untyped"));
        assert!(
            text.contains("topics whose message type is nowhere on disk"),
            "{text}"
        );
        // And the node must not keep an endpoint pointing at a topic that is
        // not declared — a dangling ref is worse than an omission.
        assert!(
            !manifest.nodes["/talker"].publishers.contains_key("untyped"),
            "{text}"
        );
    }

    #[test]
    fn a_run_whose_endpoints_match_no_model_node_says_so() {
        let err = build(
            &[row("/x", "/x", Direction::Pub, "/t", "a/msg/A")]
                .into_iter()
                .collect(),
            0,
            &BTreeMap::new(),
            &BTreeMap::new(),
            &model_keys(&["/y"]),
            &BTreeSet::new(),
            false,
            Path::new("r"),
            Path::new("m.yaml"),
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("different launches"), "{err}");
    }

    #[test]
    fn run_paths_accepts_the_run_dir_and_the_interception_dir() {
        let (run, idir) = run_paths(Path::new("play_log/ts"));
        assert_eq!(idir, Path::new("play_log/ts/interception"));
        assert_eq!(run, Path::new("play_log/ts"));
        let (run, idir) = run_paths(Path::new("play_log/ts/interception"));
        assert_eq!(idir, Path::new("play_log/ts/interception"));
        assert_eq!(run, Path::new("play_log/ts"));
    }

    #[test]
    fn the_header_names_what_it_refuses_to_state() {
        let text = build_of(&[row("/n", "/n", Direction::Pub, "/t", "a/msg/A")], &["/n"]);
        assert!(text.contains("STRUCTURE ONLY"), "{text}");
        assert!(text.contains("states no requirement"), "{text}");
        assert!(text.contains("no CONDITIONS"), "{text}");
        assert!(text.contains("ABSOLUTE FQNs"), "{text}");
    }
}
