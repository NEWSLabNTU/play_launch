//! A link's transport, as one run observed it (phase 97 P3/P4, rlm
//! `docs/design/links.md` L3/L4).
//!
//! The interception record (`interception/events.jsonl`) carries, per
//! message, the topic, the `header.stamp` and `CLOCK_MONOTONIC` at the
//! publish and at each take. A take's delta from the publish of the same
//! stamp on the same topic is that message's transport. Both ends are
//! processes play_launch ran on this host, so the two monotonic readings are
//! comparable; a subscriber off this host (a board behind a serial link)
//! takes nothing the record sees, and its link reads as unmeasured -- the
//! honest answer, since across hosts the clocks are not comparable.
//!
//! Two callers: `check --against` (`transport-exceeded`, `link-unmeasured`)
//! and `measure`'s link proposals. Each builds the [`LinkEdge`]s from what it
//! has (the manifest index, or the model).

use std::{
    collections::{BTreeMap, HashMap},
    path::{Path, PathBuf},
};

/// One edge that crosses a declared link: the hop into `sub_node` on `topic`.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkEdge {
    pub link: String,
    pub topic: String,
    /// The subscriber's node FQN, as the interception record names the
    /// node that took (a composable node in a container is named by the
    /// container's ring, so its takes are not attributed: unmeasured).
    pub sub_node: String,
    /// The link's `max_transport`, ms.
    pub bound_ms: f64,
}

/// What a run showed for one link.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinkObservation {
    /// One transport per matched take, ms.
    pub samples_ms: Vec<f64>,
    /// Takes over the bound, as `(topic, subscriber node, ms)`.
    pub over: Vec<(String, String, f64)>,
}

impl LinkObservation {
    pub fn max_ms(&self) -> Option<f64> {
        self.samples_ms.iter().copied().reduce(f64::max)
    }
}

/// `events.jsonl` under a run directory, the interception dir, or the path
/// itself.
pub fn events_path(arg: &Path) -> Option<PathBuf> {
    [
        arg.to_path_buf(),
        arg.join("events.jsonl"),
        arg.join("interception/events.jsonl"),
    ]
    .into_iter()
    .find(|c| c.is_file())
}

/// Every link's samples in `events` (the `events.jsonl` text), keyed by link
/// name. A link with no matched take is present with no samples.
pub fn observe(events: &str, edges: &[LinkEdge]) -> BTreeMap<String, LinkObservation> {
    let mut out: BTreeMap<String, LinkObservation> = edges
        .iter()
        .map(|e| (e.link.clone(), LinkObservation::default()))
        .collect();
    let mut topics: HashMap<u64, String> = HashMap::new();
    // (topic hash, stamp) -> earliest publish, ns
    let mut published: HashMap<(u64, u64), u64> = HashMap::new();
    // (topic hash, node, stamp, take ns)
    let mut takes: Vec<(u64, String, u64, u64)> = Vec::new();
    for line in events.lines() {
        let Ok(v) = serde_json::from_str::<serde_json::Value>(line) else {
            continue;
        };
        if let Some(t) = v.get("t") {
            if let (Some(h), Some(n)) = (t["h"].as_u64(), t["n"].as_str()) {
                topics.insert(h, n.to_string());
            }
            continue;
        }
        let Some(e) = v.get("e") else { continue };
        let (Some(h), Some(t), Some(d)) = (e["h"].as_u64(), e["t"].as_u64(), e["d"].as_str())
        else {
            continue;
        };
        let (s, ns) = (e["s"].as_i64().unwrap_or(0), e["ns"].as_u64().unwrap_or(0));
        if s == 0 && ns == 0 {
            continue; // unstamped: no identity to pair by
        }
        let stamp = ((s as u32 as u64) << 32) | (ns & 0xffff_ffff);
        match d {
            "pub" => {
                published
                    .entry((h, stamp))
                    .and_modify(|p| *p = (*p).min(t))
                    .or_insert(t);
            }
            "take" => takes.push((h, e["n"].as_str().unwrap_or("").to_string(), stamp, t)),
            _ => {}
        }
    }
    for (h, node, stamp, t) in takes {
        let Some(topic) = topics.get(&h) else {
            continue;
        };
        let Some(edge) = edges
            .iter()
            .find(|e| &e.topic == topic && e.sub_node == node)
        else {
            continue;
        };
        let Some(p) = published.get(&(h, stamp)) else {
            continue;
        };
        if t < *p {
            continue;
        }
        let ms = (t - p) as f64 / 1e6;
        let o = out.entry(edge.link.clone()).or_default();
        o.samples_ms.push(ms);
        if ms > edge.bound_ms {
            o.over.push((topic.clone(), node, ms));
        }
    }
    out
}

/// The edges the contracts route over a declared link, from the index: a
/// subscriber that names a link, on each topic that lists it.
pub fn link_edges(index: &super::manifest_loader::ManifestIndex) -> Vec<LinkEdge> {
    // (node FQN, endpoint) -> (link, bound)
    let mut linked: HashMap<(String, String), (String, f64)> = HashMap::new();
    for m in index.manifests.values() {
        for (node, decl) in &m.manifest.nodes {
            let fqn = super::manifest_loader::resolve_node_fqn(index, m.scope_id, &m.ns, node);
            for (ep, props) in &decl.subscribers {
                if let (Some(link), Some(d)) = (&props.link, props.max_transport.as_ref()) {
                    let ms = d.ms().unwrap_or(0.0);
                    linked.insert((fqn.clone(), ep.clone()), (link.clone(), ms));
                }
            }
        }
    }
    let mut out = Vec::new();
    for (topic, t) in &index.topics {
        for r in &t.subscribers {
            let Some((node, ep)) = r.rsplit_once('/') else {
                continue;
            };
            if let Some((link, ms)) = linked.get(&(node.to_string(), ep.to_string())) {
                out.push(LinkEdge {
                    link: link.clone(),
                    topic: topic.clone(),
                    sub_node: node.to_string(),
                    bound_ms: *ms,
                });
            }
        }
    }
    out
}

/// `check --against <run>` for links (phase 97 P4): `transport-exceeded`
/// (an error: a message in the run took longer than its link's bound) and
/// `link-unmeasured` (a warning: the run has no sample for a declared link,
/// because it recorded no `events.jsonl` or because no subscriber of the
/// link is a process the run intercepted, as across hosts). Returns the
/// diagnostics and a one-line summary.
pub fn check_against(
    index: &super::manifest_loader::ManifestIndex,
    run: &Path,
) -> (Vec<ros_launch_manifest_check::Diagnostic>, Option<String>) {
    use ros_launch_manifest_check::{Diagnostic, Severity};
    let edges = link_edges(index);
    if edges.is_empty() {
        return (Vec::new(), None);
    }
    let diag = |rule: &str, severity, path: String, message: String| Diagnostic {
        rule_id: rule.to_string(),
        severity,
        message,
        path,
        span: None,
    };
    let Some(path) = events_path(run) else {
        let mut names: Vec<&String> = edges.iter().map(|e| &e.link).collect();
        names.dedup();
        let out = names
            .into_iter()
            .map(|l| {
                diag(
                    "link-unmeasured",
                    Severity::Warning,
                    format!("links.{l}"),
                    format!(
                        "link '{l}': the run recorded no events.jsonl, so no transport was \
                         measured (record with `--record-events` and interception on)"
                    ),
                )
            })
            .collect();
        return (
            out,
            Some(format!("no events.jsonl under {}", run.display())),
        );
    };
    let body = std::fs::read_to_string(&path).unwrap_or_default();
    let obs = observe(&body, &edges);
    let mut out = Vec::new();
    for (link, o) in &obs {
        let bound = edges
            .iter()
            .find(|e| &e.link == link)
            .map(|e| e.bound_ms)
            .unwrap_or(0.0);
        if o.samples_ms.is_empty() {
            out.push(diag(
                "link-unmeasured",
                Severity::Warning,
                format!("links.{link}"),
                format!(
                    "link '{link}': no message across it in {} -- none of its subscribers is \
                     a process this run intercepted (across hosts the clocks are not \
                     comparable, so the transport cannot be measured here)",
                    path.display()
                ),
            ));
            continue;
        }
        if let Some((topic, node, worst)) = o
            .over
            .iter()
            .cloned()
            .reduce(|a, b| if b.2 > a.2 { b } else { a })
        {
            out.push(diag(
                "transport-exceeded",
                Severity::Error,
                format!("links.{link}.max_transport"),
                format!(
                    "link '{link}': {} of {} message(s) took longer than its max_transport \
                     {bound:.2}ms in {}; the worst {worst:.2}ms on {topic} into {node}",
                    o.over.len(),
                    o.samples_ms.len(),
                    path.display()
                ),
            ));
        }
    }
    let summary = obs
        .iter()
        .map(|(l, o)| match o.max_ms() {
            Some(m) => format!("{l}: {} sample(s), max {m:.2}ms", o.samples_ms.len()),
            None => format!("{l}: unmeasured"),
        })
        .collect::<Vec<_>>()
        .join("; ");
    (out, Some(summary))
}

/// The `measure` proposal for a link: observed max x (1 + margin), rounded
/// up to the next ms, as a measured budget would be sized.
pub fn propose_ms(max_ms: f64, margin: f64) -> f64 {
    (max_ms * (1.0 + margin) - 1e-9).ceil()
}

#[cfg(test)]
mod tests {
    use super::*;

    const RUN: &str = r#"{"t":{"h":7,"n":"/t"}}
{"e":{"n":"/p","d":"pub","h":7,"s":1,"ns":0,"t":1000000000,"c":0,"tid":1}}
{"e":{"n":"/far","d":"take","h":7,"s":1,"ns":0,"t":1030000000,"c":0,"tid":2}}
{"e":{"n":"/near","d":"take","h":7,"s":1,"ns":0,"t":1001000000,"c":0,"tid":3}}
{"e":{"n":"/p","d":"pub","h":7,"s":2,"ns":0,"t":1100000000,"c":0,"tid":1}}
{"e":{"n":"/far","d":"take","h":7,"s":2,"ns":0,"t":1160000000,"c":0,"tid":2}}
"#;

    fn edges() -> Vec<LinkEdge> {
        vec![
            LinkEdge {
                link: "serial".into(),
                topic: "/t".into(),
                sub_node: "/far".into(),
                bound_ms: 56.0,
            },
            LinkEdge {
                link: "unseen".into(),
                topic: "/u".into(),
                sub_node: "/far".into(),
                bound_ms: 5.0,
            },
        ]
    }

    /// A take on a linked edge is a sample; one over the bound is named; a
    /// take on another edge of the topic is not the link's; a link with no
    /// take is present and empty (unmeasured).
    #[test]
    fn a_run_gives_each_link_its_samples_and_its_overruns() {
        let o = observe(RUN, &edges());
        let s = &o["serial"];
        assert_eq!(s.samples_ms.len(), 2);
        assert_eq!(s.max_ms(), Some(60.0));
        assert_eq!(s.over, vec![("/t".into(), "/far".into(), 60.0)]);
        assert!(o["unseen"].samples_ms.is_empty());
    }

    #[test]
    fn a_proposal_is_the_max_with_its_margin_rounded_up() {
        assert_eq!(propose_ms(46.42, 0.2), 56.0);
        assert_eq!(propose_ms(50.0, 0.2), 60.0);
    }
}
