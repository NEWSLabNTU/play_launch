//! How close a recorded run came to each subscriber's silence bound
//! (issue #0097, GitHub #25).
//!
//! A subscriber that states a QoS `deadline`, a QoS `lease_duration` or a
//! `max_age` trips when its input goes quiet for longer. Before a detector
//! is relied on, a run under load should show it would not have tripped
//! falsely, and by how much. `interception/events.jsonl` holds every take
//! with `CLOCK_MONOTONIC`, so for each such subscriber this measures:
//!
//! - the gap between consecutive takes (what a `deadline` or a lease
//!   watches at the reader), its maximum and 99.9th percentile; and
//! - the age a message reached before the next one arrived (what a
//!   `max_age` watches): the next take minus that message's PUBLISH, when
//!   the run recorded the publish (same stamp, same topic), else the gap,
//!   which is then a lower bound and reported as one.
//!
//! Three details keep the numbers honest. A take of a stamp the process
//! already took is the same message reaching a second subscription and is
//! not an arrival, or two subscriptions would halve every gap. Recording
//! switched off and on again appends to the same file and re-emits each
//! topic's name before its first message in the new segment, so a repeated
//! name restarts that topic's gaps rather than reading the pause as
//! silence. And a node is named by its process's ring, so a composable
//! node's takes are recorded under its container and do not match it.

use std::{
    collections::{BTreeMap, HashMap, HashSet},
    path::Path,
};

use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::OptBound;

/// The default `--silence-margin`: warn when less than this share of a
/// bound was left at the run's worst.
pub const DEFAULT_MARGIN_PCT: f64 = 20.0;

/// One subscriber with at least one silence bound.
#[derive(Debug, Clone, PartialEq)]
pub struct Watched {
    pub topic: String,
    /// The subscriber's node FQN, as the record names the taking process.
    pub node: String,
    /// The endpoint name in the contract (`sub.<ep>`).
    pub endpoint: String,
    pub deadline_ms: Option<f64>,
    pub lease_ms: Option<f64>,
    pub max_age_ms: Option<f64>,
}

impl Watched {
    /// Each declared bound: (key, ms, judged on the age rather than the gap).
    fn bounds(&self) -> Vec<(&'static str, f64, bool)> {
        [
            ("qos.deadline", self.deadline_ms, false),
            ("qos.lease_duration", self.lease_ms, false),
            ("max_age", self.max_age_ms, true),
        ]
        .into_iter()
        .filter_map(|(k, v, age)| v.map(|v| (k, v, age)))
        .collect()
    }
}

/// What a run showed for one subscriber.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Arrivals {
    /// Arrivals (distinct messages taken).
    pub takes: usize,
    /// Gaps between consecutive arrivals within a segment, ms.
    pub gaps_ms: Vec<f64>,
    /// When the largest gap ended, ms after the subscriber's first take.
    pub worst_gap_at_ms: Option<f64>,
    /// Per gap, the age the previous message reached, ms.
    pub ages_ms: Vec<f64>,
    /// How many of `ages_ms` had no recorded publish and are the gap.
    pub ages_from_gap: usize,
}

impl Arrivals {
    pub fn max_gap(&self) -> Option<f64> {
        self.gaps_ms.iter().copied().reduce(f64::max)
    }
    pub fn max_age(&self) -> Option<f64> {
        self.ages_ms.iter().copied().reduce(f64::max)
    }
    /// The nearest-rank 99.9th percentile gap.
    pub fn p999_gap(&self) -> Option<f64> {
        percentile(&self.gaps_ms, 99.9)
    }
}

fn percentile(v: &[f64], p: f64) -> Option<f64> {
    if v.is_empty() {
        return None;
    }
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    // Less a hair: 0.999 x 2000 is 1998.0000000000002 in binary.
    let rank = ((p / 100.0) * s.len() as f64 - 1e-9).ceil() as usize;
    Some(s[rank.clamp(1, s.len()) - 1])
}

/// Each watched subscriber's arrivals in `events` (the `events.jsonl` text),
/// keyed by `(topic, node)`. A subscriber with no take is present and empty.
pub fn observe(events: &str, watched: &[Watched]) -> BTreeMap<(String, String), Arrivals> {
    let mut out: BTreeMap<(String, String), Arrivals> = watched
        .iter()
        .map(|w| ((w.topic.clone(), w.node.clone()), Arrivals::default()))
        .collect();
    let mut topics: HashMap<u64, String> = HashMap::new();
    // (topic hash, stamp) -> earliest publish, ns
    let mut published: HashMap<(u64, u64), u64> = HashMap::new();
    // (topic hash, node) -> stamps taken in this segment
    let mut taken: HashMap<(u64, String), HashSet<u64>> = HashMap::new();
    // (topic hash, node) -> (previous arrival ns, its stamp, first arrival ns)
    let mut last: HashMap<(u64, String), (u64, u64, u64)> = HashMap::new();
    // Takes are paired with a publish once the whole file is read: rings
    // drain in turn, so a publish is often recorded after its take.
    // (key, topic hash, gap ns, arrival ns, previous stamp, first arrival ns)
    let mut pending: Vec<((String, String), u64, u64, u64, u64, u64)> = Vec::new();
    for line in events.lines() {
        let Ok(v) = serde_json::from_str::<serde_json::Value>(line) else {
            continue;
        };
        if let Some(t) = v.get("t") {
            if let (Some(h), Some(n)) = (t["h"].as_u64(), t["n"].as_str()) {
                if topics.insert(h, n.to_string()).is_some() {
                    // A new recording segment: its gaps start afresh.
                    last.retain(|(th, _), _| *th != h);
                    taken.retain(|(th, _), _| *th != h);
                }
            }
            continue;
        }
        let Some(e) = v.get("e") else { continue };
        let (Some(h), Some(t), Some(d)) = (e["h"].as_u64(), e["t"].as_u64(), e["d"].as_str())
        else {
            continue;
        };
        let (s, ns) = (e["s"].as_i64().unwrap_or(0), e["ns"].as_u64().unwrap_or(0));
        let stamp = if s == 0 && ns == 0 {
            0
        } else {
            ((s as u32 as u64) << 32) | (ns & 0xffff_ffff)
        };
        match d {
            "pub" if stamp != 0 => {
                published
                    .entry((h, stamp))
                    .and_modify(|p| *p = (*p).min(t))
                    .or_insert(t);
            }
            "take" => {
                let Some(topic) = topics.get(&h) else {
                    continue;
                };
                let node = e["n"].as_str().unwrap_or("").to_string();
                let key = (topic.clone(), node.clone());
                let Some(arr) = out.get_mut(&key) else {
                    continue;
                };
                if stamp != 0 && !taken.entry((h, node.clone())).or_default().insert(stamp) {
                    continue; // a second subscription's take of one message
                }
                arr.takes += 1;
                match last.get(&(h, node.clone())).copied() {
                    Some((prev_t, prev_stamp, first)) if t >= prev_t => {
                        pending.push((key, h, t - prev_t, t, prev_stamp, first));
                        last.insert((h, node), (t, stamp, first));
                    }
                    Some(_) => {} // out of order within a ring: not an arrival gap
                    None => {
                        last.insert((h, node), (t, stamp, t));
                    }
                }
            }
            _ => {}
        }
    }
    for (key, h, gap, at, prev_stamp, first) in pending {
        let arr = out.get_mut(&key).expect("keyed above");
        let gap_ms = gap as f64 / 1e6;
        if arr.max_gap().is_none_or(|m| gap_ms > m) {
            arr.worst_gap_at_ms = Some((at - first) as f64 / 1e6);
        }
        arr.gaps_ms.push(gap_ms);
        let publish = (prev_stamp != 0)
            .then(|| published.get(&(h, prev_stamp)))
            .flatten()
            .filter(|p| **p <= at);
        match publish {
            Some(p) => arr.ages_ms.push((at - p) as f64 / 1e6),
            None => {
                arr.ages_ms.push(gap_ms);
                arr.ages_from_gap += 1;
            }
        }
    }
    out
}

/// Every contracted subscriber that states a silence bound, from the index.
pub fn watched(index: &super::manifest_loader::ManifestIndex) -> Vec<Watched> {
    use ros_launch_manifest_types::QosDecl;
    let ms = |d: Option<ros_launch_manifest_types::duration::Duration>| {
        d.map(|d| d.as_millis_f64()).filter(|v| *v > 0.0)
    };
    // (node FQN, endpoint) -> props
    let mut subs: HashMap<(String, String), &ros_launch_manifest_types::EndpointProps> =
        HashMap::new();
    for m in index.manifests.values() {
        for (node, decl) in &m.manifest.nodes {
            let fqn = super::manifest_loader::resolve_node_fqn(index, m.scope_id, &m.ns, node);
            for (ep, props) in &decl.subscribers {
                subs.insert((fqn.clone(), ep.clone()), props);
            }
        }
    }
    let mut out = Vec::new();
    for (topic, t) in &index.topics {
        for r in &t.subscribers {
            let Some((node, ep)) = r.rsplit_once('/') else {
                continue;
            };
            let Some(props) = subs.get(&(node.to_string(), ep.to_string())) else {
                continue;
            };
            let qos = QosDecl::effective(t.qos.as_ref(), props.qos.as_ref());
            let w = Watched {
                topic: topic.clone(),
                node: node.to_string(),
                endpoint: ep.to_string(),
                deadline_ms: ms(qos.deadline),
                lease_ms: ms(qos.lease_duration),
                max_age_ms: ms(props.max_age.duration()),
            };
            if !w.bounds().is_empty() && !out.contains(&w) {
                out.push(w);
            }
        }
    }
    out
}

/// The report lines and diagnostics for `watched` against `events`.
/// `margin_pct` is the share of a bound that must be left at the worst.
pub fn judge(
    events: &str,
    source: &Path,
    watched: &[Watched],
    margin_pct: f64,
) -> (Vec<Diagnostic>, Vec<String>) {
    let obs = observe(events, watched);
    let mut diags = Vec::new();
    let mut lines = Vec::new();
    for w in watched {
        let at = format!("nodes.{}.sub.{}", w.node, w.endpoint);
        let a = &obs[&(w.topic.clone(), w.node.clone())];
        let (Some(max_gap), Some(p999)) = (a.max_gap(), a.p999_gap()) else {
            lines.push(format!(
                "{} into {}: {} take(s), no gap measured",
                w.topic, w.node, a.takes
            ));
            diags.push(diag(
                "silence-unmeasured",
                Severity::Warning,
                at,
                format!(
                    "{} into {}: {} in {} -- a composable node's takes are recorded under \
                     its container, and a subscriber off this host takes nothing the record \
                     sees",
                    w.topic,
                    w.node,
                    if a.takes == 0 {
                        "no take".to_string()
                    } else {
                        "one take, so no gap".to_string()
                    },
                    source.display()
                ),
            ));
            continue;
        };
        let max_age = a.max_age().unwrap_or(max_gap);
        let mut line = format!(
            "{} into {}: {} take(s), max gap {max_gap:.2}ms ({:.2}s in), p99.9 gap {p999:.2}ms",
            w.topic,
            w.node,
            a.takes,
            a.worst_gap_at_ms.unwrap_or(0.0) / 1e3
        );
        for (key, bound, on_age) in w.bounds() {
            let (worst, what) = if on_age {
                let lower = if a.ages_from_gap > 0 {
                    " (a lower bound: some publishes were not recorded)"
                } else {
                    ""
                };
                (max_age, format!("age {max_age:.2}ms{lower}"))
            } else {
                (max_gap, format!("gap {max_gap:.2}ms"))
            };
            let margin = bound - worst;
            let pct = margin / bound * 100.0;
            line.push_str(&format!(
                "; {key} {bound:.2}ms, margin {margin:.2}ms ({pct:.0}%)"
            ));
            if worst > bound {
                diags.push(diag(
                    "silence-exceeded",
                    Severity::Warning,
                    format!("{at}.{key}"),
                    format!(
                        "{} into {}: the run's worst {what} is past its {key} {bound:.2}ms in \
                         {}, so the detector would have tripped; on a run with no fault \
                         injected that is a false trip",
                        w.topic,
                        w.node,
                        source.display()
                    ),
                ));
            } else if pct < margin_pct {
                diags.push(diag(
                    "silence-margin",
                    Severity::Warning,
                    format!("{at}.{key}"),
                    format!(
                        "{} into {}: the run's worst {what} leaves {margin:.2}ms ({pct:.0}%) \
                         of its {key} {bound:.2}ms, under the {margin_pct:.0}% asked for \
                         (`--silence-margin`)",
                        w.topic, w.node
                    ),
                ));
            }
        }
        lines.push(line);
    }
    (diags, lines)
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

/// `check --against <run>` for silence bounds. Returns the diagnostics and
/// the report lines (empty when no subscriber states a bound).
pub fn check_against(
    index: &super::manifest_loader::ManifestIndex,
    run: &Path,
    margin_pct: f64,
) -> (Vec<Diagnostic>, Vec<String>) {
    let watched = watched(index);
    if watched.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let Some(path) = super::link_samples::events_path(run) else {
        return (
            vec![diag(
                "silence-unmeasured",
                Severity::Warning,
                String::new(),
                format!(
                    "{} subscriber(s) state a silence bound, and {} recorded no events.jsonl \
                     (record with `--record-events` and interception on)",
                    watched.len(),
                    run.display()
                ),
            )],
            vec![format!("no events.jsonl under {}", run.display())],
        );
    };
    let body = std::fs::read_to_string(&path).unwrap_or_default();
    judge(&body, &path, &watched, margin_pct)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn w(max_age: Option<f64>) -> Watched {
        Watched {
            topic: "/hb".into(),
            node: "/island".into(),
            endpoint: "hb".into(),
            deadline_ms: Some(30.0),
            lease_ms: None,
            max_age_ms: max_age,
        }
    }

    fn ev(n: &str, d: &str, s: i64, t_ms: f64) -> String {
        format!(
            "{{\"e\":{{\"n\":\"{n}\",\"d\":\"{d}\",\"h\":7,\"s\":{s},\"ns\":0,\"t\":{},\"c\":0,\"tid\":1}}}}\n",
            (t_ms * 1e6) as u64
        )
    }

    const NAME: &str = "{\"t\":{\"h\":7,\"n\":\"/hb\"}}\n";

    /// Gaps are between arrivals; a second subscription's take of a stamp
    /// already taken is not one; the age runs from the previous message's
    /// publish, recorded after its take or not.
    #[test]
    fn gaps_and_ages_from_the_takes_and_publishes() {
        let mut run = NAME.to_string();
        run += &ev("/island", "take", 1, 1002.0);
        run += &ev("/hpc", "pub", 1, 1000.0); // recorded after its take
        run += &ev("/island", "take", 1, 1003.0); // second subscription
        run += &ev("/hpc", "pub", 2, 1010.0);
        run += &ev("/island", "take", 2, 1012.0);
        run += &ev("/island", "take", 3, 1037.0); // publish not recorded
        let a = &observe(&run, &[w(Some(40.0))])[&("/hb".into(), "/island".into())];
        assert_eq!(a.takes, 3);
        assert_eq!(a.gaps_ms, vec![10.0, 25.0]);
        assert_eq!(a.ages_ms, vec![12.0, 27.0]);
        assert_eq!(a.ages_from_gap, 0);
        assert_eq!(a.max_gap(), Some(25.0));
        assert_eq!(a.worst_gap_at_ms, Some(35.0));
    }

    /// A repeated topic name opens a new recording segment, and the pause
    /// between segments is not a gap.
    #[test]
    fn a_new_segment_restarts_the_gaps() {
        let mut run = NAME.to_string();
        run += &ev("/island", "take", 1, 1000.0);
        run += &ev("/island", "take", 2, 1010.0);
        run += NAME;
        run += &ev("/island", "take", 3, 9000.0);
        run += &ev("/island", "take", 4, 9010.0);
        let a = &observe(&run, &[w(None)])[&("/hb".into(), "/island".into())];
        assert_eq!(a.gaps_ms, vec![10.0, 10.0]);
    }

    #[test]
    fn the_percentile_is_nearest_rank() {
        let v: Vec<f64> = (1..=2000).map(f64::from).collect();
        assert_eq!(percentile(&v, 99.9), Some(1998.0));
        assert_eq!(percentile(&[5.0], 99.9), Some(5.0));
    }

    /// Past the bound is `silence-exceeded`; inside it with less than the
    /// margin left is `silence-margin`; a comfortable margin is quiet.
    #[test]
    fn each_bound_is_judged_with_its_margin() {
        let mut run = NAME.to_string();
        for (i, t) in [0.0, 10.0, 36.0, 46.0].iter().enumerate() {
            run += &ev("/hpc", "pub", i as i64 + 1, 1000.0 + t);
            run += &ev("/island", "take", i as i64 + 1, 1001.0 + t);
        }
        // gap 26 vs deadline 30: 13% left, under 20%. Age 27 vs 100: fine.
        let (d, lines) = judge(&run, Path::new("e"), &[w(Some(100.0))], 20.0);
        assert_eq!(d.len(), 1, "{d:?}");
        assert_eq!(d[0].rule_id, "silence-margin");
        assert_eq!(d[0].path, "nodes./island.sub.hb.qos.deadline");
        assert!(
            lines[0].contains("max gap 26.00ms")
                && lines[0].contains("qos.deadline 30.00ms, margin 4.00ms (13%)")
                && lines[0].contains("max_age 100.00ms, margin 73.00ms (73%)"),
            "{lines:?}"
        );
        // Age 27 past a 25 ms max_age.
        let (d, _) = judge(&run, Path::new("e"), &[w(Some(25.0))], 10.0);
        let rules: Vec<_> = d.iter().map(|d| d.rule_id.as_str()).collect();
        assert_eq!(rules, vec!["silence-exceeded"], "{d:?}");
        assert!(d[0].message.contains("age 27.00ms"), "{}", d[0].message);
    }

    #[test]
    fn a_subscriber_with_no_take_is_unmeasured() {
        let (d, _) = judge(NAME, Path::new("e"), &[w(None)], 20.0);
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].rule_id, "silence-unmeasured");
    }
}
