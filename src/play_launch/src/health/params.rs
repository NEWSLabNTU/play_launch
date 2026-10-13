//! Parameters as running against parameters as launched
//! (`big-system-observability.md` §J), and the time base they set (§I).
//!
//! Expected values come from the model: a node's `param_sources` folded in
//! order (globals first, then its own inline values and files), each file's
//! sections applied when their key matches the node's FQN as rcl matches it
//! (`/**`, `/ns/*`, a bare name meaning the root namespace). Actual values
//! come from asking each node, one `get_parameters` client at a time, for
//! exactly the names its launch sets plus `use_sim_time` — once startup
//! completes, then every [`PROBE_PERIOD`]. Not `/parameter_events`: it is
//! volatile, and play_launch subscribes after the nodes start, so the
//! declarations a node makes in its constructor are gone before the match.
//!
//! - `param_drift`: the node runs a value other than the one launched with —
//!   changed at runtime, or the override was overridden by the node itself.
//! - `param_ignored`: the launch sets a parameter the node never declared.
//!   rclcpp drops an override for an undeclared name without a word (unless
//!   the node allows undeclared parameters), so a typo in a parameter name
//!   or a YAML section that names the wrong node looks exactly like a launch
//!   that worked.
//! - `sim_time_mixed`, `sim_time_no_clock`: §I.

use std::{
    collections::{BTreeMap, HashMap},
    time::Duration,
};

use ros_launch_manifest_model::{NodeInstance, ParamSource, ParamValue as ModelValue};
use serde::Serialize;

use super::{Condition, Link, Severity, Subject};
use crate::ros::parameter_types::ParamValue as RosValue;

/// How often the nodes are asked again after the first probe.
pub const PROBE_PERIOD: Duration = Duration::from_secs(600);
/// The first probe waits this long after startup completes, for late
/// declarations (a node that declares on its first timer).
/// `PLAY_LAUNCH_PARAM_PROBE_DELAY_S` overrides it (tests use seconds).
pub fn probe_delay() -> Duration {
    std::env::var("PLAY_LAUNCH_PARAM_PROBE_DELAY_S")
        .ok()
        .and_then(|v| v.parse().ok())
        .map_or(Duration::from_secs(15), Duration::from_secs)
}

/// A launched value, typed the way YAML 1.1 types it.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(untagged)]
pub enum Expected {
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
    List(Vec<Expected>),
}

impl Expected {
    fn from_yaml(v: &serde_yaml_ng::Value) -> Option<Self> {
        use serde_yaml_ng::Value as Y;
        Some(match v {
            Y::Bool(b) => Expected::Bool(*b),
            Y::Number(n) => n
                .as_i64()
                .map(Expected::Int)
                .or_else(|| n.as_f64().map(Expected::Float))?,
            Y::String(s) => Expected::Str(s.clone()),
            Y::Sequence(items) => Expected::List(
                items
                    .iter()
                    .map(Expected::from_yaml)
                    .collect::<Option<_>>()?,
            ),
            _ => return None,
        })
    }

    /// One element of a model `StrList`: YAML scalar text.
    fn from_scalar_text(text: &str) -> Self {
        serde_yaml_ng::from_str::<serde_yaml_ng::Value>(text)
            .ok()
            .and_then(|v| Expected::from_yaml(&v))
            .unwrap_or_else(|| Expected::Str(text.to_string()))
    }

    fn from_model(v: &ModelValue) -> Self {
        match v {
            ModelValue::Bool(b) => Expected::Bool(*b),
            ModelValue::Int(i) => Expected::Int(*i),
            ModelValue::Float(f) => Expected::Float(*f),
            ModelValue::Str(s) => Expected::Str(s.clone()),
            ModelValue::StrList(items) => Expected::List(
                items
                    .iter()
                    .map(|t| Expected::from_scalar_text(t))
                    .collect(),
            ),
        }
    }

    /// Whether a running value is this one. Numbers compare by value across
    /// integer and double: a launch writing `1` for a double parameter is
    /// not drift.
    fn matches(&self, actual: &RosValue) -> bool {
        fn num(e: &Expected) -> Option<f64> {
            match e {
                Expected::Int(i) => Some(*i as f64),
                Expected::Float(f) => Some(*f),
                _ => None,
            }
        }
        fn close(a: f64, b: f64) -> bool {
            a == b || (a - b).abs() <= 1e-9 * a.abs().max(b.abs())
        }
        match (self, actual) {
            (Expected::Bool(a), RosValue::Bool(b)) => a == b,
            (e, RosValue::Integer(b)) => num(e).is_some_and(|a| close(a, *b as f64)),
            (e, RosValue::Double(b)) => num(e).is_some_and(|a| close(a, *b)),
            (Expected::Str(a), RosValue::String(b)) => a == b,
            (Expected::List(a), RosValue::BoolArray(b)) => {
                a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.matches(&RosValue::Bool(*y)))
            }
            (Expected::List(a), RosValue::IntegerArray(b)) => {
                a.len() == b.len()
                    && a.iter()
                        .zip(b)
                        .all(|(x, y)| x.matches(&RosValue::Integer(*y)))
            }
            (Expected::List(a), RosValue::DoubleArray(b)) => {
                a.len() == b.len()
                    && a.iter()
                        .zip(b)
                        .all(|(x, y)| x.matches(&RosValue::Double(*y)))
            }
            (Expected::List(a), RosValue::StringArray(b)) => {
                a.len() == b.len()
                    && a.iter()
                        .zip(b)
                        .all(|(x, y)| x.matches(&RosValue::String(y.clone())))
            }
            // An empty list in YAML has no type; any empty array matches.
            (Expected::List(a), RosValue::ByteArray(b)) => a.is_empty() && b.is_empty(),
            _ => false,
        }
    }
}

/// rcl's match of a parameter-file key against a node FQN: a leading `/` is
/// implied, `*` matches within one segment, `**` across any number.
fn key_matches(key: &str, fqn: &str) -> bool {
    let key = if key.starts_with('/') {
        key.to_string()
    } else {
        format!("/{key}")
    };
    fn go(p: &[u8], s: &[u8]) -> bool {
        match p {
            [] => s.is_empty(),
            [b'*', b'*', rest @ ..] => (0..=s.len()).any(|i| go(rest, &s[i..])),
            [b'*', rest @ ..] => {
                let seg = s.iter().position(|&c| c == b'/').unwrap_or(s.len());
                (0..=seg).any(|i| go(rest, &s[i..]))
            }
            [c, rest @ ..] => s.first() == Some(c) && go(rest, &s[1..]),
        }
    }
    go(key.as_bytes(), fqn.as_bytes())
}

fn flatten(prefix: &str, v: &serde_yaml_ng::Value, out: &mut BTreeMap<String, Expected>) {
    if let serde_yaml_ng::Value::Mapping(m) = v {
        for (k, v) in m {
            let Some(k) = k.as_str() else { continue };
            let name = if prefix.is_empty() {
                k.to_string()
            } else {
                format!("{prefix}.{k}")
            };
            if matches!(v, serde_yaml_ng::Value::Mapping(_)) {
                flatten(&name, v, out);
            } else if let Some(e) = Expected::from_yaml(v) {
                out.insert(name, e);
            }
        }
    }
}

fn apply_file(content: &str, fqn: &str, out: &mut BTreeMap<String, Expected>) {
    let Ok(serde_yaml_ng::Value::Mapping(top)) = serde_yaml_ng::from_str(content) else {
        return;
    };
    for (key, section) in &top {
        let Some(key) = key.as_str() else { continue };
        if !key_matches(key, fqn) {
            continue;
        }
        if let Some(params) = section.get("ros__parameters") {
            flatten("", params, out);
        }
    }
}

/// The values a node was launched with, by parameter name.
pub fn expected_params(node: &NodeInstance, fqn: &str) -> BTreeMap<String, Expected> {
    let mut out = BTreeMap::new();
    if node.param_sources.is_empty() {
        for content in &node.params_files {
            apply_file(content, fqn, &mut out);
        }
        for (name, v) in &node.params {
            out.insert(name.clone(), Expected::from_model(v));
        }
    } else {
        for source in &node.param_sources {
            match source {
                ParamSource::Inline { name, value } => {
                    out.insert(name.clone(), Expected::from_model(value));
                }
                ParamSource::File { content } => apply_file(content, fqn, &mut out),
            }
        }
    }
    out
}

/// One node's probe.
#[derive(Debug, Clone, Default, Serialize)]
pub struct NodeParams {
    pub fqn: String,
    pub member: String,
    /// `None` when the node did not answer.
    pub use_sim_time: Option<bool>,
    pub checked: usize,
    pub drift: Vec<Drift>,
    /// Launched names the node never declared.
    pub ignored: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Drift {
    pub name: String,
    pub launched: Expected,
    pub running: RosValue,
}

/// Compare one node's answer with what it was launched with. `values` are
/// in the order of `names`.
pub fn compare(
    fqn: &str,
    member: &str,
    expected: &BTreeMap<String, Expected>,
    names: &[String],
    values: &[RosValue],
) -> NodeParams {
    let mut p = NodeParams {
        fqn: fqn.to_string(),
        member: member.to_string(),
        ..Default::default()
    };
    for (name, actual) in names.iter().zip(values) {
        if name == "use_sim_time" {
            p.use_sim_time = match actual {
                RosValue::Bool(b) => Some(*b),
                _ => None,
            };
        }
        let Some(e) = expected.get(name) else {
            continue;
        };
        p.checked += 1;
        if matches!(actual, RosValue::NotSet) {
            p.ignored.push(name.clone());
        } else if !e.matches(actual) {
            p.drift.push(Drift {
                name: name.clone(),
                launched: e.clone(),
                running: actual.clone(),
            });
        }
    }
    p
}

/// The probe's result across the run.
#[derive(Debug, Clone, Default, Serialize)]
pub struct ParamsReport {
    /// Unix ms of the probe.
    pub probed_ms: u64,
    pub nodes: Vec<NodeParams>,
    /// Whether `/clock` had a publisher at the last graph read.
    pub clock_published: Option<bool>,
    /// `/clock` as published, when interception reads it.
    pub clock: Option<crate::interception::ClockStatus>,
    /// Launched names undeclared on [`BROADCAST_NODES`] or more nodes, with
    /// how many: a value broadcast to every node (a vehicle description, a
    /// shared diagnostics setting), which most nodes do not use. Removed
    /// from the nodes' `ignored` lists.
    pub broadcast: Vec<(String, usize)>,
}

/// A name undeclared on this many nodes is a broadcast, not a mistake.
pub const BROADCAST_NODES: usize = 3;

impl ParamsReport {
    /// Move names undeclared on [`BROADCAST_NODES`] or more nodes out of
    /// the per-node lists. Measured on Autoware: 1163 undeclared names on
    /// 85 nodes, 720 of them the vehicle description sent to 72 nodes.
    pub fn separate_broadcasts(&mut self) {
        let mut counts: HashMap<String, usize> = HashMap::new();
        for n in &self.nodes {
            for name in &n.ignored {
                *counts.entry(name.clone()).or_default() += 1;
            }
        }
        for n in &mut self.nodes {
            n.ignored.retain(|name| counts[name] < BROADCAST_NODES);
        }
        let mut b: Vec<(String, usize)> = counts
            .into_iter()
            .filter(|(_, c)| *c >= BROADCAST_NODES)
            .collect();
        b.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
        self.broadcast = b;
    }
}

fn show(v: &impl Serialize) -> String {
    serde_json::to_string(v).unwrap_or_default()
}

/// A running value as it would be written: `5`, `"a"`, `[1, 2]`.
fn show_running(v: &RosValue) -> String {
    match serde_json::to_value(v) {
        Ok(j) => j
            .get("value")
            .map_or_else(|| "unset".to_string(), |x| x.to_string()),
        Err(_) => "?".to_string(),
    }
}

pub fn param_conditions(r: &ParamsReport) -> Vec<Condition> {
    let mut out = Vec::new();
    for n in &r.nodes {
        let subject = Subject {
            kind: "node",
            name: n.fqn.clone(),
        };
        let link = Some(Link {
            view: "nodes",
            target: Some(n.member.clone()),
        });
        if !n.drift.is_empty() {
            let shown: Vec<String> = n
                .drift
                .iter()
                .take(5)
                .map(|d| {
                    format!(
                        "{}: launched {}, running {}",
                        d.name,
                        show(&d.launched),
                        show_running(&d.running)
                    )
                })
                .collect();
            out.push(Condition {
                key: format!("param_drift:{}", n.fqn),
                kind: "param_drift",
                severity: Severity::Warning,
                subject: subject.clone(),
                title: format!(
                    "{} parameter{} differ from the launch",
                    n.drift.len(),
                    if n.drift.len() == 1 { "" } else { "s" }
                ),
                detail: format!(
                    "{}{}. Set at runtime (`ros2 param set`, a node changing its own), or \
                     overridden by the node after it declared them",
                    shown.join("; "),
                    if n.drift.len() > 5 { "; …" } else { "" }
                ),
                link: link.clone(),
            });
        }
        if !n.ignored.is_empty() {
            let shown: Vec<&str> = n.ignored.iter().take(8).map(String::as_str).collect();
            out.push(Condition {
                key: format!("param_ignored:{}", n.fqn),
                kind: "param_ignored",
                // Info: a launch file written for several configurations
                // sets parameters of modules a run does not enable (Autoware's
                // planners: up to 109 each). A misspelling looks the same.
                severity: Severity::Info,
                subject,
                title: format!(
                    "the launch sets {} parameter{} the node never declared",
                    n.ignored.len(),
                    if n.ignored.len() == 1 { "" } else { "s" }
                ),
                detail: format!(
                    "{}{}: the values were dropped without a word. A misspelled name, a \
                     YAML section keyed to another node, a module this run does not enable, \
                     or a parameter the node declares only later",
                    shown.join(", "),
                    if n.ignored.len() > 8 { ", …" } else { "" }
                ),
                link,
            });
        }
    }

    // §I: one time base.
    let sim: Vec<&NodeParams> = r
        .nodes
        .iter()
        .filter(|n| n.use_sim_time == Some(true))
        .collect();
    let wall: Vec<&NodeParams> = r
        .nodes
        .iter()
        .filter(|n| n.use_sim_time == Some(false))
        .collect();
    if !sim.is_empty() && !wall.is_empty() {
        let (few, many, few_word) = if sim.len() <= wall.len() {
            (&sim, &wall, "simulated")
        } else {
            (&wall, &sim, "wall-clock")
        };
        let names: Vec<&str> = few.iter().take(6).map(|n| n.fqn.as_str()).collect();
        out.push(Condition {
            key: "sim_time_mixed".to_string(),
            kind: "sim_time_mixed",
            severity: Severity::Warning,
            subject: Subject {
                kind: "run",
                name: "use_sim_time".to_string(),
            },
            title: format!(
                "{} node{} on {few_word} time, {} on the other",
                few.len(),
                if few.len() == 1 { "" } else { "s" },
                many.len()
            ),
            detail: format!(
                "{}{}: their stamps and every age, timeout and TF lookup between the two \
                 groups disagree by the gap between the clocks",
                names.join(", "),
                if few.len() > 6 { ", …" } else { "" }
            ),
            link: None,
        });
    }
    if !sim.is_empty() && r.clock_published == Some(false) {
        out.push(Condition {
            key: "sim_time_no_clock".to_string(),
            kind: "sim_time_no_clock",
            severity: Severity::Error,
            subject: Subject {
                kind: "topic",
                name: "/clock".to_string(),
            },
            title: format!(
                "{} node(s) use sim time and nothing publishes /clock",
                sim.len()
            ),
            detail: "their clocks stand at zero: timers never fire, and every stamp is 0"
                .to_string(),
            link: None,
        });
    }
    out
}

/// `/clock` stalls stop sim time; a gap this long is one.
const CLOCK_STALL_S: f64 = 1.0;
/// A backward jump stays an issue this long.
const CLOCK_BACKWARD_RECENT_S: f64 = 60.0;

/// `clock_stalled` (only when some node runs on sim time, else nobody
/// waits for it) and `clock_backwards`.
pub fn clock_conditions(
    clock: Option<&crate::interception::ClockStatus>,
    sim_users: usize,
) -> Vec<Condition> {
    let Some(c) = clock else { return Vec::new() };
    let mut out = Vec::new();
    let subject = Subject {
        kind: "topic",
        name: "/clock".to_string(),
    };
    if sim_users > 0 && c.messages >= 5 && c.since_last_s > CLOCK_STALL_S {
        out.push(Condition {
            key: "clock_stalled".to_string(),
            kind: "clock_stalled",
            severity: Severity::Warning,
            subject: subject.clone(),
            title: format!("/clock has not moved for {:.0} s", c.since_last_s),
            detail: format!(
                "stopped at {:.3} s after {} messages at {:.0} Hz; {} node(s) on sim time \
                 wait: their timers stop and every timeout is frozen. A paused or dead \
                 simulator, or a bag at its end",
                c.value_s, c.messages, c.rate_hz, sim_users
            ),
            link: None,
        });
    }
    if let (Some(ago), Some(by)) = (c.last_backward_ago_s, c.last_backward_by_s)
        && ago < CLOCK_BACKWARD_RECENT_S
    {
        out.push(Condition {
            key: "clock_backwards".to_string(),
            kind: "clock_backwards",
            severity: Severity::Warning,
            subject,
            title: format!(
                "/clock went back {by:.3} s ({} time{} in all)",
                c.backward_jumps,
                if c.backward_jumps == 1 { "" } else { "s" }
            ),
            detail: "nodes on sim time see time reverse: TF buffers drop data, message \
                     filters wait forever. Two publishers of /clock interleaving, or a \
                     bag looping"
                .to_string(),
            link: None,
        });
    }
    out
}

/// What to ask one node: the names its launch sets, plus `use_sim_time`.
pub fn probe_names(expected: &BTreeMap<String, Expected>) -> Vec<String> {
    let mut names: Vec<String> = expected.keys().cloned().collect();
    if !expected.contains_key("use_sim_time") {
        names.push("use_sim_time".to_string());
    }
    names
}

/// The members to probe: those whose node name the run knows, with the
/// model entry their FQN keys.
pub fn probe_targets<'a>(
    model: &'a ros_launch_manifest_model::SystemModel,
    fqns: &'a HashMap<String, String>,
) -> Vec<(&'a str, &'a str, &'a NodeInstance)> {
    let mut v: Vec<_> = fqns
        .iter()
        .filter_map(|(member, fqn)| {
            model
                .structure
                .nodes
                .get(fqn)
                .map(|n| (member.as_str(), fqn.as_str(), n))
        })
        .collect();
    v.sort_by_key(|(_, f, _)| *f);
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clock_that_stops_or_goes_back_is_reported() {
        let c = crate::interception::ClockStatus {
            value_s: 12.5,
            since_last_s: 3.0,
            rate_hz: 100.0,
            messages: 500,
            backward_jumps: 2,
            last_backward_ago_s: Some(5.0),
            last_backward_by_s: Some(30.0),
        };
        let kinds = |c: &crate::interception::ClockStatus, users| -> Vec<&'static str> {
            clock_conditions(Some(c), users)
                .iter()
                .map(|c| c.kind)
                .collect()
        };
        assert_eq!(kinds(&c, 3), ["clock_stalled", "clock_backwards"]);
        assert_eq!(kinds(&c, 0), ["clock_backwards"], "nobody waits on a stall");
        let old = crate::interception::ClockStatus {
            since_last_s: 0.01,
            last_backward_ago_s: Some(600.0),
            ..c
        };
        assert!(kinds(&old, 3).is_empty());
    }

    #[test]
    fn a_name_undeclared_on_many_nodes_is_a_broadcast() {
        let node = |f: &str, ignored: &[&str]| NodeParams {
            fqn: f.into(),
            ignored: ignored.iter().map(|s| s.to_string()).collect(),
            ..Default::default()
        };
        let mut r = ParamsReport {
            nodes: vec![
                node("/a", &["wheel_base", "typo"]),
                node("/b", &["wheel_base"]),
                node("/c", &["wheel_base"]),
            ],
            ..Default::default()
        };
        r.separate_broadcasts();
        assert_eq!(r.nodes[0].ignored, ["typo"]);
        assert!(r.nodes[1].ignored.is_empty());
        assert_eq!(r.broadcast, [("wheel_base".to_string(), 3)]);
    }

    #[test]
    fn file_keys_match_as_rcl_matches_them() {
        assert!(key_matches("/**", "/a/b/c"));
        assert!(key_matches("talker", "/talker"));
        assert!(!key_matches("talker", "/ns/talker"));
        assert!(key_matches("/ns/*", "/ns/talker"));
        assert!(!key_matches("/ns/*", "/ns/deeper/talker"));
        assert!(key_matches("/**/talker", "/a/b/talker"));
        assert!(key_matches("/ns/talker", "/ns/talker"));
        assert!(!key_matches("/ns/listener", "/ns/talker"));
    }

    #[test]
    fn sources_fold_in_order_and_files_apply_by_section() {
        let file = "/**:\n  ros__parameters:\n    rate: 10\n    nested:\n      gain: 0.5\n/other:\n  ros__parameters:\n    rate: 99\n";
        let node = NodeInstance {
            param_sources: vec![
                ParamSource::Inline {
                    name: "use_sim_time".into(),
                    value: ModelValue::Bool(true),
                },
                ParamSource::File {
                    content: file.into(),
                },
                ParamSource::Inline {
                    name: "rate".into(),
                    value: ModelValue::Int(20),
                },
            ],
            ..Default::default()
        };
        let e = expected_params(&node, "/me");
        assert_eq!(e.get("rate"), Some(&Expected::Int(20)), "later wins");
        assert_eq!(e.get("nested.gain"), Some(&Expected::Float(0.5)));
        assert_eq!(e.get("use_sim_time"), Some(&Expected::Bool(true)));
    }

    #[test]
    fn values_compare_by_meaning_not_spelling() {
        assert!(Expected::Int(1).matches(&RosValue::Double(1.0)));
        assert!(Expected::Float(0.1).matches(&RosValue::Double(0.1)));
        assert!(!Expected::Int(1).matches(&RosValue::Integer(2)));
        assert!(!Expected::Str("1".into()).matches(&RosValue::Integer(1)));
        let list = Expected::from_model(&ModelValue::StrList(vec!["1".into(), "2.5".into()]));
        assert!(list.matches(&RosValue::DoubleArray(vec![1.0, 2.5])));
        let strs = Expected::from_model(&ModelValue::StrList(vec!["'5'".into(), "a".into()]));
        assert!(strs.matches(&RosValue::StringArray(vec!["5".into(), "a".into()])));
    }

    #[test]
    fn drift_ignored_and_the_time_base() {
        let mut expected = BTreeMap::new();
        expected.insert("rate".to_string(), Expected::Int(10));
        expected.insert("typo_rat".to_string(), Expected::Int(5));
        let names = probe_names(&expected);
        assert_eq!(names.last().unwrap(), "use_sim_time");
        // In the order of `names`: rate, typo_rat, use_sim_time.
        let values = [
            RosValue::Integer(15),
            RosValue::NotSet,
            RosValue::Bool(true),
        ];
        let a = compare("/a", "node:/a", &expected, &names, &values);
        assert_eq!(a.drift.len(), 1);
        assert_eq!(a.ignored, ["typo_rat"]);
        assert_eq!(a.use_sim_time, Some(true));
        let b = NodeParams {
            fqn: "/b".into(),
            use_sim_time: Some(false),
            ..Default::default()
        };
        let r = ParamsReport {
            nodes: vec![a, b],
            clock_published: Some(false),
            ..Default::default()
        };
        let kinds: Vec<&str> = param_conditions(&r).iter().map(|c| c.kind).collect();
        assert_eq!(
            kinds,
            [
                "param_drift",
                "param_ignored",
                "sim_time_mixed",
                "sim_time_no_clock"
            ]
        );
    }
}
